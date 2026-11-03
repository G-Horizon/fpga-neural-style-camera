// Weight storage for the shared 128-MAC array.
//
// Why this is not the trunk store with a different name
// ----------------------------------------------------
//
// The trunk store is banked because its consumer reads one *channel group* at a
// time and the bank select is part of the address.  This one has the opposite
// requirement: a MAC cycle needs the whole operand tile at once.  At the frozen
// IC8 x OC16 factorisation that is 8 x 16 = 128 weights = 1024 bits, every cycle,
// with no bank select anywhere.  So the word is the tile, the address is the
// tile's index, and the mapper is asked for one very wide shallow array rather
// than a narrow deep one.
//
// What the tile actually is, derived from golden/d64_int8_weights.npz rather than
// assumed.  Two different statements are true of it and conflating them is how a
// loader ends up 18 words short:
//
//   * the LOGICAL tiling has zero padding.  Every layer's weights tile into
//     IC x OC words with no channel left over: 211 words of 8 x 16, 18 more of
//     8 x 8 for ups.1, and 18 narrow ones for head and tail.  28,592 bytes, no
//     channel wasted.
//   * the PHYSICAL words do not.  Every word is 1024 bits because the array has
//     one width; head and tail use 192 of theirs and ups.1 uses 512.  228,736
//     useful bits out of 252,928.
//
// That costs nothing, because the array is *width-bound*: 52 columns of 512 words
// hold 512 tiles and the whole model is 247 of them, so the depth is free.  But
// the store is 247 words, not 220, and a loader that fills only the wide ones is a
// network running without its head and tail.  The image and the offset table come
// from tools/d64_weight_image.py, and `--golden` checks the image's weights
// reproduce the frozen D2 digests exactly before anything is loaded from it.
//
// The read is unconditional and the enable is applied to the output, which is the
// inference pattern the trunk store needed: an enabled read makes the mapper build
// logic memory instead of a block.  The `ram_style` attribute is load-bearing for
// the same measured reason.
//
//
// Which word was being lost, and where it is now not
// -------------------------------------------------
//
// **This store used to lose the last weight of every phase**, and it did it by
// answering a request with the enable of the wrong cycle.  The read is
// unconditional and the output gate used to be driven by the enable of the
// cycle the answer *arrived* in:
//
//     rd_data = rd_en ? q : 0        q <= mem[rd_addr]     (unconditional)
//
// The word for the address presented at cycle `t` is in `q` at `t+1`, so that
// gate asked every client to hold `rd_en` one cycle past its request.  Two of
// the three clients did it by accident of their shape and one could not:
//
//   * `d64_conv_engine.v:559` reads every cycle (`pe_wt_rd_en = 1'b1`), so its
//     enable never falls;
//   * `d64_head_ctrl.v:159` and `d64_down_engine.v:465` hold the enable across
//     the request *and* the capture cycle (`issue_slot || req_valid`), which is
//     why the front half passed every gate it ever ran;
//   * `d64_tconv_scheduler.v:472` presents one slot's read at a time and
//     `d64_tail_engine.v:501` is a nine-cycle window (`CY_W0`..`CY_W1`), so the
//     answer to the **last** read of a run arrives one cycle after the enable
//     has fallen and was gated to zero.
//
// Measured before the fix, by `tools/run_tail_input_stall.sh`, on every one of
// its arms:
//
//     real store   16384 answers the store refused (enable low, address 246)
//
// -- one per phase, always tile 246.  That testbench's engine was fed from a
// model of this store that holds its output, which is why the frame still came
// out right and nothing ever failed: **the real store could not drive the tail
// or the tconv engines at all, and only a behavioural stand-in could.**
//
// The word, named: for the tail it is tile **246** = `W_BASE_TL (238) + 8`, the
// ninth kernel tap (`kh = 2, kw = 2`), lost once per phase.
//
// **CORRECTED 2026-10-07 for the two ups layers, and the earlier text was
// wrong in the direction that understates the fault.**  It said "for `ups.0`
// the last tile the engine asks for, tile **219**, and for `ups.1` tile
// **237**.  One word per phase, in every decoder layer."  That came from
// `tb_d64_weight_store_lastword.v`'s `tconv` arm, whose read rhythm is
// **simplified** -- a run of consecutive reads with the enable then falling --
// and a rhythm is what decides which reads are the last one before the enable
// drops.  Measured against the **real** engines
// (`tb_d64_tconv_wstore.v`, `tools/run_tconv_wstore.sh`), with the pre-fix gate
// restored by `EN_SABOTAGE = 1`:
//
//     ups.0 wrong tile(s) 195 207 211 215 219     318 of 145924 reads
//     ups.1 wrong tile(s) 233 235                 128 of 147072 reads
//
// **Five tiles on `ups.0`, not one, and `ups.1`'s loss is not tile 237 at
// all.**  The reason is that `icg` is the innermost part of `w_rd_addr`: what
// is lost is the **last IC group of the tap that falls outside the right
// edge**, not the layer's final tile -- and a layer's genuinely final slot is
// dead and issues no read.  On the real engines that is 3263 and 1698 wrong
// output words respectively.  **The fix is not in question; the blast radius
// was.**  The tail's 246 is measured on the real chain and stands.
//
// **The whole tile, every lane of it.**  `rd_data` is gated as a word, so what
// the MAC array saw was not one wrong weight but a tile of 8 x 4 zero weights
// for the tail's last tap (8 x 16 for the two ups layers) -- the layer is
// computed with that tap contributing nothing, which comes out as a picture and
// not as an error.  Nothing downstream can distinguish it, which is why it
// survived every gate that fed the engine from a model.
//
// The fix, and the pattern it had to leave alone
// ----------------------------------------------
//
// The output gate is now driven by the enable of the cycle the read was
// **TRANSFERRED** -- one register behind `rd_en`:
//
//     rd_en_q <= rd_en;              rd_data = rd_en_q ? q : 0
//
// Nothing in the memory moved.  `q <= mem[rd_addr]` is still unconditional, the
// RAM's output register still sees nothing but `mem[rd_addr]`, and the mux is
// still after it.  That pattern is why the mapper builds a block RAM instead of
// logic memory, and `d64_burst_prefetch`'s header is the record of what a
// 61,440-bit array costs when the read-data path is a mux *ahead* of the output
// register (F3 section 3.14).  What changed is which cycle's enable the mux
// reads: one bit of control, nothing in the read path and nothing in the array.
//
// The convention this fixes the store to is the one every gate in this tree
// already modelled (`tb_d64_tail_engine.v:120`, `tb_d64_tail_input_stall.v:396`):
//
//     a read is TRANSFERRED in the cycle `rd_en` is high;
//     the word for that cycle's `rd_addr` is on `rd_data` the NEXT cycle;
//     and `rd_data` is zero in any cycle whose predecessor transferred nothing.
//
// The third line is not decoration.  The enable is on the output so that the
// memory infers, and the arm that used to lose the first line must not gain a
// "hold the last word" behaviour instead: a store that leaves another client's
// tensor standing on the port returns wrong data with no error, which is the
// failure shape this project has now found in `bank_idle` (F2 section 2.1),
// the cache's region re-point (F3 3.8), the cache's `rd_abort` (F3 6.12) and
// the tail's missing `in_stall` (F3 6.9).  Both lines are gated, on every cycle
// of every burst, by `tools/run_weight_store_lastword.sh`; the joint arm is
// `tools/run_tail_wstore.sh`.
//
// **The mapping, measured.**  `synth/d64_weight_store_synth_top.v` is this
// store's own measurement root and `efx_map` was run against it before and
// after this change, at `TILES 247 / WORD_W 1024 / ADDR_W 8`:
//
//     before (HEAD)   52 RAM10   1432 LUT4   1329 FF
//     after           52 RAM10   1432 LUT4   1330 FF
//
// -- **one flip-flop, and nothing else.**  `52` is also the count the board's
// own map carries for this store, so the two runs are like for like.  The
// `EN_SABOTAGE` arm costs nothing in the same table read the other way: at 0
// the mapper folded it away, or the LUT4 count would have moved.
//
// What that leaves unverified is narrower than it was: the RAM10 count is
// measured for this root, and not for every root this file is compiled into.
module d64_weight_store #(
    parameter TILES  = 247,     // every layer, not the wide ones alone
    parameter WORD_W = 1024,
    parameter ADDR_W = 8,
    // Optional board deployment image. It initializes this exact store in
    // place; no parallel ROM or duplicate weight RAM is created.
    parameter INIT_FILE = "",
    // Verification only, folded away at 0.  **The pre-fix output gate**: it
    // qualifies the answer with the enable of the cycle the answer arrives in
    // instead of the cycle the read was transferred in, which gates the last
    // word of every phase to zero.  Both gates in `tools/` that cover this
    // store require this arm to FAIL on data, and `tb_d64_weight_store_lastword.v`
    // requires it to fail at exactly the words the fix names above.
    parameter integer EN_SABOTAGE = 0
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // the loader port: weights arrive from off chip at a layer switch, and the
    // load and the read never overlap, so one port of each kind is enough
    input  wire                  wr_en,
    input  wire [ADDR_W-1:0]     wr_addr,
    input  wire [WORD_W-1:0]     wr_data,

    // the MAC array's operand port: one whole tile per cycle
    input  wire                  rd_en,
    input  wire [ADDR_W-1:0]     rd_addr,
    output wire [WORD_W-1:0]     rd_data
);

    (* ram_style = "block" *) reg [WORD_W-1:0] mem [0:TILES-1];
    reg [WORD_W-1:0] q;

    generate
        if (INIT_FILE != "") begin : g_weight_image_init
            initial $readmemh(INIT_FILE, mem);
        end
    endgenerate

    // **The enable the output gate reads, and the whole of the fix.**  A read
    // is transferred in the cycle `rd_en` is high; `q` takes `mem[rd_addr]` on
    // that edge and holds it through the next cycle, which is the cycle the
    // word is due.  So the gate belongs to the *request's* cycle and not to the
    // answer's, and that is one register behind `rd_en`.
    //
    // `rst_n` was in this module's port list and read by nothing before this;
    // it now resets the one bit of control state it has.  The RAM is untouched:
    // a store whose `q` is reset is a store whose output register is no longer
    // `mem[rd_addr]` alone.
    reg rd_en_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_en_q <= 1'b0;
        else        rd_en_q <= rd_en;
    end

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        q <= mem[rd_addr];
    end

    assign rd_data = (EN_SABOTAGE ? rd_en : rd_en_q) ? q : {WORD_W{1'b0}};

endmodule
