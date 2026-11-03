`timescale 1ns/1ps

// F3 section 3.16 -- the decoder's side of the one weight store.
//
// The 247-tile store already holds the decoder's weights.  `tools/d64_network_image.py`
// writes tiles 0..246 and says of 184..246 in its own output line, "ups, tail;
// not read yet"; `tools/d64_decoder_image.py` rewrote those 63 into the lane order
// `d64_mac128_compat_map` asks for and left them in `golden/d64_decoder_image.mem`.
// **Nothing reads them.**  This module is what reads them, and it also carries the
// two things a decoder engine needs beside its weights: the requant and the bias.
//
// A third time-multiplexed client, not a third port
// -------------------------------------------------
//
// `memory/d64_weight_store.v` has **one** read port and is instantiated once, at
// `control/d64_encoder_trunk.v:751`, for the encoder and the five trunk blocks.
// The decoder's three layers run **in sequence** -- ups.0, then ups.1, then tail --
// and never overlap each other or the front half, so the store does not need a
// second port.  It needs a second *client*, and this is it: the three decoder
// engines' own `w_rd_en`/`w_rd_addr` are muxed onto the store's one port by
// `layer`.
//
// **What this module does not touch is the 1024-bit data bus.**  `w_rd_data` is
// `st_rd_data` assigned through; there is no register, no mux and no gate in it.
// That is deliberate and it is the alignment argument, not a size argument:
// `d64_tconv_engine.v`'s pipeline has the store answering slot N at T+2 with
// stage B latching it on that same edge, and a register anywhere between the
// store's output register and the engine's stage B moves every weight one cycle
// off its activation.  The mux is in the 8-bit address and the enable, where the
// engine's own scheduler already assumes exactly one cycle of store latency.
//
// The three arms, and the rule that makes the switch safe
// ------------------------------------------------------
//
// Because the address is muxed *combinationally*, the layer a read belongs to is
// the layer that was selected when the store latched the address -- one cycle
// before the data comes back.  So a switch that lands between a request and its
// return hands one engine another tensor's tile: correct-looking data, wrong
// weights, no error anywhere.  That is the same shape as `bank_idle` in
// F2_INTERFACES section 2.1 and the same shape as the region re-point in section
// 3.8, and it gets the same treatment:
//
//     **`layer` may change only while `sw_safe` is high.**
//
// `sw_safe` is high when no read was accepted this cycle and none was accepted
// on the previous one -- i.e. when nothing is in flight.  It is the hardware's
// statement of the rule; the testbench drives a switch with `sw_safe` low and
// requires the read-back to be *another layer's tile*, so the rule is shown to be
// load-bearing rather than asserted.  In the network the switch is a layer
// boundary, where the previous engine has drained and `done` has been seen, so
// the rule is satisfiable by construction -- which is why it is stated here and
// checked rather than engineered around.
//
// Where the bias comes from -- the thing that is easy to forget
// ------------------------------------------------------------
//
// `golden/d64_decoder_cfg.mem` is 27 words and carries **M, shift, zero, qmin and
// qmax**.  It does not carry the bias: `tools/d64_decoder_image.py` says so in its
// own docstring ("The decoder has no bias in this table"), because the bias is not
// a requant field -- it is the bank's third operand, and both decoder engines take
// it as `rq_bias[OC_LANES*ACC_W-1:0]`.  A decoder built from that table alone runs
// with a bias of zero and produces a plausible, wrong image.
//
// The bias is in **`golden/d64_int8.json`**, `layers[].bias_int32`, one value per
// output channel -- the frozen D2.1 contract, the same file the weights and the
// requant already come from, read by **`tools/d64_decoder_bias_rom.py`**, which
// transcribes it into `golden/d64_decoder_bias.mem` (27 words, two's complement
// 32-bit) and checks that `golden/d64_decoder_cfg.json`'s `bias_int32` field
// carries the same 27 values for the same 27 (layer, oc) pairs.  Nothing here
// re-derives it.
//
// Both ROMs are indexed by the same `(layer, offset)` pair, so this module holds
// one window table and both tables share it.
//
// The window table, and why section 2 appears here as well as in the engines
// -------------------------------------------------------------------------
//
// `W_BASE` is a parameter of the *engine* (`d64_tconv_scheduler.v:51`,
// `d64_tail_engine.v:133`) and that is where the tile index is computed:
//
//     w_rd_addr = W_BASE + ((kh*3 + kw) * IC_GROUPS + icg) * OC_GROUPS
//
// This module carries the same three bases and the same three tile counts for two
// purposes of its own -- to range-check the address it is asked to pass on (an
// engine addressing outside its layer is reading another layer's tensor) and to
// address the two ROMs.  So section 2 is transcribed twice, and a divergence
// between the copies is a real defect with no error message.  The gate drives the
// engine side from the *testbench's* copy of section 2 and requires the read-back
// at the index section 2 gives to equal the image's tile at that index, so what a
// one-tile divergence costs is measured (`-gTB_BASE_SHIFT=1`) and not argued.
//
// Two numbers that are not the same width, and the hazard between them
// -------------------------------------------------------------------
//
// The tail engine's address port is `W_ADDR_W = 12` bits
// (`d64_tail_engine.v:134`) and the store's is 8.  A tail address of 256 or more
// therefore **wraps silently** at this mux: the four high bits are dropped and the
// read goes to tile 0..255 of the same store.  With `W_BASE = 238` and nine taps
// the real addresses are 238..246, so the hazard is not live today -- but the tail
// engine's `W_BASE` **defaults to 0** (`d64_tail_engine.v:133`), and a tail
// instantiated without it reads tiles 0..8, which are the encoder's, and the
// result is a plausible image rather than an error.  `addr_over` is this module
// saying so: it is high whenever the tail port presents an address with any bit
// above the store's width set.  The engine's `W_BASE` is not this module's to set.
//
// The one store, and one rule for all three clients
// ------------------------------------------------
//
// `memory/d64_weight_store.v` used to answer a request with the enable of the
// wrong cycle -- the cycle its answer arrived in rather than the cycle the read
// was transferred in -- and the three clients paid for it differently.  The
// front half held `rd_en` across its capture cycle and was unaffected; a
// one-per-slot pulse (`d64_tconv_scheduler.v:472`) and the tail's nine-cycle
// window (`d64_tail_engine.v:501`) each lost the **last** tile of every phase to
// a zero.  The store now states one rule, and it is the rule all three clients
// already express:
//
//     a read is TRANSFERRED in the cycle `rd_en` is high;
//     the word for that cycle's `rd_addr` is on `rd_data` the next cycle;
//     and `rd_data` is zero in any cycle whose predecessor transferred nothing.
//
// Nothing in this module had to move for it: the address and the enable are
// still muxed combinationally onto one port and the 1024-bit word is still
// straight through, with no register.  What the rule does is give this module's
// two state outputs their exact meaning:
//
//     sw_safe       nothing is in flight -- no transfer this cycle or last
//     st_rsp_valid  the word on `w_rd_data` right now is the answer to the
//                   transfer presented last cycle, and it belongs to `layer_q`
//
// **What the two owners must agree on.**  The one store's read port is handed
// between two owners -- this module and the front half's owner mux
// (`d64_encoder_trunk.v`'s `sel_enc`, which leaves the module as `ws_req_*`) --
// and a hand-over is only safe if both answer the same question the same way.
// They must agree on all three of:
//
//   1. **the same definition of "nothing in flight"**: a transfer this cycle or
//      last -- `sw_safe` here, `ws_idle` there, and the owner of the store ANDs
//      the two.  Neither is sufficient alone and both are the same expression on
//      purpose: two different expressions for one event is how a hand-over lands
//      between a request and its return.
//   2. that the hand-over happens **only** in a cycle both are high.
//   3. **that the third line of the rule is what makes the hand-over safe.**
//      Under the old convention the port still held the outgoing client's last
//      word on the first cycle it was idle, which is why
//      `d64_encoder_trunk.v`'s header records an extra cycle of margin.  With
//      the enable taken from the transfer's own cycle, `rd_data` is **zero** in
//      every cycle whose predecessor transferred nothing, so the incoming client
//      cannot pick up a word it did not ask for and that margin is no longer
//      load-bearing -- though keeping it costs nothing and nothing here requires
//      it to be dropped.  What used to be a convention is now a measurement:
//      `tools/run_weight_store_lastword.sh` counts the idle cycles that carry
//      data, and requires the count to be zero.
//
// The intent is still to switch at a layer boundary, where the previous engine
// has drained and `done` has been seen, so all of this is satisfied by
// construction.  What changed is that a violation is now a zero on the port
// instead of another layer's tensor, silently.
//
// What is NOT verified
// --------------------
//
//   * **No synthesis, no board, no resource claim.**  Nothing in this file has
//     been through `efx_map` or onto the silicon; the only claim is the simulation
//     in `tb_d64_decoder_wreader.v`, reported in
//     `docs/DECODER_WEIGHT_READER_2026-10-07.md`.
//   * **The store's mapping is the one thing here with a precedent.**
//     `memory/d64_weight_store.v`'s "unconditional read, enable on the output"
//     idiom is the one `memory/d64_tc_ram.v` uses and the one section 3.14 found
//     `d64_burst_prefetch` does not -- so the store is expected to map, and that
//     expectation is *not measured here*.  **Anything this module adds beside it
//     has no precedent**: a 27x64 ROM and a 27x32 ROM read 16 lanes at a time is
//     1.7 kb and 0.86 kb of table fanned out over 16 lanes, which on the Efinity
//     mapper is distributed logic and not block RAM.  It is small; it is also
//     unmeasured, and section 3.14 is the reason that sentence is here.
//   * **No engine is instantiated.**  The three request ports are driven by the
//     testbench standing in for the schedulers, not by the schedulers.  The
//     engines' `W_BASE` parameters are set by whoever instantiates them, which is
//     the network top this section's task does not build.
//   * **The zero/qmin/qmax fields are carried and not consumed.**  The table has
//     them, both engines take `QMIN`/`QMAX` as parameters (`0`/`255`, which is what
//     all 27 entries say), and `rq_zero` is not a decoder engine port at all -- it
//     is `d64_mac128_compat_map`'s, driven from the bank.  They are brought out so
//     the table is checkable end to end, and they are marked here as read by
//     nothing.
module d64_decoder_wreader #(
    // ---- the store ---------------------------------------------------------
    parameter W_ADDR_W   = 8,
    parameter W_WORD_W   = 1024,

    // ---- the requant table's widths, `d64_down_cfg_rom.v`'s field split ----
    parameter OC_MAX     = 16,     // PE_OC_MAX: the widest lane set of the three
    parameter M_W        = 16,
    parameter SHIFT_W    = 5,
    parameter ACC_W      = 32,
    parameter ZQ_W       = 10,
    parameter M_BIT      = 32,
    parameter SHIFT_BIT  = 48,
    parameter ZERO_BIT   = 0,
    parameter QMIN_BIT   = 10,
    parameter QMAX_BIT   = 20,
    parameter CFG_WORDS  = 27,
    parameter CFG_INIT   = "",     // golden/d64_decoder_cfg.mem
    parameter BIAS_WORDS = 27,
    parameter BIAS_INIT  = "",     // golden/d64_decoder_bias.mem
    parameter CFG_BIAS_W = 32,
    parameter integer RUNTIME_CFG = 0,
    parameter integer CFG_STREAM_SRL = 0, // 27 full bias/M/shift words loaded while idle

    // ---- docs/F3_INTERFACES_2026-10-07.md section 2, transcribed -----------
    parameter W_BASE_U0  = 184, parameter W_TILES_U0 = 36,
    parameter W_BASE_U1  = 220, parameter W_TILES_U1 = 18,
    parameter W_BASE_TL  = 238, parameter W_TILES_TL =  9,
    //: The two ROMs are indexed by the same pair, so the table's entry offsets
    //: live here too: 16 ups.0 entries, then 8 ups.1, then 3 tail.
    parameter CFG_OFF_U0 = 0, parameter CFG_OFF_U1 = 16, parameter CFG_OFF_TL = 24,
    parameter CFG_N_U0   = 16, parameter CFG_N_U1   =  8, parameter CFG_N_TL   =  3,

    // Verification only, folded away at 0.
    parameter integer MUX_SABOTAGE = 0   // the ups.1 and tail select arms exchanged
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  dec_cfg_we,
    input  wire [4:0]            dec_cfg_addr,
    input  wire [63:0]           dec_cfg_data,

    // ---- which layer owns the store this phase -----------------------------
    //: 0 ups.0, 1 ups.1, 2 tail, **3 the external port** (a loader or a
    //: testbench; it is also what the `default` arm selects, so an undefined
    //: `layer` reads through the verification port rather than through a layer's
    //: window).  Arm 3 has **no window and no requant**: its `win_tiles` and
    //: `cfg_n` are zero, so a client that selects it reads tiles and cannot pick
    //: up the tail's table by accident.
    input  wire [1:0]            layer,

    // ---- the three engines' weight ports (they drive these) ----------------
    input  wire                  u0_w_rd_en,
    input  wire [W_ADDR_W-1:0]   u0_w_rd_addr,
    input  wire                  u1_w_rd_en,
    input  wire [W_ADDR_W-1:0]   u1_w_rd_addr,
    //: The tail engine's port is twelve bits wide and the store's is eight;
    //: `addr_over` is where that is said out loud.
    input  wire                  tl_w_rd_en,
    input  wire [11:0]           tl_w_rd_addr,

    // ---- the external / verification port ---------------------------------
    input  wire                  ext_w_rd_en,
    input  wire [W_ADDR_W-1:0]   ext_w_rd_addr,

    // ---- the one store's read port ----------------------------------------
    output wire                  st_rd_en,
    output wire [W_ADDR_W-1:0]   st_rd_addr,
    input  wire [W_WORD_W-1:0]   st_rd_data,
    //: Straight through.  See the header: a register here is a one-cycle skew
    //: between every weight and its activation.
    output wire [W_WORD_W-1:0]   w_rd_data,
    //: **The answer half of the store's rule.**  High exactly in the cycle the
    //: word on `w_rd_data` is the answer to the transfer presented in the
    //: previous cycle, for all four arms alike -- which is what makes the
    //: request/response validity the same statement here as it is at the store.
    //: It is the transfer registered, so it is also the whole of `sw_safe`'s
    //: second term; and because `sw_safe` forbids a `layer` change while a
    //: transfer is in flight, a high `st_rsp_valid` implies the word belongs to
    //: `layer_q`.  A client that stalls may gate on it; a client that does not
    //: is reading the port in a cycle its own scheduler already knows about.
    output wire                  st_rsp_valid,

    // ---- the layer's requant, one entry per lane ---------------------------
    output wire [OC_MAX*M_W-1:0]     rq_m,
    output wire [OC_MAX*SHIFT_W-1:0] rq_shift,
    output wire [OC_MAX*ACC_W-1:0]   rq_bias,
    output wire [OC_MAX*ZQ_W-1:0]    rq_zero,
    output wire [OC_MAX*ZQ_W-1:0]    rq_qmin,
    output wire [OC_MAX*ZQ_W-1:0]    rq_qmax,
    //: Which lanes the selected layer really has.  The tail is `MAC_P4_I8_O4`
    //: against a three-channel layer, so lane 3 of 4 is a fill and reads zero
    //: here -- the contract has no value for it (`d64_decoder_cfg.json`'s tail
    //: entry is `cout: 3` with no fill field) and `d64_tail_engine` drops it.
    output wire [OC_MAX-1:0]         cfg_lane_ok,

    // ---- the window, and the state the switch rule needs -------------------
    output wire [W_ADDR_W-1:0]   win_base,
    output wire [W_ADDR_W-1:0]   win_tiles,
    output wire [4:0]            cfg_off,
    output wire [4:0]            cfg_n,
    //: Is the address being presented inside the selected layer's window?
    output wire                  win_ok,
    //: Any bit above the store's address width on the tail port.
    output wire                  addr_over,
    //: **The switch rule.**  High exactly when nothing is in flight: no read
    //: accepted this cycle and none accepted on the previous one.  `layer` may
    //: change only while this is high.
    output wire                  sw_safe,
    //: The requant and bias registers are driven by `layer` through a clocked
    //: ROM read, so they are correct one cycle after `layer` settles.  A layer
    //: boundary is many cycles, so this is not a timing argument -- it is the
    //: flag that makes it not an argument.
    output wire                  cfg_settled
);

    // ------------------------------------------------------------------
    // the layer table -- section 2, and the two ROMs, share this frame
    // ------------------------------------------------------------------
    reg [W_ADDR_W-1:0] base_s;
    reg [W_ADDR_W-1:0] tiles_s;
    reg [4:0]          off_s;
    reg [4:0]          n_s;

    always @* begin
        case (layer)
            2'd0: begin base_s = W_BASE_U0; tiles_s = W_TILES_U0;
                        off_s = CFG_OFF_U0; n_s = CFG_N_U0; end
            2'd1: begin base_s = W_BASE_U1; tiles_s = W_TILES_U1;
                        off_s = CFG_OFF_U1; n_s = CFG_N_U1; end
            2'd2: begin base_s = W_BASE_TL; tiles_s = W_TILES_TL;
                        off_s = CFG_OFF_TL; n_s = CFG_N_TL; end
            //: The external port is **not a layer**: no window, and an empty
            //: requant table.  `n_s = 0` means the lane guard below never indexes
            //: the ROM, so `off_s` is not read either -- and a client that
            //: selects arm 3 cannot pick up the tail's M, shift or bias by
            //: accident.  It reads tiles and nothing else.
            default: begin base_s = {W_ADDR_W{1'b0}}; tiles_s = {W_ADDR_W{1'b0}};
                        off_s = 5'd0; n_s = 5'd0; end
        endcase
    end

    assign win_base  = base_s;
    assign win_tiles = tiles_s;
    assign cfg_off   = off_s;
    assign cfg_n     = n_s;

    // ------------------------------------------------------------------
    // the read port mux
    // ------------------------------------------------------------------
    // Combinational, and the reason is in the header: the store's own registered
    // read already costs the one cycle the scheduler accounts for.
    reg [W_ADDR_W-1:0] sel_addr;
    reg                sel_en;

    always @* begin
        case (layer)
            2'd0: begin sel_en = u0_w_rd_en; sel_addr = u0_w_rd_addr; end
            2'd1: begin
                if (MUX_SABOTAGE == 0) begin
                    sel_en = u1_w_rd_en; sel_addr = u1_w_rd_addr;
                end
                else begin
                    sel_en = tl_w_rd_en; sel_addr = tl_w_rd_addr[W_ADDR_W-1:0];
                end
            end
            2'd2: begin
                if (MUX_SABOTAGE == 0) begin
                    sel_en = tl_w_rd_en; sel_addr = tl_w_rd_addr[W_ADDR_W-1:0];
                end
                else begin
                    sel_en = u1_w_rd_en; sel_addr = u1_w_rd_addr;
                end
            end
            default: begin sel_en = ext_w_rd_en; sel_addr = ext_w_rd_addr; end
        endcase
    end

    assign st_rd_en   = sel_en;
    assign st_rd_addr = sel_addr;
    assign w_rd_data  = st_rd_data;

    //: `base + tiles` is the first tile of the next layer, and the store is
    //: eight bits wide, so the sum is taken wide and compared wide -- a compare
    //: that wrapped at 256 would silently pass an out-of-range address.
    wire [W_ADDR_W:0] win_end = {1'b0, base_s} + {1'b0, tiles_s};
    assign win_ok = (layer == 2'd3)
                  ? 1'b1                                  // not a layer: no window
                  : (({1'b0, sel_addr} >= {1'b0, base_s})
                     && ({1'b0, sel_addr} < win_end));

    assign addr_over = (layer == 2'd2) && tl_w_rd_en && (|tl_w_rd_addr[11:W_ADDR_W]);

    // ------------------------------------------------------------------
    // the switch rule
    // ------------------------------------------------------------------
    reg rd_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_q <= 1'b0;
        else        rd_q <= sel_en;
    end
    assign sw_safe = ~(sel_en | rd_q);
    //: The transfer of the previous cycle, and nothing else.  See the port: this
    //: is the answer to `rd_q`'s transfer and therefore to `layer_q`, which is
    //: also why `sw_safe` -- the same two terms, negated -- is a licence for the
    //: *next* cycle and not for this one.
    assign st_rsp_valid = rd_q;

    reg [1:0] layer_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) layer_q <= 2'd3;
        else        layer_q <= layer;
    end
    wire stream_ready;
    assign cfg_settled = (layer_q == layer) && (CFG_STREAM_SRL==0 || RUNTIME_CFG==0 || stream_ready);

    // ------------------------------------------------------------------
    // the requant ROM and the bias ROM
    // ------------------------------------------------------------------
    (* ram_style = "block" *) reg [63:0] cfg_rom  [0:CFG_WORDS-1];
    (* ram_style = "block" *) reg [31:0] bias_rom [0:BIAS_WORDS-1];
    reg [63:0] cfg_q  [0:OC_MAX-1];
    reg [CFG_BIAS_W-1:0] bias_q [0:OC_MAX-1];

    generate
        if (CFG_INIT != "") begin : g_cfg_init
            initial $readmemh(CFG_INIT, cfg_rom);
        end
        if (BIAS_INIT != "") begin : g_bias_init
            initial $readmemh(BIAS_INIT, bias_rom);
        end
    endgenerate

    // Constant-index layer selection prevents 16 independent variable-address
    // reads of the same 27-word table from becoming 32 duplicated RAM10s.
    // Runtime entries are individual registers, never an inferred multiport RAM;
    // bias and requant remain separate fields to avoid the cfg field-alias bug.
    wire [31:0] runtime_bias [0:BIAS_WORDS-1];
    wire [20:0] runtime_rq [0:CFG_WORDS-1];
    genvar ce;
    generate for (ce=0; ce<CFG_WORDS; ce=ce+1) begin : g_runtime_entry
        if (RUNTIME_CFG != 0 && CFG_STREAM_SRL==0) begin : g_live
            reg [CFG_BIAS_W-1:0] b;
            reg [20:0] r;
            always @(posedge clk) begin
                if (dec_cfg_we && dec_cfg_addr == ce) begin
                    b <= dec_cfg_data[CFG_BIAS_W-1:0];
                    r <= dec_cfg_data[52:32];
                end
            end
            assign runtime_bias[ce] = {{(32-CFG_BIAS_W){b[CFG_BIAS_W-1]}},b};
            assign runtime_rq[ce] = r;
        end else begin : g_static
            assign runtime_bias[ce] = 32'd0;
            assign runtime_rq[ce] = 21'd0;
        end
    end endgenerate


    // Production decoder profiles stream exactly entries0..26. Columns0..10
    // receive two entries, columns11..15 one. After a complete round the first
    // set occupies cell1/cell0 respectively; the second set occupies cell0.
    // Default CFG_STREAM_SRL=0 keeps the original random-entry write backend.
    wire [CFG_BIAS_W+20:0] stream_column[0:15];
    generate if(CFG_STREAM_SRL!=0 && RUNTIME_CFG!=0)begin:g_cfg_stream
      // Validity has the same lifetime as the unreset configuration storage.
      // Job/owner reset does not clear it. First entry0 of a new profile resets
      // the guard; a complete ordered round restores it. Model CRC validity
      // outside this module still owns whether any CNN job may start.
      reg [4:0] expected_address=0;
      reg loaded=0,bad=0;
      wire restarting=dec_cfg_we && dec_cfg_addr==0;
      wire write_ok=dec_cfg_we &&
        (restarting || (!bad && dec_cfg_addr==expected_address && dec_cfg_addr<27));
      assign stream_ready=loaded && !bad && !dec_cfg_we;
      always @(posedge clk)begin
        if(dec_cfg_we)begin
          if(restarting)begin expected_address<=1;loaded<=0;bad<=0;end
          else if(write_ok)begin
            expected_address<=dec_cfg_addr==26 ? 0 : expected_address+1'b1;
            loaded<=dec_cfg_addr==26;
          end else begin loaded<=0;bad<=1;end
        end
      end
      wire [CFG_BIAS_W+20:0] inverse_word=~{dec_cfg_data[52:32],dec_cfg_data[CFG_BIAS_W-1:0]};
      genvar column,bitno;
      for(column=0;column<16;column=column+1)begin:g_column
        wire ce=write_ok && dec_cfg_addr[3:0]==column;
        wire [2:0] history_address=(layer==0 && column<11)?3'd6:3'd7;
        for(bitno=0;bitno<CFG_BIAS_W+21;bitno=bitno+1)begin:g_bit
          EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
            .D(inverse_word[bitno]),.A(history_address),.CLK(clk),.CE(ce),
            .Q(stream_column[column][bitno]),.Q7());
        end
      end
    end else begin:g_no_cfg_stream
      assign stream_ready=1'b1;
      genvar column;
      for(column=0;column<16;column=column+1)assign stream_column[column]=0;
    end endgenerate

    // synthesis translate_off
    always @(posedge clk) if (dec_cfg_we && RUNTIME_CFG && CFG_BIAS_W < 32) begin
        if (dec_cfg_data[31:0] !== {{(32-CFG_BIAS_W){dec_cfg_data[CFG_BIAS_W-1]}},dec_cfg_data[CFG_BIAS_W-1:0]}) begin
            $display("DOMAIN_ERROR: decoder bias does not fit signed CFG_BIAS_W");
            $finish;
        end
    end
    // synthesis translate_on
    genvar lj;
    generate for (lj=0; lj<OC_MAX; lj=lj+1) begin : g_cfg_rd
        wire [63:0] word_u0;
        wire [63:0] word_u1;
        wire [63:0] word_tl;
        wire [31:0] bias_u0;
        wire [31:0] bias_u1;
        wire [31:0] bias_tl;
        if (lj<CFG_N_U0) begin : g_u0
            assign word_u0 = RUNTIME_CFG ? {11'd0,(CFG_STREAM_SRL?stream_column[lj][CFG_BIAS_W+:21]:runtime_rq[CFG_OFF_U0+lj]),2'd0,10'd255,20'd0} : cfg_rom[CFG_OFF_U0+lj];
            assign bias_u0 = RUNTIME_CFG ? (CFG_STREAM_SRL?{{(32-CFG_BIAS_W){stream_column[lj][CFG_BIAS_W-1]}},stream_column[lj][CFG_BIAS_W-1:0]}:runtime_bias[CFG_OFF_U0+lj]) : bias_rom[CFG_OFF_U0+lj];
        end else begin : g_u0_pad
            assign word_u0=64'd0; assign bias_u0=32'd0;
        end
        if (lj<CFG_N_U1) begin : g_u1
            assign word_u1 = RUNTIME_CFG ? {11'd0,(CFG_STREAM_SRL?stream_column[lj][CFG_BIAS_W+:21]:runtime_rq[CFG_OFF_U1+lj]),2'd0,10'd255,20'd0} : cfg_rom[CFG_OFF_U1+lj];
            assign bias_u1 = RUNTIME_CFG ? (CFG_STREAM_SRL?{{(32-CFG_BIAS_W){stream_column[lj][CFG_BIAS_W-1]}},stream_column[lj][CFG_BIAS_W-1:0]}:runtime_bias[CFG_OFF_U1+lj]) : bias_rom[CFG_OFF_U1+lj];
        end else begin : g_u1_pad
            assign word_u1=64'd0; assign bias_u1=32'd0;
        end
        if (lj<CFG_N_TL) begin : g_tl
            assign word_tl = RUNTIME_CFG ? {11'd0,(CFG_STREAM_SRL?stream_column[8+lj][CFG_BIAS_W+:21]:runtime_rq[CFG_OFF_TL+lj]),2'd0,10'd255,20'd0} : cfg_rom[CFG_OFF_TL+lj];
            assign bias_tl = RUNTIME_CFG ? (CFG_STREAM_SRL?{{(32-CFG_BIAS_W){stream_column[8+lj][CFG_BIAS_W-1]}},stream_column[8+lj][CFG_BIAS_W-1:0]}:runtime_bias[CFG_OFF_TL+lj]) : bias_rom[CFG_OFF_TL+lj];
        end else begin : g_tl_pad
            assign word_tl=64'd0; assign bias_tl=32'd0;
        end
        always @(posedge clk) begin
            case(layer)
                2'd0:begin cfg_q[lj]<=word_u0; bias_q[lj]<=bias_u0; end
                2'd1:begin cfg_q[lj]<=word_u1; bias_q[lj]<=bias_u1; end
                2'd2:begin cfg_q[lj]<=word_tl; bias_q[lj]<=bias_tl; end
                default:begin cfg_q[lj]<=64'd0; bias_q[lj]<=32'd0; end
            endcase
        end
    end endgenerate

    genvar gi;
    generate
        for (gi = 0; gi < OC_MAX; gi = gi + 1) begin : g_lane
            assign rq_m[gi*M_W +: M_W]             = cfg_q[gi][M_BIT +: M_W];
            assign rq_shift[gi*SHIFT_W +: SHIFT_W] = cfg_q[gi][SHIFT_BIT +: SHIFT_W];
            assign rq_zero[gi*ZQ_W +: ZQ_W]        = cfg_q[gi][ZERO_BIT +: ZQ_W];
            assign rq_qmin[gi*ZQ_W +: ZQ_W]        = cfg_q[gi][QMIN_BIT +: ZQ_W];
            assign rq_qmax[gi*ZQ_W +: ZQ_W]        = cfg_q[gi][QMAX_BIT +: ZQ_W];
            assign rq_bias[gi*ACC_W +: ACC_W]      = {{(ACC_W-CFG_BIAS_W){bias_q[gi][CFG_BIAS_W-1]}},bias_q[gi]};
            assign cfg_lane_ok[gi]                 = (gi < n_s);
        end
    endgenerate

endmodule
