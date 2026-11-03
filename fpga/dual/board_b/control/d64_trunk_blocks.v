`timescale 1ns/1ps

// D3c5-c -- five residual blocks through one engine, one weight store and two
// DDR buffers.
//
// D3c5-a and D3c5-b proved the block: its arithmetic, its schedule, its memory
// boundary, and -- after D3c5-b1 -- that the whole thing closes at 120 MHz.  What
// none of them proved is that it can *start again*.  Every gate so far began from
// a reset and ran exactly one block, and the trunk is five of them back to back
// with each one's input being the one before's output.
//
//     DDR buffer A  --block0-->  DDR buffer B
//     DDR buffer B  --block1-->  DDR buffer A
//     DDR buffer A  --block2-->  DDR buffer B
//     DDR buffer B  --block3-->  DDR buffer A
//     DDR buffer A  --block4-->  DDR buffer B
//
// Nothing is copied and nothing is resident: 256 KiB a tensor, two of them, and
// the engine's own 16-row ring is the only X storage in the design.
//
// The barrier, which is the whole point of this module
// ---------------------------------------------------
//
// **The engine's `done` is not the end of a block.**  `d64_trunk_scheduler`
// finishes when the schedule finishes, and D3c5-b1 put a pipeline stage after it
// -- the testbench for that stage found the difference by watching the write
// ports and counting 16,383 words instead of 16,384.  A barrier built on `done`
// would start block N+1 while the last word of block N was still in the
// pipeline, and block N+1 reads what block N wrote: the failure would be a
// handful of values at the end of a tensor, which is a picture and not a crash.
//
// So a block ends when three things are true, in this order:
//
//   1. `done`       the schedule has finished
//   2. `out_last`   the final trunk word reached the write FIFO
//   3. `wd_flag`    the far side has written everything it was given
//
// (3) arrives from the DDR side as one toggle, because there it is one event --
// the last write's response -- and splitting it here would be inventing a
// distinction the memory does not have.
//
// The restart gate
// ----------------
//
// A block boundary re-arms the fill sequencer: which row is wanted, how far into
// it the fill is, and the band scan.  It does **not** reset the ring, the FIFOs,
// or the evidence counters, because those are supposed to have been left in a
// known state by the block that just ended -- and resetting them would turn "the
// ring still holds a row nobody released" from a failure into a silence.
// `blk_clean` is checked on the cycle before the re-arm and a violation is
// counted, never corrected.
//
// What is deliberately absent
// ---------------------------
//
// No AXI master.  The DDR side of this module is the same row-request and
// beat-stream contract `d64_h0_block` has, plus the buffer select, so that this
// stage measures the *control* seam and not the vendor controller's arbitration.
// `board/ddr3/ddr3_h0_replay.v` is where that goes next.
module d64_trunk_blocks #(
    parameter ROWS        = 64,
    parameter COLS        = 64,
    parameter BAND        = 8,
    parameter A_ROWS      = 11,
    parameter B_ROWS      = 8,
    parameter X_RING_ROWS = 16,
    parameter RF_AW       = 5,
    parameter WF_AW       = 11,

    parameter NB          = 5,
    parameter integer TRUNK_FIRST_BLOCK = 0,
    parameter integer TRUNK_BLOCK_COUNT = NB,        // residual blocks in the trunk
    parameter CFG_NB      = 64,       // requant entries a block
    parameter WBASE0      = 54,       // block 0's weight tile base
    parameter WSTRIDE     = 26,       // tiles a residual block
    parameter CFG_AW      = 9,        // log2(NB * CFG_NB) = log2(320)
    parameter CFG_BIAS_W  = 32,
    parameter integer CFG_STREAM_SRL=0,
    parameter BLK_W       = 3,

    parameter W_TILES     = 247,
    parameter W_WORD_W    = 1024,
    parameter W_ADDR_W    = 8,
    parameter C1_CYC      = 4,
    parameter C2_CYC      = 18,
    parameter C3_CYC      = 4,
    parameter C3_IC       = 8,
    parameter Z1          = 413,
    parameter ZERO_T      = 413,
    parameter C2_DIAG = 0,
    parameter A_ROM_INIT = "../../golden/a_consumer_rows01_256x64.mem",
    parameter B_ROM_INIT = "../../golden/b_head128_128x64.mem",
    // **The stage monitor's expected-chunk table, and it is the shape's.**  The
    // window is `4 * ROWS` chunks, so a 36-row run is measured against a
    // 144-entry table and not against the square's 256.  It travels as a
    // parameter rather than being written in `d64_stage_chunk_crc` for the same
    // reason `A_ROM_INIT` does: the module cannot know which directory the flow
    // runs from, and a table that does not open is silent -- the ROM keeps the
    // zeros it clears itself to and every chunk reports a mismatch.  At 64 rows
    // this default is the file it always was.
    parameter CHUNK_ROM_INIT = "../../golden/d64_chunk_crc32.mem",
    // D3c6-e.  Passed straight to `d64_h0_block`; at 0 nothing about this
    // module changes, which is what keeps D3c5-c's five-block regression the
    // same design it closed on.
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0
) (
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire                clk_pe,
    input  wire                clk_axi,
    input  wire                arst_n,

    input  wire                start,       // begin the whole five-block run
    output wire                busy,
    output wire                done,
    output wire [31:0]         cycles,      // PE cycles for all five blocks
    output wire [BLK_W-1:0]    blk,
    // Which DDR buffer this block reads and which it writes.  Block 0 reads A,
    // block 1 reads what block 0 wrote, and so on; nothing is copied.
    output wire                src_buf,
    output wire [31:0]  trk_add_cnt,
    output wire [31:0]  trk_add_diff,
    output wire [31:0]  trk_no_yv,
    output wire [31:0]  trk_zero_br,
    output wire [31:0]  trk_tw_cnt,
    // D3c6-e6b: lane 0 of the conv3 requant word against the image's entry for
    // that block, one sticky bit a block, from the engine's own cfg.
    output wire [4:0]   trk_cfg_ok,
    output wire cfg_protocol_fault,
    // **The config image, checked against the golden CRC before anything runs.**
    // Five of the three hundred and twenty entries were verified by the stream
    // probes (`cfg_local_ok` and `stream_ok` compare slot 32 of every block) and
    // the rest were not.  The image is loaded at run time by a 320-cycle walk over
    // `cfgrom`, so unlike the weights it cannot be audited from the netlist -- the
    // only way to know what is in that array on the device is to read it back
    // there.  `tools/d64_trunk_cfg_rtl.py` records the image's CRC32 and this
    // compares against it, so one bit answers all 320.
    output wire         cfg_image_ok,
    output wire         cfg_selftest_done,
    output wire [4:0]   trk_stream_ok,
    output wire [20:0]  trk_refill_c3,
    output wire         trk_refill_seen,
    output wire         trk_refill_ok,
    //
    // D3c6-e7: the stage monitor's result, and nothing else -- the window it
    // folds over is generated here, because this is where the block index and
    // the redo counter live and the engine has no idea what block it is in.
    output wire [272:0] trk_obs_chunk,
    output wire [287:0] trk_c2_chunk,
    output wire [2:0]    trk_c2_dbg,
    // P6i: the harness's verdict that this block's input was stale, and the
    // redo's own evidence.  `cnt` reaching 7 means the reread was stale too
    // and the block advanced anyway -- counted, not absorbed.
    //
    // **The raw mismatch, not a permission to retry.**  The harness used to send
    // `blk_redo_r && !tr_redo_full`, which answers both questions with one wire:
    // at `redo_cnt == 7` the masked request is already 0, so the bound branch
    // below could not be reached and the sticky it sets could not be set.  The
    // gate carried a term that no run could ever fail -- which is why every P6t
    // capture reads `redo_bound = 0`.  Decision and permission are separate now:
    // this is the decision, and `trk_redo_full` is the permission.
    input  wire         blk_redo,
    output wire [2:0]   trk_redo_cnt,
    output wire         trk_redo_seen,
    // **The bound, exported.**  At `redo_cnt == 7` this block advances whether or
    // not the mismatch is still there -- so the AXI side has to advance with it.
    // It did not: `hold_blk` was `blk_redo_w`, which stays high, and the two sides
    // parted company with the PE on the next block and the adapter still
    // addressing and CRC-ing the previous one.  That is the whole of the nine
    // broken P6m captures, and it is a wiring rule, not a timing one.
    output wire         trk_redo_full,
    // **The bound was reached somewhere in this run.**  A block that advanced with
    // its input CRC still wrong has computed from the wrong tensor, and the run
    // must not report completion as though it had not.  This is a sticky and it
    // is a fail term in `board_gate_pass`.
    output reg          trk_redo_bound,
    // **One pass's compute, checked where the pass ends.**  `compute_sum` is a
    // running total, so a run that redid a block has six engine runs in it and
    // 648,720 cycles where a clean run has 540,600.  A gate that says
    // `== 540600` therefore rejects a recovery -- the second review's fourth
    // finding -- and the total becomes a floor.  This is the part that still says
    // each *pass* was right: every engine run owes the same core count, and one
    // that does not is a sticky the gate fails on.  A sum cannot see one bad pass
    // among six; this can.
    output reg          trk_eng_bad,
    output wire [3:0]   trk_p7_ok,
    output wire [1:0]   trk_p7_seen,
    output wire [63:0]  trk_p7_acc, trk_p7_fs,
    output wire         trk_p7_biaseen,
    output wire [1:0]   trk_p7_cfg_seen,
    output wire [63:0]  trk_p7_bias, trk_p7_br,
    output wire [31:0]  trk_p7_cfg0, trk_p7_cfg1,
    output wire [11:0]  trk_p7_ctrl,
    output wire                dst_buf,

    // host load, once, before `start`
    input  wire                w_we,
    input  wire [W_ADDR_W-1:0] w_wa,
    input  wire [W_WORD_W-1:0] w_wd,
    // Asserted once the loader has finished, so the self-test cannot read an
    // array that is still being written.
    input  wire                cfg_selftest,
    input  wire                c_we,
    input  wire [CFG_AW-1:0]   c_wa,
    input  wire [63:0]         c_wd,

    // ---- D3c6-e: the shared-PE seam, `EXTERNAL_PE` only -------------------
    // One group for the whole module, because one block runs at a time.  The
    // barrier above is what makes that true, and it is the same barrier that
    // has to hold before a shared bank may be handed from one block to the
    // next -- so the seam does not need a second one.
    output wire                pe_req_slot_valid,
    output wire [1407:0]       pe_req_act,
    output wire [1023:0]       pe_req_wgt,
    output wire                pe_req_pix_start,
    output wire [4095:0]       pe_req_bias,
    output wire                pe_req_rq_start,
    output wire [255:0]        pe_req_rq_m,
    output wire [79:0]         pe_req_rq_shift,
    output wire [9:0]          pe_req_rq_zero,
    output wire [9:0]          pe_req_rq_qmin,
    output wire [9:0]          pe_req_rq_qmax,
    input  wire                pe_rsp_rq_ready,
    input  wire                pe_rsp_rq_ready_n,
    input  wire [79:0]         pe_rsp_y,
    input  wire [7:0]          pe_rsp_y_valid,
    input  wire                pe_rsp_rq_busy,
    input  wire                pe_rsp_rq_done,
    input  wire                pe_rsp_rq_overrun,
    input  wire [31:0]         pe_rsp_mode_viol,
    input  wire [4095:0]       pe_rsp_acc,
    input  wire [4095:0]       pe_rsp_snapshot,
    output wire                pe_wt_rd_en,
    output wire [W_ADDR_W-1:0] pe_wt_rd_addr,
    input  wire [W_WORD_W-1:0] pe_wt_rd_data,

    // ---- the DDR side, `clk_axi` ------------------------------------------
    output wire [6:0]          x_req_row,
    // **The buffer the request is for rides with the request.**  `src_buf`
    // already says which buffer block N reads -- it is `blk_r[0]`, and block N
    // reads what block N-1 wrote because `dst_buf(N-1) = ~blk_r[0](N-1) =
    // blk_r[0](N)`.  What it cannot do is cross to `clk_axi` on its own: the
    // AXI side was comparing it against a synchronised completion toggle, which
    // makes the answer depend on which side of the crossing won a race.  Sent
    // with the row, and latched with the row at the same edge, it is not
    // sampled at all.
    output wire                x_req_buf,
    output wire                x_req_tog,
    input  wire                x_ack_tog,
    input  wire                rf_wr_en,
    input  wire [127:0]        rf_wr_data,
    output wire                rf_wr_full,
    input  wire                wf_rd_en,
    output wire [127:0]        wf_rd_data,
    output wire                wf_rd_valid,
    output wire                wf_rd_empty,
    output wire [WF_AW:0]      wf_rd_level,
    input  wire                wr_done_tog,  // the far side finished this block

    // ---- evidence ---------------------------------------------------------
    output reg  [31:0]         cfg_cycles,
    output reg  [31:0]         bar_cycles,
    output reg  [31:0]         clean_viol,   // a block started dirty
    output reg  [31:0]         blk_starts,
    output reg  [31:0]         compute_sum,
    output wire [31:0]         wait_cycles,
    output wire [31:0]         x_ovf,
    output wire [31:0]         x_rbf,
    output wire [31:0]         rf_udf,
    output wire [31:0]         rf_wait,
    output wire [31:0]         wf_ovf,
    output wire [31:0]         dep_viol,
    output wire [31:0]         x_beats,
    output wire [31:0]         wf_words,
    output wire [31:0]         rq_overruns,
    output wire                out_last,
    output wire                blk_clean,
    output wire [5:0]          clean_why,
    output reg  [5:0]          clean_first,
    output reg  [31:0]         cfg_writes
);

    // **The schedule's own cycle count, derived from the schedule.**
    // `ENG_CYCLES` was the literal 108,120, which is the *measured* 64-row
    // pass -- and it was compared for equality, so at `ROWS = 36` (a measured
    // 60,919) every pass was "bad" and `trk_eng_bad` -- a `board_gate_pass`
    // term -- was set on a correct run.  The three components are the
    // scheduler's own: `C1_CYC`/`C2_CYC`/`C3_CYC` (4/18/4) cycles a pixel, three
    // mode switches of `SWITCH_CYCLES` in each band, and the drain.
    //
    // **And it is a floor now, not an equality.**  At 64 rows the derivation
    // below is 106,496 + 1,536 = 108,032 and the measurement is 108,120; at 36
    // rows it is 59,904 + 960 = 60,864 against a measured 60,919.  The 88-cycle
    // drain the old number also carried is the one term that is *not* the
    // schedule's -- it is a measurement of the 64-row last band -- so it is not
    // in the floor at all, and the check is "this pass computed at least what
    // its own schedule owes".  A pass that computed less did not run the
    // schedule; one that computed more is a wait, which `wait_cycles` carries
    // where a total cannot.
    localparam [31:0] ENG_SLOTS_PER_PIX = C1_CYC + C2_CYC + C3_CYC;
    localparam integer ENG_BANDS = (ROWS + BAND - 1) / BAND;
    localparam [31:0] ENG_CYCLES_FLOOR = ENG_SLOTS_PER_PIX * (ROWS * COLS)
                                       + 32'd3 * ENG_BANDS * 32'd64;
    localparam [CFG_AW-1:0] CFG_N  = CFG_NB[CFG_AW-1:0];
    localparam [BLK_W-1:0]  NB_Q   = NB[BLK_W-1:0];
    localparam integer SEG_END = TRUNK_FIRST_BLOCK + TRUNK_BLOCK_COUNT;
    localparam integer SEG_WBASE = WBASE0 + TRUNK_FIRST_BLOCK * WSTRIDE;
    localparam integer SEG_CFG_BASE = TRUNK_FIRST_BLOCK * CFG_NB;
    localparam [2:0] S_IDLE = 3'd0, S_CFG = 3'd1, S_RUN = 3'd2, S_BAR = 3'd3,
                     S_DONE = 3'd4;
    reg  [2:0] state;
    // The wait counter is cumulative across the run, so what a block actually
    // spent waiting is the difference between two of these.
    reg  [31:0] wait_at [0:(NB > 0 ? NB : 1) - 1];
    // **P6c lived here and is gone.**  It held the barrier off for 64 cycles
    // after `wd_flag && last_seen`, the same settle the encoder -> block 0
    // handover needed, and it made the inter-block chain *worse*: intact in seven
    // of ten board runs against eight of ten without it, with both failures at a
    // boundary it was meant to cover.  The two boundaries are not the same fault.
    // The encoder-side fence stays -- it is in the board top, not here -- and this
    // one is removed rather than left at zero, because a disabled counter in the
    // PE domain is logic in the domain that has 0.069 ns of slack for nothing.
    //
    // What replaced it is P6d, in `board/ddr3/d64_encoder_trunk_board_top.v`: the
    // residual CRC moved onto the `m1` W handshake, and four masks that compare
    // the addresses the arbiter *accepted* against the base each block should use.
    // Every break P6c found was at a buffer-A boundary, and no reading the board
    // had could tell a read of A that went to B from a read of A that went to A
    // and got old data.
    integer wi;

    wire pe_rst_n, axi_rst_n;
    d64_reset_sync u_pe_rst  (.clk(clk_pe),  .arst_n(arst_n), .rst_n(pe_rst_n));
    d64_reset_sync u_axi_rst (.clk(clk_axi), .arst_n(arst_n), .rst_n(axi_rst_n));

    // ------------------------------------------------------------------
    // the requant image, resident
    // ------------------------------------------------------------------
    //
    // Sixty-four entries a block -- conv1's at 0..15, conv2's at 16..31, conv3's
    // at 32..63, the engine's own layout -- for five blocks.  Loaded once and
    // streamed in at each boundary, which is sixty-five cycles against a block's
    // 108,120 and therefore free.  It is also why the engine needs no notion of a
    // block index at all beyond its weight base.
    //
    // **conv1's entries carry the folded bias.**  This is not a re-pack of the
    // model's parameters and must never be rebuilt from `b_q` directly.
    (* ram_style = "block" *) reg [63:0] cfgrom [0:(NB * CFG_NB) - 1];
    reg [63:0]       cfg_q;
    reg [CFG_AW-1:0] cfg_k;

    // The address for entry `cfg_k` goes out during cycle `cfg_k` and its data
    // is written on cycle `cfg_k + 1`, which is what the ROM's own read register
    // costs.  The last cycle of the stream presents one address past the block
    // and is clamped: on the fifth block that would be entry 320 of a 320-entry
    // array, and an out-of-range read is a thing that works in simulation and is
    // undefined in synthesis.
    reg  [CFG_AW-1:0] cfg_base_r;

    // ------------------------------------------------------------------
    // the config image self-test
    // ------------------------------------------------------------------
    //
    // Walks all `NB * CFG_NB` entries through the **existing read port** -- a
    // second port would make the mapper duplicate the array -- and CRCs what comes
    // back with the same convention the block CRCs use (poly 0xEDB88320, init all
    // ones, inverted at the end), which is `zlib.crc32` over the image's
    // little-endian words and is what `tools/d64_trunk_cfg_rtl.py` prints.
    //
    // `cfg_q` is the port's registered output, so at address k it holds entry k-1:
    // the walk runs k from 0 to NB*CFG_NB and folds the word on every cycle where
    // k is not zero, which is entries 0..NB*CFG_NB-1 in address order.
    reg  [8:0]  st_k;
    reg         st_done;
    reg  [31:0] st_crc, st_next;
    reg         st_ok;
    wire        st_active = (state == S_IDLE) && cfg_selftest && !st_done;
    wire        st_last   = (st_k == (NB_Q * CFG_NB));
    integer     sb, sbit;

    always @* begin
        st_next = st_crc;
        for (sb = 0; sb < 8; sb = sb + 1) begin
            st_next = st_next ^ {24'd0, cfg_q[sb*8 +: 8]};
            for (sbit = 0; sbit < 8; sbit = sbit + 1)
                st_next = st_next[0] ? (st_next >> 1) ^ 32'hEDB88320
                                     : (st_next >> 1);
        end
    end

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            st_k <= 9'd0; st_done <= 1'b0; st_crc <= 32'hFFFFFFFF; st_ok <= 1'b0;
        end
        else if (st_active) begin
            if (st_last) begin
                st_done <= 1'b1;
                st_ok   <= (~st_next) == 32'h13E3_7B49;
            end
            else begin
                st_k <= st_k + 9'd1;
                if (st_k != 9'd0) st_crc <= st_next;
            end
        end
    end

    assign cfg_image_ok     = st_ok;
    assign cfg_selftest_done = st_done;

    wire [CFG_AW-1:0] cfg_ra = st_active
                             ? st_k[CFG_AW-1:0]
                             : cfg_base_r + ((cfg_k == CFG_N) ? {CFG_AW{1'b0}} : cfg_k);

    always @(posedge clk_pe) begin
        if (c_we) cfgrom[c_wa] <= c_wd;
        cfg_q <= cfgrom[cfg_ra];
    end

    // The engine's own `cfg` is sixty-four entries, so the address it sees is six
    // bits and not the nine the image is addressed with.  Spelled out rather than
    // left to a width-mismatched connection, which is a warning and a silent
    // truncation in one.
    localparam CFG_ENG_AW = 6;
    // Declared, not left to `assign` to create.  Verilog-2001 makes a one-bit
    // implicit net for an undeclared name on first use, so `assign cfg_wa =
    // cfg_wa_r;` silently drives one bit and the engine's six-bit port reads Z
    // in the other five -- and then every accumulator in the design is unknown
    // and the whole five-block tensor comes out `x`.  This repo has a rule about
    // this and this is the third time it has been the answer.
    wire                cfg_we;
    wire [CFG_ENG_AW-1:0] cfg_wa;
    wire [63:0]         cfg_wd;
    // **Combinational, and that is the whole alignment.**  `cfg_q` is the ROM's
    // registered output, so at cycle k it holds the entry whose address went out
    // on cycle k-1 -- that is, entry k-1.  A registered address would be one
    // behind the data it is written with, and every entry would land one slot
    // late: the engine would run with conv1's parameters in conv2's registers and
    // produce numbers that are the right shape and the wrong values, which is the
    // failure this stage exists to find and not the one it wants to have.
    // **The last cycle writes.**  `cfg_k` runs 0..CFG_N, and at cycle k the data
    // in `cfg_q` is entry k-1 -- so k = CFG_N writes entry 63, which is conv3's
    // last output channel.  Excluding it as "the cycle we leave on" leaves
    // `cfg[63]` unknown, and the only symptom is that one channel of one trunk
    // bank is `x`: 4096 wrong words out of 131,072, on a block whose every other
    // value is exact.
    assign cfg_we = (state == S_CFG) && (cfg_k != {CFG_AW{1'b0}});
    assign cfg_wa = cfg_k[CFG_ENG_AW-1:0] - {{(CFG_ENG_AW-1){1'b0}}, 1'b1};
    assign cfg_wd = cfg_q;

    // ------------------------------------------------------------------
    // the block index, the weight base and the buffers
    // ------------------------------------------------------------------

    reg [BLK_W-1:0] blk_r;

    // **The weight base is an accumulator, not a multiply.**  It moves twenty-six
    // tiles once per block and never in between, so `wbase_r <= wbase_r +
    // WSTRIDE` at a boundary is the whole of it.  Written as
    // `WBASE0 + blk_r * WSTRIDE` it is a 3x8 multiply by a *constant* -- which is
    // not a thing a DSP48 should ever be spent on, and efx_map spent one on it.
    // Measured, not argued: replacing the multiply with a plain add in a probe
    // root took the wrapper from 145 DSP to 144, so the block index's multiply
    // into the weight address was the whole of the difference.
    //
    // It is also the better shape.  The multiply sat combinationally in front of
    // the weight store's address, on the path the engine's `w_ra` reads through;
    // an accumulator is one adder off that path and it is updated on the cycle the
    // block index is, so the engine sees the new base at exactly the same time.
    reg [W_ADDR_W-1:0] wbase_r;
    wire [W_ADDR_W-1:0] wbase = wbase_r;

    assign blk     = blk_r;
    assign src_buf = blk_r[0];
    assign dst_buf = ~blk_r[0];
    // The read buffer for the request in flight.  It is the same bit as
    // `src_buf`; keeping the name separate is what makes the port list say what
    // it carries rather than where it comes from.
    assign x_req_buf = blk_r[0];

    // ------------------------------------------------------------------
    // the far side's completion, synchronised
    // ------------------------------------------------------------------

    // `blk_restart` clears the barrier's two halves, so it is declared with them
    // rather than with the engine it feeds -- a forward reference to a reg is
    // not a thing Verilog-2001 allows, and the compiler says so.
    reg         eng_start, blk_restart, blk_arm;

    // ------------------------------------------------------------------
    // D3c6-e7: the window the stage monitor folds over.
    // ------------------------------------------------------------------
    //
    // **Block 0, first pass, no redo** -- the same window the model reference
    // was taken over, and it has to be the same one or the 256 expected chunk
    // CRCs describe a different stream.  It arms on the first cycle the block
    // index is 0 and the engine has been started, and closes when the index
    // leaves 0 or the redo counter becomes non-zero, exactly as the research
    // observer in `sim/investigation_2026_10_04/` did.
    //
    // **Once closed it stays closed.**  A redo inside block 0 re-runs the same
    // writes with a different read behind them, so re-opening would fold a
    // second pass into the same chunk counters and the counts would stop being
    // 8,192 / 8,192 / 16,384 -- which is the reading that says the stream was
    // the one the reference describes.
    // P6i: the harness asking for the block to be redone, and the bound.
    reg  [2:0]  redo_cnt;
    reg         redo_seen;
    reg obs_armed, obs_closed;
    always @(posedge clk_pe or negedge arst_n) begin
        if (!arst_n) begin
            obs_armed  <= 1'b0;
            obs_closed <= 1'b0;
        end
        else if (!obs_closed) begin
            if (blk_r === {BLK_W{1'b0}} && eng_start === 1'b1) obs_armed <= 1'b1;
            if (obs_armed && ((blk_r !== {BLK_W{1'b0}}) || (redo_cnt !== 3'd0)))
                obs_closed <= 1'b1;
        end
    end
    wire obs_en = obs_armed & ~obs_closed;
    // Both read the barrier's own inputs, and both are read only inside the
    // branch that runs once a pass has finished.
    wire        redo_again = blk_redo && (redo_cnt != 3'd7);
    wire        redo_stop  = blk_redo && (redo_cnt == 3'd7);
    reg [1:0]   wd_s;
    reg         wd_seen, wd_flag, last_seen;
    wire        wd_edge = wd_s[1] ^ wd_seen;

    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n) begin
            wd_s <= 2'd0; wd_seen <= 1'b0; wd_flag <= 1'b0; last_seen <= 1'b0;
        end
        else begin
            wd_s <= {wd_s[0], wr_done_tog};
            if (wd_edge) begin wd_seen <= wd_s[1]; wd_flag <= 1'b1; end
            if (out_last) last_seen <= 1'b1;
            if (start || blk_restart) begin
                wd_flag <= 1'b0; last_seen <= 1'b0;
            end
        end

    // ------------------------------------------------------------------
    // the block engine
    // ------------------------------------------------------------------

    wire        eng_busy, eng_done;
    wire [31:0] eng_cycles;
    d64_h0_block #(.RUNTIME_PROFILE(RUNTIME_PROFILE), .CFG_BIAS_W(CFG_BIAS_W), .CFG_STREAM_SRL(CFG_STREAM_SRL),
        .ROWS(ROWS), .COLS(COLS), .BAND(BAND), .A_ROWS(A_ROWS), .B_ROWS(B_ROWS),
        .X_RING_ROWS(X_RING_ROWS), .RF_AW(RF_AW), .WF_AW(WF_AW),
        .C1_CYC(C1_CYC), .C2_CYC(C2_CYC), .C3_CYC(C3_CYC), .C3_IC(C3_IC),
        .W_TILES(W_TILES), .W_WORD_W(W_WORD_W), .W_ADDR_W(W_ADDR_W),
        .Z1(Z1), .ZERO_T(ZERO_T), .WBASE0(WBASE0), .EXTERNAL_PE(EXTERNAL_PE),
        .C2_DIAG(C2_DIAG), .A_ROM_INIT(A_ROM_INIT), .B_ROM_INIT(B_ROM_INIT),
        .CHUNK_ROM_INIT(CHUNK_ROM_INIT)
    ) u_blk (
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .clk_pe(clk_pe), .clk_axi(clk_axi), .arst_n(arst_n),
        .start(eng_start), .restart(blk_restart), .fill_arm(blk_arm),
        .busy(eng_busy), .done(eng_done),
        .cycles(eng_cycles), .wait_cycles(wait_cycles),
        .cfg_we(cfg_we), .cfg_wa(cfg_wa), .cfg_wd(cfg_wd),
        .w_we(w_we), .w_wa(w_wa), .w_wd(w_wd),
        .block_wbase(wbase),
        .eng_add_cnt(trk_add_cnt), .eng_add_diff(trk_add_diff),
        .eng_no_yv(trk_no_yv), .eng_zero_br(trk_zero_br),
        .eng_tw_cnt(trk_tw_cnt),
        .blk_idx(blk_r), .eng_cfg_ok(trk_cfg_ok), .cfg_protocol_fault(cfg_protocol_fault),
        .eng_stream_ok(trk_stream_ok), .eng_refill_c3(trk_refill_c3),
        .eng_refill_seen(trk_refill_seen), .eng_refill_ok(trk_refill_ok),
        .obs_en(obs_en), .eng_obs_chunk(trk_obs_chunk), .eng_c2_chunk(trk_c2_chunk),
        .eng_p7_ok(trk_p7_ok),
        .eng_p7_seen(trk_p7_seen), .eng_p7_acc(trk_p7_acc),
        .eng_p7_fs(trk_p7_fs), .eng_p7_biaseen(trk_p7_biaseen),
        .eng_p7_cfg_seen(trk_p7_cfg_seen),
        .eng_p7_bias(trk_p7_bias), .eng_p7_br(trk_p7_br),
        .eng_p7_cfg0(trk_p7_cfg0), .eng_p7_cfg1(trk_p7_cfg1),
        .eng_p7_ctrl(trk_p7_ctrl),
        .pe_req_slot_valid(pe_req_slot_valid), .pe_req_act(pe_req_act),
        .pe_req_wgt(pe_req_wgt), .pe_req_pix_start(pe_req_pix_start),
        .pe_req_bias(pe_req_bias), .pe_req_rq_start(pe_req_rq_start),
        .pe_req_rq_m(pe_req_rq_m), .pe_req_rq_shift(pe_req_rq_shift),
        .pe_req_rq_zero(pe_req_rq_zero), .pe_req_rq_qmin(pe_req_rq_qmin),
        .pe_req_rq_qmax(pe_req_rq_qmax),
        .pe_rsp_rq_ready(pe_rsp_rq_ready), .pe_rsp_rq_ready_n(pe_rsp_rq_ready_n),
        .pe_rsp_y(pe_rsp_y), .pe_rsp_y_valid(pe_rsp_y_valid),
        .pe_rsp_rq_busy(pe_rsp_rq_busy), .pe_rsp_rq_done(pe_rsp_rq_done),
        .pe_rsp_rq_overrun(pe_rsp_rq_overrun),
        .pe_rsp_mode_viol(pe_rsp_mode_viol),
        .pe_rsp_acc(pe_rsp_acc), .pe_rsp_snapshot(pe_rsp_snapshot),
        .pe_wt_rd_en(pe_wt_rd_en), .pe_wt_rd_addr(pe_wt_rd_addr),
        .pe_wt_rd_data(pe_wt_rd_data),
        .x_req_row(x_req_row), .x_req_tog(x_req_tog), .x_ack_tog(x_ack_tog),
        .rf_wr_en(rf_wr_en), .rf_wr_data(rf_wr_data), .rf_wr_full(rf_wr_full),
        .wf_rd_en(wf_rd_en), .wf_rd_data(wf_rd_data),
        .wf_rd_valid(wf_rd_valid), .wf_rd_empty(wf_rd_empty), .wf_rd_level(wf_rd_level),
        .x_ovf(x_ovf), .x_rbf(x_rbf), .rf_udf(rf_udf), .rf_wait(rf_wait),
        .wf_ovf(wf_ovf), .dep_viol(dep_viol), .x_beats(x_beats),
        .wf_words(wf_words), .rq_overruns(rq_overruns),
        .out_last(out_last), .blk_clean(blk_clean), .clean_why(clean_why),
        .fill_row_o(), .req_row_o()
    );

    // ------------------------------------------------------------------
    // the sequence
    // ------------------------------------------------------------------

    reg [31:0] cyc;

    assign busy   = (state != S_IDLE) && (state != S_DONE);
    assign cycles = cyc;

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            state <= S_IDLE; blk_r <= TRUNK_FIRST_BLOCK;
            eng_start <= 1'b0; blk_restart <= 1'b0; blk_arm <= 1'b0;
            redo_cnt <= 3'd0; redo_seen <= 1'b0;
            trk_redo_bound <= 1'b0; trk_eng_bad <= 1'b0;
            cfg_k <= {CFG_AW{1'b0}}; cfg_base_r <= SEG_CFG_BASE;
            cyc <= 32'd0; cfg_cycles <= 32'd0; bar_cycles <= 32'd0;
            clean_viol <= 32'd0; blk_starts <= 32'd0; compute_sum <= 32'd0;
            clean_first <= 6'd0; cfg_writes <= 32'd0;
            wbase_r <= SEG_WBASE;
            for (wi = 0; wi < NB; wi = wi + 1) wait_at[wi] <= 32'd0;
            blk_restart <= 1'b0; blk_arm <= 1'b0;
        end
        else begin
            eng_start <= 1'b0;
            blk_restart <= 1'b0;
            // The engine's own core count, summed across the five blocks: the
            // number that has to stay 5 x 108,120 however long the boundaries
            // take.
            if (eng_done) compute_sum <= compute_sum + eng_cycles;
            if (eng_done && (eng_cycles < ENG_CYCLES_FLOOR)) trk_eng_bad <= 1'b1;
            if (eng_done) wait_at[blk_r] <= wait_cycles;
            if (state != S_IDLE && state != S_DONE) cyc <= cyc + 32'd1;

            // The config stream, one entry a cycle with the ROM's own read
            // register ahead of it: the address for entry k goes out on cycle k
            // and the data for entry k is written on cycle k+1.
            if (cfg_we) cfg_writes <= cfg_writes + 32'd1;

            case (state)
                // **`S_DONE` is the same idle state, one job later.**  A second
                // frame enters here exactly as the first one entered `S_IDLE`,
                // and nothing else about the sequence moves: the re-arm goes
                // through `S_CFG`, which is where the boundary's `blk_clean`
                // check already lives, so the state frame one left behind is
                // checked before frame two is allowed to use it.
                //
                // **The ring, the FIFOs and the barrier are deliberately not
                // reset.**  Three of them are supposed to have been left in a
                // known state, and resetting them would turn a failure into a
                // silence -- the same argument this module's header makes for a
                // block boundary.  The fourth is stronger than that: `wd_seen`
                // is the two-flop record of the far side's `wr_done_tog`, which
                // is a *level* the DDR writer holds after the last response of a
                // block.  Clearing it would make the level still standing at the
                // end of the previous frame read as a fresh write-done edge a
                // couple of cycles into the next one, and the barrier would
                // release on a write nobody answered.
                //
                // `redo_cnt` is cleared here and not left behind.  It is *not*
                // already zero by construction: `S_BAR` advances to `S_DONE`
                // down the branch that skips `redo_cnt <= 0` whenever the block
                // was redone to the bound, so a frame whose last block rode the
                // redo budget would hand the next frame a `redo_cnt` of 7 -- at
                // which `redo_again` is false and block 0 of the new frame can
                // never be redone, silently.
                S_IDLE, S_DONE: if (start) begin
                    blk_r <= TRUNK_FIRST_BLOCK;
                    wbase_r <= SEG_WBASE;
                    cfg_base_r <= SEG_CFG_BASE;
                    cfg_k <= {CFG_AW{1'b0}};
                    cyc <= 32'd0;
                    cfg_cycles <= 32'd0; bar_cycles <= 32'd0;
                    clean_viol <= 32'd0; blk_starts <= 32'd0;
                    compute_sum <= 32'd0;
                    redo_cnt <= 3'd0;
                    state <= S_CFG;
                end

                // Sixty-five cycles: sixty-four entries and the one that lets
                // the ROM's read register come out.  The boundary's check is on
                // the first of them, before the re-arm.
                S_CFG: begin
                    cfg_cycles <= cfg_cycles + 32'd1;
                    if (cfg_k == {CFG_AW{1'b0}}) begin
                        // The check is *now*, before the re-arm: this is the
                        // state the previous block left behind, and a block that
                        // ended with its ring still holding a row is a failure
                        // this stage exists to report.
                        if (!blk_clean) begin
                            clean_viol <= clean_viol + 32'd1;
                            if (clean_first == 6'd0) clean_first <= clean_why;
                        end
                        blk_starts <= blk_starts + 32'd1;
                        blk_restart <= 1'b1;
                        blk_arm <= 1'b1;
                    end
                    cfg_k <= cfg_k + {{(CFG_AW-1){1'b0}}, 1'b1};
                    if (cfg_k == CFG_N) begin
                        cfg_k <= {CFG_AW{1'b0}};
                        eng_start <= 1'b1;
                        state <= S_RUN;
                    end
                end

                S_RUN: if (eng_done) state <= S_BAR;

                // Both halves of the barrier, in either order: the last word
                // reaching the FIFO on this side, and the far side saying it has
                // written everything.
                S_BAR: begin
                    bar_cycles <= bar_cycles + 32'd1;
                    if (wd_flag && last_seen) begin
                        // **No settle here, and that is a measurement, not an
                        // omission.**  See the note at the top of this module: 64
                        // cycles of it at every boundary left the chain intact in
                        // seven of ten runs against eight without it.  `wd_flag` is
                        // the far side saying the last write response has come back,
                        // which is the right event to leave on -- what is in doubt
                        // is whether the next block's reads see that write, and if
                        // they do not, P6d's address masks and its W-side CRC say
                        // which of the two it is.
                        // **P6i: a block whose input was stale is redone, not
                        // advanced past.**  `blk_redo` comes from the harness, which
                        // compares the CRC of this block's *read* stream against the
                        // CRC of the previous block's *write* stream -- the two
                        // numbers every board round in this stage has compared by
                        // hand.  Both are complete here: `in_blk` finished when this
                        // block's reads did, and `res_{blk-1}` at the last barrier.
                        //
                        // Not advancing `blk_r` is the whole mechanism: the next
                        // `S_CFG` pulses `blk_restart`/`blk_arm`/`eng_start` for the
                        // same index, which re-arms the engine and re-issues the
                        // block's row requests.  The reread is later than the first
                        // one, so the writes it is waiting on have had that much
                        // longer to land.
                        //
                        // Bounded, because "the CRCs never agree" is a possibility and
                        // a design that retries forever is a design that hangs.  At
                        // the bound the block advances anyway and the violation is
                        // counted, so a stuck redo appears as a number rather than as
                        // a board that stopped.
                        //
                        // **Two questions, and they were one expression.**  This
                        // pass has settled: `redo_again` asks whether to run it
                        // again, `redo_stop` whether the mismatch has now survived
                        // the whole budget.  Both are decided *here*, where a pass
                        // has actually finished -- a level condition anywhere else
                        // alarms a pass early, because `redo_cnt` reaches 7 on the
                        // way into the eighth pass while `blk_redo` still holds the
                        // seventh pass's answer.
                        if (redo_stop) trk_redo_bound <= 1'b1;
                        if (redo_again) begin
                            redo_cnt <= redo_cnt + 3'd1;
                            redo_seen <= 1'b1;
                            state <= S_CFG;
                        end
                        else if (blk_r + {{(BLK_W-1){1'b0}}, 1'b1} >= SEG_END) begin
                            state <= S_DONE;
                        end
                        else begin
                            blk_r <= blk_r + {{(BLK_W-1){1'b0}}, 1'b1};
                            wbase_r <= wbase_r + WSTRIDE[W_ADDR_W-1:0];
                            cfg_base_r <= cfg_base_r + CFG_N;
                            cfg_k <= {CFG_AW{1'b0}};
                            redo_cnt <= 3'd0;   // the bound is per block
                            state <= S_CFG;
                        end
                    end
                end

                S_DONE: ;
            endcase
        end
    end

    // P6i: the redo's counters reach the harness, and `board_gate_pass` reads
    // them there -- which is what keeps them out of the mapper's bin.
    assign trk_redo_cnt  = redo_cnt;
    assign trk_redo_seen = redo_seen;
    assign trk_redo_full = (redo_cnt == 3'd7);
    // **`trk_redo_bound` is set in the FSM's own block now**, beside the decision
    // it reports.  It used to be set from a second always block on the level
    // `blk_redo && redo_cnt == 7`.  Two blocks writing one register is a synthesis
    // error waiting for a mapper that feels like reporting it, and the level form
    // fired a pass early for the reason written at the barrier.

    assign done = (state == S_DONE);

endmodule
