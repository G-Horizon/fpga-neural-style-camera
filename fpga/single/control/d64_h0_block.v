`timescale 1ns/1ps

// D3c5-b -- one residual block with H0's memory boundary around it.
//
// D3c5-a proved the block's arithmetic and its schedule *together*, and it did it
// with the whole 64-row tensor sitting inside the engine: `d64_tc_ram #(.DW(80),
// .AW(14))`, 16,384 words, 160 KiB.  That is not a memory this part has, and it
// is not the shape D3c1 froze.  This module is the same block with the boundary
// the design actually has:
//
//     DDR3  U16 rows, 2 x 2 KiB bursts a row
//       |                                                        (clk_axi)
//       v
//     RF32   d64_async_fifo, 32 x 80 bit, 4 RAM10
//       |                                                        (clk_pe)
//       v
//     unpack   8 x U16 -> 8 x 10 bit; one DDR beat is one ring bank word
//              (the 6 bits between lanes were always zero, so the FIFO now
//               carries the 8 x 10 payload and the U16 container is rebuilt
//               combinationally on the way out)
//       |
//       v
//     X16 ring  16 rows x 64 cols x 4 banks x 80 bit, 32 RAM10
//       |
//       v
//     d64_conv_engine, X_RING_ROWS = 16   -- arithmetic and schedule untouched
//       |
//       v
//     residual add -> pack 8 x 10 bit -> 8 x U16, one trunk word is one beat
//       |
//       v
//     WF16   d64_async_fifo, 2048 x 80 bit = 16 bursts, 16 RAM10
//       |
//       v
//     DDR3
//
// Nothing in the middle changed.  `d64_conv_engine`'s pipeline, its two-cycle
// write-back, its skip capture on `trunk_rd_d`, and `d64_tconv_mac` itself are the
// modules D3c5-a closed; `tools/run_conv_engine.sh` still passes against them in
// flat mode.  What changed is where the X words come from.
//
// One beat is one bank word, on both sides
// ----------------------------------------
//
// The DDR row is 64 columns x 32 channels x 16 bits = 4096 bytes = 256 beats.
// Ordering the beats `col*4 + bank` makes one 128-bit beat exactly the 80-bit bank
// word the engine already reads -- eight channels of ten bits in a U16 container.
// The unpacker is eight slices and the packer is eight slices, no shifter on
// either side and nothing crossing a beat boundary.  The alternative, packed
// 10-bit on the wire, was costed in D3c1 (160 KiB a tensor instead of 256) and
// rejected: it needs a bit-packer spanning the beat on both sides and buys
// bandwidth this design already has.
//
// The wait, and why it is only at a band boundary
// ----------------------------------------------
//
// 16 rows hold a 64-row tensor only if rows are dropped as fast as they arrive,
// and they are dropped by the residual add in conv3 -- a whole conv2 after conv1
// wanted them.  So the live set is the band plus its lookahead plus as much of the
// next band as the released slots allow, and at a band boundary conv1 can arrive
// before its first row does.
//
// The wait is put there and nowhere else.  It could not go mid-phase: the engine
// has no clock enable, `slot_valid` at the MAC would stay high through a freeze
// and the accumulator would take the same product twice.  The band-boundary
// switch is the one place in the block where nothing is live, nothing is still
// owed and nothing is in flight, so `hold` extends that switch and touches no
// other cycle.  `d64_trunk_scheduler` stops counting `cycles` while held, so the
// compute core stays the 108,120 cycles D3c3 froze and the wait is counted here,
// separately, because it is the DDR's number and not the block's.
//
// Two crossings, both by handshake
// --------------------------------
//
// The FIFOs cross on their own (Gray pointers, `async_reg`, domain resets).  The
// row *request* crosses too, and it is not safe to hand a PE-domain level to the
// AXI side and hope: `board/ddr3/ddr3_h0_replay.v` runs the same handshake on the
// board.  So a request is a bundled-data transfer -- the row number is registered
// and held, then a toggle is synchronised across, and the far side toggles an ack
// back on its own clock.  Nothing samples the row number except in the domain
// that owns it, which is the only formulation in which a multi-bit crossing is
// safe without a FIFO.
//
// What is counted, and what is claimed
// ------------------------------------
//
//   x_ovf      a fill landed on a ring slot that was still live
//   x_rbf      the schedule read a slot that did not hold the row it asked for
//   rf_udf     a beat was consumed from an empty read FIFO
//   wf_ovf     a trunk word arrived with the write FIFO full
//   dep_viol   the beat stream and the request stream disagreed -- a beat with
//              nobody filling, or a fill that ran past its 256th beat
//
// Four of the five are zero by construction, and they exist because "by
// construction" is a claim.  `wf_ovf` is the one that is not structurally
// impossible: conv3 produces a row in 256 cycles and the DDR takes 157 AXI cycles
// to drain each of its two bursts, so the write side is faster than the memory by
// about 1.5x for as long as conv3 runs.  The FIFO absorbs the band's backlog and
// conv2's idle cycles drain it; D3c1's model puts the peak at 11 bursts against
// this FIFO's 16.  A word arriving on a full FIFO is **counted and dropped**
// rather than silently overwriting: there is no back-pressure to conv3 without
// the clock enable this design does not have, so the honest thing is to make that
// failure loud and then show it does not happen.
module d64_h0_block #(
    parameter ROWS        = 64,
    parameter COLS        = 64,
    parameter BAND        = 8,
    parameter A_ROWS      = 11,
    parameter B_ROWS      = 8,
    parameter X_RING_ROWS = 16,

    // H0's own staging numbers.  RF 32 x 128 was 7 RAM10 and WF 2048 x 128 was
    // 26; both now store the 8 x 10 payload only and are 4 and 16 (isolated MAP,
    // `fast_style_fpga/sim/investigation_2026_10_08/ram_1e8c5db/FIFO_AB_MANIFEST.json`),
    // at unchanged depth.
    parameter RF_AW       = 5,
    parameter WF_AW       = 11,
    parameter integer OUTPUT_CREDIT_ENABLE = 1,

    parameter C1_CYC      = 4,
    parameter C2_CYC      = 18,
    parameter C3_CYC      = 4,
    parameter C3_IC       = 8,

    parameter W_TILES     = 247,
    parameter W_WORD_W    = 1024,
    parameter W_ADDR_W    = 8,
    parameter CFG_N       = 64,
    parameter CFG_AW      = 6,
    parameter CFG_BIAS_W  = 32,
    parameter integer CFG_STREAM_SRL=0,

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
    parameter WBASE0      = 54,
    // D3c6-e.  When set, this block owns neither its multiplier bank nor its
    // weight store: both come in over the seam below and are shared with every
    // other scheduler in the network.  At 0 the block is exactly the one
    // D3c5-b/D3c5-c closed, and the seam is inert -- `d64_conv_engine` reads
    // none of it when its own `EXTERNAL_PE` is 0.
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0
) (
    output wire [7:0] fill_debug,
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire                clk_pe,      // 120 MHz: ring, engine, FIFO near side
    input  wire                clk_axi,     // 100 MHz: FIFO far side
    input  wire                arst_n,

    input  wire                start,
    // D3c5-c.  `restart` re-arms the fill sequencer and `fill_arm` says the
    // prefetcher may run at all.  They are separate from `start`, which is the
    // engine's, because a single-block caller wants the prefetch running from
    // reset -- it is what the first band's wait absorbs -- and a five-block
    // caller must not have it running before the block it belongs to has begun.
    //
    // Arm the prefetcher from reset while a restart is also possible and the two
    // deadlock: rows 0..15 are already resident, `restart` puts `req_row` back to
    // 0, slot 0 is still live, and the prefetcher waits for a release that only
    // conv3 can give -- and conv3 never starts.  `tb_d64_h0_block` ties
    // `fill_arm` high and `restart` low and is unchanged to the cycle.
    input  wire                restart,
    input  wire                fill_arm,
    output wire                busy,
    output wire                done,
    output wire [31:0]         cycles,      // the compute core, held cycles excluded
    output wire [31:0]         wait_cycles, // the band-boundary wait: the DDR's time

    input  wire                cfg_we,
    input  wire [CFG_AW-1:0]   cfg_wa,
    input  wire [63:0]         cfg_wd,
    input  wire                w_we,
    input  wire [W_ADDR_W-1:0] w_wa,
    input  wire [W_WORD_W-1:0] w_wd,
    // D3c5-c: which block's weights to read.  The store holds the whole image and
    // is loaded once; the base moves twenty-six tiles a block.
    input  wire [W_ADDR_W-1:0] block_wbase,
    // D3c6-e6b: which residual block this run is, for the engine's own cfg
    // probe.  Straight through; nothing here decides anything.
    input  wire [2:0]          blk_idx,

    // ---- D3c6-e: the shared-PE seam, `EXTERNAL_PE` only -------------------
    // Straight through to the engine, with no logic and no mux.  Nothing here
    // decides anything: the operand mapping, the requant handshake and the
    // weight-read address are the engine's, and this module's only part in the
    // seam is to not be in the way.
    //
    // In the default configuration `pe_rsp_*` and `pe_wt_rd_data` are read by
    // nobody, so a caller that leaves them open is not making a mistake -- but
    // they are declared unconditionally, because a port list that changes shape
    // with a parameter is a port list two builds can disagree about, and this
    // project has paid for that already.
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
    // A request is a row number held stable plus a toggle; the far side answers
    // with a toggle of its own and then delivers exactly 256 beats into the read
    // FIFO.  Deliberately not AXI yet: what is being measured here is whether the
    // block's own memory geometry holds, and a burst master would put the
    // controller's arbitration inside the measurement.
    output wire [6:0]          x_req_row,
    output wire [31:0]  eng_add_cnt,
    output wire [31:0]  eng_tw_cnt,
    output wire [31:0]  eng_add_diff,
    output wire [31:0]  eng_no_yv,
    output wire [31:0]  eng_zero_br,
    // Five sticky bits: lane 0 of the conv3 requant word this block's image
    // says it should be, one bit a block.
    output wire [4:0]   eng_cfg_ok,
    output wire cfg_protocol_fault,
    output wire [4:0]   eng_stream_ok,
    output wire         eng_refill_ok,
    output wire [3:0]   eng_p7_ok,
    output wire [1:0]   eng_p7_seen,
    output wire [63:0]  eng_p7_acc, eng_p7_fs,
    output wire         eng_p7_biaseen,
    output wire [1:0]   eng_p7_cfg_seen,
    output wire [63:0]  eng_p7_bias, eng_p7_br,
    output wire [31:0]  eng_p7_cfg0, eng_p7_cfg1,
    output wire [11:0]  eng_p7_ctrl,
    output wire [20:0]  eng_refill_c3,
    output wire         eng_refill_seen,
    // D3c6-e7: the stage monitor's whole result, straight from the engine.
    input  wire         obs_en,
    output wire [272:0] eng_obs_chunk,
    output wire [287:0] eng_c2_chunk,
    output wire [2:0]    eng_c2_dbg,
    output wire                x_req_tog,
    input  wire                x_ack_tog,

    input  wire                rf_wr_en,
    input  wire [127:0]        rf_wr_data,
    output wire                rf_wr_full,

    input  wire                wf_rd_en,
    output wire [127:0]        wf_rd_data,
    output wire                wf_rd_valid,
    output wire                wf_rd_empty,
    // Conservative occupancy, already in clk_axi (FIFO read domain).
    output wire [WF_AW:0]      wf_rd_level,

    // ---- evidence ---------------------------------------------------------
    output wire [31:0]         x_ovf,
    output wire [31:0]         x_rbf,
    output reg  [31:0]         rf_udf,
    output reg  [31:0]         rf_wait,
    output reg  [31:0]         wf_ovf,
    output reg  [31:0]         dep_viol,
    output reg  [31:0]         x_beats,     // bank words written into the ring
    output reg  [31:0]         wf_words,    // trunk words handed to the write FIFO
    output wire [31:0]         rq_overruns,
    output wire [6:0]          fill_row_o,
    output wire [6:0]          req_row_o,

    // ---- D3c5-c: what a block boundary needs ------------------------------
    // `out_last` is the block's final trunk word going into the write FIFO.  It
    // is the engine's *registered* write transaction, so it is the last moment
    // the data exists -- a barrier built on the scheduler's `done` would start
    // the next block with this word still in the pipeline, which is the defect
    // D3c5-b1's testbench found by counting 16,383 writes instead of 16,384.
    output wire                out_last,
    // `blk_clean` is the state a block must start from, and it is **checked, not
    // created**: the controller samples it on the cycle before it re-arms and
    // counts a violation if it is false.  Resetting into a clean state instead
    // would hide exactly the failures this stage exists to find.
    output wire                blk_clean,
    // Which term of `blk_clean` is false, so a dirty boundary says *what* it
    // was rather than only that it happened.  {busy, filling, rf_not_empty,
    // wf_not_empty, ring_live, row_midflight}
    output wire [5:0]          clean_why
);

    localparam ROWS_Q  = ROWS;
    localparam SW_W    = 4;                        // covers 0..BAND with BAND = 8
    localparam [6:0] BAND_Q = BAND[6:0];

    // ---- the wiring between the sequencer, the scan and the block --------
    // Declared here because Verilog-2001 creates a one-bit implicit net on first
    // use, and a one-bit net carrying an 80-bit word or a 32-bit counter is a
    // failure this repo has already paid for twice.
    wire [31:0]  e_cycles;
    wire [31:0]  e_ovf, e_rbf, e_overruns;
    wire [3:0]   cur_band;
    wire         cur_bandsw;
    wire         hold;
    wire [5:0]   x_q_row, x_f_row;
    wire         x_q_ready, x_q_free;
    wire [5:0]   x_rel_row;
    wire         x_rel_en;
    wire [11:0]  t_wr_addr;
    wire         t_wr_en;
    wire         t_wr_last;
    wire [1:0]   t_wr_bank;
    wire         ring_live;
    wire [79:0]  tw_word;
    // The FIFOs store eight 10-bit lanes and nothing else.  `genvar wi` is
    // declared here because both the read FIFO's pack and the write FIFO's
    // unpack are generate-for loops and Verilog-2001 wants the genvar in scope
    // before the block that uses it.
    genvar wi;
    wire         wf_wr_en, wf_full;
    wire [WF_AW:0] wf_wr_level;
    reg  [31:0]  wait_q;
    reg  [SW_W-1:0] scan_k;
    reg  [3:0]      scan_band;
    reg  [BAND:0]   ready_mask;

    wire pe_rst_n, axi_rst_n;
    d64_reset_sync u_pe_rst  (.clk(clk_pe),  .arst_n(arst_n), .rst_n(pe_rst_n));
    d64_reset_sync u_axi_rst (.clk(clk_axi), .arst_n(arst_n), .rst_n(axi_rst_n));

    // ------------------------------------------------------------------
    // the read FIFO: DDR -> ring
    // ------------------------------------------------------------------

    wire [79:0]  rf_wr_data80;
    wire [79:0]  rf_rd_data80;
    wire         rf_rd_valid, rf_rd_empty;
    wire         rf_rd_ask, rf_rd_en;

    // 128 -> 80 on the way in.  `fill_word` below reads only bits [i*16 +: 10]
    // of each lane, so the 6 bits this drops are bits nothing on the PE side
    // ever reads; the trunk writer packs them as zero and the DDR ABI does not
    // change.  This is a narrowing, not a re-encoding.
    generate
        for (wi = 0; wi < 8; wi = wi + 1) begin : g_rf_pack
            assign rf_wr_data80[wi*10 +: 10] = rf_wr_data[wi*16 +: 10];
        end
    endgenerate

    d64_async_fifo #(.DW(80), .AW(RF_AW)) u_rf (
        .wr_clk(clk_axi), .wr_rst_n(axi_rst_n),
        .wr_en(rf_wr_en), .wr_data(rf_wr_data80), .wr_full(rf_wr_full),
        .rd_clk(clk_pe), .rd_rst_n(pe_rst_n),
        .rd_en(rf_rd_en), .rd_data(rf_rd_data80), .rd_valid(rf_rd_valid),
        .rd_empty(rf_rd_empty)
    );

    // ------------------------------------------------------------------
    // the fill sequencer: one row at a time, 256 beats
    // ------------------------------------------------------------------
    //
    // A request goes out as soon as the target slot is free, which is the whole
    // prefetch policy: rows are wanted in order and released in order, so "the
    // next row" is the only row that can be asked for and "is its slot free" is
    // the only question.  No priority, no reordering, no depth search -- and the
    // reason there does not have to be is that H0 is a streaming schedule.

    reg  [6:0]  req_row;        // the next row to ask for
    reg  [6:0]  req_row_r;      // the row the outstanding request names
    reg         req_busy;
    reg         req_tog;
    reg  [1:0]  ack_s;
    reg         ack_seen;

    reg         fill_act;
    reg  [5:0]  fill_row;
    reg  [8:0]  fill_beat;      // 9 bits, so running past 255 is visible

    // `req_busy` covers the whole span from issuing the request to finishing the
    // row, not just the handshake.  It is what stops the next request being
    // issued while the current row is still arriving -- there is one row in
    // flight at a time, and `req_row` is incremented by the row's *last* beat,
    // so a request issued a cycle early would name the row that just finished.
    wire req_go = fill_arm && !req_busy && (req_row < ROWS_Q) && x_q_free;
    wire ack_edge = ack_s[1] ^ ack_seen;

    assign x_req_row = req_row_r;
    assign x_req_tog = req_tog;

    // The FIFO is read only when a beat is wanted and one exists.
    assign rf_rd_ask = fill_act;
    assign rf_rd_en  = rf_rd_ask && !rf_rd_empty;
    // `rf_udf` is the FIFO's own contract checked, not assumed: a read the FIFO
    // accepted has to produce `rd_valid` on the next cycle.  Anything else is a
    // beat that was consumed out of nothing, and it is the one underflow this
    // design can actually have -- the other candidate, reading while `rd_empty`,
    // is already excluded by the guard two lines up and would only prove the
    // guard is written down.  `rf_wait` is the different and *expected* number:
    // cycles the filler spent waiting for the DDR, which is not a fault.
    reg rd_en_d;

    wire fill_beat_valid = fill_act && rf_rd_valid;

    wire [5:0]  xr_row  = fill_row;
    wire [5:0]  xr_col  = fill_beat[7:2];
    wire [1:0]  xr_bank = fill_beat[1:0];
    wire        xr_en   = fill_beat_valid;
    wire        xr_last = fill_beat_valid && (fill_beat == 9'd255);

    // One DDR beat, eight U16 containers, eight 10-bit channels -- no shifter and
    // no crossing, because the beat order is the ring word order.
    // `begin`/`end` around each generate-for body.  Verilog-2001 requires it and
    // ModelSim accepts its absence, which is exactly the combination that makes it
    // dangerous: efx_map warns "begin/end is required for generate-for in this
    // mode of Verilog" and then elaborates something that drops the assignment
    // entirely.  With `fill_word` undriven and `wf_wr_data` undriven behind it,
    // the ring, the write FIFO, the weight store and all 144 DSP came back as
    // zero -- a map that reported success and a design that was not there.
    // 80 in, 80 out: the FIFO now holds exactly the payload, so the unpack that
    // used to gather it out of the U16 container is an identity, combinational,
    // on the same cycle `rf_rd_valid` says the beat is there.
    wire [79:0] fill_word = rf_rd_data80;

    // ------------------------------------------------------------------
    // the block
    // ------------------------------------------------------------------

    d64_conv_engine #(.RUNTIME_PROFILE(RUNTIME_PROFILE), .CFG_BIAS_W(CFG_BIAS_W), .CFG_STREAM_SRL(CFG_STREAM_SRL),
        .ROWS(ROWS), .COLS(COLS), .BAND(BAND), .A_ROWS(A_ROWS),
        .B_ROWS(B_ROWS), .C1_CYC(C1_CYC), .C2_CYC(C2_CYC), .C3_CYC(C3_CYC),
        .C3_IC(C3_IC), .X_RING_ROWS(X_RING_ROWS),
        .W_TILES(W_TILES), .W_WORD_W(W_WORD_W), .W_ADDR_W(W_ADDR_W),
        .CFG_N(CFG_N), .CFG_AW(CFG_AW), .Z1(Z1), .ZERO_T(ZERO_T),
        .C2_DIAG(C2_DIAG), .A_ROM_INIT(A_ROM_INIT), .B_ROM_INIT(B_ROM_INIT),
        .CHUNK_ROM_INIT(CHUNK_ROM_INIT),
        .WBASE0(WBASE0), .EXTERNAL_PE(EXTERNAL_PE)
    ) u_eng (
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .clk(clk_pe), .rst_n(pe_rst_n), .start(start), .hold(hold),
        .cur_band(cur_band), .cur_bandsw(cur_bandsw),
        .busy(busy), .done(done), .cycles(e_cycles),
        .cfg_we(cfg_we), .cfg_wa(cfg_wa), .cfg_wd(cfg_wd), .cfg_protocol_fault(cfg_protocol_fault),
        // The flat X port is tied off in ring mode, so it cannot be mistaken for
        // a second write path into the tensor.
        .x_we(1'b0), .x_wa({14{1'b0}}), .x_wd({80{1'b0}}),
        .w_we(w_we), .w_wa(w_wa), .w_wd(w_wd), .block_wbase(block_wbase),
        .blk_idx(blk_idx), .cfg_local_ok_o(eng_cfg_ok),
        .stream_ok_o(eng_stream_ok), .refill_c3_o(eng_refill_c3),
        .refill_seen_o(eng_refill_seen), .refill_ok_o(eng_refill_ok),
        .p7_ok_o(eng_p7_ok), .p7_seen_o(eng_p7_seen),
        .p7_acc_o(eng_p7_acc), .p7_fs_o(eng_p7_fs),
        .p7_bias_seen_o(eng_p7_biaseen), .p7_bias_o(eng_p7_bias),
        .p7_cfg_seen_o(eng_p7_cfg_seen),
        .p7_br_o(eng_p7_br), .p7_cfg0_o(eng_p7_cfg0), .p7_cfg1_o(eng_p7_cfg1),
        .p7_ctrl_o(eng_p7_ctrl),
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
        .xr_en(xr_en), .xr_row(xr_row), .xr_col(xr_col), .xr_bank(xr_bank),
        .xr_word(fill_word), .xr_last(xr_last),
        .x_q_row(x_q_row), .x_q_ready(x_q_ready),
        .x_f_row(x_f_row), .x_q_free(x_q_free),
        .x_rel_row(x_rel_row), .x_rel_en(x_rel_en),
        .x_ovf(e_ovf), .x_rbf(e_rbf), .x_live(ring_live),
        .t_wr_en_o(t_wr_en), .t_wr_addr_o(t_wr_addr), .t_wr_last_o(t_wr_last),
        .t_wr_bank_o(t_wr_bank),
        .obs_en(obs_en), .obs_chunk(eng_obs_chunk), .c2_chunk(eng_c2_chunk),
        .a_writes(), .b_writes(), .t_writes(),
        .rq_accepts(), .rq_overruns(e_overruns),
        .a_wr_data_o(), .b_wr_data_o(), .t_wr_data_o(tw_word),
        .add_cnt_o(eng_add_cnt), .add_diff_o(eng_add_diff),
        .no_yv_o(eng_no_yv), .zero_br_o(eng_zero_br),
        .tw_cnt_o(eng_tw_cnt)
    );

    assign cycles = e_cycles;

    assign x_ovf       = e_ovf;
    assign x_rbf       = e_rbf;
    assign rq_overruns = e_overruns;

    // ------------------------------------------------------------------
    // the write FIFO: ring -> DDR
    // ------------------------------------------------------------------

    // `tw_word` is already the 8 x 10 payload, so the write side needs no
    // packing at all; the 128-bit container is rebuilt on the AXIS side,
    // combinationally, so `wf_rd_valid` and the cycle the word is presented on
    // are unchanged and the DDR ABI -- one burst of 128 bits -- is unchanged.
    wire [79:0] wf_rd_data80;

    d64_async_fifo #(.DW(80), .AW(WF_AW)) u_wf (
        .wr_clk(clk_pe), .wr_rst_n(pe_rst_n),
        .wr_en(wf_wr_en), .wr_data(tw_word), .wr_full(wf_full), .wr_level(wf_wr_level),
        .rd_clk(clk_axi), .rd_rst_n(axi_rst_n),
        .rd_en(wf_rd_en), .rd_data(wf_rd_data80), .rd_valid(wf_rd_valid),
        .rd_empty(wf_rd_empty), .rd_level(wf_rd_level)
    );

    generate
        for (wi = 0; wi < 8; wi = wi + 1) begin : g_wf_unpack
            assign wf_rd_data[wi*16 +: 16] = {6'b0, wf_rd_data80[wi*10 +: 10]};
        end
    endgenerate

    assign wf_wr_en = t_wr_en && !wf_full;

    // ------------------------------------------------------------------
    // the band scan, and the wait it produces
    // ------------------------------------------------------------------
    //
    // "Is the next band here" is nine questions, and asking them combinationally
    // is nine comparators on the switch's exit path.  Asking one a cycle is nine
    // cycles of a switch that is at least sixty-four long, so the cheap version is
    // free -- and the scan restarts whenever the band changes, so the mask can
    // never answer for a band that is no longer current.
    //
    // Bits 0..BAND-1 are the current band and bit BAND is the lookahead; D3c5-c1
    // split the two apart in `hold` and left the scan itself alone.

    wire [6:0] scan_row = scan_band * BAND_Q + {{(7-SW_W){1'b0}}, scan_k};
    wire       scan_hit = (scan_row < ROWS_Q);

    // D3c5-c1.  **Eight rows are a dependency; the ninth is lookahead.**
    //
    // The scan asks nine questions -- the band's own eight rows and the first row
    // of the next -- because conv1's 3x3 window does reach one row past the last
    // row of the band.  But it reaches it *last*: row `b*8 + 8` is wanted for the
    // final row of the band's conv1, an entire band of work after the switch.  So
    // holding the switch on nine rows makes the boundary wait for a row the band
    // does not need yet.
    //
    // What is being waited for is the prefetcher, and the prefetcher is **not**
    // held: `req_go`, the fill sequencer and the ring writes have their own always
    // block and keep running through the hold, filling into whatever slots conv3
    // of the previous band released.  Row `b*8 + 8` lives in exactly the slot row
    // `b*8 - 8` occupied, and that row was released before the switch -- so the
    // lookahead row is fetchable from the first cycle of the hold, and waiting for
    // it is waiting for a fetch that the band's own work would have covered.
    //
    // The mask still asks all nine and is still observable; only `hold` narrows.
    // If the claim is wrong the failure is `x_rbf` -- the schedule reading a slot
    // that does not hold the row it asked for -- which is a counter that fires on
    // the cycle of the bad read, and not a tensor that happens to be wrong.
    // `tb_d64_trunk_blocks` records when bit BAND arrives against when the hold
    // ended, so the margin is a number and not an argument.
    // Reserve the maximum output band before entering C1. The scheduler only
    // samples hold in M_SW->C1; its previous C3 has drained. The actual final
    // write transaction can still be high on the first M_SW cycle, so require
    // !t_wr_en as well as capacity. A transient empty before that write is not
    // a reservation. No MAC/RQ/arithmetic phase is stalled mid-flight.
    localparam integer OUTPUT_FIFO_WORDS = (1 << WF_AW);
    localparam integer OUTPUT_BAND_ROWS = (ROWS < BAND) ? ROWS : BAND;
    localparam integer OUTPUT_BAND_WORDS = OUTPUT_BAND_ROWS * COLS * 4;
    wire output_credit_ok = (OUTPUT_FIFO_WORDS >= OUTPUT_BAND_WORDS) &&
        (wf_wr_level <= (OUTPUT_FIFO_WORDS - OUTPUT_BAND_WORDS)) && !t_wr_en;
    assign hold = cur_bandsw && (!(&ready_mask[BAND-1:0]) ||
        ((OUTPUT_CREDIT_ENABLE != 0) && !output_credit_ok));

    assign x_q_row = scan_row[5:0];
    assign x_f_row = req_row[5:0];
    assign wait_cycles = wait_q;

    // ------------------------------------------------------------------
    // the registers
    // ------------------------------------------------------------------

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            req_row    <= 7'd0;
            req_row_r  <= 7'd0;
            req_busy   <= 1'b0;
            req_tog    <= 1'b0;
            ack_s      <= 2'd0;
            ack_seen   <= 1'b0;
            fill_act   <= 1'b0;
            fill_row   <= 6'd0;
            fill_beat  <= 9'd0;
            scan_k     <= {SW_W{1'b0}};
            scan_band  <= 4'd0;
            ready_mask <= {(BAND+1){1'b0}};
            wait_q     <= 32'd0;
            rf_udf     <= 32'd0;
            rd_en_d    <= 1'b0;
            rf_wait    <= 32'd0;
            wf_ovf     <= 32'd0;
            dep_viol   <= 32'd0;
            x_beats    <= 32'd0;
            wf_words   <= 32'd0;
        end
        else if (restart) begin
            // D3c5-c.  A block boundary re-arms the *sequencing* -- which row is
            // wanted, how far into it the fill is, and the band scan -- and
            // nothing else.  The FIFOs are not reset, the ring is not reset, and
            // the evidence counters are totals across the whole run, because all
            // of those are things a block boundary is supposed to have left in a
            // known state rather than things a reset should impose on it.
            req_row    <= 7'd0;
            req_busy   <= 1'b0;
            fill_act   <= 1'b0;
            fill_beat  <= 9'd0;
            scan_k     <= {SW_W{1'b0}};
            scan_band  <= 4'd0;
            ready_mask <= {(BAND+1){1'b0}};
        end
        else begin
            // ---- the wait ------------------------------------------------
            if (hold) wait_q <= wait_q + 32'd1;

            // ---- the band scan -------------------------------------------
            if (scan_band != cur_band) begin
                scan_band  <= cur_band;
                scan_k     <= {SW_W{1'b0}};
                ready_mask <= {(BAND+1){1'b0}};
            end
            else begin
                ready_mask[scan_k] <= scan_hit ? x_q_ready : 1'b1;
                scan_k <= (scan_k == BAND_Q[SW_W-1:0]) ? {SW_W{1'b0}}
                                                       : (scan_k + 1'b1);
            end

            // ---- the request handshake -----------------------------------
            ack_s <= {ack_s[0], x_ack_tog};
            if (ack_edge) begin
                ack_seen  <= ack_s[1];
                fill_act  <= 1'b1;
                fill_row  <= req_row_r[5:0];
                fill_beat <= 9'd0;
            end
            if (req_go) begin
                req_row_r <= req_row;
                req_tog   <= ~req_tog;
                req_busy  <= 1'b1;
            end

            // ---- the fill ------------------------------------------------
            if (fill_beat_valid) begin
                fill_beat <= fill_beat + 9'd1;
                x_beats   <= x_beats + 32'd1;
                if (fill_beat == 9'd255) begin
                    fill_act <= 1'b0;
                    req_busy <= 1'b0;
                    req_row  <= req_row + 7'd1;
                end
            end

            // ---- the counters --------------------------------------------
            rd_en_d <= rf_rd_en;
            if (rd_en_d && !rf_rd_valid) rf_udf <= rf_udf + 32'd1;
            if (fill_act && rf_rd_empty && !rf_rd_valid)
                rf_wait <= rf_wait + 32'd1;
            if (t_wr_en && wf_full) wf_ovf <= wf_ovf + 32'd1;
            if (wf_wr_en) wf_words <= wf_words + 32'd1;
            if ((rf_rd_valid && !fill_act) || (fill_act && (fill_beat > 9'd255)))
                dep_viol <= dep_viol + 32'd1;
        end
    end

    assign fill_row_o = {1'b0, fill_row};
    assign req_row_o  = req_row;

    // Everything a block boundary has to have settled: no row request in
    // flight, nothing being filled, the read FIFO empty, every ring slot handed
    // back, and the write FIFO drained on its far side.
    assign out_last  = t_wr_last && !wf_full;
    assign blk_clean = !req_busy && !fill_act && rf_rd_empty && wf_rd_empty
                    && !ring_live && (req_row == 7'd0 || req_row == ROWS_Q);
    assign clean_why = {req_busy, fill_act, ~rf_rd_empty, ~wf_rd_empty,
                        ring_live, (req_row != 7'd0) && (req_row != ROWS_Q)};

    // FSB5 passive APP fill phase; ack_s[1] is the existing second stage.
    assign fill_debug = {hold,fill_arm,rf_rd_valid,rf_rd_empty,ack_edge,req_go,req_busy,fill_act};
endmodule
