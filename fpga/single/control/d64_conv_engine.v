`timescale 1ns/1ps

// D3c5-a -- the residual block: `d64_trunk_scheduler` driving the shared backend.
//
// The schedule is D3c3's and is closed; the arithmetic is D3c4c's and is closed.
// Neither was proved *together*, and this module is where they meet.  It contains
// no new arithmetic and no new schedule -- every operator here is either the MAC
// backend or the one residual adder -- and it is the seams that it exists for.
//
// The pipeline, in scheduler cycles
// ---------------------------------
//
//   T     the scheduler issues slot N: `cur_*` describes it and the memory
//         address is *assigned*
//   T+1   the address is presented to the store; stage A registers `cur_*`
//   T+2   the store answers with slot N's data; stage B holds slot N's control
//   T+3   the operands are one register further on and stage C holds slot N's
//         control, so the MAC consumes it
//
// so the control needs **three** register stages, not one: the scheduler's own
// address register spends a cycle and the X ring's block RAM spends another.
// Stage A carries the weight address too, and the four operands -- activation,
// weight, bias and control -- arrive on the same edge, which is this file's own
// rule and the one it learned by breaking.
//
// **It was two stages until D3c5-b1**, and the third exists because of a
// measurement rather than a principle.  This backend closed at +0.789 ns in
// D3c5-a with the X words arriving from a staging register; with the same
// arithmetic and the words coming out of the ring it measured **-0.13 ns**, on
// all five placements, and every one of the five worst paths began at
// `g_bank[*].mem|RCLK`.  A block RAM's 1.188 ns clock-to-out plus the route from
// where the RAM lands to where the DSP array lands was the whole of it, and a
// fabric register splits that into two paths instead of trying to shorten one.
// It costs a cycle of pipeline and no cycle of schedule.
//
// Where the two closed halves disagree
// ------------------------------------
//
// They disagree by one cycle per extra stage, and only for conv1 and conv2.  The
// scheduler's write-back sets its pending flag on the phase's last slot and
// raises the write enable on the cycle after, so bank 0's A or B write lands
// two cycles later -- while the requant's first pass does not emerge until three,
// because the accept edge is the end of that cycle and the lane's output is
// registered behind it.  conv3 is not short: its first accept is two cycles
// earlier, so its four trunk words are covered by one stage and not two.
//
// So the A and B write transaction is registered three times and the trunk write
// once, and **those are constants, not derivations**.  What establishes them is
// the testbench saying every one of the 32,768 words matched; a latency argued
// from the shape of a pipeline is a hypothesis until something counts the cycles,
// which is what D3c4c-b and then D3c5-a both found out.
//
// One backend, at the widest format
// ---------------------------------
//
// conv1's operand is 11 bits signed (trunk, zero 413), conv2 and conv3's are 9
// ([0, 255]) -- but a 9-bit signed value sign-extended into 11 is the same
// number, and 11 x 8 is one DSP48 tile exactly as 9 x 8 is.  So there is **one**
// MAC, built at 11 bits.
//
// Its output format is the trunk's -- 10 bits, qmax 1023 -- because conv3 writes
// the trunk stream.  conv1 and conv2 want a ReLU tensor's 8 bits at qmax 255,
// and the lane's clamp cannot do both from one parameter, so this module
// saturates their two outputs to 8 bits on the way out.  That is the same
// operator either way: for a zero-point-0 tensor the ReLU *is* the format clamp.
module d64_conv_engine #(
    parameter ROWS         = 64,
    parameter COLS         = 64,
    parameter BAND         = 8,
    parameter A_ROWS       = 11,
    parameter B_ROWS       = 8,
    parameter TRUNK_BANK_W = 2,
    parameter AB_BANK_W    = 1,
    parameter SWITCH_CYCLES = 64,
    parameter C1_CYC       = 4,
    parameter C2_CYC       = 18,
    parameter C3_CYC       = 4,
    parameter C3_IC        = 8,

    parameter IC_PAR   = 8,
    parameter OC_LANES = 16,
    parameter RQ_LANES = 8,
    parameter WGT_W    = 8,
    parameter ACC_W    = 32,
    parameter CFG_BIAS_W = ACC_W,
    parameter integer CFG_STREAM_SRL = 0,
    parameter M_W      = 16,
    parameter SHIFT_W  = 5,
    parameter OUT_W    = 10,      // the trunk's stored width
    parameter ACT_W    = 11,      // the trunk's operand width
    parameter O8_W     = 8,       // a ReLU tensor's stored width

    parameter Z1 = 413,           // the zero point conv1's bias was folded against
    parameter ZERO_T = 413,       // the same, for the residual add

    // D3c5-c: the weight store holds the whole network's 247 tiles and is loaded
    // once, so a block's base is **runtime state, not a parameter**.  A residual
    // block is twenty-six tiles -- conv1 four, conv2 eighteen, conv3 four -- so
    // the three layers' bases are `block_wbase`, `+4` and `+22`, and the next
    // block starts twenty-six along.  `WBASE0` is block 0's, kept as the record
    // of where the trunk starts in the image; `tools/d64_weight_image.py` is
    // where the number comes from.
    parameter WBASE0   = 54,

    parameter W_TILES  = 247,
    parameter W_WORD_W = 1024,
    parameter W_ADDR_W = 8,
    parameter X_AW     = 14,      // 64 rows x 64 cols x 4 banks
    // D3c5-b.  0 keeps D3c5-a's whole-frame store -- 16,384 words of 80 bits in
    // one array, which is the shape the arithmetic was proved against and is not
    // the shape the design has.  16 is H0's ring: rows come from the DDR a row at
    // a time through `d64_x16_ring`, and the scheduler waits at the band boundary
    // for the rows it is about to need.
    parameter X_RING_ROWS = 0,
    parameter AB_AW    = 11,      // 32 rows x 64 cols x 2 banks
    parameter CFG_N    = 64,
    parameter CFG_AW   = 6,
    // D3c6-e.  When set, this engine owns neither its multiplier bank nor
    // its weight store: both come in over the seam below and are shared with
    // every other scheduler in the network.  The schedule, the stores and the
    // metadata do not move -- only where the operands go.
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0,
    // Cycles between the scheduler's `next_last` warning and the multiplier,
    // carried to the bank.  The trunk's warning and its multiplier have not
    // moved since D3c5-b1, so this stays 3.
    parameter integer RQ_LOOKAHEAD = 3,
    // D3c6-e7: the stage monitor's expected-chunk table.  Relative to the
    // directory the flow is run from, which is `board/ddr3` for both Efinity
    // and the composite board model -- and an empty string is the deliberate
    // all-zero table, not an accident.
    parameter CHUNK_ROM_INIT = "../../golden/d64_chunk_crc32.mem",
    // **D3c6-e8: one diagnostic or the other, never both.**  XLR was at 59,677
    // of 60,800 after the chunk monitor went in -- 1,123 spare, of which 902 are
    // routing -- so the two instruments cannot be stacked.  0 builds the
    // three-stage chunk summary; 1 builds the C2 word-by-word probe instead.
    // The functional pipeline is identical either way.
    parameter C2_DIAG = 0,
    parameter A_ROM_INIT = "../../golden/a_consumer_rows01_256x64.mem",
    parameter B_ROM_INIT = "../../golden/b_head128_128x64.mem"
) (
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,
    // D3c5-b: hold the schedule.  Only ever asserted inside the band-boundary
    // switch, where the block has nothing live -- see the note in
    // `d64_trunk_scheduler`.  The engine's own pipeline is deliberately NOT
    // gated: by the time the switch is entered every pending write-back has been
    // emitted and `slot_valid` at the MAC is low, so the stages behind are
    // shifting zeros and gating them would add a way to get the alignment wrong
    // for no benefit.  The testbench checks the claim by holding the same path
    // and comparing all 32,768 words.
    input  wire                hold,
    output wire                busy,
    // D3c5-b: the band the schedule is on and whether the switch in progress
    // leads into conv1.  The X ring's owner is the only thing outside that needs
    // them, and it cannot derive either: `mode` alone cannot say which of the
    // three switches this is, and the band counter belongs to the scheduler.
    output wire [3:0]          cur_band,
    output wire                cur_bandsw,
    output wire                done,
    output wire [31:0]         cycles,

    // The per-channel requant parameters, loaded by whatever owns the layer:
    // {shift[4:0], M[15:0], bias[31:0]} at 0..15 conv1, 16..31 conv2,
    // 32..63 conv3's thirty-two channels.
    input  wire                cfg_we,
    output wire                cfg_protocol_fault,
    input  wire [CFG_AW-1:0]   cfg_wa,
    input  wire [63:0]         cfg_wd,

    // The block input, and nothing else -- A and B are written by this module.
    input  wire                x_we,
    input  wire [X_AW-1:0]     x_wa,
    input  wire [79:0]         x_wd,
    // ---- the shared-PE seam, `EXTERNAL_PE` only ---------------------------
    // Bank-native: the operands are 128 lanes wide because that is what the bank
    // reads, and the mapping from this engine's flat ports happens inside.  Same
    // shape as `d64_down_engine`'s seam, which is the one already proven.
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

    input  wire                w_we,
    input  wire [W_ADDR_W-1:0] w_wa,
    input  wire [W_WORD_W-1:0] w_wd,

    // D3c5-c: which block's weights to read.  One base and not three, because
    // the three are four and twenty-two apart at every block and a controller
    // that could disagree with that is a controller that eventually will.
    input  wire [W_ADDR_W-1:0] block_wbase,
    // D3c6-e6b: which residual block this run is.  The engine has never needed
    // to know -- the cfg is refilled at every boundary and that is the whole of
    // the block's identity -- and this exists so the probe below can say which
    // block's entry it should have been given.
    input  wire [2:0]          blk_idx,

    // ---- the X ring, when X_RING_ROWS > 0 ---------------------------------
    // The fill side is one bank word a beat, in the DDR's own order: the beat
    // index within a row is `col*4 + bank`.  The release side is the engine's
    // own -- the residual add's last write to a row is the last moment that row
    // is read, and nothing outside can know when that was.
    input  wire                xr_en,
    input  wire [5:0]          xr_row,
    input  wire [5:0]          xr_col,
    input  wire [1:0]          xr_bank,
    input  wire [79:0]         xr_word,
    input  wire                xr_last,
    input  wire [5:0]          x_q_row,
    output wire                x_q_ready,
    input  wire [5:0]          x_f_row,
    output wire                x_q_free,
    output wire [5:0]          x_rel_row,
    output wire                x_rel_en,
    output wire [31:0]         x_ovf,
    output wire [31:0]         x_rbf,
    // **Whether any ring slot still holds a row nobody has released.**  The ring
    // has computed this since D3c5-c and the port was left open -- `.live()` --
    // so `blk_clean` in `d64_h0_block` was testing an undriven net: `x` in
    // simulation, tied low in synthesis, and permissive in exactly the check that
    // exists to catch a block boundary starting from a dirty ring.
    output wire                x_live,

    // ---- observability ----
    output reg  [31:0] a_writes,
    output reg  [31:0] b_writes,
    output reg  [31:0] t_writes,
    output reg  [31:0] rq_accepts,
    output reg  [31:0] rq_overruns,
    // The requantised activation as it leaves the backend, clamped to the layer's
    // format, with the write that carries it.
    output wire [63:0]  a_wr_data_o,
    output wire [63:0]  b_wr_data_o,
    output wire [79:0]  t_wr_data_o,
    output wire [31:0]  tw_cnt_o,
    output wire [31:0]  add_cnt_o,
    output wire [31:0]  add_diff_o,
    // **How many trunk writes happened with the engine's result not valid.**
    // The design asserts, from simulation, that not one of the 16,384 trunk
    // writes of a block happens without `y_valid` -- and `y_valid` gates
    // nothing, the write goes out regardless.  If the assertion does not hold on
    // the board the accumulator is whatever it was, the requant puts out its
    // zero point, and the residual add passes its skip through unchanged, which
    // is exactly what blocks 0, 1 and 2 are measured doing.  Zero here means the
    // assertion holds and the accumulator is genuinely zero.
    output wire [31:0]  no_yv_o,
    // **How many trunk writes carried an all-zero-point branch.**  A working
    // block writes `pack10` values that are mostly not 413; three whole blocks
    // of 413 is 49,152 words and is what the residual add passing its skip
    // through looks like from here.  `tw_en` is the write cycle, so this
    // observes `pack10` on the cycle it is actually written -- which is the
    // thing a probe on `y_bus` at a MAC slot cannot tell you.
    output wire [31:0]  zero_br_o,
    // **Five sticky bits, one per block, and the pair is the whole point.**
    // `zero_br` above cannot tell an `M` of zero from an accumulator of zero:
    // `q = (acc * M) >> shift + ZERO` is the zero point either way, and the
    // board's three echoing blocks are consistent with both.  This is the
    // first of two probes that separate them -- is the requant configuration
    // the one this block's image has? -- and it costs five flip-flops and no
    // wide bus.  `rq_bank_ok` in `d64_encoder_trunk` asks the same question
    // again on the far side of the compat map and the owner mux.
    output wire [4:0]   cfg_local_ok_o,
    // **The same question asked of the refill instead of the storage.**  These
    // five are set at the moment the stream writes slot 32, comparing the word
    // on `cfg_wd` against the same table -- so they say whether the ROM and the
    // block-boundary refill *delivered* the right value, while `cfg_local_ok`
    // says whether the engine's `cfg` still *has* it at the accept.  A 11111
    // here with a 11000 there is the storage; a 11000 here is the ROM or the
    // addressing, and the two are different repairs.
    output wire [4:0]   stream_ok_o,
    // **What block 0 actually holds**, {og, shift, M} of lane 0 of the conv3
    // requant word at that block's first accept, 22 bits and no wide bus.  The
    // masks say a value is wrong; this says which value it is, which is the
    // difference between naming the fault and bisecting for it.
    // **What the refill actually delivered for slot 32.**  The masks say a value
    // is wrong; this says which value it is, and one 21-bit word scanned against
    // the image offline names the fault -- another block's entry is an addressing
    // error, zero is an unloaded or wrong ROM, and a word that appears nowhere in
    // the image is corrupt content.  `{shift, M}` of lane 0, og 0, block 0.
    output wire [20:0]  refill_c3_o,
    output wire         refill_seen_o,
    // `refill_c3_o` against the image's entry, so that it has a consumer that is
    // not a telemetry slot -- twenty-one registers whose only reader is a report
    // slot is the shape the mapper has folded away four times in this stage --
    // and so that it is a pass criterion rather than decoration.
    output wire         refill_ok_o,
    // **P7d: the first conv1 slot the MAC consumes, against constants frozen in
    // simulation.**  A sticky "was it ever non-zero" says nothing; this compares
    // four specific things at one deterministic point -- the schedule is fixed, so
    // the point is -- and the four answers bisect block 0's datapath:
    //
    //   p7_waddr_ok   the weight tile address      -> scheduler / address generation
    //   p7_wdata_ok   the weight bytes and the lane -> runtime store read / the map
    //   p7_act_ok     the activation lanes          -> the X ring / window / packing
    //   p7_acc_ok     one accumulator at the accept -> the MAC reduction
    //
    // The constants are `tb_d64_encoder_trunk_board_model`'s P7d print, taken at
    // `c_live && c_mode == 2` and at the first `rq_accept`; that testbench is where
    // they can be re-derived if the schedule ever moves.
    output wire [3:0]   p7_ok_o,
    output wire [1:0]   p7_seen_o,   // {acc_seen, slot_seen}
    output wire [63:0]  p7_acc_o,    // conv1 phase-end accumulator
    output wire [63:0]  p7_fs_o,     // ...and after the first slot
    output wire         p7_bias_seen_o,
    output wire [63:0]  p7_bias_o,   // the bias, before any product
    output wire [63:0]  p7_br_o,     // ...and bias_r, where it was read from
    output wire [ACC_W-1:0] p7_cfg0_o, p7_cfg1_o,  // the array's first two entries
    // **Whether block 0's entries 0 and 1 were written at all**, one bit each.
    // Without these the two values above cannot be told from their reset state, and
    // the model's required assertion that the capture happened would have nothing to
    // read.  Exported as one two-bit port rather than two, so the chain up to the
    // telemetry map costs one wire per level instead of two.
    output wire [1:0]   p7_cfg_seen_o,            // {cfg1_seen, cfg0_seen}
    // ---- P7h: the load edge, and what the array was handed ----------------
    //
    // **The previous capture was one cycle late.**  It fired on a registered copy
    // of `b_pstart` and read `bias_r` there, so it explained the load at cycle N
    // with the control of cycle N+1.  Stable control across that boundary makes
    // the two readings agree, so a model could not have caught it and the board
    // is where it came apart.
    //
    // `{b_mode[3:0], b_ocg, abase[5:0], 1'b0}`.  `cfg_base` returns 0 for mode 2
    // whatever `b_ocg` is, so `12'h200` is what conv1 of block 0 must read: if it
    // does, the address is right and the array's *contents* are what is wrong.
    output wire [11:0]  p7_ctrl_o,
    // D3c5-b: the trunk write's own enable and address, so the DDR boundary can
    // pack the word onto the write FIFO and, at the last word of a row, hand the
    // ring slot back.  Reading `t_writes` and inferring the moment would be the
    // same mistake as inferring a latency from the shape of a pipeline.
    output wire         t_wr_en_o,
    output wire [11:0]  t_wr_addr_o,
    output wire [TRUNK_BANK_W-1:0] t_wr_bank_o,
    // D3c5-c: the last word of the block has been written.  This is the
    // *registered* write transaction and not the scheduler's enable, because
    // D3c5-b1 proved the two are a cycle apart and the enable is the early one --
    // a block barrier built on the enable starts the next block while the last
    // word of this one is still in the pipeline.
    output wire         t_wr_last_o,
    // D3c6-e7: the stage monitor.  `obs_en` is the block-0 first-pass window,
    // generated one level up where `blk` and the redo counter live, and
    // `obs_chunk` is every result it produces -- flat, because five levels of
    // hierarchy would otherwise each need fifteen more ports.
    input  wire         obs_en,
    output wire [272:0] obs_chunk,
    output wire [287:0] c2_chunk,
    output wire [2:0]    c2_dbg
);

    // ------------------------------------------------------------------
    // the schedule
    // ------------------------------------------------------------------

    wire [11:0] s_trunk_rd_addr;
    wire [TRUNK_BANK_W-1:0] s_trunk_rd_bank;
    wire        s_trunk_rd_en, s_trunk_wr_en;
    wire [11:0] s_trunk_wr_addr;
    wire [TRUNK_BANK_W-1:0] s_trunk_wr_bank;
    wire        s_a_wr_en, s_a_rd_en, s_b_wr_en, s_b_rd_en;
    wire [9:0]  s_a_wr_addr, s_a_rd_addr, s_b_wr_addr, s_b_rd_addr;
    wire [AB_BANK_W-1:0] s_a_wr_bank, s_a_rd_bank, s_b_wr_bank, s_b_rd_bank;
    wire        s_b_rd_sub;
    wire [3:0]  s_mode;
    wire [3:0]  s_icg, s_tap;
    wire        s_ocg, s_live, s_pstart, s_plast;
    wire [3:0]  s_dbg;

    d64_trunk_scheduler #(
        .ROWS(ROWS), .COLS(COLS), .BAND(BAND), .A_ROWS(A_ROWS),
        .B_ROWS(B_ROWS), .TRUNK_BANK_W(TRUNK_BANK_W),
        .AB_BANK_W(AB_BANK_W), .SWITCH_CYCLES(SWITCH_CYCLES),
        .C1_CYC(C1_CYC), .C2_CYC(C2_CYC), .C3_CYC(C3_CYC), .C3_IC(C3_IC)
    ) u_sched (
        .clk(clk), .rst_n(rst_n), .start(start), .hold(hold),
        .busy(busy), .done(done), .cycles(cycles),
        .cur_band(cur_band), .cur_bandsw(cur_bandsw),
        .trunk_rd_en(s_trunk_rd_en), .trunk_rd_addr(s_trunk_rd_addr),
        .trunk_rd_bank(s_trunk_rd_bank),
        .trunk_wr_en(s_trunk_wr_en), .trunk_wr_addr(s_trunk_wr_addr),
        .trunk_wr_bank(s_trunk_wr_bank),
        .a_wr_en(s_a_wr_en), .a_wr_addr(s_a_wr_addr), .a_wr_bank(s_a_wr_bank),
        .a_rd_en(s_a_rd_en), .a_rd_addr(s_a_rd_addr), .a_rd_bank(s_a_rd_bank),
        .b_wr_en(s_b_wr_en), .b_wr_addr(s_b_wr_addr), .b_wr_bank(s_b_wr_bank),
        .b_rd_en(s_b_rd_en), .b_rd_addr(s_b_rd_addr), .b_rd_bank(s_b_rd_bank),
        .b_rd_sub(s_b_rd_sub), .mode(s_mode),
        .trunk_reads(), .trunk_writes(), .switches(),
        .cur_icg(s_icg), .cur_tap(s_tap), .cur_ocg(s_ocg), .cur_live(s_live),
        .cur_pstart(s_pstart), .cur_plast(s_plast),
        .dbg_pix_row(), .dbg_pix_col(), .dbg_phase_cyc(),
        .dbg_a_wr_row(), .dbg_a_wr_col(), .dbg_a_rd_row(), .dbg_a_rd_col(),
        .dbg_b_wr_row(), .dbg_b_wr_col(), .dbg_b_rd_row(), .dbg_b_rd_col(),
        .dbg_a_wr_slot(), .dbg_a_rd_slot(), .dbg_b_wr_slot(), .dbg_b_rd_slot()
    );

    // ------------------------------------------------------------------
    // the stores
    // ------------------------------------------------------------------
    //
    // Inside the engine rather than on ports, so the read latency that the
    // pipeline above reasons about is this module's own and not a promise about
    // somebody else's memory.

    wire [79:0] x_q;
    wire [63:0] a_q, b_q;
    wire [W_WORD_W-1:0] w_q;

    // The A and B write transaction, registered once here rather than taken
    // straight from the scheduler.  See the timing note at the top: without this
    // stage both writes arrive a cycle before the requant can fill them.
    wire [AB_AW-1:0] a_wa;
    wire [AB_AW-1:0] b_wa;
    wire [63:0]      a_wd;
    wire [63:0]      b_wd;
    wire             a_we;
    wire             b_we;

    // D3c5-b1.  The trunk write's own transaction, registered once so that its enable,
    // address and bank stay together -- the rule this file has now had to learn
    // in both directions.
    reg        tw_en;
    reg [11:0] tw_addr;
    reg [TRUNK_BANK_W-1:0] tw_bank;


    // The requant handshake, declared here and assigned below: several stages
    // before the MAC consume it, and the MAC is what makes it meaningful.
    wire rq_accept;

    wire [X_AW-1:0]  x_ra  = {s_trunk_rd_addr, s_trunk_rd_bank};
    wire [AB_AW-1:0] a_ra  = {s_a_rd_addr, s_a_rd_bank};
    wire [AB_AW-1:0] b_ra  = {s_b_rd_addr, s_b_rd_bank};

    // The release: conv3 writes a row's four channel words as it passes each
    // pixel, one band word a cycle, so the write of (row, COLS-1, bank 3) is the
    // last thing that row is ever wanted for.  It lands three cycles after the
    // row's last read was captured into `skip_r`, which is what makes it safe to
    // hand the slot back here rather than at the read.
    // D3c5-b1: the release is taken from the *registered* write transaction, not
    // from the scheduler's enable.  The row stops being wanted when the last word
    // of it is actually written, and the write is now a cycle behind the enable --
    // so releasing on the enable would hand the slot back a cycle before the
    // write that still needs it has happened.
    assign x_rel_row = tw_addr[11:6];
    assign x_rel_en  = tw_en
                    && (tw_addr[5:0] == COLS[5:0] - 6'd1)
                    && (tw_bank == {TRUNK_BANK_W{1'b1}});

    generate
        if (X_RING_ROWS > 0) begin : g_ring
            d64_x16_ring #(.ROWS(X_RING_ROWS), .COLS(COLS), .ROW_W(6)) u_x (
                .clk(clk), .rst_n(rst_n),
                .rd_addr(s_trunk_rd_addr), .rd_bank(s_trunk_rd_bank),
                .rd_used(s_trunk_rd_en), .rd_data(x_q),
                .wr_row(xr_row), .wr_col(xr_col), .wr_bank(xr_bank),
                .wr_word(xr_word), .wr_en(xr_en), .wr_last(xr_last),
                .rel_row(x_rel_row), .rel_en(x_rel_en),
                .q_row(x_q_row), .q_ready(x_q_ready),
                .f_row(x_f_row), .f_free(x_q_free),
                .ovf(x_ovf), .rbf(x_rbf), .live(x_live)
            );
        end
        else begin : g_flat
            // D3c5-a's store, unchanged, so that testbench keeps passing against
            // the module it was written for rather than against a rewrite.
            d64_tc_ram #(.DW(80), .AW(X_AW)) u_x (
                .clk(clk), .we(x_we), .wa(x_wa), .wd(x_wd),
                .re(1'b1), .ra(x_ra), .rd(x_q)
            );
            assign x_q_ready = 1'b1;
            assign x_q_free  = 1'b1;
            assign x_ovf     = 32'd0;
            assign x_rbf     = 32'd0;
            // There is no ring in this shape and so nothing can be held in one:
            // "no slot is live" is true, not unknown, and D3c5-a's `blk_clean`
            // term therefore keeps its meaning.
            assign x_live    = 1'b0;
        end
    endgenerate

    d64_tc_ram #(.DW(64), .AW(AB_AW)) u_a (
        .clk(clk), .we(a_we), .wa(a_wa), .wd(a_wd),
        .re(1'b1), .ra(a_ra), .rd(a_q)
    );

    // **B is ten bits wide and the tenth is load-bearing, not slack.**  Every
    // address B is handed is `(slot*64 + col)*2 + bank`: the slot is three bits
    // and wraps at `B_ROWS-1`, `col` is six, and the bank is one, so the largest
    // is `7*64*2 + 63*2 + 1 = 1023` -- ten bits with the last code used and
    // nothing to spare.
    //
    // That is **measured, not argued**.  The integration testbench watches both
    // B ports and reports the extremes; on the full encoder+trunk run it reads
    // `max write 1023  max read 1023  bit10 seen 0`, and the same monitor is
    // left in place as the tooth -- `b_wa` stays eleven bits, so if a later
    // change ever drives bit 10 the gate fires instead of the address silently
    // aliasing onto a word that is no longer there.
    //
    // Declaring `AW(AB_AW)` built 2048 words for an address space that reaches
    // 1023, and the mapper does not prune the half that is never named.  A never
    // changes: its slot and column reach further and its eleven bits are all
    // used.
    localparam B_AW = AB_AW - 1;
    d64_tc_ram #(.DW(64), .AW(B_AW)) u_b (
        .clk(clk), .we(b_we), .wa(b_wa[B_AW-1:0]), .wd(b_wd),
        .re(1'b1), .ra(b_ra[B_AW-1:0]), .rd(b_q)
    );

    // The weight tile index, in the image's own layout:
    //   base + (tap * IC_GROUPS + icg) * OC_GROUPS + ocg
    // which is `tools/d64_weight_image.py`'s rule and not a convenient one.
    reg [W_ADDR_W-1:0] w_ra;
    always @* begin
        if (s_mode == 4'd2)       // conv1: 4 IC groups, 1 tap, 1 OC group
            w_ra = block_wbase + {4'd0, s_icg};
        else if (s_mode == 4'd3)  // conv2: 9 taps x 2 IC groups
            // `(s_tap * 2)`, not `{2'd0, s_tap, 1'b0}`.  The two are the
            // same number and are **not** the same to the mapper: with the
            // constant 4 written in front of a concatenated shift, Efinity
            // 2026.1.132.4.5 emits a netlist that computes this sum minus 2,
            // on every one of the 65536 inputs.  `sim/map_equiv/out/p_wra_c2`
            // is that sweep, and it is what the board read as weight address
            // 56 where the RTL says 58.  The multiply is exact on every
            // vector; see `docs/EFINITY_MAP_EQUIVALENCE_2026-10-05.md` and
            // `MAP_EQUIVALENCE_NOTES.md` before rewriting this line.
            w_ra = block_wbase + 8'd4 + (s_tap * 2) + {4'd0, s_icg};
        else                      // conv3: 2 IC groups x 2 OC groups
            w_ra = block_wbase + 8'd22 + {4'd0, s_icg, 1'b0} + {5'd0, s_ocg};
    end

    // **Registered, and that is the whole stage-A argument.**  The activation and
    // the control ride the scheduler's *own* address register, which is one cycle
    // behind `cur_*`; this address was combinational from `cur_*`, so its data
    // arrived a cycle before theirs and the multiplier paired slot N's weights
    // with slot N-1's activation.  The engine's header says a register that
    // timing forces goes on all three at once, never on one -- and this is that
    // rule being broken from the other direction, by leaving one *out*.
    reg [W_ADDR_W-1:0] w_ra_r;
    always @(posedge clk) w_ra_r <= w_ra;

    // D3c6-e: the weight image is the network's, not this engine's.  There is one
    // 247-tile store and every scheduler reads it; giving the trunk a second copy
    // would be 52 more RAM10 for bytes that are already on the device.
    assign pe_wt_rd_en   = 1'b1;          // this engine reads every cycle
    assign pe_wt_rd_addr = w_ra_r;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_weight_store
            assign w_q = pe_wt_rd_data;
        end else begin : g_local_weight_store
            d64_weight_store #(.TILES(W_TILES), .WORD_W(W_WORD_W), .ADDR_W(W_ADDR_W))
            u_w (
                .clk(clk), .rst_n(rst_n),
                .wr_en(w_we), .wr_addr(w_wa), .wr_data(w_wd),
                .rd_en(1'b1), .rd_addr(w_ra_r), .rd_data(w_q)
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // the requant configuration
    // ------------------------------------------------------------------

    // **Two arrays, disjoint bit ranges, one write.**  The requant reads bits
    // 32..52 with stage C's `bbase`; the bias reads bits 0..31 with stage B's
    // `abase`.  Sharing one 64-bit array, the mapped netlist dropped the entire
    // bias field -- 2048 flip-flops, bits 0..31, loaded into no register anywhere
    // -- and handed the bank `cfg_2`, a 32-bit pair carrying `cfg_wd[48:63]`.
    // That is the field mixing: `bias_r` came out as `0x00180019` on the board in
    // ten runs of ten, which is the requant shift field of two entries and not a
    // value any address in the image can produce.  See
    // `docs/P7_BIAS_NETLIST_2026-10-03.md`.
    //
    // Nothing else moves.  Same `cfg_we`, same `cfg_wa`, same cycle, address 63
    // included; no added read latency, no array clearing, and the field order in
    // the write is the protocol's and not the array's.
    localparam RQ_W = M_W + SHIFT_W;      // 21
    reg [CFG_BIAS_W-1:0]  cfg_bias [0:CFG_N-1];
    reg [RQ_W-1:0]   cfg_rq   [0:CFG_N-1];
    integer ci;
    always @(posedge clk) if (cfg_we && CFG_STREAM_SRL==0) begin
        cfg_bias[cfg_wa] <= cfg_wd[CFG_BIAS_W-1:0];
        cfg_rq[cfg_wa]   <= {cfg_wd[48 +: SHIFT_W], cfg_wd[32 +: M_W]};
    end

    // synthesis translate_off
    always @(posedge clk) if (cfg_we && CFG_BIAS_W < ACC_W) begin
        if (cfg_wd[ACC_W-1:0] !== {{(ACC_W-CFG_BIAS_W){cfg_wd[CFG_BIAS_W-1]}},cfg_wd[CFG_BIAS_W-1:0]}) begin
            $display("DOMAIN_ERROR: trunk bias does not fit signed CFG_BIAS_W");
            $finish;
        end
    end
    // synthesis translate_on
    // P7f: what the loader wrote into entries 0 and 1 **of block 0**, and whether
    // either was written at all.
    //
    // **Two defects lived here, and both were in the measurement rather than in the
    // design.**  The capture had no block condition and no freeze, so it kept whatever
    // the loader wrote last -- and every residual block runs `S_CFG` and rewrites this
    // 64-entry array, so "last" is block 4.  The board read `FFFCDD30` / `0001EB59`,
    // and those are entries **256 and 257** of `d64_trunk_cfg_rom`: block 4's cfg[0]
    // and cfg[1].  Block 0's are entries 0 and 1.  The two numbers were the probe's own
    // explanation for the disagreement it was blamed for, and nothing in them says the
    // engine's array is wrong.
    //
    // **One owner.**  These two registers used to be written here *and* reset in the
    // bias probe's block, which `tools/check_double_drivers.py` reports as
    // "p7_cfg0_v, p7_cfg1_v ... written from two always blocks".  The duplicate was
    // load-bearing: this block had `always @(posedge clk)` and no reset at all, so
    // deleting the other assignment would have left both registers X.  The reset lives
    // here now and the bias probe no longer writes them.
    //
    // **`blk_idx`, not a delayed copy.**  It is `blk_r` from the trunk, held for the
    // whole of a block, so it is stable across the entire 64-cycle load -- and this
    // file already trusts it twice for exactly that (`stream_ok`, and `refill_c3` at
    // the `cfg_wa == 6'd32` probe).  The one-cycle delay on the enable, address and
    // data stays: `cfg_wd` arrives with the write rather than with the address, which
    // is a skew that has misled a testbench in this project before.
    //
    // **The freeze, and why a redo is the reason for it.**  A redo re-enters `S_CFG`
    // and rewrites these entries with identical content, so it cannot change the
    // *value* -- but it would set a seen flag a second time, and a seen bit that can be
    // set twice is not a statement that block 0 was loaded.  The flags were declared as
    // `p7_w0_v`/`p7_w1_v` and never read anywhere; they are the seen flags now, because
    // that is what they have to be for the captured value to mean anything.
    reg [ACC_W-1:0] p7_cfg0_v, p7_cfg1_v;
    reg             p7_cfg0_seen, p7_cfg1_seen;
    reg             cfg_we_q;
    reg  [5:0]      cfg_wa_q;
    reg  [63:0]     cfg_wd_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_we_q <= 1'b0; cfg_wa_q <= 6'd0; cfg_wd_q <= 64'd0;
            p7_cfg0_v <= {ACC_W{1'b0}}; p7_cfg1_v <= {ACC_W{1'b0}};
            p7_cfg0_seen <= 1'b0; p7_cfg1_seen <= 1'b0;
        end
        else begin
            cfg_we_q <= cfg_we;
            cfg_wa_q <= cfg_wa;
            cfg_wd_q <= cfg_wd;
            if (!p7_cfg0_seen && cfg_we_q && (blk_idx == 3'd0) && (cfg_wa_q == 6'd0)) begin
                p7_cfg0_v    <= cfg_wd_q[ACC_W-1:0];
                p7_cfg0_seen <= 1'b1;
            end
            if (!p7_cfg1_seen && cfg_we_q && (blk_idx == 3'd0) && (cfg_wa_q == 6'd1)) begin
                p7_cfg1_v    <= cfg_wd_q[ACC_W-1:0];
                p7_cfg1_seen <= 1'b1;
            end
        end
    end

    // conv1 0..15, conv2 16..31, conv3 32..63 in two OC groups of sixteen.
    function [CFG_AW-1:0] cfg_base;
        input [3:0] md;
        input       og;
        begin
            cfg_base = (md == 4'd2) ? 6'd0
                     : (md == 4'd3) ? 6'd16
                     : (6'd32 + {1'b0, og, 4'b0});
        end
    endfunction

    // ------------------------------------------------------------------
    // stage A and stage B, and D3c5-b1's stage C
    // ------------------------------------------------------------------
    //
    // The third stage is a **fabric register on the ring's output**, and it
    // exists because of what the X16 ring's block RAM costs.  D3c5-a closed this
    // backend at +0.789 ns with the X words arriving from a staging register; the
    // same backend with the same arithmetic measured **-0.13 ns** once they
    // arrived from the ring, on every one of five placements, and all five worst
    // paths began at `g_bank[*].mem|RCLK`.  A block RAM's 1.188 ns clock-to-out
    // plus the routing from where the RAM lands to where the DSP array lands is
    // the whole of the difference.
    //
    // A register breaks that into two paths instead of shortening one: RAM to
    // flip-flop, and flip-flop to DSP.  It costs a cycle of pipeline and no cycle
    // of schedule -- `cycles` is still 108,120, because the scheduler is not what
    // got longer.
    //
    // All four operands move together, which is this file's own rule from
    // D3c5-a: a register that timing forces goes on activation, weight and
    // control at once, never on one of them.  That rule was learned by breaking
    // it -- `w_ra_r` was left out and slot N's weights met slot N-1's activation,
    // and every error in A and in B came from it.
    reg [3:0] a_mode, a_icg, a_tap;
    reg       a_ocg, a_live, a_pstart, a_plast;
    reg [3:0] b_mode;
    reg       b_ocg, b_live, b_pstart, b_plast;
    reg [3:0] c_mode;
    reg       c_ocg, c_live, c_plast;

    reg [79:0] x_q_d;
    reg [63:0] a_q_d, b_q_d;
    reg [W_WORD_W-1:0] w_q_d;
    reg [IC_PAR*ACT_W-1:0] act_q;
    wire [63:0] oper_word_before_c = (b_mode == 4'd3) ? a_q : b_q;

    reg [IC_PAR*ACT_W-1:0] act_before_c;
    integer abi;
    always @* begin
        act_before_c = {(IC_PAR*ACT_W){1'b0}};
        for (abi = 0; abi < IC_PAR; abi = abi + 1)
            if (b_mode == 4'd2)
                act_before_c[abi*ACT_W +: ACT_W] = $signed({1'b0, x_q[abi*10 +: 10]});
            else
                act_before_c[abi*ACT_W +: ACT_W] =
                    $signed({1'b0, oper_word_before_c[abi*8 +: 8]});
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_mode <= 4'd0; a_icg <= 4'd0; a_tap <= 4'd0; a_ocg <= 1'b0;
            a_live <= 1'b0; a_pstart <= 1'b0; a_plast <= 1'b0;
            b_mode <= 4'd0; b_ocg <= 1'b0; b_live <= 1'b0;
            b_pstart <= 1'b0; b_plast <= 1'b0;
            c_mode <= 4'd0; c_ocg <= 1'b0; c_live <= 1'b0; c_plast <= 1'b0;
            x_q_d <= 80'd0; a_q_d <= 64'd0; b_q_d <= 64'd0;
            if(EXTERNAL_PE==0) w_q_d <= {W_WORD_W{1'b0}};
            act_q <= {(IC_PAR*ACT_W){1'b0}};
        end
        else begin
            a_mode <= s_mode; a_icg <= s_icg; a_tap <= s_tap; a_ocg <= s_ocg;
            a_live <= s_live; a_pstart <= s_pstart; a_plast <= s_plast;
            b_mode <= a_mode; b_ocg <= a_ocg; b_live <= a_live;
            b_pstart <= a_pstart; b_plast <= a_plast;
            c_mode <= b_mode; c_ocg <= b_ocg; c_live <= b_live;
            c_plast <= b_plast;
            // The data registers, on the same edge as the control that describes
            // them.  `x_q` is the ring's own output register promoted into fabric.
            x_q_d <= x_q; a_q_d <= a_q; b_q_d <= b_q; if(EXTERNAL_PE==0) w_q_d <= w_q;
            act_q <= act_before_c; // OLD b_mode/data; not gated by b_live.
        end
    end

    // ------------------------------------------------------------------
    // the operand
    // ------------------------------------------------------------------
    //
    // One word is one slot's activation: eight channels of one IC group, because
    // that is exactly what a weight tile's eight IC lanes want.
    //
    // **No zero-point conversion, and that is D3c5-b1.**  Every operand is now
    // its stored value extended to the multiplier's width -- conv1's 10-bit
    // trunk word zero-extended into 11 bits signed, conv2 and conv3's 8-bit ReLU
    // values into the same field.  The trunk's zero point of 413 is gone from
    // this path, and where it went is the bias: `sum((q - z) * w) + b` is
    // `sum(q * w) + (b - z * sum(w))` exactly, so folding `- z * sum(w)` into the
    // preloaded bias lets the multiplier read the stored word directly.
    // `tools/generate_conv_arithmetic_golden.py` computes the fold and re-derives
    // it through the correlation before it will write anything.  It is worth
    // about a tenth of a nanosecond -- real, and on its own not enough.
    //
    // `Z1` stays a parameter because it is the record of which zero point the
    // fold was taken against -- the engine no longer needs it, and whoever
    // computes the bias does.

    // Production preselection before the existing stage-C edge; no extra cycle.
    wire [IC_PAR*ACT_W-1:0] act=EXTERNAL_PE ? act_before_c : act_q;
    // synthesis translate_off
    wire [63:0] oper_word = (c_mode == 4'd3) ? a_q_d : b_q_d;

    reg [IC_PAR*ACT_W-1:0] act_expected_after_c;
    integer aei;
    always @* begin
        act_expected_after_c = {(IC_PAR*ACT_W){1'b0}};
        for (aei = 0; aei < IC_PAR; aei = aei + 1)
            if (c_mode == 4'd2)
                act_expected_after_c[aei*ACT_W +: ACT_W] = $signed({1'b0, x_q_d[aei*10 +: 10]});
            else
                act_expected_after_c[aei*ACT_W +: ACT_W] =
                    $signed({1'b0, oper_word[aei*8 +: 8]});
    end

    always @(negedge clk) if(rst_n && act_q !== act_expected_after_c)
        $fatal(1,"STAGE_C_ACT_ALIGNMENT_ERROR old b_mode must select data for new c_mode; no valid gating");
    // synthesis translate_on

    // ------------------------------------------------------------------
    // the backend
    // ------------------------------------------------------------------

    wire [OC_LANES*M_W-1:0]     m_bus;
    wire [OC_LANES*SHIFT_W-1:0] sh_bus;
    wire [OC_LANES*ACC_W-1:0]   bias_bus;
    wire [OUT_W-1:0]            zero_bus;

    // The bias is read at `pix_start`, which stage B now carries one cycle before
    // the phase's first slot, so stage B's own phase is the one to look up.
    reg [CFG_AW-1:0] abase;
    always @* abase = cfg_base(b_mode, b_ocg);

    // The requant's M, shift and zero have to survive the whole pass sequence,
    // and at conv3 a new accept arrives while the previous phase is still being
    // emitted.  So they are latched at the accept and switched combinationally on
    // the accept cycle itself, which is the only cycle the MAC reads them
    // directly.  The accept is stage C's, because the MAC consumes stage C.
    reg [CFG_AW-1:0] bbase;
    always @* bbase = cfg_base(c_mode, c_ocg);
    wire [OC_LANES*CFG_BIAS_W-1:0] cfg_bias_now;
    wire [OC_LANES*M_W-1:0] cfg_m_now;
    wire [OC_LANES*SHIFT_W-1:0] cfg_shift_now;
    wire cfg_ready,cfg_srl_fault;
    assign cfg_protocol_fault=cfg_srl_fault ||
        ((CFG_STREAM_SRL!=0)&&(c_live||b_pstart||c_plast)&&!cfg_ready);
    genvar cfg_lane;
    generate if(CFG_STREAM_SRL!=0)begin:g_cfg_srl
        d64_trunk_cfg_srl_bank #(.BIAS_W(CFG_BIAS_W),.M_W(M_W),.SHIFT_W(SHIFT_W)) u_cfg(
            .clk(clk),.rst_n(rst_n),.cfg_we(cfg_we),.cfg_wa(cfg_wa),.cfg_wd(cfg_wd),
            .bias_base(abase),.rq_base(bbase),.bias_o(cfg_bias_now),
            .m_o(cfg_m_now),.shift_o(cfg_shift_now),.ready(cfg_ready),.protocol_fault(cfg_srl_fault));
        // synthesis translate_off
        initial if(CFG_N!=64||CFG_AW!=6||OC_LANES!=16)
            $fatal(1,"TRUNK_CFG_SRL_REQUIRES_64_ENTRIES_16_LANES");
        // synthesis translate_on
    end else begin:g_cfg_array
        assign cfg_ready=1'b1;assign cfg_srl_fault=1'b0;
        for(cfg_lane=0;cfg_lane<OC_LANES;cfg_lane=cfg_lane+1)begin:g_lane
            assign cfg_bias_now[cfg_lane*CFG_BIAS_W+:CFG_BIAS_W]=cfg_bias[abase+cfg_lane[CFG_AW-1:0]];
            assign cfg_m_now[cfg_lane*M_W+:M_W]=cfg_rq[bbase+cfg_lane[CFG_AW-1:0]][0+:M_W];
            assign cfg_shift_now[cfg_lane*SHIFT_W+:SHIFT_W]=cfg_rq[bbase+cfg_lane[CFG_AW-1:0]][M_W+:SHIFT_W];
        end
    end endgenerate

    wire [OUT_W-1:0] trunk_zero_live = RUNTIME_PROFILE ? profile_trunk_zero : ZERO_T[OUT_W-1:0];

    reg [OC_LANES*M_W-1:0]     hm;
    reg [OC_LANES*SHIFT_W-1:0] hs;
    reg [OUT_W-1:0]            hz;

    integer mi;
    always @(posedge clk) begin
        if (rq_accept) begin
            for (mi = 0; mi < OC_LANES; mi = mi + 1) begin
                hm[mi*M_W +: M_W]         <= cfg_m_now[mi*M_W +: M_W];
                hs[mi*SHIFT_W +: SHIFT_W] <= cfg_shift_now[mi*SHIFT_W +: SHIFT_W];
            end
            hz <= (c_mode == 4'd4) ? trunk_zero_live : {OUT_W{1'b0}};
        end
    end

    // conv1 and conv2 write a ReLU tensor, whose zero point is 0; only conv3
    // writes the trunk stream.
    wire [OUT_W-1:0] zero_now = (c_mode == 4'd4) ? trunk_zero_live : {OUT_W{1'b0}};

    reg [OC_LANES*M_W-1:0]     m_now;
    reg [OC_LANES*SHIFT_W-1:0] s_now;
    integer ni;
    always @* begin
        for (ni = 0; ni < OC_LANES; ni = ni + 1) begin
            m_now[ni*M_W +: M_W]         = cfg_m_now[ni*M_W +: M_W];
            s_now[ni*SHIFT_W +: SHIFT_W] = cfg_shift_now[ni*SHIFT_W +: SHIFT_W];
        end
    end

    assign m_bus     = rq_accept ? m_now : hm;
    assign sh_bus    = rq_accept ? s_now : hs;
    assign zero_bus  = rq_accept ? zero_now : hz;

    // ------------------------------------------------------------------
    // D3c6-e6b: is the conv3 requant word this block's image says it is?
    // ------------------------------------------------------------------
    //
    // Lane 0 of the conv3 requant word, as the bank is about to receive it.
    // `rq_accept` selects `m_now`/`s_now`, which are `cfg[32 + og*16]` by
    // construction, so at the accept this is exactly the entry the block's
    // stream was supposed to have written and nothing else.
    wire [20:0] rq_lane0_now = {sh_bus[SHIFT_W-1:0], m_bus[M_W-1:0]};

    // **A copy of the image's table and not a reference to it.**  The values
    // are `control/generated/d64_trunk_cfg_rom.v` entries 32/48, 96/112,
    // 160/176, 224/240 and 288/304 -- M[15:0] and shift[4:0] of lane 0 for
    // conv3's two output groups -- and `sim/d3c5/blkcfg.mem` holds the same
    // words at the same indices.  A reference would have to be read through
    // `cfgrom`, `cfg_base_r` and `cfg_k`, which is the addressing this probe
    // exists to test: comparing a value against itself is not a measurement,
    // and this project has already paid once for a probe whose two sides moved
    // together.  `tools/d64_trunk_cfg_rtl.py` owns the image; if it is ever
    // regenerated this table has to be regenerated with it.
    function [20:0] c3_lane0;
        input [2:0] b;
        input       og;
        begin
            case ({b, og})
                4'd0: c3_lane0 = {5'd24, 16'hC5F4};
                4'd1: c3_lane0 = {5'd24, 16'h9B3E};
                4'd2: c3_lane0 = {5'd23, 16'hBBBE};
                4'd3: c3_lane0 = {5'd24, 16'hDBC2};
                4'd4: c3_lane0 = {5'd23, 16'hDD17};
                4'd5: c3_lane0 = {5'd23, 16'hDC3E};
                4'd6: c3_lane0 = {5'd24, 16'hD7B8};
                4'd7: c3_lane0 = {5'd22, 16'h96E4};
                4'd8: c3_lane0 = {5'd24, 16'hBB99};
                4'd9: c3_lane0 = {5'd24, 16'hF349};
                default: c3_lane0 = 21'd0;
            endcase
        end
    endfunction

    // Only the first conv3 accept of a block is compared.  One accept is one
    // (band, icg, ocg) phase and a block has thirty-two of them, all of them
    // reading the same two entries for their lanes -- so the first is the whole
    // of the evidence and the other thirty-one would only add flops.
    //
    // `c_ocg` is stage C's, and stage C is what the MAC consumes and what
    // `m_bus` is built from, so the og the comparison uses is the og the word
    // came from.
    //
    // Cleared on `start` and on a change of `blk_idx`.  Both happen at a block
    // boundary and neither can coincide with a conv3 accept.
    reg [2:0] blk_q;
    reg       c3_seen;
    reg [4:0] cfg_ok;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blk_q <= 3'd0; c3_seen <= 1'b0; cfg_ok <= 5'd0;
        end
        else begin
            blk_q <= blk_idx;
            if (start || (blk_idx != blk_q)) c3_seen <= 1'b0;
            if (rq_accept && (c_mode == 4'd4) && !c3_seen) begin
                c3_seen <= 1'b1;
                cfg_ok[blk_idx] <= (rq_lane0_now == c3_lane0(blk_idx, c_ocg));
            end
        end
    end

    assign cfg_local_ok_o = cfg_ok;

    // ------------------------------------------------------------------
    // the same comparison, taken at the refill instead of at the accept
    // ------------------------------------------------------------------
    //
    // Slot 32 is written once a block, at `cfg_wa == 32`, and `cfg_wd` is the
    // ROM's own registered output for the entry the block's base addresses --
    // so this compares the *delivered* word against the table without asking
    // `cfg` anything.  `blk_idx` is the block the stream belongs to, which the
    // boundary has already advanced to.
    reg [4:0] stream_ok;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) stream_ok <= 5'd0;
        else if (cfg_we && (cfg_wa == 6'd32))
            stream_ok[blk_idx] <= ({cfg_wd[52:48], cfg_wd[47:32]}
                                   == c3_lane0(blk_idx, 1'b0));
    end

    assign stream_ok_o = stream_ok;

    // ------------------------------------------------------------------
    // and what the refill actually put there
    // ------------------------------------------------------------------
    //
    // Latched once, on block 0's slot 32, and kept.  A run that hangs before that
    // block's conv3 still has this, which is why it is here rather than in the
    // `rq_accept` probe it replaces: the board hung on the second build and the
    // accept never came, so the one measurement that would have named the value
    // came back zero while the refill evidence was already in hand.
    wire [20:0] refill_now = {cfg_wd[52:48], cfg_wd[47:32]};

    reg [20:0] refill_c3;
    reg        refill_seen;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refill_c3 <= 21'd0; refill_seen <= 1'b0;
        end
        else if (cfg_we && (cfg_wa == 6'd32) && (blk_idx == 3'd0)) begin
            refill_c3   <= refill_now;
            refill_seen <= 1'b1;
        end
    end

    assign refill_c3_o   = refill_c3;
    assign refill_seen_o = refill_seen;
    assign refill_ok_o   = refill_seen && (refill_c3 == c3_lane0(3'd0, 1'b0));



    // One whole-field array read; slicing that same array a second time can
    // trigger Efinity's MergeCompatiblePorts width assertion.
    function [ACC_W-1:0] extend_cfg_bias;
        input [CFG_BIAS_W-1:0] value;
        begin extend_cfg_bias={{(ACC_W-CFG_BIAS_W){value[CFG_BIAS_W-1]}},value}; end
    endfunction
    reg [OC_LANES*ACC_W-1:0] bias_r;
    integer bi;
    always @* begin
        for (bi = 0; bi < OC_LANES; bi = bi + 1)
            bias_r[bi*ACC_W +: ACC_W] = extend_cfg_bias(cfg_bias_now[bi*CFG_BIAS_W +: CFG_BIAS_W]);
    end

    localparam LANES  = 128;
    localparam OC_MAX = 16;
    wire [RQ_LANES*OUT_W-1:0] y;
    wire [RQ_LANES-1:0]       y_valid;
    wire rq_ready, rq_ready_n, mac_overrun;

    // **The bank, through the adapter that keeps this interface.**  D3c6-a3:
    // the array becomes the runtime-repartitioned one and nothing else in this
    // module moves -- not the schedule, not the stores, not the metadata.  The
    // gates that closed this engine are the ones that say whether the swap is
    // what it claims to be, and they compare cycles as well as values.
    // **The mapping always happens; only where the operands go changes.**  The
    // lane permutation is pure wiring, so `d64_mac128_compat_map` runs in both
    // configurations and the bank is what moves behind the seam.
    wire [LANES*ACT_W-1:0]  mac_act;
    wire [LANES*WGT_W-1:0]  mac_wgt;
    wire [LANES*ACC_W-1:0]  mac_bias;
    wire [OC_MAX*M_W-1:0]   mac_m;
    wire [OC_MAX*SHIFT_W-1:0] mac_sh;
    wire [1:0]              mac_mode;
    wire [OUT_W-1:0]        mac_qmin, mac_qmax;
    wire [LANES*ACC_W-1:0]  mac_acc, mac_snap;

    d64_mac128_compat_map #(
        .IC_PAR(IC_PAR), .OC_LANES(OC_LANES), .PIX_PAR(1),
        .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W),
        .SHIFT_W(SHIFT_W), .OUT_W(OUT_W), .RQ_LANES(RQ_LANES),
        .QMIN(0), .QMAX(1023), .LANES(LANES), .OC_MAX(OC_MAX)
    ) u_map (
        .act(act), .wgt(w_q_d), .bias(bias_r),
        .rq_m(m_bus), .rq_shift(sh_bus),
        .b_act(mac_act), .b_wgt(mac_wgt), .b_bias(mac_bias),
        .b_m(mac_m), .b_shift(mac_sh), .b_mode(mac_mode),
        .b_qmin(mac_qmin), .b_qmax(mac_qmax),
        .b_acc(mac_acc), .b_snapshot(mac_snap),
        .acc_o(), .rq_snap_o()
    );

    assign pe_req_slot_valid = c_live && !cfg_protocol_fault;
    assign pe_req_act        = mac_act;
    assign pe_req_wgt        = mac_wgt;
    assign pe_req_pix_start  = b_pstart && !cfg_protocol_fault;
    assign pe_req_bias       = mac_bias;
    assign pe_req_rq_start   = rq_accept;
    assign pe_req_rq_m       = mac_m;
    assign pe_req_rq_shift   = mac_sh;
    assign pe_req_rq_zero    = zero_bus;
    assign pe_req_rq_qmin    = mac_qmin;
    assign pe_req_rq_qmax    = mac_qmax;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_pe
            assign rq_ready     = pe_rsp_rq_ready;
            assign rq_ready_n   = pe_rsp_rq_ready_n;
            assign y            = pe_rsp_y;
            assign y_valid      = pe_rsp_y_valid;
            assign mac_overrun  = pe_rsp_rq_overrun;
            assign mac_acc      = pe_rsp_acc;
            assign mac_snap     = pe_rsp_snapshot;
            // `pe_rsp_mode_viol` is the shared bank's own contract and belongs to
            // whoever drives `mode`, which is the owner and not this engine.
        end else begin : g_local_pe
            d64_mac128 #(.ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W),
                         .SHIFT_W(SHIFT_W), .OUT_W(OUT_W), .RQ_LANES(RQ_LANES),
                         .LANES(LANES), .OC_MAX(OC_MAX),
                         .RQ_LOOKAHEAD(RQ_LOOKAHEAD)) u_bank (
                .clk(clk), .rst_n(rst_n), .mode(mac_mode),
                .slot_valid(pe_req_slot_valid), .act(mac_act), .wgt(mac_wgt),
                .pix_start(pe_req_pix_start), .bias(mac_bias),
                .rq_start(rq_accept), .rq_ready(rq_ready),
                .rq_ready_n(rq_ready_n), .rq_m(mac_m), .rq_shift(mac_sh),
                .rq_zero(zero_bus), .rq_qmin(mac_qmin), .rq_qmax(mac_qmax),
                .y(y), .y_valid(y_valid), .rq_busy(), .rq_done(),
                .rq_overrun(mac_overrun), .mode_viol(),
                .acc_o(mac_acc), .rq_snap_o(mac_snap)
            );
        end
    endgenerate

    // The accept is stage C's, because stage C is what the MAC consumes.  The
    // requant's passes therefore come out one cycle later than they did in
    // D3c5-a, and everything downstream of them moves with them.
    assign rq_accept = c_plast & rq_ready & !cfg_protocol_fault;

    // ------------------------------------------------------------------
    // P7d
    // ------------------------------------------------------------------
    // **Written as concatenations, not as hex.**  The first version spelled the
    // activation out as `22'h1ab1a3`, which is what {0x1AB, 0x1A3} would be if the
    // two fields were twelve bits wide; at eleven they are `(0x1AB << 11) | 0x1A3`
    // = 0x0D59A3, and the probe read a zero on a design the model says is correct.
    // A constant whose fields are named cannot slip by a bit width.
    localparam [7:0]  P7_WADDR = 8'd56;
    localparam [63:0] P7_WQ    = 64'h52042af010e22b0b;
    localparam [21:0] P7_ACT   = {11'h1ab, 11'h1a3};
    localparam [63:0] P7_ACC   = {32'h0000ed58, 32'hffffcfe2};

    // `w_q_d[7:0]` is lane 0's first byte, and the four bytes of a lane are
    // contiguous, so a lane is a byte group and not a bit slice.
    wire [63:0] p7_wq_now = {w_q_d[1016 +: 8], w_q_d[512 +: 8], w_q_d[504 +: 8],
                             w_q_d[120 +: 8], w_q_d[64 +: 8], w_q_d[56 +: 8],
                             w_q_d[8 +: 8], w_q_d[0 +: 8]};
    wire [21:0] p7_act_now = {act[21:11], act[10:0]};
    wire [63:0] p7_acc_now = mac_acc[63:0];

    reg  p7_slot_seen, p7_acc_seen;
    reg [3:0] p7_ok;
    // **The phase's starting value, before a single product.**  The two
    // accumulators already travelling say the conv1 phase ends somewhere the
    // model does not, and their magnitudes say something the individual operands
    // do not: the board's is ~1.47M, which is what a *full* eight-lane sum looks
    // like, while the model's is ~12k, which is what one slot looks like.  That
    // is the shape of an accumulator that was not initialised at `pix_start`
    // rather than of a multiply that took a wrong sign -- and the sign
    // hypothesis is dead numerically anyway: its error would be a multiple of
    // 2^19 and 1,482,806 is not.
    //
    // So this captures `mac_acc` one cycle after the first `pix_start` of the
    // run, which is block 0's conv1 by construction and is the bias alone.
    reg         p7_bstart_q;
    reg         p7_ld_go;
    reg         p7_bias_seen;
    reg [63:0]  p7_bias_v;
    reg [63:0]  p7_br_v;
    reg [11:0]  p7_ctrl_v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p7_bstart_q <= 1'b0; p7_ld_go <= 1'b0; p7_bias_seen <= 1'b0;
            p7_bias_v <= 64'd0; p7_br_v <= 64'd0;
            p7_ctrl_v <= 12'd0;
            // `p7_cfg0_v`/`p7_cfg1_v` are **not** reset here.  They are owned by the
            // loader probe's block, which has the reset; two blocks writing one
            // register is what `tools/check_double_drivers.py` exists to catch.
        end
        else begin
            p7_bstart_q <= b_pstart;
            // **The load edge, not the cycle after it.**  `b_pstart` is the cycle
            // the bank takes `mac_bias`, so everything captured in this arm is the
            // value that load actually used.  The previous version registered
            // `b_pstart` first and read `bias_r` off the registered copy, so it
            // explained cycle N's load with cycle N+1's control.  Control that is
            // stable across that boundary makes the two readings equal and the
            // error invisible -- which is exactly what a model would have said, and
            // the reason the model's agreement was not evidence about this.
            p7_ld_go <= b_pstart && !p7_bias_seen;
            if (b_pstart && !p7_bias_seen) begin
                p7_bias_seen <= 1'b1;
                // `bias_r`, the register the accumulator was loaded from.  Lane 0
                // and lane 1 are the two words conv1's bias is taken from.
                p7_br_v      <= bias_r[63:0];
                // **`mac_bias` is not shipped from here any more.**  It is
                // `bias_r` after the compat map but still *before* the owner mux,
                // and the value that matters is the one the bank takes.  `p7_pb`
                // is captured in `d64_encoder_trunk` now, on the muxed
                // `pe_bias` at the bank's own load edge.
                // `{b_mode[3:0], b_ocg, abase[5:0], 1'b0}`.  `cfg_base` returns 0
                // for mode 2 whatever `b_ocg` is, so `12'h200` is what conv1 of
                // block 0 must read -- and if it does, the address is right and
                // whatever is wrong is the array's contents, not the read of it.
                p7_ctrl_v    <= {b_mode, b_ocg, abase[5:0], 1'b0};
                // **What the loader put in, not what the array returns.**  The
                // first version read `cfg[0]` and `cfg[1]` directly and the
                // mapper took a SIGSEGV on it -- a second read port on a
                // 64-bit array is not something this flow has been asked for
                // before.  Watching `cfg_we`/`cfg_wa`/`cfg_wd` answers the same
                // question without touching the array: these are the two words
                // the loader wrote into the entries conv1's bias comes from.
            end
            // **The accumulator one cycle later, from the same event.**
            // `p7_ld_go` is the load edge delayed by one and fires exactly once, so
            // `p7_bias_v` is always the state that this edge's `bias_r` and `p7_ctrl`
            // produced.  The three travel as one observation rather than as three
            // readings a host has to hope belong together.
            if (p7_ld_go) p7_bias_v <= mac_acc[63:0];
        end
    end

    // **The values, not only the verdicts.**  `p7_ok[3] = 0` on the board says the
    // conv1 phase accumulator is not the model's, and that is all it says: a bit
    // cannot be told from a bit that never fired, and it carries no information
    // about *how* the number is wrong.  So the two accumulators travel -- the one
    // at the end of the phase, and the one right after the first slot, whose
    // operands are the three the other bits already pin.  A difference between
    // them localises the divergence to the first slot or to what follows it.
    //
    // `seen` travels with them for the same reason `seen` travels with the buffer
    // masks: a sticky that never armed and a sticky that armed and failed are the
    // same zero.
    reg [63:0] p7_acc_v, p7_fs_v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p7_slot_seen <= 1'b0; p7_acc_seen <= 1'b0; p7_ok <= 4'd0;
            p7_acc_v <= 64'd0; p7_fs_v <= 64'd0;
        end
        else begin
            if (!p7_slot_seen && c_live && (c_mode == 4'd2)) begin
                p7_slot_seen <= 1'b1;
                p7_fs_v  <= mac_acc[63:0];      // after the first slot
                p7_ok[0] <= (w_ra_r == P7_WADDR);
                p7_ok[1] <= (p7_wq_now == P7_WQ) && (mac_wgt[7:0] == 8'h0b);
                p7_ok[2] <= (p7_act_now == P7_ACT) && (mac_act[10:0] == 11'h1a3);
            end
            if (!p7_acc_seen && rq_accept && (c_mode == 4'd2)) begin
                p7_acc_seen <= 1'b1;
                p7_acc_v <= mac_acc[63:0];      // at the end of the phase
                p7_ok[3] <= (p7_acc_now == P7_ACC);
            end
        end
    end

    assign p7_ok_o   = p7_ok;
    assign p7_seen_o = {p7_acc_seen, p7_slot_seen};
    assign p7_acc_o  = p7_acc_v;
    assign p7_fs_o   = p7_fs_v;
    assign p7_bias_seen_o = p7_bias_seen;
    assign p7_bias_o = p7_bias_v;
    assign p7_br_o   = p7_br_v;
    assign p7_cfg0_o = p7_cfg0_v;
    assign p7_cfg1_o = p7_cfg1_v;
    assign p7_cfg_seen_o = {p7_cfg1_seen, p7_cfg0_seen};
    assign p7_ctrl_o = p7_ctrl_v;

    // ------------------------------------------------------------------
    // the write-back
    // ------------------------------------------------------------------
    //
    // A and B are registered once (the one-cycle disagreement above); the trunk
    // write is not, because conv3's is already covered.

    // conv1 and conv2 write eight bits; conv3 writes ten.
    reg [O8_W-1:0] y8 [0:RQ_LANES-1];
    integer yi;
    always @* begin
        for (yi = 0; yi < RQ_LANES; yi = yi + 1)
            y8[yi] = (y[yi*OUT_W +: OUT_W] > 255) ? 8'hFF
                                                 : y[yi*OUT_W +: O8_W];
    end

    wire [63:0] pack8 = {y8[7], y8[6], y8[5], y8[4], y8[3], y8[2], y8[1], y8[0]};
    wire [79:0] pack10 = {y[7*OUT_W +: OUT_W], y[6*OUT_W +: OUT_W],
                          y[5*OUT_W +: OUT_W], y[4*OUT_W +: OUT_W],
                          y[3*OUT_W +: OUT_W], y[2*OUT_W +: OUT_W],
                          y[1*OUT_W +: OUT_W], y[0*OUT_W +: OUT_W]};


    // A and B: one registered transaction, address and data together, so the two
    // words of a pixel stay paired with the passes that fill them.
    reg [9:0]  a_wa_r, b_wa_r;
    reg        a_wb_r, b_wb_r;
    reg        a_we_r, b_we_r;

    // **Three stages now, not two.**  The reasoning at the top of this file said
    // one and the simulation said two: the scheduler raises the enable at T+2 and
    // the requant's first pass does not appear until T+3 *after the accept edge*,
    // which is itself the end of T+2 -- so the pass was at T+4 in the scheduler's
    // frame, two cycles after the enable and not one.  The trunk write, whose
    // first accept was two cycles earlier, needed no delay at all.
    //
    // D3c5-b1 moved the accept to stage C, so both of those numbers move by one
    // and the constant changes from 2 to 3.  It is a constant, not a derivation:
    // the count is whatever makes `a_we` land on the cycle `pack8` is valid, and
    // the only thing that establishes it is the testbench saying every word
    // matched.
    reg [2:0] awd, bwd;
    reg [9:0] awa1, bwa1, awa2, bwa2, awa3, bwa3;
    reg       awb1, bwb1, awb2, bwb2, awb3, bwb3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awd <= 3'd0; bwd <= 3'd0;
            awa1 <= 10'd0; bwa1 <= 10'd0; awb1 <= 1'b0; bwb1 <= 1'b0;
            awa2 <= 10'd0; bwa2 <= 10'd0; awb2 <= 1'b0; bwb2 <= 1'b0;
            awa3 <= 10'd0; bwa3 <= 10'd0; awb3 <= 1'b0; bwb3 <= 1'b0;
        end
        else begin
            awd <= {awd[1:0], s_a_wr_en};
            bwd <= {bwd[1:0], s_b_wr_en};
            awa1 <= s_a_wr_addr; awb1 <= s_a_wr_bank[0];
            bwa1 <= s_b_wr_addr; bwb1 <= s_b_wr_bank[0];
            awa2 <= awa1; awb2 <= awb1;
            bwa2 <= bwa1; bwb2 <= bwb1;
            awa3 <= awa2; awb3 <= awb2;
            bwa3 <= bwa2; bwb3 <= bwb2;
        end
    end

    assign a_we = awd[2];
    assign b_we = bwd[2];
    assign a_wa = {awa3, awb3};
    assign b_wa = {bwa3, bwb3};
    assign a_wd = pack8;
    assign b_wd = pack8;

    assign a_wr_data_o = pack8;
    assign b_wr_data_o = pack8;

    // ------------------------------------------------------------------
    // conv3's write-back
    // ------------------------------------------------------------------
    //
    // conv1 and conv2's writes are registered; conv3's is not, because its first
    // accept is two cycles earlier and its four words and four passes land on the
    // same four cycles.  Nothing is staged, and the register dump said so after
    // two wrong guesses in the other direction.

    // **Why not a tag.**  The first version carried a one-bit `is conv3` tag
    // through a two-deep FIFO, pushed at the accept and popped with the second
    // pass -- which cleared it after the *first*.  Every conv3 pass after the
    // first was then dropped, `tstg` stayed zero, and the trunk write came out as
    // `skip - zero` instead of `skip + branch - zero`: a plausible number, wrong,
    // and with the skip operand perfectly correct and staring at you from the
    // register dump.  A counter that arms once and runs for exactly four passes
    // has no edge to get wrong.

    // The trunk write carries the residual add, and its skip operand is the X
    // word conv3 read a few cycles earlier -- held by bank, because the read and
    // the write are in bank order but three cycles apart.
    //
    // **Capture and write move together.**  `skip_r[bank]` has to hold pixel
    // N-1's word on the cycle the write for pixel N-1 reads it, and the next
    // capture into that bank is four cycles after this one -- so the capture is
    // three cycles ahead of the write and no more.  D3c5-b1's stage C moves the
    // write one cycle later; moving the capture with it keeps the gap at three,
    // and moving only one of them would have put the write on the cycle the next
    // pixel's word lands.  That is the same shape as the `w_ra_r` omission, in
    // the other direction.
    reg [TRUNK_BANK_W-1:0] xb_d, xb_d2;
    reg [79:0] skip_r [0:(1<<TRUNK_BANK_W)-1];
    reg        trunk_rd_d, trunk_rd_d2;
    integer si;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xb_d <= {TRUNK_BANK_W{1'b0}};
            xb_d2 <= {TRUNK_BANK_W{1'b0}};
            trunk_rd_d <= 1'b0;
            trunk_rd_d2 <= 1'b0;
            for (si = 0; si < (1<<TRUNK_BANK_W); si = si + 1) skip_r[si] <= 80'd0;
        end
        else begin
            xb_d  <= s_trunk_rd_bank;
            xb_d2 <= xb_d;
            trunk_rd_d  <= s_trunk_rd_en;
            trunk_rd_d2 <= trunk_rd_d;
            // **The read enable, not `cur_live`.**  `cur_live` falls when the
            // scheduler enters its drain, and the last pixel of every band has
            // its fourth read in flight at exactly that moment -- so the capture
            // was skipped once per 512 pixels and those words carried the
            // previous pixel's skip.  Twenty-seven values a block, all of them
            // word 3 of a band's last pixel, and every one of them a few counts
            // off.  `trunk_rd_en` marks the read that actually happened, which is
            // what the data on the wire is the answer to.
            if (trunk_rd_d2 && (b_mode == 4'd4)) skip_r[xb_d2] <= x_q_d;
        end
    end

    // One word of the residual add: eight channels of ten bits.
    // **No staging.**  conv3's write and its requant pass land on the same cycle
    // -- which the diagnostic above says directly, since not one of the 16,384
    // trunk writes happens without `y_valid` asserted.  Registering the pass to
    // "line it up" therefore moves it a cycle *past* the write that wants it, and
    // the four-entry staging register this replaces was doing exactly that.
    // One word of the residual add: eight channels of ten bits.
    //
    // **One stage now, and it used to need none.**  D3c5-a found that conv3's
    // write and its requant pass land on the same cycle, and proved it: not one
    // of the 16,384 trunk writes happened without `y_valid` asserted, and
    // registering the pass to "line it up" moved it a cycle *past* the write that
    // wanted it.  D3c5-b1's stage C moves the pass one cycle later, so the
    // schedule's write enable is now the thing that is early and it has to be
    // held back to meet it -- the same constant moved in the other direction.
    //
    // The skip capture moves with the write and not with the read, which is the
    // part worth being careful about: `skip_r[bank]` has to hold pixel N-1's word
    // on the cycle the write for pixel N-1 reads it, so capture and write are one
    // relationship, and moving one alone breaks it.
    wire [79:0] br_word = pack10;
    // **Per channel.**  The skip word is eight channels of ten bits, exactly like
    // the branch, and handing the whole 80-bit word to a 10-bit port gives every
    // channel channel 0's value -- which is a real number, in range, and wrong.
    // Eight channels of one pixel share a row and a column, so the difference is
    // a few counts either way and the picture looks like a picture.
    wire [79:0] skip_word = skip_r[tw_bank];
    wire [79:0] add_word;
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : g_add
            d64_residual_add #(.DW(10), .QMIN(0), .QMAX(1023), .ZERO(ZERO_T), .RUNTIME_PROFILE(RUNTIME_PROFILE))
            u_add (
                .profile_zero(trunk_zero_live),
                .skip(skip_word[gi*10 +: 10]), .branch(br_word[gi*10 +: 10]),
                .y(add_word[gi*10 +: 10])
            );
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tw_en <= 1'b0; tw_addr <= 12'd0;
            tw_bank <= {TRUNK_BANK_W{1'b0}};
        end
        else begin
            tw_en   <= s_trunk_wr_en;
            tw_addr <= s_trunk_wr_addr;
            tw_bank <= s_trunk_wr_bank;
        end
    end

    // **The write count is the design's own `t_writes`, not a second counter.**
    // `t_writes` has been incremented on `tw_en` since D3c5 and was never routed
    // out -- `.t_writes()` is unconnected in d64_h0_block.  A duplicate written
    // beside it read zero on the board while tr_write_beats was 81,920, so the
    // duplicate is what was wrong, not `tw_en`.  Driving the port from the
    // original removes the second copy and makes the reading mean something.
    //
    // `add_diff` stays a counter of its own because nothing else counts it.
    // **Two counters, and the pair is the point.**  `add_diff` alone cannot
    // distinguish "the residual add changed nothing" from "the engine never
    // wrote": both leave it at zero.  `tw_cnt` counts `tw_en` itself, so the two
    // together do.  Both are terms of board_gate_pass, which drives led[3], so
    // both have a pin for a consumer -- `t_writes` and `tw_no_yv` were folded out
    // of an earlier build for lacking one, and both reported zeroes that were
    // tie-offs rather than measurements.
    reg [31:0] add_diff, tw_cnt, no_yv, zero_br;
    wire [79:0] ZERO_WORD = {8{trunk_zero_live}};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            add_diff <= 32'd0; tw_cnt <= 32'd0; no_yv <= 32'd0; zero_br <= 32'd0;
        end
        else if (tw_en) begin
            tw_cnt <= tw_cnt + 32'd1;
            if (y_valid != {(RQ_LANES){1'b1}}) no_yv <= no_yv + 32'd1;
            if (br_word == ZERO_WORD) zero_br <= zero_br + 32'd1;
            if (add_word !== skip_word) add_diff <= add_diff + 32'd1;
        end
    end

    assign t_wr_data_o = add_word;
    assign tw_cnt_o  = tw_cnt;
    assign add_cnt_o  = t_writes;
    assign add_diff_o = add_diff;
    assign no_yv_o    = no_yv;
    assign zero_br_o  = zero_br;
    assign t_wr_en_o   = tw_en;
    assign t_wr_addr_o = tw_addr;
    assign t_wr_bank_o = tw_bank;
    assign t_wr_last_o = tw_en && (tw_addr == (ROWS * COLS - 1))
                      && (tw_bank == {TRUNK_BANK_W{1'b1}});

    // ------------------------------------------------------------------
    // counters
    // ------------------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_writes <= 32'd0; b_writes <= 32'd0; t_writes <= 32'd0;
            rq_accepts <= 32'd0; rq_overruns <= 32'd0;
        end
        else begin
            if (awd[2]) a_writes <= a_writes + 32'd1;
            if (bwd[2]) b_writes <= b_writes + 32'd1;
            if (tw_en) t_writes <= t_writes + 32'd1;
            if (rq_accept) rq_accepts <= rq_accepts + 32'd1;
            if (mac_overrun) rq_overruns <= rq_overruns + 32'd1;
        end
    end

    // ------------------------------------------------------------------
    // D3c6-e7: which stage, and which 128 writes, first differs
    // ------------------------------------------------------------------
    //
    // Tapped here because this is where the three streams exist: the A and B
    // ring writes and the requant's registered trunk write are all in this
    // module, and the hierarchy above knows nothing about them.  The window
    // (`obs_en`) comes from above because the block index and the redo counter
    // are what define it, and they live two levels up.
    //
    // **`br_word` and not `add_word`.**  The reference is the requant's own
    // output, before the residual add; the word that reaches DDR is the sum,
    // and the difference between the two is the whole of C3's question.
    wire [272:0] obs_chunk_w;

    // **The window a clean run produces, derived from the shape.**  These three
    // numbers were the 64-row literals (8,192 / 16,384 / 256) written into the
    // tie-off below *and* into the board top's acceptance terms, so a build
    // without the monitor reported a 64-row window at every shape -- and the
    // board top, which compares against its own `OBS_*`, read that as a
    // measurement rather than as a tie-off.  The monitor's window is
    // `(2+2+4) * ROWS * COLS / 128` writes over `4 * ROWS` chunks.
    localparam [13:0] OBS_CNT_AB_Q = 2 * ROWS * COLS;   // conv1, conv2: two words a pixel
    localparam [14:0] OBS_CNT_C_Q  = 4 * ROWS * COLS;   // conv3: four
    localparam [8:0]  OBS_CHUNKS_Q = 4 * ROWS;          // 128 writes a chunk
    generate
    if (C2_DIAG == 0) begin : g_mon
    d64_stage_chunk_crc #(.CHUNK_ROM_INIT(CHUNK_ROM_INIT), .ROWS(ROWS)) u_stage_obs (
        .clk(clk), .rst_n(rst_n), .obs_en(obs_en),
        .a_we(a_we), .a_wa(a_wa), .a_wd(a_wd),
        .b_we(b_we), .b_wa(b_wa), .b_wd(b_wd),
        .c_en(tw_en), .c_addr(tw_addr), .c_bank(tw_bank), .c_word(br_word),
        .cnt_a(obs_chunk_w[13:0]),   .cnt_b(obs_chunk_w[27:14]),
        .cnt_c(obs_chunk_w[42:28]),
        .bad_seen(obs_chunk_w[45:43]),
        .bad_chunk_a(obs_chunk_w[53:46]),
        .bad_chunk_b(obs_chunk_w[61:54]),
        .bad_chunk_c(obs_chunk_w[69:62]),
        .bad_exp_a(obs_chunk_w[101:70]),  .bad_obs_a(obs_chunk_w[133:102]),
        .bad_exp_b(obs_chunk_w[165:134]), .bad_obs_b(obs_chunk_w[197:166]),
        .bad_exp_c(obs_chunk_w[229:198]), .bad_obs_c(obs_chunk_w[261:230]),
        .chunks_done(obs_chunk_w[270:262]),
        .conflict(obs_chunk_w[271]), .overflow(obs_chunk_w[272]),
        .pipe_busy()
    );
    end else begin : g_none
        // The monitor is not in this build.  Its result vector is tied to the
        // values a clean run would produce -- **derived from the shape, not
        // typed** -- and the board top compares against its own `OBS_*` terms,
        // which are the same derivation, so a tie-off and a measurement agree
        // at every shape rather than only at the square.
        assign obs_chunk_w = {3'd0, OBS_CHUNKS_Q,
                              32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0,
                              8'd0, 8'd0, 8'd0, 3'd0,
                              OBS_CNT_C_Q, OBS_CNT_AB_Q, OBS_CNT_AB_Q};
    end
    endgenerate
    assign obs_chunk = obs_chunk_w;

    // ------------------------------------------------------------------
    // D3c6-e8: C2, word by word
    // ------------------------------------------------------------------
    //
    // The chunk monitor localised the board's first divergence to C2's first
    // 128 writes.  This says *which* write, and what C2 read when it made it:
    // the A request address, the word the ring returned, the weight address it
    // used, and the first accept's requant parameters.
    //
    // **The taps are the real edges.**  The request is `s_a_rd_en` with
    // `a_ra`/`w_ra_r`/`a_tap`/`a_icg`/`a_pstart`/`a_plast` beside it; the
    // consumption is `c_live && c_mode == M_C2` with `a_q_d` on that cycle; the
    // write is `b_we`.  The reference was recorded through exactly these, at the
    // same two-cycle request-to-consumption distance, so the comparison is
    // against the reconstruction and not against a second opinion about where
    // the edges are.
    generate
    if (C2_DIAG == 1) begin : g_c2
        d64_c2_diag #(.A_ROM_INIT(A_ROM_INIT), .B_ROM_INIT(B_ROM_INIT)) u_c2 (
            .clk(clk), .rst_n(rst_n), .en(obs_en),
            .a_req(s_a_rd_en), .a_req_addr(a_ra), .a_req_waddr(w_ra_r),
            // **`a_icg[0]` is the truncation written down, not a new one.**
            // A four-bit connection to a one-bit port is a truncation, which the
            // elaborator does correctly -- the probe's `a_req_icg` has been handed
            // `a_icg[0]` on every run.  The port is one bit because it is an edge
            // flag between `a_req_first` and `a_req_last` (`d64_c2_diag.v:56-59`),
            // and widening it would change `s1_tag`'s width and the probe's ROM
            // address space -- a design change, not a width repair.  Note that
            // `cur_icg` reaches 3 in a conv1 phase, so the tag records the PARITY
            // of the IC group, not the group; that is pre-existing and unchanged.
            .a_req_tap(a_tap), .a_req_icg(a_icg[0]),
            .a_req_first(a_pstart), .a_req_last(a_plast),
            .c_live(c_live), .c_mode(c_mode), .a_q_d(a_q_d),
            .b_we(b_we), .b_wa(b_wa), .b_wd(b_wd),
            .c2_accept(c_mode == 4'd3 && rq_accept),
            .rq_m0(pe_req_rq_m[15:0]), .rq_s0(pe_req_rq_shift[4:0]),
            .c2(c2_chunk), .c2_dbg(c2_dbg));
    end else begin : g_noc2
        assign c2_chunk = 415'd0;
        assign c2_dbg = 3'd0;
    end
    endgenerate

    // External bank consumes weights only with its valid slot. Sampling the
    // common response every clock, with no owner-specific mask/reset, allows
    // equivalent raw response bits to share FFs across mutually exclusive engines.
    // The standalone backend retains its original reset and capture contract.
    generate if(EXTERNAL_PE!=0)begin:g_shared_weight_capture
      always @(posedge clk) w_q_d <= w_q;
    end endgenerate

    // synthesis translate_off
    wire [128*ACT_W-1:0] act_registered_ghost;
    genvar ga;generate for(ga=0;ga<128;ga=ga+1)begin:g_act_ghost
      assign act_registered_ghost[ga*ACT_W+:ACT_W]=act_q[(ga%IC_PAR)*ACT_W+:ACT_W];
    end endgenerate
    // synthesis translate_on
endmodule
