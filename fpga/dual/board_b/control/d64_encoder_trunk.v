`timescale 1ns/1ps

// D3c6-e5 -- one PE, one weight store, two clients: the encoder and the trunk.
//
// D3c6-e3 made the five residual blocks a client and D3c6-e4 made the encoder
// one.  Both expose the same seam.  This is the module that owns the thing
// they are clients of:
//
//     d64_encoder      (EXTERNAL_PE = 1) --+
//                                          +-- owner mux --> ONE d64_mac128
//     d64_trunk_blocks (EXTERNAL_PE = 1) --+
//
//     d64_encoder      --+
//                        +------------------------------> ONE 247-tile weight store
//     d64_trunk_blocks --+
//
// 144 DSP and 52 RAM10 for the whole front of the network instead of twice
// that, which is what every stage from D3c6-a on has been arranging.
//
// The owner FSM, and the two conditions that are not the same condition
// ---------------------------------------------------------------------
//
//     IDLE -> ENCODER -> TRUNK -> DONE
//
// The handover is **not** "down1 and residual 0 are both mode 0, so the array
// does not care".  Two separate things have to hold, and the second is not
// implied by the first:
//
//   owner-safe   the encoder has finished *and* its last trunk-A word has been
//                accepted.  A block boundary built on `done` alone starts the
//                trunk while the final word is still in the pipeline -- the
//                defect D3c5-b1 found by counting 16,383 writes instead of
//                16,384 -- and here the trunk reads exactly that word.
//   mode-safe    `d64_encoder`'s own `mac_mode_safe`: no slot offered, no
//                requant busy, both layer clients drained.  A shape may only
//                change while nothing is in flight under the old one.
//
// Waiting on both is what the FSM does.  `owner_viol` counts the case that can
// still happen with the FSM correct: **a client presenting a slot while the
// other one owns the array**.  That is the real hazard of a shared array and
// it is not prevented by the handover condition -- it would be prevented by
// each client's own drain being right -- so it is counted and not corrected.
//
// The weight image
// ----------------
//
// The two clients do not read a tile the same way, and `tools/d64_network_image.py`
// is where that is written down: tiles 0..53 are the encoder's readers' byte
// order, tiles 54..183 the compat map's, and the ranges are disjoint so one
// store can hold both.  Nothing about that is decided here -- this module takes
// whatever image it is given -- but a load that used the wrong one would come
// out as a picture and not as a failure, which is why the generator checks it.
//
// The weight store, and who owns its one port
// -------------------------------------------
//
// **`memory/d64_weight_store.v` has one read port, and this module is not its
// owner once `EXTERNAL_WSTORE` is 1.**  At 0 (`u_w`, below) the owner is this
// module and its two clients -- the encoder's engine and the five trunk blocks
// -- are muxed onto the port by `sel_enc`, which the owner FSM above may move
// only at a drained boundary.  At 1 `u_w` is **not instantiated** and the same
// mux leaves the module as `ws_req_rd_en` / `ws_req_rd_addr` /
// `ws_rsp_rd_data`.
//
// **A mux that owns no RAM is not ownership, and the decoder is that mux.**
// `d64_decoder_wreader` presents three engines' `w_rd_en`/`w_rd_addr` on one
// port and holds no store of its own -- its header says so under the same
// heading.  So the network has **one** store, **one** driver on its `rd_en`
// and `rd_addr`, **one** driver on its `wr_*`, and **four engines
// time-multiplexed onto that one port**: this front half's two (the encoder's
// engine, tiles 0..53, and the five trunk blocks, tiles 54..183) and the
// decoder's three (ups.0, ups.1, tail; tiles 184..246, section 2 of
// `docs/F3_INTERFACES_2026-10-07.md`).  Below the store there are exactly two
// *modules* -- this one and `d64_decoder_wreader` -- and the second is a client
// only because of the seam below; the four engines are muxed twice, once
// inside each.  Every layer runs in sequence, so the port is *handed over*,
// never split; no second port is added and no arbiter is built here.
//
// Who may move the port, and the rule
// -----------------------------------
//
// The owner of the port is whoever instantiates the store -- the network top,
// because the store has to sit beside the arbiter that the front half and the
// decoder are both clients of.  It moves the port only while **both** sides are
// quiet, and each side states its own half of that in hardware:
//
//     ws_idle   here, on the front half: no store read accepted this cycle and
//               none accepted on the previous one.
//     sw_safe   in `d64_decoder_wreader`, the same two-cycle rule on its side.
//
// Neither is sufficient alone and the owner ANDs them.  This is `bank_idle`'s
// rule from `docs/F2_INTERFACES_2026-10-06.md` section 2.1, and it is the rule
// for the same reason: `d64_weight_store` answers the address it was given one
// cycle later, so a hand-over that lands between a request and its return hands
// one client another tensor's tile -- correct-looking data, wrong weights, no
// error anywhere.  A rule that is only asserted is not a rule, so the
// differential shows it load-bearing: `TB_WS_SABOTAGE` in
// `tb_d64_encoder_trunk_wstore.v` makes a client read the wrong tile and the
// differential is required to fail.
//
// **One cycle of margin, and `ws_idle` alone is not a licence to start.**  The
// store's enable is on its *output* (`rd_data = rd_en ? q : 0`) and its read is
// unconditional, so the output register still holds the outgoing client's last
// address's word on the first cycle `ws_idle` is high -- and a client that
// asserts `rd_en` on that cycle takes that word as its own answer.  So the
// owner takes the port on `ws_idle` and holds the incoming client off for one
// cycle after.  Both ends here are layer boundaries many cycles wide, which is
// why this is satisfied by construction and written down rather than
// engineered around -- the same way `d64_decoder_wreader` writes its own
// boundary rule down.
//
// The loader port belongs to the store's owner too, and therefore travels with
// the read port rather than being left behind: at `EXTERNAL_WSTORE = 1` this
// module's `wt_wr_en`/`wt_wr_addr`/`wt_wr_data` inputs are the raised store's
// write port and leave again as `ws_wr_*`.  At the board they are tied low and
// the image arrives through the store's `INIT_FILE`; a seam that handed out one
// of the store's two ports and dropped the other would be half an ownership
// statement.
//
// **Not verified here**: no arbiter, no second client and no board.  What
// `tools/run_encoder_trunk_wstore.sh` measures is that the front half is
// *unchanged* by the store being on the far side of the seam, plus the
// elaborated instance counts that say the store really left.  What it cannot
// measure is a hand-over, because nothing here performs one.
module d64_encoder_trunk #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter integer ENABLE_ENCODER = 1,
    parameter WT_INIT_FILE = "",
    parameter CFG_INIT_FILE = "",

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
    parameter integer TRUNK_BLOCK_COUNT = NB,
    parameter CFG_NB      = 64,
    parameter WBASE0      = 54,
    parameter WSTRIDE     = 26,
    parameter T_CFG_AW    = 9,
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
    // ACC25 -- the shared bank's internal accumulator width.  The board passes
    // 25; the generic module keeps 32 so every unit harness is unmoved.  See
    // `docs/RESOURCE_REFACTOR_STUDY_2026-10-03.md` section 1 for the bound that
    // makes 25 safe for this instance.
    parameter ACC_W       = 32,
    // CFG storage is independent of the bank accumulator width. Final profiles
    // use signed20 bias storage; old callers retain their 32-bit field.
    parameter CFG_BIAS_W  = 32,
    parameter integer CFG_STREAM_SRL=0,

    // **Zero keeps the bank inside this module; non-zero hands it out.**  The
    // arithmetic, the schedule, the stores and the clocks do not move either
    // way -- only which side of the seam the multipliers live on.  The network
    // has one bank because 144 DSP blocks cannot be spent twice, so the front
    // half will run with this at 1 and an arbiter driving `pe_req_*` into a bank
    // it does not own.  This is `d64_tconv_engine`'s seam (commit `e35bdb0`),
    // same shape, same widths.
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0,
    parameter integer HEAD_STORE_CREDIT_ENABLE = 0,

    // **The store's seam, and a second parameter because it is a second
    // decision.**  `EXTERNAL_PE` moves the multipliers; this moves the 247-tile
    // weight store.  A network that lifts the bank and leaves the store inside
    // still cannot give the decoder a weight read port, and raising the store
    // with the bank still inside is a legal half-step -- it is what
    // `tools/run_encoder_trunk_wstore.sh`'s differential arm builds, so that
    // the store's seam is measured with nothing else moving.
    //
    // **Zero keeps `u_w` inside this module exactly where it has always been**,
    // which is what every existing gate and the shipped board build.  At 1
    // `u_w` is not instantiated at all and the store's one read port and its
    // loader port appear at this module's top as `ws_*`; see the header for who
    // owns them and how the three clients are arbitrated.
    parameter integer EXTERNAL_WSTORE = 0,

    // **The seam's widths, and they are parameters rather than localparams
    // because a port list cannot see a localparam.**  They belong on the
    // interface anyway: every owner meets the same bank, so the widths are the
    // contract and not this module's private business.
    //
    // **`PE_ACT_W` is the contract's 11 and not this layer's width.**  The F2-a
    // work declared `pe_req_act` at `PE_LANES * ACT_W`, and `ACT_W` is a
    // property of the *layer*: one owner then presented 1152 bits and another
    // 1408, two widths for one interface, while the shared bank is built once
    // at the network's widest.  This module's stream happens to be 11 bits
    // wide, which is exactly why the mistake is invisible here and has to be
    // written down rather than remembered.
    parameter PE_LANES  = 128,
    parameter PE_OC_MAX = 16,
    parameter PE_OUT_W  = 10,
    parameter PE_ACT_W  = 11,
    parameter PE_ACC_W  = 32
) (
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire          clk_pe,
    input  wire          clk_axi,
    input  wire          arst_n,
    input  wire          start,

    // ---- the encoder's configuration load --------------------------------
    input  wire          e_cfg_wr_en,
    input  wire [5:0]    e_cfg_wr_addr,
    input  wire [63:0]   e_cfg_wr_data,

    // ---- the trunk's, a different table at a different width -------------
    input  wire          t_cfg_we,
    input  wire [T_CFG_AW-1:0] t_cfg_wa,
    input  wire [63:0]   t_cfg_wd,

    // ---- the trunk's DDR side --------------------------------------------
    output wire [6:0]    x_req_row,
    output wire          x_req_tog,
    input  wire          x_ack_tog,
    input  wire          rf_wr_en,
    input  wire [127:0]  rf_wr_data,
    output wire          rf_wr_full,
    input  wire          wf_rd_en,
    output wire [127:0]  wf_rd_data,
    output wire          wf_rd_valid,
    output wire          wf_rd_empty,
    output wire [WF_AW:0] wf_rd_level,
    input  wire          wr_done_tog,

    // ---- the composite state ---------------------------------------------
    output wire          busy,
    output wire          done,
    output wire [2:0]    owner,
    output reg  [31:0]   owner_viol,
    output wire [31:0]   wall_cycles,
    output reg  [31:0]   hand_cycles,
    output reg           out_committed,
    // ---- the re-arm's own evidence ---------------------------------------
    //
    // `frame_cnt` is how many frames this instance has begun -- the first one
    // from `O_IDLE`, every later one from the re-arm.  It is the number that
    // separates "the second frame ran" from "the second `start` was ignored",
    // which without it are indistinguishable: both leave the module sitting in
    // `O_DONE` looking exactly like a finished run.
    //
    // `rearm_early` counts a `start` the FSM could not honour -- one that
    // arrived while a frame was still in flight.  It is counted and not acted
    // on: a host on a frame timer sends `start` early, and aborting a frame
    // that is mid-drain to satisfy it is precisely the defect this stage
    // exists to prevent.
    output reg  [31:0]   frame_cnt,
    output reg  [31:0]   rearm_early,
    output wire [1:0]    pe_mode,
    // **The third and last term of the handover, and the one that was missing.**
    // `out_committed` below is the *stream* handshake -- the last trunk-A word
    // taken by the memory boundary -- and behind it there is still a write path:
    // the async FIFO, `clk_axi`, AW, 128 W beats, and the DDR controller's B
    // response.  Until that response the write is not in memory, and the trunk's
    // first act is to *read* TRUNK_A back.  AXI orders a read after a write only
    // if the master waited for the B response, so without this term the trunk can
    // overtake the encoder's last bursts and read stale DDR -- which is what the
    // board does, bimodally, with `in0_crc` either `0x343C03EE` (the encoder's
    // own write, correctly read back) or something else entirely.
    //
    // `d64_encoder_ddr_mem` has computed this since it was written and its own
    // comment says the caller must combine it: "the caller combines the two,
    // because it is the caller that owns both halves".  The caller never did.
    input  wire          trunk_store_committed,

    // ---- the trunk's evidence --------------------------------------------
    output wire [BLK_W-1:0] t_blk,
    output wire          t_src_buf,
    output wire          t_req_buf,
    output wire          t_dst_buf,
    output wire [31:0]   t_compute_sum,
    // **How many passes the PE began, exported.**  It was an internal wire with
    // no port, so the board had no way to say how many blocks the PE had run --
    // and "the PE advanced and the adapter did not" is exactly the failure that
    // number is for.  It is compared against the write beats the adapter
    // retired, which the board does have.
    output wire [31:0]   t_blk_starts,
    // True once any engine run has had a core count other than 108,120.  A total
    // cannot see one bad pass among a redo's six.
    output wire          t_eng_bad,
    output wire [31:0]   t_clean_viol,
    output wire [31:0]   t_dep_viol,
    output wire [31:0]   t_rq_overruns,
    output wire [31:0]   t_x_ovf,
    output wire [31:0]   t_x_rbf,
    output wire [31:0]   t_rf_udf,
    output wire [31:0]   t_wf_ovf,
    output wire [31:0]   t_wait_cycles,
    output wire          t_out_last,
    output wire          t_blk_clean,
    output wire [5:0]    t_clean_why,
    // The trunk's requant cfg is loaded at run time, 320 entries, and how
    // many actually landed was never reported.  If it is not 320 the cfg
    // is wrong, the requant's m/shift are wrong with it, and the residual
    // add's `branch` operand goes to the zero point -- which makes a block
    // an echo of its input, which is exactly what the board shows.
    output wire [31:0]   t_cfg_writes_o,
    // Words the trunk wrote, and how many the residual add changed.
    output wire [31:0]   t_tw_cnt,
    output wire [31:0]   t_add_cnt,
    output wire [31:0]   t_add_diff,
    output wire [31:0]   t_no_yv,
    output wire [31:0]   t_zero_br,
    // ---- D3c6-e6b: the two requant-configuration probes -------------------
    // `t_cfg_ok` is the engine's own five sticky bits -- lane 0 of the conv3
    // requant word against the image's entry for that block -- and
    // `t_rq_bank_ok` is the same comparison taken on the far side of the
    // compat map and the owner mux.  They exist because `t_zero_br` cannot
    // tell an `M` of zero from an accumulator of zero, and the board's three
    // echoing blocks are consistent with both.
    output wire [4:0]    t_cfg_ok,
    output wire cfg_protocol_fault,
    output wire          t_cfg_image_ok,
    output wire          t_cfg_selftest_done,
    input  wire          t_cfg_selftest,
    output wire [4:0]    t_rq_bank_ok,
    // The refill's own answer to the same question, and what block 0 is
    // actually holding: {og, shift, M} of lane 0 at its first conv3 accept.
    output wire [4:0]    t_stream_ok,
    output wire [20:0]   t_refill_c3,
    output wire          t_refill_seen,
    output wire          t_refill_ok,
    // D3c6-e7: the stage monitor's result, passed straight through.  The
    // window and the fold are both inside the trunk; this level only carries
    // the answer out.
    output wire [272:0]  t_obs_chunk,
    output wire [287:0]  t_c2_chunk,
    output wire [2:0]    t_c2_dbg,
    output wire [3:0]    t_p7_ok,
    output wire [1:0]    t_p7_seen,
    output wire [63:0]   t_p7_acc, t_p7_fs,
    output wire          t_p7_biaseen,
    output wire [1:0]    t_p7_cfg_seen,
    output wire [63:0]   t_p7_bias, t_p7_br,
    output wire [31:0]   t_p7_cfg0, t_p7_cfg1,
    output wire [11:0]   t_p7_ctrl,
    output wire [63:0]   t_p7_pb,       // the muxed bank input, at its load edge
    output wire          t_p7_pb_seen,
    // ---- P7r: the last slot -> the requant input -> the requant output -----
    //
    // P7d stops at the accept edge and therefore reports `acc_q`, not the
    // accumulator the requant actually consumes.  These are the three
    // observations that close that gap, all taken from the one trunk accept
    // that is block 0's conv1.  `t_p7r_seen` is one bit per record -- the two
    // T+1 records share theirs, because they are one capture -- so an empty
    // record cannot be read as a zero, and `t_p7r_state` carries the state in
    // use: {owner[2:0], pix_start, slot_valid, sacc_ne_snap, s_ok[0], run,
    // y_valid[3], y_valid[0]}.
    //
    // `t_p7r_y0` and `t_p7r_y3` are two lanes of the same requant pass, and the
    // pair is deliberate: lane 0 clamps to the low bound on this operand set,
    // so it cannot be wrong about M or the shift, while lane 3 does not clamp
    // and can be.  A clamped lane travelling on its own would look like a
    // passing check and discriminate nothing.
    output wire [24:0]   t_p7r_acc,
    output wire [15:0]   t_p7r_m,
    output wire [4:0]    t_p7r_shift,
    output wire [9:0]    t_p7r_zero,
    output wire [9:0]    t_p7r_y0,
    output wire [9:0]    t_p7r_y3,
    output wire [9:0]    t_p7r_state,
    output wire [2:0]    t_p7r_seen,    // {seen_c, seen_a, seen_a}
    output wire [3:0]    t_p7r_ok,      // {identity, y, params, acc}

    // ---- the encoder's ports, straight through ---------------------------
    input  wire          d64_valid,
    output wire          d64_ready,
    input  wire [31:0]   d64_frame_id,
    input  wire          d64_sof,
    input  wire [7:0]    d64_x0,
    input  wire [7:0]    d64_y,
    input  wire [383:0]  d64_rgb888,
    input  wire          d64_eol,
    input  wire          d64_eof,
    input  wire          wt_wr_en,
    input  wire [7:0]    wt_wr_addr,
    input  wire [1023:0] wt_wr_data,
    output wire          head_store_wr_valid,
    input  wire          head_store_wr_ready,
    input  wire          head_phase_ready,
    output wire [7:0]    head_store_wr_x,
    output wire [7:0]    head_store_wr_y,
    output wire [63:0]   head_store_wr_c8,
    output wire [31:0]   head_store_wr_frame_id,
    output wire          head_store_wr_sof,
    output wire          head_store_wr_eol,
    output wire          head_store_wr_eof,
    input  wire          head_store_commit_ready,
    output wire          head_read_start,
    input  wire          head_read_valid,
    output wire          head_read_ready,
    input  wire [7:0]    head_read_x,
    input  wire [7:0]    head_read_y,
    input  wire [63:0]   head_read_c8,
    input  wire [31:0]   head_read_frame_id,
    input  wire          head_read_sof,
    input  wire          head_read_eol,
    input  wire          head_read_eof,
    output wire          down0_store_wr_valid,
    input  wire          down0_store_wr_ready,
    output wire [7:0]    down0_store_wr_x,
    output wire [7:0]    down0_store_wr_y,
    output wire [1:0]    down0_store_wr_word,
    output wire [79:0]   down0_store_wr_data,
    output wire [31:0]   down0_store_wr_frame_id,
    output wire          down0_store_wr_sof,
    output wire          down0_store_wr_eol,
    output wire          down0_store_wr_eof,
    input  wire          down0_store_commit_ready,
    output wire          down0_read_start,
    input  wire          down0_read_valid,
    output wire          down0_read_ready,
    input  wire [7:0]    down0_read_x,
    input  wire [7:0]    down0_read_y,
    input  wire          down0_read_icg,
    input  wire [63:0]   down0_read_c8,
    input  wire [31:0]   down0_read_frame_id,
    input  wire          down0_read_sof,
    input  wire          down0_read_eol,
    input  wire          down0_read_eof,
    output wire          out_valid,
    input  wire          out_ready,
    output wire [7:0]    out_x,
    output wire [7:0]    out_y,
    output wire [1:0]    out_word,
    output wire [79:0]   out_data,
    output wire [31:0]   out_frame_id,
    output wire          out_sof,
    output wire          out_eol,
    output wire          out_eof,
    output wire          enc_busy,
    output wire          enc_done,
    // P6i: the harness's verdict on whether this block's input was stale, and
    // the redo's own evidence.
    input  wire          blk_redo,
    output wire [2:0]    t_redo_cnt,
    output wire          t_redo_seen,
    output wire          t_redo_full,
    output wire          t_redo_bound,

    output wire [3:0]    enc_state,
    output wire [31:0]   bank_mode_viol,
    output wire  [31:0]   layer_mode_viol,
    output wire  [31:0]   frame_id_violations,
    output wire  [31:0]   head_store_overflow,
    output wire [31:0]   head_mac_slots,
    output wire [31:0]   down_mac_slots,
    output wire [31:0]   head_rq_overrun,
    output wire [31:0]   down_rq_overrun,
    output wire [31:0]   head_rq_wait,
    output wire [31:0]   down_rq_wait,
    output wire [31:0]   down_read_before_ready,
    output wire  [31:0]   head_layer_cycles,
    output wire  [31:0]   down0_layer_cycles,
    output wire  [31:0]   down1_layer_cycles,
    output wire  [31:0]   head_compute_cycles,
    output wire  [31:0]   down0_compute_cycles,
    output wire  [31:0]   down1_compute_cycles,
    output wire  [31:0]   encoder_compute_cycles,
    output wire  [31:0]   encoder_store_wait_cycles,
    output wire  [31:0]   encoder_wall_cycles,
    output wire [31:0]   head_accepted_beats,
    output wire [31:0]   head_accepted_rows,
    output wire [31:0]   head_marker_violations,
    output wire [31:0]   head_read_before_ready,

    // ---- the PE seam -------------------------------------------------------
    //
    // Section 1 of `docs/F2_INTERFACES_2026-10-06.md`, at this module's top
    // ports.  At `EXTERNAL_PE = 0` the internal `u_bank` is driven from the
    // owner mux and these are the same nets the array is wired to, read by
    // nobody; at 1 the array is not instantiated and the owner mux drives the
    // seam instead.  Nothing between the two arms moves -- the arithmetic, the
    // schedule, the stores, the weight image and the clocks are identical, and
    // that is the claim the differential arm measures rather than asserts.
    //
    // **Fixed network widths, not this module's.**  Everything below is the
    // contract's, and this module is already built at them: `PE_LANES` 128,
    // `PE_OC_MAX` 16, `PE_OUT_W` 10, `PE_ACC_W` 32.  Where the module's own
    // nets are *narrower* -- only `pe_req_act`, at the layer's 11 per lane --
    // the bridge is a lane-by-lane sign extension, spelled out in the body.
    output wire                           pe_req_slot_valid,
    output wire [PE_LANES*PE_ACT_W-1:0]   pe_req_act,
    output wire [PE_LANES*8-1:0]          pe_req_wgt,
    output wire                           pe_req_pix_start,
    output wire [PE_LANES*PE_ACC_W-1:0]   pe_req_bias,
    // **The live shape, out loud.**  The front half changes mode between the
    // encoder, the head and the trunk; an arbiter cannot reach into a peer's
    // hierarchy the way a testbench can, so a module that does not own its
    // array has to say which shape it is running.
    output wire [1:0]                     pe_req_mode,
    output wire                           pe_req_rq_start,
    output wire [PE_OC_MAX*16-1:0]        pe_req_rq_m,
    output wire [PE_OC_MAX*5-1:0]         pe_req_rq_shift,
    output wire [PE_OUT_W-1:0]            pe_req_rq_zero,
    output wire [PE_OUT_W-1:0]            pe_req_rq_qmin,
    output wire [PE_OUT_W-1:0]            pe_req_rq_qmax,
    input  wire                           pe_rsp_rq_ready,
    input  wire                           pe_rsp_rq_ready_n,
    input  wire [8*PE_OUT_W-1:0]          pe_rsp_y,
    input  wire [7:0]                     pe_rsp_y_valid,
    input  wire                           pe_rsp_rq_busy,
    input  wire                           pe_rsp_rq_done,
    input  wire                           pe_rsp_rq_overrun,
    input  wire [31:0]                    pe_rsp_mode_viol,
    input  wire [PE_LANES*PE_ACC_W-1:0]   pe_rsp_acc,
    input  wire [PE_LANES*PE_ACC_W-1:0]   pe_rsp_snapshot,
    // Section 1's owner output and not part of the bank: every valid, read and
    // output drained, so that a mode change now is safe.  Necessary and not
    // sufficient -- the arbiter ANDs it with `bank_idle`.
    output wire                           mac_mode_safe,

    // ---- the weight store's seam -------------------------------------------
    //
    // The store or the seam that replaces it, exactly as `EXTERNAL_PE` did for
    // the bank one layer up.  At `EXTERNAL_WSTORE = 0` these are the same nets
    // `u_w` is wired to and nobody reads them; at 1 `u_w` does not exist and
    // the module's internal owner mux drives the seam instead.  Nothing between
    // the two arms moves -- the arithmetic, the schedule, the clients and the
    // FSM are identical, and `tools/run_encoder_trunk_wstore.sh` measures that
    // rather than asserting it.
    //
    // **One read port, and it is the port the front half has always had.**  The
    // store is not given a second port for the decoder: the decoder is a third
    // *time-multiplexed* client of this one, which is why what crosses the seam
    // is a request/response pair and not an arbiter.  The header states the rule
    // that makes handing the port over safe.
    output wire                           ws_req_rd_en,
    output wire [W_ADDR_W-1:0]            ws_req_rd_addr,
    input  wire [W_WORD_W-1:0]            ws_rsp_rd_data,
    // The loader side, carried out unchanged rather than left behind.  At the
    // board `wt_wr_en` is tied low and the image arrives through the store's
    // `INIT_FILE`, but a seam that handed out one of the store's two ports and
    // silently dropped the other would be half an ownership statement.  These
    // are this module's own `wt_wr_*` inputs, passed through.
    output wire                           ws_wr_en,
    output wire [W_ADDR_W-1:0]            ws_wr_addr,
    output wire [W_WORD_W-1:0]            ws_wr_data,
    // **The switch rule, in hardware.**  High exactly when no store read was
    // accepted this cycle and none was accepted on the previous one -- the same
    // two-cycle margin `bank_idle` uses (F2_INTERFACES section 2.1) and the same
    // rule `d64_decoder_wreader`'s `sw_safe` states on the decoder's side.  A
    // hand-over that lands between a read and its return gives a client another
    // tensor's tile, so the owner changes which client drives `ws_req_*` only
    // while this is high -- and only while the decoder's `sw_safe` is high too.
    output wire                           ws_idle
);


    // ------------------------------------------------------------------
    // clocks and reset
    // ------------------------------------------------------------------

    // The trunk takes `arst_n` directly, as it always has -- its own
    // `d64_h0_block` synchronizes what it needs -- so only the PE domain needs
    // one here, for the array and the store.
    wire pe_rst_n;
    d64_reset_sync u_pe_rst (.clk(clk_pe), .arst_n(arst_n), .rst_n(pe_rst_n));

    // ------------------------------------------------------------------
    // the two clients' seams
    // ------------------------------------------------------------------

    wire          e_pe_slot_valid, e_pe_pix_start, e_pe_rq_start;
    wire [1407:0] e_pe_act;
    wire [1023:0] e_pe_wgt;
    wire [4095:0] e_pe_bias;
    wire [255:0]  e_pe_rq_m;
    wire [79:0]   e_pe_rq_shift;
    wire [9:0]    e_pe_rq_zero, e_pe_rq_qmin, e_pe_rq_qmax;
    wire          e_pe_wt_rd_en;
    wire [7:0]    e_pe_wt_rd_addr;
    wire [1023:0] e_pe_wt_rd_data;
    wire [1:0]    e_pe_mode;

    wire          t_pe_slot_valid, t_pe_pix_start, t_pe_rq_start;
    wire [1407:0] t_pe_act;
    wire [1023:0] t_pe_wgt;
    wire [4095:0] t_pe_bias;
    wire [255:0]  t_pe_rq_m;
    wire [79:0]   t_pe_rq_shift;
    wire [9:0]    t_pe_rq_zero, t_pe_rq_qmin, t_pe_rq_qmax;
    wire          t_pe_wt_rd_en;
    wire [7:0]    t_pe_wt_rd_addr;
    wire [1023:0] t_pe_wt_rd_data;

    // The array's answers.  Declared here because both clients and the array
    // itself are below, and an identifier first used in a port connection is an
    // implicit 1-bit net -- which is a defect this project has now met twice.
    wire          rq_ready, rq_ready_n, rq_busy, rq_done, rq_overrun;
    wire [79:0]   y_bus;
    wire [7:0]    y_valid_bus;
    wire [4095:0] acc_bus, snap_bus;
    // ACC25 -- the bank is built narrower than the 32-bit interface it sits
    // behind.  `ACC_W` is the bank's internal accumulator/snapshot width and
    // `BUS_W` is the frozen downstream lane width.  The bank's inputs are
    // narrowed and its outputs sign-extended **here and nowhere else**, so
    // `acc_bus`/`snap_bus` carry 32-bit lanes whatever `ACC_W` is and no client
    // below can observe the narrow value.
    localparam BANK_LANES = 128;          // the bank's multipliers/accumulators/snapshots
    localparam BUS_W      = 32;           // the lane width pe_rsp_acc/snapshot are built at
    // The *array's* operand width -- what `u_bank` is built at, and a property
    // of the layers this module runs and not of the seam.  `PE_ACT_W` is the
    // contract's.  They are equal today (11) and are deliberately two names.
    localparam BANK_ACT_W = 11;
    wire [BANK_LANES*ACC_W-1:0] bank_bias, bank_acc, bank_snap;
    wire [BANK_LANES-1:0]       bank_bias_ext_bad;
    wire [1023:0] wt_rd_data;

    wire          enc_mac_mode_safe;
    // The encoder's own view of the array's contract, kept separate from the
    // array's.  They carry the same number and they are not the same net: an
    // output connected to a net another instance already drives is a
    // multiply-driven net, which is `x`, and `x` in a `||` gate reads as
    // false.  This stage found that the way it finds everything -- the count
    // printed as `x` while every tensor was exact.
    wire [31:0]   enc_mode_viol;
    wire          t_busy, t_done_w;
    wire [31:0]   t_cycles, t_cfg_cycles, t_bar_cycles, t_cfg_writes;
    wire [5:0]    t_clean_first;
    wire [BLK_W-1:0] t_blk_w;
    wire          t_src_w, t_dst_w;
    wire          t_req_buf_w;

    // ------------------------------------------------------------------
    // the owner
    // ------------------------------------------------------------------

    localparam [2:0] O_IDLE = 3'd0, O_ENC = 3'd1, O_TRK = 3'd2, O_DONE = 3'd3,
                     O_CLR  = 3'd4;

    reg  [2:0]  owner_q;
    reg         enc_start, trk_start;
    reg         enc_started, trk_started;
    reg  [31:0] cyc;

    assign owner = owner_q;
    assign busy  = (owner_q != O_IDLE) && (owner_q != O_DONE);
    // **Registered, and the register is the whole of it.**  `done` used to be
    // this same comparison as a *combinational* decode of three state bits, and
    // the handover that produces its predecessor is `O_ENC(001) -> O_TRK(010)` --
    // two bits changing at once, so a decode of three can glitch in between.
    //
    // The board top samples this into `clk_axi` through a two-flop synchroniser
    // and sets `crc_owner` on the first edge it sees (board top :679-684).  Two
    // flops make a *stable level* safe to cross; they do not make a glitch safe
    // to sample.  And `crc_owner` is never cleared except by reset, so one
    // sampled glitch hands the A and R channels to the CRC reader for the rest of
    // the run -- which is the shape of the second board capture: `encoder_done =
    // 0`, `crc_done = 1`, and not one trunk read or write beat.
    //
    // One frame and one shot, so a sticky level is sufficient and no toggle
    // protocol is needed.  **It is no longer one shot**, so the level is now
    // cleared -- in `O_CLR`, the cycle after the re-arm is accepted, and from a
    // branch *above* the one that raises it, because a `done` raised and
    // cleared on one edge is a `done` no synchroniser is obliged to see.
    reg done_q;
    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n)              done_q <= 1'b0;
        else if (owner_q == O_CLR)  done_q <= 1'b0;
        else if (owner_q == O_DONE) done_q <= 1'b1;

    assign done  = done_q;
    assign wall_cycles = cyc;

    // **Owner-safe and mode-safe are two conditions, not one.**  The first is
    // this module's: the encoder has finished and its last trunk-A word has
    // been taken.  The second is the encoder's: no slot offered, no requant
    // busy, both layer clients drained.  `down1` and the trunk's conv1 happen
    // to be the same mode, and that is exactly why the mode term has to be a
    // term -- "the mode did not change" is not "nothing is in flight".
    // Compute finished, stream drained, memory committed -- three stages and not
    // one, so the three terms stay even though the last implies the middle.
    wire enc_owner_safe = enc_done && out_committed && trunk_store_committed;
    wire enc_mode_safe  = enc_mac_mode_safe;

    // The one thing the handover condition cannot prevent and can count: a
    // client offering a slot while the other one owns the array.  Each client's
    // own drain is what should stop it; if a drain is wrong this is where it
    // shows, and it is not corrected here.
    wire enc_slot_while_trk = (owner_q == O_TRK) && e_pe_slot_valid;
    wire trk_slot_while_enc = (owner_q == O_ENC) && t_pe_slot_valid;

    // ------------------------------------------------------------------
    // the re-arm -- clearing a finished job, and what has to be true first
    // ------------------------------------------------------------------
    //
    // **`O_DONE` used to be terminal.**  One frame, one `start`, and the only
    // way back was `arst_n` -- which also takes the weight store with it.  A
    // second frame is therefore not a second `start`: the job state has to be
    // *cleared*, and the clear has to happen where nothing can still be in
    // flight, or a frame that is half-drained becomes a frame that is half
    // computed and the difference is a picture and not a crash.
    //
    // What the clear does, and what it deliberately does not
    // ------------------------------------------------------
    //
    // `O_CLR` is one cycle.  In it:
    //
    //   * `d64_encoder` is held in reset through `enc_rst_n`.  At
    //     `EXTERNAL_PE = 1` that module owns no weight store and holds no
    //     runtime configuration -- `d64_head_ctrl`'s bias, multiplier and shift
    //     are `function`s over the channel index, and the downs table is
    //     `d64_down_cfg_rom`, a `case` over literals whose `cfg_we` port has
    //     nothing to write to -- so its reset clears *job* state and nothing
    //     else.  A reset is the only route in: the head's sequencer (`sched_y`,
    //     `sched_done`, `done`, `final_eof_seen`, `out_frame_id_valid_q`) has
    //     no other way back to zero and that module is in flight for review.
    //     Nothing on the path from either weight port reaches the encoder.
    //
    //   * `d64_trunk_blocks` is **not** reset.  Its `S_DONE` is re-entered as
    //     an idle state by its own `start`, which re-arms the fill sequencer
    //     through the same `S_CFG` every block boundary already uses.  Resetting
    //     it instead would clear `wd_seen` -- the record of the far side's
    //     `wr_done_tog`, a *level* the DDR writer holds after its last response
    //     of a block -- and the level still standing from the previous frame
    //     would be read as a fresh write-done edge a couple of cycles into the
    //     next one.  The barrier would then release on a write nobody answered.
    //
    //   * the weight store `u_w`, the trunk's `cfgrom` and every requant table
    //     are untouched.  Nothing in `O_CLR` names them, and the three-frame
    //     gate compares all 247 weight words and all 320 cfg words after every
    //     frame rather than taking the absence of a write on trust.
    //
    // What has to be true before it
    // -----------------------------
    //
    //     owner_q == O_DONE   the five blocks finished.  The trunk's own barrier
    //                         is what that means: each block's schedule
    //                         drained, its last word reached the write FIFO, and
    //                         the far side answered the whole block.
    //  && done_q             `done` has been *presented* for a full cycle.  A
    //                         level raised and cleared on one edge is a pulse a
    //                         two-flop synchroniser in another clock domain is
    //                         not obliged to see -- 8.33 ns of `clk_pe` fits
    //                         between two `clk_axi` edges.  That is the failure
    //                         the `done` comment above already records once,
    //                         with the board's `crc_owner` left unset for the
    //                         rest of the run.
    //  && !enc_busy          neither client mid-pass,
    //  && !t_busy            the trunk is not halfway through a block,
    //  && !rq_busy           and the shared requant is idle.
    //  && rearm_quiet        **no client offering a slot this cycle or the
    //                         last.**  This is the arbiter's `bank_idle` margin
    //                         and the cache's re-point rule: the bank is
    //                         pipelined, and "no slot right now" does not cover
    //                         the slot the owner mux handed over one cycle ago.
    //                         A one-cycle hole in exactly this shape has been
    //                         found in this project three times.
    //  && !wt_wr_en          the host is not mid-load of the image this re-arm
    //                         exists to preserve.
    //
    // **And `O_TRK` leaves on an *edge* of the trunk's done, not its level.**
    // That is the hole this re-arm met first.  `t_done_w` is
    // `(state == S_DONE)`, and the trunk sits in `S_DONE` from the end of the
    // previous frame until its `start` reaches it -- so a level test walks
    // `O_ENC -> O_TRK -> O_DONE` in two cycles and retires a frame whose trunk
    // never left `S_DONE`.  What is left in the ping-pong buffers is then the
    // previous frame's, and because the new frame's encoder overwrites the
    // *other* buffer, the frame-to-frame comparison is what says so.
    //
    // A `start` the FSM cannot honour -- during `O_ENC` or `O_TRK`, which is
    // what a host on a frame timer sends when it is early -- is counted in
    // `rearm_early` and otherwise ignored.  It does not abort the frame in
    // flight, and it does not become the next frame's start.
    wire job_clear = (owner_q == O_CLR);
    wire enc_rst_n = pe_rst_n && !job_clear;

    reg  slot_quiet_q;
    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n) slot_quiet_q <= 1'b0;
        else           slot_quiet_q <= !e_pe_slot_valid && !t_pe_slot_valid;

    wire rearm_quiet = slot_quiet_q && !e_pe_slot_valid && !t_pe_slot_valid;
    wire rearm_ok    = done_q && !enc_busy && !t_busy && !rq_busy
                    && rearm_quiet && !wt_wr_en;

    // **The trunk's done as an event.**  `trk_done_q` holds the level one cycle
    // back, so `trk_done_edge` is the rise and not the state.
    reg  trk_done_q;
    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n) trk_done_q <= 1'b0;
        else           trk_done_q <= t_done_w;
    wire trk_done_edge = t_done_w && !trk_done_q;

    // **The host's `start` as an event too, for the same reason.**  A host that
    // holds `start` high while it waits for `done` would otherwise re-arm the
    // instant the frame it waits on finishes, and then again, and the number of
    // frames would be the number of cycles rather than the number of pulses.
    // One rising edge, one frame.
    reg  start_q;
    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n) start_q <= 1'b0;
        else           start_q <= start;
    wire start_edge = start && !start_q;

    // A `start` that reached neither `O_IDLE` nor a re-armable `O_DONE`.
    wire start_unserved = start_edge && (owner_q != O_IDLE)
                       && !((owner_q == O_DONE) && rearm_ok);

    // ------------------------------------------------------------------
    // the owner mux
    // ------------------------------------------------------------------

    // Register the route flag beside owner_q from the same next-state event.
    reg sel_enc_q;
    wire sel_enc=sel_enc_q;

    wire          pe_slot_valid = sel_enc ? e_pe_slot_valid : t_pe_slot_valid;
    wire [1407:0] pe_act        = sel_enc ? e_pe_act        : t_pe_act;
    wire [1023:0] pe_wgt        = sel_enc ? e_pe_wgt        : t_pe_wgt;
    wire          pe_pix_start  = sel_enc ? e_pe_pix_start  : t_pe_pix_start;
    wire [4095:0] pe_bias       = sel_enc ? e_pe_bias       : t_pe_bias;

    // ------------------------------------------------------------------
    // P7h: the value the bank is actually handed
    // ------------------------------------------------------------------
    //
    // **After the owner mux, at the bank's own load edge.**  The engine's own
    // probe reads `bias_r` inside `d64_conv_engine`, which is the right array
    // but is still upstream of this mux and of the compat map; a bank handed
    // the wrong half can sit behind a probe that reads the right one.  These
    // three observations are the chain and not three views of one point:
    //
    //     p7_br    the array's read, inside the engine
    //     p7_pb    what the bank takes, here, after the mux
    //     p7_bias  the accumulator one cycle later
    //
    // `!sel_enc` is the trunk owning the bank, and the first such load edge of
    // a one-shot run is block 0's conv1: there is no block index at this level
    // and there does not need to be one, because the encoder owns the bank
    // first and hands it over exactly once.
    reg        p7_pb_seen;
    reg [63:0] p7_pb_v;
    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n) begin
            p7_pb_seen <= 1'b0; p7_pb_v <= 64'd0;
        end
        else if (!p7_pb_seen && pe_pix_start && !sel_enc) begin
            p7_pb_seen <= 1'b1;
            p7_pb_v    <= pe_bias[63:0];
        end

    assign t_p7_pb      = p7_pb_v;
    assign t_p7_pb_seen = p7_pb_seen;
    wire          pe_rq_start   = sel_enc ? e_pe_rq_start   : t_pe_rq_start;
    wire [255:0]  pe_rq_m       = sel_enc ? e_pe_rq_m       : t_pe_rq_m;
    wire [79:0]   pe_rq_shift   = sel_enc ? e_pe_rq_shift   : t_pe_rq_shift;
    wire [9:0]    pe_rq_zero    = sel_enc ? e_pe_rq_zero    : t_pe_rq_zero;
    wire [9:0]    pe_rq_qmin    = sel_enc ? e_pe_rq_qmin    : t_pe_rq_qmin;
    wire [9:0]    pe_rq_qmax    = sel_enc ? e_pe_rq_qmax    : t_pe_rq_qmax;

    // The store's read port, address and enable together -- the rule every
    // store read in this project has had to be taught, in both directions.
    //
    // **And the enable is a function of the owner state, which is the only way
    // `ws_idle` becomes producible at all.**  Both engines hold `pe_wt_rd_en`
    // at 1 (`d64_conv_engine.v:559`, "this engine reads every cycle") and each
    // is muxed onto this port by `sel_enc`, so an ungated mux presented a store
    // read in **every** cycle -- including the ones where the owner FSM sat in
    // `O_IDLE` and no slot could be in flight.  `ws_idle` is
    // `~(wt_rd_en | ws_rd_q)`, so it was constant 0: F3 6.21.4's hand-over
    // licence could never be issued by this module, and a root job that named
    // the front half could never hand the port to the decoder.  Measured in the
    // root at 100 us before this gate: `owner_q 0, sel_enc 0, t_pe_wt_rd_en 1,
    // wt_rd_en 1, ws_idle 0`.
    //
    // **The gate is on `O_IDLE` and `O_DONE` and on nothing else.**  `O_ENC`
    // and `O_TRK` are the two compute states, where the enable is the muxed
    // engine's and is bit for bit what it was; `O_CLR` is the one-cycle re-arm
    // and is left reading for the same reason.  So every cycle in which either
    // engine can be scheduled has the enable it always had, and both engines'
    // prefetch and response pipelines are exactly what they were -- what
    // changes is that an owner sitting in `O_IDLE` or `O_DONE` stops asking,
    // and `ws_idle` rises one cycle after the last read of a run (the
    // registered `ws_rd_q` below is that cycle, and it is the margin F3 6.21.4
    // names).  **`tools/run_encoder_trunk_joint.sh` is the protection gate and
    // it passes with neither its stimulus nor its expectations edited** -- all
    // nine of its arms, and the three-frame arm's numbers are the ones F3 6.17
    // records, down to `ACC25 boundary ... narrowing CHANGED a value on 0` and
    // `store port 0, ws_idle rule 0`.  `tools/run_encoder_trunk_wstore.sh` is
    // the other gate that reads this port and it has **not** been re-run: what
    // it would measure is inert here (the enable in `O_ENC` and `O_TRK` is bit
    // for bit what it was, and nothing is consumed in `O_IDLE`), and that is an
    // argument rather than a measurement, which is why it is written down as
    // one.
    wire          wt_owner_active = (owner_q != O_IDLE) && (owner_q != O_DONE);
    wire          wt_rd_en   = wt_owner_active
                             ? (sel_enc ? e_pe_wt_rd_en : t_pe_wt_rd_en) : 1'b0;
    wire [W_ADDR_W-1:0] wt_rd_addr = sel_enc ? e_pe_wt_rd_addr : t_pe_wt_rd_addr;

    assign e_pe_wt_rd_data = wt_rd_data;
    assign t_pe_wt_rd_data = wt_rd_data;
    assign pe_mode = sel_enc ? e_pe_mode : 2'd0;

    // ------------------------------------------------------------------
    // the seam's request side
    // ------------------------------------------------------------------
    //
    // **Driven unconditionally**, exactly as `d64_tconv_engine` drives its own:
    // at `EXTERNAL_PE = 0` these are the same nets the local array is wired to
    // and nothing reads them, and at 1 the local array does not exist.  A
    // conditional driver would be a second thing that has to be right, and the
    // arm that is wrong would be the one nobody builds.
    //
    // Everything here is already the contract's width -- the front half's
    // clients present 1408-bit activations, 1024-bit weights and 32-bit bias
    // lanes -- except the activation, which is bridged lane by lane below.
    assign pe_req_slot_valid = pe_slot_valid;
    assign pe_req_wgt        = pe_wgt;
    assign pe_req_pix_start  = pe_pix_start;
    assign pe_req_bias       = pe_bias;
    // **The live mode, not a copy of it.**  `pe_mode` is the net the local
    // array's `mode` port is driven from, so a comparison against the local
    // bank's own port is a comparison against the shape the array is actually
    // running under -- which is the only thing that makes exporting it worth
    // anything.  Wrong here is not a cosmetic difference: `d64_mac128` picks the
    // requant constant slice from `mode`, so a bank told the wrong shape
    // requantises every accumulator against the wrong output channel's M and
    // shift, with the schedule and the write count unchanged.
    assign pe_req_mode       = pe_mode;
    assign pe_req_rq_start   = pe_rq_start;
    assign pe_req_rq_m       = pe_rq_m;
    assign pe_req_rq_shift   = pe_rq_shift;
    assign pe_req_rq_zero    = pe_rq_zero;
    assign pe_req_rq_qmin    = pe_rq_qmin;
    assign pe_req_rq_qmax    = pe_rq_qmax;

    // The one bridge between the layer's width and the seam's.
    //
    // **A sign extension, not a zero extension and not a truncation.**  A lane
    // is `q - zero` and is read signed inside the bank, so padding with zeros
    // turns every negative operand into a large positive one; `d64_mac128`'s own
    // header records that exact bug costing a build.  Written lane by lane
    // rather than left to a concatenation, because a concatenation that turned
    // the expression unsigned is the other half of the same story.
    genvar sl_a;
    generate
        for (sl_a = 0; sl_a < PE_LANES; sl_a = sl_a + 1) begin : g_act
            if (BANK_ACT_W < PE_ACT_W)
                assign pe_req_act[sl_a*PE_ACT_W +: PE_ACT_W] =
                    {{(PE_ACT_W-BANK_ACT_W){pe_act[sl_a*BANK_ACT_W + BANK_ACT_W - 1]}},
                     pe_act[sl_a*BANK_ACT_W +: BANK_ACT_W]};
            else
                assign pe_req_act[sl_a*PE_ACT_W +: PE_ACT_W] =
                    pe_act[sl_a*BANK_ACT_W +: PE_ACT_W];
        end
    endgenerate

    // Section 1's `mac_mode_safe`, stated at this module's level: **every valid,
    // read and output drained**, so a mode change now is safe.  What the module
    // has to say is that nothing is on *its* seam -- no slot presented, no
    // requant accept happening this cycle, none running, and no requant output
    // still owed.  `necessary and not sufficient`, in section 1's own words: the
    // requant's `run` register is set on the accept edge and `rq_busy` is that
    // register delayed by one, so there is a cycle this is high in while the
    // sequencer is still mid-flight -- which is exactly why the arbiter ANDs it
    // with `bank_idle`, and why `bank_idle` needs the bank quiet on two cycles
    // and not one.
    assign mac_mode_safe = !pe_slot_valid && !pe_rq_start && !rq_busy &&
                           !(|y_valid_bus);

    // ------------------------------------------------------------------
    // D3c6-e6b: the same question, asked at the bank's own input
    // ------------------------------------------------------------------
    //
    // The engine's probe says its `cfg` holds this block's entry.  This one
    // says the word that actually reaches the array is still that entry after
    // `d64_mac128_compat_map` and the owner mux -- the half of the path a probe
    // inside the engine cannot see.  Together they cut the chain in three:
    //
    //   cfg_local_ok != 11111            the image, its loader, the block
    //                                    boundary refill or the engine's cfg
    //   == 11111, rq_bank_ok != 11111    the compat map or the owner mux
    //   both == 11111                    the parameters arrived; the next
    //                                    suspects are the accumulator, the
    //                                    activation and the weights
    //
    // **The request is identified by its zero point.**  conv3 is the only mode
    // whose requant zero is 413 -- conv1 and conv2 write ReLU tensors and carry
    // zero -- so no other phase can set a bit here, and `!sel_enc` keeps the
    // encoder's own phases out even if one of them ever did.
    //
    // **A copy of the image's table, not a reference to it.**  The values are
    // `control/generated/d64_trunk_cfg_rom.v` entries 32/48, 96/112, 160/176,
    // 224/240 and 288/304, which are `sim/d3c5/blkcfg.mem` words at the same
    // indices; the same table is written out in `d64_conv_engine` because
    // anything reachable from here is read through the addressing under test.
    // `tools/d64_trunk_cfg_rtl.py` owns the image.
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

    wire [20:0] bank_lane0 = {pe_rq_shift[4:0], pe_rq_m[15:0]};
    wire        t_c3_req   = !sel_enc && pe_rq_start && (pe_rq_zero == (RUNTIME_PROFILE ? profile_trunk_zero : ZERO_T[9:0]));

    // **Both output groups are accepted, and that is deliberate.**  Which og a
    // block's first conv3 phase carries is the scheduler's business and this
    // module does not know it; lane 0 of og 0 and lane 0 of og 1 are two
    // different entries.  Matching either is still a comparison against the
    // image: a wrong M or a wrong shift would have to collide with the other
    // group's entry to pass, and no two of these five pairs are near each
    // other.
    reg [2:0] b_blk_q;
    reg       b_c3_seen;
    reg [4:0] bank_ok;
    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            b_blk_q <= 3'd0; b_c3_seen <= 1'b0; bank_ok <= 5'd0;
        end
        else begin
            b_blk_q <= t_blk_w;
            if (t_blk_w != b_blk_q) b_c3_seen <= 1'b0;
            if (t_c3_req && !b_c3_seen) begin
                b_c3_seen <= 1'b1;
                bank_ok[t_blk_w] <= (bank_lane0 == c3_lane0(t_blk_w, 1'b0))
                                 || (bank_lane0 == c3_lane0(t_blk_w, 1'b1));
            end
        end
    end

    assign t_rq_bank_ok = bank_ok;


    // ------------------------------------------------------------------
    // the store, or the seam that replaces it
    // ------------------------------------------------------------------
    //
    // **Driven unconditionally, in both arms**, exactly as the PE seam above
    // drives `pe_req_*`: at `EXTERNAL_WSTORE = 0` these are the same nets the
    // local store is wired to and nothing reads them, and at 1 the local store
    // does not exist.  A conditional driver would be a second thing that has to
    // be right, and the arm that is wrong would be the one nobody builds.
    assign ws_req_rd_en   = wt_rd_en;
    assign ws_req_rd_addr = wt_rd_addr;
    assign ws_wr_en       = wt_wr_en;
    assign ws_wr_addr     = wt_wr_addr;
    assign ws_wr_data     = wt_wr_data;

    // The store's read enable, delayed one cycle.  `d64_weight_store` latches
    // `mem[rd_addr]` on the edge that sees `rd_en` and puts the enable on its
    // *output*, so a read is in flight for exactly one cycle after its enable.
    // `ws_idle` is the complement of "this cycle or the last", which is the
    // whole of the margin -- one cycle, not zero and not two.
    reg ws_rd_q;
    always @(posedge clk_pe or negedge pe_rst_n)
        if (!pe_rst_n) ws_rd_q <= 1'b0;
        else           ws_rd_q <= wt_rd_en;

    assign ws_idle = ~(wt_rd_en | ws_rd_q);

    // **Only one of these elaborates.**  `g_local_wstore` is what this module
    // has always been: the array's operand store inside, driven by the owner
    // mux above.  `g_external_wstore` is a client of a store above *it*, and
    // every net the local instance used to drive comes off the seam instead.
    // Nothing else in the module knows which arm was built -- the owner mux,
    // both weight clients and the FSM read `wt_rd_data` either way.
    //
    // The elaborated instance count is what catches an arm that built nothing:
    // a generate block with no module instantiation in it never appears in the
    // listing, so the gate counts `d64_weight_store` and its parentage rather
    // than reading this comment.
    generate
        if (EXTERNAL_WSTORE != 0) begin : g_external_wstore
            // The one place the raised store's answer enters.  **Straight
            // through, with no register**, which is the store's own alignment
            // argument and not a size argument: the store already registers its
            // read, and a register here would move every weight one cycle off
            // its activation.  `d64_decoder_wreader` makes the same point for
            // the decoder's side of the same port.
            assign wt_rd_data = ws_rsp_rd_data;
        end else begin : g_local_wstore
            d64_weight_store #(.TILES(W_TILES), .WORD_W(W_WORD_W),
                               .ADDR_W(W_ADDR_W), .INIT_FILE(WT_INIT_FILE)) u_w (
                .clk(clk_pe), .rst_n(pe_rst_n),
                .wr_en(wt_wr_en), .wr_addr(wt_wr_addr), .wr_data(wt_wr_data),
                .rd_en(wt_rd_en), .rd_addr(wt_rd_addr), .rd_data(wt_rd_data)
            );
        end
    endgenerate

    // The bank's requant operands, lane 0, at the widths this instance's
    // `u_bank` is built with (`.M_W(16)`, `.SHIFT_W(5)`).  `bank_snap` is the
    // bank's own `rq_snap_o`, so lane 0 is `[ACC_W-1:0]` of it -- the same
    // field, taken the same way, as `bank_s_acc`.
    wire [ACC_W-1:0] bank_s_acc;
    wire [15:0]      bank_s_m;
    wire [4:0]       bank_s_shift;
    wire             bank_s_ok0, bank_run;

    // ------------------------------------------------------------------
    // the bank, or the seam that replaces it
    // ------------------------------------------------------------------
    //
    // **Two arms, and only one of them elaborates.**  `g_local_pe` is what this
    // module has always been: the array inside, driven by the owner mux above.
    // `g_external_pe` is a client of an array above *it*, and every net the
    // local port list used to drive comes off the seam instead.  Nothing else
    // in the module knows which arm was built -- the mux, the clients, the
    // store and the FSM read `rq_ready`, `y_bus` and the rest either way.
    //
    // `u_bank` is inside `g_local_pe` and therefore **not instantiated at all**
    // at `EXTERNAL_PE = 1`; that is the point of the parameter, and the
    // structural count in the gate is what catches an arm that elaborated to
    // nothing.  The shared bank's `obs_*` probes are not on the seam (section 1
    // has no entry for them), so the P7r probe below is blind in the external
    // arm; it is tied to a deterministic zero rather than left dangling,
    // because a dangling input is `z` and `z` in a `!==` is not a reading.
    generate
        if (EXTERNAL_PE != 0) begin : g_external_pe
            // The response side, straight from the seam.  Every width agrees
            // because this module's array is already the contract's shape --
            // `ACC_W` lanes are widened back to `BUS_W` before they leave, so
            // `acc_bus`/`snap_bus` are the contract's 128x32 whatever the array
            // is built at.
            assign rq_ready       = pe_rsp_rq_ready;
            assign rq_ready_n     = pe_rsp_rq_ready_n;
            assign rq_busy        = pe_rsp_rq_busy;
            assign rq_done        = pe_rsp_rq_done;
            assign rq_overrun     = pe_rsp_rq_overrun;
            assign bank_mode_viol = pe_rsp_mode_viol;
            assign acc_bus        = pe_rsp_acc;
            assign snap_bus       = pe_rsp_snapshot;
            // Lane by lane: the seam's lane is `PE_OUT_W` wide, this module's
            // scratch is ten.  Saturation already happened in the bank against
            // the `qmax` this module asked for, so the low bits are the value
            // and not a truncation of it.
            genvar sl_y;
            for (sl_y = 0; sl_y < 8; sl_y = sl_y + 1) begin : g_y
                assign y_bus[sl_y*10 +: 10] = pe_rsp_y[sl_y*PE_OUT_W +: 10];
            end
            assign y_valid_bus = pe_rsp_y_valid;
            assign bank_s_acc   = {ACC_W{1'b0}};
            assign bank_s_m     = 16'd0;
            assign bank_s_shift = 5'd0;
            assign bank_s_ok0   = 1'b0;
            assign bank_run     = 1'b0;
            // `bank_bias` and `bank_bias_ext_bad` are the ACC25 boundary and
            // they exist only where the array is narrow.  Nothing drives them
            // here: the bias goes out over the seam at the contract's 32-bit
            // lanes.  Unused, not dangling.
        end else begin : g_local_pe
            d64_mac128 #(
        .FAULT_STICKY(FAULT_STICKY),.ACT_W(11), .WGT_W(8), .ACC_W(ACC_W), .M_W(16),
                         .SHIFT_W(5),
                         .OUT_W(10), .RQ_LANES(8), .LANES(128), .OC_MAX(16),
                         .RQ_LOOKAHEAD(3)) u_bank (
                .clk(clk_pe), .rst_n(pe_rst_n), .mode(pe_mode),
                .slot_valid(pe_slot_valid), .act(pe_act), .wgt(pe_wgt),
                .pix_start(pe_pix_start), .bias(bank_bias),
                .rq_start(pe_rq_start), .rq_ready(rq_ready),
                .rq_ready_n(rq_ready_n),
                .rq_m(pe_rq_m), .rq_shift(pe_rq_shift), .rq_zero(pe_rq_zero),
                .rq_qmin(pe_rq_qmin), .rq_qmax(pe_rq_qmax),
                .y(y_bus), .y_valid(y_valid_bus), .rq_busy(rq_busy),
                .rq_done(rq_done), .rq_overrun(rq_overrun),
                .mode_viol(bank_mode_viol),
                .acc_o(bank_acc), .rq_snap_o(bank_snap),
                .obs_s_acc0(bank_s_acc), .obs_s_m0(bank_s_m),
                .obs_s_shift0(bank_s_shift), .obs_s_ok0(bank_s_ok0),
                .obs_run(bank_run)
            );

            // --------------------------------------------------------------
            // ACC25 -- the bank's one boundary, pure wiring
            // --------------------------------------------------------------
            //
            // The bias is the only narrow-facing input: it is truncated to
            // `ACC_W` and `bank_bias_ext_bad` witnesses any lane whose dropped
            // high bits are not a legal sign extension.  The study's range
            // proof says no lane of the board image can set it, so it is a
            // witness left unread here rather than a new port on a module whose
            // interface every harness is closed against.
            //
            // The accumulator and the requant snapshot are the narrow-facing
            // outputs, and they are sign-extended back to `BUS_W` lanes on the
            // way out.  At `ACC_W == BUS_W` (the default) both branches are
            // plain assignments and the module is what it was.
            //
            // **The whole boundary is inside `g_local_pe` on purpose.**  Its
            // two ends are the local array and this module's contract-width
            // buses; a shared bank is built at the network's width already, so
            // narrowing a value that is not narrow would sign-extend lane 0's
            // low `ACC_W` bits out of a 32-bit lane and hand every client a
            // wrong accumulator.
            genvar bl;
            if (ACC_W == BUS_W) begin : g_bank_native
                assign bank_bias         = pe_bias;
                assign bank_bias_ext_bad = {BANK_LANES{1'b0}};
                assign acc_bus           = bank_acc;
                assign snap_bus          = bank_snap;
            end
            else begin : g_bank_narrow
                for (bl = 0; bl < BANK_LANES; bl = bl + 1) begin : g_lane
                    assign bank_bias[bl*ACC_W +: ACC_W] =
                        pe_bias[bl*BUS_W +: ACC_W];
                    assign bank_bias_ext_bad[bl] =
                        (pe_bias[bl*BUS_W+ACC_W +: (BUS_W-ACC_W)] !==
                         {(BUS_W-ACC_W){pe_bias[bl*BUS_W+ACC_W-1]}});
                    assign acc_bus[bl*BUS_W +: BUS_W] =
                        {{(BUS_W-ACC_W){bank_acc[bl*ACC_W+ACC_W-1]}},
                         bank_acc[bl*ACC_W +: ACC_W]};
                    assign snap_bus[bl*BUS_W +: BUS_W] =
                        {{(BUS_W-ACC_W){bank_snap[bl*ACC_W+ACC_W-1]}},
                         bank_snap[bl*ACC_W +: ACC_W]};
                end
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // P7r: the last slot -> the requant input -> the requant output
    // ------------------------------------------------------------------
    //
    // **The gap P7d leaves, stated exactly.**  P7d samples `mac_acc[63:0]` on
    // the accept edge itself, and under `EXTERNAL_PE` that comes from this
    // bank's `acc_o`, which is `acc_q` -- read by Verilog's non-blocking rule
    // *before* that edge's own update.  The requant does not read that value:
    // `d64_mac128` loads `s_acc` (and `rq_snap`) from `done_acc`, which on
    // `slot_valid` is `acc_q + red` and therefore *includes* the slot being
    // presented.  A matching P7d proves the array up to the accept and says
    // nothing about the last addition, the pass-0 operand, the latched M and
    // shift, or Y.  That is the gap this closes.
    //
    // **T** is this bank's first `rq_accept` while the trunk owns it.  That is
    // block 0's conv1 by construction -- the encoder owns the array first and
    // hands it over exactly once -- and three observations hang off it:
    //
    //   T    what the accept was: slot_valid, pix_start, owner, block index
    //   T+1  what that edge loaded: s_acc[0], s_m[0], s_shift[0], rq_zero,
    //        run, s_ok[0], and whether s_acc diverged from rq_snap
    //   T+2  the first output those operands produce: y[9:0], y_valid[0]
    //
    // "T+1" is the cycle *after* the accept edge -- the value that edge wrote.
    // Sampling on the accept edge would reproduce P7d's one-cycle-late error,
    // which is the whole reason this probe exists.
    //
    // **The lane is taken as a field and not as a slice.**  The bank's flat
    // bus has stride `ACC_W` per lane, so `s_acc[31:0]` would carry seven bits
    // of lane 1; `[24:0]` at this instance's `ACC_W == 25` is the whole lane.
    //
    // **The expectations are derived, never copied.**  `P7_ACC` is the value
    // *at* the accept and is explicitly not the completed accumulation.  These
    // come from `tb_d64_encoder_trunk_board_model`'s P7r print at PE 100 MHz /
    // `AXI_PHASE=0`; the model re-derives each `Y` from `d64_requant_lane`'s own
    // formula on the captured operands rather than reading it off the DUT, so
    // the probe tests the arithmetic and not its own transcription.
    //
    // **Two lanes, and the pair is the point.**  Lane 0's output clamps -- its
    // product is negative and the whole thing saturates to the low bound -- and
    // a clamped value cannot fail when `M` or `shift` moves, because every
    // wrong operand on that side clamps to zero too.  Lane 3 does not clamp, so
    // it is the one that can be wrong.  Both travel, side by side and from the
    // same edge, so that the clamp itself is a reading rather than an
    // inference: a lane 0 of 1023 against a lane 3 of 8 is the sign-and-shift
    // defect `d64_requant_lane` documents, visible in one word.
    localparam [24:0] P7R_ACC  = 25'h1FFFAD6;  // -1322: done_acc[0] at the accept
    localparam [15:0] P7R_M    = 16'h9590;
    localparam [4:0]  P7R_SH   = 5'h17;        // 23
    localparam [9:0]  P7R_ZERO = 10'h000;      // conv1 writes a ReLU tensor
    localparam [9:0]  P7R_Y0   = 10'h000;      // lane 0: clamps to the low bound
    localparam [9:0]  P7R_Y3   = 10'h008;      // lane 3: unclamped, and can fail

    wire p7r_accept = pe_rq_start & rq_ready & ~sel_enc;

    reg        p7r_armed, p7r_d1, p7r_d2;
    // `seen_a` and `seen_c` and nothing else.  The two T+1 records are one
    // capture, so a second register for the parameter arm would be a
    // register the mapper proves equal to the first and deletes -- which is
    // what it did: the first build of this probe had a `p7r_seen_b` and the
    // mapped netlist carries one FF for the two.  The bit is broadcast
    // instead.
    reg        p7r_seen_a, p7r_seen_c;
    reg [24:0] p7r_acc;
    reg [15:0] p7r_m;
    reg [4:0]  p7r_shift;
    reg [9:0]  p7r_zero;
    reg [9:0]  p7r_y0, p7r_y3;
    // {owner[2:0], pix_start, slot_valid, ne, s_ok[0], run, y_valid[3],
    //  y_valid[0]}
    reg [9:0]  p7r_state;
    reg [3:0]  p7r_ok;
    // latched at T, because T+1's control is the *next* phase's on a rollover
    reg        p7r_t_slotv, p7r_t_pstart;
    reg [2:0]  p7r_t_owner;
    reg [BLK_W-1:0] p7r_t_blk;

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            p7r_armed <= 1'b1; p7r_d1 <= 1'b0; p7r_d2 <= 1'b0;
            p7r_seen_a <= 1'b0; p7r_seen_c <= 1'b0;
            p7r_acc <= 25'd0; p7r_m <= 16'd0; p7r_shift <= 5'd0;
            p7r_zero <= 10'd0; p7r_y0 <= 10'd0; p7r_y3 <= 10'd0;
            p7r_state <= 10'd0;
            p7r_ok <= 4'd0;
            p7r_t_slotv <= 1'b0; p7r_t_pstart <= 1'b0;
            p7r_t_owner <= 3'd0; p7r_t_blk <= {BLK_W{1'b0}};
        end
        else begin
            if (p7r_armed && p7r_accept) begin
                p7r_armed    <= 1'b0;
                p7r_d1       <= 1'b1;
                p7r_t_slotv  <= pe_slot_valid;
                p7r_t_pstart <= pe_pix_start;
                p7r_t_owner  <= owner_q;
                p7r_t_blk    <= t_blk_w;
            end
            else p7r_d1 <= 1'b0;
            p7r_d2 <= p7r_d1;

            // ---- T+1: what the accept edge loaded into the requant --------
            if (p7r_d1 && !p7r_seen_a) begin
                p7r_seen_a <= 1'b1;
                p7r_acc    <= bank_s_acc;
                p7r_m      <= bank_s_m;
                p7r_shift  <= bank_s_shift;
                p7r_zero   <= pe_rq_zero;
                p7r_ok[0]  <= (bank_s_acc == P7R_ACC);
                p7r_ok[1]  <= (bank_s_m == P7R_M)
                           && (bank_s_shift == P7R_SH)
                           && (pe_rq_zero == P7R_ZERO);
                // The identity arm.  A capture that never fired and a capture
                // that fired on the wrong accept are the same four zeros
                // otherwise, and every term below is a property the accept
                // must have: the trunk owned the array, it was block 0, the
                // slot and the pixel start were both asserted, the requant
                // was running with lane 0 live, the two loads of `done_acc`
                // did not diverge, and the zero point is conv1's and not
                // conv3's 413.
                p7r_ok[3]  <= p7r_t_slotv && p7r_t_pstart
                           && (p7r_t_owner == O_TRK)
                           && (p7r_t_blk == {BLK_W{1'b0}})
                           && bank_run && bank_s_ok0
                           && (bank_s_acc === bank_snap[ACC_W-1:0]);
                // **`sacc_ne_snap` is zero by construction, and a one on the
                // board is a mapper verdict and not an arithmetic difference.**
                // `s_acc[0]` and `rq_snap[0]` are both loaded from the same
                // `done_acc[0]` expression on the same accept edge
                // (`d64_mac128.v:371` and `:381`), so the two nets can only
                // disagree if the mapper gave two copies of one number
                // different sources.  Read it as "two nets that must be equal
                // were not", never as "the requant's operand was wrong" -- that
                // question belongs to the accumulator record beside it.
                p7r_state[4] <= (bank_s_acc !== bank_snap[ACC_W-1:0]);
                p7r_state[3] <= bank_s_ok0;
                p7r_state[2] <= bank_run;
                p7r_state[6] <= p7r_t_pstart;
                p7r_state[5] <= p7r_t_slotv;
                p7r_state[9:7] <= p7r_t_owner;
            end

            // ---- T+2: the first output those operands produce ------------
            if (p7r_d2 && !p7r_seen_c) begin
                p7r_seen_c    <= 1'b1;
                p7r_y0        <= y_bus[9:0];
                p7r_y3        <= y_bus[39:30];
                p7r_state[0]  <= y_valid_bus[0];
                p7r_state[1]  <= y_valid_bus[3];
                // Both lanes, and both valids: a lane whose `y_valid` never
                // rose has a `y` from the reset state, which is a zero that
                // looks exactly like a correct clamp.
                p7r_ok[2]     <= (y_bus[9:0] == P7R_Y0) & y_valid_bus[0]
                              && (y_bus[39:30] == P7R_Y3) & y_valid_bus[3];
            end
        end
    end

    assign t_p7r_acc   = p7r_acc;
    assign t_p7r_m     = p7r_m;
    assign t_p7r_shift = p7r_shift;
    assign t_p7r_zero  = p7r_zero;
    assign t_p7r_y0    = p7r_y0;
    assign t_p7r_y3    = p7r_y3;
    assign t_p7r_state = p7r_state;
    assign t_p7r_seen  = {p7r_seen_c, p7r_seen_a, p7r_seen_a};
    assign t_p7r_ok    = p7r_ok;

    // Both clients connect directly to the shared bank's response buses below.

    // ------------------------------------------------------------------
    // the clients
    // ------------------------------------------------------------------

    // `WT_INIT_FILE` is deliberately empty: the encoder has no store here, this
    // module owns the one image.  `CFG_INIT_FILE` is the caller's, because
    // the encoder's requant table is still the encoder's.
    generate if (ENABLE_ENCODER) begin : g_encoder
    d64_encoder #(
        .FAULT_STICKY(FAULT_STICKY),.RUNTIME_PROFILE(RUNTIME_PROFILE), .HEAD_STORE_CREDIT_ENABLE(HEAD_STORE_CREDIT_ENABLE), .WT_INIT_FILE(""), .CFG_INIT_FILE(CFG_INIT_FILE),
                  .EXTERNAL_PE(1),
                  // **`H` and `W` are derived from this module's own `ROWS`
                  // and `COLS`.**  Those two are the residual tensor's shape
                  // (`:129`, `:130`); the image is four times each, because
                  // DOWN1 is the image after two stride-2 downsamplings and
                  // two transposed convolutions bring it back up.  One shape,
                  // one pair of parameters, reaching both halves -- the
                  // scheduler is already given the same pair on `:1502`.
                  // Without this the head window and head controller sit on
                  // their `H = 256, W = 256` defaults while the scheduler
                  // walks `ROWS` rows.  At `ROWS = 64, COLS = 64` both
                  // defaults, this passes `256` and `256`: bit-identical.
                  .H(ROWS * 4), .W(COLS * 4)) u_enc (
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        // **`enc_rst_n` and not `pe_rst_n`.**  This is the re-arm's clear, and
        // the encoder's reset is the only net in the module it moves.
        .clk(clk_pe), .rst_n(enc_rst_n), .start(enc_start),
        .d64_valid(d64_valid), .d64_ready(d64_ready), .d64_frame_id(d64_frame_id),
        .d64_sof(d64_sof), .d64_x0(d64_x0), .d64_y(d64_y),
        .d64_rgb888(d64_rgb888), .d64_eol(d64_eol), .d64_eof(d64_eof),
        // No store here to load: this module owns the one image, and feeding
        // the load beats in as well would be the silent second copy.
        .wt_wr_en(1'b0), .wt_wr_addr(8'd0), .wt_wr_data(1024'd0),
        .cfg_wr_en(e_cfg_wr_en), .cfg_wr_addr(e_cfg_wr_addr),
        .cfg_wr_data(e_cfg_wr_data),
        .pe_req_slot_valid(e_pe_slot_valid), .pe_req_act(e_pe_act),
        .pe_req_wgt(e_pe_wgt), .pe_req_pix_start(e_pe_pix_start),
        .pe_req_bias(e_pe_bias), .pe_req_rq_start(e_pe_rq_start),
        .pe_req_rq_m(e_pe_rq_m), .pe_req_rq_shift(e_pe_rq_shift),
        .pe_req_rq_zero(e_pe_rq_zero), .pe_req_rq_qmin(e_pe_rq_qmin),
        .pe_req_rq_qmax(e_pe_rq_qmax),
        .pe_rsp_rq_ready(rq_ready), .pe_rsp_rq_ready_n(rq_ready_n),
        .pe_rsp_y(y_bus), .pe_rsp_y_valid(y_valid_bus),
        .pe_rsp_rq_busy(rq_busy), .pe_rsp_rq_done(rq_done),
        .pe_rsp_rq_overrun(rq_overrun), .pe_rsp_mode_viol(bank_mode_viol),
        .pe_rsp_acc(acc_bus), .pe_rsp_snapshot(snap_bus),
        .pe_wt_rd_en(e_pe_wt_rd_en), .pe_wt_rd_addr(e_pe_wt_rd_addr),
        .pe_wt_rd_data(e_pe_wt_rd_data),
        .head_store_wr_valid(head_store_wr_valid),
        .head_store_wr_ready(head_store_wr_ready),
        .head_phase_ready(head_phase_ready),
        .head_store_wr_x(head_store_wr_x), .head_store_wr_y(head_store_wr_y),
        .head_store_wr_c8(head_store_wr_c8),
        .head_store_wr_frame_id(head_store_wr_frame_id),
        .head_store_wr_sof(head_store_wr_sof),
        .head_store_wr_eol(head_store_wr_eol),
        .head_store_wr_eof(head_store_wr_eof),
        .head_store_commit_ready(head_store_commit_ready),
        .head_read_start(head_read_start), .head_read_valid(head_read_valid),
        .head_read_ready(head_read_ready), .head_read_x(head_read_x),
        .head_read_y(head_read_y), .head_read_c8(head_read_c8),
        .head_read_frame_id(head_read_frame_id), .head_read_sof(head_read_sof),
        .head_read_eol(head_read_eol), .head_read_eof(head_read_eof),
        .down0_store_wr_valid(down0_store_wr_valid),
        .down0_store_wr_ready(down0_store_wr_ready),
        .down0_store_wr_x(down0_store_wr_x), .down0_store_wr_y(down0_store_wr_y),
        .down0_store_wr_word(down0_store_wr_word),
        .down0_store_wr_data(down0_store_wr_data),
        .down0_store_wr_frame_id(down0_store_wr_frame_id),
        .down0_store_wr_sof(down0_store_wr_sof),
        .down0_store_wr_eol(down0_store_wr_eol),
        .down0_store_wr_eof(down0_store_wr_eof),
        .down0_store_commit_ready(down0_store_commit_ready),
        .down0_read_start(down0_read_start), .down0_read_valid(down0_read_valid),
        .down0_read_ready(down0_read_ready), .down0_read_x(down0_read_x),
        .down0_read_y(down0_read_y), .down0_read_icg(down0_read_icg),
        .down0_read_c8(down0_read_c8),
        .down0_read_frame_id(down0_read_frame_id),
        .down0_read_sof(down0_read_sof), .down0_read_eol(down0_read_eol),
        .down0_read_eof(down0_read_eof),
        .out_valid(out_valid), .out_ready(out_ready), .out_x(out_x),
        .out_y(out_y), .out_word(out_word), .out_data(out_data),
        .out_frame_id(out_frame_id), .out_sof(out_sof), .out_eol(out_eol),
        .out_eof(out_eof),
        .busy(enc_busy), .done(enc_done), .state(enc_state),
        .pe_mode(e_pe_mode), .mac_mode_safe(enc_mac_mode_safe),
        .mode_viol(enc_mode_viol), .layer_mode_viol(layer_mode_viol),
        .frame_id_violations(frame_id_violations),
        .head_store_overflow(head_store_overflow),
        .head_mac_slots(head_mac_slots), .down_mac_slots(down_mac_slots),
        .head_rq_overrun(head_rq_overrun), .down_rq_overrun(down_rq_overrun),
        .head_rq_wait(head_rq_wait), .down_rq_wait(down_rq_wait),
        .down_read_before_ready(down_read_before_ready),
        .head_layer_cycles(head_layer_cycles),
        .down0_layer_cycles(down0_layer_cycles),
        .down1_layer_cycles(down1_layer_cycles),
        .head_compute_cycles(head_compute_cycles),
        .down0_compute_cycles(down0_compute_cycles),
        .down1_compute_cycles(down1_compute_cycles),
        .encoder_compute_cycles(encoder_compute_cycles),
        .encoder_store_wait_cycles(encoder_store_wait_cycles),
        .encoder_wall_cycles(encoder_wall_cycles),
        .head_accepted_beats(head_accepted_beats),
        .head_accepted_rows(head_accepted_rows),
        .head_marker_violations(head_marker_violations),
        .head_read_before_ready(head_read_before_ready)
    );
    end else begin : g_encoder_disabled
        assign enc_busy = 1'b0;
        assign d64_ready = 1'b0;
        assign enc_done = 1'b0;
        assign down0_compute_cycles = 1'b0;
        assign down0_layer_cycles = 1'b0;
        assign down0_read_ready = 1'b0;
        assign down0_read_start = 1'b0;
        assign down0_store_wr_data = 1'b0;
        assign down0_store_wr_eof = 1'b0;
        assign down0_store_wr_eol = 1'b0;
        assign down0_store_wr_frame_id = 1'b0;
        assign down0_store_wr_sof = 1'b0;
        assign down0_store_wr_valid = 1'b0;
        assign down0_store_wr_word = 1'b0;
        assign down0_store_wr_x = 1'b0;
        assign down0_store_wr_y = 1'b0;
        assign down1_compute_cycles = 1'b0;
        assign down1_layer_cycles = 1'b0;
        assign down_mac_slots = 1'b0;
        assign down_read_before_ready = 1'b0;
        assign down_rq_overrun = 1'b0;
        assign down_rq_wait = 1'b0;
        assign encoder_compute_cycles = 1'b0;
        assign encoder_store_wait_cycles = 1'b0;
        assign encoder_wall_cycles = 1'b0;
        assign frame_id_violations = 1'b0;
        assign head_accepted_beats = 1'b0;
        assign head_accepted_rows = 1'b0;
        assign head_compute_cycles = 1'b0;
        assign head_layer_cycles = 1'b0;
        assign head_mac_slots = 1'b0;
        assign head_marker_violations = 1'b0;
        assign head_read_before_ready = 1'b0;
        assign head_read_ready = 1'b0;
        assign head_read_start = 1'b0;
        assign head_rq_overrun = 1'b0;
        assign head_rq_wait = 1'b0;
        assign head_store_overflow = 1'b0;
        assign head_store_wr_c8 = 1'b0;
        assign head_store_wr_eof = 1'b0;
        assign head_store_wr_eol = 1'b0;
        assign head_store_wr_frame_id = 1'b0;
        assign head_store_wr_sof = 1'b0;
        assign head_store_wr_valid = 1'b0;
        assign head_store_wr_x = 1'b0;
        assign head_store_wr_y = 1'b0;
        assign layer_mode_viol = 1'b0;
        assign enc_mac_mode_safe = 1'b0;
        assign enc_mode_viol = 1'b0;
        assign out_data = 1'b0;
        assign out_eof = 1'b0;
        assign out_eol = 1'b0;
        assign out_frame_id = 1'b0;
        assign out_sof = 1'b0;
        assign out_valid = 1'b0;
        assign out_word = 1'b0;
        assign out_x = 1'b0;
        assign out_y = 1'b0;
        assign e_pe_mode = 1'b0;
        assign e_pe_act = 1'b0;
        assign e_pe_bias = 1'b0;
        assign e_pe_pix_start = 1'b0;
        assign e_pe_rq_m = 1'b0;
        assign e_pe_rq_qmax = 1'b0;
        assign e_pe_rq_qmin = 1'b0;
        assign e_pe_rq_shift = 1'b0;
        assign e_pe_rq_start = 1'b0;
        assign e_pe_rq_zero = 1'b0;
        assign e_pe_slot_valid = 1'b0;
        assign e_pe_wgt = 1'b0;
        assign e_pe_wt_rd_addr = 1'b0;
        assign e_pe_wt_rd_en = 1'b0;
        assign enc_state = 1'b0;
    end endgenerate

    d64_trunk_blocks #(.RUNTIME_PROFILE(RUNTIME_PROFILE), .CFG_BIAS_W(CFG_BIAS_W), .CFG_STREAM_SRL(CFG_STREAM_SRL),
        .ROWS(ROWS), .COLS(COLS), .BAND(BAND), .A_ROWS(A_ROWS), .B_ROWS(B_ROWS),
        .X_RING_ROWS(X_RING_ROWS), .RF_AW(RF_AW), .WF_AW(WF_AW),
        .TRUNK_FIRST_BLOCK(TRUNK_FIRST_BLOCK), .TRUNK_BLOCK_COUNT(TRUNK_BLOCK_COUNT),
        .NB(NB), .CFG_NB(CFG_NB), .WBASE0(WBASE0), .WSTRIDE(WSTRIDE),
        .CFG_AW(T_CFG_AW), .BLK_W(BLK_W),
        .W_TILES(W_TILES), .W_WORD_W(W_WORD_W), .W_ADDR_W(W_ADDR_W),
        .C1_CYC(C1_CYC), .C2_CYC(C2_CYC), .C3_CYC(C3_CYC), .C3_IC(C3_IC),
        .Z1(Z1), .ZERO_T(ZERO_T), .EXTERNAL_PE(1),
        .C2_DIAG(C2_DIAG), .A_ROM_INIT(A_ROM_INIT), .B_ROM_INIT(B_ROM_INIT),
        .CHUNK_ROM_INIT(CHUNK_ROM_INIT)
    ) u_trk (
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .clk_pe(clk_pe), .clk_axi(clk_axi), .arst_n(arst_n),
        .start(trk_start), .busy(t_busy), .done(t_done_w), .cycles(t_cycles),
        .blk(t_blk_w), .src_buf(t_src_w), .dst_buf(t_dst_w),
        .x_req_buf(t_req_buf_w),
        .w_we(1'b0), .w_wa(8'd0), .w_wd(1024'd0),
        .c_we(t_cfg_we), .c_wa(t_cfg_wa), .c_wd(t_cfg_wd),
        .trk_add_cnt(t_add_cnt), .trk_add_diff(t_add_diff),
        .trk_no_yv(t_no_yv), .trk_zero_br(t_zero_br),
        .trk_tw_cnt(t_tw_cnt),
        .trk_cfg_ok(t_cfg_ok), .trk_stream_ok(t_stream_ok), .cfg_protocol_fault(cfg_protocol_fault),
        .cfg_selftest(t_cfg_selftest), .cfg_image_ok(t_cfg_image_ok),
        .cfg_selftest_done(t_cfg_selftest_done),
        .trk_refill_c3(t_refill_c3), .trk_refill_seen(t_refill_seen),
        .trk_refill_ok(t_refill_ok),
        .trk_obs_chunk(t_obs_chunk), .trk_c2_chunk(t_c2_chunk), .trk_c2_dbg(t_c2_dbg),
        .blk_redo(blk_redo),
        .trk_redo_cnt(t_redo_cnt), .trk_redo_seen(t_redo_seen),
        .trk_redo_full(t_redo_full), .trk_redo_bound(t_redo_bound),
        .trk_eng_bad(t_eng_bad),
        .trk_p7_ok(t_p7_ok),
        .trk_p7_seen(t_p7_seen), .trk_p7_acc(t_p7_acc),
        .trk_p7_fs(t_p7_fs), .trk_p7_biaseen(t_p7_biaseen),
        .trk_p7_cfg_seen(t_p7_cfg_seen),
        .trk_p7_bias(t_p7_bias), .trk_p7_br(t_p7_br),
        .trk_p7_cfg0(t_p7_cfg0), .trk_p7_cfg1(t_p7_cfg1),
        .trk_p7_ctrl(t_p7_ctrl),
        .pe_req_slot_valid(t_pe_slot_valid), .pe_req_act(t_pe_act),
        .pe_req_wgt(t_pe_wgt), .pe_req_pix_start(t_pe_pix_start),
        .pe_req_bias(t_pe_bias), .pe_req_rq_start(t_pe_rq_start),
        .pe_req_rq_m(t_pe_rq_m), .pe_req_rq_shift(t_pe_rq_shift),
        .pe_req_rq_zero(t_pe_rq_zero), .pe_req_rq_qmin(t_pe_rq_qmin),
        .pe_req_rq_qmax(t_pe_rq_qmax),
        .pe_rsp_rq_ready(rq_ready), .pe_rsp_rq_ready_n(rq_ready_n),
        .pe_rsp_y(y_bus), .pe_rsp_y_valid(y_valid_bus),
        .pe_rsp_rq_busy(rq_busy), .pe_rsp_rq_done(rq_done),
        .pe_rsp_rq_overrun(rq_overrun), .pe_rsp_mode_viol(bank_mode_viol),
        .pe_rsp_acc(acc_bus), .pe_rsp_snapshot(snap_bus),
        .pe_wt_rd_en(t_pe_wt_rd_en), .pe_wt_rd_addr(t_pe_wt_rd_addr),
        .pe_wt_rd_data(t_pe_wt_rd_data),
        .x_req_row(x_req_row), .x_req_tog(x_req_tog), .x_ack_tog(x_ack_tog),
        .rf_wr_en(rf_wr_en), .rf_wr_data(rf_wr_data), .rf_wr_full(rf_wr_full),
        .wf_rd_en(wf_rd_en), .wf_rd_data(wf_rd_data),
        .wf_rd_valid(wf_rd_valid), .wf_rd_empty(wf_rd_empty), .wf_rd_level(wf_rd_level),
        .wr_done_tog(wr_done_tog),
        .cfg_cycles(t_cfg_cycles), .bar_cycles(t_bar_cycles),
        .clean_viol(t_clean_viol), .blk_starts(t_blk_starts),
        .compute_sum(t_compute_sum), .wait_cycles(t_wait_cycles),
        .x_ovf(t_x_ovf), .x_rbf(t_x_rbf), .rf_udf(t_rf_udf), .rf_wait(),
        .wf_ovf(t_wf_ovf), .dep_viol(t_dep_viol), .x_beats(), .wf_words(),
        .rq_overruns(t_rq_overruns), .out_last(t_out_last),
        .blk_clean(t_blk_clean), .clean_why(t_clean_why),
        .clean_first(t_clean_first), .cfg_writes(t_cfg_writes)
    );

    assign t_blk = t_blk_w;
    assign t_cfg_writes_o = t_cfg_writes;
    assign t_src_buf = t_src_w;
    assign t_req_buf = t_req_buf_w;
    assign t_dst_buf = t_dst_w;

    // ------------------------------------------------------------------
    // the owner sequence
    // ------------------------------------------------------------------
    //
    // Neither client is reset at the *handover*.  `d64_encoder` and
    // `d64_trunk_blocks` are each written to run one block sequence, and the
    // encoder's own `blk_clean`-style discipline is the trunk's -- what a
    // handover has to leave behind is a drained array and nothing in flight,
    // which is exactly what `enc_mode_safe` asserts before the mux moves.
    //
    // **A frame boundary is not a handover, and it is handled differently.**
    // The encoder is reset in `O_CLR`: its head sequencer has no other route
    // back to zero, and at `EXTERNAL_PE = 1` there is no configuration inside
    // it for a reset to take.  The trunk is *not* reset -- it is re-entered
    // through its own `S_DONE`-as-idle -- because its barrier's `wd_seen` is a
    // two-flop record of a level the far side holds, and clearing it would read
    // that standing level as a fresh write-done edge.  See the re-arm block
    // above for both arguments in full.

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            begin owner_q <= O_IDLE; sel_enc_q <= 1'b0; end
            enc_start <= 1'b0; trk_start <= 1'b0;
            enc_started <= 1'b0; trk_started <= 1'b0;
            out_committed <= 1'b0;
            owner_viol <= 32'd0; cyc <= 32'd0; hand_cycles <= 32'd0;
            frame_cnt <= 32'd0; rearm_early <= 32'd0;
        end
        else begin
            enc_start <= 1'b0;
            trk_start <= 1'b0;
            if (owner_q != O_IDLE && owner_q != O_DONE) cyc <= cyc + 32'd1;

            // The encoder's last trunk-A word, taken by the memory side.  It is
            // the *stream* handshake and not `done`, for the reason D3c5-b1
            // gave: the schedule finishing and the last word existing are a
            // cycle apart, and the trunk reads precisely that word.
            if (out_valid && out_ready && out_eof) out_committed <= 1'b1;

            if (enc_slot_while_trk || trk_slot_while_enc)
                owner_viol <= FAULT_STICKY ? 32'd1 : owner_viol + 32'd1;

            if (start_unserved) rearm_early <= rearm_early + 32'd1;

            case (owner_q)
                O_IDLE: if (start_edge) begin
                    begin owner_q <= ENABLE_ENCODER ? O_ENC : O_TRK; sel_enc_q <= ENABLE_ENCODER; end
                    enc_start <= ENABLE_ENCODER;
                    enc_started <= ENABLE_ENCODER;
                    if (!ENABLE_ENCODER) begin trk_start <= 1'b1; trk_started <= 1'b1; end
                    out_committed <= 1'b0;
                    owner_viol <= 32'd0;
                    cyc <= 32'd0;
                    frame_cnt <= frame_cnt + 32'd1;
                end

                // **Both conditions, and they are not the same condition.**  A
                // mode that did not change is not a pipeline that is empty:
                // down1 and the trunk's conv1 are both mode 0, and handing the
                // array over on that alone would move it while a slot was still
                // in flight.
                O_ENC: if (enc_owner_safe && enc_mode_safe) begin
                    begin owner_q <= O_TRK; sel_enc_q <= 1'b0; end
                    trk_start <= 1'b1;
                    trk_started <= 1'b1;
                    hand_cycles <= cyc;
                end

                // **The edge and not the level.**  `t_done_w` is
                // `(state == S_DONE)` and the trunk is *still* in `S_DONE` when
                // a new frame reaches here, so the level test would retire the
                // frame before the trunk had run it.  See the re-arm block
                // above.
                O_TRK: if (trk_done_edge) begin owner_q <= O_DONE; sel_enc_q <= 1'b0; end

                // The re-arm.  `rearm_ok` is the whole of the condition.
                O_DONE: if (start_edge && rearm_ok) begin owner_q <= O_CLR; sel_enc_q <= 1'b0; end

                // **One cycle, and the encoder is in reset for all of it.**
                // `enc_start` is registered and is therefore asserted on the
                // cycle *after* this one, when `enc_rst_n` is already high --
                // a start delivered while the module that is supposed to see it
                // is still held in reset is a start nobody sees, which is what
                // a one-pulse-and-go form of this would have produced.
                O_CLR: begin
                    begin owner_q <= ENABLE_ENCODER ? O_ENC : O_TRK; sel_enc_q <= ENABLE_ENCODER; end
                    enc_start <= ENABLE_ENCODER;
                    enc_started <= ENABLE_ENCODER;
                    if (!ENABLE_ENCODER) begin trk_start <= 1'b1; trk_started <= 1'b1; end
                    out_committed <= 1'b0;
                    owner_viol <= 32'd0;
                    cyc <= 32'd0;
                    frame_cnt <= frame_cnt + 32'd1;
                end

                default: ;
            endcase
        end
    end

endmodule
