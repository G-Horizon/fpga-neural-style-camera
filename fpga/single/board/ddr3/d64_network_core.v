`timescale 1ns/1ps

// ===========================================================================
// F3 / root-project: the fused network core -- the network's compute top.
// ===========================================================================
//
// Date: 2026-10-07.  Branch `debug/integrate`.
//
// This is the module the root-project review calls `d64_network_core`
// (`docs/root_project_2026-10-07/ROOT_INTEGRATION.md` section 1): the four PE
// owners, the one bank, the one weight store, the one runtime cache, the
// decoder's engines and their writers, the job sequencer and the fault side.
// It is **compute plus abstract storage/transaction service** and it binds no
// board pin: the DDR PHY, the PLLs, the camera and the display belong to
// `d64_network_board_top`, which is the next increment and not this one.
//
// Why it is in `board/ddr3/`
// --------------------------
// F3 section 3.13 fixes the network root there: the DDR IP's relative path
// length is load-bearing (`map.v`'s encrypted payload changes with it), and
// this root instantiates four modules that already live in that directory --
// `d64_axi_read_burst`, `d64_ups_u16_writer`, `d64_rgbx_beat_writer`,
// `d64_done_tracker`.  A new `board/root/` would be a second convention for
// the same thing.
//
// What is REAL here, module by module
// -----------------------------------
//   real  control/d64_encoder_trunk.v      EXTERNAL_PE=1, EXTERNAL_WSTORE=1,
//                                          ACC_W=25 -- the front half as owner 0
//   real  control/d64_pe_owner.v           NOWNERS=4
//   real  control/d64_acc25_bridge.v       seam 32 -> bank 25, per lane
//   real  control/d64_mac128.v             one bank, ACC_W=25, OUT_W=10
//   real  memory/d64_weight_store.v        247 tiles x 1024 bits, one read port
//   real  control/d64_decoder_wreader.v    the read port's owner mux + cfg ROMs
//   real  control/d64_tconv_engine.v   x2  ups.0 and ups.1, EXTERNAL_PE=1
//   real  board/ddr3/d64_tail_output_chain.v  the tail engine, its RGBX writer
//                                          and its done tracker, joined
//   real  control/d64_burst_prefetch.v     the runtime row cache, IN_PORTS=1
//   real  board/ddr3/d64_axi_read_burst.v  the cache's DDR side
//   real  board/ddr3/d64_ups_u16_writer.v x2  UPS0 and UPS1 into DDR
//   real  control/d64_net_addr.v           the region map, instantiated
//   real  control/d64_axi_reset_ctl.v      the coordinated AXI-domain reset
//
// There are **no stubs and no models** in this file.  `d64_mac128 1` and
// `d64_weight_store 1` are the counts the joint gate established
// (F3 6.17), and the testbench counts them by elaborated instance.
//
// ---------------------------------------------------------------------------
// What is deliberately NOT built, and why -- read this before adding to it
// ---------------------------------------------------------------------------
//
// 1. **The multi-port cache.**  `d64_burst_prefetch` is instantiated once, at
//    its default `IN_PORTS = 1` -- one port, 8 RAM10 -- and that one port is
//    UPS0's.  UPS0 is the only one of the three decoder consumers whose
//    store-facing side is one port (`docs/F3_INTERFACES_2026-10-07.md` 6.9:
//    "UPS0 reads one port, UPS1 two, TAIL six").  A six-port cache is 48 RAM10
//    against a 30-block decoder margin and it does not fit; more to the point
//    another agent is measuring whether a rolling three-row window removes the
//    requirement altogether, and building the thing that may be about to become
//    unnecessary is the wrong order.  **So UPS1's two ports and the tail's six
//    are on this module's boundary** (`u1_in_*`, `tl_in_*`), which is where the
//    answer of that measurement lands without re-wiring this file.
//    Consequence, stated rather than discovered: **the frame runs through
//    UPS0 and stops there.**  UPS1 and the tail are instantiated, wired to the
//    real bank, the real store and their real writers, and cannot be driven
//    until the store question is settled, because a two-port store fanned from
//    a one-port cache returns the wrong word quietly.
//
// 2. **The front half's DDR service.**  `d64_encoder_trunk`'s HEAD / DOWN0 /
//    TRUNK stream boundary is on this module's ports and is served by
//    `d64_encoder_ddr_mem` + `d64_trunk_row_ddr`, which are already gated
//    (`tools/run_encoder_trunk_board_model.sh`) and are a separate increment.
//    The reason is the plan's own two-layer split, quoted above: the core holds
//    the computation and the abstract transaction service, the shell holds the
//    physical side.  The front half is nevertheless **instantiated for real and
//    is owner 0 of the shared bank**, which is the whole point of the fusion.
//
// 3. **`d64_axi_reset_ctl` is wired, and the request it is wired to is a
//    pulse.**  F3 6.19a is explicit: the release runs on `req_n = arst_n &
//    ~req`, so the requester owes the request's withdrawal.  A level held until
//    the sequence finishes **deadlocks** -- measured, not argued.  The FSM below
//    therefore counts resets and pulses `req` for exactly one cycle on the
//    fault's rising edge, and the pulse width is a number the testbench reads
//    (`reset_pulses`).
//
// ---------------------------------------------------------------------------
// The bank's `RQ_LOOKAHEAD`, and the one number this root cannot make uniform
// ---------------------------------------------------------------------------
// F3 6.16 settled it and left it per-owner: the front half needs 3 and the
// tconv needs 4, the prediction is computed *inside* the bank from a *bank*
// parameter, and nothing in the section 1 seam lets an owner say which engine
// is asking.  This root builds the bank at `BANK_RQLA = 3` -- the front half's
// number and the shipped board's -- and records that the tconv is therefore one
// cycle short here rather than silently picking a side.  On every frozen
// stimulus in this repository the parameter is inert (6.16's table: `rq_wait` 0
// and `rq_overrun` 0 at every setting tried), so this is a recorded margin and
// not a live defect.  `BANK_RQLA` is a parameter so the arm can be rebuilt the
// other way.
//
// ---------------------------------------------------------------------------
// The clock scope -- WHICH PHYSICAL CLOCK SERVES THE AXI PORT
// ---------------------------------------------------------------------------
// Recorded here because **a testbench tie is not a specification**.  The root's
// own gate drives `clk_pe` and `clk_axi` from one `always #4.1667` in
// `board/ddr3/tb_d64_network_core.v`, which says both are 120 MHz and says
// nothing whatever about whether they are one clock.  Same frequency is not the
// same domain, so the scope is stated and then checked rather than assumed:
//
//     clk_pe is the physical clock of every AXI port of this module.
//
// Every AXI master port on the boundary below is the port list of an instance
// clocked by `clk_pe` and by nothing else -- not through an `assign`, not
// through a mux and not through any other instance:
//
//   rd_axi_*  board/ddr3/d64_axi_read_burst.v     u_rdaxi   .clk(clk_pe)
//   w0_axi_*  board/ddr3/d64_ups_u16_writer.v     u_w0      .clk(clk_pe)
//   w1_axi_*  board/ddr3/d64_ups_u16_writer.v     u_w1      .clk(clk_pe)
//   tw_axi_*  board/ddr3/d64_tail_output_chain.v  u_tail    .clk(clk_pe)
//
// `tools/check_clock_scope.py` parses **this file**, finds the instance that
// drives each of those ports, and requires both halves of that sentence;
// `tools/run_iface_clock_scope.sh` runs it against the real file and against
// two sabotaged copies, each of which must fail.  A static check is the right
// instrument here and a simulation arm is not: no stimulus can make a
// clock-domain mistake visible in a run whose two clock inputs come off the
// same `always`.
//
// `clk_axi` is a **second name on the same physical clock**.  It reaches exactly
// two instances and no AXI master:
//
//   u_rst    control/d64_axi_reset_ctl.v   the AXI side of the coordinated reset
//   u_front  control/d64_encoder_trunk.v   the front half's own crossing, which
//                                          is a real crossing *inside* that
//                                          module and is F3 3.11/3.12's subject
//
// A shell that instantiates this module therefore owes one of two things, and
// this module provides neither:
//
//   (a) **tie `clk_axi` to the same physical clock as `clk_pe`** -- the
//       declaration this root is written to; or
//   (b) add an explicit CDC on every one of the AXI command, write-data,
//       response and commit paths listed above, which is a change to those
//       four modules and not to this one.
//
// What must not be done is what the root's first gate does: put two clock port
// names on one `always` and call the crossing closed.  The root is correct
// under (a) and says so; it makes no CDC claim at all.
//
// ---------------------------------------------------------------------------
// Verilog-2001, and the conventions that keep it elaborating
// ---------------------------------------------------------------------------
// * Every identifier is declared before it is first used in a port
//   connection.  A name first seen inside a connection is an implicit 1-bit
//   net, and a wide probe read from a 1-bit net is an error nowhere in this
//   flow.
// * No zero-width replication, for the reason F2 section 0 gives.
// * A compile is not an elaboration: `tools/check_elaborates.py` is run inside
//   `tools/run_network_core.sh` before any arm.
//
// No synthesis, no place-and-route, no bitstream.  The tail engine's
// `efx_map` crash (VDB-1011 then EXCEPTION_ACCESS_VIOLATION on the
// variable-index read of `rf` inside an `always @*`) is task #130 and is the
// first thing that will stop this root going through PNR.  It does not stop it
// simulating.
// ===========================================================================

module d64_network_core #(
    parameter integer FAULT_STICKY = 0,
    // **The frame's shape, and this is the only place it is stated.**
    //
    // The network's target is `256 x 144`: the sensor is 2560x1440, the project's
    // decision doc fixes the network at 256x144, and the display is 1280x720.
    // `256 x 256` is the accident -- `control/d64_camera_ingress_contract.vh` v1,
    // which nothing consumes, and the stale `ROWS = 64` default here.
    //
    // **Only `H` moves.**  Every width stays where it is, which is why the
    // row-level constants elsewhere (`d64_ups_u16_writer`'s `ROW_WORDS`, the two
    // writers' `BURST_BEATS`) are not part of the shape.
    //
    // The pyramid law, and it is the law `d64_net_addr`'s defaults already
    // state (see `docs/RECT_H144_2026-10-08.md`, `H = 4 * ROWS`):
    //
    //     RES4 = (H/4, W/4)     UPS0 = (H/2, W/2)     UPS1 = TAIL = (H, W)
    //
    // It is forwarded to **every** child that has a shape: `u_u0`, `u_u1`,
    // `u_tail` and the root's own `u_map`.  A child left at its default is a
    // child that silently disagrees with the map -- which is exactly the defect
    // this parameter replaces: before it, `.H(64), .W(64)` and `.H(128),
    // .W(128)` were literals, the chain had no shape at all, and `u_map` was
    // default-configured.  `tools/check_shape_forwarding.py` is the static guard.
    //
    // The rectangle regions all fit their existing slopes, so the map's
    // 256 KiB-aligned bases do not move.  See section 12 of
    // `docs/BATCH_B_TAIL_HOP_2026-10-08.md`.
    parameter integer IMG_H = 256,
    parameter integer IMG_W = 256,
    parameter WT_INIT_FILE = "",
    parameter integer RUNTIME_PROFILE = 0,
    parameter integer TRUNK_CFG_BIAS_W = 32,
    parameter integer CFG_STREAM_SRL=0,
    parameter integer HEAD_STORE_CREDIT_ENABLE=0,
    parameter integer DEC_CFG_BIAS_W = 32,
    parameter DEC_CFG_INIT = "../../golden/d64_decoder_cfg.mem",
    parameter DEC_BIAS_INIT = "../../golden/d64_decoder_bias.mem",

    // The bank's requant lookahead.  3 is the front half's, the trunk's and the
    // board's; 4 is the tconv's.  See the header.
    parameter integer BANK_RQLA = 3,

    // The cache's line and burst geometry, from `d64_net_addr`'s policy.  A row
    // of every tensor the decoder reads is 2048 bytes at `RES4_ELEM_B = 1` and
    // 4096 at 2; the U16 transport image makes it 4096, which is 256 beats of
    // sixteen bytes.  One burst is one row, so no row straddles a burst.
    parameter integer LINE_WORDS  = 256,
    parameter integer LINES       = 3,
    parameter integer BURST_WORDS = 256,

    // The AXI adapter's watchdog, in `clk_axi` cycles.
    parameter integer RD_TIMEOUT  = 2048,

    // The tail writer's bus geometry, through `d64_tail_output_chain`.  64 is
    // the writer's own default and the number `docs/TAIL_WRITE_BACKPRESSURE`
    // measured a 600-cycle window absorbed with; the chain's shipped 1 gave a
    // four-beat credit and lost writes (issue #109).
    parameter integer BURST_BEATS = 64,
    parameter integer TAIL_RESERVE_WRITE_BURST = 0,
    parameter integer BQ_DEPTH    = 4,

    // Verification only, folded away at 0.  The root's own two teeth:
    //   SAB_CACHE_REGION   the region handed to the cache is the byte address
    //                      instead of the word address, so every fetch lands
    //                      16x past the tensor and the read returns a word from
    //                      the wrong row -- a failure on data.
    //   SAB_STORE_HANDOVER the store's read port is handed over on the stage
    //                      boundary alone, with the whole quiescence licence
    //                      removed (see "the hand-over licence" below and F3
    //                      6.21.4).
    //   SAB_TL_FIXTURE     the TAIL's input comes from the boundary ports
    //                      (`tl_in_rd_data`/`tl_in_stall`) instead of the one
    //                      runtime cache through its own six-port adapter.
    //                      **0 is the design**: the tail is the job's third
    //                      hop and reads the image UPS1 wrote to DDR in this
    //                      same job.  1 is the pre-hop build, kept as a
    //                      verification arm so that a green run can be paired
    //                      with one that goes red -- it is the fixture this
    //                      arm exists to replace.  Its gate is
    //                      `board/ddr3/tb_d64_root_tail_fault.v`, run by
    //                      `tools/run_root_tail_fault.sh`.
    //   SAB_SHAPE_STALE    the pre-change build, kept as the control this
    //                      change exists to replace: every child is instantiated
    //                      at the SQUARE shape regardless of `IMG_H`/`IMG_W`,
    //                      exactly as the four literals and the two
    //                      default-configured instances had it.  **0 is the
    //                      design.**  A green rectangle run cannot show that its
    //                      shape came from the parameter; a green run with a
    //                      stale arm that goes red can.
    parameter integer SAB_CACHE_REGION   = 0,
    parameter integer SAB_STORE_HANDOVER = 0,
    parameter integer SAB_TL_FIXTURE     = 0,
    parameter integer SAB_SHAPE_STALE    = 0
    ,
    // ==================================================================
    // THE FUSION -- batch H, 2026-10-08
    // ==================================================================
    // **0 is the pre-fusion root and 1 fuses the block in.** The two switches
    // exist for one measured reason: `vopt` does not resolve a module reference
    // inside a FALSE `generate`, so a reference at 0 costs an existing gate
    // nothing and the same reference at 1 is a hard elaboration error.  That was
    // measured in `sim/_agent_rootfusion/genrepro/` before this file was edited,
    // both halves -- a false branch with an absent module PASSES elaboration and
    // a true branch with the same absent module FAILS with
    // `Module 'totally_absent_module' is not defined`.
    //
    // FUSE_CAM  the camera front end (`cam_csi2_top`: the I2C master and the
    //           SC431HAI register table), the contract-A adapter
    //           (`board/cam/rtl/d64_csi2_raw_to_contract_a.v`) and the seam
    //           (`board/integrate/d64_cam_rgb256.v`).  At 1 the front half's
    //           contract-B inputs come from the seam; at 0 they come from this
    //           module's own `d64_*` ports, exactly as they did before.
    // FUSE_DISP the display chain (`d64_video_timing`, `d64_video_row_ddr`,
    //           `d64_rgbx_display_adapter`, `d64_video_scaler`, `dvi_encoder`)
    //           with its own AXI read master, reading the tail's region.
    parameter integer FUSE_CAM  = 0,
    parameter integer FUSE_DISP = 0,

    // The camera's mode and CFA phase -- shell configuration, as the contract
    // says ("the phase is programmed and validated by firmware before capture is
    // enabled").  No source in this tree establishes the SC431HAI's phase.
    parameter integer CAM_W      = 1920,
    parameter integer CAM_H      = 1080,
    parameter [1:0]   CAM_CFA    = 2'd0,
    parameter [9:0]   CAM_BLACK  = 10'd64,
    parameter [15:0]  CAM_GAIN_Q8 = 16'd256,

    // The display raster: C1 of `docs/HDMI_30HZ_CANDIDATES_2026-10-07.md` --
    // VIC 62's totals at the 74.375 MHz pixel clock this board already has,
    // 30.0505 Hz.
    parameter integer H_ACT = 1280, parameter integer H_FP = 1760,
    parameter integer H_SYNC = 40,  parameter integer H_BP = 220,
    parameter integer V_ACT = 720,  parameter integer V_FP = 5,
    parameter integer V_SYNC = 5,   parameter integer V_BP = 20,
    // The region the display reads.  The tail's base, and the same number
    // `d64_tail_output_chain`'s own `TAIL_BASE` carries.
    parameter [31:0]  DISP_SRC_BASE = 32'h005C_0000
) (
    output wire [31:0] front_debug,
    output wire [31:0] trunk_debug,
    // **`clk_pe` is the physical clock of every AXI port of this module** and
    // `clk_axi` is a second name on the same physical clock -- it reaches
    // `u_rst` and `u_front` and no AXI master.  See "the clock scope" in the
    // header; a shell that drives a genuinely separate AXI clock owes a CDC on
    // each of those ports, and this module does not provide one.
    input  wire          clk_pe,
    input  wire          clk_axi,
    input  wire          arst_n,

    // ---- the job ----------------------------------------------------------
    // `start` is taken on its rising edge.  `stage_en` is the job's stage mask,
    // `{front, ups.0, ups.1, tail}`; a stage whose bit is clear is skipped and
    // the sequencer moves to the next one.  A descriptor field, not a mode
    // switch: `4'b0100` is UPS0 alone, which is what this increment can drive,
    // and `4'b0110` is both decoder layers.
    input  wire          start,
    input  wire [3:0]    stage_en,
    output reg  [2:0]    stage,        // 0 idle, 1..4 the stage running, 7 done
    output reg           busy,
    output reg           done,
    output wire          failed,

    // ---- the front half's stream boundary ---------------------------------
    // Served by `d64_encoder_ddr_mem` + `d64_trunk_row_ddr` in the shell.  The
    // names are `d64_encoder_trunk`'s own, so a wiring mistake is a name that
    // does not resolve rather than a name that resolves to the wrong thing.
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
    input  wire [8:0]    wt_wr_addr,
    input  wire [511:0] wt_wr_data,
    input  wire clk_weight_fast,
    input  wire [1:0] profile_style,
    input  wire [9:0] profile_trunk_zero,
    input  wire dec_cfg_we,
    input  wire [4:0] dec_cfg_addr,
    input  wire [63:0] dec_cfg_data,
    input  wire          t_cfg_we,
    input  wire [8:0]    t_cfg_wa,
    input  wire [63:0]   t_cfg_wd,
    input  wire          blk_redo,
    input  wire          t_cfg_selftest,
    input  wire          trunk_store_committed,

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

    output wire [6:0]    x_req_row,
    output wire          x_req_tog,
    output wire          t_req_buf,
    output wire          t_src_buf,
    output wire          t_dst_buf,
    output wire          front_done,
    input  wire          x_ack_tog,
    input  wire          rf_wr_en,
    input  wire [127:0]  rf_wr_data,
    output wire          rf_wr_full,
    input  wire          wf_rd_en,
    output wire [127:0]  wf_rd_data,
    output wire          wf_rd_valid,
    output wire          wf_rd_empty,
    // u_front uses the default WF_AW11; this level belongs to clk_axi.
    output wire [11:0]   wf_rd_level,
    input  wire          wr_done_tog,

    // ---- the decoder's store boundary, and what is deliberately not built ---
    // UPS1's two ports and the tail's six.  See note 1 in the header.
    output wire [1:0]        u1_in_rd_en,
    output wire [31:0]       u1_in_rd_addr,
    input  wire [127:0]      u1_in_rd_data,
    input  wire              u1_in_stall,
    output wire [5:0]        tl_in_rd_en,
    output wire [95:0]       tl_in_rd_addr,
    input  wire [383:0]      tl_in_rd_data,
    // **Added, and it is the same port `u1_in_stall` above has had all along.**
    // F3 6.9 amended `d64_tail_engine` to take `in_stall` and the chain did not
    // carry it (section 4.2), so the tail's input-accept path was unreachable
    // from here and the child's `in_stall` read `z` -- "never stall" -- in
    // whichever build this root was.  The chain now carries it, and this root
    // brings it out beside the three ports that were already on the boundary
    // (note 1: UPS1's two ports and the tail's six are the store this root does
    // not build), so the shell that points the tail at a real store drives one
    // more net and no other change is owed anywhere.
    input  wire              tl_in_stall,

    // ---- the cache's region, and the sequencer's own evidence --------------
    output wire [31:0]       cache_region_base,
    output wire [31:0]       cache_region_span,
    output wire              cache_region_req,
    output wire              cache_region_ack,
    output wire [31:0]       cache_region_waits,
    output wire [1:0]        dec_layer,       // the reader's window select
    output wire              store_owner,     // 0 the front half, 1 the decoder

    // ---- AXI4 masters, one per client, owned by the shell's arbiter ---------
    // Read: the cache's adapter.  Write: UPS0, UPS1 and the tail chain.
    //
    // **Every port in this group is registered in `clk_pe`**, from the instance
    // named in the header's clock-scope table: `u_rdaxi` for `rd_axi_*`, `u_w0`
    // for `w0_axi_*`, `u_w1` for `w1_axi_*` and `u_tail` for `tw_axi_*`.
    // `tools/check_clock_scope.py` enforces it; a shell that has one AXI clock
    // must make it this one.
    output wire          rd_axi_arvalid,
    input  wire          rd_axi_arready,
    output wire [31:0]   rd_axi_araddr,
    output wire [7:0]    rd_axi_arlen,
    output wire [2:0]    rd_axi_arsize,
    output wire [1:0]    rd_axi_arburst,
    input  wire          rd_axi_rvalid,
    output wire          rd_axi_rready,
    input  wire [127:0]  rd_axi_rdata,
    input  wire          rd_axi_rlast,
    input  wire [1:0]    rd_axi_rresp,

    output wire          w0_axi_awvalid,
    input  wire          w0_axi_awready,
    output wire [31:0]   w0_axi_awaddr,
    output wire [7:0]    w0_axi_awlen,
    output wire [2:0]    w0_axi_awsize,
    output wire [1:0]    w0_axi_awburst,
    output wire          w0_axi_wvalid,
    input  wire          w0_axi_wready,
    output wire [127:0]  w0_axi_wdata,
    output wire [15:0]   w0_axi_wstrb,
    output wire          w0_axi_wlast,
    input  wire          w0_axi_bvalid,
    output wire          w0_axi_bready,
    input  wire [1:0]    w0_axi_bresp,

    output wire          w1_axi_awvalid,
    input  wire          w1_axi_awready,
    output wire [31:0]   w1_axi_awaddr,
    output wire [7:0]    w1_axi_awlen,
    output wire [2:0]    w1_axi_awsize,
    output wire [1:0]    w1_axi_awburst,
    output wire          w1_axi_wvalid,
    input  wire          w1_axi_wready,
    output wire [127:0]  w1_axi_wdata,
    output wire [15:0]   w1_axi_wstrb,
    output wire          w1_axi_wlast,
    input  wire          w1_axi_bvalid,
    output wire          w1_axi_bready,
    input  wire [1:0]    w1_axi_bresp,

    output wire          tw_axi_awvalid,
    input  wire          tw_axi_awready,
    output wire [31:0]   tw_axi_awaddr,
    output wire [7:0]    tw_axi_awlen,
    output wire [2:0]    tw_axi_awsize,
    output wire [1:0]    tw_axi_awburst,
    output wire          tw_axi_wvalid,
    input  wire          tw_axi_wready,
    output wire [127:0]  tw_axi_wdata,
    output wire [15:0]   tw_axi_wstrb,
    output wire          tw_axi_wlast,
    input  wire          tw_axi_bvalid,
    output wire          tw_axi_bready,
    input  wire [1:0]    tw_axi_bresp,

    // ---- the coordinated AXI-domain reset ----------------------------------
    output wire          reset_req,       // a ONE-CYCLE pulse; see the header
    output wire          pe_arst_n,
    output wire          axi_arst_n,
    output wire          pe_busy,
    output wire          axi_busy,

    // ---- evidence ----------------------------------------------------------
    output wire [31:0]   frame_cnt,
    output wire [31:0]   owner_viol,
    output wire [31:0]   bank_mode_viol,
    output wire [31:0]   reset_pulses,
    output wire          dec_owns_store_seen,   // the store handover happened
    output wire [31:0]   ddr_reads,
    output wire [31:0]   ddr_fills,
    output wire [31:0]   ddr_bursts,
    output wire [31:0]   cache_stalls,
    output wire [31:0]   cache_viol,
    output wire [31:0]   ups0_words,
    output wire [31:0]   ups0_beats,
    output wire [31:0]   ups0_bursts,
    output wire [31:0]   ups0_acks,
    output wire [31:0]   ups0_bresp_errors,
    output wire [31:0]   ups0_overflows,
    output wire [31:0]   u0_slots,
    output wire [31:0]   u0_writes,
    output wire [31:0]   u0_rq_overrun,
    output wire [31:0]   u0_rq_wait,
    output wire [31:0]   u0_stall_cyc,
    output wire [31:0]   u1_slots,
    output wire [31:0]   tl_pixels,
    output wire [31:0]   tl_wr_responses,

    // ---- the job's verdict: THREE conditions and not one -------------------
    // `drained by count` is not success.  `d64_ups_u16_writer` retires and
    // acknowledges on a non-OKAY `bresp` as well as on an OKAY one -- its own
    // header says so and says why -- so `words == beats && bursts == acks` is
    // equally true of a transport that failed.  The aggregate is in the
    // sequencer below, named three times: `stage_finished`, then
    // `transport_retired`, then `stage_success`.
    output wire          stage_finished,
    output wire          transport_retired,
    output wire          stage_success,
    // **R6: the idle the caller needs.**  `admit_rejects` counts refusals but
    // nothing said when a job MAY be started, so a continuous caller had no
    // time reference at all.  `can_accept` is `job_idle` -- no stage running
    // and none pending -- brought out, so the shell can start frame N+1 on
    // the frame boundary it wants instead of discovering the refusal through
    // `admit_rejects`.  It is the admission rule above, published; it is not
    // a new rule.
    output wire          can_accept,

    // ---- the hold domain's evidence ----------------------------------------
    // **Reset by the external `arst_n` alone** (see the block at the end), so
    // every number here survives the reset the supervisor itself asked for.
    // That is the whole of defect 1: the request, its counter and the job's
    // failure record used to be held in `pe_rst_n`, and `pe_rst_n` is what the
    // request produces.  Measured on the micro-gate before this change:
    // `request_width_ns = 0`, `rises 1 sampled_high 0`, `retained_counter 0`.
    output wire [3:0]    reset_reason,       // which fault asked (sticky)
    output wire [31:0]   admit_rejects,      // `start` refused while busy
    output wire [31:0]   job_fails,          // stages that retired with a fault
    output wire [31:0]   ups0_duplicates,
    output wire [31:0]   ups0_oob,
    output wire [31:0]   ups1_bresp_errors,
    output wire [31:0]   ups1_overflows,
    output wire [31:0]   ups1_duplicates,
    output wire [31:0]   ups1_oob,
    // ==================================================================
    // THE CAMERA FRONT END, FUSED (FUSE_CAM) -- batch H
    // ==================================================================
    // **The vendor pad list, promoted rather than retyped.**  These are
    // `board/cam/rtl/cam_csi2_top.v`'s own 184 ports plus the six this batch
    // added to it, generated from that file's port list so no name is a second
    // copy.  It is the camera bring-up project's boundary and it binds board
    // pads -- which the header above says this module does not do.  That
    // tension is recorded rather than hidden: see
    // `docs/BATCH_H_ROOT_FUSION_2026-10-08.md` 3a.2.
    input  wire           i_arstn,
    input  wire           io_cam_scl_IN,
    input  wire           io_cam_sda_IN,
    input  wire           clk_100m,
    input  wire           pll_locked,
    input  wire           user_pll_locked,
    input  wire           i_pll_locked,
    input  wire           i_mipi_rx_pclk,
    input  wire           fb,
    input  wire           vid_clk_dvi2,
    input  wire           pll_inst4_CLKOUT0,
    input  wire           CLK_5M,
    input  wire           i_mipi_tx_pclk,
    input  wire           i_mipi_txc_sclk,
    input  wire           core_clk,
    input  wire           i_mipi_txd_sclk,
    input  wire           CLK_25M,
    input  wire           sdram_clk,
    input  wire           tx_cal_clk_90edge,
    input  wire           rx_cal_clk,
    input  wire           tx_cal_clk,
    input  wire           i_cam_ck_CLKOUT,
    input  wire           jtag_inst1_CAPTURE,
    input  wire           jtag_inst1_DRCK,
    input  wire           jtag_inst1_RESET,
    input  wire           jtag_inst1_RUNTEST,
    input  wire           jtag_inst1_SEL,
    input  wire           jtag_inst1_SHIFT,
    input  wire           jtag_inst1_TCK,
    input  wire           jtag_inst1_TDI,
    input  wire           jtag_inst1_TMS,
    input  wire           jtag_inst1_UPDATE,
    input  wire [15:0]    ddr_dq_in_hi,
    input  wire [15:0]    ddr_dq_in_lo,
    input  wire [1:0]     ddr_dqs_in_hi,
    input  wire [1:0]     ddr_dqs_in_lo,
    input  wire           mipi_dp_data0_LP_N_IN,
    input  wire           mipi_dp_data0_LP_P_IN,
    input  wire           mipi_dp_data10_LP_N_IN,
    input  wire           mipi_dp_data10_LP_P_IN,
    input  wire           i_cam_ck_LP_N_IN,
    input  wire           i_cam_ck_LP_P_IN,
    input  wire           cam_d2_FIFO_EMPTY,
    input  wire [7:0]     cam_d2_HS_IN,
    input  wire           cam_d2_LP_N_IN,
    input  wire           cam_d2_LP_P_IN,
    input  wire           cam_d3_FIFO_EMPTY,
    input  wire [7:0]     cam_d3_HS_IN,
    input  wire           cam_d3_LP_N_IN,
    input  wire           cam_d3_LP_P_IN,
    input  wire           cam_d0_FIFO_EMPTY,
    input  wire [7:0]     cam_d0_HS_IN,
    input  wire           cam_d0_LP_N_IN,
    input  wire           cam_d0_LP_P_IN,
    input  wire           cam_d1_FIFO_EMPTY,
    input  wire [7:0]     cam_d1_HS_IN,
    input  wire           cam_d1_LP_N_IN,
    input  wire           cam_d1_LP_P_IN,
    output wire           LCD_POWER,
    output wire           LCD_RST_P,
    output wire           io_cam_scl_OUT,
    output wire           io_cam_scl_OE,
    output wire           io_cam_sda_OUT,
    output wire           io_cam_sda_OE,
    output wire [2:0]     led,
    output wire           txd,
    output wire           o_cam_rst_p,
    output wire           LCD_POWER1,
    output wire           LCD_RST_P1,
    output wire           DDR3_PLL_RSTN,
    output wire [2:0]     pll_shift,
    output wire           pll_shift_ena,
    output wire [4:0]     pll_shift_sel,
    output wire           USER_PLL_RSTN,
    output wire           pll_inst1_RSTN,
    output wire           jtag_inst1_TDO,
    output wire [15:0]    ddr_addr,
    output wire [2:0]     ddr_ba,
    output wire           ddr_cas_n,
    output wire           ddr_ck_hi,
    output wire           ddr_ck_lo,
    output wire [0:0]     ddr_cke,
    output wire [0:0]     ddr_cs_n,
    output wire [1:0]     ddr_dm_hi,
    output wire [1:0]     ddr_dm_lo,
    output wire [15:0]    ddr_dq_out_hi,
    output wire [15:0]    ddr_dq_out_lo,
    output wire [15:0]    ddr_dq_oe,
    output wire [1:0]     ddr_dqs_out_hi,
    output wire [1:0]     ddr_dqs_out_lo,
    output wire [1:0]     ddr_dqs_oe,
    output wire [1:0]     ddr_dqs_oe_n,
    output wire [0:0]     ddr_odt,
    output wire           ddr_ras_n,
    output wire           ddr_reset_n,
    output wire           ddr_we_n,
    output wire           mipi_dp_clk_HS_OE,
    output wire [7:0]     mipi_dp_clk_HS_OUT,
    output wire           mipi_dp_clk_LP_N_OE,
    output wire           mipi_dp_clk_LP_N_OUT,
    output wire           mipi_dp_clk_LP_P_OE,
    output wire           mipi_dp_clk_LP_P_OUT,
    output wire           mipi_dp_clk_RST,
    output wire           mipi_dp_data0_HS_OE,
    output wire [7:0]     mipi_dp_data0_HS_OUT,
    output wire           mipi_dp_data0_LP_N_OE,
    output wire           mipi_dp_data0_LP_N_OUT,
    output wire           mipi_dp_data0_LP_P_OE,
    output wire           mipi_dp_data0_LP_P_OUT,
    output wire           mipi_dp_data0_RST,
    output wire           mipi_dp_data1_HS_OE,
    output wire [7:0]     mipi_dp_data1_HS_OUT,
    output wire           mipi_dp_data1_LP_N_OE,
    output wire           mipi_dp_data1_LP_N_OUT,
    output wire           mipi_dp_data1_LP_P_OE,
    output wire           mipi_dp_data1_LP_P_OUT,
    output wire           mipi_dp_data1_RST,
    output wire           mipi_dp_data2_HS_OE,
    output wire [7:0]     mipi_dp_data2_HS_OUT,
    output wire           mipi_dp_data2_LP_N_OE,
    output wire           mipi_dp_data2_LP_N_OUT,
    output wire           mipi_dp_data2_LP_P_OE,
    output wire           mipi_dp_data2_LP_P_OUT,
    output wire           mipi_dp_data2_RST,
    output wire           mipi_dp_data3_HS_OE,
    output wire [7:0]     mipi_dp_data3_HS_OUT,
    output wire           mipi_dp_data3_LP_N_OE,
    output wire           mipi_dp_data3_LP_N_OUT,
    output wire           mipi_dp_data3_LP_P_OE,
    output wire           mipi_dp_data3_LP_P_OUT,
    output wire           mipi_dp_data3_RST,
    output wire           o_cam_ck_HS_ENA,
    output wire           o_cam_ck_HS_TERM,
    output wire           cam_d2_FIFO_RD,
    output wire           cam_d2_HS_ENA,
    output wire           cam_d2_HS_TERM,
    output wire           cam_d2_RST,
    output wire           cam_d3_FIFO_RD,
    output wire           cam_d3_HS_ENA,
    output wire           cam_d3_HS_TERM,
    output wire           cam_d3_RST,
    output wire           cam_d0_FIFO_RD,
    output wire           cam_d0_HS_ENA,
    output wire           cam_d0_HS_TERM,
    output wire           cam_d0_RST,
    output wire           cam_d1_FIFO_RD,
    output wire           cam_d1_HS_ENA,
    output wire           cam_d1_HS_TERM,
    output wire           cam_d1_RST,
    output wire           mipi_dp_clk1_HS_OE,
    output wire [7:0]     mipi_dp_clk1_HS_OUT,
    output wire           mipi_dp_clk1_LP_N_OE,
    output wire           mipi_dp_clk1_LP_N_OUT,
    output wire           mipi_dp_clk1_LP_P_OE,
    output wire           mipi_dp_clk1_LP_P_OUT,
    output wire           mipi_dp_clk1_RST,
    output wire           mipi_dp_data10_HS_OE,
    output wire [7:0]     mipi_dp_data10_HS_OUT,
    output wire           mipi_dp_data10_LP_N_OE,
    output wire           mipi_dp_data10_LP_N_OUT,
    output wire           mipi_dp_data10_LP_P_OE,
    output wire           mipi_dp_data10_LP_P_OUT,
    output wire           mipi_dp_data10_RST,
    output wire           mipi_dp_data11_HS_OE,
    output wire [7:0]     mipi_dp_data11_HS_OUT,
    output wire           mipi_dp_data11_LP_N_OE,
    output wire           mipi_dp_data11_LP_N_OUT,
    output wire           mipi_dp_data11_LP_P_OE,
    output wire           mipi_dp_data11_LP_P_OUT,
    output wire           mipi_dp_data11_RST,
    output wire           mipi_dp_data12_HS_OE,
    output wire [7:0]     mipi_dp_data12_HS_OUT,
    output wire           mipi_dp_data12_LP_N_OE,
    output wire           mipi_dp_data12_LP_N_OUT,
    output wire           mipi_dp_data12_LP_P_OE,
    output wire           mipi_dp_data12_LP_P_OUT,
    output wire           mipi_dp_data12_RST,
    output wire           mipi_dp_data13_HS_OE,
    output wire [7:0]     mipi_dp_data13_HS_OUT,
    output wire           mipi_dp_data13_LP_N_OE,
    output wire           mipi_dp_data13_LP_N_OUT,
    output wire           mipi_dp_data13_LP_P_OE,
    output wire           mipi_dp_data13_LP_P_OUT,
    output wire           mipi_dp_data13_RST,
    output wire           rx_pix_valid_o,
    output wire [63:0]    rx_pix_data_o,
    output wire [3:0]     rx_pix_per_clk_o,
    output wire           rx_hsync_o,
    output wire           rx_vsync_o,
    output wire           rst_pix_n_o,

    // ---- the display, fused (FUSE_DISP) ----------------------------------
    // The pixel clock is a genuinely separate clock: `d64_video_row_ddr`
    // carries its own asynchronous bridge and `d64_reset_sync` crosses the
    // reset, so this is a declared crossing and not a tie.
    input  wire         px_clk,
    output wire [9:0]   tmds_data0_o,
    output wire [9:0]   tmds_data1_o,
    output wire [9:0]   tmds_data2_o,
    output wire [9:0]   tmds_clk_o,
    output wire         tmds_data0_TX_OE,
    output wire         tmds_data1_TX_OE,
    output wire         tmds_data2_TX_OE,
    output wire         tmds_clk_TX_OE,
    output wire         tmds_data0_TX_RST,
    output wire         tmds_data1_TX_RST,
    output wire         tmds_data2_TX_RST,
    output wire         tmds_clk_TX_RST,
    // The display's DDR reader.  Its own AXI4 read master, arbited by the
    // shell beside the root's four.
    output wire         disp_axi_arvalid,
    input  wire         disp_axi_arready,
    output wire [31:0]  disp_axi_araddr,
    output wire [7:0]   disp_axi_arlen,
    output wire [2:0]   disp_axi_arsize,
    output wire [1:0]   disp_axi_arburst,
    input  wire         disp_axi_rvalid,
    output wire         disp_axi_rready,
    input  wire [127:0] disp_axi_rdata,
    input  wire         disp_axi_rlast,
    input  wire [1:0]   disp_axi_rresp,
    // ---- the fused camera's contract A, brought out for observation --------
    output wire         raw_valid,
    output wire [9:0]   raw10,
    output wire         raw_sof, raw_sol, raw_eol, raw_eof,
    output wire [31:0]  raw_frame_id
);

    // ---- the shape every child is instantiated at --------------------------
    // **One place, so the stale arm is the old build and not a second copy of
    // it.**  At `SAB_SHAPE_STALE = 0` these are the parameters and at 1 they are
    // the literals the file used to carry.
    localparam integer SH_FRAME_H = SAB_SHAPE_STALE ? 256 : IMG_H;
    localparam integer SH_FRAME_W = SAB_SHAPE_STALE ? 256 : IMG_W;
    localparam integer SH_RES4_H  = SH_FRAME_H / 4;   // 64  / 36
    localparam integer SH_RES4_W  = SH_FRAME_W / 4;
    localparam integer SH_UPS0_H  = SH_FRAME_H / 2;   // 128 / 72
    localparam integer SH_UPS0_W  = SH_FRAME_W / 2;
    localparam integer SH_UPS1_H  = SH_FRAME_H;       // 256 / 144
    localparam integer SH_UPS1_W  = SH_FRAME_W;

    // ======================================================================
    // the contract's numbers, once
    // ======================================================================
    localparam integer NOWNERS    = 4;
    localparam integer O_FRONT    = 0;
    localparam integer O_UPS0     = 1;
    localparam integer O_UPS1     = 2;
    localparam integer O_TAIL     = 3;

    localparam integer PE_LANES   = 128;
    localparam integer PE_OC_MAX  = 16;
    localparam integer PE_OUT_W   = 10;
    localparam integer PE_ACT_W   = 11;
    localparam integer PE_ACC_W   = 32;   // the SEAM's, not the bank's
    localparam integer BANK_ACC_W = 23;   // fresh full-domain four-style proof; seam remains32
    localparam integer ACT_W      = 11;
    localparam integer WGT_W      = 8;
    localparam integer M_W        = 16;
    localparam integer SHIFT_W    = 5;
    localparam integer RQ_LANES   = 8;

    localparam integer TILES      = 247;
    localparam integer WSTORE_W   = 1024;
    localparam integer WADDR_W    = 8;

    localparam [1:0] T_RES4 = 2'd0, T_UPS0 = 2'd1, T_UPS1 = 2'd2, T_TAIL = 2'd3;

    // ups.0's frozen shape (`sim/d3c4c/ups0_params.vh`).  ups.1's is the other.
    localparam integer U0_ICG  = 4;         // IC_GROUPS
    localparam integer U0_PIX  = 1;         // PIX_PAR == IN_PORTS
    localparam integer U0_BASE = 184;       // the tile window's base
    localparam integer U1_BASE = 220;

    // ======================================================================
    // the reset, sequenced -- the one module F3 6.19 built for exactly this
    // ======================================================================
    // `req` is a PULSE and not the `fatal` level: see note 3 in the header.
    // The pulse is generated below, one cycle wide, on the fault's rising edge.
    wire pe_rst_n, axi_rst_n;

    d64_axi_reset_ctl #(.SYNC_STAGES(2), .PE_HOLD(8)) u_rst (
        .clk_pe(clk_pe), .clk_axi(clk_axi), .arst_n(arst_n),
        .req(reset_req),
        .pe_arst_n(pe_rst_n), .axi_arst_n(axi_rst_n),
        .pe_busy(pe_busy), .axi_busy(axi_busy)
    );

    assign pe_arst_n  = pe_rst_n;
    assign axi_arst_n = axi_rst_n;

    // ======================================================================
    // the front half -- owner 0, and the store's other client
    // ======================================================================
    wire [31:0]   f_owner_viol, f_wall_cycles, f_hand_cycles;
    wire [2:0]    f_owner;
    wire          f_busy, f_done, f_out_committed;
    wire [31:0]   f_frame_cnt, f_rearm_early;
    wire [1:0]    f_pe_mode;
    wire          f_mac_mode_safe;
    wire [31:0]   f_mode_viol;
    wire [3:0]    f_enc_state;

    // the front half's seam to the bank
    wire          f_slot_valid, f_pix_start, f_rq_start;
    wire [PE_LANES*PE_ACT_W-1:0] f_pe_act;
    wire [PE_LANES*WGT_W-1:0]    f_pe_wgt;
    wire [PE_LANES*PE_ACC_W-1:0] f_pe_bias;
    wire [1:0]    f_req_mode;
    wire [PE_OC_MAX*M_W-1:0]     f_rq_m;
    wire [PE_OC_MAX*SHIFT_W-1:0] f_rq_shift;
    wire [PE_OUT_W-1:0]          f_rq_zero, f_rq_qmin, f_rq_qmax;
    wire          f_rsp_ready, f_rsp_ready_n, f_rsp_busy, f_rsp_done, f_rsp_overrun;
    wire [31:0]   f_rsp_mode_viol;
    wire [RQ_LANES*PE_OUT_W-1:0] f_rsp_y;
    wire [RQ_LANES-1:0]          f_rsp_y_valid;
    wire [PE_LANES*PE_ACC_W-1:0] f_rsp_acc, f_rsp_snapshot;

    // the front half's seam to the store
    wire          f_ws_req_rd_en, f_ws_idle;
    wire [WADDR_W-1:0]  f_ws_req_rd_addr, f_ws_wr_addr;
    wire [WSTORE_W-1:0] f_ws_rsp_rd_data, f_ws_wr_data;
    wire          f_ws_wr_en;

    // The encoder's cfg load -- tied, and said so out loud.  The encoder's cfg
    // ROM is inside the module and this port has never been driven in
    // production; `board/ddr3/d64_encoder_trunk_board_top.v` ties it too.
    wire        f_e_cfg_wr_en   = 1'b0;
    wire [5:0]  f_e_cfg_wr_addr = 6'd0;
    wire [63:0] f_e_cfg_wr_data = 64'd0;

    wire        front_start;

    // ---- declared before the engines that read them (implicit-net trap) ----
    wire [PE_OC_MAX*M_W-1:0]      rd_rq_m;
    wire [PE_OC_MAX*SHIFT_W-1:0]  rd_rq_shift;
    wire [PE_OC_MAX*PE_ACC_W-1:0] rd_rq_bias;
    wire [WSTORE_W-1:0]           rd_w_rd_data;
    wire u0_in_stall;
    wire u0_write_reserve,u1_write_reserve,ups_write_ready;
    wire [8:0] u0_write_row,u1_write_row;
    wire [15:0] u0_write_addr,u1_write_addr;


    // ======================================================================
    // the camera front end and the seam -- batch H, FUSE_CAM
    // ======================================================================
    // Declared before the instance that drives them and before `u_front`, whose
    // port connections are the nets below.
    wire        cam_seam_valid, cam_seam_sof, cam_seam_eol, cam_seam_eof;
    wire [31:0] cam_seam_fid;
    wire [7:0]  cam_seam_x0, cam_seam_y;
    wire [383:0] cam_seam_rgb;

    // The front half's contract-B inputs: the seam's when fused, this module's
    // own ports when not.  A ternary and not a second instance, so the pre-fusion
    // build is the same net it always was.
    wire        f_d64_valid    = FUSE_CAM ? cam_seam_valid : d64_valid;
    wire [31:0] f_d64_frame_id = FUSE_CAM ? cam_seam_fid   : d64_frame_id;
    wire        f_d64_sof      = FUSE_CAM ? cam_seam_sof   : d64_sof;
    wire [7:0]  f_d64_x0       = FUSE_CAM ? cam_seam_x0    : d64_x0;
    wire [7:0]  f_d64_y        = FUSE_CAM ? cam_seam_y     : d64_y;
    wire [383:0] f_d64_rgb888  = FUSE_CAM ? cam_seam_rgb   : d64_rgb888;
    wire        f_d64_eol      = FUSE_CAM ? cam_seam_eol   : d64_eol;
    wire        f_d64_eof      = FUSE_CAM ? cam_seam_eof   : d64_eof;

    wire        cam_pix_valid, cam_pix_hsync, cam_pix_vsync, cam_rst_pix_n;
    wire [63:0] cam_pix_data;
    wire [3:0]  cam_pix_per_clk;

    generate if (FUSE_CAM == 1) begin : g_cam
        // ---- 1. the front end: I2C, the register table, the receiver ----
        // **The real `cam_csi2_top`**, so the sensor is actually programmed:
        // `cam_i2c_master` and `cam_sc431hai_regs` live inside it and without
        // them nothing sets resolution, CFA phase or exposure.  In simulation
        // the encrypted `csi_rx_controller` is stood in for by
        // `board/cam/tb_csi_rx_stub.v` -- the same module name, chosen by the
        // file list, which is the pattern `tools/run_cam_top_i2c.sh` already
        // uses.  In an Efinity build the real IP is what is compiled.
        cam_csi2_top u_camtop (
            .i_arstn(i_arstn),
            .io_cam_scl_IN(io_cam_scl_IN),
            .io_cam_sda_IN(io_cam_sda_IN),
            .clk_100m(clk_100m),
            .pll_locked(pll_locked),
            .user_pll_locked(user_pll_locked),
            .i_pll_locked(i_pll_locked),
            .i_mipi_rx_pclk(i_mipi_rx_pclk),
            .fb(fb),
            .vid_clk_dvi2(vid_clk_dvi2),
            .pll_inst4_CLKOUT0(pll_inst4_CLKOUT0),
            .CLK_5M(CLK_5M),
            .i_mipi_tx_pclk(i_mipi_tx_pclk),
            .i_mipi_txc_sclk(i_mipi_txc_sclk),
            .core_clk(core_clk),
            .i_mipi_txd_sclk(i_mipi_txd_sclk),
            .CLK_25M(CLK_25M),
            .sdram_clk(sdram_clk),
            .tx_cal_clk_90edge(tx_cal_clk_90edge),
            .rx_cal_clk(rx_cal_clk),
            .tx_cal_clk(tx_cal_clk),
            .i_cam_ck_CLKOUT(i_cam_ck_CLKOUT),
            .jtag_inst1_CAPTURE(jtag_inst1_CAPTURE),
            .jtag_inst1_DRCK(jtag_inst1_DRCK),
            .jtag_inst1_RESET(jtag_inst1_RESET),
            .jtag_inst1_RUNTEST(jtag_inst1_RUNTEST),
            .jtag_inst1_SEL(jtag_inst1_SEL),
            .jtag_inst1_SHIFT(jtag_inst1_SHIFT),
            .jtag_inst1_TCK(jtag_inst1_TCK),
            .jtag_inst1_TDI(jtag_inst1_TDI),
            .jtag_inst1_TMS(jtag_inst1_TMS),
            .jtag_inst1_UPDATE(jtag_inst1_UPDATE),
            .ddr_dq_in_hi(ddr_dq_in_hi),
            .ddr_dq_in_lo(ddr_dq_in_lo),
            .ddr_dqs_in_hi(ddr_dqs_in_hi),
            .ddr_dqs_in_lo(ddr_dqs_in_lo),
            .mipi_dp_data0_LP_N_IN(mipi_dp_data0_LP_N_IN),
            .mipi_dp_data0_LP_P_IN(mipi_dp_data0_LP_P_IN),
            .mipi_dp_data10_LP_N_IN(mipi_dp_data10_LP_N_IN),
            .mipi_dp_data10_LP_P_IN(mipi_dp_data10_LP_P_IN),
            .i_cam_ck_LP_N_IN(i_cam_ck_LP_N_IN),
            .i_cam_ck_LP_P_IN(i_cam_ck_LP_P_IN),
            .cam_d2_FIFO_EMPTY(cam_d2_FIFO_EMPTY),
            .cam_d2_HS_IN(cam_d2_HS_IN),
            .cam_d2_LP_N_IN(cam_d2_LP_N_IN),
            .cam_d2_LP_P_IN(cam_d2_LP_P_IN),
            .cam_d3_FIFO_EMPTY(cam_d3_FIFO_EMPTY),
            .cam_d3_HS_IN(cam_d3_HS_IN),
            .cam_d3_LP_N_IN(cam_d3_LP_N_IN),
            .cam_d3_LP_P_IN(cam_d3_LP_P_IN),
            .cam_d0_FIFO_EMPTY(cam_d0_FIFO_EMPTY),
            .cam_d0_HS_IN(cam_d0_HS_IN),
            .cam_d0_LP_N_IN(cam_d0_LP_N_IN),
            .cam_d0_LP_P_IN(cam_d0_LP_P_IN),
            .cam_d1_FIFO_EMPTY(cam_d1_FIFO_EMPTY),
            .cam_d1_HS_IN(cam_d1_HS_IN),
            .cam_d1_LP_N_IN(cam_d1_LP_N_IN),
            .cam_d1_LP_P_IN(cam_d1_LP_P_IN),
            .LCD_POWER(LCD_POWER),
            .LCD_RST_P(LCD_RST_P),
            .io_cam_scl_OUT(io_cam_scl_OUT),
            .io_cam_scl_OE(io_cam_scl_OE),
            .io_cam_sda_OUT(io_cam_sda_OUT),
            .io_cam_sda_OE(io_cam_sda_OE),
            .led(led),
            .txd(txd),
            .o_cam_rst_p(o_cam_rst_p),
            .LCD_POWER1(LCD_POWER1),
            .LCD_RST_P1(LCD_RST_P1),
            .DDR3_PLL_RSTN(DDR3_PLL_RSTN),
            .pll_shift(pll_shift),
            .pll_shift_ena(pll_shift_ena),
            .pll_shift_sel(pll_shift_sel),
            .USER_PLL_RSTN(USER_PLL_RSTN),
            .pll_inst1_RSTN(pll_inst1_RSTN),
            .jtag_inst1_TDO(jtag_inst1_TDO),
            .ddr_addr(ddr_addr),
            .ddr_ba(ddr_ba),
            .ddr_cas_n(ddr_cas_n),
            .ddr_ck_hi(ddr_ck_hi),
            .ddr_ck_lo(ddr_ck_lo),
            .ddr_cke(ddr_cke),
            .ddr_cs_n(ddr_cs_n),
            .ddr_dm_hi(ddr_dm_hi),
            .ddr_dm_lo(ddr_dm_lo),
            .ddr_dq_out_hi(ddr_dq_out_hi),
            .ddr_dq_out_lo(ddr_dq_out_lo),
            .ddr_dq_oe(ddr_dq_oe),
            .ddr_dqs_out_hi(ddr_dqs_out_hi),
            .ddr_dqs_out_lo(ddr_dqs_out_lo),
            .ddr_dqs_oe(ddr_dqs_oe),
            .ddr_dqs_oe_n(ddr_dqs_oe_n),
            .ddr_odt(ddr_odt),
            .ddr_ras_n(ddr_ras_n),
            .ddr_reset_n(ddr_reset_n),
            .ddr_we_n(ddr_we_n),
            .mipi_dp_clk_HS_OE(mipi_dp_clk_HS_OE),
            .mipi_dp_clk_HS_OUT(mipi_dp_clk_HS_OUT),
            .mipi_dp_clk_LP_N_OE(mipi_dp_clk_LP_N_OE),
            .mipi_dp_clk_LP_N_OUT(mipi_dp_clk_LP_N_OUT),
            .mipi_dp_clk_LP_P_OE(mipi_dp_clk_LP_P_OE),
            .mipi_dp_clk_LP_P_OUT(mipi_dp_clk_LP_P_OUT),
            .mipi_dp_clk_RST(mipi_dp_clk_RST),
            .mipi_dp_data0_HS_OE(mipi_dp_data0_HS_OE),
            .mipi_dp_data0_HS_OUT(mipi_dp_data0_HS_OUT),
            .mipi_dp_data0_LP_N_OE(mipi_dp_data0_LP_N_OE),
            .mipi_dp_data0_LP_N_OUT(mipi_dp_data0_LP_N_OUT),
            .mipi_dp_data0_LP_P_OE(mipi_dp_data0_LP_P_OE),
            .mipi_dp_data0_LP_P_OUT(mipi_dp_data0_LP_P_OUT),
            .mipi_dp_data0_RST(mipi_dp_data0_RST),
            .mipi_dp_data1_HS_OE(mipi_dp_data1_HS_OE),
            .mipi_dp_data1_HS_OUT(mipi_dp_data1_HS_OUT),
            .mipi_dp_data1_LP_N_OE(mipi_dp_data1_LP_N_OE),
            .mipi_dp_data1_LP_N_OUT(mipi_dp_data1_LP_N_OUT),
            .mipi_dp_data1_LP_P_OE(mipi_dp_data1_LP_P_OE),
            .mipi_dp_data1_LP_P_OUT(mipi_dp_data1_LP_P_OUT),
            .mipi_dp_data1_RST(mipi_dp_data1_RST),
            .mipi_dp_data2_HS_OE(mipi_dp_data2_HS_OE),
            .mipi_dp_data2_HS_OUT(mipi_dp_data2_HS_OUT),
            .mipi_dp_data2_LP_N_OE(mipi_dp_data2_LP_N_OE),
            .mipi_dp_data2_LP_N_OUT(mipi_dp_data2_LP_N_OUT),
            .mipi_dp_data2_LP_P_OE(mipi_dp_data2_LP_P_OE),
            .mipi_dp_data2_LP_P_OUT(mipi_dp_data2_LP_P_OUT),
            .mipi_dp_data2_RST(mipi_dp_data2_RST),
            .mipi_dp_data3_HS_OE(mipi_dp_data3_HS_OE),
            .mipi_dp_data3_HS_OUT(mipi_dp_data3_HS_OUT),
            .mipi_dp_data3_LP_N_OE(mipi_dp_data3_LP_N_OE),
            .mipi_dp_data3_LP_N_OUT(mipi_dp_data3_LP_N_OUT),
            .mipi_dp_data3_LP_P_OE(mipi_dp_data3_LP_P_OE),
            .mipi_dp_data3_LP_P_OUT(mipi_dp_data3_LP_P_OUT),
            .mipi_dp_data3_RST(mipi_dp_data3_RST),
            .o_cam_ck_HS_ENA(o_cam_ck_HS_ENA),
            .o_cam_ck_HS_TERM(o_cam_ck_HS_TERM),
            .cam_d2_FIFO_RD(cam_d2_FIFO_RD),
            .cam_d2_HS_ENA(cam_d2_HS_ENA),
            .cam_d2_HS_TERM(cam_d2_HS_TERM),
            .cam_d2_RST(cam_d2_RST),
            .cam_d3_FIFO_RD(cam_d3_FIFO_RD),
            .cam_d3_HS_ENA(cam_d3_HS_ENA),
            .cam_d3_HS_TERM(cam_d3_HS_TERM),
            .cam_d3_RST(cam_d3_RST),
            .cam_d0_FIFO_RD(cam_d0_FIFO_RD),
            .cam_d0_HS_ENA(cam_d0_HS_ENA),
            .cam_d0_HS_TERM(cam_d0_HS_TERM),
            .cam_d0_RST(cam_d0_RST),
            .cam_d1_FIFO_RD(cam_d1_FIFO_RD),
            .cam_d1_HS_ENA(cam_d1_HS_ENA),
            .cam_d1_HS_TERM(cam_d1_HS_TERM),
            .cam_d1_RST(cam_d1_RST),
            .mipi_dp_clk1_HS_OE(mipi_dp_clk1_HS_OE),
            .mipi_dp_clk1_HS_OUT(mipi_dp_clk1_HS_OUT),
            .mipi_dp_clk1_LP_N_OE(mipi_dp_clk1_LP_N_OE),
            .mipi_dp_clk1_LP_N_OUT(mipi_dp_clk1_LP_N_OUT),
            .mipi_dp_clk1_LP_P_OE(mipi_dp_clk1_LP_P_OE),
            .mipi_dp_clk1_LP_P_OUT(mipi_dp_clk1_LP_P_OUT),
            .mipi_dp_clk1_RST(mipi_dp_clk1_RST),
            .mipi_dp_data10_HS_OE(mipi_dp_data10_HS_OE),
            .mipi_dp_data10_HS_OUT(mipi_dp_data10_HS_OUT),
            .mipi_dp_data10_LP_N_OE(mipi_dp_data10_LP_N_OE),
            .mipi_dp_data10_LP_N_OUT(mipi_dp_data10_LP_N_OUT),
            .mipi_dp_data10_LP_P_OE(mipi_dp_data10_LP_P_OE),
            .mipi_dp_data10_LP_P_OUT(mipi_dp_data10_LP_P_OUT),
            .mipi_dp_data10_RST(mipi_dp_data10_RST),
            .mipi_dp_data11_HS_OE(mipi_dp_data11_HS_OE),
            .mipi_dp_data11_HS_OUT(mipi_dp_data11_HS_OUT),
            .mipi_dp_data11_LP_N_OE(mipi_dp_data11_LP_N_OE),
            .mipi_dp_data11_LP_N_OUT(mipi_dp_data11_LP_N_OUT),
            .mipi_dp_data11_LP_P_OE(mipi_dp_data11_LP_P_OE),
            .mipi_dp_data11_LP_P_OUT(mipi_dp_data11_LP_P_OUT),
            .mipi_dp_data11_RST(mipi_dp_data11_RST),
            .mipi_dp_data12_HS_OE(mipi_dp_data12_HS_OE),
            .mipi_dp_data12_HS_OUT(mipi_dp_data12_HS_OUT),
            .mipi_dp_data12_LP_N_OE(mipi_dp_data12_LP_N_OE),
            .mipi_dp_data12_LP_N_OUT(mipi_dp_data12_LP_N_OUT),
            .mipi_dp_data12_LP_P_OE(mipi_dp_data12_LP_P_OE),
            .mipi_dp_data12_LP_P_OUT(mipi_dp_data12_LP_P_OUT),
            .mipi_dp_data12_RST(mipi_dp_data12_RST),
            .mipi_dp_data13_HS_OE(mipi_dp_data13_HS_OE),
            .mipi_dp_data13_HS_OUT(mipi_dp_data13_HS_OUT),
            .mipi_dp_data13_LP_N_OE(mipi_dp_data13_LP_N_OE),
            .mipi_dp_data13_LP_N_OUT(mipi_dp_data13_LP_N_OUT),
            .mipi_dp_data13_LP_P_OE(mipi_dp_data13_LP_P_OE),
            .mipi_dp_data13_LP_P_OUT(mipi_dp_data13_LP_P_OUT),
            .mipi_dp_data13_RST(mipi_dp_data13_RST),
            .rx_pix_valid_o(rx_pix_valid_o),
            .rx_pix_data_o(rx_pix_data_o),
            .rx_pix_per_clk_o(rx_pix_per_clk_o),
            .rx_hsync_o(rx_hsync_o),
            .rx_vsync_o(rx_vsync_o),
            .rst_pix_n_o(rst_pix_n_o)
        );

        assign cam_pix_valid     = rx_pix_valid_o;
        assign cam_pix_data      = rx_pix_data_o;
        assign cam_pix_per_clk   = rx_pix_per_clk_o;
        assign cam_pix_hsync     = rx_hsync_o;
        assign cam_pix_vsync     = rx_vsync_o;
        assign cam_rst_pix_n     = rst_pix_n_o;

        // ---- 2. the adapter that did not exist --------------------------
        // The receiver drives FOUR 10-bit pixels a clock and contract A is ONE
        // sample a clock with raster markers; nothing in this tree converted
        // between them.  The beat crosses and the samples are unpacked on the
        // far side -- see the module's own header.
        d64_csi2_raw_to_contract_a #(.AW(5), .PPC_MAX(8)) u_a2a (
            .clk_pix(i_mipi_rx_pclk), .rst_pix_n(cam_rst_pix_n),
            .pix_valid(cam_pix_valid), .pix_data(cam_pix_data),
            .pix_per_clk(cam_pix_per_clk),
            .hsync(cam_pix_hsync), .vsync(cam_pix_vsync),
            .clk_a(clk_pe), .rst_a_n(pe_rst_n),
            .raw_valid(raw_valid), .raw10(raw10),
            .raw_sof(raw_sof), .raw_sol(raw_sol),
            .raw_eol(raw_eol), .raw_eof(raw_eof),
            .raw_frame_id(raw_frame_id),
            .beats_in(), .px_in(), .beats_out(), .px_out(), .frames_out(),
            .overflow(), .ppc_bad()
        );

        // ---- 3. the seam: contract A -> contract B ----------------------
        d64_cam_rgb256 #(
            .W_SRC(CAM_W), .H_SRC(CAM_H), .W_OUT(IMG_W), .H_OUT(IMG_H),
            .CFA(CAM_CFA), .BLACK(CAM_BLACK), .GAIN_Q8(CAM_GAIN_Q8), .FIFO_AW(3)
        ) u_cam (
            .clk(clk_pe), .rst_n(pe_rst_n),
            .raw_valid(raw_valid), .raw10(raw10),
            .raw_sof(raw_sof), .raw_sol(raw_sol),
            .raw_eol(raw_eol), .raw_eof(raw_eof),
            .raw_frame_id(raw_frame_id),
            .d64_valid(cam_seam_valid), .d64_ready(d64_ready),
            .d64_frame_id(cam_seam_fid), .d64_sof(cam_seam_sof),
            .d64_x0(cam_seam_x0), .d64_y(cam_seam_y),
            .d64_rgb888(cam_seam_rgb),
            .d64_eol(cam_seam_eol), .d64_eof(cam_seam_eof),
            .src_samples(), .src_lines(), .src_frames(), .demos_px(),
            .out_px(), .beats_out(), .frames_out(), .dropped_beats(),
            .short_lines(), .geom_margin_ok()
        );
    end
    else begin : g_nocam
        // The pre-fusion build.  The seam's nets are left undriven because the
        // ternary above never selects them; nothing here drives them, so a
        // build that did select them would read z rather than a stale value.
    end
    endgenerate

    wire [31:0] fsb6_t_wf_ovf;
    wire front_cfg_protocol_fault;
    d64_encoder_trunk #(
        .FAULT_STICKY(FAULT_STICKY),
        .WT_INIT_FILE(""), .CFG_INIT_FILE(""),
        .EXTERNAL_PE(1), .EXTERNAL_WSTORE(1),
        .ACC_W(BANK_ACC_W), .ROWS(SH_RES4_H), .COLS(SH_RES4_W),
        .RUNTIME_PROFILE(RUNTIME_PROFILE), .HEAD_STORE_CREDIT_ENABLE(HEAD_STORE_CREDIT_ENABLE), .CFG_BIAS_W(TRUNK_CFG_BIAS_W), .CFG_STREAM_SRL(CFG_STREAM_SRL)
    ) u_front (
        .clk_pe(clk_pe), .clk_axi(clk_axi), .arst_n(pe_rst_n),
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .start(front_start),
        .e_cfg_wr_en(f_e_cfg_wr_en), .e_cfg_wr_addr(f_e_cfg_wr_addr),
        .e_cfg_wr_data(f_e_cfg_wr_data),
        .t_cfg_we(t_cfg_we), .t_cfg_wa(t_cfg_wa), .t_cfg_wd(t_cfg_wd),
        .trunk_debug(trunk_debug),
        .t_wf_ovf(fsb6_t_wf_ovf),
        .owner(f_owner), .owner_viol(f_owner_viol),
        .wall_cycles(f_wall_cycles), .hand_cycles(f_hand_cycles),
        .out_committed(f_out_committed),
        .trunk_store_committed(trunk_store_committed),
        .frame_cnt(f_frame_cnt), .rearm_early(f_rearm_early),
        .busy(f_busy), .done(f_done), .pe_mode(f_pe_mode),
        .enc_state(f_enc_state),
        .bank_mode_viol(f_mode_viol), .cfg_protocol_fault(front_cfg_protocol_fault),
        .d64_valid(f_d64_valid), .d64_ready(d64_ready),
        .d64_frame_id(f_d64_frame_id), .d64_sof(f_d64_sof),
        .d64_x0(f_d64_x0), .d64_y(f_d64_y), .d64_rgb888(f_d64_rgb888),
        .d64_eol(f_d64_eol), .d64_eof(f_d64_eof),
        .wt_wr_en(wt_wr_en), .wt_wr_addr(8'd0), .wt_wr_data(1024'd0),
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
        .head_read_frame_id(head_read_frame_id),
        .head_read_sof(head_read_sof), .head_read_eol(head_read_eol),
        .head_read_eof(head_read_eof),
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
        .out_valid(out_valid), .out_ready(out_ready),
        .out_x(out_x), .out_y(out_y), .out_word(out_word), .out_data(out_data),
        .out_frame_id(out_frame_id), .out_sof(out_sof), .out_eol(out_eol),
        .out_eof(out_eof),
        .blk_redo(blk_redo),
        .t_cfg_selftest(t_cfg_selftest),
        .x_req_row(x_req_row), .x_req_tog(x_req_tog), .x_ack_tog(x_ack_tog),
        .t_req_buf(t_req_buf), .t_src_buf(t_src_buf), .t_dst_buf(t_dst_buf),
        .rf_wr_en(rf_wr_en), .rf_wr_data(rf_wr_data), .rf_wr_full(rf_wr_full),
        .wf_rd_en(wf_rd_en), .wf_rd_data(wf_rd_data),
        .wf_rd_valid(wf_rd_valid), .wf_rd_empty(wf_rd_empty), .wf_rd_level(wf_rd_level),
        .wr_done_tog(wr_done_tog),
        // ---- the PE seam, section 1 ---------------------------------------
        .pe_req_slot_valid(f_slot_valid), .pe_req_act(f_pe_act),
        .pe_req_wgt(f_pe_wgt), .pe_req_pix_start(f_pix_start),
        .pe_req_bias(f_pe_bias), .pe_req_mode(f_req_mode),
        .pe_req_rq_start(f_rq_start), .pe_req_rq_m(f_rq_m),
        .pe_req_rq_shift(f_rq_shift), .pe_req_rq_zero(f_rq_zero),
        .pe_req_rq_qmin(f_rq_qmin), .pe_req_rq_qmax(f_rq_qmax),
        .pe_rsp_rq_ready(f_rsp_ready), .pe_rsp_rq_ready_n(f_rsp_ready_n),
        .pe_rsp_y(f_rsp_y), .pe_rsp_y_valid(f_rsp_y_valid),
        .pe_rsp_rq_busy(f_rsp_busy), .pe_rsp_rq_done(f_rsp_done),
        .pe_rsp_rq_overrun(f_rsp_overrun),
        .pe_rsp_mode_viol(f_rsp_mode_viol),
        .pe_rsp_acc(f_rsp_acc), .pe_rsp_snapshot(f_rsp_snapshot),
        .mac_mode_safe(f_mac_mode_safe),
        // ---- the store's seam, section 3.12 --------------------------------
        .ws_req_rd_en(f_ws_req_rd_en), .ws_req_rd_addr(f_ws_req_rd_addr),
        .ws_rsp_rd_data(f_ws_rsp_rd_data),
        .ws_wr_en(f_ws_wr_en), .ws_wr_addr(f_ws_wr_addr),
        .ws_wr_data(f_ws_wr_data), .ws_idle(f_ws_idle)
    );

    // ======================================================================
    // the arbiter, the ACC25 bridge, the bank and the store
    // ======================================================================
    wire [NOWNERS-1:0]             arb_slot_i, arb_pix_i, arb_rqs_i, arb_safe_i;
    wire [NOWNERS*2-1:0]           arb_mode_i;
    wire [NOWNERS*PE_LANES*PE_ACT_W-1:0] arb_act_i;
    wire [NOWNERS*PE_LANES*WGT_W-1:0]    arb_wgt_i;
    wire [NOWNERS*PE_LANES*PE_ACC_W-1:0] arb_bias_i;
    wire [NOWNERS*PE_OC_MAX*M_W-1:0]     arb_m_i;
    wire [NOWNERS*PE_OC_MAX*SHIFT_W-1:0] arb_sh_i;
    wire [NOWNERS*PE_OUT_W-1:0]          arb_zero_i, arb_qmin_i, arb_qmax_i;
    wire [NOWNERS-1:0]             arb_ready_o, arb_ready_n_o, arb_busy_o;
    wire [NOWNERS-1:0]             arb_done_o, arb_over_o;
    wire [NOWNERS*RQ_LANES*PE_OUT_W-1:0] arb_y_o;
    wire [NOWNERS*RQ_LANES-1:0]    arb_yv_o;
    wire [NOWNERS*32-1:0]          arb_mv_o;
    wire [NOWNERS*PE_LANES*PE_ACC_W-1:0] arb_acc_o, arb_snap_o;

    wire [1:0]  arb_owner_sel;
    wire        arb_sel_req, arb_sel_ack;

    // ======================================================================
    // ups.0 -- the one decoder layer the single-port cache can drive
    // ======================================================================
    wire        u0_start_i;
    wire        u0_busy, u0_done;
    wire [31:0] u0_cycles, u0_rq_req, u0_rq_acc;
    wire [31:0] u0_peak;

    wire [U0_PIX-1:0]              u0_in_en;
    wire [U0_PIX*16-1:0]           u0_in_addr;
    wire [U0_PIX*8*10-1:0]         u0_in_data;
    wire                           u0_w_en;
    wire [WADDR_W-1:0]             u0_w_addr;
    wire [8*16*WGT_W-1:0]          u0_w_data;
    wire                           u0_ow_en, u0_ow_row;
    wire [15:0]                    u0_ow_addr;
    wire [8:0]                     u0_ow_rowabs;
    wire [8*8-1:0]                 u0_ow_data;

    wire         u0_slot_valid, u0_pix_start, u0_rq_start;
    wire [PE_LANES*PE_ACT_W-1:0] u0_pe_act;
    wire [PE_LANES*WGT_W-1:0]    u0_pe_wgt;
    wire [PE_LANES*PE_ACC_W-1:0] u0_pe_bias;
    wire [1:0]   u0_req_mode;
    wire [PE_OC_MAX*M_W-1:0]     u0_rq_m_o;
    wire [PE_OC_MAX*SHIFT_W-1:0] u0_rq_shift_o;
    wire [PE_OUT_W-1:0]          u0_rq_zero, u0_rq_qmin, u0_rq_qmax;
    wire         u0_rsp_ready, u0_rsp_ready_n, u0_rsp_busy, u0_rsp_done, u0_rsp_overrun;
    wire [31:0]  u0_rsp_mode_viol;
    wire [RQ_LANES*PE_OUT_W-1:0] u0_rsp_y;
    wire [RQ_LANES-1:0]          u0_rsp_y_valid;
    wire [PE_LANES*PE_ACC_W-1:0] u0_rsp_acc, u0_rsp_snapshot;
    wire         u0_mode_safe;

    d64_tconv_engine #(
        .FAULT_STICKY(FAULT_STICKY),
        .H(SH_RES4_H), .W(SH_RES4_W),
        .PIX_PAR(1), .IC_GROUPS(4), .OC_GROUPS(1),
        .OC_LANES(16), .IC_PAR(8), .WORD_CH(8), .IN_PORTS(1),
        .W_BASE(184), .W_ADDR_W(WADDR_W), .IN_ADDR_W(14), .IN_WORD_W(16),
        .OUT_ADDR_W(16),
        .IN_W(10), .IN_ZERO(413), .ACT_W(11), .WGT_W(WGT_W),
        .RUNTIME_IN_ZERO(RUNTIME_PROFILE),
        .ACC_W(PE_ACC_W), .M_W(M_W), .SHIFT_W(SHIFT_W), .OUT_W(8),
        .RQ_LANES(RQ_LANES), .QMIN(0), .QMAX(255), .RQ_LOOKAHEAD(4),
        .PE_LANES(PE_LANES), .PE_OC_MAX(PE_OC_MAX), .PE_OUT_W(PE_OUT_W),
        .PE_ACT_W(PE_ACT_W), .EXTERNAL_PE(1),.WRITE_RESERVATION(1)
    ) u_u0 (
        .write_phase_ready(ups_write_ready),.write_phase_reserve(u0_write_reserve),
        .write_phase_row(u0_write_row),.write_phase_addr(u0_write_addr),
        .clk(clk_pe), .rst_n(pe_rst_n), .start(u0_start_i),
        .runtime_in_zero({1'b0,profile_trunk_zero}),
        .busy(u0_busy), .done(u0_done), .cycles(u0_cycles),
        .rq_m(rd_rq_m[16*M_W-1:0]),
        .rq_shift(rd_rq_shift[16*SHIFT_W-1:0]),
        .rq_bias(rd_rq_bias[16*PE_ACC_W-1:0]),
        .in_stall(u0_in_stall),
        .in_rd_en(u0_in_en), .in_rd_addr(u0_in_addr), .in_rd_data(u0_in_data),
        .w_rd_en(u0_w_en), .w_rd_addr(u0_w_addr), .w_rd_data(u0_w_data),
        .out_wr_en(u0_ow_en), .out_wr_addr(u0_ow_addr), .out_wr_row(u0_ow_row),
        .out_wr_row_abs(u0_ow_rowabs), .out_wr_data(u0_ow_data),
        .rq_requests(u0_rq_req), .rq_accepts(u0_rq_acc),
        .rq_wait_cycles(u0_rq_wait), .rq_overrun_count(u0_rq_overrun),
        .slot_count(u0_slots), .out_write_count(u0_writes), .mfifo_peak(u0_peak),
        .in_stall_cycles(u0_stall_cyc),
        .pe_req_slot_valid(u0_slot_valid), .pe_req_act(u0_pe_act),
        .pe_req_wgt(u0_pe_wgt), .pe_req_pix_start(u0_pix_start),
        .pe_req_bias(u0_pe_bias), .pe_req_mode(u0_req_mode),
        .pe_req_rq_start(u0_rq_start), .pe_req_rq_m(u0_rq_m_o),
        .pe_req_rq_shift(u0_rq_shift_o), .pe_req_rq_zero(u0_rq_zero),
        .pe_req_rq_qmin(u0_rq_qmin), .pe_req_rq_qmax(u0_rq_qmax),
        .pe_rsp_rq_ready(u0_rsp_ready), .pe_rsp_rq_ready_n(u0_rsp_ready_n),
        .pe_rsp_y(u0_rsp_y), .pe_rsp_y_valid(u0_rsp_y_valid),
        .pe_rsp_rq_busy(u0_rsp_busy), .pe_rsp_rq_done(u0_rsp_done),
        .pe_rsp_rq_overrun(u0_rsp_overrun),
        .pe_rsp_mode_viol(u0_rsp_mode_viol),
        .pe_rsp_acc(u0_rsp_acc), .pe_rsp_snapshot(u0_rsp_snapshot),
        .mac_mode_safe(u0_mode_safe)
    );

    // ======================================================================
    // ups.1 -- instantiated and wired; its two input ports are on the
    // boundary because the two-port store does not exist yet
    // ======================================================================
    wire        u1_start_i;
    wire        u1_busy, u1_done;
    wire [31:0] u1_cycles, u1_rq_req, u1_rq_acc, u1_rq_wait, u1_rq_overrun;
    wire [31:0] u1_writes, u1_peak, u1_stall_cyc;

    wire [1:0]              u1_in_en;
    wire [2*16-1:0]         u1_in_addr;
    wire                    u1_w_en;
    wire [WADDR_W-1:0]      u1_w_addr;
    wire [8*8*WGT_W-1:0]    u1_w_data;
    wire                    u1_ow_en, u1_ow_row;
    wire [15:0]             u1_ow_addr;
    wire [8:0]              u1_ow_rowabs;
    wire [8*8-1:0]          u1_ow_data;

    wire         u1_slot_valid, u1_pix_start, u1_rq_start;
    wire [PE_LANES*PE_ACT_W-1:0] u1_pe_act;
    wire [PE_LANES*WGT_W-1:0]    u1_pe_wgt;
    wire [PE_LANES*PE_ACC_W-1:0] u1_pe_bias;
    wire [1:0]   u1_req_mode;
    wire [PE_OC_MAX*M_W-1:0]     u1_rq_m_o;
    wire [PE_OC_MAX*SHIFT_W-1:0] u1_rq_shift_o;
    wire [PE_OUT_W-1:0]          u1_rq_zero, u1_rq_qmin, u1_rq_qmax;
    wire         u1_rsp_ready, u1_rsp_ready_n, u1_rsp_busy, u1_rsp_done, u1_rsp_overrun;
    wire [31:0]  u1_rsp_mode_viol;
    wire [RQ_LANES*PE_OUT_W-1:0] u1_rsp_y;
    wire [RQ_LANES-1:0]          u1_rsp_y_valid;
    wire [PE_LANES*PE_ACC_W-1:0] u1_rsp_acc, u1_rsp_snapshot;
    wire         u1_mode_safe;

    // ---- ups.1's store is this root's own cache now ----------------------
    // The engine reads the single-port adapter's `e_*` side -- "the one store
    // port, and the two engines that take turns on it", beside the stage bits
    // below -- and `u1_in_rd_data` / `u1_in_stall`, the boundary store the
    // shell provided, are not read here at all any more.  Both stay on the
    // port list because the shell and the gates still wire them; moving a
    // boundary is a second change and not this one.
    wire [127:0] u1_in_data_i;      // {port 1, port 0}: the adapter's engine side
    wire         u1_in_stall_i;

    assign u1_in_rd_en   = u1_in_en;
    assign u1_in_rd_addr = u1_in_addr;

    d64_tconv_engine #(
        .FAULT_STICKY(FAULT_STICKY),
        .H(SH_UPS0_H), .W(SH_UPS0_W),
        .PIX_PAR(2), .IC_GROUPS(2), .OC_GROUPS(1),
        .OC_LANES(8), .IC_PAR(8), .WORD_CH(8), .IN_PORTS(2),
        .W_BASE(220), .W_ADDR_W(WADDR_W), .IN_ADDR_W(14), .IN_WORD_W(16),
        .OUT_ADDR_W(16),
        .IN_W(8), .IN_ZERO(0), .ACT_W(9), .WGT_W(WGT_W),
        .ACC_W(PE_ACC_W), .M_W(M_W), .SHIFT_W(SHIFT_W), .OUT_W(8),
        .RQ_LANES(RQ_LANES), .QMIN(0), .QMAX(255), .RQ_LOOKAHEAD(4),
        .PE_LANES(PE_LANES), .PE_OC_MAX(PE_OC_MAX), .PE_OUT_W(PE_OUT_W),
        .PE_ACT_W(PE_ACT_W), .EXTERNAL_PE(1),.WRITE_RESERVATION(1)
    ) u_u1 (
        .write_phase_ready(ups_write_ready),.write_phase_reserve(u1_write_reserve),
        .write_phase_row(u1_write_row),.write_phase_addr(u1_write_addr),
        .clk(clk_pe), .rst_n(pe_rst_n), .start(u1_start_i),
        .busy(u1_busy), .done(u1_done), .cycles(u1_cycles),
        .rq_m(rd_rq_m[8*M_W-1:0]),
        .rq_shift(rd_rq_shift[8*SHIFT_W-1:0]),
        .rq_bias(rd_rq_bias[8*PE_ACC_W-1:0]),
        .in_stall(u1_in_stall_i),
        .in_rd_en(u1_in_en), .in_rd_addr(u1_in_addr), .in_rd_data(u1_in_data_i),
        .w_rd_en(u1_w_en), .w_rd_addr(u1_w_addr), .w_rd_data(u1_w_data),
        .out_wr_en(u1_ow_en), .out_wr_addr(u1_ow_addr), .out_wr_row(u1_ow_row),
        .out_wr_row_abs(u1_ow_rowabs), .out_wr_data(u1_ow_data),
        .rq_requests(u1_rq_req), .rq_accepts(u1_rq_acc),
        .rq_wait_cycles(u1_rq_wait), .rq_overrun_count(u1_rq_overrun),
        .slot_count(u1_slots), .out_write_count(u1_writes), .mfifo_peak(u1_peak),
        .in_stall_cycles(u1_stall_cyc),
        .pe_req_slot_valid(u1_slot_valid), .pe_req_act(u1_pe_act),
        .pe_req_wgt(u1_pe_wgt), .pe_req_pix_start(u1_pix_start),
        .pe_req_bias(u1_pe_bias), .pe_req_mode(u1_req_mode),
        .pe_req_rq_start(u1_rq_start), .pe_req_rq_m(u1_rq_m_o),
        .pe_req_rq_shift(u1_rq_shift_o), .pe_req_rq_zero(u1_rq_zero),
        .pe_req_rq_qmin(u1_rq_qmin), .pe_req_rq_qmax(u1_rq_qmax),
        .pe_rsp_rq_ready(u1_rsp_ready), .pe_rsp_rq_ready_n(u1_rsp_ready_n),
        .pe_rsp_y(u1_rsp_y), .pe_rsp_y_valid(u1_rsp_y_valid),
        .pe_rsp_rq_busy(u1_rsp_busy), .pe_rsp_rq_done(u1_rsp_done),
        .pe_rsp_rq_overrun(u1_rsp_overrun),
        .pe_rsp_mode_viol(u1_rsp_mode_viol),
        .pe_rsp_acc(u1_rsp_acc), .pe_rsp_snapshot(u1_rsp_snapshot),
        .mac_mode_safe(u1_mode_safe)
    );

    // ======================================================================
    // the tail -- engine, RGBX writer and done tracker, joined (F3 3.15)
    // ======================================================================
    // **A port the chain did not have, and now does.**  F3 6.9 amended
    // `d64_tail_engine` to take `in_stall`, and the tail's own
    // `in_rd_en/in_rd_addr/in_rd_data` came out here while the **child's
    // `in_stall` did not**: `d64_tail_output_chain.v` connected three of the
    // four and the fourth read `z` (false, "never stall") in every build of the
    // chain.  That was correct for the RAM store the chain was originally
    // built against and wrong for the real `d64_burst_prefetch`, which can
    // miss -- and the engine's own gate (`docs/TAIL_INPUT_STALL_2026-10-07.md`)
    // measures what ignoring it costs: 22,528 of 65,536 words wrong, `done`
    // high, every fault count zero.  The chain now carries `in_stall` and its
    // two counters (`in_stall_cycles`/`in_stall_events`), and this root brings
    // the input out as `tl_in_stall` beside the three ports that were already
    // on the boundary.  `docs/ROOT_2026-10-07.md` section 4.2 named this as one
    // of the tail arm's two blockers; the other (the six-port store) stands.
    wire        tl_start_i;
    wire        tl_busy, tl_done, tl_responses_done, tl_tail_done, tl_drained;
    wire [1:0]  tl_done_reason;
    wire [31:0] tl_tail_cycles, tl_exp_count;
    wire        tl_w_en;
    wire [11:0] tl_w_addr;
    wire [5:0]  tl_in_en;
    wire [6*16-1:0] tl_in_addr;
    wire [8*4*8-1:0] tl_w_data;

    // ---- the tail's input, and the port that feeds it ----------------------
    // **The tail is the job's THIRD HOP.**  Its six read ports are behind its
    // own `d64_store_rd_mux`, and that adapter's ONE store port is the same
    // `u_cache` ups.0 and ups.1 take turns on.  `SAB_TL_FIXTURE` is the
    // pre-hop build: the three boundary ports, a fixture by another name.
    // Everything the two adapters need is declared here, above the tail's own
    // instance, because a name first seen inside a port connection is an
    // implicit one-bit net in this flow.
    wire [383:0] tl_in_data_i;      // what the tail actually reads
    wire         tl_in_stall_i;
    wire         tl_stall_timeout;  // the tail tracker's stall report
    wire         c_port_sel_u1;     // 1 while ups.1 owns the store port
    wire         c_port_sel_tl;     // 1 while the tail owns the store port
    wire [79:0]  c_port_data;       // the cache's word: IC_PAR(8) * IN_W(10)
    wire         c_port_stall;      // ... and its own stall
    wire         u1_to_en;          // ups.1's adapter, store side: ONE port
    wire [15:0]  u1_to_addr;
    wire         u1_to_stall;
    wire         tl_to_en;          // the tail's adapter, store side: ONE port
    wire [15:0]  tl_to_addr;
    wire         tl_to_stall;
    wire [383:0] tl_to_data;        // ... and its engine side, six words
    wire         tl_mux_stall;      // the adapter's OWN stall, and its one driver

    wire         tl_slot_valid, tl_pix_start, tl_rq_start;
    wire [PE_LANES*PE_ACT_W-1:0] tl_pe_act;
    wire [PE_LANES*WGT_W-1:0]    tl_pe_wgt;
    wire [PE_LANES*PE_ACC_W-1:0] tl_pe_bias;
    wire [1:0]   tl_req_mode;
    wire [PE_OC_MAX*M_W-1:0]     tl_rq_m_o;
    wire [PE_OC_MAX*SHIFT_W-1:0] tl_rq_shift_o;
    wire [PE_OUT_W-1:0]          tl_rq_zero, tl_rq_qmin, tl_rq_qmax;
    wire         tl_rsp_ready, tl_rsp_ready_n, tl_rsp_busy, tl_rsp_done, tl_rsp_overrun;
    wire [31:0]  tl_rsp_mode_viol;
    wire [RQ_LANES*PE_OUT_W-1:0] tl_rsp_y;
    wire [RQ_LANES-1:0]          tl_rsp_y_valid;
    wire [PE_LANES*PE_ACC_W-1:0] tl_rsp_acc, tl_rsp_snapshot;
    wire         tl_mode_safe;

    d64_tail_output_chain #(
        .TAIL_BASE(32'h005C_0000), .H(SH_FRAME_H), .W(SH_FRAME_W),
        .BURST_BEATS(BURST_BEATS), .BQ_DEPTH(BQ_DEPTH),
        .RESERVE_WRITE_BURST(TAIL_RESERVE_WRITE_BURST),
        .W_BASE(238), .W_ADDR_W(12)
    ) u_tail (
        .clk(clk_pe), .rst_n(pe_rst_n), .start(tl_start_i),
        .busy(tl_busy), .done(tl_done), .responses_done(tl_responses_done),
        .done_reason(tl_done_reason), .tail_done(tl_tail_done),
        .tail_cycles(tl_tail_cycles),
        .rq_m(rd_rq_m[4*M_W-1:0]),
        .rq_shift(rd_rq_shift[4*SHIFT_W-1:0]),
        .rq_bias(rd_rq_bias[4*PE_ACC_W-1:0]),
        .in_rd_en(tl_in_en),
        .in_rd_addr(tl_in_addr),
        .in_rd_data(tl_in_data_i),
        .in_stall(tl_in_stall_i),
        .w_rd_en(tl_w_en), .w_rd_addr(tl_w_addr),
        .w_rd_data(rd_w_rd_data[8*4*WGT_W-1:0]),
        .pe_req_slot_valid(tl_slot_valid), .pe_req_act(tl_pe_act),
        .pe_req_wgt(tl_pe_wgt), .pe_req_pix_start(tl_pix_start),
        .pe_req_bias(tl_pe_bias), .pe_req_mode(tl_req_mode),
        .pe_req_rq_start(tl_rq_start), .pe_req_rq_m(tl_rq_m_o),
        .pe_req_rq_shift(tl_rq_shift_o), .pe_req_rq_zero(tl_rq_zero),
        .pe_req_rq_qmin(tl_rq_qmin), .pe_req_rq_qmax(tl_rq_qmax),
        .pe_rsp_rq_ready(tl_rsp_ready), .pe_rsp_rq_ready_n(tl_rsp_ready_n),
        .pe_rsp_y(tl_rsp_y), .pe_rsp_y_valid(tl_rsp_y_valid),
        .pe_rsp_rq_busy(tl_rsp_busy), .pe_rsp_rq_done(tl_rsp_done),
        .pe_rsp_rq_overrun(tl_rsp_overrun),
        .pe_rsp_mode_viol(tl_rsp_mode_viol),
        .pe_rsp_acc(tl_rsp_acc), .pe_rsp_snapshot(tl_rsp_snapshot),
        .mac_mode_safe(tl_mode_safe),
        .axi_awaddr(tw_axi_awaddr), .axi_awlen(tw_axi_awlen),
        .axi_awsize(tw_axi_awsize), .axi_awburst(tw_axi_awburst),
        .axi_awvalid(tw_axi_awvalid), .axi_awready(tw_axi_awready),
        .axi_wvalid(tw_axi_wvalid), .axi_wready(tw_axi_wready),
        .axi_wdata(tw_axi_wdata), .axi_wstrb(tw_axi_wstrb),
        .axi_wlast(tw_axi_wlast),
        .axi_bvalid(tw_axi_bvalid), .axi_bready(tw_axi_bready),
        .axi_bresp(tw_axi_bresp),
        .exp_count(tl_exp_count),
        .tail_pixels(tl_pixels),
        .wr_responses(tl_wr_responses),
        .trk_stall_timeout(tl_stall_timeout),
        .drained(tl_drained)
    );

    assign tl_in_rd_en   = tl_in_en;
    assign tl_in_rd_addr = tl_in_addr;

    // **The six-port adapter, and the switch that feeds it.**  `SAB_TL_FIXTURE
    // = 0` is the design: the tail reads the one runtime cache at ups.1's
    // region, through `u_tlmux`.  `1` is the pre-hop build -- the three
    // boundary ports, which answered from a fixture -- kept so that a green
    // run has a red arm beside it.
    //
    // **`tl_mux_stall` and not `tl_to_stall`.**  `d64_store_rd_mux`'s
    // `e_stall` is an OUTPUT -- `assign e_stall = (NO_IDLE_STALL && idle) ?
    // 1'b0 : ~finish;` (`control/d64_store_rd_mux.v:196`) -- so a net that is
    // both the adapter's `e_stall` and the left-hand side of an `assign` has
    // **two drivers**.  `vlog` reports that with `Errors: 0, Warnings: 0`,
    // ModelSim's `vopt` reports it with no warning either, and Efinity's
    // elaborator refuses it (`VDB-1000`, `VDB-1001`).  The switch therefore
    // reads the adapter's own net and drives a different one, and
    // `tools/check_mux_drivers.py` is the static check that keeps it that way.
    // (`u_u1mux` above has the same shape without the switch, which is why
    // `u1_in_stall_i` has always had exactly one driver.)
    assign tl_in_data_i  = SAB_TL_FIXTURE ? tl_in_rd_data : tl_to_data;
    assign tl_in_stall_i = SAB_TL_FIXTURE ? tl_in_stall   : tl_mux_stall;


    // ======================================================================
    // the one runtime cache, its region and its DDR side
    // ======================================================================
    // `IN_PORTS = 1` and that one port is UPS0's.  See note 1 in the header.
    wire             c_rd_req, c_rd_ack, c_rd_valid, c_rd_last;
    wire [31:0]      c_rd_addr;
    wire [15:0]      c_rd_len;
    wire [8*16-1:0]  c_rd_data;
    wire             c_rd_abort, c_rd_fatal;
    wire [31:0]      c_region_waits;
    wire             c_region_ack;
    wire             c_cache_fatal;
    wire [31:0]      c_axi_errs, c_axi_timeouts, c_axi_late, c_axi_pageref;
    wire             c_axi_rd_err, c_axi_rd_timeout;

    reg  [1:0]       cache_tsel_r;
    wire [31:0]      map_sel_base, map_sel_size;
    wire [31:0]      region_base_r;

    // The map, instantiated -- not a base written down here.  `b74c9d4` is why
    // (a hardcoded `TAIL_BASE` and the map drifted apart, both wrong together).
    // **The map carries the shape too, and it is not decoration.**  `u_map`'s
    // `sel_size` is what the cache fetches and what `exp_count` is derived from;
    // a default-configured map beside correct children is the whole square
    // region handed to a rectangle job.
    d64_net_addr #(
        .RES4_H(SH_RES4_H), .RES4_W(SH_RES4_W),
        .UPS0_H(SH_UPS0_H), .UPS0_W(SH_UPS0_W),
        .UPS1_H(SH_UPS1_H), .UPS1_W(SH_UPS1_W),
        .TAIL_H(SH_UPS1_H), .TAIL_W(SH_UPS1_W)
    ) u_map (
        .tsel(cache_tsel_r), .t_n(16'd0), .t_c(16'd0), .t_h(16'd0), .t_w(16'd0),
        .addr(), .row_base(),
        .sel_base(map_sel_base), .sel_size(map_sel_size),
        .sel_s_c(), .sel_s_w(), .sel_s_h(), .sel_s_n(),
        .sel_n(), .sel_c(), .sel_h(), .sel_w(),
        .sel_row_b(), .sel_rows(), .sel_bursts(), .sel_row_bursts(),
        .sel_row_whole(), .sel_cache_b(), .sel_in_region(),
        .tail_word_addr(), .burst_b(), .cache_rows(),
        .map_lo(), .map_hi(), .map_span(), .map_used()
    );

    // A byte address to a WORD address: `BASE_BYTE >> 4`, once, here.  The
    // adapter shifts back the other way on the AXI side.  The tooth is the
    // shift left out, and it lands every fetch sixteen times past the tensor.
    assign region_base_r = SAB_CACHE_REGION ? map_sel_base
                                            : {4'b0000, map_sel_base[31:4]};

    assign cache_region_base = region_base_r;
    assign cache_region_span = {4'b0000, map_sel_size[31:4]};
    assign cache_region_ack  = c_region_ack;
    assign cache_region_waits = c_region_waits;

    // ---- the cache's single port, and the three engines that take turns ----
    // `c_port_*` are `u_cache`'s own port, and it is the three consumers'
    // *inputs* that are selected: `in_rd_data` is an output and cannot be
    // muxed onto one wire.  The two adapters, the narrowing and the rule that
    // moves the port are stated with the stage bits below, where `B_UPS1` and
    // `B_TAIL` are; the three port-selection wires are declared above the
    // tail's own instance.

    d64_burst_prefetch #(
        .IC_PAR(8), .IN_W(10), .IN_PORTS(1), .IN_WORD_W(16),
        .DDR_LANE_W(16), .SAB_HIGH_LANE(0),
        .BASE_ADDR(32'h0040_0000), .SPAN_WORDS(16384),
        .LINE_WORDS(LINE_WORDS), .LINES(LINES), .BURST_WORDS(BURST_WORDS),
        .BYPASS(0), .SAB_REPOINT(0), .SAB_ABORT(0)
    ) u_cache (
        .clk(clk_pe), .rst_n(pe_rst_n),
        .in_rd_en(c_port_sel_tl ? tl_to_en :
                  (c_port_sel_u1 ? u1_to_en : u0_in_en)),
        .in_rd_addr(c_port_sel_tl ? tl_to_addr :
                    (c_port_sel_u1 ? u1_to_addr : u0_in_addr)),
        .in_rd_data(c_port_data),
        .in_stall(c_port_stall),
        .rd_req(c_rd_req), .rd_addr(c_rd_addr), .rd_len(c_rd_len),
        .rd_ack(c_rd_ack), .rd_data(c_rd_data), .rd_valid(c_rd_valid),
        .rd_last(c_rd_last),
        .rd_abort(c_rd_abort), .rd_fatal(c_rd_fatal),
        .region_req(cache_region_req), .region_base(region_base_r),
        .region_span(cache_region_span),
        .region_ack(c_region_ack), .region_waits(c_region_waits),
        .reads(ddr_reads), .hit_count(), .fill_count(ddr_fills),
        .ddr_bursts(ddr_bursts), .ddr_beats(), .stall_cycles(cache_stalls),
        .addr_viol(cache_viol), .aborts(), .fatal(c_cache_fatal)
    );

    d64_axi_read_burst #(.BEAT_LSB(4), .TIMEOUT(RD_TIMEOUT)) u_rdaxi (
        .clk(clk_pe), .rst_n(pe_rst_n),
        .rd_req(c_rd_req), .rd_addr(c_rd_addr), .rd_len(c_rd_len),
        .rd_ack(c_rd_ack), .rd_data(c_rd_data), .rd_valid(c_rd_valid),
        .rd_last(c_rd_last),
        .axi_arvalid(rd_axi_arvalid), .axi_arready(rd_axi_arready),
        .axi_araddr(rd_axi_araddr), .axi_arlen(rd_axi_arlen),
        .axi_arsize(rd_axi_arsize), .axi_arburst(rd_axi_arburst),
        .axi_rvalid(rd_axi_rvalid), .axi_rready(rd_axi_rready),
        .axi_rdata(rd_axi_rdata), .axi_rlast(rd_axi_rlast),
        .axi_rresp(rd_axi_rresp),
        .bursts(), .beats(), .axi_errors(c_axi_errs), .timeouts(c_axi_timeouts),
        .rd_err(c_axi_rd_err), .rd_timeout(c_axi_rd_timeout),
        .rd_abort(c_rd_abort), .rd_fatal(c_rd_fatal),
        .late_beats(c_axi_late), .page_refusals(c_axi_pageref)
    );

    // ======================================================================
    // the one weight store, and the two owners that take turns on its port
    // ======================================================================
    // F3 6.21.4: the hand-over is safe only in a cycle **both** owners say
    // nothing is in flight, and the owner of the store ANDs the two.  `ws_idle`
    // is the front half's statement and `sw_safe` the reader's; neither is
    // sufficient alone.  **The front half's half cannot be produced as shipped
    // -- see "the hand-over licence" below -- and `SAB_STORE_HANDOVER` is the
    // arm that removes the licence whole.**
    wire             st_rd_en;
    wire [WADDR_W-1:0] st_rd_addr;
    wire [WSTORE_W-1:0] st_rd_data;

    wire [WADDR_W-1:0] rd_st_rd_addr;
    wire [WSTORE_W-1:0] rd_st_rd_data;
    wire             rd_st_rd_en, rd_sw_safe, rd_cfg_settled, rd_win_ok, rd_addr_over;
    wire [WADDR_W-1:0] rd_win_base, rd_win_tiles;
    wire [4:0]       rd_cfg_off, rd_cfg_n;
    wire [PE_OC_MAX-1:0] rd_cfg_lane_ok;
    wire             rd_st_rsp_valid;
    wire             rd_ext_en;
    wire [WADDR_W-1:0] rd_ext_addr;

    assign rd_ext_en   = 1'b0;
    assign rd_ext_addr = {WADDR_W{1'b0}};

    assign st_rd_en   = store_owner ? rd_st_rd_en   : f_ws_req_rd_en;
    assign st_rd_addr = store_owner ? rd_st_rd_addr : f_ws_req_rd_addr;
    assign rd_st_rd_data = st_rd_data;
    assign f_ws_rsp_rd_data = st_rd_data;

    d64_weight_store_abi2_mp2 #(.INIT_FILE(WT_INIT_FILE),.UNGATED_READ(1)) u_wst (
        .clk(clk_pe), .clk_fast(clk_weight_fast), .rst_n(pe_rst_n),
        .wr_en(wt_wr_en), .wr_addr(wt_wr_addr), .wr_data(wt_wr_data),
        .rd_en(st_rd_en), .rd_addr(st_rd_addr), .rd_valid(), .rd_data(st_rd_data)
    );

    d64_decoder_wreader #(
        .W_ADDR_W(WADDR_W), .W_WORD_W(WSTORE_W),
        .OC_MAX(PE_OC_MAX), .M_W(M_W), .SHIFT_W(SHIFT_W),
        .ACC_W(PE_ACC_W), .ZQ_W(PE_OUT_W),
        .W_BASE_U0(184), .W_TILES_U0(36),
        .W_BASE_U1(220), .W_TILES_U1(18),
        .W_BASE_TL(238), .W_TILES_TL(9),
        .CFG_OFF_U0(0), .CFG_OFF_U1(16), .CFG_OFF_TL(24),
        .CFG_N_U0(16), .CFG_N_U1(8), .CFG_N_TL(3),
        .CFG_INIT(DEC_CFG_INIT), .BIAS_INIT(DEC_BIAS_INIT),
        .RUNTIME_CFG(RUNTIME_PROFILE), .CFG_STREAM_SRL(CFG_STREAM_SRL), .CFG_BIAS_W(DEC_CFG_BIAS_W)
    ) u_wr (
        .clk(clk_pe), .rst_n(pe_rst_n),
        .layer(dec_layer),
        .dec_cfg_we(dec_cfg_we), .dec_cfg_addr(dec_cfg_addr), .dec_cfg_data(dec_cfg_data),
        .u0_w_rd_en(u0_w_en), .u0_w_rd_addr(u0_w_addr),
        .u1_w_rd_en(u1_w_en), .u1_w_rd_addr(u1_w_addr),
        .tl_w_rd_en(tl_w_en), .tl_w_rd_addr(tl_w_addr),
        .ext_w_rd_en(rd_ext_en), .ext_w_rd_addr(rd_ext_addr),
        .st_rd_en(rd_st_rd_en), .st_rd_addr(rd_st_rd_addr),
        .st_rd_data(rd_st_rd_data), .w_rd_data(rd_w_rd_data),
        .st_rsp_valid(rd_st_rsp_valid),
        .rq_m(rd_rq_m), .rq_shift(rd_rq_shift), .rq_bias(rd_rq_bias),
        .rq_zero(), .rq_qmin(), .rq_qmax(),
        .cfg_lane_ok(rd_cfg_lane_ok),
        .win_base(rd_win_base), .win_tiles(rd_win_tiles),
        .cfg_off(rd_cfg_off), .cfg_n(rd_cfg_n),
        .win_ok(rd_win_ok), .addr_over(rd_addr_over),
        .sw_safe(rd_sw_safe), .cfg_settled(rd_cfg_settled)
    );

    // The three engines' weight ports are the reader's one answer, sliced to
    // each layer's lane count: ups.0 takes the whole 1024-bit tile, ups.1 the
    // low 512 and the tail the low 256 (F3 section 2).
    assign u0_w_data = rd_w_rd_data;
    assign u1_w_data = rd_w_rd_data[8*8*WGT_W-1:0];
    assign tl_w_data = rd_w_rd_data[8*4*WGT_W-1:0];

    // ======================================================================
    // the arbiter, the ACC25 bridge and the one bank
    // ======================================================================
    // Owner 0 the front half, 1 ups.0, 2 ups.1, 3 the tail.  The buses are
    // concatenations, so the offsets are the arbiter's and not a number
    // retyped here.
    assign arb_slot_i = {tl_slot_valid, u1_slot_valid, u0_slot_valid, f_slot_valid};
    assign arb_pix_i  = {tl_pix_start,  u1_pix_start,  u0_pix_start,  f_pix_start};
    assign arb_rqs_i  = {tl_rq_start,   u1_rq_start,   u0_rq_start,   f_rq_start};
    assign arb_mode_i = {tl_req_mode,   u1_req_mode,   u0_req_mode,   f_req_mode};
    assign arb_safe_i = {tl_mode_safe,  u1_mode_safe,  u0_mode_safe,  f_mac_mode_safe};

    assign arb_act_i  = {tl_pe_act,  u1_pe_act,  u0_pe_act,  f_pe_act};
    assign arb_wgt_i  = {tl_pe_wgt,  u1_pe_wgt,  u0_pe_wgt,  f_pe_wgt};
    assign arb_bias_i = {tl_pe_bias, u1_pe_bias, u0_pe_bias, f_pe_bias};

    // Every owner presents the same 16-entry requant window; the mode picks the
    // slice the bank uses and `cfg_lane_ok` at the reader says which lanes are
    // real.  A narrower owner's bus is zero-extended into the wide one and that
    // is a fill, not a value -- the bank's `qmin`/`qmax` and the engine's
    // `OC_LANES` are what bound it.
    assign arb_m_i    = {tl_rq_m_o,     u1_rq_m_o,     u0_rq_m_o,     f_rq_m};
    assign arb_sh_i   = {tl_rq_shift_o, u1_rq_shift_o, u0_rq_shift_o, f_rq_shift};
    assign arb_zero_i = {tl_rq_zero,    u1_rq_zero,    u0_rq_zero,    f_rq_zero};
    assign arb_qmin_i = {tl_rq_qmin,    u1_rq_qmin,    u0_rq_qmin,    f_rq_qmin};
    assign arb_qmax_i = {tl_rq_qmax,    u1_rq_qmax,    u0_rq_qmax,    f_rq_qmax};

    // the responses back, one slot each
    assign f_rsp_ready      = arb_ready_o[0];
    assign f_rsp_ready_n    = arb_ready_n_o[0];
    assign f_rsp_y          = arb_y_o[0*RQ_LANES*PE_OUT_W +: RQ_LANES*PE_OUT_W];
    assign f_rsp_y_valid    = arb_yv_o[0*RQ_LANES +: RQ_LANES];
    assign f_rsp_busy       = arb_busy_o[0];
    assign f_rsp_done       = arb_done_o[0];
    assign f_rsp_overrun    = arb_over_o[0];
    assign f_rsp_mode_viol  = arb_mv_o[0*32 +: 32];
    assign f_rsp_acc        = arb_acc_o[0*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];
    assign f_rsp_snapshot   = arb_snap_o[0*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];

    assign u0_rsp_ready     = arb_ready_o[1];
    assign u0_rsp_ready_n   = arb_ready_n_o[1];
    assign u0_rsp_y         = arb_y_o[1*RQ_LANES*PE_OUT_W +: RQ_LANES*PE_OUT_W];
    assign u0_rsp_y_valid   = arb_yv_o[1*RQ_LANES +: RQ_LANES];
    assign u0_rsp_busy      = arb_busy_o[1];
    assign u0_rsp_done      = arb_done_o[1];
    assign u0_rsp_overrun   = arb_over_o[1];
    assign u0_rsp_mode_viol = arb_mv_o[1*32 +: 32];
    assign u0_rsp_acc       = arb_acc_o[1*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];
    assign u0_rsp_snapshot  = arb_snap_o[1*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];

    assign u1_rsp_ready     = arb_ready_o[2];
    assign u1_rsp_ready_n   = arb_ready_n_o[2];
    assign u1_rsp_y         = arb_y_o[2*RQ_LANES*PE_OUT_W +: RQ_LANES*PE_OUT_W];
    assign u1_rsp_y_valid   = arb_yv_o[2*RQ_LANES +: RQ_LANES];
    assign u1_rsp_busy      = arb_busy_o[2];
    assign u1_rsp_done      = arb_done_o[2];
    assign u1_rsp_overrun   = arb_over_o[2];
    assign u1_rsp_mode_viol = arb_mv_o[2*32 +: 32];
    assign u1_rsp_acc       = arb_acc_o[2*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];
    assign u1_rsp_snapshot  = arb_snap_o[2*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];

    assign tl_rsp_ready     = arb_ready_o[3];
    assign tl_rsp_ready_n   = arb_ready_n_o[3];
    assign tl_rsp_y         = arb_y_o[3*RQ_LANES*PE_OUT_W +: RQ_LANES*PE_OUT_W];
    assign tl_rsp_y_valid   = arb_yv_o[3*RQ_LANES +: RQ_LANES];
    assign tl_rsp_busy      = arb_busy_o[3];
    assign tl_rsp_done      = arb_done_o[3];
    assign tl_rsp_overrun   = arb_over_o[3];
    assign tl_rsp_mode_viol = arb_mv_o[3*32 +: 32];
    assign tl_rsp_acc       = arb_acc_o[3*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];
    assign tl_rsp_snapshot  = arb_snap_o[3*PE_LANES*PE_ACC_W +: PE_LANES*PE_ACC_W];

    // ---- the bank side, declared before the arbiter that names them ---------
    wire [PE_LANES*BANK_ACC_W-1:0] bank_bias_25, bank_acc_25, bank_snap_25;
    wire [1:0]              arb_bank_mode;
    wire                    arb_bank_slot_valid, arb_bank_pix_start, arb_bank_rq_start;
    wire [PE_LANES*PE_ACT_W-1:0] arb_bank_act;
    wire [PE_LANES*WGT_W-1:0]    arb_bank_wgt;
    wire [PE_LANES*PE_ACC_W-1:0] arb_bank_bias, arb_bank_acc, arb_bank_snap;
    wire [PE_OC_MAX*M_W-1:0]     arb_bank_rq_m;
    wire [PE_OC_MAX*SHIFT_W-1:0] arb_bank_rq_shift;
    wire [PE_OUT_W-1:0]          arb_bank_rq_zero, arb_bank_rq_qmin, arb_bank_rq_qmax;
    wire                    arb_rq_ready, arb_rq_ready_n, arb_rq_busy, arb_rq_done;
    wire                    arb_rq_overrun, arb_bank_idle;
    wire [31:0]             arb_mode_viol;
    wire [RQ_LANES*PE_OUT_W-1:0] arb_y;
    wire [RQ_LANES-1:0]     arb_y_valid;
    wire [31:0]             ups1_words, ups1_beats, ups1_bursts, ups1_acks;
    wire [PE_LANES-1:0]     bias_trunc_live, bias_sign_bad, acc_extended;
    wire [PE_LANES-1:0]     bias_whole_bus_diff, acc_widen_diff;

    d64_pe_owner #(
        .NOWNERS(NOWNERS), .LANES(PE_LANES), .OC_MAX(PE_OC_MAX),
        .ACT_W(PE_ACT_W), .WGT_W(WGT_W), .ACC_W(PE_ACC_W), .M_W(M_W),
        .SHIFT_W(SHIFT_W), .OUT_W(PE_OUT_W), .RQ_LANES(RQ_LANES)
    ) u_arb (
        .clk(clk_pe), .rst_n(pe_rst_n),
        .owner_sel(arb_owner_sel), .sel_req(arb_sel_req), .sel_ack(arb_sel_ack),
        .pe_req_slot_valid_i(arb_slot_i), .pe_req_act_i(arb_act_i),
        .pe_req_wgt_i(arb_wgt_i), .pe_req_pix_start_i(arb_pix_i),
        .pe_req_bias_i(arb_bias_i), .pe_req_mode_i(arb_mode_i),
        .pe_req_rq_start_i(arb_rqs_i), .pe_req_rq_m_i(arb_m_i),
        .pe_req_rq_shift_i(arb_sh_i), .pe_req_rq_zero_i(arb_zero_i),
        .pe_req_rq_qmin_i(arb_qmin_i), .pe_req_rq_qmax_i(arb_qmax_i),
        .mac_mode_safe_i(arb_safe_i),
        .pe_rsp_rq_ready_i(arb_ready_o), .pe_rsp_rq_ready_n_i(arb_ready_n_o),
        .pe_rsp_y_i(arb_y_o), .pe_rsp_y_valid_i(arb_yv_o),
        .pe_rsp_rq_busy_i(arb_busy_o), .pe_rsp_rq_done_i(arb_done_o),
        .pe_rsp_rq_overrun_i(arb_over_o), .pe_rsp_mode_viol_i(arb_mv_o),
        .pe_rsp_acc_i(arb_acc_o), .pe_rsp_snapshot_i(arb_snap_o),
        .bank_rq_ready(arb_rq_ready), .bank_rq_ready_n(arb_rq_ready_n),
        .bank_y(arb_y), .bank_y_valid(arb_y_valid),
        .bank_rq_busy(arb_rq_busy), .bank_rq_done(arb_rq_done),
        .bank_rq_overrun(arb_rq_overrun), .bank_mode_viol(arb_mode_viol),
        .bank_acc(arb_bank_acc), .bank_rq_snap(arb_bank_snap),
        .bank_mode(arb_bank_mode), .bank_slot_valid(arb_bank_slot_valid),
        .bank_act(arb_bank_act), .bank_wgt(arb_bank_wgt),
        .bank_pix_start(arb_bank_pix_start), .bank_bias(arb_bank_bias),
        .bank_rq_start(arb_bank_rq_start), .bank_rq_m(arb_bank_rq_m),
        .bank_rq_shift(arb_bank_rq_shift), .bank_rq_zero(arb_bank_rq_zero),
        .bank_rq_qmin(arb_bank_rq_qmin), .bank_rq_qmax(arb_bank_rq_qmax),
        .bank_idle(arb_bank_idle)
    );

    d64_acc25_bridge #(
        .LANES(PE_LANES), .SEAM_W(PE_ACC_W), .BANK_W(BANK_ACC_W)
    ) u_br (
        .seam_bias(arb_bank_bias), .seam_acc(arb_bank_acc),
        .seam_snap(arb_bank_snap),
        .bank_bias(bank_bias_25), .bank_acc(bank_acc_25), .bank_snap(bank_snap_25),
        .bias_trunc_live(bias_trunc_live), .bias_sign_bad(bias_sign_bad),
        .acc_extended(acc_extended),
        .bias_whole_bus_diff(bias_whole_bus_diff),
        .acc_widen_diff(acc_widen_diff)
    );

    d64_mac128 #(
        .FAULT_STICKY(FAULT_STICKY),
        .ACT_W(PE_ACT_W), .WGT_W(WGT_W), .ACC_W(BANK_ACC_W), .M_W(M_W),
        .SHIFT_W(SHIFT_W), .OUT_W(PE_OUT_W), .RQ_LANES(RQ_LANES),
        .LANES(PE_LANES), .ACC_N(32), .OC_MAX(PE_OC_MAX),
        .RQ_LOOKAHEAD(BANK_RQLA),.ACT_INPUT_REG(1)
    ) u_bank (
        .clk(clk_pe), .rst_n(pe_rst_n), .mode(arb_bank_mode),
        .slot_valid(arb_bank_slot_valid), .act(arb_bank_act), .wgt(arb_bank_wgt),
        .pix_start(arb_bank_pix_start), .bias(bank_bias_25),
        .rq_start(arb_bank_rq_start), .rq_ready(arb_rq_ready),
        .rq_ready_n(arb_rq_ready_n), .rq_m(arb_bank_rq_m),
        .rq_shift(arb_bank_rq_shift), .rq_zero(arb_bank_rq_zero),
        .rq_qmin(arb_bank_rq_qmin), .rq_qmax(arb_bank_rq_qmax),
        .y(arb_y), .y_valid(arb_y_valid), .rq_busy(arb_rq_busy),
        .rq_done(arb_rq_done), .rq_overrun(arb_rq_overrun),
        .mode_viol(arb_mode_viol), .acc_o(bank_acc_25), .rq_snap_o(bank_snap_25),
        .obs_s_acc0(), .obs_s_m0(), .obs_s_shift0(), .obs_s_ok0(), .obs_run()
    );

    assign front_done = f_done;
    assign owner_viol     = f_owner_viol;
    assign bank_mode_viol = arb_mode_viol | f_mode_viol;

    localparam [3:0] B_FRONT = 4'b1000, B_UPS0 = 4'b0100,
                     B_UPS1  = 4'b0010, B_TAIL = 4'b0001;

    reg [3:0] sen_q;        // stages still to run
    reg [3:0] run_q;        // the stage running now, one-hot; 0 is idle
    reg       start_q, armed_q, region_done_r, started_q, front_done_q;

    // One physical UPS writer. The engine stages are strictly sequential.
    // Snapshot UPS0 before clearing the drained panel pool for UPS1. Reuse the
    // W0 AXI endpoint; W1 stays electrically inactive, never fabricates a reply.
    reg [1:0] ups_writer_region;
    reg [1:0] ups_writer_hold;
    reg [255:0] ups0_saved;
    wire [31:0] sw_words,sw_beats,sw_bursts,sw_acks;
    wire [31:0] sw_bresp,sw_overflows,sw_duplicates,sw_oob;
    wire sw_drained;
    wire ups_stage=(run_q==B_UPS0)||(run_q==B_UPS1);
    wire [1:0] ups_wanted=(run_q==B_UPS1)?T_UPS1:T_UPS0;
    wire ups_writer_prepared=!ups_stage ||
        ((ups_writer_region==ups_wanted)&&(ups_writer_hold==0));
    wire sw_reset_n=pe_rst_n && (ups_writer_hold==0);
    wire sw_is_u1=(ups_writer_region==T_UPS1);
    wire sw_we=(ups_writer_hold==0) && (sw_is_u1?u1_ow_en:u0_ow_en);
    always @(posedge clk_pe or negedge pe_rst_n)begin
        if(!pe_rst_n)begin ups_writer_region<=T_UPS0;ups_writer_hold<=0;ups0_saved<=0;end
        else if(ups_writer_hold!=0)ups_writer_hold<=ups_writer_hold-1'b1;
        else if(ups_stage && ups_writer_region!=ups_wanted && sw_drained)begin
            if(ups_writer_region==T_UPS0)
                ups0_saved<={sw_words,sw_beats,sw_bursts,sw_acks,sw_bresp,sw_overflows,sw_duplicates,sw_oob};
            ups_writer_region<=ups_wanted;
            ups_writer_hold<=2; // reset metadata/counters for two quiet clocks
        end
    end
    wire [255:0] sw_metrics={sw_words,sw_beats,sw_bursts,sw_acks,sw_bresp,sw_overflows,sw_duplicates,sw_oob};
    assign {ups0_words,ups0_beats,ups0_bursts,ups0_acks,ups0_bresp_errors,ups0_overflows,ups0_duplicates,ups0_oob}=
        (ups_writer_region==T_UPS0)?sw_metrics:ups0_saved;
    assign {ups1_words,ups1_beats,ups1_bursts,ups1_acks,ups1_bresp_errors,ups1_overflows,ups1_duplicates,ups1_oob}=
        (ups_writer_region==T_UPS1)?sw_metrics:256'd0;
    d64_ups_u16_writer #(
        .FAULT_STICKY(FAULT_STICKY),.TSEL(T_UPS0),.DYNAMIC_REGION(1),.RESERVE_PHASE(1),
        .ROW_WORDS(256),.PANEL_BEATS(64),.NBUF(8),
        .WORD_CH(8),.VALUE_W(8),.DDR_LANE_W(16)) u_ups_writer(
        .reserve_en(sw_is_u1?u1_write_reserve:u0_write_reserve),
        .reserve_row(sw_is_u1?u1_write_row:u0_write_row),
        .reserve_addr(sw_is_u1?u1_write_addr:u0_write_addr),.reserve_ready(ups_write_ready),
        .clk(clk_pe),.rst_n(sw_reset_n),.region_sel(ups_writer_region),
        .out_wr_en(sw_we),.out_wr_addr(sw_is_u1?u1_ow_addr:u0_ow_addr),
        .out_wr_row_abs(sw_is_u1?u1_ow_rowabs:u0_ow_rowabs),
        .out_wr_data(sw_is_u1?u1_ow_data:u0_ow_data),
        .axi_awaddr(w0_axi_awaddr),.axi_awlen(w0_axi_awlen),
        .axi_awsize(w0_axi_awsize),.axi_awburst(w0_axi_awburst),
        .axi_awvalid(w0_axi_awvalid),.axi_awready(w0_axi_awready),
        .axi_wvalid(w0_axi_wvalid),.axi_wready(w0_axi_wready),
        .axi_wdata(w0_axi_wdata),.axi_wstrb(w0_axi_wstrb),.axi_wlast(w0_axi_wlast),
        .axi_bvalid(w0_axi_bvalid),.axi_bready(w0_axi_bready),.axi_bresp(w0_axi_bresp),
        .words_in(sw_words),.beats_out(sw_beats),.bursts_out(sw_bursts),.wr_ack(),.wr_acks(sw_acks),
        .bresp_errors(sw_bresp),.oob_words(sw_oob),.duplicate_words(sw_duplicates),
        .panel_overflows(sw_overflows),.stall_cycles(),.panels_ready(),.drained(sw_drained),
        .region_base(),.region_size(),.region_row_b(),.region_burst_b());
    assign w1_axi_awvalid=1'b0;assign w1_axi_awaddr=32'd0;assign w1_axi_awlen=8'd0;
    assign w1_axi_awsize=3'd4;assign w1_axi_awburst=2'd1;
    assign w1_axi_wvalid=1'b0;assign w1_axi_wdata=128'd0;assign w1_axi_wstrb=16'd0;
    assign w1_axi_wlast=1'b0;assign w1_axi_bready=1'b1;

    // ======================================================================
    // the job sequencer
    // ======================================================================
    // One frame is a sequence of stages, each one owner of the shared bank.
    // `stage_en` is the job's mask, `{front, ups.0, ups.1, tail}`; the set bits
    // run MSB first and then the job ends.  The stages are sequential clients
    // of one bank, one store and one cache, so a hand-over between them is a
    // stage boundary and never an arbitration -- the plan's job order, and the
    // shape every gate in this tree has built.
    //
    // Four things must be true before a stage may be started, and each is a
    // state and not a delay:
    //   1. the arbiter acknowledged the owner selection -- and it acknowledges
    //      only at `mac_mode_safe & bank_idle` (F2 section 2.1);
    //   2. the store's read port is on the right side -- handed over only while
    //      `ws_idle & sw_safe` (F3 6.21.4);
    //   3. the cache acknowledged the region for this stage's input;
    //   4. the reader's requant table settled (`cfg_settled`).
    // ======================================================================
    // the one store port, and the two engines that take turns on it
    // ======================================================================
    // `u_cache` keeps ONE engine-facing port and ups.1's engine presents TWO
    // (`docs/F3_INTERFACES_2026-10-07.md` 6.9: "UPS0 reads one port, UPS1 two,
    // TAIL six"), so the second needs an adapter and not a second cache: a
    // two-port `d64_burst_prefetch` is 84,029 LUT4 against a device with 60,800
    // (`control/d64_store_rd_mux.v`'s header).  The adapter spends two cycles on
    // the two words of a group and holds ups.1 with `in_stall` while it does,
    // which is the contract the cache itself keeps.
    //
    // **The narrowing is not a convenience.**  The cache's port word is
    // `IC_PAR(8) * IN_W(10)` = 80 bits and `d64_tconv_engine.v:443` slices an
    // 8-bit-packed word, so `SW = 80, IC_PAR = 8` takes bits [7:0] of each
    // 10-bit lane.  Feeding the 80-bit word straight in would read channel 1
    // out of bits 8..15 of lane 0: a wrong activation at the right address, and
    // no address check can see it.
    //
    // **`NO_IDLE_STALL = 1` is load-bearing.**  `d64_tconv_engine` registers
    // `in_stall` as a level that holds its schedule (`:324-334`,
    // `d64_tconv_scheduler.v:254-270`) and does not AND it with a read-valid,
    // so a store that stalls a consumer asking for nothing stops the engine
    // presenting its first slot.  Measured at 0 on ups.1: 300,329 cycles, 0
    // groups served.  The TAIL's adapter leaves it 0 and reproduces the
    // measured tail numbers net for net; a decoder layer sets 1.
    //
    // `c_port_sel_u1` is `run_q == B_UPS1` and `c_port_sel_tl` is
    // `run_q == B_TAIL`, and nothing else: ups.0 owns the port in every other
    // stage.  Through the switch **every other consumer is held** --
    // `u0_in_stall` forced high, and each adapter's `s_stall` forced high --
    // which is the same margin the store's own hand-over keeps
    // (`mac_mode_safe & bank_idle`, "the hand-over licence" below).
    assign c_port_sel_u1 = (run_q == B_UPS1);
    assign c_port_sel_tl = (run_q == B_TAIL);

    assign u0_in_data  = (c_port_sel_u1 | c_port_sel_tl)
                       ? {(U0_PIX*8*10){1'b0}} : c_port_data;
    assign u0_in_stall = (c_port_sel_u1 | c_port_sel_tl)
                       ? 1'b1                  : c_port_stall;
    assign u1_to_stall = c_port_sel_u1 ? c_port_stall : 1'b1;
    assign tl_to_stall = c_port_sel_tl ? c_port_stall : 1'b1;

    d64_store_rd_mux #(
        .IN_PORTS(2), .AW(16), .DW(64), .SW(80), .IC_PAR(8),
        .NO_IDLE_STALL(1), .SAB_SWAP(0)
    ) u_u1mux (
        .clk(clk_pe), .rst_n(pe_rst_n),
        .e_en(u1_in_en), .e_addr(u1_in_addr),
        .e_stall(u1_in_stall_i), .e_data(u1_in_data_i),
        .s_en(u1_to_en), .s_addr(u1_to_addr),
        .s_data(c_port_data), .s_stall(u1_to_stall),
        .transfers(), .stall_cycles(), .groups(), .mux_cycles()
    );

    // ======================================================================
    // the TAIL's adapter: six read ports onto the one store port
    // ======================================================================
    // The tail presents SIX ports (F3 6.9: "UPS0 reads one port, UPS1 two,
    // TAIL six") and the same `u_cache` serves all three owners -- a
    // six-port `d64_burst_prefetch` is 270,794 LUT4 on a 60,800-LUT4 device
    // (`control/d64_store_rd_mux.v`'s header), so the port count is a
    // parameter of the adapter and not of the cache.
    //
    // `SW = 80, IC_PAR = 8` is ups.1's narrowing and it is the tail's too: the
    // cache's port word is `IC_PAR(8) * IN_W(10)` and `d64_tail_engine.v`
    // slices its input **8 bits a channel out of an 8-bit-packed word**, so
    // taking bits [7:0] of each 10-bit lane is what makes channel `c` of the
    // engine's word the byte the writer stored in container `c`.  Handing the
    // 80-bit word straight in would read channel 1 from bits 8..15 of lane 0.
    //
    // `NO_IDLE_STALL = 1`, and here it is **not** load-bearing, which is worth
    // saying rather than copying.  The tail reads its stall through
    // `wire s_stall = s_rd && in_stall` (`control/d64_tail_engine.v:513`), so a
    // stall in a cycle the engine presents no read has no effect on it at all;
    // 1 is the cache's own rule about its own port and the reading
    // `tb_d64_chain_tail`'s verified one-port arm ran at.  It is the *decoder*
    // layer that requires 1 -- see the comment on `u_u1mux` above.
    d64_store_rd_mux #(
        .IN_PORTS(6), .AW(16), .DW(64), .SW(80), .IC_PAR(8),
        .NO_IDLE_STALL(1), .SAB_SWAP(0)
    ) u_tlmux (
        .clk(clk_pe), .rst_n(pe_rst_n),
        .e_en(tl_in_en), .e_addr(tl_in_addr),
        .e_stall(tl_mux_stall), .e_data(tl_to_data),
        .s_en(tl_to_en), .s_addr(tl_to_addr),
        .s_data(c_port_data), .s_stall(tl_to_stall),
        .transfers(), .stall_cycles(), .groups(), .mux_cycles()
    );

    // **The hand-over licence, and the half of it that cannot be produced.**
    //
    // F3 6.21.4 asks the owner of the store to AND the outgoing client's
    // `ws_idle` with the incoming client's `sw_safe`.  Measured on the shipped
    // RTL, `ws_idle` **is zero in every cycle the trunk side owns the port**:
    // `d64_conv_engine.v:559` is `assign pe_wt_rd_en = 1'b1;` -- "this engine
    // reads every cycle" -- and the front half's `ws_idle` is
    // `~(wt_rd_en | ws_rd_q)` (`d64_encoder_trunk.v:1035`), so a front half whose
    // owner FSM sits in `O_IDLE` still presents a store read and still reports
    // "something is in flight".  Probe, at 100 us with the front half never
    // started: `owner_q 0, sel_enc 0, t_pe_wt_rd_en 1, wt_rd_en 1, ws_idle 0`.
    //
    // What that costs, and what it does not: **not the data**.  `d64_weight_store`
    // gates its output with the enable of the cycle the read was *transferred*
    // (F3 6.21.2), so an incoming client gets zero in every cycle whose
    // predecessor transferred nothing and cannot pick up the outgoing client's
    // last word -- 6.21.4's own item 3, and the reason it says that margin "is no
    // longer load-bearing".  What it costs is the *licence*: a job that runs the
    // front half cannot satisfy the rule as written, and no gate in this tree has
    // ever run one.
    //
    // So the licence here is the contract's, with one term added for the case
    // the contract does not cover: **a front half that is not a client of this
    // job has nothing in flight**.  `front_owed` is true from the moment the job
    // names the front stage until that stage ends, and while it is true the
    // contract's AND is required -- which is the deadlock the report records
    // rather than hides.  `SAB_STORE_HANDOVER` removes the whole licence.
    wire front_owed = sen_q[3] | front_done_q;
    wire store_handover_ok = SAB_STORE_HANDOVER ? 1'b1
                           : (rd_sw_safe & (f_ws_idle | ~front_owed));

    // ---- defect 3: ADMISSION, and a `start` while busy is refused ----------
    // There was no admission control at all.  `start`'s rising edge overwrote
    // `sen_q`, `run_q` and `armed_q` -- the whole job's bookkeeping -- while a
    // child could still be in the middle of the *previous* job, so a second
    // start named a new stage list over a running one and neither job's
    // hand-over conditions were true of the leaves.  `job_idle` is the whole
    // rule: a start is taken only when no stage is running and none is
    // pending.
    //
    // **Idle-only acceptance with a reject counter, and not a full abort, is
    // the choice, and this is why:** an abort has to unwind whatever a child
    // is holding -- the bank's owner, the weight store's read port, the
    // cache's region, a writer's half-issued burst -- and not one of those has
    // a withdraw port in the contract.  A counter is honest about what this
    // root can actually do; an abort that silently leaves a child mid-burst
    // would be a worse defect than the one it closed.  `admit_rejects` is the
    // number, and the arm asserts it.
    wire job_idle = (run_q == 4'd0) && (sen_q == 4'd0);

    // ---- the HOLD DOMAIN's registers ---------------------------------------
    // Declared here because the sequencer reads them and this flow makes an
    // undeclared name an implicit one-bit net.  **They are reset by `arst_n`
    // alone** -- the block is at the end of the file.
    reg          fault_q, reset_req_r;
    reg [31:0]   reset_pulses_r;
    reg [3:0]    reset_reason_r;
    reg          job_failed_r, mask_bad_r;
    reg [31:0]   admit_rejects_r, job_fails_r;

    // the `clk_pe` domain's one-cycle messages into it
    reg          admit_reject_p, mask_bad_p, stage_halt_p;
    // ... and the level the outputs block has to see on the very next edge.
    // **`job_bad` is read the cycle after the job is decided to be bad, and
    // `job_failed_r`/`mask_bad_r` are a cycle behind that** -- they are the
    // hold domain's copies, written from the `clk_pe` pulses.  Without this
    // register the outputs block sees a clean `job_bad` for exactly one cycle
    // and latches `done`; both arms found it.  The injection arm measured
    // `stage 0 done 1 failed 1` on a bad transport and the mask arm measured
    // the same on an empty stage mask.
    //
    // It is the sequencer's own verdict, in the sequencer's own domain, and it
    // is cleared only by the next accepted start.
    reg          job_bad_r;

    // `job_bad` is "this job did not finish": it must not read `done` on an
    // empty stage mask or on a stage that retired with a fault.
    wire job_bad  = job_failed_r | mask_bad_r | job_bad_r;
    wire job_over = (run_q == 4'd0) && (sen_q == 4'd0) && started_q;

    reg       front_start_r, u0_start_r, u1_start_r, tl_start_r;
    reg [1:0] arb_owner_sel_r, layer_r;
    reg       arb_sel_req_r, store_owner_r, region_req_r;

    function [3:0] first_bit;
        input [3:0] m;
        begin
            if      (m[3]) first_bit = B_FRONT;
            else if (m[2]) first_bit = B_UPS0;
            else if (m[1]) first_bit = B_UPS1;
            else           first_bit = B_TAIL;
        end
    endfunction

    function [1:0] owner_of;
        input [3:0] b;
        begin
            if      (b == B_FRONT) owner_of = 2'd0;
            else if (b == B_UPS0)  owner_of = 2'd1;
            else if (b == B_UPS1)  owner_of = 2'd2;
            else                   owner_of = 2'd3;
        end
    endfunction

    // the tensor a stage READS: UPS0 reads RES4, UPS1 reads UPS0, the tail
    // reads UPS1.  The map's encoding, named once.
    function [1:0] tsel_of;
        input [3:0] b;
        begin
            if      (b == B_UPS0) tsel_of = T_RES4;
            else if (b == B_UPS1) tsel_of = T_UPS0;
            else                  tsel_of = T_UPS1;
        end
    endfunction

    function [1:0] layer_of;
        input [3:0] b;
        begin
            if      (b == B_UPS0) layer_of = 2'd0;
            else if (b == B_UPS1) layer_of = 2'd1;
            else                  layer_of = 2'd2;
        end
    endfunction

    assign front_start      = front_start_r;
    assign u0_start_i       = u0_start_r;
    assign u1_start_i       = u1_start_r;
    assign tl_start_i       = tl_start_r;
    assign arb_owner_sel    = arb_owner_sel_r;
    assign arb_sel_req      = arb_sel_req_r;
    assign dec_layer        = layer_r;
    assign store_owner      = store_owner_r;
    assign cache_region_req = region_req_r;
    assign dec_owns_store_seen = store_owner_r;

    // the stage's completion, as one expression
    // **A stage ends when its work is committed, not when its engine stops.**
    // F3 6.8: "the engine stopped", "the responses arrived" and "the frame may
    // be displayed" are three separate states.  The writer commits one beat a
    // word and one response a burst, so its drain is `words == beats` **and**
    // `bursts == responses` -- and the first run of this gate measured exactly
    // that gap: the engine's last 71 words were still in the writer's queue
    // when `u0_done` rose.  The tail gets the same rule from its own tracker,
    // whose `done` is already `publishable && drained` (F3 6.20).
    wire w0_drained = (ups0_words == ups0_beats) & (ups0_bursts == ups0_acks)
                     & ((ups_writer_region==T_UPS0) && sw_drained && ups_writer_hold==0);
    wire w1_drained = (ups1_words == ups1_beats) & (ups1_bursts == ups1_acks)
                     & ((ups_writer_region==T_UPS1) && sw_drained && ups_writer_hold==0);

    // **... and that count is a legal DRAIN, not a verdict.**  The writer's bus
    // FSM: "The response retires whatever it says; the verdict travels beside
    // it."  `wr_acks` counts the error `bresp` too, so a transport that failed
    // reaches `words == beats && bursts == acks` exactly like one that
    // succeeded -- which is defect 2.  The verdict is these four counters, each
    // one a number the writer raises by itself and none of them tied off.
    wire w0_clean = (ups0_bresp_errors == 32'd0) & (ups0_overflows == 32'd0)
                  & (ups0_duplicates    == 32'd0) & (ups0_oob       == 32'd0);
    wire w1_clean = (ups1_bresp_errors == 32'd0) & (ups1_overflows == 32'd0)
                  & (ups1_duplicates    == 32'd0) & (ups1_oob       == 32'd0);

    // The frame's own fault, and it is the OTHER polarity.  On the chain
    // `tl_done` is `publishable && drained` (F3 6.20), so a successful `done`
    // *already* requires `responses_done` and `tl_done && !tl_responses_done`
    // is a contradiction that cannot fire -- it was written for the bad-B case
    // and could never report it.  The case left over is the real one: every
    // response arrived and the frame is still not publishable.
    //
    // **`tl_stall_timeout` is added here, and it is the second half of the
    // same rule.**  A transport that never answers does not reach
    // `responses_done` at all, so the first term cannot see it -- and the
    // chain's tracker is the instrument that does, after `IDLE_TIMEOUT` cycles
    // of silence.  It retires the stage (`tl_transport_retired`) and it fails
    // it, and the two together are what turn "no response" from a permanent
    // wait into a verdict.
    wire frame_fault  = (tl_responses_done & ~tl_done) | tl_stall_timeout;
    wire depth_fault  = (u0_rq_overrun != 32'd0) | (u1_rq_overrun != 32'd0);
    wire any_fatal    = c_cache_fatal | c_rd_fatal | front_cfg_protocol_fault;

    // **The engine's `done` is an event and the writer's drain is later, so the
    // first is remembered.**  Measured by this gate's first run: `u0_done` rose
    // with the engine's last 71 words still in the writer's queue, so a plain
    // conjunction of "the engine finished" and "the writes committed" **never
    // held in any cycle** -- the frame reached the memory beat for beat and the
    // job never reported completion.  A stage therefore ends when both have
    // happened, in either order.
    // The tail's own three conditions, and they are three because the tail's
    // chain already names them:
    //
    //   `tl_tail_done`      the ENGINE stopped.  **Not a success.**
    //   `tl_responses_done` every expected response arrived.
    //   `tl_done`           `publishable && drained`: the frame may be shown.
    //
    // **The defect this replaces.**  The tail's branch of `eng_finished` used
    // to select `tl_done`, the *success* signal.  A bad `BRESP` retires its
    // transaction like any other, so the tail reaches
    // `responses_done = 1, publishable = 0` -- and `tl_done` therefore never
    // rose, `eng_done_q` was never remembered, `run_retired` never held, and
    // the stage never ended.  `job_over` is `run_q == 0`, so `failed`'s
    // `(job_over & (transport_fault | frame_fault))` term was blocked too:
    // **busy 1, done 0, failed 0, for ever**, on a job whose every transaction
    // had already drained.  Measured in `sim/_agent_root_tail/` with
    // `tools/run_root_tail_fault.sh` before the change and after it.
    //
    // So the end of the *work* is the engine's stop, the end of the *transport*
    // is the drain, and the *verdict* is `frame_fault` -- which is what makes
    // a failed job retire and then fail rather than wait.
    //
    // **`tl_stall_timeout` retires the transport too, and that is a choice.**
    // A response that never comes leaves `responses_done` and `drained` at 0
    // for ever; the chain's tracker declares the stall after `IDLE_TIMEOUT`
    // (4096) cycles of silence and reports it on `stall_timeout`.  A job that
    // waited on a transport the tracker has already declared dead would be the
    // same permanent wait in a different guise, so the timeout ends the wait
    // and `frame_fault` makes it a failure -- a definite terminal state, and
    // not a published frame.
    wire tl_transport_retired = (tl_responses_done & tl_drained) | tl_stall_timeout;

    wire eng_finished = (run_q == B_FRONT) ? f_done :
                        (run_q == B_UPS0)  ? u0_done :
                        (run_q == B_UPS1)  ? u1_done : tl_tail_done;
    wire committed    = (run_q == B_UPS0) ? w0_drained :
                        (run_q == B_UPS1) ? w1_drained :
                        (run_q == B_TAIL) ? tl_transport_retired : 1'b1;

    reg eng_done_q;
    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n)          eng_done_q <= 1'b0;
        else if (run_q == 4'd0) eng_done_q <= 1'b0;
        else if (eng_finished)  eng_done_q <= 1'b1;
    end

    wire run_retired = eng_done_q & committed;

    wire transport_clean = (run_q == B_UPS0) ? w0_clean :
                           (run_q == B_UPS1) ? w1_clean : 1'b1;

    // ---- the three conditions, named once ---------------------------------
    // `stage_finished`  the stage's engine stopped (an event, remembered)
    // `transport_retired` and its transport legally drained
    // `stage_success`   and nothing was reported bad about it
    // They are three because they can be three: the middle one is true of a
    // failed transport, and a job that has to wait for the drain must still be
    // able to say the drain was not a success.
    // **All three are gated on a stage being CURRENT, and that gate is a
    // finding, not tidiness.**  `committed` and `transport_clean` are muxes on
    // `run_q` whose default arm is `1'b1` -- there is no bad answer for a stage
    // that is not running -- and `eng_done_q` is remembered one cycle past the
    // end of the stage (`run_q == 0` clears it on the *next* edge).  Without
    // this term the two of them meet for exactly one cycle after a bad stage
    // ends and `stage_success` reads **1 over a transport that failed**.  The
    // injection arm measured it: `saw stage_success 1` on a job that correctly
    // halted and correctly reported `failed`.
    wire stage_running = (run_q != 4'd0);

    assign stage_finished    = stage_running & eng_done_q;
    assign transport_retired = stage_running & run_retired;
    assign stage_success     = stage_running & eng_done_q & committed & transport_clean
                             & ~frame_fault & ~any_fatal & ~depth_fault;

    wire transport_fault = (ups0_bresp_errors != 32'd0) | (ups1_bresp_errors != 32'd0)
                         | (ups0_overflows     != 32'd0) | (ups1_overflows     != 32'd0)
                         | (ups0_duplicates    != 32'd0) | (ups1_duplicates    != 32'd0)
                         | (ups0_oob           != 32'd0) | (ups1_oob           != 32'd0);
    wire run_is_front = (run_q == B_FRONT);
    wire run_needs_region = (run_q != B_FRONT) && (run_q != 4'd0);

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            sen_q           <= 4'd0;
            run_q           <= 4'd0;
            start_q         <= 1'b0;
            armed_q         <= 1'b0;
            region_done_r   <= 1'b0;
            front_start_r   <= 1'b0;
            u0_start_r      <= 1'b0;
            u1_start_r      <= 1'b0;
            tl_start_r      <= 1'b0;
            arb_owner_sel_r <= 2'd0;
            arb_sel_req_r   <= 1'b0;
            layer_r         <= 2'd0;
            store_owner_r   <= 1'b0;
            region_req_r    <= 1'b0;
            cache_tsel_r    <= T_RES4;
            front_done_q    <= 1'b0;
            job_bad_r       <= 1'b0;
        end
        else begin
            start_q       <= start;
            front_start_r <= 1'b0;
            u0_start_r    <= 1'b0;
            u1_start_r    <= 1'b0;
            tl_start_r    <= 1'b0;
            region_req_r  <= 1'b0;
            admit_reject_p <= 1'b0;
            mask_bad_p     <= 1'b0;
            stage_halt_p   <= 1'b0;

            // ---- the start, and the admission (defect 3) ------------------
            if (start & ~start_q) begin
                if (job_idle) begin
                    sen_q <= stage_en;
                    run_q <= 4'd0;
                    armed_q <= 1'b0;
                    region_done_r <= 1'b0;
                    front_done_q <= 1'b0;
                    // **An empty mask names no stage, and it is an ERROR rather
                    // than a wait.**  `4'b0000` used to fall straight through
                    // `first_bit` into a job that never picked a stage and then
                    // reported `done`, because `started_q` was set and `sen_q`
                    // was empty: a successful-looking end to a job that never
                    // ran.  It raises `mask_bad` here instead, and the job is
                    // over as a failure on the spot -- no stage, no wait, and
                    // no `done`.
                    mask_bad_p <= (stage_en == 4'b0000);
                    // the sequencer's own verdict, cleared by every accepted
                    // start so a good job after a bad one may finish
                    job_bad_r  <= (stage_en == 4'b0000);
                end
                else begin
                    // Busy.  Refuse, count it, and **touch nothing**: the job
                    // that is running keeps its stage list, its arming and its
                    // child's job.
                    admit_reject_p <= 1'b1;
                end
            end
            else if (run_q == 4'd0) begin
                if (sen_q != 4'd0) begin
                    run_q <= first_bit(sen_q);
                    armed_q <= 1'b0;
                    region_done_r <= 1'b0;
                end
            end
            else begin
                // ---- the arbiter's owner selection ------------------------
                // `sel_req` is a **level**, held for the whole stage: the
                // arbiter acknowledges immediately when `owner_sel` already
                // names the selected owner, and a request that is dropped the
                // cycle after it is made never sees an acknowledgement at all.
                if (arb_owner_sel_r != owner_of(run_q)) arb_owner_sel_r <= owner_of(run_q);
                arb_sel_req_r <= 1'b1;

                // ---- the store's read port --------------------------------
                // Both owners must say nothing is in flight (F3 6.21.4).
                if (!run_is_front && store_handover_ok) store_owner_r <= 1'b1;
                if ( run_is_front && store_handover_ok) store_owner_r <= 1'b0;

                // ---- the layer window and the cache's region --------------
                if (run_needs_region) begin
                    if (rd_sw_safe) begin
                        layer_r      <= layer_of(run_q);
                        cache_tsel_r <= tsel_of(run_q);
                    end
                    if (!region_done_r) region_req_r <= 1'b1;
                    if (c_region_ack)   region_done_r <= 1'b1;
                end

                // ---- the start, once every condition holds ----------------
                if (!armed_q && arb_sel_ack
                    && (arb_owner_sel_r == owner_of(run_q))
                    && (run_is_front ? (store_owner_r == 1'b0)
                                     : (store_owner_r == 1'b1))
                    && (run_is_front ? 1'b1 : (region_done_r && rd_cfg_settled))
                    && ups_writer_prepared) begin
                    armed_q <= 1'b1;
                    if (run_q == B_FRONT) front_start_r <= 1'b1;
                    if (run_q == B_UPS0)  u0_start_r    <= 1'b1;
                    if (run_q == B_UPS1)  u1_start_r    <= 1'b1;
                    if (run_q == B_TAIL)  tl_start_r    <= 1'b1;
                end

                // ---- the end of the stage ---------------------------------
                // **`run_retired` is the drain and `stage_success` is the
                // verdict, and both are needed here.**  The transactions
                // already in flight have to finish whatever their response
                // says -- deliberately dropping them to suppress a success
                // would turn a reported fault into a corrupt frame -- so the
                // end of the *work* is the drain.  The advance is the verdict.
                if (armed_q && run_retired) begin
                    if (run_is_front) front_done_q <= 1'b1;
                    run_q         <= 4'd0;
                    armed_q       <= 1'b0;
                    region_done_r <= 1'b0;
                    arb_sel_req_r <= 1'b0;
                    if (stage_success) begin
                        sen_q <= sen_q & ~run_q;
                    end
                    else begin
                        // **A stage that retired with a fault does not
                        // advance.**  The whole job stops here: the remaining
                        // mask is dropped so no later stage can be started
                        // over the bank, the store or the cache that this one
                        // left, and `stage_halt_p` takes the failure into the
                        // hold domain where the reset cannot erase it.
                        sen_q        <= 4'd0;
                        stage_halt_p <= 1'b1;
                        job_bad_r    <= 1'b1;
                    end
                end
            end
        end
    end

    // ---- the outputs ------------------------------------------------------
    // `stage` is 0 idle, 1 front, 2 ups.0, 3 ups.1, 4 tail, 7 done.  It is
    // driven from registers and not from an `always @*` over `run_q` alone, so
    // it cannot be read as the *next* stage in the cycle the last one ends.
    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            stage <= 3'd0;
            busy  <= 1'b0;
            done  <= 1'b0;
        end
        else if ((start & ~start_q) && job_idle) begin
            stage <= 3'd0;
            busy  <= 1'b0;
            done  <= 1'b0;
        end
        else if (run_q != 4'd0) begin
            stage <= (run_q == B_FRONT) ? 3'd1 :
                     (run_q == B_UPS0)  ? 3'd2 :
                     (run_q == B_UPS1)  ? 3'd3 : 3'd4;
            busy  <= 1'b1;
        end
        else begin
            // **A job that ended on a fault is not a job that finished.**  The
            // stage does not read 7 and `done` does not rise; `failed` is the
            // output that carries this verdict, and it is driven from the hold
            // domain so the reset that a fatal asks for cannot clear it.
            stage <= (sen_q == 4'd0 && started_q && !job_bad) ? 3'd7 : 3'd0;
            busy  <= 1'b0;
            if (sen_q == 4'd0 && started_q && !job_bad) done <= 1'b1;
        end
    end

    // ======================================================================
    // the fault side, the reset request, and the evidence
    // ======================================================================
    // **THE HOLD DOMAIN: `clk_pe`, and reset by the external `arst_n` ALONE.**
    //
    // This is defect 1, and the defect was a reset domain and not a sequence.
    // The request and its counter used to be reset by `pe_rst_n` -- and
    // `pe_rst_n` is what the request produces, through the real
    // `d64_axi_reset_ctl` above.  That controller's assertion is
    // `req_n = arst_n & ~req`, asynchronous and combinational, so the instant
    // `reset_req` rose the same `req` took `pe_arst_n` down **in the same
    // delta**, `reset_req_r` was cleared by the reset it had just asked for,
    // and the request the controller was asked with was 0 ns wide.  The
    // micro-gate's three numbers are that mechanism exactly:
    //
    //     request_width_ns = 0      rises = 1      sampled_high = 0
    //     retained_counter = 0
    //
    // Every register in this block is therefore cleared by `arst_n` and by
    // nothing else, and the consequences are the two the defect asks for:
    //
    //   * the request is a **genuine full-`clk_pe`-cycle pulse**.  It is
    //     withdrawn by `fault_q` on the next edge -- which is the withdrawal
    //     F3 6.19a requires, because the release runs on `req_n` and a level
    //     held until the sequence finishes deadlocks -- and it is not cleared
    //     by the sequence it started;
    //   * the fault's **identity** (`reset_reason`), the number of resets
    //     (`reset_pulses`), the job's failure record (`job_failed_r`,
    //     `job_fails`) and the admission rejections survive the reset.
    //
    // The PE datapath is still reset by `pe_rst_n`, and that is correct: what
    // it may not do is clear **its own request** or **its own fault record**.
    // Note that the fault *sources* (`c_cache_fatal`, `c_rd_fatal`) are in the
    // `pe_rst_n` domain and are cleared by the reset -- which is precisely why
    // the identity is latched here rather than re-read from the source after
    // the sequence.  Re-reading it is how a fault becomes unobservable.
    wire [3:0] fault_id = {2'b00, c_rd_fatal, c_cache_fatal};
    wire       reset_asked = (reset_reason_r != 4'd0);

    always @(posedge clk_pe or negedge arst_n) begin
        if (!arst_n) begin
            fault_q         <= 1'b0;
            reset_req_r     <= 1'b0;
            reset_pulses_r  <= 32'd0;
            reset_reason_r  <= 4'd0;
            job_failed_r    <= 1'b0;
            mask_bad_r      <= 1'b0;
            admit_rejects_r <= 32'd0;
            job_fails_r     <= 32'd0;
            started_q       <= 1'b0;
        end
        else begin
            // ---- the request: one full `clk_pe` cycle, on the rising edge --
            // `fault_q` is what withdraws it, and `fault_q` lives here, so the
            // pulse cannot be shortened by the reset it asks for.
            reset_req_r <= any_fatal & ~fault_q;
            fault_q     <= any_fatal;
            if (any_fatal & ~fault_q) begin
                reset_pulses_r <= reset_pulses_r + 32'd1;
                reset_reason_r <= reset_reason_r | fault_id;
            end

            // ---- the job's failure record ----------------------------------
            // A level out of the sequencer's own verdict, latched here so that
            // the reset a fatal asks for does not erase why the job stopped.
            if (stage_halt_p) begin
                job_failed_r <= 1'b1;
                job_fails_r  <= job_fails_r + 32'd1;
            end
            if (mask_bad_p)     mask_bad_r      <= 1'b1;
            if (admit_reject_p) admit_rejects_r <= admit_rejects_r + 32'd1;

            // `started_q` is this job's bookkeeping and it survives the reset
            // with the rest of it: a reset that cleared it would let the
            // sequencer report an idle job as a finished one.
            if ((start & ~start_q) && job_idle) started_q <= 1'b1;
        end
    end

    assign reset_req     = reset_req_r;
    assign reset_pulses  = reset_pulses_r;
    assign reset_reason  = reset_reason_r;
    assign admit_rejects = admit_rejects_r;
    assign job_fails     = job_fails_r;

    // `failed` is the JOB's verdict, and every term of it is either sticky in
    // this block or gated on the job being over.  **The gate is not
    // decoration.**  A transport verdict read while the writer still holds a
    // beat is not a verdict -- `d64_ups_u16_writer` raises `bresp_errors` in
    // the middle of a frame, many beats before the drain -- and asserting
    // `failed` on it would halt a caller on a number that is still moving.  The
    // two fault sources and the request's own sound are not verdicts; those
    // assert immediately.
    assign failed = any_fatal | reset_asked | job_failed_r | mask_bad_r
                  | (job_over & (transport_fault | frame_fault));

    assign frame_cnt = f_frame_cnt;

    // R6: the admission rule, published.  `job_idle` is declared above.
    assign can_accept = job_idle;

    // The bridge's two witnesses, brought out as numbers rather than as
    // decoration: `bias_sign_bad` must stay zero and `acc_extended` must move.
    // They are not ports of this module (the root does not read them) and are
    // not tied off either -- the arms read them hierarchically, as the joint
    // gate does, which is what stops `vopt` folding the bridge away.

    // ======================================================================
    // the display, fused -- batch H, FUSE_DISP
    // ======================================================================
    // **The same wiring `board/integrate/d64_cam_hdmi_top.v` uses**, and the
    // same modules: `d64_video_timing` -> `d64_video_row_ddr` (the reader, with
    // its own AXI4 read master) -> `d64_rgbx_display_adapter` (**the one and
    // only red/blue swap in the RTL**, required by
    // `tools/run_rgb_display_chain.sh` to be in exactly one file) ->
    // `d64_video_scaler` -> `dvi_encoder`.
    //
    // **`d64_rgbx_beat_writer` and not `board/preview/rtl/rgbx_frame_writer.v`.**
    // The bytes on the screen must be the bytes the *tail engine* wrote to
    // `DISP_SRC_BASE`; `rgbx_frame_writer` is the display-only bring-up's own
    // writer and using it would put a second writer on the same region.  See
    // `docs/BATCH_H_ROOT_FUSION_2026-10-08.md` 3a.5.
    wire        fused_read_n;
    d64_reset_sync u_disp_reset (
        .clk(px_clk), .arst_n(arst_n), .rst_n(fused_read_n)
    );

    wire        t_hs, t_vs, t_de, t_frame;
    wire [11:0] t_x, t_y, t_ax, t_ay;
    wire        load_req, row_ready;
    wire [7:0]  load_row, rd_wi, ld_wi;
    wire        rd_we, ld_we;
    wire [23:0] rd_wd, ld_wd;
    wire [31:0] rows_loaded, stale_beats, underrun, axi_errors_v, bad_rows;
    wire [7:0]  first_bad_row;

    reg         seen_done, disp_on;
    always @(posedge px_clk or negedge fused_read_n) begin
        if (!fused_read_n) begin
            seen_done <= 1'b0;
            disp_on   <= 1'b0;
        end
        else begin
            // **`stage_success` and not `done`**: a drained transport is not a
            // successful one, and the first frame shown should be a whole one.
            if (stage_success) seen_done <= 1'b1;
            if (seen_done && t_frame) disp_on <= 1'b1;
        end
    end

    wire [9:0] fused_tmds0, fused_tmds1, fused_tmds2, fused_tmds_clk;

    generate if (FUSE_DISP == 1) begin : g_disp
        d64_video_timing #(
            .H_ACT(H_ACT), .H_FP(H_FP), .H_SYNC(H_SYNC), .H_BP(H_BP),
            .V_ACT(V_ACT), .V_FP(V_FP), .V_SYNC(V_SYNC), .V_BP(V_BP),
            .XW(12), .YW(12)
        ) u_timing (
            .clk(px_clk), .rst_n(fused_read_n),
            .hs(t_hs), .vs(t_vs), .de(t_de), .frame(t_frame),
            .x(t_x), .y(t_y), .ax(t_ax), .ay(t_ay)
        );

        d64_video_row_ddr #(
        .FAULT_STICKY(FAULT_STICKY),
            .SRC_W(IMG_W), .PX_BYTES(4), .AW(6), .TIMEOUT(1024),
            .SRC_BASE(DISP_SRC_BASE)
        ) u_reader (
            .clk_pe(px_clk), .rst_pe_n(fused_read_n & disp_on),
            .load_req(load_req), .load_row(load_row), .row_ready(row_ready),
            .load_we(rd_we), .load_wi(rd_wi), .load_wd(rd_wd),
            .frame_id(frame_cnt[1:0]), .src_base(DISP_SRC_BASE),
            .frame_tick(t_frame),
            .clk_axi(clk_axi), .arst_axi_n(axi_arst_n & ~reset_req),
            .axi_arvalid(disp_axi_arvalid), .axi_arready(disp_axi_arready),
            .axi_araddr(disp_axi_araddr), .axi_arlen(disp_axi_arlen),
            .axi_arsize(disp_axi_arsize), .axi_arburst(disp_axi_arburst),
            .axi_rvalid(disp_axi_rvalid), .axi_rready(disp_axi_rready),
            .axi_rdata(disp_axi_rdata), .axi_rlast(disp_axi_rlast),
            .axi_rresp(disp_axi_rresp),
            .rows_loaded(rows_loaded), .stale_beats(stale_beats),
            .underrun(underrun), .axi_errors(axi_errors_v),
            .bad_rows(bad_rows), .first_bad_row(first_bad_row)
        );

        d64_rgbx_display_adapter u_adapt (
            .load_we_i(rd_we), .load_wi_i(rd_wi), .load_wd_i(rd_wd),
            .load_we_o(ld_we), .load_wi_o(ld_wi), .load_wd_o(ld_wd)
        );

        wire        s_hs, s_vs, s_de, starved;
        wire [23:0] s_rgb;

        d64_video_scaler #(
            .SRC_W(IMG_W), .SRC_H(IMG_H), .WIN_W(H_ACT), .WIN_H(V_ACT),
            .X0(0), .Y0(0)
        ) u_sc (
            .clk(px_clk), .rst_n(fused_read_n),
            .hs_i(t_hs), .vs_i(t_vs), .de_i(t_de), .frame_i(t_frame),
            .ax_i(t_ax), .ay_i(t_ay),
            .load_req(load_req), .load_row(load_row), .row_ready(row_ready),
            .load_we(ld_we), .load_wi(ld_wi), .load_wd(ld_wd),
            .hs_o(s_hs), .vs_o(s_vs), .de_o(s_de), .rgb_o(s_rgb),
            .starved(starved)
        );

        // The monitor's own wiring, copied from `board/preview/preview_top.v`
        // and checked against it by `tools/run_rgb_display_chain.sh`.
        dvi_encoder u_dvi (
            .pixelclk(px_clk),
            .rstin(~fused_read_n),
            .blue_din(s_rgb[7:0]), .green_din(s_rgb[15:8]), .red_din(s_rgb[23:16]),
            .hsync(s_hs), .vsync(s_vs), .de(s_de),
            .tmds_data0(fused_tmds0), .tmds_data1(fused_tmds1),
            .tmds_data2(fused_tmds2), .tmds_clk(fused_tmds_clk)
        );
    end
    endgenerate

    // Inverted, as `board/hdmi_tx` does it.
    assign tmds_data0_o = ~fused_tmds0;
    assign tmds_data1_o = ~fused_tmds1;
    assign tmds_data2_o = ~fused_tmds2;
    assign tmds_clk_o   = ~fused_tmds_clk;
    assign tmds_data0_TX_OE  = 1'b1;
    assign tmds_data1_TX_OE  = 1'b1;
    assign tmds_data2_TX_OE  = 1'b1;
    assign tmds_clk_TX_OE    = 1'b1;
    assign tmds_data0_TX_RST = 1'b0;
    assign tmds_data1_TX_RST = 1'b0;
    assign tmds_data2_TX_RST = 1'b0;
    assign tmds_clk_TX_RST   = 1'b0;



    // synthesis translate_off
    wire [1407:0] front_act_registered_ghost = u_front.sel_enc ?
      (u_front.u_enc.head_owner ? u_front.u_enc.u_head_client.act_registered_ghost :
       (u_front.u_enc.down_owner ? u_front.u_enc.u_down_client.act_registered_ghost:1408'd0)):
      u_front.u_trk.u_blk.u_eng.act_registered_ghost;
    wire [1407:0] bank_act_registered_ghost = u_arb.sel_q==0 ? front_act_registered_ghost :
      u_arb.sel_q==1 ? u_u0.act_registered_ghost :
      u_arb.sel_q==2 ? u_u1.act_registered_ghost : u_tail.u_eng.act_registered_ghost;
    integer act_retime_valid_checks=0;
    always @(negedge clk_pe)if(pe_rst_n&&arb_bank_slot_valid)begin
      act_retime_valid_checks=act_retime_valid_checks+1;
      if(u_bank.act_registered_shadow!==bank_act_registered_ghost)
        $fatal(1,"AREG_VALID_SLOT_GHOST_DIFF owner=%0d frontowner=%0d count=%0d",u_arb.sel_q,f_owner,act_retime_valid_checks);
    end
    // synthesis translate_on

    // FSB4 passive APP-domain diagnostic; no control feedback or added state.
    assign front_debug = {f_out_committed,head_phase_ready,f_busy,front_start_r,
        trunk_store_committed,down0_store_commit_ready,down0_store_wr_ready,down0_store_wr_valid,
        head_store_commit_ready,head_store_wr_ready,head_store_wr_valid,d64_ready,(fsb6_t_wf_ovf != 32'd0),
        f_mac_mode_safe,f_done,sen_q,run_q,arb_sel_ack,f_enc_state,f_owner,armed_q};
endmodule
