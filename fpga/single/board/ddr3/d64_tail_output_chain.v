`timescale 1ns/1ps

// ============================================================================
// d64_tail_output_chain -- the tail, its pixels, and the memory's answer.
// ============================================================================
//
// F3 `docs/F3_INTERFACES_2026-10-07.md` section 3.15.  Three modules are built,
// gated and committed and **nothing joined them**:
//
//   `control/d64_tail_engine.v`          one 32-bit `0x00RRGGBB` word a pixel
//   `board/ddr3/d64_rgbx_beat_writer.v`  four pixels into one 128-bit beat
//   `board/ddr3/d64_done_tracker.v`      what DONE actually waits for
//   `control/d64_net_addr.v`             the TAIL region, and the only place
//                                        its base is defined
//
// This module is the join, and it is a **top module** rather than a chain drawn
// inside a testbench.  The reason is that the seams it fixes are the seams a
// network top will need: the store ports the engine reads, the section 1 PE
// seam, the AXI4 write master, and a DONE that means "the memory has answered".
// A chain that existed only in a testbench would have to be redrawn by whoever
// writes that top -- and the redrawing is where a fixed seam gets renegotiated.
//
// The three joins, and what each one is
// -------------------------------------
//
// 1.  **The tail into the writer, by name.**  The writer's input ports are
//     spelled with the engine's own names (`out_wr_en`, `out_wr_addr`,
//     `out_wr_data`) precisely so that this join is a name that resolves or
//     does not.  There is no repacking here and no straddle: four 32-bit pixels
//     are 128 bits and the writer's header says so at length.
//
// 2.  **The writer's response into the tracker.**  The writer's `wr_ack` is one
//     cycle wide, one per committed write response; the tracker counts rising
//     edges.  They are wired together and **nothing in this module turns one
//     response into more than one edge** -- the tracker's header makes that the
//     parent's job, and this is the parent.
//
// 3.  **The engine's `done` into the tracker's `last_valid`, and only there.**
//     The chain's `done` is the **tracker's** DONE.  The engine's `done` is
//     exposed as `tail_done` and fed to `last_valid`, which the tracker never
//     acts on.  That is the conflation the tracker exists to prevent, and this
//     is exactly the module where it would have been easy to commit: the
//     engine's `done` is right there, is already called `done`, and arrives
//     first.
//
// The expectation, and the number that is wrong twice
// --------------------------------------------------
//
// `exp_count` is **derived here and not typed**: the region's byte size, out of
// an instantiated `d64_net_addr`, divided by the bytes one committed write
// response carries.
//
//     exp_count = sel_size / (16 bytes a beat * BURST_BEATS beats a response)
//
// The divisor is a power of two in both configurations this gate builds, so it
// is a shift and not a divider -- this project's divisions have come back
// 1.25x, 2.11x and 2.39x above their floor, and none of them is in this path.
//
// **The contract's number, and the arithmetic that produces it.**  Section 3.15
// fixes `exp_count` at **16384 per TAIL frame, not 65536**: 16384 is the
// region's count of 128-bit units (262144 / 16) and 65536 is its count of
// *pixels*.  A tracker armed at 65536 and fed from `wr_ack` never completes --
// that much is arithmetic and settled.
//
// What that sentence does not settle on its own is how many *responses* the
// writer emits, and the two are the same number only if one response carries
// one beat.  **`wr_ack` is one pulse per AXI write response, and an AXI write
// response is one per burst, not one per beat.**  With the writer's own default
// `BURST_BEATS = 64`, a TAIL frame is **256** responses carrying 16384 beats --
// which is what the writer's own gate measures and prints ("65536 pixels in,
// 16384 beats out, 256 bursts, 256 acks").  A tracker armed at 16384 and fed
// from `wr_ack` would then never complete either, by a different factor.
//
// So the arithmetic closes in exactly one configuration, and this module states
// it as a parameter rather than assuming it:
//
//   `BURST_BEATS = 1`   one 128-bit beat a burst, **one response a beat**
//                       -> 16384 responses a frame.  The default, and the
//                       configuration section 3.15 names.
//   `BURST_BEATS = 64`  the writer's own default, one burst a row
//                       -> 256 responses a frame, and `exp_count` is 256.
//
// Both are built and run by `tools/run_tail_output_chain.sh`, both carry all
// 262144 bytes to the memory, and both raise DONE exactly once at their own
// count.  The parameter is not a tuning knob: it is the one number the two
// documents disagree about, so it is on the port list where it can be seen.
// **This is the seam that does not reconcile, and it is stated rather than
// smoothed over.**
//
// The region, and why there are two `d64_net_addr` instances in a run
// ------------------------------------------------------------------
//
// `TAIL_BASE` is a parameter here because a parameter port cannot be a wire in
// Verilog-2001.  So the chain instantiates its own `d64_net_addr` with this
// value, the writer takes its base from the same parameter, and the gate
// instantiates a **second, default-configured** `d64_net_addr` and requires the
// chain's `map_sel_base` to equal it.  `d64_net_addr.v` is the only file where
// `0x005C_0000` is written; either side moving alone fails the run rather than
// moving the region.
//
// What is NOT here
// ----------------
//
// No input store and no weight store: the engine's read ports are on this
// module's port list, because in the network they are the shared cache and the
// shared weight store and neither belongs to this chain.  No PE bank either --
// the section 1 seam is passed through, so the bank stays the network's single
// instance.  No DDR controller: the AXI write master is a port.
//
// No synthesis, no board, no frame rate.  See
// `docs/TAIL_OUTPUT_CHAIN_2026-10-07.md`.
// ============================================================================
module d64_tail_output_chain #(
    // ---- the region -------------------------------------------------------
    // The map owns this number.  See the header for why it is a parameter and
    // what keeps it from drifting.
    // **The frame's shape, forwarded and not assumed.**  `exp_count` is derived
    // from the map's TAIL region, so a chain that takes `TAIL_BASE` and not the
    // shape derives the SQUARE frame's expectation -- 256 responses -- over a
    // rectangle job's 144 bursts.  And `u_eng` reads rows 0..H-1: at its default
    // 256 it would address 112 rows of a 144-row tensor that are not there.  Both
    // are the same defect the engine's own reflection had, one level up.
    parameter integer H           = 256,
    parameter integer W           = 256,
    parameter [31:0]  TAIL_BASE   = 32'h005C_0000,

    // ---- the bus's geometry, and therefore the tracker's expectation ------
    // Beats a write response.  1 is section 3.15's configuration; 64 is the
    // writer's own default.  See the header.
    parameter integer BURST_BEATS = 1,
    parameter integer RESERVE_WRITE_BURST = 0,
    // Beats buffered between the packer and the bus.
    parameter integer BQ_DEPTH    = 4,
    // Bytes a 128-bit beat.  Not a knob: 128 bits.
    parameter integer BEAT_B      = 16,

    // ---- the tracker's stall report ---------------------------------------
    parameter integer IDLE_TIMEOUT = 32768,

    // ---- the tail's weights, in the decoder's shared image ----------------
    // F3 section 2: the tail's nine tiles are **238..246** of the 247-tile
    // image (`W_TILES = 247`), mode `MAC_P4_I8_O4`, and the engine reads
    // `w_rd_addr = W_BASE + tap` for tap 0..8.  This default is the real base,
    // not the convenience of a small test array; the gate relocates the tail's
    // nine tile words into a 247-entry store at 238..246 rather than moving the
    // base down to the store.
    parameter integer W_BASE      = 238,
    parameter integer W_ADDR_W    = 12,

    // ---- verification only, both folded away at 0 -------------------------
    // One acknowledgement withheld from the tracker.  The frame's first
    // `wr_ack` pulse never reaches it, so the count stops one short and DONE
    // does not rise.  The tooth for "DONE rises after the acknowledgement and
    // not before", built inside the joined chain rather than on a testbench's
    // stub of it.
    parameter integer SAB_HOLD_ONE_ACK = 0,
    // Byte lanes 0 and 1 of the assembled beat exchange places, in the writer.
    // Every address, beat count and burst stays right; only the byte position
    // moves.
    parameter integer SAB_SWAP_BYTES   = 0
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,

    // ---- the engine's own verdict, and the chain's ------------------------
    output wire                       busy,        // the engine, still working
    // **`done` is the PUBLISHABLE claim, not the arrival claim.**  Three things
    // that were one: "the engine stopped" (`tail_done`), "every response
    // arrived" (`responses_done`), and "the frame may be displayed" (`done`).
    // A faulted frame raises `done` **and leaves `done` low** -- see
    // `docs/AXI_ERROR_DRAIN_2026-10-07.md`.  A consumer that gates on
    // `responses_done` instead can still publish a frame whose memory answered
    // with an error, which is the bug this separation exists to make impossible.
    output wire                       done,        // publishable: count met AND no fault
    output wire                       responses_done,  // the arrival claim, for observation
    output wire [1:0]                 done_reason,
    output wire                       tail_done,   // the engine's: fed to last_valid
    output wire [31:0]                tail_cycles,

    // ---- the layer's quantisation -----------------------------------------
    input  wire [4*16-1:0]            rq_m,        // OC_LANES * M_W
    input  wire [4*5-1:0]             rq_shift,    // OC_LANES * SHIFT_W
    input  wire [4*32-1:0]            rq_bias,     // OC_LANES * ACC_W

    // ---- the input store, external, registered read -----------------------
    output wire [5:0]                 in_rd_en,    // IN_PORTS = 6
    output wire [6*16-1:0]            in_rd_addr,
    input  wire [6*8*8-1:0]           in_rd_data,
    // **The input-side accept, and it is a port and not a tie.**  F3 6.9
    // amended `d64_tail_engine` to take `in_stall` -- "a read is TRANSFERRED in
    // the cycle `in_stall` is low, and the word for that address appears on
    // `in_rd_data` in the following cycle" -- and the engine's three store
    // ports came out here while the fourth did not.  So the child's `in_stall`
    // read `z` in every build of this chain: an input left unconnected is `z`,
    // `z` reads as "never stall" in the clocked `if`s that consume it, and the
    // engine's input-accept path was therefore **unreachable through this
    // module**.  That is correct for the RAM store this chain has always been
    // built against, which never stalls, and wrong the moment the chain is
    // pointed at the real `d64_burst_prefetch`, which can miss.
    //
    // `docs/TAIL_INPUT_STALL_2026-10-07.md` is the engine's own gate: the real
    // cache, six ports, every output word compared, and a tooth that ignores
    // `in_stall` failing **on data**.  This port is what makes that gate's
    // subject reachable from the network root; `tools/run_iface_tail_stall.sh`
    // is the chain-level arm, and it drives this port rather than leaving it
    // unconnected.
    input  wire                       in_stall,

    // ---- the weight store, external, registered read ----------------------
    output wire                       w_rd_en,
    output wire [11:0]                w_rd_addr,
    input  wire [8*4*8-1:0]           w_rd_data,   // IC_PAR*OC_LANES*WGT_W

    // ---- the PE seam, section 1, passed through ---------------------------
    output wire                       pe_req_slot_valid,
    output wire [128*11-1:0]          pe_req_act,
    output wire [128*8-1:0]           pe_req_wgt,
    output wire                       pe_req_pix_start,
    output wire [128*32-1:0]          pe_req_bias,
    output wire [1:0]                 pe_req_mode,
    output wire                       pe_req_rq_start,
    output wire [16*16-1:0]           pe_req_rq_m,
    output wire [16*5-1:0]            pe_req_rq_shift,
    output wire [9:0]                 pe_req_rq_zero,
    output wire [9:0]                 pe_req_rq_qmin,
    output wire [9:0]                 pe_req_rq_qmax,
    input  wire                       pe_rsp_rq_ready,
    input  wire                       pe_rsp_rq_ready_n,
    input  wire [8*10-1:0]            pe_rsp_y,
    input  wire [7:0]                 pe_rsp_y_valid,
    input  wire                       pe_rsp_rq_busy,
    input  wire                       pe_rsp_rq_done,
    input  wire                       pe_rsp_rq_overrun,
    input  wire [31:0]                pe_rsp_mode_viol,
    input  wire [128*32-1:0]          pe_rsp_acc,
    input  wire [128*32-1:0]          pe_rsp_snapshot,
    output wire                       mac_mode_safe,

    // ---- raw AXI4 write master, 128-bit beats -----------------------------
    output wire [31:0]                axi_awaddr,
    output wire [7:0]                 axi_awlen,
    output wire [2:0]                 axi_awsize,
    output wire [1:0]                 axi_awburst,
    output wire                       axi_awvalid,
    input  wire                       axi_awready,
    output wire                       axi_wvalid,
    input  wire                       axi_wready,
    output wire [127:0]               axi_wdata,
    output wire [15:0]                axi_wstrb,
    output wire                       axi_wlast,
    input  wire                       axi_bvalid,
    output wire                       axi_bready,
    input  wire [1:0]                 axi_bresp,

    // ---- evidence ---------------------------------------------------------
    output wire [31:0]                exp_count,      // derived, see the header
    output wire [31:0]                map_sel_base,   // the chain's own map
    output wire [31:0]                map_sel_size,
    output wire [31:0]                map_tail_word,  // the map's TAIL word address
    output wire [31:0]                tail_pixels,     // the engine's own count
    output wire [31:0]                tail_rq_requests,
    output wire [31:0]                tail_rq_waits,
    output wire [31:0]                tail_rq_overruns,
    output wire [31:0]                tail_mfifo_peak,
    // The engine's own two input-side counters, brought out here for the same
    // reason the five above are: a stall that is not a number is a stall
    // nobody can see (`docs/TAIL_INPUT_STALL_2026-10-07.md`, rule 4).  Neither
    // is a fault -- a cache that never stalled is a cache with no DDR behind
    // it -- and both are zero in a frame against a store that never refuses.
    output wire [31:0]                in_stall_cycles,
    output wire [31:0]                in_stall_events,
    output wire [31:0]                wr_pixels,
    output wire [31:0]                wr_beats,
    output wire [31:0]                wr_bursts,
    output wire [31:0]                wr_responses,
    output wire [31:0]                wr_bresp_errors,
    output wire [31:0]                wr_oob,
    output wire [31:0]                wr_disorder,
    output wire [31:0]                wr_misgrouped,
    output wire [31:0]                wr_overflows,
    output wire [31:0]                wr_stall_cycles,
    output wire [31:0]                trk_acks,
    output wire [31:0]                trk_outstanding,
    output wire [31:0]                trk_extra_acks,
    output wire [31:0]                trk_last_valids,
    output wire [31:0]                trk_stall_cycles,
    output wire                       trk_stall_timeout,

    // ---- the credit, the commit and the drain, named ---------------------
    // **`drained` is the writer's own**, brought out because `done` is no longer
    // the same claim as "every response arrived" and a consumer that wants the
    // arrival claim alone must be able to read it: `responses_done` is the
    // tracker's count and this is the writer having nothing left to write.
    output wire                       drained,
    output wire [31:0]                credit_beats,
    output wire [31:0]                credit_free,
    // **The page rule, as a port.**  A TAIL region whose base is not 4 KiB
    // aligned cannot be served by a writer with no end-of-transfer input, so the
    // writer refuses and counts rather than crossing a page.  Both numbers are
    // brought out so that a caller which moves the base finds out from a port.
    output wire                       base_aligned,
    output wire [31:0]                page_refusals,
    output wire [31:0]                burst_clamps,
    output wire [15:0]                burst_beats,

    output reg  [31:0]                sab_acks_withheld
);

    // -----------------------------------------------------------------------
    // Shape.  None of these is on the port list, because a port list cannot
    // see a `localparam` -- the trap section 0 of the F3 contract names -- so
    // nothing on any interface depends on one.
    // -----------------------------------------------------------------------

    localparam [1:0]  T_TAIL = 2'd3;                  // `d64_net_addr`'s own

    // **The burst that can actually be asked for.**  An AXI INCR burst may not
    // cross a 4 KiB boundary, so a page holds `4096 / 16 = 256` beats and a
    // request for more is clamped to that -- here, *once*, and the clamped value
    // is what the writer is built with, so the writer's own clamp is a second
    // line of defence and not a second opinion.
    //
    // **The response count is a property of the page and not of the requested
    // burst**, which is the correction this line carries: `exp_count` used to be
    // `sel_size / (16 * BURST_BEATS)`, so a caller who asked for 512-beat bursts
    // would have armed the tracker at 32 responses over a frame the writer
    // answers **64** times -- a tracker that raises DONE at the 32nd of 64, with
    // half the frame still in flight.  `drained` is what makes that impossible
    // now; this is what makes the count itself right.
    localparam integer BURST_PAGE = 4096 / BEAT_B;                      // 256
    localparam integer BURST_W    = (BURST_BEATS > BURST_PAGE)
                                  ? BURST_PAGE : ((BURST_BEATS < 1) ? 1 : BURST_BEATS);
    localparam [31:0] BEAT_BYTES = BEAT_B * BURST_W;

    // -----------------------------------------------------------------------
    // The map, instantiated, so that the region is asked for and not retyped.
    // -----------------------------------------------------------------------

    wire [31:0] map_addr, map_row_base;
    wire [31:0] map_s_c, map_s_w, map_s_h, map_s_n;
    wire [15:0] map_n, map_c, map_h, map_w;
    wire [31:0] map_row_b, map_rows, map_bursts, map_row_bursts, map_cache_b;
    wire        map_row_whole, map_in_region;
    wire [17:0] map_word;
    wire [31:0] map_burst_b, map_cache_rows;
    wire [31:0] map_lo, map_hi, map_span, map_used;

    // Every port connected.  A `vopt` warning per omitted port is a line a real
    // warning has to be read through, and the numbers are worth having: `sel_w`
    // is the region's row in words and `sel_row_b` is it in bytes, and both are
    // checked against the gate's own byte count.
    d64_net_addr #(.TAIL_BASE(TAIL_BASE), .TAIL_H(H), .TAIL_W(W)) u_map (
        .tsel(T_TAIL),
        .t_n(16'd0), .t_c(16'd0), .t_h(16'd0), .t_w(16'd0),
        .addr(map_addr), .row_base(map_row_base),
        .sel_base(map_sel_base), .sel_size(map_sel_size),
        .sel_s_c(map_s_c), .sel_s_w(map_s_w),
        .sel_s_h(map_s_h), .sel_s_n(map_s_n),
        .sel_n(map_n), .sel_c(map_c), .sel_h(map_h), .sel_w(map_w),
        .sel_row_b(map_row_b), .sel_rows(map_rows),
        .sel_bursts(map_bursts), .sel_row_bursts(map_row_bursts),
        .sel_row_whole(map_row_whole), .sel_cache_b(map_cache_b),
        .sel_in_region(map_in_region),
        .tail_word_addr(map_word),
        .burst_b(map_burst_b), .cache_rows(map_cache_rows),
        .map_lo(map_lo), .map_hi(map_hi),
        .map_span(map_span), .map_used(map_used)
    );

    assign map_tail_word = {14'b0, map_word};

    // -----------------------------------------------------------------------
    // The expectation.  **Derived, not typed.**  262144 B / (16 B * 1 beat) =
    // 16384, and the same expression with `BURST_BEATS = 64` is 256.  See the
    // header for why the number is a function of the geometry.
    // -----------------------------------------------------------------------
    assign exp_count = map_sel_size / BEAT_BYTES;

    // -----------------------------------------------------------------------
    // The engine, with its store ports brought out and its output wired
    // straight into the writer.
    // -----------------------------------------------------------------------

    wire        eng_busy;
    wire        eng_done;
    wire [31:0] eng_cycles;
    wire [31:0] eng_pixels;
    wire [31:0] eng_rq_requests, eng_rq_accepts, eng_rq_waits, eng_rq_overruns;
    wire [31:0] eng_slots, eng_mfifo_peak;
    wire [31:0] eng_in_stall_cycles, eng_in_stall_events;
    wire        eng_out_wr_en;
    wire [17:0] eng_out_wr_addr;
    wire [31:0] eng_out_wr_data;

    d64_tail_engine #(
        .H(H), .W(W),
        .W_BASE(W_BASE),
        .W_ADDR_W(W_ADDR_W)
    ) u_eng (
        .clk(clk), .rst_n(rst_n), .start(start),
        .busy(eng_busy), .done(eng_done), .cycles(eng_cycles),
        .rq_m(rq_m), .rq_shift(rq_shift), .rq_bias(rq_bias),
        .in_rd_en(in_rd_en), .in_rd_addr(in_rd_addr), .in_rd_data(in_rd_data),
        .in_stall(in_stall),
        .w_rd_en(w_rd_en), .w_rd_addr(w_rd_addr), .w_rd_data(w_rd_data),
        .out_wr_en(eng_out_wr_en), .out_wr_addr(eng_out_wr_addr),
        .out_wr_data(eng_out_wr_data),
        .rq_requests(eng_rq_requests), .rq_accepts(eng_rq_accepts),
        .rq_wait_cycles(eng_rq_waits), .rq_overrun_count(eng_rq_overruns),
        .slot_count(eng_slots), .out_write_count(eng_pixels),
        .mfifo_peak(eng_mfifo_peak),
        .in_stall_cycles(eng_in_stall_cycles),
        .in_stall_events(eng_in_stall_events),
        .pe_req_slot_valid(pe_req_slot_valid), .pe_req_act(pe_req_act),
        .pe_req_wgt(pe_req_wgt), .pe_req_pix_start(pe_req_pix_start),
        .pe_req_bias(pe_req_bias), .pe_req_mode(pe_req_mode),
        .pe_req_rq_start(pe_req_rq_start), .pe_req_rq_m(pe_req_rq_m),
        .pe_req_rq_shift(pe_req_rq_shift), .pe_req_rq_zero(pe_req_rq_zero),
        .pe_req_rq_qmin(pe_req_rq_qmin), .pe_req_rq_qmax(pe_req_rq_qmax),
        .pe_rsp_rq_ready(pe_rsp_rq_ready),
        .pe_rsp_rq_ready_n(pe_rsp_rq_ready_n),
        .pe_rsp_y(pe_rsp_y), .pe_rsp_y_valid(pe_rsp_y_valid),
        .pe_rsp_rq_busy(pe_rsp_rq_busy), .pe_rsp_rq_done(pe_rsp_rq_done),
        .pe_rsp_rq_overrun(pe_rsp_rq_overrun),
        .pe_rsp_mode_viol(pe_rsp_mode_viol),
        .pe_rsp_acc(pe_rsp_acc), .pe_rsp_snapshot(pe_rsp_snapshot),
        .mac_mode_safe(mac_mode_safe)
    );

    assign busy        = eng_busy;
    assign tail_done   = eng_done;
    assign tail_cycles = eng_cycles;
    assign tail_pixels     = eng_pixels;
    assign tail_rq_requests = eng_rq_requests;
    assign tail_rq_waits    = eng_rq_waits;
    assign tail_rq_overruns = eng_rq_overruns;
    assign tail_mfifo_peak  = eng_mfifo_peak;
    assign in_stall_cycles  = eng_in_stall_cycles;
    assign in_stall_events  = eng_in_stall_events;

    // -----------------------------------------------------------------------
    // The writer.  Four pixels a beat, and the base it bursts at is the same
    // parameter the chain's map instance was configured with -- so the two
    // cannot disagree inside this module.
    // -----------------------------------------------------------------------

    wire [31:0] wr_pixels_in;
    wire        wr_ack_raw;
    // The writer's own verdict on the frame, which the tracker consumes:
    // `wr_ack_raw` is "the memory answered", `wr_ok_raw` is "and the answer was
    // not an error", and `wr_frame_fault` is the sticky verdict.
    wire        wr_ok_raw;
    wire [31:0] wr_oks;
    wire        wr_frame_fault;
    // The writer's three claims, brought out of the instance: the credit it will
    // build, the drain it will report, and the page rule it is enforcing.
    wire        wr_drained;
    wire [31:0] wr_credit_beats, wr_credit_free, wr_page_refusals, wr_burst_clamps;
    wire [15:0] wr_burst_beats;
    wire        wr_base_aligned;
    // The tracker's own verdict, declared here and not beside the instance: a
    // name used above its declaration makes an implicit net and the explicit one
    // is then a redeclaration -- `vlog` says so, and the reason is worth keeping.
    wire        trk_publishable;

    d64_rgbx_beat_writer #(
        .TAIL_BASE(TAIL_BASE),
        .PIX_W(18),
        .PIX_PER_BEAT(4),
        .BURST_BEATS(BURST_W),
        .RESERVE_WRITE_BURST(RESERVE_WRITE_BURST),
        .BQ_DEPTH(BQ_DEPTH),
        .SWAP_BYTES_SABOTAGE(SAB_SWAP_BYTES),
        .PACK3_SABOTAGE(0)
    ) u_wr (
        .clk(clk), .rst_n(rst_n),
        .out_wr_en(eng_out_wr_en),
        .out_wr_addr(eng_out_wr_addr),
        .out_wr_data(eng_out_wr_data),
        .axi_awaddr(axi_awaddr), .axi_awlen(axi_awlen),
        .axi_awsize(axi_awsize), .axi_awburst(axi_awburst),
        .axi_awvalid(axi_awvalid), .axi_awready(axi_awready),
        .axi_wvalid(axi_wvalid), .axi_wready(axi_wready),
        .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb), .axi_wlast(axi_wlast),
        .axi_bvalid(axi_bvalid), .axi_bready(axi_bready),
        .axi_bresp(axi_bresp),
        .pixels_in(wr_pixels_in), .beats_out(wr_beats), .bursts_out(wr_bursts),
        .wr_ack(wr_ack_raw), .wr_acks(wr_responses),
        .bresp_errors(wr_bresp_errors),
        .oob_pixels(wr_oob), .disorder_pixels(wr_disorder),
        .misgrouped_pixels(wr_misgrouped),
        .queue_overflows(wr_overflows), .stall_cycles(wr_stall_cycles),
        .wr_ok(wr_ok_raw), .wr_oks(wr_oks), .frame_fault(wr_frame_fault),
        .credit_beats(wr_credit_beats), .credit_free(wr_credit_free),
        .drained(wr_drained),
        .page_refusals(wr_page_refusals), .burst_clamps(wr_burst_clamps),
        .base_aligned(wr_base_aligned), .burst_beats(wr_burst_beats)
    );

    assign wr_pixels = wr_pixels_in;

    // -----------------------------------------------------------------------
    // **`done` is the DRAINED claim.**  Three states that used to be one:
    // "the engine stopped" (`tail_done`), "every response arrived"
    // (`responses_done`), and "the frame may be published" -- which is this.
    //
    // It is the tracker's publishable (the count met with no fault) **and the
    // writer's `drained`**: the queue empty and no burst open.  A response
    // retires a burst and says nothing about what is behind it, so a frame whose
    // responses have all arrived can still have a beat the writer accepted
    // waiting to go -- and at `BURST_BEATS` above a page the count itself was
    // wrong, which is what `BURST_W` above corrects.  `drained` also cannot rise
    // for a frame the writer refused (`page_refusals`), so a base it cannot
    // serve never publishes.
    // -----------------------------------------------------------------------
    assign done         = trk_publishable && drained;
    assign drained      = wr_drained;
    assign credit_beats = wr_credit_beats;
    assign credit_free  = wr_credit_free;
    assign base_aligned = wr_base_aligned;
    assign page_refusals= wr_page_refusals;
    assign burst_beats  = wr_burst_beats;
    assign burst_clamps = wr_burst_clamps;

    // -----------------------------------------------------------------------
    // The acknowledgement, and the tooth that withholds one of them.
    //
    // `wr_ack` is one cycle wide and one per committed response.  With
    // `SAB_HOLD_ONE_ACK` at 0 this is the writer's pulse untouched, which is
    // the whole of the seam.  At 1 the frame's **first** pulse is dropped: the
    // tracker's count stops one short and DONE must not rise.
    // -----------------------------------------------------------------------

    reg  sab_first_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          sab_first_seen <= 1'b0;
        else if (start)      sab_first_seen <= 1'b0;
        else if (wr_ack_raw) sab_first_seen <= 1'b1;
    end

    wire ack_trk = (SAB_HOLD_ONE_ACK != 0) ? (wr_ack_raw && sab_first_seen)
                                           : wr_ack_raw;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                          sab_acks_withheld <= 32'd0;
        else if (start)                      sab_acks_withheld <= 32'd0;
        else if (wr_ack_raw && !ack_trk)     sab_acks_withheld <= sab_acks_withheld + 32'd1;
    end

    // -----------------------------------------------------------------------
    // The tracker.  Armed by the same `start`, expecting the derived count, fed
    // the writer's responses -- and fed the engine's `done` on `last_valid`,
    // which it never acts on.
    // -----------------------------------------------------------------------

    d64_done_tracker #(
        .IDLE_TIMEOUT(IDLE_TIMEOUT),
        .SAB_DONE_ON_LAST_VALID(0),
        .SAB_NO_TIMEOUT(0)
    ) u_trk (
        .clk(clk), .rst_n(rst_n),
        .start(start), .exp_count(exp_count),
        .ack(ack_trk), .last_valid(tail_done),
        .done(responses_done), .publishable(trk_publishable), .fault(wr_frame_fault),
        .done_reason(done_reason), .outstanding(trk_outstanding),
        .acks(trk_acks), .extra_acks(trk_extra_acks),
        .last_valids(trk_last_valids),
        .stall_cycles(trk_stall_cycles), .stall_timeout(trk_stall_timeout)
    );

endmodule
