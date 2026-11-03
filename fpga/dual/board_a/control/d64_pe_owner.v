`timescale 1ns/1ps

// F2 2.1 -- one bank, N owners.
//
// The finished network has one 128-multiplier bank and four things that want
// it: the trunk chain, ups.0, ups.1 and the head.  144 DSP blocks do not fit
// twice, so the bank is built once and handed around, and this is the thing
// that hands it.
//
// What this is not
// ----------------
//
// It is not a scheduler and it does not know what a layer is.  It is a
// **combinational mux on the request side, a fan-out of the response side, and
// one register that says whose turn it is**.  Everything else here exists to
// make sure that register moves at a moment when moving it cannot be observed
// by anything downstream.
//
// The seam is section 1 of `docs/F2_INTERFACES_2026-10-06.md` and the widths
// are the network's, not any one engine's: one bank, four shapes, and every
// request carries its own `qmin`/`qmax` so an 8-bit layer is a `qmax` of 255 on
// a 10-bit lane instead of a second bank.
//
// The three decisions that are not obvious
// ----------------------------------------
//
// **1. `pe_req_mode` is a port and it is the reason this module can exist.**
// `tb_d64_conv_engine_ext` drives its bank's `mode` as `u_ext.mac_mode`, read
// out of the engine's hierarchy.  That works in simulation and is not a signal
// a synthesised arbiter can see.  An owner that does not own its bank says
// which shape it is, out loud, on the seam -- section 1 lists `pe_req_mode`
// for exactly this reason.
//
// **2. The mode follows the registered selected owner.**
// A do_switch-selected target mode formed a combinational loop in mapped RTL:
// bank mode -> rq_ready_n -> client request -> bank idle -> do_switch -> mode.
// The mode now changes only when sel_q changes at a clock edge. Every client
// remains idle while deselected and starts after the registered sel_ack, so
// the bank samples the new mode on a quiet edge before any new request arrives.
// The root provides this interval in its selection/start sequencer. The owner
// differential gate retains all original data and mode-violation expectations:
// arbiter/private banks have zero mode violations, and both sabotage arms fail.
//
// **3. `bank_idle` is not `!rq_busy`.**  The bank's `run` register is set on
// the accept edge and `rq_busy` is `run` delayed by one, so there is a cycle
// where the requant sequencer is mid-flight and the bank still reports idle.
// The owners that exist say `mac_mode_safe` is high in exactly that cycle:
// `d64_tconv_engine` drives `!b_live && !mac_busy` and `d64_head_ctrl` drives
// `!mac_slot_valid && !op_valid && !req_valid && !mac_rq_busy`, and in both the
// busy term is the bank's `rq_busy`.
//
// A switch there changes `rq_cyc` and the requant channel index underneath a
// pass sequence that is already half-decoded, silently, and the wrong `y` comes
// back -- and the bank's own `mode_viol` does **not** catch it, because its
// condition is `slot_valid || rq_busy || acc_dirty` and in that cycle all three
// are zero.  `bank_idle` therefore requires the bank quiet on this cycle **and
// on the previous one**, which is one cycle of margin and no more.
//
//   `IDLE_SABOTAGE` builds the naive `~rq_busy` version.  The gate requires
//   that build to **fail** -- otherwise the margin is untested code -- and the
//   run reports *how* it fails: on values, with the bank's own `mode_viol` at
//   zero throughout.
//
// What every owner sees when it is not selected
// ---------------------------------------------
//
// Section 2.1 fixes the constants and they are driven from a generate loop so
// that nothing but the selected owner can read the bank: `rq_ready` 1,
// `rq_ready_n` 1, and zero on `y`, `y_valid`, `rq_busy`, `rq_done`,
// `rq_overrun`, `mode_viol`, `acc` and `snapshot`.  An owner that is held off
// therefore sees `rq_ready` and accepts into nothing -- which is what the
// contract says happens and is why an owner must not run while deselected.
//
// A held-off owner's `acc` and `snapshot` are zero and not the selected
// owner's, and that is not observability lost: section 2.1 lists them as
// per-owner outputs, so an unselected owner that could read the bank through
// them would be reading a peer's accumulators.
//
// The handshake
// -------------
//
// `sel_req` requests the switch, `owner_sel` names the target, `sel_ack`
// answers.  The switch commits on the first cycle `sel_req` is high **and**
// `owner_sel` differs from the selected owner **and** the current owner's
// `mac_mode_safe` is high **and** `bank_idle` is high.  `sel_ack` is
// registered, so it is high on the first cycle the new owner is connected --
// a level held to it, not a pulse the requester has to catch.  `sel_req` held
// with `owner_sel` equal to the selected owner is acknowledged immediately and
// moves nothing.
//
// **`owner_sel` changing is not a switch.**  A change with no `sel_req` does
// nothing at all, and a change with `sel_req` that lands while the condition is
// false is not lost: it is adopted on the first cycle the condition holds, and
// this is why `owner_sel` is sampled at the commit and not at the request.
module d64_pe_owner #(
    // Network widths.  A port list cannot see a `localparam`, so everything a
    // port needs is a `parameter`.
    parameter NOWNERS = 4,
    parameter LANES   = 128,
    parameter OC_MAX  = 16,
    parameter ACT_W   = 11,
    parameter WGT_W   = 8,
    parameter ACC_W   = 32,
    parameter M_W     = 16,
    parameter SHIFT_W = 5,
    parameter OUT_W   = 10,
    parameter RQ_LANES = 8,

    // Verification only, folded away at 0.
    parameter integer SEL_SABOTAGE  = 0,  // switch on `owner_sel` alone
    parameter integer IDLE_SABOTAGE = 0   // `bank_idle` drops its one-cycle margin
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // ---- the selection handshake -----------------------------------------
    input  wire [$clog2(NOWNERS)-1:0]        owner_sel,
    input  wire                              sel_req,
    output reg                               sel_ack,

    // ---- per-owner request, owner `i` at offset `i` of each bus ----------
    //
    // One flat bus per seam signal rather than `NOWNERS` port names: the
    // offsets are the contract's, the widths do not change with the owner
    // count, and an owner is added by widening a parameter instead of by
    // editing this module and everyone who instantiates it.
    input  wire [NOWNERS-1:0]                    pe_req_slot_valid_i,
    input  wire [NOWNERS*LANES*ACT_W-1:0]        pe_req_act_i,
    input  wire [NOWNERS*LANES*WGT_W-1:0]        pe_req_wgt_i,
    input  wire [NOWNERS-1:0]                    pe_req_pix_start_i,
    input  wire [NOWNERS*LANES*ACC_W-1:0]        pe_req_bias_i,
    input  wire [NOWNERS*2-1:0]                  pe_req_mode_i,
    input  wire [NOWNERS-1:0]                    pe_req_rq_start_i,
    input  wire [NOWNERS*OC_MAX*M_W-1:0]         pe_req_rq_m_i,
    input  wire [NOWNERS*OC_MAX*SHIFT_W-1:0]     pe_req_rq_shift_i,
    input  wire [NOWNERS*OUT_W-1:0]              pe_req_rq_zero_i,
    input  wire [NOWNERS*OUT_W-1:0]              pe_req_rq_qmin_i,
    input  wire [NOWNERS*OUT_W-1:0]              pe_req_rq_qmax_i,
    // Not part of the bank: the owner's own statement that a mode change now
    // is safe.  It is what makes the switch land on a boundary instead of in
    // the middle of a phase.
    input  wire [NOWNERS-1:0]                    mac_mode_safe_i,

    // ---- per-owner response ---------------------------------------------
    output wire [NOWNERS-1:0]                    pe_rsp_rq_ready_i,
    output wire [NOWNERS-1:0]                    pe_rsp_rq_ready_n_i,
    output wire [NOWNERS*RQ_LANES*OUT_W-1:0]     pe_rsp_y_i,
    output wire [NOWNERS*RQ_LANES-1:0]           pe_rsp_y_valid_i,
    output wire [NOWNERS-1:0]                    pe_rsp_rq_busy_i,
    output wire [NOWNERS-1:0]                    pe_rsp_rq_done_i,
    output wire [NOWNERS-1:0]                    pe_rsp_rq_overrun_i,
    output wire [NOWNERS*32-1:0]                 pe_rsp_mode_viol_i,
    output wire [NOWNERS*LANES*ACC_W-1:0]        pe_rsp_acc_i,
    output wire [NOWNERS*LANES*ACC_W-1:0]        pe_rsp_snapshot_i,

    // ---- the bank's response side (inputs here) ---------------------------
    input  wire                              bank_rq_ready,
    input  wire                              bank_rq_ready_n,
    input  wire [RQ_LANES*OUT_W-1:0]         bank_y,
    input  wire [RQ_LANES-1:0]               bank_y_valid,
    input  wire                              bank_rq_busy,
    input  wire                              bank_rq_done,
    input  wire                              bank_rq_overrun,
    input  wire [31:0]                       bank_mode_viol,
    input  wire [LANES*ACC_W-1:0]            bank_acc,
    input  wire [LANES*ACC_W-1:0]            bank_rq_snap,

    // ---- the bank's request side (outputs here) ---------------------------
    output wire [1:0]                        bank_mode,
    output wire                              bank_slot_valid,
    output wire [LANES*ACT_W-1:0]            bank_act,
    output wire [LANES*WGT_W-1:0]            bank_wgt,
    output wire                              bank_pix_start,
    output wire [LANES*ACC_W-1:0]            bank_bias,
    output wire                              bank_rq_start,
    output wire [OC_MAX*M_W-1:0]             bank_rq_m,
    output wire [OC_MAX*SHIFT_W-1:0]         bank_rq_shift,
    output wire [OUT_W-1:0]                  bank_rq_zero,
    output wire [OUT_W-1:0]                  bank_rq_qmin,
    output wire [OUT_W-1:0]                  bank_rq_qmax,

    // The one thing the testbench could otherwise only guess at: whether a
    // switch happened where it was supposed to.
    output wire                              bank_idle
);

    localparam SEL_W = $clog2(NOWNERS);

    // Offsets, named, because `selx*1408` at a call site is a number nobody can
    // check against the contract.
    localparam integer OFF_ACT  = LANES*ACT_W;
    localparam integer OFF_WGT  = LANES*WGT_W;
    localparam integer OFF_BIAS = LANES*ACC_W;
    localparam integer OFF_MODE = 2;
    localparam integer OFF_M    = OC_MAX*M_W;
    localparam integer OFF_SH   = OC_MAX*SHIFT_W;
    localparam integer OFF_ONE  = OUT_W;
    localparam integer OFF_Y    = RQ_LANES*OUT_W;
    localparam integer OFF_YV   = RQ_LANES;
    localparam integer OFF_MV   = 32;
    localparam integer OFF_ACC  = LANES*ACC_W;

    // Zero-extends the selection to a width the offset multiply cannot
    // truncate in.  Written as an assignment through a wider function return
    // and not as `{{(32-SEL_W){1'b0}}, sel_q}`: a zero-width replication is not
    // legal Verilog when the widths are equal, and `NOWNERS == 2^32` is a
    // configuration nobody builds until the day someone does.
    function [31:0] zx;
        input [SEL_W-1:0] v;
        begin
            zx = v;
        end
    endfunction

    reg  [SEL_W-1:0] sel_q;
    wire [31:0]      selx = zx(sel_q);
    wire [31:0]      tgtx = zx(owner_sel);
    wire             sel_same = (owner_sel == sel_q);

    // ------------------------------------------------------------------
    // the bank's idle report
    // ------------------------------------------------------------------
    //
    // **Nothing here reads the bank's outputs through a path the bank reads
    // back.**  `bank_rq_ready` is a function of `mode` (`rq_cyc`), so a
    // `rq_start & rq_ready` term in this expression would close a combinational
    // loop through `bank_mode`.  `rq_start` alone says the same thing about
    // the accept edge -- an accept is a request -- and cannot loop.

    wire bank_req_live   = bank_slot_valid | bank_pix_start | bank_rq_start;
    wire bank_active_now = bank_rq_busy | bank_rq_done | bank_req_live;

    reg  bank_active_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bank_active_d <= 1'b1;   // conservative out of reset
        else        bank_active_d <= bank_active_now;
    end

    // One cycle of margin, and the reason is `d64_mac128`'s own registers:
    // `run` is set on the accept edge, `rq_busy` is `run` delayed, and
    // `s_ok` is loaded on that same accept edge.  A switch in the gap between
    // them is a switch in the middle of a pass sequence.
    assign bank_idle = (IDLE_SABOTAGE != 0)
                     ? ~bank_rq_busy
                     : (~bank_active_now & ~bank_active_d);

    // ------------------------------------------------------------------
    // the selection
    // ------------------------------------------------------------------

    wire safe_sel = mac_mode_safe_i[sel_q];

    wire do_switch = SEL_SABOTAGE != 0
                   ? (sel_req & ~sel_same)
                   : (sel_req & ~sel_same & safe_sel & bank_idle);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                sel_q <= {SEL_W{1'b0}};
        else if (do_switch)        sel_q <= owner_sel;
    end

    // Registered, so it is a level over the first cycle the new owner is
    // connected rather than a pulse the requester has to be watching for.  A
    // request naming the owner that is already selected is acknowledged on the
    // same terms and moves nothing.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sel_ack <= 1'b0;
        else        sel_ack <= sel_req & (sel_same | do_switch);
    end

    // ------------------------------------------------------------------
    // the request mux
    // ------------------------------------------------------------------
    //
    // Mode and every other request are driven from the registered selected
    // owner. The mux is a part-select with a variable base,
    // which is Verilog-2001 and is not a case statement with `NOWNERS` arms.

    assign bank_slot_valid = pe_req_slot_valid_i[sel_q];
    assign bank_pix_start  = pe_req_pix_start_i[sel_q];
    assign bank_rq_start   = pe_req_rq_start_i[sel_q];

    assign bank_act   = pe_req_act_i[selx*OFF_ACT +: OFF_ACT];
    assign bank_wgt   = pe_req_wgt_i[selx*OFF_WGT +: OFF_WGT];
    assign bank_bias  = pe_req_bias_i[selx*OFF_BIAS +: OFF_BIAS];
    assign bank_rq_m  = pe_req_rq_m_i[selx*OFF_M +: OFF_M];
    assign bank_rq_shift = pe_req_rq_shift_i[selx*OFF_SH +: OFF_SH];
    assign bank_rq_zero  = pe_req_rq_zero_i[selx*OFF_ONE +: OFF_ONE];
    assign bank_rq_qmin  = pe_req_rq_qmin_i[selx*OFF_ONE +: OFF_ONE];
    assign bank_rq_qmax  = pe_req_rq_qmax_i[selx*OFF_ONE +: OFF_ONE];

    // Selection is registered. Do not feed do_switch back into the bank's mode:
    // request live -> idle -> switch -> mode -> rq_ready_n -> request live loops.
    // The root holds each client idle until the registered acknowledgement.
    assign bank_mode = pe_req_mode_i[selx*OFF_MODE +: OFF_MODE];

    // ------------------------------------------------------------------
    // the response fan-out
    // ------------------------------------------------------------------
    //
    // The constants for the held-off owners are section 2.1's, written once
    // and not inferred.  A held-off owner that saw the bank's real `rq_ready`
    // would accept into nothing with a handshake that said otherwise; one that
    // saw the bank's `acc` would read a peer's accumulators.

    genvar gi;
    generate
        for (gi = 0; gi < NOWNERS; gi = gi + 1) begin : g_rsp
            localparam [SEL_W-1:0] GI_SEL = gi;
            wire sel_this = (sel_q == GI_SEL);

            assign pe_rsp_rq_ready_i[gi]   = sel_this ? bank_rq_ready   : 1'b1;
            assign pe_rsp_rq_ready_n_i[gi] = sel_this ? bank_rq_ready_n : 1'b1;
            assign pe_rsp_rq_busy_i[gi]    = sel_this ? bank_rq_busy    : 1'b0;
            assign pe_rsp_rq_done_i[gi]    = sel_this ? bank_rq_done    : 1'b0;
            assign pe_rsp_rq_overrun_i[gi] = sel_this ? bank_rq_overrun : 1'b0;

            assign pe_rsp_y_i[gi*OFF_Y +: OFF_Y] =
                sel_this ? bank_y : {OFF_Y{1'b0}};
            assign pe_rsp_y_valid_i[gi*OFF_YV +: OFF_YV] =
                sel_this ? bank_y_valid : {OFF_YV{1'b0}};
            assign pe_rsp_mode_viol_i[gi*OFF_MV +: OFF_MV] =
                sel_this ? bank_mode_viol : {OFF_MV{1'b0}};
            assign pe_rsp_acc_i[gi*OFF_ACC +: OFF_ACC] =
                sel_this ? bank_acc : {OFF_ACC{1'b0}};
            assign pe_rsp_snapshot_i[gi*OFF_ACC +: OFF_ACC] =
                sel_this ? bank_rq_snap : {OFF_ACC{1'b0}};
        end
    endgenerate

endmodule
