`timescale 1ns/1ps

// D3c6-a3 -- the parameterised MAC's interface over a bank.
//
// `d64_conv_engine` and `d64_tconv_engine` are both closed and both instantiate
// `d64_tconv_mac` directly, and the schedulers around them are what those
// regressions actually gate on.  This is the seam that lets a bank go underneath
// them **without touching a scheduler, a store, a metadata path or a scratch
// address**: same ports, same handshake, same widths.
//
// **D3c6-e split it in two, and the split is the whole point.**  The operand
// mapping is pure wiring and the bank is a bank, so a client that does not own
// its own PE needs the first half on its own:
//
//     d64_mac128_compat_map   legacy operands -> bank operands  (wiring)
//     d64_mac128_compat       compat_map + a local d64_mac128   (what was here)
//
// `d64_down_engine` already hands its operands to a bank it does not own.  The
// trunk engine needs the same, and in the finished network every scheduler
// time-shares one bank -- 144 DSP blocks, not 144 each.  Keeping this module as
// the map plus a *local* bank is what lets every existing regression keep
// running unchanged while that happens.
//
// What is *not* here
// ------------------
//
// No mode special-casing inside the bank.  `mode` is a function of the shape and
// is fixed for the life of the instance -- a layer is a shape, and a shape is a
// mode -- so the runtime mux the bank carries is not exercised by these two
// engines at all.  It exists for head.
module d64_mac128_compat #(
    parameter IC_PAR    = 8,
    parameter OC_LANES  = 16,
    parameter PIX_PAR   = 1,
    parameter ACT_W     = 11,
    parameter WGT_W     = 8,
    parameter ACC_W     = 32,
    parameter M_W       = 16,
    parameter SHIFT_W   = 5,
    parameter OUT_W     = 10,
    parameter RQ_LANES  = 8,
    parameter integer QMIN = 0,
    parameter integer QMAX = 1023,
    // Verification only, folded away at 0, and it has to be here: the engine's
    // own testbench builds this variant and requires it to **fail**.  An adapter
    // that dropped the parameter would make the sabotaged build pass, which is a
    // tooth that does not fire and worse than no tooth at all.
    parameter integer ROLLOVER_SABOTAGE = 0,
    parameter integer RQ_LOOKAHEAD = 3
) (
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          slot_valid,
    input  wire [PIX_PAR*IC_PAR*ACT_W-1:0]   act,
    input  wire [IC_PAR*OC_LANES*WGT_W-1:0]  wgt,

    input  wire                          pix_start,
    input  wire [PIX_PAR*OC_LANES*ACC_W-1:0] bias,

    input  wire                          rq_start,
    output wire                          rq_ready,
    output wire                          rq_ready_n,
    input  wire [OC_LANES*M_W-1:0]       rq_m,
    input  wire [OC_LANES*SHIFT_W-1:0]   rq_shift,
    input  wire signed [OUT_W-1:0]       rq_zero,

    output wire [RQ_LANES*OUT_W-1:0]     y,
    output wire [RQ_LANES-1:0]           y_valid,
    output wire                          rq_busy,
    output wire                          rq_done,

    output wire [PIX_PAR*OC_LANES*ACC_W-1:0] acc_o,
    output wire [PIX_PAR*OC_LANES*ACC_W-1:0] rq_snap_o,
    output wire                          rq_overrun
);

    localparam LANES  = 128;
    localparam OC_MAX = 16;

    wire [LANES*ACT_W-1:0]    b_act;
    wire [LANES*WGT_W-1:0]    b_wgt;
    wire [LANES*ACC_W-1:0]    b_bias;
    wire [OC_MAX*M_W-1:0]     b_m;
    wire [OC_MAX*SHIFT_W-1:0] b_shift;
    wire [1:0]                b_mode;
    wire [OUT_W-1:0]          b_qmin, b_qmax;
    wire [LANES*ACC_W-1:0]    b_acc, b_snap;

    d64_mac128_compat_map #(
        .IC_PAR(IC_PAR), .OC_LANES(OC_LANES), .PIX_PAR(PIX_PAR),
        .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W),
        .SHIFT_W(SHIFT_W), .OUT_W(OUT_W), .RQ_LANES(RQ_LANES),
        .QMIN(QMIN), .QMAX(QMAX), .LANES(LANES), .OC_MAX(OC_MAX)
    ) u_map (
        .act(act), .wgt(wgt), .bias(bias), .rq_m(rq_m), .rq_shift(rq_shift),
        .b_act(b_act), .b_wgt(b_wgt), .b_bias(b_bias), .b_m(b_m),
        .b_shift(b_shift), .b_mode(b_mode), .b_qmin(b_qmin), .b_qmax(b_qmax),
        .b_acc(b_acc), .b_snapshot(b_snap),
        .acc_o(acc_o), .rq_snap_o(rq_snap_o)
    );

    d64_mac128 #(.ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W),
                 .SHIFT_W(SHIFT_W), .OUT_W(OUT_W), .RQ_LANES(RQ_LANES),
                 .LANES(LANES), .OC_MAX(OC_MAX),
                 .ROLLOVER_SABOTAGE(ROLLOVER_SABOTAGE),
                 .RQ_LOOKAHEAD(RQ_LOOKAHEAD)) u_bank (
        .clk(clk), .rst_n(rst_n), .mode(b_mode),
        .slot_valid(slot_valid), .act(b_act), .wgt(b_wgt),
        .pix_start(pix_start), .bias(b_bias),
        .rq_start(rq_start), .rq_ready(rq_ready), .rq_ready_n(rq_ready_n),
        .rq_m(b_m), .rq_shift(b_shift), .rq_zero(rq_zero),
        .rq_qmin(b_qmin), .rq_qmax(b_qmax),
        .y(y), .y_valid(y_valid), .rq_busy(rq_busy), .rq_done(rq_done),
        .rq_overrun(rq_overrun), .mode_viol(), .acc_o(b_acc), .rq_snap_o(b_snap)
    );

endmodule
