`timescale 1ns/1ps

// D3c6-e -- the legacy-to-bank operand mapping, and nothing else.
//
// This is the whole of `d64_mac128_compat` except the bank.  It exists so a
// client can present the parameterised MAC's interface and still hand its
// operands to a PE it does not own: the down engine already does that, the
// trunk engine needs to, and in the finished network every scheduler time-shares
// one bank.
//
// The mapping, unchanged from the compat
// -------------------------------------
//
// The reference numbers its products `gi = PP*IC_PAR + II` with
// `PP = pixel*OC + oc`, and the bank numbers its multipliers
// `m = (p*OC + o)*IC + i`.  Those are the same number -- reduction innermost --
// so the only thing that changes is which **lane** of the reference's flat
// operand ports each multiplier reads:
//
//     b_act[m] = act[(PP/OC)*IC + II]
//     b_wgt[m] = wgt[II*OC + (PP%OC)]
//     PP = m/IC,  II = m%IC
//
// `IC_PAR` and `OC_LANES` are parameters, so every one of those is an
// elaboration-time constant: no divider, no mux, no logic at all.  It is a
// permutation of wires, which is what made the split safe.
//
// The mode encoding, and the table that has to match it
// -----------------------------------------------------
//
// The bank (`d64_mac128`) decodes its two-bit mode port as four repartitions,
// and this table is the only thing in the design that turns a shape into one of
// them:
//
//     2'd0  MAC_P1_I8_O16   PIX_PAR 1 / IC_PAR 8 / OC_LANES 16   trunk, ups.0
//     2'd1  MAC_P2_I8_O8    PIX_PAR 2 / IC_PAR 8 / OC_LANES  8   ups.1
//     2'd2  MAC_P4_I8_O4    PIX_PAR 4 / IC_PAR 8 / OC_LANES  4   tail
//     2'd3  MAC_P4_I4_O8    PIX_PAR 4 / IC_PAR 4 / OC_LANES  8   head
//
// **`2'd3` is a reused encoding, and `d64_mac128.v` reused it.**  Head used to
// be `PIX 16 / IC 1 / OC 8` -- sixteen pixels and no reduction -- and that shape
// held `2'd3` for as long as it was head's.  When head became four pixels by
// four input channels (commit `c80a06d`, `MAC_P4_I4_O8`) the bank took the
// **free** encoding over, and its own comment records the reasoning: "Nothing
// else in the design drove `2'd3`".  That was true of the design and false of
// this table, which went on decoding the retired shape.
//
// The consequence, and it is the reason this header exists: a caller presenting
// **`P4/I4/O8`, head's live shape**, matched nothing here and fell through to
// the old default, `2'd0` -- **a legal layer silently computed as a different
// one**, with no error raised anywhere.  And because `d64_head_ctrl` drives
// `2'd3` at its own bank directly rather than through this map, a front half
// that passes on the board never exercised this table for head's shape at all.
//
// A shape that is not one of the four is refused, not defaulted
// -------------------------------------------------------------
//
// `b_mode_valid` is low for a shape that is not one of the four above, and while
// it is low **`b_mode` carries `2'bxx`** rather than a mode -- so a caller that
// ignores the flag fails loudly in a simulator instead of quietly computing mode
// 0's layer.
//
// What a caller must do when `b_mode_valid` is low:
//
//   * **do not** pass `b_mode` on to a bank and let the layer run.  Mode 0 is
//     neither an error nor a no-op: it is a different reduction depth over a
//     different operand permutation, and it produces numbers.
//   * stop the layer and raise a fault -- hold `slot_valid`, `pix_start` and
//     `rq_start` low -- or refuse to elaborate the shape at all.
//   * `b_mode_valid` is an elaboration-time constant, so a caller that wants
//     this caught at **build** time has to gate its own instantiation on it.
//     This module cannot fail a build from inside, and `2'bxx` is only loud in a
//     simulator: after mapping, an illegal shape still produces some two-bit
//     value.  The flag is the contract; the `x` is the alarm.
//
// `tools/run_mac128_compat_map.sh` is the gate that holds the table to the
// bank's four modes, requires the retired shape and a factorisation that was
// never a mode to come back invalid, and carries a tooth that puts the
// `P16/I1/O8` entry back and requires the gate to fail.
//
// The bank places 128 accumulators whatever the mode is and an `IC 8` shape uses
// the first sixteen, so the rest take zero; and the requant constants are per
// output channel, so lanes past `OC_LANES` take zero too rather than being left
// to a read past the end of the port.
module d64_mac128_compat_map #(
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
    parameter LANES     = 128,
    parameter OC_MAX    = 16
) (
    // ---- the parameterised MAC's operand side ------------------------------
    input  wire [PIX_PAR*IC_PAR*ACT_W-1:0]   act,
    input  wire [IC_PAR*OC_LANES*WGT_W-1:0]  wgt,
    input  wire [PIX_PAR*OC_LANES*ACC_W-1:0] bias,
    input  wire [OC_LANES*M_W-1:0]           rq_m,
    input  wire [OC_LANES*SHIFT_W-1:0]       rq_shift,

    // ---- the bank's operand side -------------------------------------------
    output wire [LANES*ACT_W-1:0]    b_act,
    output wire [LANES*WGT_W-1:0]    b_wgt,
    output wire [LANES*ACC_W-1:0]    b_bias,
    output wire [OC_MAX*M_W-1:0]     b_m,
    output wire [OC_MAX*SHIFT_W-1:0] b_shift,
    output wire [1:0]                b_mode,
    // **The refusal.**  Low for a shape that is not one of the bank's
    // four, in which case `b_mode` is `2'bxx` and must not be used.  The
    // header says what a caller has to do when this is low.
    output wire                      b_mode_valid,
    output wire [OUT_W-1:0]          b_qmin,
    output wire [OUT_W-1:0]          b_qmax,

    // ---- the bank's response side ------------------------------------------
    input  wire [LANES*ACC_W-1:0]    b_acc,
    input  wire [LANES*ACC_W-1:0]    b_snapshot,
    output wire [PIX_PAR*OC_LANES*ACC_W-1:0] acc_o,
    output wire [PIX_PAR*OC_LANES*ACC_W-1:0] rq_snap_o
);

    localparam NPIX = PIX_PAR * OC_LANES;

    // Which of the four repartitions this shape is.  One predicate a shape, so
    // that the table and the validity flag cannot drift apart and an unmatched
    // shape is a *value* rather than a fall-through.
    localparam IS_P1_I8_O16 = (PIX_PAR ==  1 && IC_PAR == 8 && OC_LANES == 16);
    localparam IS_P2_I8_O8  = (PIX_PAR ==  2 && IC_PAR == 8 && OC_LANES ==  8);
    localparam IS_P4_I8_O4  = (PIX_PAR ==  4 && IC_PAR == 8 && OC_LANES ==  4);
    // `2'd3`, and **which shape this predicate names is where the bug was**: it
    // read `PIX 16 / IC 1 / OC 8`, head's retired shape, until the bank reused
    // the encoding for `MAC_P4_I4_O8`.  Do not put the old shape back here; the
    // header says what happened and why the legal shapes are these four.
    localparam IS_P4_I4_O8  = (PIX_PAR ==  4 && IC_PAR == 4 && OC_LANES ==  8);

    localparam MODE_VALID = IS_P1_I8_O16 | IS_P2_I8_O8 | IS_P4_I8_O4 |
                            IS_P4_I4_O8;

    // **Unmatched is refused, not defaulted.**  The `2'bxx` arm is the refusal:
    // a build that ignores `b_mode_valid` gets `x` into the bank's mode and
    // fails in a simulator rather than quietly computing mode 0's layer.  The
    // `2'd0` that used to be here is the defect this table is being corrected
    // for -- a fall-through that is a wrong answer which looks like a right one.
    localparam [1:0] MODE = IS_P1_I8_O16 ? 2'd0 :
                            IS_P2_I8_O8  ? 2'd1 :
                            IS_P4_I8_O4  ? 2'd2 :
                            IS_P4_I4_O8  ? 2'd3 : 2'bxx;

    assign b_mode       = MODE;
    assign b_mode_valid = MODE_VALID;
    assign b_qmin       = QMIN[OUT_W-1:0];
    assign b_qmax       = QMAX[OUT_W-1:0];

    genvar gm;
    generate
        for (gm = 0; gm < LANES; gm = gm + 1) begin : g_map
            localparam PP = gm / IC_PAR;                     // pixel*OC + oc
            localparam II = gm % IC_PAR;                     // input channel
            localparam AP = (PP / OC_LANES) * IC_PAR + II;   // activation lane
            localparam WP = II * OC_LANES + (PP % OC_LANES); // weight lane

            assign b_act[gm*ACT_W +: ACT_W] = act[AP*ACT_W +: ACT_W];
            assign b_wgt[gm*WGT_W +: WGT_W] = wgt[WP*WGT_W +: WGT_W];
            assign b_bias[gm*ACC_W +: ACC_W] = (gm < NPIX)
                ? bias[gm*ACC_W +: ACC_W] : {ACC_W{1'b0}};
        end
    endgenerate

    generate
        for (gm = 0; gm < OC_MAX; gm = gm + 1) begin : g_rq
            if (gm < OC_LANES) begin
                assign b_m[gm*M_W +: M_W]         = rq_m[gm*M_W +: M_W];
                assign b_shift[gm*SHIFT_W +: SHIFT_W] = rq_shift[gm*SHIFT_W +: SHIFT_W];
            end
            else begin
                assign b_m[gm*M_W +: M_W]         = {M_W{1'b0}};
                assign b_shift[gm*SHIFT_W +: SHIFT_W] = {SHIFT_W{1'b0}};
            end
        end
    endgenerate

    // Lane `m` is the reference's accumulator `m` -- the numbering is the same
    // one -- so the exposed views are the bank's first `NPIX` lanes and nothing
    // is reordered here.
    assign acc_o     = b_acc[NPIX*ACC_W-1:0];
    assign rq_snap_o = b_snapshot[NPIX*ACC_W-1:0];

endmodule
