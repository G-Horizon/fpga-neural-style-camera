`timescale 1ns/1ps

// The residual add, which is one constant.
//
// D2: `q_out = q_skip + q_conv3 - zero`, then saturate, **no requant between the
// branches**.  Two's complement makes it exact -- `(a - z) + (b - z) = (a + b - z)
// - z` -- so the sum of two values in one format is that format's sum minus one
// zero point, and that single constant is the whole operator.
//
// It is a module rather than two lines in a testbench because it is the only
// arithmetic in a residual block that the shared MAC backend does not do, and
// "the block is the backend plus one adder" is a claim worth being able to point
// at.  The trunk scheduler issues conv3's skip read and its result write in the
// same four cycles, so this sits on that write path and costs no extra cycle.
//
// **No requant, and that is the contract, not an omission.**  Both operands are
// already in the trunk's format and the output is too.  Putting a requant here
// would be the "dual" residual contract, which D2 does not use; the assert in
// `tools/generate_conv_arithmetic_golden.py` is what keeps the two from drifting
// apart silently.
module d64_residual_add #(
    parameter integer DW   = 10,      // the trunk stream's stored width
    parameter integer QMIN = 0,
    parameter integer QMAX = 1023,
    parameter integer ZERO = 413,
    parameter integer RUNTIME_PROFILE = 0
) (
    input  wire [DW-1:0] profile_zero,
    input  wire [DW-1:0] skip,
    input  wire [DW-1:0] branch,
    output wire [DW-1:0] y
);

    // Sized and signed explicitly.  `- ZERO` against an `integer` parameter
    // makes the whole expression unsigned in Verilog, and an unsigned compare
    // against a negative sum is a comparison the design never wanted to make.
    localparam signed [DW+1:0] ZERO_V = ZERO;
    localparam [DW-1:0]        QMIN_V = QMIN;
    localparam [DW-1:0]        QMAX_V = QMAX;

    wire signed [DW+1:0] zero_live = RUNTIME_PROFILE ?
        $signed({2'b00, profile_zero}) : ZERO_V;

    wire signed [DW+1:0] s = $signed({1'b0, skip})
                           + $signed({1'b0, branch})
                           - zero_live;

    assign y = (s < $signed({1'b0, QMIN_V})) ? QMIN_V
             : (s > $signed({1'b0, QMAX_V})) ? QMAX_V
             : s[DW-1:0];

endmodule
