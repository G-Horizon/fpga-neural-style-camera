// One bit across a clock boundary: N flops, all in the destination domain.
//
// This is the primitive the other two classes are built from, and it is on its own
// because "synchronise a bit" is the one thing a crossing can do that needs no
// agreement about the value: a single bit is either the old value or the new one,
// and both are legal, so nothing has to be held, coded or acknowledged.
//
// **It is also the module that must not be used on a bus.**  A bus is not a set of
// independent bits: the moment two of them change, a synchroniser can sample one
// bit's new value beside another bit's old one and produce a word that was never
// written.  That is what `d64_cdc_gray_sync` (cross a counter whose every step moves
// one bit) and `d64_cdc_descriptor_src`/`_dst` (hold the payload still and qualify
// the sampling with a toggle) exist to prevent, and it is what the descriptor
// testbench's tooth demonstrates by compiling exactly this mistake.
//
// STAGES is a parameter and two is the minimum that means anything: one flop passes
// metastability through, and the tools treat `async_reg` as a directive to keep the
// chain together so the MTBF arithmetic is about the chain rather than the parts.
//
// `async_reg` is not the same instruction as `set_clock_groups`.  The attribute tells
// the mapper and the placer what the flops are for; the SDC tells static timing not to
// analyse the path.  Neither substitutes for the other, and neither is what makes the
// *payload* safe -- see docs/CDC_CLOSURE_2026-10-07.md.
module d64_cdc_bit_sync #(
    parameter STAGES = 2
) (
    input  wire clk,
    input  wire rst_n,
    input  wire d,
    output wire q
);
    (* async_reg = "true" *) reg [STAGES-1:0] s;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) s <= {STAGES{1'b0}};
        else        s <= {s[STAGES-2:0], d};

    assign q = s[STAGES-1];

endmodule
