// The source half of a Gray-coded counter crossing.  Source domain only.
//
// ---------------------------------------------------------------------------
// The class: one bit per step
// ---------------------------------------------------------------------------
//
// A counter crosses in *binary* badly, and it is worth being exact about why, because
// "it is only a counter" is how it gets done by accident.  7 -> 8 is `0111 -> 1000`:
// four bits move.  They do not arrive at the destination's sampling flop at the same
// time -- they arrive within some skew, which is a routing property and not a design
// property -- so a destination edge that lands inside that window sees some bits new
// and some bits old, and the word it captures is one the counter never held.  Two flops
// do not fix this: two flops fix metastability on *one* signal, and a bus in motion is
// not metastability.
//
// Gray coding removes the window by removing the multi-bit step.  Consecutive Gray
// values differ in exactly one bit, so at most one bit of the crossed bus is ever in
// motion, and whatever the destination samples is either the old value or the new one
// -- both of them values the source held.  That is the whole of the class, it is a
// property of the *coding*, and no constraint, no handshake and no synchroniser depth
// substitutes for it.
//
// ---------------------------------------------------------------------------
// Why this is a half and not a whole
// ---------------------------------------------------------------------------
//
// `d64_cdc_toggle_sync` owns both ends of its crossing, because a single bit has
// nothing to hold and nothing to code: whatever the destination samples is legal.  A
// bus is different in kind.  Its safety is a property of the sender and the receiver
// *jointly*, so the word that crosses has to be a net somebody can point at -- to
// constrain, to probe, and to name in a table.  Splitting the two domains into two
// modules is what makes `x_gray` that net, and it is also what keeps every module here
// single-clock, which is the property a static crossing check can actually read.
//
// The encode is combinational off the caller's binary register, exactly as
// `d64_async_fifo`'s `wgray`/`rgray` are derived from their binary pointers.  This half
// owns no state and therefore has no reset.
module d64_cdc_gray_src #(
    parameter W = 6
) (
    input  wire         src_clk,     // declared for the reader; see below
    input  wire [W-1:0] src_bin,     // source-domain binary counter, caller-owned
    output wire [W-1:0] x_gray       // the crossing: at most one bit moves per step
);
    // `src_clk` is taken and unused.  The encode belongs to that domain and a reader of
    // an instantiation should be able to see which clock `src_bin` is registered on
    // without going to find the caller.
    wire unused_src_clk = src_clk;

    assign x_gray = (src_bin >> 1) ^ src_bin;

endmodule
