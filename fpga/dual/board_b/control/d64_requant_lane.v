`timescale 1ns/1ps

// D3c4c-a -- one requantisation lane, the exact D2 contract.
//
//     y = clamp( ((acc + bias) * M + (1 << (shift-1))) >> shift + zero )
//
// with an **arithmetic** shift, so it floors; with the half added first, so ties
// go up.  That is not round-half-away-from-zero, and the difference lands on
// exactly the values that make a golden diff look like a bug.
//
// One shifter, not two
// --------------------
//
// The obvious form has two variable shifts in it: one to build `1 << (shift-1)`
// and one to do the shift down.  They collapse.  For a two's-complement `p` and
// a shift `s > 0`,
//
//     (p + 2**(s-1)) >> s   ==   (p >> s) + bit_{s-1}(p)
//
// because the arithmetic shift is floor division, so the left side is
// `floor(p/2**s + 1/2)` and the addend is exactly the round-up carry.  So the
// rounding constant is a *bit select* out of the product rather than a shifted
// constant, and `s = 0` needs the bit forced to zero to agree with the
// reference's `half = 0 when shift = 0`.
//
// The testbench checks the collapsed form against the literal one, because a
// simplification that is argued and not measured is how this project has been
// wrong before.
//
// M is unsigned, and the output is unsigned
// -----------------------------------------
//
// `M` is `round(R * 2**shift)` with `R` a positive ratio, so it always lands in
// `[2**(M_W-1), 2**M_W)`; it is zero-extended to `M_W+1` signed rather than
// driven as a signed operand, because a signed multiplier infers a different
// primitive than the design has.
//
// `y` is unsigned because both layers here have `operand_signed = false` with
// `qmin = 0`.  QMIN/QMAX are plain integers and the comparison happens at the
// full product width, so a format whose `qmax` does not fit the output width
// sign-extends correctly instead of wrapping -- at `OUT_W = 8` signed, 255 is
// not representable and a signed clamp would saturate at 127.
//
// The zero point is NOT a signed OUT_W -- it is a non-negative U10
// ----------------------------------------------------------------
//
// `zero` used to be `signed [OUT_W-1:0]`, and that was a live defect, not a
// corner.  A quantisation zero point is a **non-negative** integer: the four
// shipped styles carry 413 (candy), 497 (vangogh), 402 (inkwash) and 512
// (ukiyoe) in a 10-bit field.  Read as two's complement, 512 is -512 and 1023
// is -1, so the lane produced a value below `QMIN` and clamped to the low
// bound -- ukiyoe's zero came out 0 where the contract says 512.  Only the
// zeros at or above 512 were wrong, which is why candy, vangogh and inkwash
// never showed it and why it survived until a fourth style was calibrated.
//
// The port is now `ZERO_W` bits wide and **unsigned by default**, and the value
// is *positively* (zero-) extended into the signed wide arithmetic before it is
// added.  `ZERO_W` defaults to `OUT_W`, so every existing instantiation keeps
// its port shape and its numbers.  `ZERO_SIGNED = 1` restores the
// two's-complement reading for a format that genuinely needs a negative zero;
// **it is not the default and nothing in this design uses it.**
//
// Two ways NOT to repair this, both of which fix one style by breaking the
// others: substituting 512 for 413 (a constant, not a semantics), and folding
// the extension into the right-shift expression as one concatenation (`>>>` in
// an unsigned context degrades to a logical shift -- see the note above).
module d64_requant_lane #(
    parameter ACC_W   = 32,
    parameter M_W     = 16,
    parameter SHIFT_W = 5,
    parameter OUT_W   = 8,
    parameter integer QMIN = 0,
    parameter integer QMAX = 255,
    // The zero point's own width and semantics, parameterised **separately**
    // from `OUT_W` so that the default -- a non-negative, `OUT_W`-wide zero --
    // does not move.  `ZERO_W` defaults to `OUT_W`, so every existing
    // instantiation keeps its port shape and its numbers.
    parameter ZERO_W = OUT_W,
    parameter integer ZERO_SIGNED = 0
) (
    input  wire signed [ACC_W-1:0]   acc,
    input  wire signed [ACC_W-1:0]   bias,
    input  wire        [M_W-1:0]     m,
    input  wire        [SHIFT_W-1:0] shift,
    input  wire        [ZERO_W-1:0]  zero,
    output wire        [OUT_W-1:0]   y
);

    localparam PW = ACC_W + M_W + 1;

    // Part-selects need a vector, and an `integer` parameter is one only by
    // convention; naming them makes the clamp read as the four constants it is.
    localparam [OUT_W-1:0] QMIN_V = QMIN;
    localparam [OUT_W-1:0] QMAX_V = QMAX;

    // The bias joins the accumulator here rather than in the MAC array.  The
    // frozen digest is over the *bias-included* accumulator, so the array
    // preloads the bias into the accumulator instead and drives this port as
    // zero; the port exists so the lane can be verified on its own.
    wire signed [ACC_W:0] sum = acc + bias;

    wire signed [M_W:0]   ms  = {1'b0, m};
    wire signed [PW-1:0]  prod = sum * ms;

    // bit_{shift-1}(prod), and 0 when shift is 0.  Indexing with `shift-1`
    // directly is index -1 at shift = 0, which synthesises to something and
    // simulates to x; clamping the index and masking is the safe form.
    wire [SHIFT_W-1:0]    shm1 = (shift == 0) ? {SHIFT_W{1'b0}} : (shift - 1);
    wire [PW-1:0]         pbits = prod;
    wire                  round_up = pbits[shm1] & (shift != 0);

    // **The shift is separated from the rounding add on purpose.**  Verilog makes
    // a whole expression unsigned if ANY of its operands is unsigned, and a
    // concatenation always is.  Writing `(prod >>> shift) + {.., round_up}` as
    // one expression therefore makes it unsigned, and with the context unsigned
    // the `>>>` degrades to a logical shift: a negative product becomes a large
    // positive quotient, and a channel that must clamp to zero comes out 255.
    // The product itself was right the whole time, which is why the only symptom
    // was saturation in one direction.
    wire signed [PW-1:0]  shifted = prod >>> shift;
    wire signed [PW-1:0]  half_v  = {{(PW-1){1'b0}}, round_up};
    wire signed [PW-1:0]  q = shifted + half_v;

    // The zero point, **positively** extended into the signed wide arithmetic.
    //
    // This is the fix for the U10 zero defect described in the header.  The
    // filler bits are 0 for the default (non-negative) semantics -- never a copy
    // of the zero's top bit.  `ZERO_SIGNED = 1` is the two's-complement reading
    // and exists only for a format that needs a negative zero.
    //
    // `PW > ZERO_W` at every instantiation in this design (`PW >= ACC_W+M_W+1
    // >= 17`, `ZERO_W <= OUT_W`); the `: 1` is only so the replication is never
    // zero-width, which this project forbids.  Both branches are `PW` bits wide
    // and both are declared signed, so the add below stays signed.
    localparam ZEXT_W = (PW > ZERO_W) ? (PW - ZERO_W) : 1;
    wire signed [PW-1:0] zero_pos = {{ZEXT_W{1'b0}},           zero};
    wire signed [PW-1:0] zero_neg = {{ZEXT_W{zero[ZERO_W-1]}}, zero};
    wire signed [PW-1:0] zero_ext = ZERO_SIGNED ? zero_neg : zero_pos;

    // The zero point and then the saturation, in that order.  On a ReLU tensor
    // whose zero point is 0 these are the same clamp, which is why a ReLU layer
    // needs one comparison and not two.
    wire signed [PW-1:0]  biased = q + zero_ext;

    assign y = (biased < QMIN) ? QMIN_V :
               (biased > QMAX) ? QMAX_V : biased[OUT_W-1:0];

endmodule
