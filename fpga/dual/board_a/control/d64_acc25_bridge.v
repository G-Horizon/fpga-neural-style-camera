`timescale 1ns/1ps

// F3 6.15 -- the ACC25 boundary: **the seam is 32 bits and the bank is 25.**
//
// `docs/F3_INTERFACES_2026-10-07.md` section 1 fixes the seam at `PE_LANES`
// 128 lanes of `PE_ACC_W` 32 bits, because the seam is the network's and every
// owner meets the same bank.  `board/ddr3/d64_encoder_trunk_board_top.v`
// builds the bank at **25** -- the accumulator bound
// `docs/RESOURCE_REFACTOR_STUDY_2026-10-03.md` section 1 measured -- so the two
// widths have to meet somewhere, and this module is that place.
//
// Where it goes, and why it is not inside anything
// ------------------------------------------------
//
// Between the **arbiter** and the **bank**, and not inside either:
//
//     owner  --  seam, 32-bit lanes  --  arbiter  --  this  --  bank, 25-bit lanes
//
// `d64_pe_owner`'s ports are the contract's (`ACC_W`, default 32) and its
// `bank_bias` / `bank_acc` / `bank_rq_snap` are the bank's.  Putting the
// narrowing on the *owner* side instead -- an arbiter built at 25, or each
// owner narrowing before its `pe_req_bias_i` slice -- would make one owner's
// presentation differ from another's on a port the contract says is one width,
// which is the F2-a mistake one level down.  Both members of the pair keep
// their own width and exactly one module knows about both.
//
// `d64_encoder_trunk` has the same conversion inside `g_local_pe` for the bank
// it instantiates itself, and that copy stays where it is: at `EXTERNAL_PE = 0`
// this module is not in the design at all and the board's compile list does not
// name it.  The rule is written twice and the *values* are what both copies are
// held to -- this one by `tb_d64_encoder_trunk_joint.v`'s `bias_sign_bad` and
// `acc_extended` witnesses, counted every cycle of a three-frame run.
//
// Both directions, per lane, and both are one line
// ------------------------------------------------
//
//   bias   32 -> 25   **per-lane truncation.**  The bank's `bias` port is
//                     `LANES*BANK_W` wide and lane `l` takes the low `BANK_W`
//                     bits of `seam_bias`'s lane `l`.  The bits dropped are
//                     reported rather than assumed: `bias_sign_bad[l]` is high
//                     when they are not a sign extension of the kept sign bit,
//                     which is the only case in which the truncation *changed*
//                     the number.  Section 1's range study says the board's
//                     image cannot set it; the joint gate counts it over three
//                     frames rather than citing the study.
//   acc    25 -> 32    **per-lane SIGN extension**, for `acc_o` and
//                     `rq_snap_o`.  A lane is a signed accumulator that has
//                     already been requantised against `qmin`/`qmax`, so
//                     padding with zeros turns every negative accumulator into
//                     a large positive one -- the bug `d64_mac128`'s own header
//                     records costing a build.
//
// **The whole-bus version of the first is this project's F2-a mistake in a new
// place, and it is silent.**  A 32-bit-lane bus is not a 32-bit word: taking
// `seam_bias[LANES*BANK_W-1:0]` gives lane 0 the right bits and every other
// lane a slice that straddles two of them, so the lanes below the cut are right
// and the picture is wrong with no error anywhere.  `WHOLE_BUS_BIAS` builds
// exactly that and `ZERO_EXT_ACC` builds the other direction's mistake -- a
// zero extension instead of a sign extension, which is the *same* error made
// semantically.  Both are "verification only, folded away at 0", and
// `tb_d64_encoder_trunk_joint.v` requires each to **fail on data**.
module d64_acc25_bridge #(
    // The bank's multiplier count, the contract's lane width, and the bank's
    // accumulator width.  All three are parameters and not localparams for the
    // reason the seam's are: a wide module meeting a narrow one has to say
    // which side of it each number belongs to.
    parameter LANES  = 128,
    parameter SEAM_W = 32,
    parameter BANK_W = 25,

    // Verification only, folded away at 0.
    parameter integer WHOLE_BUS_BIAS = 0,  // the bus truncated instead of a lane
    parameter integer ZERO_EXT_ACC   = 0   // zero-extended instead of sign
) (
    // ---- the seam side, from `d64_pe_owner`, at the contract's width -------
    input  wire [LANES*SEAM_W-1:0] seam_bias,
    output wire [LANES*SEAM_W-1:0] seam_acc,
    output wire [LANES*SEAM_W-1:0] seam_snap,

    // ---- the bank side, at the bank's width -------------------------------
    output wire [LANES*BANK_W-1:0] bank_bias,
    input  wire [LANES*BANK_W-1:0] bank_acc,
    input  wire [LANES*BANK_W-1:0] bank_snap,

    // ---- witnesses --------------------------------------------------------
    //
    // Three readings and not three signals: what they are for is a count taken
    // over a whole run, and a run in which all three stay zero is a run in
    // which this boundary could have been a wire and nobody would know.
    //
    //   bias_trunc_live  the dropped bits were not all zero, so the truncation
    //                    removed something.  Must be non-zero somewhere, or the
    //                    narrowing is not being exercised.
    //   bias_sign_bad    the dropped bits were not a sign extension of the kept
    //                    sign bit -- **the truncation changed the bias**, which
    //                    is a wrong number reaching the array.  Must stay zero.
    //   acc_extended     the accumulator's own sign bit was 1, so the extension
    //                    wrote ones above it.  A zero count means no negative
    //                    accumulator was ever extended, and the sign-extension
    //                    tooth then has nothing to bite.
    output wire [LANES-1:0] bias_trunc_live,
    output wire [LANES-1:0] bias_sign_bad,
    output wire [LANES-1:0] acc_extended,

    // Two more, and they are the difference between "the tooth did not fire"
    // and "the tooth was not built".  Both are computed from the *data* and
    // from nothing else, so they read the same in either build:
    //
    //   bias_whole_bus_diff  on this lane, the whole-bus slice and the per-lane
    //                        slice disagree.  **A count of zero over a run is
    //                        the measurement that says the F2-a mistake is
    //                        invisible on that data** -- the two slices coincide
    //                        when adjacent lanes' dropped bits agree, and for a
    //                        bus of sign-extended biases that is a property of
    //                        the picture and not of the wiring.
    //   acc_widen_diff       the seam's accumulator lane differs from the
    //                        per-lane sign extension of the bank's.  Zero in
    //                        the build that sign-extends, non-zero under
    //                        `ZERO_EXT_ACC` -- and it also says whether
    //                        anything consumes the value, which is a separate
    //                        question the gate answers by grepping.
    output wire [LANES-1:0] bias_whole_bus_diff,
    output wire [LANES-1:0] acc_widen_diff
);

    genvar l;

    generate
    if (BANK_W >= SEAM_W) begin : g_wide
        // Not built today: the bank is at least as wide as the seam.  Written
        // so that `BANK_W == SEAM_W` -- which every unit harness's `ACC_W` 32
        // gives -- is a plain assignment and not a zero-width replication,
        // which Verilog does not allow.  The lane-by-lane form is kept so that
        // the two arms differ in widths and not in shape.
        for (l = 0; l < LANES; l = l + 1) begin : g_lane
            assign bank_bias[l*BANK_W +: BANK_W] = seam_bias[l*SEAM_W +: SEAM_W];
            assign seam_acc[l*SEAM_W +: SEAM_W] = bank_acc[l*BANK_W +: BANK_W];
            assign seam_snap[l*SEAM_W +: SEAM_W] = bank_snap[l*BANK_W +: BANK_W];
        end
        assign bias_trunc_live = {LANES{1'b0}};
        assign bias_sign_bad   = {LANES{1'b0}};
        assign acc_extended    = {LANES{1'b0}};
        assign bias_whole_bus_diff = {LANES{1'b0}};
        assign acc_widen_diff      = {LANES{1'b0}};
    end
    else begin : g_narrow
        for (l = 0; l < LANES; l = l + 1) begin : g_lane
            // ---- 32 -> 25, per lane --------------------------------------
            if (WHOLE_BUS_BIAS != 0) begin : g_whole_bus_bias
                // The F2-a mistake: the *bus* truncated instead of each lane.
                // This is `seam_bias[LANES*BANK_W-1:0]` written per lane, and
                // it is why the mistake is silent -- lane 0 is right, and every
                // lane above it reads bits that straddle its neighbours.
                assign bank_bias[l*BANK_W +: BANK_W] =
                    seam_bias[l*BANK_W +: BANK_W];
            end
            else begin : g_lane_bias
                assign bank_bias[l*BANK_W +: BANK_W] =
                    seam_bias[l*SEAM_W +: BANK_W];
            end
            assign bias_trunc_live[l] =
                (seam_bias[l*SEAM_W+BANK_W +: (SEAM_W-BANK_W)] !==
                 {(SEAM_W-BANK_W){1'b0}});
            // A legal narrowing is one the *number* survived: the dropped bits
            // are all copies of the sign bit that is kept.  Anything else moved
            // the value, and the array then multiplies by a bias the image
            // never asked for.
            assign bias_sign_bad[l] =
                (seam_bias[l*SEAM_W+BANK_W +: (SEAM_W-BANK_W)] !==
                 {(SEAM_W-BANK_W){seam_bias[l*SEAM_W+BANK_W-1]}});

            // ---- 25 -> 32, per lane, SIGN ---------------------------------
            if (ZERO_EXT_ACC != 0) begin : g_zero_ext
                // Zero-extended: every negative accumulator becomes a large
                // positive one, the requant clamps it, and nothing downstream
                // stops -- a wrong picture and not an error.
                assign seam_acc[l*SEAM_W +: SEAM_W] =
                    {{(SEAM_W-BANK_W){1'b0}}, bank_acc[l*BANK_W +: BANK_W]};
                assign seam_snap[l*SEAM_W +: SEAM_W] =
                    {{(SEAM_W-BANK_W){1'b0}}, bank_snap[l*BANK_W +: BANK_W]};
            end
            else begin : g_sign_ext
                assign seam_acc[l*SEAM_W +: SEAM_W] =
                    {{(SEAM_W-BANK_W){bank_acc[l*BANK_W+BANK_W-1]}},
                     bank_acc[l*BANK_W +: BANK_W]};
                assign seam_snap[l*SEAM_W +: SEAM_W] =
                    {{(SEAM_W-BANK_W){bank_snap[l*BANK_W+BANK_W-1]}},
                     bank_snap[l*BANK_W +: BANK_W]};
            end
            // The sign bit itself.  A count of zero over a run says no negative
            // accumulator ever crossed this boundary, and then `ZERO_EXT_ACC`
            // would be a sabotage of a path nothing takes.
            assign acc_extended[l] = bank_acc[l*BANK_W+BANK_W-1];

            // The two data readings.  Neither depends on a sabotage parameter:
            // one compares the two ways of *narrowing* on the values actually
            // on the bus, the other compares the one thing `seam_acc` is
            // supposed to be against what it is.
            assign bias_whole_bus_diff[l] =
                (seam_bias[l*BANK_W +: BANK_W] !==
                 seam_bias[l*SEAM_W +: BANK_W]);
            assign acc_widen_diff[l] =
                (seam_acc[l*SEAM_W +: SEAM_W] !==
                 {{(SEAM_W-BANK_W){bank_acc[l*BANK_W+BANK_W-1]}},
                  bank_acc[l*BANK_W +: BANK_W]});
        end
    end
    endgenerate

endmodule
