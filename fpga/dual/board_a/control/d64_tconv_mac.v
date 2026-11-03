`timescale 1ns/1ps

// D3c4c-a -- the real 128-MAC transposed-convolution datapath.
//
// This is the arithmetic half of D3c4c.  `control/d64_tconv_scheduler.v` already
// proved the memory schedule -- where every input read, weight read and output
// write goes on every cycle, 147,456 cycles a layer, with the MAC and the
// requant left as a token pipeline because the question there was memory.  This
// replaces the token with the arithmetic and leaves the addressing exactly where
// it is.
//
// The shape is fixed by the two layers rather than chosen
// ------------------------------------------------------
//
//             in          out        PIX  IC  OC   ic-groups  slots/pixel
//   ups.0   64x64x32   128x128x16    1    8  16       4         9*4 = 36
//   ups.1  128x128x16  256x256x8     2    8   8       2         9*2 = 18
//
// so the array is `PIX_PAR x IC_PAR x OC_LANES = 128` multipliers in both
// layers, and what changes between them is only how those 128 are partitioned:
// one pixel by sixteen output channels, or two pixels by eight.  Hence the
// accumulator count is `PIX_PAR*OC_LANES` and not a layer-specific number.
//
// The accumulator includes the bias, because the frozen digest does
// ---------------------------------------------------------------
//
// `golden/d64_int_taps.json` records its accumulator sha256 over `tconv_int`'s
// return, which is `acc + b_q`.  So the bias is *preloaded* into the array at
// the start of a pixel rather than added at requant time.  That costs nothing --
// it is a load either way -- and it makes the number this module exposes
// bit-identical to the frozen one, so the gate compares one quantity with
// itself instead of two different quantities that happen to be close.
//
// Slot order
// ----------
//
// A slot is one `(tap, ic_group)`: `PIX_PAR*IC_PAR` activations and
// `IC_PAR*OC_LANES` weights of one `(kh, kw, ic_group)`.  The caller's tap order
// is free -- integer addition does not care.  What is not free is which
// `(ih, iw)` a tap reads, and that belongs to the caller.  This module contains
// no address arithmetic at all, which is the point: the one place a transposed
// convolution goes wrong is the tap set, and a datapath that cannot form an
// address cannot get one wrong.
//
// Why the requant is pipelined and the MAC is not
// -----------------------------------------------
//
// The MAC's operands arrive on wires from a memory, so there is nothing to
// select and the multiply starts immediately.  The requant's do not: one lane
// serves `OC_LANES/RQ_LANES` different output channels, so its accumulator,
// its `M` and its `shift` all come through a mux.  Putting that mux in front of
// a 25x16 multiplier is a long path, and it is the same shape as the one that
// made the statistics read the critical path in the earlier unit.  So the
// operands are registered first and the lane sits after the register; a pass
// costs two cycles and the total is `RQ_CYCLES + 1`.
module d64_tconv_mac #(
    parameter IC_PAR    = 8,
    parameter OC_LANES  = 16,
    parameter PIX_PAR   = 1,          // 1 for ups.0, 2 for ups.1
    parameter ACT_W     = 8,          // one activation operand, signed
    parameter WGT_W     = 8,
    parameter ACC_W     = 32,
    parameter M_W       = 16,
    parameter SHIFT_W   = 5,
    parameter OUT_W     = 8,
    parameter RQ_LANES  = 8,          // requant units; OC_LANES/RQ_LANES passes
    parameter integer QMIN = 0,
    parameter integer QMAX = 255,
    // Verification only; the expression folds away at 0, which is the default
    // and the only value that is ever built.  At 1 the later requant passes read
    // the live accumulator instead of the snapshot, which is the design this
    // module had before the snapshot existed -- and the reason the parameter
    // exists is that the mistake is invisible unless a phase is requantised
    // while the MAC is already running the next one.  `tools/run_tconv_mac.sh`
    // builds it and requires the overlapped mode to fail while the sequential
    // mode still passes; a tooth that does not fire is not a tooth.
    parameter integer SNAP_SABOTAGE = 0,
    // Verification only, folded away at 0.  At 1 `done_acc` omits the final
    // slot, which is precisely what the priority `if (pix_start) ... else if
    // (slot_valid)` did on its own -- so the rollover edge silently drops the
    // last MAC contribution instead of folding it in.
    parameter integer ROLLOVER_SABOTAGE = 0
) (
    input  wire                          clk,
    input  wire                          rst_n,

    // ---- the MAC slot ----------------------------------------------------
    input  wire                          slot_valid,
    input  wire [PIX_PAR*IC_PAR*ACT_W-1:0]   act,
    input  wire [IC_PAR*OC_LANES*WGT_W-1:0]  wgt,

    // ---- pixel framing ---------------------------------------------------
    // `pix_start` loads the bias into every accumulator, so the array is ready
    // for the first slot on the next cycle and the exposed accumulator is the
    // bias-included one the frozen digest covers.
    input  wire                          pix_start,
    input  wire [PIX_PAR*OC_LANES*ACC_W-1:0] bias,
    input  wire                          rq_start,
    output wire                          rq_ready,
    // Whether the requant will be ready to accept K cycles from now, assuming
    // nothing is accepted in between.  The engine needs this and not `rq_ready`:
    // the scheduler's phase-end warning comes two cycles before the slot it
    // warns about reaches this module, so testing the present state freezes the
    // schedule for a conflict that has already passed.  Exact, because `ph`
    // saturates at RQ_CYCLES and `run` only falls when it does.
    output wire                          rq_ready_n,
    input  wire [OC_LANES*M_W-1:0]       rq_m,
    input  wire [OC_LANES*SHIFT_W-1:0]   rq_shift,
    input  wire signed [OUT_W-1:0]       rq_zero,

    output reg  [RQ_LANES*OUT_W-1:0]     y,
    output reg  [RQ_LANES-1:0]           y_valid,
    output reg                           rq_busy,
    output reg                           rq_done,

    // ---- observability ---------------------------------------------------
    // The live accumulator.  On a rollover edge this is already the next
    // phase's bias, so it is not the reference a checker wants.
    output wire [PIX_PAR*OC_LANES*ACC_W-1:0] acc_o,
    // The accumulator the requant actually uses: the completed value of the
    // phase being requantised, latched on the accept edge.  Exposed because a
    // rollover makes `acc_o` unusable as a reference at exactly the moment it
    // matters, which is the moment worth checking.
    output wire [PIX_PAR*OC_LANES*ACC_W-1:0] rq_snap_o,
    // Pulses if a phase is started while a completed accumulator has never been
    // accepted: the final MAC contribution is destroyed and nothing recovers
    // it.  That is a property of the caller rather than of this module, so it is
    // reported and not enforced -- and the caller has to qualify `pix_start`
    // with `rq_accept`, which is what the rollover contract says.
    output reg                               rq_overrun
);

    localparam NPIX = PIX_PAR * OC_LANES;
    localparam PW   = ACT_W + WGT_W;              // one product
    localparam SW   = PW + $clog2(IC_PAR);        // a full IC_PAR-wide sum
    // The requant walks NPIX accumulators, not OC_LANES: at PIX_PAR = 2
    // there are two pixels' worth and only one of them would be requantised.
    // M and shift are per OUTPUT CHANNEL, so they index by idx % OC_LANES.
    localparam RQ_CYCLES = (NPIX + RQ_LANES - 1) / RQ_LANES;
    localparam RQW = $clog2(RQ_CYCLES + 1);

    integer p, st, lv, ri;

    // ------------------------------------------------------------------
    // The multipliers and the reduction tree
    // ------------------------------------------------------------------

    wire signed [ACT_W-1:0] a_s [0:PIX_PAR*IC_PAR-1];
    wire signed [WGT_W-1:0] w_s [0:IC_PAR*OC_LANES-1];

    genvar gi, rl;
    generate
        for (gi = 0; gi < PIX_PAR*IC_PAR; gi = gi + 1)
            assign a_s[gi] = act[gi*ACT_W +: ACT_W];
        for (gi = 0; gi < IC_PAR*OC_LANES; gi = gi + 1)
            assign w_s[gi] = wgt[gi*WGT_W +: WGT_W];
    endgenerate

    // Indexed [pixel][ic][oc] with oc innermost, matching the weight word the
    // generator writes: for one ic, OC_LANES weights in oc order.  One output
    // channel's products are then IC_PAR apart and the reduction reads a stride.
    wire signed [PW-1:0] prod [0:NPIX*IC_PAR-1];
    generate
        for (gi = 0; gi < NPIX*IC_PAR; gi = gi + 1) begin : g_prod
            localparam PP = gi / IC_PAR;          // which (pixel, oc)
            localparam II = gi % IC_PAR;          // which input channel
            assign prod[gi] =
                a_s[(PP/OC_LANES)*IC_PAR + II] * w_s[II*OC_LANES + (PP%OC_LANES)];
        end
    endgenerate

    // Pairwise reduction over IC_PAR, one sum per (pixel, oc).  Written as a
    // heap so the mapper sees a tree and not a chain: eight 16-bit adds in
    // series is several nanoseconds and this has to fit beside a DSP at
    // 120 MHz.  IC_PAR must be a power of two (both layers use 8).
    reg signed [SW-1:0] red [0:NPIX*IC_PAR-1];

    always @* begin
        for (p = 0; p < NPIX; p = p + 1) begin
            for (st = 0; st < IC_PAR; st = st + 1)
                red[p*IC_PAR + st] = prod[p*IC_PAR + st];
            for (lv = IC_PAR/2; lv >= 1; lv = lv >> 1)
                for (st = 0; st < lv; st = st + 1)
                    red[p*IC_PAR + st] = red[p*IC_PAR + 2*st]
                                       + red[p*IC_PAR + 2*st + 1];
        end
    end

    // ------------------------------------------------------------------
    // The accumulators
    // ------------------------------------------------------------------

    reg signed [ACC_W-1:0] acc_q [0:NPIX-1];

    // **The phase rollover.**  `pix_start` has priority over `slot_valid`, so
    // an edge carrying both would drop the phase's final MAC contribution.  The
    // real schedule wants exactly that edge: with a one-cycle RAM read the last
    // slot of phase k lands on the cycle the scheduler is already starting phase
    // k+1, and waiting would cost a cycle per phase -- which at ups.1, where a
    // phase can be two MAC cycles, is not a rounding error.
    //
    // So the completed value is formed once and used three ways on one edge:
    // it is what the snapshot latches, what pass 0 of the requant reads, and
    // what `acc_q` gives up when it takes the new bias.
    reg signed [ACC_W-1:0] done_acc [0:NPIX-1];
    reg signed [ACC_W-1:0] acc_sum  [0:NPIX-1];
    integer d;

    always @* begin
        for (d = 0; d < NPIX; d = d + 1) begin
            // One adder, two readers.  `done_acc` and the accumulator want the
            // same sum on a slot edge, and building it twice cost 0.35 ns of
            // setup slack at 120 MHz -- which is most of the margin.
            acc_sum[d]  = acc_q[d] + red[d*IC_PAR];
            // `slot_valid` alone, not `pix_start && slot_valid`.  A slot on the
            // bus belongs to the phase being completed whether or not the next
            // one is starting on the same edge, and an accept that lands on a
            // slot's edge would otherwise snapshot a value missing that slot
            // while the accumulator kept it -- the phase's contribution split
            // across two accumulators.
            done_acc[d] = (slot_valid && !ROLLOVER_SABOTAGE) ? acc_sum[d] : acc_q[d];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            for (p = 0; p < NPIX; p = p + 1)
                acc_q[p] <= {ACC_W{1'b0}};
        else
            for (p = 0; p < NPIX; p = p + 1)
                if (pix_start)       acc_q[p] <= bias[p*ACC_W +: ACC_W];
                else if (slot_valid) acc_q[p] <= acc_sum[p];
    end

    generate
        for (gi = 0; gi < NPIX; gi = gi + 1)
            assign acc_o[gi*ACC_W +: ACC_W] = acc_q[gi];
    endgenerate

    // ------------------------------------------------------------------
    // The requant sequencer: stage 1 selects and registers the operands
    // ------------------------------------------------------------------

    reg signed [ACC_W-1:0] s_acc   [0:RQ_LANES-1];
    reg        [M_W-1:0]   s_m     [0:RQ_LANES-1];
    reg        [SHIFT_W-1:0] s_shift [0:RQ_LANES-1];
    reg        [RQ_LANES-1:0] s_ok;

    // **The accumulator snapshot.**
    //
    // Without it the requant reads `acc_q` live, and its second pass reads it a
    // cycle or more after the request -- by which time the MAC has loaded the
    // next phase's bias and the second half of the output channels is computed
    // from the wrong numbers.  The standalone testbench never saw this because
    // it ran a pixel to completion, requantised it, and only then started the
    // next one.  A schedule where a new phase begins while the previous one is
    // still being requantised is exactly what the real scheduler does, and for
    // ups.1's EE phase -- one tap, two input groups, two MAC cycles -- it does
    // it every two cycles.
    //
    // Taking the snapshot in the same non-blocking step that loads the new bias
    // means the two are independent and no bubble is needed: `rq_snap` gets the
    // completed accumulator while `acc_q` gets the bias, on one edge.
    reg signed [ACC_W-1:0] rq_snap [0:NPIX-1];

    generate
        for (gi = 0; gi < NPIX; gi = gi + 1)
            assign rq_snap_o[gi*ACC_W +: ACC_W] = rq_snap[gi];
    endgenerate

    // `ph` counts the passes LOADED, 1..RQ_CYCLES.  The lane is emitting pass
    // `ph - 1`, so the run ends the moment the last pass has been loaded rather
    // than one cycle later -- which is what lets a new request be accepted
    // every RQ_CYCLES cycles instead of every RQ_CYCLES + 1.
    reg [RQW-1:0] ph;
    reg           run;

    // A request is accepted only when the sequencer can take it.  Without this
    // the `rq_start` branch would pre-empt an in-flight requant and silently
    // drop a pass -- the failure mode is not a wrong value, it is a missing one.
    assign rq_ready = ~run | (ph == RQ_CYCLES);
    localparam integer RQ_LOOKAHEAD = 3;
    wire [RQW:0] ph_k = ph + RQ_LOOKAHEAD[RQW:0];
    assign rq_ready_n = ~run | (ph_k >= RQ_CYCLES);
    wire   rq_accept = rq_start & rq_ready;

    // **The width is not decoration.**  Verilog evaluates the whole right-hand
    // side at the width of the left-hand side, so a narrow base makes RQ_LANES
    // -- 8 -- truncate to 0, the product is identically zero, and every pass
    // reloads pass 0.  It maps, it simulates, and it looks like a working
    // sequencer emitting the wrong channels.  The same trap as the
    // `{12'b0, idx} << 24` that lost its top bits, so the constant is widened
    // explicitly rather than trusted to the context rules.
    localparam SBW = $clog2(NPIX + RQ_LANES) + 1;
    wire [SBW-1:0] rq_lanes_w = RQ_LANES;
    wire [SBW-1:0] sel_base   = ph * rq_lanes_w;      // the pass being loaded

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph <= {RQW{1'b0}};
            run <= 1'b0;
            s_ok <= {RQ_LANES{1'b0}};
            for (ri = 0; ri < NPIX; ri = ri + 1)
                rq_snap[ri] <= {ACC_W{1'b0}};
            for (ri = 0; ri < RQ_LANES; ri = ri + 1) begin
                s_acc[ri]   <= {ACC_W{1'b0}};
                s_m[ri]     <= {M_W{1'b0}};
                s_shift[ri] <= {SHIFT_W{1'b0}};
            end
        end
        else begin
            if (rq_accept)
                for (ri = 0; ri < NPIX; ri = ri + 1)
                    rq_snap[ri] <= done_acc[ri];
            if (rq_accept) begin
                // Pass 0 reads `done_acc` directly rather than the snapshot:
                // at this edge the snapshot register is still the previous
                // request's value, so it cannot be read, and `done_acc` is the
                // same number -- including the final slot of a rollover edge,
                // which is the whole reason it exists.
                ph  <= {{(RQW-1){1'b0}}, 1'b1};
                run <= 1'b1;
                for (ri = 0; ri < RQ_LANES; ri = ri + 1) begin
                    s_acc[ri]   <= done_acc[ri];
                    s_m[ri]     <= rq_m[(ri % OC_LANES)*M_W +: M_W];
                    s_shift[ri] <= rq_shift[(ri % OC_LANES)*SHIFT_W +: SHIFT_W];
                    s_ok[ri]    <= (ri < NPIX);
                end
            end
            else if (run) begin
                if (ph == RQ_CYCLES) begin
                    run  <= 1'b0;
                    s_ok <= {RQ_LANES{1'b0}};
                end
                else begin
                    // Every later pass comes from the snapshot, which the MAC
                    // cannot move.
                    ph <= ph + 1'b1;
                    for (ri = 0; ri < RQ_LANES; ri = ri + 1) begin
                        if (sel_base + ri < NPIX) begin
                            s_acc[ri]   <= SNAP_SABOTAGE ? acc_q[sel_base + ri]
                                                         : rq_snap[sel_base + ri];
                            s_m[ri]     <= rq_m[((sel_base + ri) % OC_LANES)*M_W +: M_W];
                            s_shift[ri] <= rq_shift[((sel_base + ri) % OC_LANES)*SHIFT_W
                                                    +: SHIFT_W];
                            s_ok[ri]    <= 1'b1;
                        end
                        else
                            s_ok[ri] <= 1'b0;
                    end
                end
            end
        end
    end

    // A phase that has received at least one slot and has not been accepted.
    // `pix_start` destroys it, so `pix_start` without an accept is a lost
    // accumulator -- and the caller has to qualify the two together, which is
    // the rollover contract.
    reg acc_dirty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_dirty  <= 1'b0;
            rq_overrun <= 1'b0;
        end
        else begin
            rq_overrun <= pix_start & acc_dirty & ~rq_accept;
            // The accept has to clear this too.  Clearing it only on
            // `pix_start` leaves every phase after the first looking dirty, and
            // the flag then fires on a schedule that is perfectly correct --
            // which is worse than not having it, because a warning that is
            // always on is a warning nobody reads.
            if (pix_start)        acc_dirty <= 1'b0;
            else if (rq_accept)   acc_dirty <= 1'b0;
            else if (slot_valid)  acc_dirty <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Stage 2: the lanes, whose results are registered straight into `y`
    // ------------------------------------------------------------------

    wire [RQ_LANES*OUT_W-1:0] lane_y;
    wire [RQ_LANES-1:0]       lane_ok;

    generate
        for (rl = 0; rl < RQ_LANES; rl = rl + 1) begin : g_rq
            d64_requant_lane #(.ACC_W(ACC_W), .M_W(M_W), .SHIFT_W(SHIFT_W),
                               .OUT_W(OUT_W), .QMIN(QMIN), .QMAX(QMAX)) u_lane (
                .acc   (s_acc[rl]),
                .bias  ({ACC_W{1'b0}}),   // already in the accumulator
                .m     (s_m[rl]),
                .shift (s_shift[rl]),
                .zero  (rq_zero),
                .y     (lane_y[rl*OUT_W +: OUT_W])
            );
            // The operand register holds pass `ph - 1` and is live exactly while
            // `1 <= ph <= RQ_CYCLES`, which `run` and `s_ok` already bracket.
            assign lane_ok[rl] = s_ok[rl] & run;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y        <= {RQ_LANES*OUT_W{1'b0}};
            y_valid  <= {RQ_LANES{1'b0}};
            rq_busy  <= 1'b0;
            rq_done  <= 1'b0;
        end
        else begin
            y       <= lane_y;
            y_valid <= lane_ok;
            rq_busy <= run;
            rq_done <= run & (ph == RQ_CYCLES);
        end
    end

endmodule
