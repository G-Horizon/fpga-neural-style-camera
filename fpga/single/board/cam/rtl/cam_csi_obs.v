// What the D-PHY is doing, read from the one domain that is always alive.
//
// The pixel-domain monitor (`cam_raw_monitor`) answers "what arrived".  It cannot
// answer "is anything happening at all", because it is clocked by `clk_pix`, and
// `clk_pix` only runs once the link is up.  When the link is *not* up the monitor
// is not merely silent -- it is stopped, so its numbers are frozen rather than
// zero, and a frozen zero and a real zero read the same in a capture.
//
// So everything here is sampled on `clk_25m`, the board's own oscillator, which
// runs from power-up whatever the camera does.  That is what makes this readable
// during exactly the failure it exists to diagnose.
//
// **`state[31:0]`, and the layout is the contract:**
//
//   [31]     arst_n          PLL POR complete
//   [30]     rst_25_n
//   [29]     rst_pix_n
//   [28]     byte_clk_seen   sticky: the recovered byte clock has ticked
//   [27]     pix_clk_seen    sticky: `clk_pix` has ticked.  A control: it comes
//                            from `pll_inst4`, which is referenced to the board
//                            oscillator, so it should be 1 on any working board.
//                            A 0 here means nothing else in the design means
//                            anything -- and see the detector note below, because
//                            the first version of this bit read 0 while the pixel
//                            clock was demonstrably running.
//   [26]     ck_ena_seen     sticky: clock lane HS enable asserted
//   [25]     ck_term_seen    sticky: clock lane termination asserted
//   [24]     d_ena_seen      sticky: any data lane HS enable asserted
//   [23:20]  cam_HS_ENA_w    live, per lane
//   [19:16]  cam_HS_TERM_w   live, per lane
//   [15:12]  cam_FIFO_EMPTY_w  live
//   [11]     vsync_seen      sticky
//   [10]     hsync_seen      sticky
//   [9]      irq             the IP's only error output, sampled as a level
//   [8]      ck_term         live
//   [7:0]    the data lanes' LP pair, {d3_P,d3_N, d2_P,d2_N, d1_P,d1_N, d0_P,d0_N}
//
// **The LP bits are here because "no data lane enable" has two very different
// causes and the enables cannot tell them apart.**  A D-PHY data lane idles at
// LP-11 and leaves it only when the sensor starts a transmission; so LP-11 on
// all four says the sensor never sent a frame, while anything else says it did
// and the receiver did not follow.  Without them a dead link reads the same
// whether the camera is silent or the wiring is wrong.
//
// **Sticky means "ever", and it is the point of the module.**  A live level
// sampled once says nothing about a signal that is high for microseconds; "was
// it ever high" is exactly the question, and it survives the moment passing.
//
// **The clock detectors divide before they toggle.**  The obvious version -- a
// flop that inverts on every edge of the clock being watched, synchronised into
// `clk_25m` and edge-detected -- does not measure the clock; it measures the
// phase between the two.  `clk_pix` is 50 MHz and this domain samples at 25, so
// every sample lands on the same phase of the toggle, the synchroniser sees a
// constant, and the bit reads **0 on a clock that is running**.  That was
// measured on the board: `pix_clk_seen` was 0 in a capture whose telemetry was
// arriving, which is impossible -- the snapshot handshake that feeds the
// telemetry is clocked by exactly that clock.  Dividing by 1024 in the watched
// domain first puts the toggle two decades below Nyquist, where "seen" means
// seen.
module cam_csi_obs (
    input  wire        clk_25m,
    input  wire        rst_25_n,

    // Levels and stickies that come from the always-on side.
    input  wire        arst_n,
    input  wire        rst_pix_n,
    input  wire        irq,

    // The two clocks whose *existence* is the question.
    input  wire        byte_clk,      // i_cam_ck_CLKOUT, the recovered clock lane
    input  wire        pix_clk,       // clk_pix

    // MIPI levels, all from the receiver's own outputs.
    input  wire        ck_ena,
    input  wire        ck_term,
    input  wire [3:0]  d_ena,
    input  wire [3:0]  d_term,
    input  wire [3:0]  fifo_empty,
    // Data-lane LP pairs, wired {d3_P,d3_N, d2_P,d2_N, d1_P,d1_N, d0_P,d0_N}.
    // LP-11 (both high) is the idle state; a lane leaves it only to transmit.
    input  wire [7:0]  d_lp,

    // Holds and markers, from the pixel domain.  Levels, not pulses.
    input  wire        vsync,
    input  wire        hsync,

    output wire [31:0] state
);

    // ---- clock presence -------------------------------------------------
    // A 10-bit divider in each watched domain, toggling on its carry.  The
    // toggle flop is deliberately not reset: a reset on a flop in a domain whose
    // clock never runs is a flop that never leaves reset, which is the whole
    // failure this module is here to see.
    reg [9:0] bdiv, pdiv;
    reg       byte_tog, pix_tog;
    always @(posedge byte_clk) begin
        bdiv <= bdiv + 10'd1;
        if (&bdiv) byte_tog <= ~byte_tog;
    end
    always @(posedge pix_clk) begin
        pdiv <= pdiv + 10'd1;
        if (&pdiv) pix_tog <= ~pix_tog;
    end

    reg [1:0] bs, ps;
    always @(posedge clk_25m or negedge rst_25_n)
        if (!rst_25_n) begin bs <= 2'd0; ps <= 2'd0; end
        else           begin bs <= {bs[0], byte_tog}; ps <= {ps[0], pix_tog}; end

    // ---- the stickies ---------------------------------------------------
    reg byte_seen, pix_seen, ck_ena_seen, ck_term_seen, d_ena_seen;
    reg vs_seen, hs_seen;

    // Levels from the pixel domain are sampled after two flops.  `vsync` is high
    // for a whole frame's worth of lines and `hsync` for a line's worth of
    // pixels, so at 25 MHz both are caught many times over while a frame is
    // running -- this is not a pulse catcher and does not claim to be one.
    reg [1:0] vs_s, hs_s;
    always @(posedge clk_25m or negedge rst_25_n)
        if (!rst_25_n) begin vs_s <= 2'd0; hs_s <= 2'd0; end
        else           begin vs_s <= {vs_s[0], vsync}; hs_s <= {hs_s[0], hsync}; end

    always @(posedge clk_25m or negedge rst_25_n) begin
        if (!rst_25_n) begin
            byte_seen <= 1'b0; pix_seen <= 1'b0;
            ck_ena_seen <= 1'b0; ck_term_seen <= 1'b0; d_ena_seen <= 1'b0;
            vs_seen <= 1'b0; hs_seen <= 1'b0;
        end
        else begin
            if (bs[1] ^ bs[0]) byte_seen <= 1'b1;
            if (ps[1] ^ ps[0]) pix_seen  <= 1'b1;
            if (ck_ena)  ck_ena_seen  <= 1'b1;
            if (ck_term) ck_term_seen <= 1'b1;
            if (|d_ena)  d_ena_seen   <= 1'b1;
            if (vs_s[1]) vs_seen <= 1'b1;
            if (hs_s[1]) hs_seen <= 1'b1;
        end
    end

    assign state = {arst_n, rst_25_n, rst_pix_n,
                    byte_seen, pix_seen,
                    ck_ena_seen, ck_term_seen, d_ena_seen,
                    d_ena, d_term, fifo_empty,
                    vs_seen, hs_seen, irq, ck_term,
                    d_lp};

endmodule
