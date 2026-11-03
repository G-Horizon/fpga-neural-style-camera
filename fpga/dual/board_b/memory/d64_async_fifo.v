// A dual-clock FIFO for the H0 DDR staging path.
//
// The staging FIFOs sit between two clock domains that have nothing to do with each
// other: the PE and its rings at 120 MHz on one side, the AXI port at 100 MHz on the
// other.  Both directions cross -- conv3's output goes out through the write FIFO, and
// the X ring's input arrives through the read FIFO -- so both are this module.
//
// The measurement root that sized them (`synth/d64_ddr_fifo_synth_top.v`) is deliberately
// single-clock: it answers what the *storage* costs in RAM10 and says so.  It is not a CDC
// and must not be read as one.  This is the CDC.
//
// Four things make it one, and only the first two are the crossing itself:
//
//   1. Gray-coded pointers, so a transfer changes exactly one bit and a synchroniser can
//      never sample a value that was never written
//   2. `async_reg` on the synchroniser flops, so synthesis does not merge or duplicate
//      them and place-and-route can pull the two stages together for metastability MTBF.
//      This is a different instruction from the timing constraint: `set_clock_groups`
//      tells static timing not to analyse the crossing, `async_reg` tells the physical
//      tools what the flops are for.  Neither substitutes for the other.
//   3. Each domain's reset is asserted asynchronously and released synchronously to its
//      own clock, so a release cannot arrive in one domain a cycle before the other and
//      walk the pointers apart.
//   4. `rd_valid` says when the read data is there.  Without it a consumer has to infer
//      validity from `rd_empty`, and the last word of a burst arrives on the cycle
//      `rd_empty` asserts -- so the obvious `if (!rd_empty) consume(rd_data)` drops it.
//
// **Both sides reset together.**  Resetting one domain while the other runs is not
// supported: the pointers would have to be re-agreed across a crossing that is by
// definition not listening, and no amount of synchroniser stages makes that safe.
//
// Depth is a power of two so the Gray arithmetic is exact.
module d64_async_fifo #(
    parameter DW = 128,
    parameter AW = 11,             // 2^AW words
    parameter ALMOST_FULL_LEVEL = (1 << AW) - 2
) (
    input  wire            wr_clk,
    input  wire            wr_rst_n,
    input  wire            wr_en,
    input  wire [DW-1:0]   wr_data,
    output wire            wr_full,
    output wire            wr_almost_full,
    output wire [AW:0]     wr_level,

    input  wire            rd_clk,
    input  wire            rd_rst_n,
    input  wire            rd_en,
    output wire [DW-1:0]   rd_data,
    output wire            rd_valid,
    output wire            rd_empty,
    output wire [AW:0]     rd_level
);

    localparam DEPTH = 1 << AW;

    // The reset inputs are the DOMAIN resets, already synchronised to each clock by
    // `d64_reset_sync` at the level that owns the domain.  The module deliberately does not
    // carry its own: a private synchroniser releases this module's reset two clocks after
    // everything around it, which is a latency the caller cannot see and cannot wait for
    // correctly.  See d64_reset_sync.v for what that cost and why the fix is to remove the
    // hidden state rather than to wait longer.
    wire wr_rst_local_n = wr_rst_n;
    wire rd_rst_local_n = rd_rst_n;

    (* ram_style = "block" *) reg [DW-1:0] mem [0:DEPTH-1];

    reg  [AW:0] wbin, wgray;
    reg  [AW:0] rbin, rgray;

    function [AW:0] gray_to_binary;
        input [AW:0] gray;
        integer gi;
        begin
            gray_to_binary[AW] = gray[AW];
            for (gi = AW-1; gi >= 0; gi = gi - 1)
                gray_to_binary[gi] = gray_to_binary[gi+1] ^ gray[gi];
        end
    endfunction

    // ---- write domain ------------------------------------------------------------
    // The full flag is REGISTERED.  Written combinationally it feeds `wbin_next`, which
    // feeds `wgray_next`, which feeds the flag again -- a loop with no clock in it.
    reg          wr_full_q;
    (* async_reg = "true" *) reg [AW:0] rgray_s1, rgray_s2;
    wire [AW:0]  wbin_next  = wbin + (wr_en & ~wr_full_q);
    wire [AW:0]  wgray_next = (wbin_next >> 1) ^ wbin_next;
    wire [AW:0]  wfull_next = (wgray_next ==
                               {~rgray_s2[AW:AW-1], rgray_s2[AW-2:0]});

    always @(posedge wr_clk or negedge wr_rst_local_n) begin
        if (!wr_rst_local_n) begin
            wbin <= 0; wgray <= 0; wr_full_q <= 1'b0;
            rgray_s1 <= 0; rgray_s2 <= 0;
        end
        else begin
            if (wr_en && !wr_full_q)
                mem[wbin[AW-1:0]] <= wr_data;
            wbin      <= wbin_next;
            wgray     <= wgray_next;
            wr_full_q <= wfull_next;
            rgray_s1  <= rgray;
            rgray_s2  <= rgray_s1;
        end
    end

    assign wr_full = wr_full_q;
    // The synchronized remote pointer intentionally makes these conservative
    // occupancy estimates. The writer can use almost-full to reserve space for
    // an in-flight packed beat without feeding a CDC pointer into ready directly.
    assign wr_level = wbin - gray_to_binary(rgray_s2);
    assign wr_almost_full = (wr_level >= ALMOST_FULL_LEVEL);

    // ---- read domain -------------------------------------------------------------
    reg          rd_empty_q;
    (* async_reg = "true" *) reg [AW:0] wgray_s1, wgray_s2;
    wire [AW:0]  rbin_next  = rbin + (rd_en & ~rd_empty_q);
    wire [AW:0]  rgray_next = (rbin_next >> 1) ^ rbin_next;

    always @(posedge rd_clk or negedge rd_rst_local_n) begin
        if (!rd_rst_local_n) begin
            rbin <= 0; rgray <= 0; rd_empty_q <= 1'b1;
            wgray_s1 <= 0; wgray_s2 <= 0;
        end
        else begin
            rbin       <= rbin_next;
            rgray      <= rgray_next;
            rd_empty_q <= (rgray_next == wgray_s2);
            wgray_s1   <= wgray;
            wgray_s2   <= wgray_s1;
        end
    end

    // The block RAM's read is registered, so the data for an accepted read appears the
    // next cycle.  `rd_valid` is that same cycle, and it is the only thing a consumer
    // should gate on -- `rd_empty` deasserting is not the same signal, and the last word
    // of a burst arrives on the cycle `rd_empty` asserts.
    reg [DW-1:0] rd_q;
    reg          rd_valid_q;

    always @(posedge rd_clk or negedge rd_rst_local_n) begin
        if (!rd_rst_local_n) begin
            rd_q      <= {DW{1'b0}};
            rd_valid_q <= 1'b0;
        end
        else begin
            if (rd_en && !rd_empty_q)
                rd_q <= mem[rbin[AW-1:0]];
            rd_valid_q <= rd_en && !rd_empty_q;
        end
    end

    assign rd_data  = rd_q;
    assign rd_valid = rd_valid_q;
    assign rd_empty = rd_empty_q;
    assign rd_level = gray_to_binary(wgray_s2) - rbin;

endmodule
