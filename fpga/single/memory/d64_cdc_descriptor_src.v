// The source half of a bundled-data descriptor crossing.  Source domain only.
//
// ---------------------------------------------------------------------------
// The class: hold the payload still, and say when
// ---------------------------------------------------------------------------
//
// A descriptor is a bus that is not a counter -- it jumps (pixel -> AXI:
// `{bank, frame_id, epoch, row, src_base}`), so Gray coding is not available to it.
// What it has instead is *time*: a display line is thousands of destination clocks,
// and the payload does not need to be sampled at any particular moment, only to be
// sampled while it is still there.
//
// So the payload is held still and a toggle beside it says "now".  The payload goes
// into `x_data` on the same source edge that flips `x_req_tog`, and **nothing in this
// module assigns `x_data` again until the acknowledge has come back from the
// destination domain** (`x_ack_tog`, synchronised by the destination half and
// returned).  Sampling a bus that is not moving cannot produce a word that was never
// written, whatever the skew between its bits: skew only matters when a bit is
// changing, and none of them are.
//
// That is the whole of the class.  **Two flops on the bus is not a cheaper version of
// it, it is a different thing**: two flops remove metastability from one signal and
// say nothing about a bus in motion, so a destination that samples a free-running
// descriptor will eventually capture a mix of the old and the new one -- which is what
// the tooth in `tb_cdc_descriptor.v` demonstrates, with a skew of 2.4 ns, inside the
// 2500 ps Efinity's own synchroniser analysis bounds.
//
// ---------------------------------------------------------------------------
// What this half owes, and what it is owed
// ---------------------------------------------------------------------------
//
// * `src_valid` is sampled, not edge-detected, and ignored while `src_busy`.  A pulse
//   is fine; a level is a request every time the previous one is acknowledged.
// * `x_data` is stable from the `src_valid` edge that accepted it until one source
//   cycle after `src_ack`.  **That is the hold time the far side gets**, and it is a
//   design property, not a constraint: no `set_max_delay` writes it down.
// * The destination half captures the payload STAGES+1 destination clocks after the
//   toggle leaves here (STAGES flops to see the toggle, one more to edge-detect it).
//   The bound the crossing wants is therefore "settle within that, hold until the
//   ack" -- a number derived from the two clocks, not "one clock period", which is
//   the vendor's default and belongs to neither domain.
//
// `d64_video_row_ddr`'s `d_*` registers and `req_tog` are this shape written inline,
// and its `ack_s` is this module's acknowledge.  See docs/CDC_CLOSURE_2026-10-07.md
// for why the existing module was written up rather than edited.
module d64_cdc_descriptor_src #(
    parameter W      = 52,
    parameter STAGES = 2
) (
    input  wire         src_clk,
    input  wire         src_rst_n,
    input  wire         src_valid,      // sampled; ignored while src_busy
    input  wire [W-1:0] src_data,
    output wire         src_busy,       // in flight
    output wire         src_ack,        // one src cycle: the far side has it

    // ---- the crossing, wired by the integrator ----
    output reg  [W-1:0] x_data,         // held from acceptance to src_ack
    output reg          x_req_tog,      // flips once per accepted request
    input  wire         x_ack_tog       // flips once per capture, from the dst half
);
    reg busy;
    reg ack_seen;

    wire ack_q;
    d64_cdc_bit_sync #(.STAGES(STAGES)) u_ack (
        .clk(src_clk), .rst_n(src_rst_n), .d(x_ack_tog), .q(ack_q)
    );

    wire ack_edge = ack_q ^ ack_seen;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            x_data    <= {W{1'b0}};
            x_req_tog <= 1'b0;
            busy      <= 1'b0;
            ack_seen  <= 1'b0;
        end
        else begin
            ack_seen <= ack_q;
            if (ack_edge) busy <= 1'b0;

            // **The one thing this half must not do.**  A second assignment to
            // `x_data` while `busy` is set is the whole defect, and the only guard
            // against it is that there is no path here to make one.
            if (src_valid && !busy) begin
                x_data    <= src_data;
                x_req_tog <= ~x_req_tog;
                busy      <= 1'b1;
            end
        end
    end

    assign src_busy = busy;
    assign src_ack  = ack_edge;

endmodule
