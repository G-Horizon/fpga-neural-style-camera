// The destination half of a bundled-data descriptor crossing.  Destination domain only.
//
// The other half of the class is in `d64_cdc_descriptor_src.v`; read its header first.
// This half does three things and nothing else:
//
//   1. re-times `x_req_tog` through `STAGES` flops and detects the change,
//   2. on that edge copies `x_data` into a destination-domain register,
//   3. and flips `x_ack_tog` on the same edge, which is what releases the source's
//      hold.  The ack is produced on the cycle the data was captured, not a cycle
//      later, so the source can never be told "taken" before it was.
//
// **`x_data` is sampled on the cycle the toggle's edge is seen, which is STAGES+1
// destination clocks after the source put it down.**  That is the settling window the
// bundled data path has, and it is what a constraint on this crossing should be
// written against -- `set_max_delay -datapath_only` on `x_data` from the source flop
// to the capture flop, bounded by (STAGES+1) destination clocks minus the destination
// flop's setup.  `docs/CDC_CLOSURE_2026-10-07.md` carries the two lines and says why
// neither is in an SDC file today.
//
// The last flop of the synchroniser is the settled one; the edge is taken against a
// registered copy of it (`req_seen`) rather than against the XOR of the two
// synchroniser stages, so a metastable first stage is never published as a pulse.
//
// Reset **low asserted** and taken from the domain's own reset, like every other
// module in this tree: a private reset synchroniser inside a CDC module releases its
// clock two cycles after everything around it, which is a latency the caller cannot
// see.  See `d64_reset_sync.v`.
module d64_cdc_descriptor_dst #(
    parameter W      = 52,
    parameter STAGES = 2
) (
    input  wire         dst_clk,
    input  wire         dst_rst_n,
    output wire         dst_valid,      // one dst cycle
    output wire [W-1:0] dst_data,       // held until the next capture

    // ---- the crossing, wired by the integrator ----
    input  wire [W-1:0] x_data,
    input  wire         x_req_tog,
    output reg          x_ack_tog
);
    reg req_seen;
    reg [W-1:0] data_q;

    wire req_q;
    d64_cdc_bit_sync #(.STAGES(STAGES)) u_req (
        .clk(dst_clk), .rst_n(dst_rst_n), .d(x_req_tog), .q(req_q)
    );

    wire req_edge = req_q ^ req_seen;

    // **`dst_valid` is registered, and that is not decoration.**  `req_edge` is high on
    // the cycle *before* the capture lands: a consumer that read `dst_data` on the edge
    // would read the previous word, because the capture at that edge is what makes the
    // new one.  Written the other way this module hands back the payload one handshake
    // late and looks entirely correct in a waveform -- the testbench below caught it on
    // its first run, on the `dst_data !== held` count.
    reg valid_q;

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_seen  <= 1'b0;
            data_q    <= {W{1'b0}};
            valid_q   <= 1'b0;
            x_ack_tog <= 1'b0;
        end
        else begin
            req_seen <= req_q;
            valid_q  <= req_edge;          // the pulse, one cycle later, beside the data
            if (req_edge) begin
                // The source is holding this still.  If it were not, this line would
                // be a latch of whatever the skew left on the wire.
                data_q <= x_data;
            end
            // The acknowledge goes out one cycle after the capture, never on the same
            // edge: "taken" must not be said before the word is in the register that
            // holds it, or the source may move the bus under the capture.
            if (valid_q) x_ack_tog <= ~x_ack_tog;
        end
    end

    assign dst_valid = valid_q;
    assign dst_data  = data_q;

endmodule
