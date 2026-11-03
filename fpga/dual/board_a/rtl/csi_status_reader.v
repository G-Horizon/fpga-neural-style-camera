module csi_status_reader #(
    parameter integer POLL_GAP_CYCLES = 1000000,
    parameter integer TIMEOUT_CYCLES = 4096
) (
    input  wire         clk,
    input  wire         rst_n,
    output wire [5:0]   axi_araddr,
    output wire         axi_arvalid,
    input  wire         axi_arready,
    input  wire [31:0]  axi_rdata,
    input  wire         axi_rvalid,
    output wire         axi_rready,
    output wire [127:0] status_snapshot,
    output reg          sample_done
);
    localparam [1:0] WAIT_GAP=2'd0, SEND_AR=2'd1, WAIT_R=2'd2;
    reg [1:0] state;
    reg [2:0] index;
    reg [31:0] gap_count, wait_count;
    reg [15:0] sequence_number;
    reg sample_valid;
    reg [6:0] ar_timeouts, r_timeouts;
    reg [15:0] work_isr, work_ier, isr_latest, ier_latest, isr_seen;
    reg [31:0] work_lanes, lanes_latest;
    reg [7:0] clock_latest;

    function [5:0] register_address;
        input [2:0] item;
        begin
            case (item)
                3'd0: register_address=6'h00;
                3'd1: register_address=6'h04;
                3'd2: register_address=6'h08;
                3'd3: register_address=6'h0c;
                3'd4: register_address=6'h10;
                3'd5: register_address=6'h14;
                default: register_address=6'h28;
            endcase
        end
    endfunction

    wire [7:0] flags = {4'b0000, (state==WAIT_R), (state==SEND_AR),
                        (state!=WAIT_GAP), sample_valid};
    assign axi_araddr=register_address(index);
    assign axi_arvalid=(state==SEND_AR);
    assign axi_rready=(state==WAIT_R);
    // MSB first: sequence, flags, clock, lanes 0..3, IER, ISR, observed ISR,
    // address-timeout mask, response-timeout mask. Timeout bits follow read order.
    assign status_snapshot={sequence_number, flags, clock_latest,
                            lanes_latest, ier_latest, isr_latest, isr_seen,
                            1'b0, ar_timeouts, 1'b0, r_timeouts};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=WAIT_GAP; index<=0; gap_count<=0; wait_count<=0;
            sequence_number<=0; sample_valid<=0; sample_done<=0;
            ar_timeouts<=0; r_timeouts<=0;
            work_isr<=0; work_ier<=0; work_lanes<=0;
            isr_latest<=0; ier_latest<=0; lanes_latest<=0;
            clock_latest<=0; isr_seen<=0;
        end else begin
            sample_done<=0;
            case (state)
                WAIT_GAP: begin
                    wait_count<=0;
                    if (gap_count==0) begin
                        index<=0;
                        state<=SEND_AR;
                    end else gap_count<=gap_count-1'b1;
                end
                SEND_AR: begin
                    if (axi_arready) begin
                        wait_count<=0;
                        state<=WAIT_R;
                    end else if (wait_count>=TIMEOUT_CYCLES-1) begin
                        ar_timeouts[index]<=1'b1;
                        // Never abandon a live AXI transaction after timeout.
                    end else wait_count<=wait_count+1'b1;
                end
                WAIT_R: begin
                    if (axi_rvalid) begin
                        wait_count<=0;
                        case (index)
                            0: begin
                                work_isr<=axi_rdata[15:0];
                                isr_seen<=isr_seen | axi_rdata[15:0];
                            end
                            1: work_ier<=axi_rdata[15:0];
                            2: work_lanes[31:24]<=axi_rdata[7:0];
                            3: work_lanes[23:16]<=axi_rdata[7:0];
                            4: work_lanes[15:8]<=axi_rdata[7:0];
                            5: work_lanes[7:0]<=axi_rdata[7:0];
                            default: begin
                                isr_latest<=work_isr;
                                ier_latest<=work_ier;
                                lanes_latest<=work_lanes;
                                clock_latest<=axi_rdata[7:0];
                                sequence_number<=sequence_number+1'b1;
                                sample_valid<=1'b1;
                                sample_done<=1'b1;
                            end
                        endcase
                        if (index==6) begin
                            state<=WAIT_GAP;
                            gap_count<=POLL_GAP_CYCLES;
                        end else begin
                            index<=index+1'b1;
                            state<=SEND_AR;
                        end
                    end else if (wait_count>=TIMEOUT_CYCLES-1) begin
                        r_timeouts[index]<=1'b1;
                        // Preserve outstanding read until its response or reset.
                    end else wait_count<=wait_count+1'b1;
                end
                default: state<=WAIT_GAP;
            endcase
        end
    end
endmodule
