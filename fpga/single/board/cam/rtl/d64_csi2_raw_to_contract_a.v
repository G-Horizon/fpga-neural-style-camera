`timescale 1ns/1ps
// Fixed camera mode: four packed RAW10 samples per CSI beat, 1920x1080.
// VS is a frame-start packet indication; HS is only a data gate.
// Markers come from configured geometry. AW=9 covers a 480-beat line.
// Both FIFO domains must reset together. Overflow/packing errors are sticky.
module d64_csi2_raw_to_contract_a #(
    parameter integer AW = 9,
    parameter integer PPC_MAX = 4,
    parameter integer W_SRC = 1920,
    parameter integer H_SRC = 1080
) (
    input wire clk_pix, rst_pix_n, pix_valid,
    input wire [63:0] pix_data,
    input wire [3:0] pix_per_clk,
    input wire hsync, vsync,
    input wire clk_a, rst_a_n,
    output wire raw_valid,
    output wire [9:0] raw10,
    output wire raw_sof, raw_sol, raw_eol, raw_eof,
    output wire [31:0] raw_frame_id,
    output wire [31:0] beats_in, px_in, beats_out, px_out, frames_out,
    output wire overflow, ppc_bad
);
    localparam integer LINE_BEATS = W_SRC / 4;
    localparam integer DW = 44;
    reg prev_v, frame_live;
    reg [15:0] bx, by;
    reg [31:0] beats_in_q, px_in_q;
    reg overflow_q, ppc_bad_q;
    wire v_rise = vsync && !prev_v;
    wire payload = pix_valid && (frame_live || v_rise);
    wire packing_ok = (pix_per_clk == 4'd4) && (pix_data[63:40] == 24'd0);
    wire [15:0] this_x = v_rise ? 16'd0 : bx;
    wire [15:0] this_y = v_rise ? 16'd0 : by;
    wire first_line_beat = this_x == 0;
    wire last_line_beat = this_x == LINE_BEATS-1;
    wire first_frame_beat = first_line_beat && (this_y == 0);
    wire last_frame_beat = last_line_beat && (this_y == H_SRC-1);
    wire fifo_full;
    wire fifo_we = payload && packing_ok && !fifo_full;
    wire [DW-1:0] fifo_wd = {last_frame_beat, last_line_beat,
                             first_line_beat, first_frame_beat, pix_data[39:0]};
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if (!rst_pix_n) begin
            prev_v <= 0; frame_live <= 0; bx <= 0; by <= 0;
            beats_in_q <= 0; px_in_q <= 0;
            overflow_q <= 0; ppc_bad_q <= 0;
        end else begin
            prev_v <= vsync;
            if (v_rise) begin frame_live <= 1; bx <= 0; by <= 0; end
            if (payload) begin
                if (!packing_ok) ppc_bad_q <= 1;
                if (fifo_full) overflow_q <= 1;
                beats_in_q <= beats_in_q + 1;
                px_in_q <= px_in_q + 4;
                if (last_line_beat) begin
                    bx <= 0; by <= this_y + 1;
                    if (last_frame_beat) frame_live <= 0;
                end else bx <= this_x + 1;
            end
        end
    end
    wire fifo_empty, fifo_valid;
    wire [DW-1:0] fifo_rd;
    reg pending;
    reg [1:0] q_count;
    reg q_rp, q_wp;
    reg [DW-1:0] q [0:1];
    reg [1:0] sample_i;
    wire fifo_re = !fifo_empty && !pending && (q_count < 2);
    wire q_pop = (q_count != 0) && (sample_i == 3);
    d64_async_fifo #(.DW(DW), .AW(AW)) u_x (
        .wr_clk(clk_pix), .wr_rst_n(rst_pix_n),
        .wr_en(fifo_we), .wr_data(fifo_wd), .wr_full(fifo_full),
        .wr_almost_full(), .wr_level(),
        .rd_clk(clk_a), .rd_rst_n(rst_a_n), .rd_en(fifo_re),
        .rd_data(fifo_rd), .rd_valid(fifo_valid),
        .rd_empty(fifo_empty), .rd_level()
    );
    reg [31:0] beats_out_q, px_out_q, frames_out_q;
    always @(posedge clk_a or negedge rst_a_n) begin
        if (!rst_a_n) begin
            pending <= 0; q_count <= 0; q_rp <= 0; q_wp <= 0;
            sample_i <= 0; beats_out_q <= 0; px_out_q <= 0;
            frames_out_q <= 0; q[0] <= 0; q[1] <= 0;
        end else begin
            if (fifo_re) pending <= 1;
            if (fifo_valid) begin
                pending <= 0;
                q[q_wp] <= fifo_rd; q_wp <= !q_wp;
            end
            case ({fifo_valid,q_pop})
                2'b10: q_count <= q_count + 1;
                2'b01: q_count <= q_count - 1;
                default: q_count <= q_count;
            endcase
            if (raw_valid) begin
                px_out_q <= px_out_q + 1;
                if (q_pop) begin
                    sample_i <= 0; q_rp <= !q_rp;
                    beats_out_q <= beats_out_q + 1;
                    if (raw_eof) frames_out_q <= frames_out_q + 1;
                end else sample_i <= sample_i + 1;
            end
        end
    end
    assign raw_valid = q_count != 0;
    assign raw10 = q[q_rp][sample_i*10 +: 10];
    assign raw_sof = raw_valid && q[q_rp][40] && (sample_i == 0);
    assign raw_sol = raw_valid && q[q_rp][41] && (sample_i == 0);
    assign raw_eol = raw_valid && q[q_rp][42] && (sample_i == 3);
    assign raw_eof = raw_valid && q[q_rp][43] && (sample_i == 3);
    assign raw_frame_id = frames_out_q;
    assign beats_in = beats_in_q;
    assign px_in = px_in_q;
    assign beats_out = beats_out_q;
    assign px_out = px_out_q;
    assign frames_out = frames_out_q;
    assign overflow = overflow_q;
    assign ppc_bad = ppc_bad_q;
endmodule

