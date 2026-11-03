`timescale 1ns/1ps
// Fixed RAW10 four-pixel contract adapter. No scalar FIFO or demultiplexer.
// HS is not used as a line boundary. FS arms the configured geometry.
// All outputs and diagnostic counters belong to clk_pix.
module d64_packed_raw_camera_adapter #(
    parameter integer W_SRC=1920,H_SRC=1080
)(
    input wire clk_pix,rst_pix_n,pix_valid,
    input wire [63:0] pix_data,
    input wire [3:0] pix_per_clk,
    input wire vsync,
    output wire raw_valid,
    output wire [39:0] raw4,
    output wire raw_sof,raw_sol,raw_eol,raw_eof,
    output wire [31:0] raw_frame_id,
    output reg [31:0] beats_in,packing_errors,incomplete_frames,
    output reg source_bad_sticky
);
    localparam integer LINE_WORDS=W_SRC/4;
    reg vs_q,armed,seen;
    reg [15:0] bx,by;
    reg [31:0] fid;
    wire fs=vsync&&!vs_q;
    wire packing=(pix_per_clk==4 && pix_data[63:40]==0);
    wire [15:0] x=fs ? 0 : bx;
    wire [15:0] y=fs ? 0 : by;
    wire payload=pix_valid&&(armed||fs);
    assign raw_valid=payload&&packing;
    assign raw4=pix_data[39:0];
    assign raw_sol=(x==0);
    assign raw_sof=(x==0&&y==0);
    assign raw_eol=(x==LINE_WORDS-1);
    assign raw_eof=(x==LINE_WORDS-1&&y==H_SRC-1);
    assign raw_frame_id=fs ? (seen ? fid+1'b1 : 0) : fid;
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if(!rst_pix_n)begin
            vs_q<=0;armed<=0;seen<=0;bx<=0;by<=0;fid<=0;
            beats_in<=0;packing_errors<=0;incomplete_frames<=0;source_bad_sticky<=0;
        end else begin
            vs_q<=vsync;
            if(fs)begin
                if(armed)begin incomplete_frames<=incomplete_frames+1'b1;source_bad_sticky<=1;end
                armed<=1;seen<=1;bx<=0;by<=0;
                if(seen)fid<=fid+1'b1;
            end
            if(payload)begin
                beats_in<=beats_in+1'b1;
                if(!packing)begin packing_errors<=packing_errors+1'b1;source_bad_sticky<=1;end
                // Bad packing does not stall geometry. Downstream marker checks
                // and source_bad_sticky reject the corrupted capture.
                if(raw_eol)begin
                    bx<=0;by<=y+1'b1;
                    if(raw_eof)armed<=0;
                end else begin bx<=x+1'b1;by<=y;end
            end
        end
    end
endmodule
