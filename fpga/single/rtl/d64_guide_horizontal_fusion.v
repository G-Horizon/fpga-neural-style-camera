`timescale 1ns/1ps
// Lightweight ORIGINAL high-resolution detail trial. This is a horizontal
// [1,2,1]/4 low-pass subtraction, not2D guided filtering or super-resolution.
// Input style and guide must already be pixel-aligned (reader has1clk latency).
// One-pixel lookahead, replicated left/right edges, all metadata delayed equally.
module d64_guide_horizontal_fusion #(
    parameter integer W=1920
) (
    input wire clk,input wire rst_n,
    input wire in_de,in_hsync,in_vsync,in_good,
    input wire [23:0] in_rgb,input wire [10:0] in_x,in_y,
    input wire [31:0] in_fid,
    input wire guide_valid,input wire [7:0] guide_gray,
    input wire [31:0] guide_fid,input wire [1:0] strength,
    output wire out_de,output reg out_hsync,out_vsync,out_good,
    output wire [23:0] out_rgb,output reg [10:0] out_x,out_y,
    output wire [31:0] out_fid,
    output wire detail_applied,pair_mismatch
);
    reg previous_de,previous_hsync,previous_vsync,previous_good;
    reg [23:0] previous_rgb;
    reg [10:0] previous_x,previous_y;
    reg [31:0] previous_fid;
    reg [7:0] previous_gray,left_gray;
    reg previous_guide_valid,left_valid;
    reg [1:0] previous_strength;
    wire right_same=in_de&&in_x==previous_x+1'b1&&in_y==previous_y&&
                    in_fid==previous_fid&&guide_fid==previous_fid&&guide_valid;
    wire [7:0] left_sample=previous_x==0 ? previous_gray : left_gray;
    wire [7:0] right_sample=previous_x==W-1||!right_same ? previous_gray : guide_gray;
    wire [9:0] sum={2'b0,left_sample}+{1'b0,previous_gray,1'b0}+
                  {2'b0,right_sample}+10'd2;
    wire [7:0] local_low=sum[9:2];
    wire neighborhood_good=previous_good&&previous_guide_valid&&
       (previous_x==0||left_valid)&&
       (previous_x==W-1||right_same);
    wire [5:0] unused_magnitude;wire unused_negative;
    d64_guide_detail_fusion u_adjust(.clk(clk),.rst_n(rst_n),
       .in_valid(previous_de),.style_rgb(previous_rgb),.style_fid(previous_fid),
       .guide_valid(neighborhood_good),.guide_high(previous_gray),
       .guide_fid(previous_fid),.low_valid(neighborhood_good),.guide_low(local_low),
       .low_fid(previous_fid),.strength(previous_strength),
       .out_valid(out_de),.out_rgb(out_rgb),.out_fid(out_fid),
       .detail_applied(detail_applied),.pair_mismatch(pair_mismatch),
       .detail_magnitude(unused_magnitude),.detail_negative(unused_negative));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            previous_de<=0;previous_hsync<=0;previous_vsync<=0;previous_good<=0;
            previous_rgb<=0;previous_x<=0;previous_y<=0;previous_fid<=0;
            previous_gray<=0;left_gray<=0;
            previous_guide_valid<=0;left_valid<=0;previous_strength<=0;
            out_hsync<=0;out_vsync<=0;out_good<=0;out_x<=0;out_y<=0;
        end else begin
            previous_de<=in_de;previous_hsync<=in_hsync;previous_vsync<=in_vsync;
            out_hsync<=previous_hsync;out_vsync<=previous_vsync;
            out_good<=previous_good;out_x<=previous_x;out_y<=previous_y;
            if(in_de)begin
                previous_good<=in_good;previous_rgb<=in_rgb;
                previous_x<=in_x;previous_y<=in_y;previous_fid<=in_fid;
                previous_gray<=guide_gray;
                previous_guide_valid<=guide_valid&&guide_fid==in_fid;
                previous_strength<=strength;
                if(in_x==0)begin
                    left_gray<=guide_gray;left_valid<=guide_valid&&guide_fid==in_fid;
                end else begin
                    left_gray<=previous_gray;
                    left_valid<=previous_guide_valid&&previous_fid==in_fid&&
                                previous_x+1'b1==in_x&&previous_y==in_y;
                end
            end
        end
    end
endmodule
