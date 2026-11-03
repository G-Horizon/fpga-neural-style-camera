`timescale 1ns/1ps
// Original guide is a demosaiced, WB/gamma processed green channel proxy.
// guide_low is the same frame's ORIGINAL CNN input green, bilinear enlarged.
// It must not be the styled green channel or a current unrelated camera frame.
module d64_guide_detail_fusion (
    input wire clk, input wire rst_n,
    input wire in_valid,
    input wire [23:0] style_rgb,
    input wire [31:0] style_fid,
    input wire guide_valid, input wire [7:0] guide_high,
    input wire [31:0] guide_fid,
    input wire low_valid, input wire [7:0] guide_low,
    input wire [31:0] low_fid,
    input wire [1:0] strength,
    output reg out_valid, output reg [23:0] out_rgb,
    output reg [31:0] out_fid,
    output reg detail_applied, output reg pair_mismatch,
    output reg [5:0] detail_magnitude, output reg detail_negative
);
    wire matched = guide_valid && low_valid &&
                   guide_fid==style_fid && low_fid==style_fid;
    wire negative = guide_high < guide_low;
    wire [7:0] distance = negative ? guide_low-guide_high : guide_high-guide_low;
    wire [5:0] clipped = distance>8'd32 ? 6'd32 : distance[5:0];
    wire [6:0] quarter_round = {1'b0,clipped}+7'd2;
    wire [6:0] half_round = {1'b0,clipped}+7'd1;
    wire [5:0] magnitude = strength==1 ? quarter_round[6:2] :
                                strength==2 ? half_round[6:1] : clipped;
    wire apply_detail = matched && strength!=0;
    function [7:0] adjust;
        input [7:0] channel;
        input [5:0] amount;
        input subtract;
        reg [8:0] sum;
        begin
            // Unsigned explicit carry and magnitude avoid mapper signedness
            // ambiguities previously found in this board's toolchain.
            sum={1'b0,channel}+{3'b0,amount};
            if(subtract) adjust=channel<{2'b0,amount} ? 8'd0 : channel-amount;
            else adjust=sum[8] ? 8'd255 : sum[7:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            out_valid<=0;out_rgb<=0;out_fid<=0;
            detail_applied<=0;pair_mismatch<=0;
            detail_magnitude<=0;detail_negative<=0;
        end else begin
            out_valid<=in_valid;
            detail_applied<=in_valid && apply_detail;
            pair_mismatch<=in_valid && strength!=0 && !matched;
            if(in_valid) begin
                out_fid<=style_fid;
                out_rgb<=apply_detail ?
                    {adjust(style_rgb[23:16],magnitude,negative),
                     adjust(style_rgb[15:8],magnitude,negative),
                     adjust(style_rgb[7:0],magnitude,negative)} : style_rgb;
                detail_magnitude<=apply_detail ? magnitude : 0;
                detail_negative<=apply_detail && negative;
            end
        end
    end
endmodule
