`timescale 1ns/1ps
module d64_counter_gray_bridge #(
    parameter integer W=32
) (
    input wire src_clk,src_rst_n,
    input wire [W-1:0] src_count,
    input wire dst_clk,dst_rst_n,
    output wire [W-1:0] dst_count
);
    reg [W-1:0] source_gray;
    (* async_reg="true" *) reg [W-1:0] sync1,sync2;
    always @(posedge src_clk or negedge src_rst_n)
        if(!src_rst_n)source_gray<=0;
        else source_gray<=src_count^(src_count>>1);
    always @(posedge dst_clk or negedge dst_rst_n)
        if(!dst_rst_n)begin sync1<=0;sync2<=0;end
        else begin sync1<=source_gray;sync2<=sync1;end
    genvar bitno;
    generate for(bitno=0;bitno<W;bitno=bitno+1)begin:g_decode
        assign dst_count[bitno]=^sync2[W-1:bitno];
    end endgenerate
endmodule
