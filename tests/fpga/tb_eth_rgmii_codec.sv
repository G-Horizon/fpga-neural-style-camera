`timescale 1ns/1ps
module tb_eth_rgmii_codec;
    reg [7:0] d;reg en,er;wire [3:0] lo,hi;wire cr,cf,clkr,clkf;
    wire [7:0] r;wire dv,re;
    eth_rgmii_codec dut(d,en,er,lo,hi,cr,cf,clkr,clkf,lo,hi,cr,cf,r,dv,re);
    integer i,j;
    initial begin
        for(i=0;i<256;i=i+1)for(j=0;j<4;j=j+1)begin
            d=i;en=j&1;er=(j>>1)&1;#1;
            if(lo!==d[3:0]||hi!==d[7:4]||cr!==en||cf!==(en^er)||r!==d||dv!==en||re!==er||clkr!==1'b1||clkf!==1'b0)$fatal(1,"DDR codec mapping");
        end
        $display("DDR_CODEC_1024_PASS");$finish;
    end
endmodule
