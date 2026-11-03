`timescale 1ns/1ps
module d2_uart_phy #(parameter integer CLK_HZ=100000000,BAUD=1000000)(
 input wire clk,rst_n,rx,output reg rx_valid,output reg [7:0] rx_data,output reg framing_error,
 input wire tx_valid,input wire [7:0] tx_data,output wire tx_ready,output wire tx
);
 localparam integer DIV=CLK_HZ/BAUD;
 (* async_reg="true" *)reg rx_meta,rx_sync;
 reg [15:0] rcount,tcount;reg [3:0] rs,ts;reg [7:0] rshift;reg [9:0] tshift;
 assign tx_ready=ts==0;assign tx=ts==0?1'b1:tshift[0];
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin rx_meta<=1;rx_sync<=1;rx_valid<=0;rx_data<=0;framing_error<=0;rs<=0;rcount<=0;rshift<=0;end
 else begin
 rx_meta<=rx;rx_sync<=rx_meta;rx_valid<=0;framing_error<=0;
 if(rs==0)begin if(!rx_sync)begin rs<=1;rcount<=DIV/2-1;end end
 else if(rcount!=0)rcount<=rcount-1;
 else begin
 rcount<=DIV-1;
 if(rs==1)begin if(rx_sync)rs<=0;else rs<=2;end
 else if(rs<=9)begin rshift[rs-2]<=rx_sync;rs<=rs+1;end
 else begin rs<=0;if(rx_sync)begin rx_valid<=1;rx_data<=rshift;end else framing_error<=1;end
 end
 end end
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin ts<=0;tcount<=0;tshift<=10'h3ff;end
 else if(ts==0)begin if(tx_valid)begin tshift<={1'b1,tx_data,1'b0};ts<=10;tcount<=DIV-1;end end
 else if(tcount!=0)tcount<=tcount-1;
 else begin tshift<={1'b1,tshift[9:1]};ts<=ts-1;tcount<=DIV-1;end
 end
 // synthesis translate_off
 initial if(DIV<4||CLK_HZ%BAUD!=0)$fatal(1,"UART requires integer divisor >=4");
 // synthesis translate_on
endmodule
