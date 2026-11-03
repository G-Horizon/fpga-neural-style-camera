`timescale 1ns/1ps
module d2_a_input_reader(
 input wire clk,rst_n,start,input wire [31:0] base,fid,
 output reg out_valid,input wire out_ready,output reg [383:0] out_rgb,
 output reg [31:0] out_fid,output reg [7:0] out_x,out_y,
 output reg out_sof,out_eol,out_eof,output reg done,failed,
 output reg [239:0] cmd,input wire [137:0] rsp
);
 localparam IDLE=0,AR=1,R=2,HOLD=3,DRAIN=4;
 reg [2:0] state;reg [31:0] base_q;reg [1:0] q;
 always @*begin
 cmd=0;cmd[192]=1;cmd[193]=state==AR;cmd[225:194]=base_q+{14'd0,out_y,10'd0}+{22'd0,out_x,2'd0};
 cmd[233:226]=3;cmd[236:234]=4;cmd[238:237]=1;cmd[239]=state==R||state==DRAIN;
 end
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin state<=IDLE;base_q<=0;q<=0;out_valid<=0;out_rgb<=0;out_fid<=0;out_x<=0;out_y<=0;out_sof<=0;out_eol<=0;out_eof<=0;done<=0;failed<=0;end
 else begin
 done<=0;
 case(state)
 IDLE:if(start&&!failed)begin base_q<=base;out_fid<=fid;out_x<=0;out_y<=0;q<=0;state<=AR;end
 AR:if(rsp[5])begin q<=0;state<=R;end
 R:if(rsp[6])begin
 if(rsp[137:136]!=0||rsp[135]!=(q==3))begin failed<=1;state<=rsp[135]?IDLE:DRAIN;end
 else begin
 out_rgb[q*96+:96]<={rsp[126:103],rsp[94:71],rsp[62:39],rsp[30:7]};
 if(q==3)begin out_valid<=1;out_sof<=out_x==0&&out_y==0;out_eol<=out_x==240;out_eof<=out_x==240&&out_y==143;state<=HOLD;end else q<=q+1;
 end end
 HOLD:if(out_ready)begin
 out_valid<=0;if(out_eof)begin done<=1;state<=IDLE;end
 else begin if(out_x==240)begin out_x<=0;out_y<=out_y+1;end else out_x<=out_x+16;state<=AR;end
 end
 DRAIN:if(rsp[6]&&rsp[135])state<=IDLE;
 default:begin failed<=1;state<=IDLE;end
 endcase
 end end
endmodule
