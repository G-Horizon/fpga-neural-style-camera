`timescale 1ns/1ps
// Import one already-qualified HWC U16 feature plane into global trunk buffer B.
// Completion means the last write response retired, never merely last W beat.
module d2_b_tensor_copy #(
 parameter integer BYTES=147456,
 parameter[31:0]SOURCE_BASE=32'h04000000,DEST_BASE=32'h00400000
)(
 input wire clk,rst_n,start,input wire[2:0]slot,
 output wire idle,output reg done,failed,output reg fault_sticky,
 output reg[31:0]copied_words,output wire[239:0]axi_cmd,input wire[137:0]axi_rsp
);
 localparam IDLE=0,AR=1,R=2,AW=3,LOAD=4,W=5,B=6;
 localparam TOTAL=BYTES/16;
 reg[2:0]state;reg[31:0]source_addr,dest_addr;reg[7:0]length,received,sent;
 reg burst_bad;reg[127:0]data_q;
 (* ram_style="block" *)reg[127:0]burst_ram[0:127];
 wire rfire=state==R&&axi_rsp[6];
 wire wfire=state==W&&axi_rsp[1];
 wire[6:0]ra=state==LOAD?0:(wfire?sent[6:0]+1'b1:sent[6:0]);
 always @(posedge clk)if(rfire&&received<length)burst_ram[received[6:0]]<=axi_rsp[7+:128];
 always @(posedge clk)if(state==LOAD||state==W)data_q<=burst_ram[ra];
 function[7:0]limit;
  input[31:0]src,dst,n;reg[31:0]z,b1,b2;
  begin z=TOTAL-n;if(z>128)z=128;b1=256-src[11:4];b2=256-dst[11:4];if(z>b1)z=b1;if(z>b2)z=b2;limit=z[7:0];end
 endfunction
 assign idle=state==IDLE;
 assign axi_cmd={state==R,2'd1,3'd4,length-8'd1,source_addr,state==AR,
   state==B,state==W&&sent==length-1'b1,16'hffff,data_q,state==W,
   2'd1,3'd4,length-8'd1,dest_addr,state==AW};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;source_addr<=0;dest_addr<=DEST_BASE;length<=0;received<=0;sent<=0;
   burst_bad<=0;done<=0;failed<=0;fault_sticky<=0;copied_words<=0;end
  else begin
   done<=0;failed<=0;
   if(start)begin
    if(idle&&!fault_sticky&&slot<8)begin
     source_addr<=SOURCE_BASE+({29'd0,slot}<<18);dest_addr<=DEST_BASE;
     copied_words<=0;length<=limit(SOURCE_BASE+({29'd0,slot}<<18),DEST_BASE,0);burst_bad<=0;state<=AR;
    end else fault_sticky<=1;
   end
   case(state)
    AR:if(axi_rsp[5])begin received<=0;burst_bad<=0;state<=R;end
    R:if(rfire)begin
     if(axi_rsp[136+:2]!=0||axi_rsp[135]!=(received==length-1'b1)||received>=length)begin burst_bad<=1;fault_sticky<=1;end
     received<=received+1'b1;
     if(axi_rsp[135])begin
      if(burst_bad||axi_rsp[136+:2]!=0||received!=length-1'b1)begin done<=1;failed<=1;state<=IDLE;end
      else state<=AW;
     end
    end
    AW:if(axi_rsp[0])begin sent<=0;state<=LOAD;end
    LOAD:state<=W;
    W:if(wfire)begin if(sent==length-1'b1)state<=B;else sent<=sent+1'b1;end
    B:if(axi_rsp[2])begin
     if(axi_rsp[3+:2]!=0)begin fault_sticky<=1;done<=1;failed<=1;state<=IDLE;end
     else if(copied_words+length==TOTAL)begin copied_words<=copied_words+length;done<=1;state<=IDLE;end
     else begin
      copied_words<=copied_words+length;source_addr<=source_addr+({24'd0,length}<<4);dest_addr<=dest_addr+({24'd0,length}<<4);
      length<=limit(source_addr+({24'd0,length}<<4),dest_addr+({24'd0,length}<<4),copied_words+length);state<=AR;
     end
    end
   endcase
  end
 end
 // synthesis translate_off
 initial if(BYTES%16||BYTES>262144||BYTES==0)$fatal(1,"D2_TENSOR_BYTES");
 // synthesis translate_on
endmodule
