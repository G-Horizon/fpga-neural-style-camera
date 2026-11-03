`timescale 1ns/1ps
// A complete MAC/FCS-qualified app packet enters here. Upper layer decides
// whether to accept the held header before any body byte becomes visible.
module d2e_header_rx(
 input wire clk,rst_n,
 input wire [7:0] s_data,input wire s_valid,output wire s_ready,input wire s_last,
 input wire [10:0] s_length,
 output wire header_valid,input wire header_ready,input wire accept_body,
 output wire [479:0] header,output wire header_has_body,
 output wire [7:0] m_data,output wire m_valid,input wire m_ready,output wire m_last,
 output reg packet_done,output reg [31:0] rejected_headers,output reg protocol_fault
);
 `include "d2e_protocol.vh"
 localparam HEAD=0,DECIDE=1,BODY=2,DRAIN=3;
 reg [1:0] state;
 reg [511:0] held;
 reg [5:0] index;
 reg [10:0] packet_length;
 reg [15:0] left;
 reg [31:0] crc;
 reg header_ended;
 wire [511:0] updated={s_data,held[511:8]};
 wire [15:0] advertised_length=updated[335:320];
 wire header_ok=updated[63:0]==`D2E_MAGIC&&updated[71:64]==`D2E_VERSION&&
  updated[383:376]==0&&updated[479:448]==0&&updated[511:480]==~crc&&
  advertised_length<=`D2E_MAX_DATA&&packet_length==64+advertised_length&&
  (s_last==(advertised_length==0));
 assign s_ready=state==HEAD||state==DRAIN||(state==BODY&&m_ready);
 assign header_valid=state==DECIDE;
 assign header=held[479:0];
 assign header_has_body=left!=0;
 assign m_data=s_data;
 assign m_valid=state==BODY&&s_valid;
 assign m_last=state==BODY&&left==1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=HEAD;held<=0;index<=0;packet_length<=0;left<=0;crc<=32'hffffffff;
   header_ended<=0;packet_done<=0;rejected_headers<=0;protocol_fault<=0;
  end else begin
   packet_done<=0;
   if(header_valid&&header_ready)begin
    if(header_ended)begin state<=HEAD;packet_done<=1;index<=0;crc<=32'hffffffff;end
    else state<=accept_body?BODY:DRAIN;
   end
   if(s_valid&&s_ready)begin
    case(state)
     HEAD:begin
      if(index==0)packet_length<=s_length;
      held<=updated;
      if(index<60)crc<=d2e_crc_byte(crc,s_data);
      if(s_last&&index!=63)begin
       rejected_headers<=rejected_headers+1'b1;state<=HEAD;index<=0;crc<=32'hffffffff;packet_done<=1;
      end else if(index==63)begin
       if(header_ok)begin state<=DECIDE;left<=advertised_length;header_ended<=s_last;end
       else begin
        rejected_headers<=rejected_headers+1'b1;state<=s_last?HEAD:DRAIN;
        if(s_last)begin packet_done<=1;index<=0;crc<=32'hffffffff;end
       end
      end else index<=index+1'b1;
     end
     BODY:begin
      if(s_last!=(left==1))protocol_fault<=1;
      left<=left-1'b1;
      if(s_last||left==1)begin
       if(s_last)begin state<=HEAD;packet_done<=1;index<=0;crc<=32'hffffffff;end
       else state<=DRAIN;
      end
     end
     DRAIN:if(s_last)begin state<=HEAD;packet_done<=1;index<=0;crc<=32'hffffffff;end
    endcase
   end
  end
 end
endmodule
