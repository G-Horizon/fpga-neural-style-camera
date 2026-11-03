`timescale 1ns/1ps
// App-packet formatter. MAC buffers the complete packet before wire emission.
module d2e_header_tx(
 input wire clk,rst_n,
 input wire request,output wire request_ready,input wire [479:0] header,
 input wire [7:0] s_data,input wire s_valid,output wire s_ready,
 output wire [7:0] m_data,output wire m_valid,input wire m_ready,output wire m_last,
 output reg packet_done,output reg protocol_fault
);
 `include "d2e_protocol.vh"
 localparam IDLE=0,HEADER=1,BODY=2;
 reg [1:0] state;
 reg [479:0] held_header;
 reg [5:0] index;
 reg [15:0] length,left;
 reg [31:0] crc;
 wire [31:0] checksum=~crc;
 assign request_ready=state==IDLE;
 assign m_valid=state==HEADER||(state==BODY&&s_valid);
 assign m_data=state==BODY?s_data:index<60?held_header[7:0]:checksum[index[1:0]*8+:8];
 assign m_last=(state==HEADER&&index==63&&length==0)||(state==BODY&&left==1);
 assign s_ready=state==BODY&&m_ready;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;held_header<=0;index<=0;length<=0;left<=0;crc<=32'hffffffff;packet_done<=0;protocol_fault<=0;end
  else begin
   packet_done<=0;
   if(request&&request_ready)begin
    if(header[63:0]!=`D2E_MAGIC||header[71:64]!=`D2E_VERSION||header[335:320]>`D2E_MAX_DATA)protocol_fault<=1;
    else begin
     held_header<=header;length<=header[335:320];left<=header[335:320];index<=0;crc<=32'hffffffff;state<=HEADER;
    end
   end
   if(m_valid&&m_ready)begin
    if(state==HEADER)begin
     if(index<60)begin crc<=d2e_crc_byte(crc,m_data);held_header<={8'd0,held_header[479:8]};end
     if(index==63)begin
      if(length==0)begin state<=IDLE;packet_done<=1;end else state<=BODY;
     end else index<=index+1'b1;
    end else begin
     left<=left-1'b1;
     if(left==1)begin state<=IDLE;packet_done<=1;end
    end
   end
  end
 end
endmodule
