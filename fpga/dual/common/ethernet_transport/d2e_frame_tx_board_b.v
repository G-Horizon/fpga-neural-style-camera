`timescale 1ns/1ps
// Stop-and-wait sender. Source DDR lease must remain immutable until done.
module d2e_frame_tx #(
 parameter integer FRAME_BYTES=147456,STREAM=1,TIMEOUT_CYCLES=100000,MAX_RETRIES=8
)(
 input wire clk,rst_n,session_valid,input wire[31:0]local_session,remote_session,
 input wire job_valid,output wire job_ready,input wire[31:0]job_base,job_fid,job_epoch,
 input wire[1:0]job_style,job_layout,input wire job_fps,input wire[7:0]job_effect,
 output reg done,done_ok,output reg[31:0]done_fid,
 output wire packet_valid,input wire packet_ready,output reg[479:0]packet_header,
 output wire[7:0]body_data,output wire body_valid,input wire body_ready,input wire packet_done,
 input wire response_valid,input wire[479:0]response_header,
 output reg[239:0]axi_cmd,input wire[137:0]axi_rsp,
 output reg fault_sticky,output reg[31:0]completed_frames,retransmissions,ignored_responses
);
 `include "d2e_protocol.vh"
 localparam IDLE=0,HEADER=1,ADDR=2,DATA=3,PACKET_WAIT=4,ACK_WAIT=5,ABORT_WAIT=6;
 reg[2:0]state;
 reg[7:0]kind;
 reg[31:0]base,fid,epoch,local_q,remote_q,offset,lease,message_sequence;
 reg[1:0]style,layout;reg fps;reg[7:0]effect;
 reg[31:0]frame_crc,working_crc,timer;
 reg[3:0]retry_count;
 reg[15:0]length,remaining;
 reg[31:0]address;
 reg[6:0]burst_beats,received;
 reg[2:0]delivered;
 reg[127:0]burst_data[0:3];
 reg bus_active,buffer_valid,data_fault,cancel_pending;
 reg[127:0]buffer;
 reg[4:0]buffer_left;
 wire read_fire=axi_cmd[239]&&axi_rsp[6];
 wire body_fire=body_valid&&body_ready;
 wire[31:0]job_remaining=FRAME_BYTES-offset;
 wire[15:0]next_length=job_remaining>`D2E_MAX_DATA?`D2E_MAX_DATA:job_remaining[15:0];
 wire[8:0]to_boundary=9'd256-{1'b0,address[11:4]};
 wire[8:0]needed_beats={1'b0,remaining[11:4]};
 wire[8:0]original_safe_beats=needed_beats<to_boundary?needed_beats:to_boundary;
 wire[8:0]safe_beats=(original_safe_beats < 9'd4) ? original_safe_beats : 9'd4;
 wire matching_response=response_header[63:0]==`D2E_MAGIC&&response_header[71:64]==1&&
  response_header[87:80]==STREAM&&response_header[127:96]==remote_q&&response_header[159:128]==local_q&&
  response_header[191:160]==fid&&response_header[223:192]==epoch&&
  response_header[255:224]==message_sequence&&response_header[287:256]==FRAME_BYTES&&response_header[335:320]==0;
 wire lease_response=kind==`D2E_BEGIN||response_header[447:416]==lease;
 wire success_response=response_header[375:368]==0;
 wire begin_ack=matching_response&&response_header[79:72]==`D2E_BEGIN_ACK&&success_response&&
  response_header[319:288]==0&&response_header[447:416]!=0;
 wire data_ack=matching_response&&response_header[79:72]==`D2E_DATA_ACK&&success_response&&lease_response&&
  response_header[319:288]==offset+length;
 wire end_ack=matching_response&&response_header[79:72]==`D2E_END_ACK&&success_response&&lease_response&&
  response_header[319:288]==FRAME_BYTES&&response_header[415:384]==~frame_crc;
 assign job_ready=state==IDLE&&session_valid&&!fault_sticky;
 assign packet_valid=state==HEADER;
 assign body_valid=state==DATA&&buffer_valid;
 assign body_data=buffer[7:0];
 always @*begin
  packet_header=0;
  packet_header[63:0]=`D2E_MAGIC;packet_header[71:64]=1;
  packet_header[79:72]=kind;packet_header[87:80]=STREAM;
  packet_header[127:96]=local_q;packet_header[159:128]=remote_q;
  packet_header[191:160]=fid;packet_header[223:192]=epoch;
  packet_header[255:224]=message_sequence;packet_header[287:256]=FRAME_BYTES;
  packet_header[319:288]=offset;packet_header[335:320]=kind==`D2E_DATA?length:0;
  packet_header[343:336]={6'd0,style};packet_header[351:344]={6'd0,layout};
  packet_header[359:352]={7'd0,fps};packet_header[367:360]=effect;
  packet_header[415:384]=kind==`D2E_END?~frame_crc:0;packet_header[447:416]=lease;
  axi_cmd=0;axi_cmd[192]=1;
  axi_cmd[193]=state==ADDR&&burst_beats!=0&&!bus_active&&!data_fault;
  axi_cmd[225:194]=address;axi_cmd[233:226]=burst_beats-1'b1;
  axi_cmd[236:234]=4;axi_cmd[238:237]=1;
  axi_cmd[239]=bus_active;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;kind<=0;base<=0;fid<=0;epoch<=0;local_q<=0;remote_q<=0;offset<=0;lease<=0;
   message_sequence<=0;style<=0;layout<=0;fps<=0;effect<=0;frame_crc<=32'hffffffff;working_crc<=32'hffffffff;
   timer<=0;retry_count<=0;length<=0;remaining<=0;address<=0;burst_beats<=0;received<=0;delivered<=0;
   bus_active<=0;buffer_valid<=0;buffer<=0;buffer_left<=0;data_fault<=0;cancel_pending<=0;
   done<=0;done_ok<=0;done_fid<=0;fault_sticky<=0;completed_frames<=0;retransmissions<=0;ignored_responses<=0;
  end else begin
   done<=0;
   if(state!=IDLE&&(!session_valid||local_session!=local_q||remote_session!=remote_q))cancel_pending<=1;
   if(response_valid&&!(state==ACK_WAIT&&matching_response&&lease_response))ignored_responses<=ignored_responses+1'b1;
   if(read_fire)begin
    received<=received+1'b1;
    if(axi_rsp[137:136]!=0||axi_rsp[135]!=(received==burst_beats-1'b1))begin data_fault<=1;fault_sticky<=1;end
    if(axi_rsp[135])bus_active<=0;
    if(!data_fault)burst_data[received[1:0]]<=axi_rsp[134:7];
   end
   if(body_fire)begin
    working_crc<=d2e_crc_byte(working_crc,body_data);
    buffer<=buffer>>8;buffer_left<=buffer_left-1'b1;remaining<=remaining-1'b1;
    if(buffer_left==1)buffer_valid<=0;
   end
   case(state)
    IDLE:if(job_valid&&job_ready)begin
     if(job_base[3:0]!=0||job_style>2||job_layout>2)begin fault_sticky<=1;done<=1;done_ok<=0;done_fid<=job_fid;end
     else begin
      base<=job_base;fid<=job_fid;epoch<=job_epoch;local_q<=local_session;remote_q<=remote_session;
      style<=job_style;layout<=job_layout;fps<=job_fps;effect<=job_effect;offset<=0;lease<=0;
      message_sequence<=message_sequence+1'b1;frame_crc<=32'hffffffff;working_crc<=32'hffffffff;
      retry_count<=0;data_fault<=0;cancel_pending<=0;kind<=`D2E_BEGIN;state<=HEADER;
     end
    end
    HEADER:if(packet_valid&&packet_ready)begin
     timer<=0;
     if(kind==`D2E_DATA)begin
      address<=base+offset;remaining<=length;working_crc<=frame_crc;buffer_valid<=0;received<=0;delivered<=0;
      // Address is registered before the burst bound is calculated in ADDR.
      burst_beats<=0;state<=ADDR;
     end else state<=PACKET_WAIT;
    end
    ADDR:begin
     if(burst_beats==0)burst_beats<=safe_beats[6:0];
     else if(axi_cmd[193]&&axi_rsp[5])begin received<=0;delivered<=0;bus_active<=1;state<=DATA;end
    end
    DATA:begin
     // Accept the complete four-beat DDR burst without byte-serialization stalls.
     // A lease is retained until all prefetched bytes have drained into the packet.
     if(!bus_active&&received==burst_beats&&!buffer_valid&&!data_fault&&delivered<burst_beats)begin
      buffer<=burst_data[delivered[1:0]];buffer_valid<=1;buffer_left<=16;delivered<=delivered+1'b1;
     end
     if(data_fault&&!buffer_valid&&remaining!=0)begin buffer<=0;buffer_valid<=1;buffer_left<=16;end
     if(body_fire&&remaining==1)state<=PACKET_WAIT;
     else if(body_fire&&buffer_left==1&&!bus_active&&received==burst_beats&&delivered==burst_beats&&remaining!=1&&!data_fault)begin
      address<=address+({25'd0,burst_beats}<<4);burst_beats<=0;state<=ADDR;
     end
    end
    PACKET_WAIT:if(packet_done)begin
     timer<=0;
     if(kind==`D2E_ABORT)state<=ABORT_WAIT;
     else if(data_fault||cancel_pending)begin kind<=`D2E_ABORT;message_sequence<=message_sequence+1'b1;state<=HEADER;end
     else state<=ACK_WAIT;
    end
    ACK_WAIT:begin
     if(cancel_pending)begin kind<=`D2E_ABORT;message_sequence<=message_sequence+1'b1;state<=HEADER;end
     else if(response_valid&&kind==`D2E_BEGIN&&begin_ack)begin
      lease<=response_header[447:416];length<=next_length;kind<=`D2E_DATA;
      message_sequence<=message_sequence+1'b1;retry_count<=0;state<=HEADER;
     end else if(response_valid&&kind==`D2E_DATA&&data_ack)begin
      frame_crc<=working_crc;offset<=offset+length;message_sequence<=message_sequence+1'b1;retry_count<=0;
      if(offset+length==FRAME_BYTES)kind<=`D2E_END;
      else length<=FRAME_BYTES-offset-length>`D2E_MAX_DATA?`D2E_MAX_DATA:FRAME_BYTES-offset-length;
      state<=HEADER;
     end else if(response_valid&&kind==`D2E_END&&end_ack)begin
      done<=1;done_ok<=1;done_fid<=fid;completed_frames<=completed_frames+1'b1;state<=IDLE;
     end else if(timer==TIMEOUT_CYCLES-1)begin
      timer<=0;
      if(retry_count==MAX_RETRIES)begin
       fault_sticky<=1;kind<=`D2E_ABORT;message_sequence<=message_sequence+1'b1;state<=HEADER;
      end else begin retry_count<=retry_count+1'b1;retransmissions<=retransmissions+1'b1;state<=HEADER;end
     end else timer<=timer+1'b1;
    end
    ABORT_WAIT:if(!bus_active)begin done<=1;done_ok<=0;done_fid<=fid;state<=IDLE;end
   endcase
  end
 end
 // synthesis translate_off
 initial if(FRAME_BYTES==0||FRAME_BYTES%16||MAX_RETRIES>15||TIMEOUT_CYCLES<2)$fatal(1,"D2E_TX_PARAMETERS");
 // synthesis translate_on
endmodule
