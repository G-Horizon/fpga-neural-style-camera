`timescale 1ns/1ps
// Single-frame receiver. Packet bodies are validated before any DDR write;
// publication requires the final write response, whole-frame CRC and commit.
module d2e_frame_rx #(
 parameter integer EXPECTED_STREAM=1,FRAME_BYTES=147456
)(
 input wire clk,rst_n,session_valid,input wire[31:0]local_session,remote_session,
 input wire header_valid,output wire header_ready,input wire[479:0]header,output wire accept_body,
 input wire[7:0]body_data,input wire body_valid,output wire body_ready,input wire body_last,
 output reg response_valid,input wire response_ready,output reg[479:0]response_header,
 output wire allocate_valid,input wire allocate_ready,allocate_ok,input wire[31:0]allocate_base,
 output wire[31:0]allocate_fid,allocate_epoch,output wire[1:0]allocate_style,allocate_layout,
 output wire allocate_fps,output wire[7:0]allocate_effect,
 output wire commit_valid,input wire commit_ready,output wire[31:0]commit_base,commit_fid,commit_epoch,commit_crc,
 output wire[1:0]commit_style,commit_layout,output wire commit_fps,output wire[7:0]commit_effect,
 output wire commit_cancel,output reg abort_valid,output reg[31:0]abort_base,abort_fid,
 output reg[239:0]axi_cmd,input wire[137:0]axi_rsp,
 output reg[31:0]protocol_errors,duplicate_packets,committed_frames,output reg fault_sticky
);
 `include "d2e_protocol.vh"
 localparam IDLE=0,ALLOC=1,BODY=2,PREP=3,AW=4,READ_REQ=5,READ_LOAD=6,WRITE=7,BRESP=8,COMMIT=9,RESP=10;
 reg[3:0]state;
 reg active,leased,cancel_pending,duplicate_body,packet_bad;
 reg[479:0]request;
 reg[31:0]base,fid,epoch,local_q,remote_q,lease,lease_counter,expected_offset;
 reg[1:0]style,layout;reg fps;reg[7:0]effect;
 reg[31:0]frame_crc,pending_crc,chunk_crc,last_chunk_crc,last_offset;
 reg[15:0]packet_length,last_length,write_left;
 reg last_chunk_valid;
 reg[9:0]body_index,read_pointer;
 reg[31:0]write_address;
 reg[6:0]burst_beats,beat_index;
 reg[3:0]pack_byte;
 reg[127:0]pack_word;
 reg[7:0]memory_read;
 reg[7:0]packet_memory[0:959];
 reg last_success_valid;
 reg[479:0]last_success;
 wire session_lost=!session_valid||local_session!=local_q||remote_session!=remote_q;
 wire cancelled=cancel_pending||(active&&session_lost);
 wire common_ok=header[63:0]==`D2E_MAGIC&&header[71:64]==`D2E_VERSION&&
  header[87:80]==EXPECTED_STREAM&&header[95:88]==0&&header[127:96]==remote_session&&
  header[159:128]==local_session&&header[287:256]==FRAME_BYTES&&
  header[343:336]<=2&&header[351:344]<=2&&header[359:352]<=1&&
  header[383:368]==0&&header[479:448]==0;
 wire metadata_match=active&&header[127:96]==remote_q&&header[159:128]==local_q&&
  header[191:160]==fid&&header[223:192]==epoch&&header[343:336]=={6'd0,style}&&
  header[351:344]=={6'd0,layout}&&header[359:352]=={7'd0,fps}&&header[367:360]==effect;
 wire length_ok=header[335:320]!=0&&header[335:320]<=960&&header[335:320]<=FRAME_BYTES&&header[323:320]==0&&header[291:288]==0&&
  header[319:288]<=FRAME_BYTES-header[335:320];
 wire next_data=header[319:288]==expected_offset;
 wire previous_data=last_chunk_valid&&header[319:288]==last_offset&&header[335:320]==last_length;
 wire data_ok=common_ok&&metadata_match&&leased&&header[79:72]==`D2E_DATA&&
  header[447:416]==lease&&header[415:384]==0&&length_ok&&(next_data||previous_data);
 wire old_end_match=last_success_valid&&header[79:72]==`D2E_END&&header[335:320]==0&&
  header[319:288]==FRAME_BYTES&&header[127:96]==last_success[127:96]&&
  header[159:128]==last_success[159:128]&&header[223:160]==last_success[223:160]&&
  header[367:336]==last_success[367:336]&&header[447:384]==last_success[447:384];
 wire[8:0]boundary_beats=9'd256-{1'b0,write_address[11:4]};
 wire[8:0]remaining_beats={1'b0,write_left[11:4]};
 wire[8:0]bounded_beats=remaining_beats<boundary_beats?remaining_beats:boundary_beats;
 wire[6:0]next_burst=bounded_beats>60?7'd60:bounded_beats[6:0];
 wire[31:0]new_chunk_crc=d2e_crc_byte(chunk_crc,body_data);
 wire[31:0]new_frame_crc=d2e_crc_byte(pending_crc,body_data);
 assign header_ready=state==IDLE&&session_valid&&!cancelled;
 assign accept_body=header_ready&&data_ok;
 assign body_ready=state==BODY;
 assign allocate_valid=state==ALLOC;
 assign allocate_fid=fid;assign allocate_epoch=epoch;assign allocate_style=style;assign allocate_layout=layout;
 assign allocate_fps=fps;assign allocate_effect=effect;
 assign commit_valid=state==COMMIT;
 assign commit_base=base;assign commit_fid=fid;assign commit_epoch=epoch;assign commit_crc=~frame_crc;
 assign commit_style=style;assign commit_layout=layout;assign commit_fps=fps;assign commit_effect=effect;
 assign commit_cancel=state==COMMIT&&cancelled;
 always @*begin
  axi_cmd=0;axi_cmd[239]=1;
  axi_cmd[0]=state==AW;axi_cmd[32:1]=write_address;
  axi_cmd[40:33]=burst_beats-1'b1;axi_cmd[43:41]=4;axi_cmd[45:44]=1;
  axi_cmd[46]=state==WRITE;axi_cmd[174:47]=pack_word;axi_cmd[190:175]=16'hffff;
  axi_cmd[191]=beat_index==burst_beats-1'b1;axi_cmd[192]=state==BRESP;
 end
 // Dedicated synchronous byte RAM port, bounded to one 960-byte packet.
 always @(posedge clk)begin
  if(state==BODY&&body_valid&&body_ready)packet_memory[body_index]<=body_data;
  if(state==READ_REQ)memory_read<=packet_memory[read_pointer];
 end
 function[479:0]reply;
  input[479:0]h;input[7:0]kind,status;input[31:0]offset,crc,token;
  reg[479:0]v;
  begin
   v=h;v[79:72]=kind;v[95:88]=0;v[127:96]=h[159:128];v[159:128]=h[127:96];
   v[319:288]=offset;v[335:320]=0;v[375:368]=status;v[383:376]=0;
   v[415:384]=crc;v[447:416]=token;v[479:448]=0;reply=v;
  end
 endfunction
 task send_reply;
  input[479:0]h;input[7:0]kind,status;input[31:0]offset,crc,token;
  begin
   response_header<=reply(h,kind,status,offset,crc,token);response_valid<=1;state<=RESP;
   if(kind==`D2E_NAK)protocol_errors<=protocol_errors+1'b1;
  end
 endtask
 task abort_lease;
  begin
   if(leased)begin abort_valid<=1;abort_base<=base;abort_fid<=fid;end
   active<=0;leased<=0;cancel_pending<=0;last_chunk_valid<=0;
  end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;active<=0;leased<=0;cancel_pending<=0;duplicate_body<=0;packet_bad<=0;request<=0;
   base<=0;fid<=0;epoch<=0;local_q<=0;remote_q<=0;lease<=0;lease_counter<=0;expected_offset<=0;
   style<=0;layout<=0;fps<=0;effect<=0;frame_crc<=32'hffffffff;pending_crc<=32'hffffffff;
   chunk_crc<=32'hffffffff;last_chunk_crc<=0;last_offset<=0;packet_length<=0;last_length<=0;write_left<=0;
   last_chunk_valid<=0;body_index<=0;read_pointer<=0;write_address<=0;burst_beats<=0;beat_index<=0;
   pack_byte<=0;pack_word<=0;last_success_valid<=0;last_success<=0;
   response_valid<=0;response_header<=0;abort_valid<=0;abort_base<=0;abort_fid<=0;
   protocol_errors<=0;duplicate_packets<=0;committed_frames<=0;fault_sticky<=0;
  end else begin
   abort_valid<=0;
   if(active&&session_lost)cancel_pending<=1;
   case(state)
    IDLE:begin
     if(active&&cancelled)abort_lease();
     else if(header_valid&&header_ready)begin
      request<=header;
      if(!common_ok)send_reply(header,`D2E_NAK,2,expected_offset,0,lease);
      else case(header[79:72])
       `D2E_BEGIN:begin
        if(header[335:320]!=0||header[319:288]!=0||header[447:384]!=0)
         send_reply(header,`D2E_NAK,2,expected_offset,0,lease);
        else if(active)begin
         if(metadata_match&&leased)begin duplicate_packets<=duplicate_packets+1'b1;send_reply(header,`D2E_BEGIN_ACK,0,0,0,lease);end
         else send_reply(header,`D2E_NAK,1,expected_offset,0,lease);
        end else begin
         active<=1;leased<=0;cancel_pending<=0;fid<=header[191:160];epoch<=header[223:192];
         remote_q<=header[127:96];local_q<=header[159:128];style<=header[337:336];layout<=header[345:344];
         fps<=header[352];effect<=header[367:360];expected_offset<=0;last_chunk_valid<=0;frame_crc<=32'hffffffff;state<=ALLOC;
        end
       end
       `D2E_DATA:begin
        if(data_ok)begin
         packet_length<=header[335:320];body_index<=0;chunk_crc<=32'hffffffff;pending_crc<=frame_crc;
         duplicate_body<=!next_data;packet_bad<=0;state<=BODY;
        end else send_reply(header,`D2E_NAK,2,expected_offset,0,lease);
       end
       `D2E_END:begin
        if(old_end_match)begin duplicate_packets<=duplicate_packets+1'b1;send_reply(header,`D2E_END_ACK,0,FRAME_BYTES,header[415:384],header[447:416]);end
        else if(!metadata_match||!leased||header[447:416]!=lease||header[335:320]!=0||header[319:288]!=FRAME_BYTES||expected_offset!=FRAME_BYTES)
         send_reply(header,`D2E_NAK,2,expected_offset,0,lease);
        else if(header[415:384]!=~frame_crc)begin abort_lease();send_reply(header,`D2E_NAK,3,expected_offset,0,lease);end
        else state<=COMMIT;
       end
       `D2E_ABORT:begin
        if(metadata_match&&leased&&header[447:416]==lease&&header[335:320]==0)abort_lease();
        else send_reply(header,`D2E_NAK,2,expected_offset,0,lease);
       end
       default:send_reply(header,`D2E_NAK,2,expected_offset,0,lease);
      endcase
     end
    end
    ALLOC:if(allocate_valid&&allocate_ready)begin
     if(cancelled)begin
      if(allocate_ok)begin abort_valid<=1;abort_base<=allocate_base;abort_fid<=fid;end
      active<=0;leased<=0;cancel_pending<=0;state<=IDLE;
     end else if(!allocate_ok)begin active<=0;send_reply(request,`D2E_NAK,1,0,0,0);end
     else if(allocate_base[3:0]!=0||allocate_base>32'hffffffff-FRAME_BYTES+1)begin
      abort_valid<=1;abort_base<=allocate_base;abort_fid<=fid;active<=0;fault_sticky<=1;send_reply(request,`D2E_NAK,4,0,0,0);
     end else begin
      base<=allocate_base;leased<=1;
      lease_counter<=lease_counter==32'hffffffff ? 32'd1:lease_counter+1'b1;
      lease<=lease_counter==32'hffffffff ? 32'd1:lease_counter+1'b1;
      send_reply(request,`D2E_BEGIN_ACK,0,0,0,lease_counter==32'hffffffff ? 32'd1:lease_counter+1'b1);
     end
    end
    BODY:if(body_valid&&body_ready)begin
     chunk_crc<=new_chunk_crc;
     if(!duplicate_body)pending_crc<=new_frame_crc;
     if(body_last!=(body_index==packet_length-1'b1))packet_bad<=1;
     if(body_last||body_index==packet_length-1'b1)begin
      if(cancelled)begin abort_lease();state<=IDLE;end
      else if(packet_bad||body_last!=(body_index==packet_length-1'b1))send_reply(request,`D2E_NAK,3,expected_offset,0,lease);
      else if(duplicate_body)begin
       if(new_chunk_crc==last_chunk_crc)begin duplicate_packets<=duplicate_packets+1'b1;send_reply(request,`D2E_DATA_ACK,0,expected_offset,0,lease);end
       else send_reply(request,`D2E_NAK,3,expected_offset,0,lease);
      end else begin write_left<=packet_length;write_address<=base+request[319:288];read_pointer<=0;state<=PREP;end
     end else body_index<=body_index+1'b1;
    end
    PREP:begin
     if(cancelled)begin abort_lease();state<=IDLE;end
     else begin burst_beats<=next_burst;beat_index<=0;state<=AW;end
    end
    AW:if(axi_cmd[0]&&axi_rsp[0])begin pack_byte<=0;state<=READ_REQ;end
    READ_REQ:state<=READ_LOAD;
    READ_LOAD:begin
     pack_word[pack_byte*8+:8]<=memory_read;read_pointer<=read_pointer+1'b1;
     if(pack_byte==15)state<=WRITE;
     else begin pack_byte<=pack_byte+1'b1;state<=READ_REQ;end
    end
    WRITE:if(axi_cmd[46]&&axi_rsp[1])begin
     write_left<=write_left-16;
     if(beat_index==burst_beats-1'b1)state<=BRESP;
     else begin beat_index<=beat_index+1'b1;pack_byte<=0;state<=READ_REQ;end
    end
    BRESP:if(axi_cmd[192]&&axi_rsp[2])begin
     if(axi_rsp[4:3]!=0)fault_sticky<=1;
     if(cancelled)begin abort_lease();state<=IDLE;end
     else if(axi_rsp[4:3]!=0)begin abort_lease();send_reply(request,`D2E_NAK,4,expected_offset,0,lease);end
     else if(write_left==0)begin
      expected_offset<=expected_offset+packet_length;frame_crc<=pending_crc;
      last_chunk_valid<=1;last_offset<=request[319:288];last_length<=packet_length;last_chunk_crc<=chunk_crc;
      send_reply(request,`D2E_DATA_ACK,0,expected_offset+packet_length,0,lease);
     end else begin write_address<=write_address+({25'd0,burst_beats}<<4);state<=PREP;end
    end
    COMMIT:if(commit_valid&&commit_ready)begin
     if(commit_cancel)begin abort_lease();state<=IDLE;end
     else begin
      committed_frames<=committed_frames+1'b1;last_success_valid<=1;last_success<=request;
      active<=0;leased<=0;last_chunk_valid<=0;cancel_pending<=0;
      send_reply(request,`D2E_END_ACK,0,FRAME_BYTES,~frame_crc,lease);
     end
    end
    RESP:if(response_valid&&response_ready)begin response_valid<=0;state<=IDLE;end
    default:begin fault_sticky<=1;state<=IDLE;end
   endcase
  end
 end
 // synthesis translate_off
 initial if(FRAME_BYTES<16||FRAME_BYTES%16||EXPECTED_STREAM<1||EXPECTED_STREAM>3)$fatal(1,"D2E_RX_PARAMETERS");
 // synthesis translate_on
endmodule
