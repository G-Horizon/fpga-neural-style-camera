`timescale 1ns/1ps
// One app-domain endpoint above the complete-packet MAC. The two AXI masters
// are intentionally separate; the system joins them with existing bus owners.
module d2e_endpoint #(
 parameter integer ROLE=0,OUT_STREAM=1,IN_STREAM=2,FRAME_BYTES=147456,
 parameter integer ACK_TIMEOUT=100000,ACK_RETRIES=8,
 parameter integer SESSION_QUIET=100000,SESSION_RETRY=1000000,SESSION_PING=10000000,SESSION_DEAD=50000000
)(
 input wire clk,rst_n,phy_qualified,application_ready,
 output wire[7:0]mac_tx_data,output wire mac_tx_valid,input wire mac_tx_ready,output wire mac_tx_last,
 input wire[7:0]mac_rx_data,input wire mac_rx_valid,output wire mac_rx_ready,input wire mac_rx_last,input wire[10:0]mac_rx_length,
 output wire session_valid,output wire[31:0]local_session,remote_session,
 input wire job_valid,output wire job_ready,input wire[31:0]job_base,job_fid,job_epoch,
 input wire[1:0]job_style,job_layout,input wire job_fps,input wire[7:0]job_effect,
 output wire job_done,job_done_ok,output wire[31:0]job_done_fid,
 output wire allocate_valid,input wire allocate_ready,allocate_ok,input wire[31:0]allocate_base,
 output wire[31:0]allocate_fid,allocate_epoch,output wire[1:0]allocate_style,allocate_layout,output wire allocate_fps,output wire[7:0]allocate_effect,
 output wire commit_valid,input wire commit_ready,output wire[31:0]commit_base,commit_fid,commit_epoch,commit_crc,
 output wire[1:0]commit_style,commit_layout,output wire commit_fps,output wire[7:0]commit_effect,
 output wire commit_cancel,output wire abort_valid,output wire[31:0]abort_base,abort_fid,
 output wire[239:0]tx_axi_cmd,input wire[137:0]tx_axi_rsp,
 output wire[239:0]rx_axi_cmd,input wire[137:0]rx_axi_rsp,
 output wire hard_fault,output reg[31:0]sent_frames,output wire[31:0]received_frames,
 output reg[31:0]send_failures,output wire[31:0]receive_protocol_errors,duplicate_packets,
 output wire[31:0]established_sessions,session_restarts,rejected_headers
);
 `include "d2e_protocol.vh"
 wire[479:0]received_header;
 wire header_valid,header_ready,has_body,accept_body;
 wire[7:0]received_body;wire body_valid,body_ready,body_last;
 wire rx_packet_done,rx_codec_fault,tx_codec_fault;
 d2e_header_rx u_header_rx(.clk(clk),.rst_n(rst_n),.s_data(mac_rx_data),.s_valid(mac_rx_valid),.s_ready(mac_rx_ready),
  .s_last(mac_rx_last),.s_length(mac_rx_length),.header_valid(header_valid),.header_ready(header_ready),.accept_body(accept_body),
  .header(received_header),.header_has_body(has_body),.m_data(received_body),.m_valid(body_valid),.m_ready(body_ready),.m_last(body_last),
  .packet_done(rx_packet_done),.rejected_headers(rejected_headers),.protocol_fault(rx_codec_fault));
 wire session_control=received_header[87:80]==0&&(received_header[79:72]==`D2E_HELLO||received_header[79:72]==`D2E_HELLO_ACK)&&!has_body;
 wire frame_response=received_header[87:80]==OUT_STREAM&&!has_body&&
  (received_header[79:72]==`D2E_BEGIN_ACK||received_header[79:72]==`D2E_DATA_ACK||received_header[79:72]==`D2E_END_ACK||received_header[79:72]==`D2E_NAK);
 wire frame_request=session_valid&&received_header[127:96]==remote_session&&received_header[159:128]==local_session&&
  received_header[87:80]==IN_STREAM&&(received_header[79:72]==`D2E_BEGIN||received_header[79:72]==`D2E_DATA||received_header[79:72]==`D2E_END||received_header[79:72]==`D2E_ABORT);
 wire session_control_ready,frame_header_ready,frame_accept_body;
 assign header_ready=session_control?session_control_ready:frame_request?frame_header_ready:1'b1;
 assign accept_body=frame_request&&frame_accept_body;
 wire session_packet_valid,session_packet_ready;wire[479:0]session_packet;
 wire tx_failed=job_done&&!job_done_ok;
 wire[31:0]rejected_controls;
 d2e_session #(.ROLE(ROLE),.QUIET_CYCLES(SESSION_QUIET),.RETRY_CYCLES(SESSION_RETRY),.PING_CYCLES(SESSION_PING),.DEAD_CYCLES(SESSION_DEAD))u_session(
  .clk(clk),.rst_n(rst_n),.link_qualified(phy_qualified&&application_ready),.force_restart(tx_failed),
  .control_valid(header_valid&&session_control),.control_ready(session_control_ready),.control_header(received_header),
  .packet_valid(session_packet_valid),.packet_ready(session_packet_ready),.packet_header(session_packet),
  .session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .established_sessions(established_sessions),.restarts(session_restarts),.rejected_controls(rejected_controls));
 wire frame_packet_valid,frame_packet_ready,frame_packet_done;wire[479:0]frame_packet;
 wire[7:0]tx_body;wire tx_body_valid,tx_body_ready,tx_fault;
 wire[31:0]tx_inner_frames,tx_retries,tx_ignored;
 reg[1:0]tx_recover_hold;
 // done-failure is emitted only after the current packet was fully assembled
 // and all issued AXI reads retired. Reinitialize this app-only sender then;
 // MAC banks, receiver and unrelated bus owners retain their state.
 always @(posedge clk or negedge rst_n)
  if(!rst_n)tx_recover_hold<=0;
  else if(tx_failed&&tx_fault)tx_recover_hold<=3;
  else if(tx_recover_hold!=0)tx_recover_hold<=tx_recover_hold-1'b1;
 wire tx_local_rst_n=rst_n&&(tx_recover_hold==0);
 d2e_frame_tx #(.FRAME_BYTES(FRAME_BYTES),.STREAM(OUT_STREAM),.TIMEOUT_CYCLES(ACK_TIMEOUT),.MAX_RETRIES(ACK_RETRIES))u_frame_tx(
  .clk(clk),.rst_n(tx_local_rst_n),.session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .job_valid(job_valid),.job_ready(job_ready),.job_base(job_base),.job_fid(job_fid),.job_epoch(job_epoch),
  .job_style(job_style),.job_layout(job_layout),.job_fps(job_fps),.job_effect(job_effect),
  .done(job_done),.done_ok(job_done_ok),.done_fid(job_done_fid),
  .packet_valid(frame_packet_valid),.packet_ready(frame_packet_ready),.packet_header(frame_packet),
  .body_data(tx_body),.body_valid(tx_body_valid),.body_ready(tx_body_ready),.packet_done(frame_packet_done),
  .response_valid(header_valid&&header_ready&&frame_response),.response_header(received_header),
  .axi_cmd(tx_axi_cmd),.axi_rsp(tx_axi_rsp),.fault_sticky(tx_fault),.completed_frames(tx_inner_frames),.retransmissions(tx_retries),.ignored_responses(tx_ignored));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sent_frames<=0;send_failures<=0;end
  else if(job_done)begin if(job_done_ok)sent_frames<=sent_frames+1'b1;else send_failures<=send_failures+1'b1;end
 end
 wire response_valid,response_ready;wire[479:0]response_header;wire receive_fault;
 d2e_frame_rx #(.EXPECTED_STREAM(IN_STREAM),.FRAME_BYTES(FRAME_BYTES))u_frame_rx(
  .clk(clk),.rst_n(rst_n),.session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .header_valid(header_valid&&frame_request),.header_ready(frame_header_ready),.header(received_header),.accept_body(frame_accept_body),
  .body_data(received_body),.body_valid(body_valid),.body_ready(body_ready),.body_last(body_last),
  .response_valid(response_valid),.response_ready(response_ready),.response_header(response_header),
  .allocate_valid(allocate_valid),.allocate_ready(allocate_ready),.allocate_ok(allocate_ok),.allocate_base(allocate_base),
  .allocate_fid(allocate_fid),.allocate_epoch(allocate_epoch),.allocate_style(allocate_style),.allocate_layout(allocate_layout),.allocate_fps(allocate_fps),.allocate_effect(allocate_effect),
  .commit_valid(commit_valid),.commit_ready(commit_ready),.commit_base(commit_base),.commit_fid(commit_fid),.commit_epoch(commit_epoch),.commit_crc(commit_crc),
  .commit_style(commit_style),.commit_layout(commit_layout),.commit_fps(commit_fps),.commit_effect(commit_effect),.commit_cancel(commit_cancel),
  .abort_valid(abort_valid),.abort_base(abort_base),.abort_fid(abort_fid),.axi_cmd(rx_axi_cmd),.axi_rsp(rx_axi_rsp),
  .protocol_errors(receive_protocol_errors),.duplicate_packets(duplicate_packets),.committed_frames(received_frames),.fault_sticky(receive_fault));
 wire response_is_current=session_valid&&response_header[127:96]==local_session&&response_header[159:128]==remote_session;
 // Old-session responses must be consumed, even while the PHY is down. The
 // RX lease cancellation path waits for this standard valid/ready handshake.
 wire drop_response=response_valid&&!response_is_current;
 wire[1:0]selection=session_packet_valid?0:response_valid&&!drop_response?1:2;
 wire format_ready,format_done;
 wire format_request=session_packet_valid||(response_valid&&!drop_response)||frame_packet_valid;
 wire[479:0]format_header=selection==0?session_packet:selection==1?response_header:frame_packet;
 assign session_packet_ready=format_ready&&selection==0;
 assign response_ready=drop_response||(format_ready&&selection==1&&!session_packet_valid);
 assign frame_packet_ready=format_ready&&selection==2&&!session_packet_valid&&!(response_valid&&!drop_response);
 reg[1:0]format_owner;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)format_owner<=0;
  else if(format_request&&format_ready)format_owner<=selection;
 assign frame_packet_done=format_done&&format_owner==2;
 wire formatter_body_ready;
 assign tx_body_ready=formatter_body_ready&&format_owner==2;
 d2e_header_tx u_header_tx(.clk(clk),.rst_n(rst_n),.request(format_request),.request_ready(format_ready),.header(format_header),
  .s_data(tx_body),.s_valid(tx_body_valid&&format_owner==2),.s_ready(formatter_body_ready),
  .m_data(mac_tx_data),.m_valid(mac_tx_valid),.m_ready(mac_tx_ready),.m_last(mac_tx_last),.packet_done(format_done),.protocol_fault(tx_codec_fault));
 assign hard_fault=receive_fault||rx_codec_fault||tx_codec_fault;
endmodule
