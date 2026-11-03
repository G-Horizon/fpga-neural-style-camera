`timescale 1ns/1ps
// Ethernet application shell; physical MAC/PHY and DDR remain outside.
module d2e_b_application_fast(
 input wire clk,clk_fast,rst_n,ddr_ready,phy_qualified,
 input wire[415:0]phy_status_words,input wire[31:0]phy_config_status,
 input wire[15:0]phy_config_before,phy_config_desired,phy_config_readback,
 output wire[7:0]mac_tx_data,output wire mac_tx_valid,input wire mac_tx_ready,output wire mac_tx_last,
 input wire[7:0]mac_rx_data,input wire mac_rx_valid,output wire mac_rx_ready,input wire mac_rx_last,input wire[10:0]mac_rx_length,
 input wire[3:0]i_spi0_io,output wire[3:0]o_spi0_io,oe_spi0_io,
 output wire o_spi_clk,oe_spi_clk,o_spi0_cs,oe_spi0_cs,
 output wire[239:0]cmd,input wire[137:0]rsp,output wire txd,fault
);
 wire[239:0]mcmd[0:2],joincmd;wire[137:0]mrsp[0:2],joinrsp;
 d64_axi_join u_compute_tx(.clk(clk),.rst_n(rst_n),.m0_cmd(mcmd[0]),.m0_rsp(mrsp[0]),.m1_cmd(mcmd[2]),.m1_rsp(mrsp[2]),.s_cmd(joincmd),.s_rsp(joinrsp));
 d64_axi_join u_rx_priority(.clk(clk),.rst_n(rst_n),.m0_cmd(mcmd[1]),.m0_rsp(mrsp[1]),.m1_cmd(joincmd),.m1_rsp(joinrsp),.s_cmd(cmd),.s_rsp(rsp));
 wire session_valid;wire[31:0]local_session,remote_session;
 wire job_valid,job_ready,job_done,job_done_ok,job_fps;wire[31:0]job_base,job_fid,job_epoch,job_done_fid;wire[1:0]job_style,job_layout;wire[7:0]job_effect;
 wire allocate_valid,allocate_ready,allocate_ok,allocate_fps;wire[31:0]allocate_base,allocate_fid,allocate_epoch;wire[1:0]allocate_style,allocate_layout;wire[7:0]allocate_effect;
 wire commit_valid,commit_ready,commit_cancel,commit_fps,abort_valid;wire[31:0]commit_base,commit_fid,commit_epoch,commit_crc,abort_base,abort_fid;
 wire[1:0]commit_style,commit_layout;wire[7:0]commit_effect;
 wire[31:0]net_sent,net_received,net_fail,rx_protocol,duplicates,sessions,restarts,rejected_headers;
 wire admission_ready,model_ready,compute_fault,network_fault;wire[3:0]assets_ready;wire[255:0]compute_status;wire[31:0]compute_flags;
 assign fault=compute_fault||network_fault;
 d2e_endpoint #(.ROLE(1),.OUT_STREAM(2),.IN_STREAM(3))u_network(
  .clk(clk),.rst_n(rst_n),.phy_qualified(phy_qualified),.application_ready(ddr_ready&&admission_ready&&!fault),
  .mac_tx_data(mac_tx_data),.mac_tx_valid(mac_tx_valid),.mac_tx_ready(mac_tx_ready),.mac_tx_last(mac_tx_last),
  .mac_rx_data(mac_rx_data),.mac_rx_valid(mac_rx_valid),.mac_rx_ready(mac_rx_ready),.mac_rx_last(mac_rx_last),.mac_rx_length(mac_rx_length),
  .session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .job_valid(job_valid),.job_ready(job_ready),.job_base(job_base),.job_fid(job_fid),.job_epoch(job_epoch),.job_style(job_style),.job_layout(job_layout),.job_fps(job_fps),.job_effect(job_effect),
  .job_done(job_done),.job_done_ok(job_done_ok),.job_done_fid(job_done_fid),
  .allocate_valid(allocate_valid),.allocate_ready(allocate_ready),.allocate_ok(allocate_ok),.allocate_base(allocate_base),.allocate_fid(allocate_fid),.allocate_epoch(allocate_epoch),
  .allocate_style(allocate_style),.allocate_layout(allocate_layout),.allocate_fps(allocate_fps),.allocate_effect(allocate_effect),
  .commit_valid(commit_valid),.commit_ready(commit_ready),.commit_cancel(commit_cancel),.commit_base(commit_base),.commit_fid(commit_fid),.commit_epoch(commit_epoch),.commit_crc(commit_crc),
  .commit_style(commit_style),.commit_layout(commit_layout),.commit_fps(commit_fps),.commit_effect(commit_effect),.abort_valid(abort_valid),.abort_base(abort_base),.abort_fid(abort_fid),
  .tx_axi_cmd(mcmd[0]),.tx_axi_rsp(mrsp[0]),.rx_axi_cmd(mcmd[1]),.rx_axi_rsp(mrsp[1]),.hard_fault(network_fault),
  .sent_frames(net_sent),.received_frames(net_received),.send_failures(net_fail),.receive_protocol_errors(rx_protocol),.duplicate_packets(duplicates),
  .established_sessions(sessions),.session_restarts(restarts),.rejected_headers(rejected_headers));
 d2e_b_compute_system_fast #(.WT_INIT_FILE(""))u_compute(
  .clk(clk),.clk_weight_fast(clk_fast),.rst_n(rst_n),.ddr_ready(ddr_ready),.session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .allocate_valid(allocate_valid),.allocate_ready(allocate_ready),.allocate_ok(allocate_ok),.allocate_base(allocate_base),.allocate_fid(allocate_fid),.allocate_epoch(allocate_epoch),
  .allocate_style(allocate_style),.allocate_layout(allocate_layout),.allocate_fps(allocate_fps),.allocate_effect(allocate_effect),
  .commit_valid(commit_valid),.commit_ready(commit_ready),.commit_cancel(commit_cancel),.commit_base(commit_base),.commit_fid(commit_fid),.commit_epoch(commit_epoch),.commit_crc(commit_crc),
  .commit_style(commit_style),.commit_layout(commit_layout),.commit_fps(commit_fps),.commit_effect(commit_effect),.abort_valid(abort_valid),.abort_base(abort_base),.abort_fid(abort_fid),
  .job_valid(job_valid),.job_ready(job_ready),.job_base(job_base),.job_fid(job_fid),.job_epoch(job_epoch),.job_style(job_style),.job_layout(job_layout),.job_fps(job_fps),.job_effect(job_effect),
  .tx_done(job_done),.tx_done_ok(job_done_ok),.tx_done_fid(job_done_fid),.axi_cmd(mcmd[2]),.axi_rsp(mrsp[2]),
  .i_spi0_io(i_spi0_io),.o_spi0_io(o_spi0_io),.oe_spi0_io(oe_spi0_io),.o_spi_clk(o_spi_clk),.oe_spi_clk(oe_spi_clk),.o_spi0_cs(o_spi0_cs),.oe_spi0_cs(oe_spi0_cs),
  .admission_ready(admission_ready),.model_ready(model_ready),.assets_ready(assets_ready),.fault_sticky(compute_fault),.status_words(compute_status),.status_flags(compute_flags));
 reg[31:0]cycles;always @(posedge clk or negedge rst_n)if(!rst_n)cycles<=0;else cycles<=cycles+1'b1;
 wire[767:0]status={cycles,rejected_headers,restarts,sessions,duplicates,rx_protocol,net_fail,net_sent,net_received,
  compute_status,compute_flags,{24'd0,assets_ready,model_ready,admission_ready,network_fault,fault},
  {31'd0,session_valid},remote_session,local_session,32'h42455645,32'd1};
 wire[1279:0]status_with_phy={{16'd0,phy_config_readback,phy_config_desired,phy_config_before},phy_config_status,phy_status_words,status};
 d64_status_uart #(.CLK_HZ(100000000),.WORDS(40),.PACKET_MAGIC(32'h31425645))u_status(.clk(clk),.rst_n(rst_n),.words(status_with_phy),.tx(txd));
endmodule
