`timescale 1ns/1ps
module d2e_a_system(
 input wire clk,clk_fast,rst_n,ddr_ready,cam_clk,cam_rst_n,pix_clk,pix_rst_n,
 input wire capture_core_rst_n,capture_cam_rst_n,output wire capture_release_n,
 input wire valid,input wire[95:0]rgb4,input wire[10:0]x0,y,input wire[31:0]fid,
 input wire sof,eof,source_bad,input wire[3:0]keys_n,
 input wire diag_packed_bad,diag_isp_bad,input wire[7:0]diag_bridge_code,
 input wire phy_qualified,input wire[8:0]diag_ethernet_app,
 input wire[415:0]phy_status_words,input wire[31:0]phy_config_status,
 input wire[15:0]phy_config_before,phy_config_desired,phy_config_readback,
 input wire[7:0]diag_sensor_orientation,
 output wire[7:0]mac_tx_data,output wire mac_tx_valid,input wire mac_tx_ready,output wire mac_tx_last,
 input wire[7:0]mac_rx_data,input wire mac_rx_valid,output wire mac_rx_ready,input wire mac_rx_last,input wire[10:0]mac_rx_length,
 input wire[3:0]i_spi0_io,output wire[3:0]o_spi0_io,oe_spi0_io,
 output wire o_spi_clk,oe_spi_clk,o_spi0_cs,oe_spi0_cs,
 output wire[9:0]tmds0,tmds1,tmds2,tmdsclk,output wire txd,fault,
 output wire[239:0]cmd,input wire[137:0]rsp
);
 wire[239:0]mcmd[0:6],joincmd[0:5];wire[137:0]mrsp[0:6],joinrsp[0:5];
 genvar g;generate for(g=0;g<6;g=g+1)begin:bus_join
  wire[239:0]left=g==0?mcmd[0]:joincmd[g-1];wire[137:0]lr;
  if(g==0)assign mrsp[0]=lr;else assign joinrsp[g-1]=lr;
  d64_axi_join u_join(.clk(clk),.rst_n(rst_n),.m0_cmd(mcmd[g+1]),.m0_rsp(mrsp[g+1]),.m1_cmd(left),.m1_rsp(lr),.s_cmd(joincmd[g]),.s_rsp(joinrsp[g]));
 end endgenerate
 assign cmd=joincmd[5];assign joinrsp[5]=rsp;
 wire cap_commit,release_valid;wire[2:0]cap_slot,release_slot;wire[31:0]cap_fid,captured,dropped;
 wire capture_fault,capture_hard_fault,capture_quiescent,compute_fault,input_fault,display_fault,owner_fault,network_fault;
 wire recovering;wire[31:0]recovery_count,recovery_fault_count;
 camera_capture_recovery u_recovery(.clk(clk),.rst_n(rst_n),.fault(capture_fault&&!capture_hard_fault),.quiescent(capture_quiescent&&!capture_hard_fault),
  .pipeline_release_n(capture_release_n),.recovery_count(recovery_count),.fault_count(recovery_fault_count),.recovering(recovering));
 reg[7:0]owned_capture,recovery_owned;reg capture_reset_seen;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin owned_capture<=0;recovery_owned<=0;capture_reset_seen<=0;end
  else begin
   if(cap_commit)owned_capture[cap_slot]<=1;
   if(release_valid)owned_capture[release_slot]<=0;
   capture_reset_seen<=!capture_core_rst_n;
   if(!capture_core_rst_n&&!capture_reset_seen)recovery_owned<=
    (owned_capture|(cap_commit?(8'b1<<cap_slot):8'd0))&~(release_valid?(8'b1<<release_slot):8'd0);
  end
 wire[191:0]diag_service,diag_capture;wire[95:0]diag_camera;wire diag_saturated,diag_camera_valid;
 d2_a_capture #(.PREVIEW_CAPTURE_ENABLE(0),.SLOT_AW(3))u_capture(
  .cam_clk(cam_clk),.cam_rst_n(cam_rst_n),.clk(clk),.rst_n(rst_n),.pipeline_core_rst_n(capture_core_rst_n),.pipeline_cam_rst_n(capture_cam_rst_n),
  .recovering(recovering),.preserve_slots(recovery_owned),.hard_fault(capture_hard_fault),.quiescent(capture_quiescent),
  .valid(valid),.rgb4(rgb4),.x0(x0),.y(y),.fid(fid),.sof(sof),.eof(eof),.source_bad(source_bad),
  .release_valid(release_valid),.release_slot(release_slot),.committed(cap_commit),.committed_slot(cap_slot),.committed_fid(cap_fid),
  .diag_packed_bad(diag_packed_bad),.diag_isp_bad(diag_isp_bad),.diag_bridge_code(diag_bridge_code),.diag_core(diag_capture),.diag_cam(diag_camera),.diag_cam_valid(diag_camera_valid),
  .diag_service(diag_service),.diag_service_saturated(diag_saturated),.fault(capture_fault),.captured(captured),.dropped(dropped),.cmd(mcmd[4]),.rsp(mrsp[4]));
 reg[1:0]ui_style;reg ui_fps;reg[31:0]ui_epoch;
 wire key_request,key_fps;wire[1:0]key_style,loaded_profile,compute_profile;
 wire compute_ready,compute_done,compute_start,input_start,model_loaded;
 wire[2:0]compute_slot;wire[31:0]compute_fid;wire[127:0]diag_compute;
 d64_ui_keys u_keys(.clk(clk),.rst_n(rst_n),.keys_n(keys_n),.active_style(loaded_profile),.model_valid(model_loaded),
  .external_style_valid(1'b0),.external_next(1'b0),.external_style(2'd0),.request_valid(key_request),.request_style(key_style),.fps_toggle(key_fps),
  .compare_toggle(),.effect_trigger(),.target_style(),.target_pending());
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin ui_style<=1;ui_fps<=1;ui_epoch<=0;end
  else begin
   if(key_request)ui_style<=key_style;
   if(key_fps)ui_fps<=!ui_fps;
   if(key_request||key_fps)ui_epoch<=ui_epoch+1'b1;
  end
 wire input_valid,input_ready,input_sof,input_eol,input_eof,input_done;wire[383:0]input_rgb;wire[31:0]input_fid;wire[7:0]input_x,input_y;
 wire[31:0]capture_base=32'h01000000+{7'd0,compute_slot,22'd0};
 d2_a_input_reader u_input(.clk(clk),.rst_n(rst_n),.start(input_start),.base(capture_base+32'h340000),.fid(compute_fid),
  .out_valid(input_valid),.out_ready(input_ready),.out_rgb(input_rgb),.out_fid(input_fid),.out_x(input_x),.out_y(input_y),
  .out_sof(input_sof),.out_eol(input_eol),.out_eof(input_eof),.done(input_done),.failed(input_fault),.cmd(mcmd[0]),.rsp(mrsp[0]));
 d2_a_compute_encoder_only u_compute(.clk_pe(clk),.clk_weight_fast(clk_fast),.rst_pe_n(rst_n),.ddr_ready(ddr_ready),.requested_style(compute_profile),
  .start(compute_start),.tensor_base(capture_base+32'h300000),.ready(compute_ready),.model_loaded(model_loaded),.active_profile(loaded_profile),
  .input_start(input_start),.done(compute_done),.failed(compute_fault),.diag_compute(diag_compute),
  .d64_valid(input_valid),.d64_ready(input_ready),.d64_rgb888(input_rgb),.d64_frame_id(input_fid),.d64_x0(input_x),.d64_y(input_y),
  .d64_sof(input_sof),.d64_eol(input_eol),.d64_eof(input_eof),.axi_cmd(mcmd[1]),.axi_rsp(mrsp[1]),
  .i_spi0_io(i_spi0_io),.o_spi0_io(o_spi0_io),.oe_spi0_io(oe_spi0_io),.o_spi_clk(o_spi_clk),.oe_spi_clk(oe_spi_clk),.o_spi0_cs(o_spi0_cs),.oe_spi0_cs(oe_spi0_cs));
 reg assets_booted;
 always @(posedge clk or negedge rst_n)if(!rst_n)assets_booted<=0;else if(model_loaded)assets_booted<=1;
 wire session_valid;wire[31:0]local_session,remote_session;
 wire job_valid,job_ready,job_done,job_done_ok,job_fps;wire[31:0]job_base,job_fid,job_epoch,job_done_fid;wire[1:0]job_style,job_layout;wire[7:0]job_effect;
 wire allocate_valid,allocate_ready,allocate_ok,allocate_fps;wire[31:0]allocate_base,allocate_fid,allocate_epoch;wire[1:0]allocate_style,allocate_layout;wire[7:0]allocate_effect;
 wire commit_valid,commit_ready,commit_cancel,commit_fps,abort_valid;wire[31:0]commit_base,commit_fid,commit_epoch,commit_crc,abort_base,abort_fid;
 wire[1:0]commit_style,commit_layout;wire[7:0]commit_effect;
 wire[31:0]net_sent,net_received,net_fail,rx_protocol,duplicates,sessions,restarts,rejected_headers;
 d2e_endpoint #(.ROLE(0),.OUT_STREAM(3),.IN_STREAM(2))u_network(
  .clk(clk),.rst_n(rst_n),.phy_qualified(phy_qualified),.application_ready(ddr_ready&&assets_booted&&!fault),
  .mac_tx_data(mac_tx_data),.mac_tx_valid(mac_tx_valid),.mac_tx_ready(mac_tx_ready),.mac_tx_last(mac_tx_last),
  .mac_rx_data(mac_rx_data),.mac_rx_valid(mac_rx_valid),.mac_rx_ready(mac_rx_ready),.mac_rx_last(mac_rx_last),.mac_rx_length(mac_rx_length),
  .session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .job_valid(job_valid),.job_ready(job_ready),.job_base(job_base),.job_fid(job_fid),.job_epoch(job_epoch),.job_style(job_style),.job_layout(job_layout),.job_fps(job_fps),.job_effect(job_effect),
  .job_done(job_done),.job_done_ok(job_done_ok),.job_done_fid(job_done_fid),
  .allocate_valid(allocate_valid),.allocate_ready(allocate_ready),.allocate_ok(allocate_ok),.allocate_base(allocate_base),.allocate_fid(allocate_fid),.allocate_epoch(allocate_epoch),
  .allocate_style(allocate_style),.allocate_layout(allocate_layout),.allocate_fps(allocate_fps),.allocate_effect(allocate_effect),
  .commit_valid(commit_valid),.commit_ready(commit_ready),.commit_cancel(commit_cancel),.commit_base(commit_base),.commit_fid(commit_fid),.commit_epoch(commit_epoch),.commit_crc(commit_crc),
  .commit_style(commit_style),.commit_layout(commit_layout),.commit_fps(commit_fps),.commit_effect(commit_effect),.abort_valid(abort_valid),.abort_base(abort_base),.abort_fid(abort_fid),
  .tx_axi_cmd(mcmd[2]),.tx_axi_rsp(mrsp[2]),.rx_axi_cmd(mcmd[3]),.rx_axi_rsp(mrsp[3]),.hard_fault(network_fault),
  .sent_frames(net_sent),.received_frames(net_received),.send_failures(net_fail),.receive_protocol_errors(rx_protocol),.duplicate_packets(duplicates),
  .established_sessions(sessions),.session_restarts(restarts),.rejected_headers(rejected_headers),.diag_state(network_diag));
 wire publish_valid,publish_ready,publish_fps,active_valid;wire[31:0]publish_base,publish_fid,publish_epoch,active_base,active_fid,active_epoch;
 wire[2:0]publish_slot;wire[7:0]reader_mask,owned_mask;wire[31:0]computed,returned,published,retired,discarded,complete_frames,distinct_frames,rejected_frames;
 d2e_a_slots u_slots(.clk(clk),.rst_n(rst_n),.session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .capture_commit(cap_commit),.capture_slot(cap_slot),.capture_fid(cap_fid),.release_valid(release_valid),.release_slot(release_slot),.owned_mask(owned_mask),
  .ui_style(ui_style),.ui_fps(ui_fps),.ui_effect(8'd0),.ui_epoch(ui_epoch),.compute_ready(compute_ready),.compute_done(compute_done),.compute_fault(compute_fault),
  .compute_start(compute_start),.compute_slot(compute_slot),.compute_profile(compute_profile),.compute_fid(compute_fid),
  .tx_job_valid(job_valid),.tx_job_ready(job_ready),.tx_base(job_base),.tx_fid(job_fid),.tx_epoch(job_epoch),.tx_style(job_style),.tx_layout(job_layout),.tx_fps(job_fps),.tx_effect(job_effect),
  .tx_done(job_done),.tx_done_ok(job_done_ok),.tx_done_fid(job_done_fid),
  .allocate_valid(allocate_valid),.allocate_ready(allocate_ready),.allocate_ok(allocate_ok),.allocate_base(allocate_base),.allocate_fid(allocate_fid),.allocate_epoch(allocate_epoch),
  .allocate_style(allocate_style),.allocate_layout(allocate_layout),.allocate_fps(allocate_fps),.allocate_effect(allocate_effect),
  .commit_valid(commit_valid),.commit_ready(commit_ready),.commit_cancel(commit_cancel),.commit_base(commit_base),.commit_fid(commit_fid),.commit_epoch(commit_epoch),
  .commit_style(commit_style),.commit_layout(commit_layout),.commit_fps(commit_fps),.commit_effect(commit_effect),.abort_valid(abort_valid),.abort_base(abort_base),.abort_fid(abort_fid),
  .publish_valid(publish_valid),.publish_ready(publish_ready),.publish_base(publish_base),.publish_fid(publish_fid),.publish_epoch(publish_epoch),.publish_slot(publish_slot),.publish_fps(publish_fps),
  .active_valid(active_valid),.active_fid(active_fid),.active_base(active_base),.active_epoch(active_epoch),.reader_protect_mask(reader_mask),.fault(owner_fault),
  .computed_frames(computed),.returned_frames(returned),.published_frames(published),.retired_frames(retired),.discarded_frames(discarded));
 wire[50:0]stage_rate_x100;wire[2:0]stage_rate_valid;
 d2e_a_display u_display(.clk(clk),.rst_n(rst_n),.pix_clk(pix_clk),.pix_rst_n(pix_rst_n),
  .camera_complete_count(captured),.style_new_count(published),.stage_rate_x100_count(stage_rate_x100),.stage_rate_valid_count(stage_rate_valid),
  .publish_valid(publish_valid),.publish_ready(publish_ready),.publish_base(publish_base),.publish_fid(publish_fid),.publish_epoch(publish_epoch),.publish_slot(publish_slot),.publish_fps(publish_fps),
  .active_valid(active_valid),.active_fid(active_fid),.active_base(active_base),.active_epoch(active_epoch),.reader_protect_mask(reader_mask),.fault(display_fault),
  .style_cmd(mcmd[5]),.style_rsp(mrsp[5]),.guide_cmd(mcmd[6]),.guide_rsp(mrsp[6]),.tmds0(tmds0),.tmds1(tmds1),.tmds2(tmds2),.tmdsclk(tmdsclk),
  .complete_frames(complete_frames),.distinct_frames(distinct_frames),.rejected_frames(rejected_frames));
 assign fault=capture_hard_fault||compute_fault||input_fault||owner_fault||network_fault||display_fault;
 reg[31:0]cycles;always @(posedge clk or negedge rst_n)if(!rst_n)cycles<=0;else cycles<=cycles+1'b1;

 wire[95:0]network_diag;
 wire[6:0]leaf_flags={mrsp[2][137:136],mrsp[2][135],mcmd[2][239],mrsp[2][6],mrsp[2][5],mcmd[2][193]};
 wire[6:0]ddr_flags={rsp[137:136],rsp[135],cmd[239],rsp[6],rsp[5],cmd[193]};
 reg[1:0]leaf_ar_events,leaf_r_events,ddr_ar_events,ddr_r_events;
 reg observer_armed,observer_frozen,observer_previous_session;
 reg[127:0]observer_first_fall;
 wire[31:0]observer_bus={ddr_r_events,ddr_ar_events,leaf_r_events,leaf_ar_events,diag_ethernet_app,phy_qualified,ddr_flags,leaf_flags};
 wire[127:0]observer_live={observer_bus,observer_frozen,observer_armed,network_diag[93:0]};
 wire[127:0]observer_fall_output={observer_first_fall[127:96],observer_frozen,observer_armed,observer_first_fall[93:0]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   leaf_ar_events<=0;leaf_r_events<=0;ddr_ar_events<=0;ddr_r_events<=0;
   observer_armed<=0;observer_frozen<=0;observer_previous_session<=0;observer_first_fall<=0;
  end else begin
   if(mcmd[2][193]&&mrsp[2][5])leaf_ar_events<=leaf_ar_events+1'b1;
   if(mcmd[2][239]&&mrsp[2][6])leaf_r_events<=leaf_r_events+1'b1;
   if(cmd[193]&&rsp[5])ddr_ar_events<=ddr_ar_events+1'b1;
   if(cmd[239]&&rsp[6])ddr_r_events<=ddr_r_events+1'b1;
   observer_previous_session<=session_valid;
   if(session_valid)observer_armed<=1;
   if(observer_armed&&observer_previous_session&&!session_valid&&!observer_frozen)begin
    observer_first_fall<=observer_live;observer_frozen<=1;
   end
  end
 end
 wire[1:0]metric_select=cycles[27:26];
 wire[16:0]metric_debug=metric_select==0?stage_rate_x100[50:34]:metric_select==1?stage_rate_x100[33:17]:metric_select==2?stage_rate_x100[16:0]:17'd0;
 wire[31:0]stage_debug_word={diag_sensor_orientation,stage_rate_valid,metric_select,metric_debug,2'd0};
 wire[767:0]status={cycles,rejected_headers,restarts,sessions,duplicates,rx_protocol,net_fail,{24'd0,owned_mask},
  {31'd0,session_valid},remote_session,local_session,rejected_frames,distinct_frames,complete_frames,published,returned,net_sent,computed,dropped,captured,
  {24'd0,recovering,capture_hard_fault,compute_fault,input_fault,owner_fault,network_fault,display_fault,fault},stage_debug_word,32'h41455645,32'd3};
 wire[1279:0]status_with_phy={{16'd0,phy_config_readback,phy_config_desired,phy_config_before},phy_config_status,phy_status_words,status};
 d64_status_uart #(.CLK_HZ(100000000),.WORDS(48),.PACKET_MAGIC(32'h33415645))u_status(.clk(clk),.rst_n(rst_n),.words({observer_fall_output,observer_live,status_with_phy}),.tx(txd));
endmodule
