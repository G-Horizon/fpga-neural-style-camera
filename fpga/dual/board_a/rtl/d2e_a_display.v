`timescale 1ns/1ps
// Same source-resolution guide and half-strength fusion as the HDMI product.
module d2e_a_display(
 input wire clk,rst_n,pix_clk,pix_rst_n,
 input wire[31:0]camera_complete_count,style_new_count,
 output wire[50:0]stage_rate_x100_count,output wire[2:0]stage_rate_valid_count,
 input wire publish_valid,output wire publish_ready,input wire[31:0]publish_base,publish_fid,publish_epoch,
 input wire[2:0]publish_slot,input wire publish_fps,
 output wire[31:0]active_base,active_fid,active_epoch,output wire active_valid,
 output wire[7:0]reader_protect_mask,output wire fault,
 output wire[239:0]style_cmd,guide_cmd,input wire[137:0]style_rsp,guide_rsp,
 output wire[9:0]tmds0,tmds1,tmds2,tmdsclk,
 output wire[31:0]complete_frames,distinct_frames,rejected_frames
);
 wire[31:0]rows,underrun,axi_errors,bad_rows,guide_faults;
 wire starved,guide_axi_fault,guide_protocol_fault,guide_outstanding;
 wire[2:0]guide_slot;wire[7:0]style_mask,guide_mask;
 wire visible_de,visible_good;wire[11:0]visible_x,visible_y;wire[31:0]visible_fid,visible_epoch;
 assign reader_protect_mask=style_mask|guide_mask;
 assign style_cmd[192:0]={1'b1,192'd0};assign guide_cmd[192:0]={1'b1,192'd0};
 d64_native_display_chain #(.FAULT_STICKY(1),.IMG_W(256),.IMG_H(144),.SCALE(1),
  .GUIDE_ENABLE(1),.GUIDE_STORED_H(1080),.GUIDE_BASE(32'h01000000),.GUIDE_SLOT_BYTES(32'h00400000),
  .BILINEAR_ENABLE(1),.DST_W(1920),.DST_H(1080),.INTERACTION_ENABLE(0),.WIPE_ENABLE(0),.REMOTE_DEBUG_ENABLE(1),
  .FPS_OSD_ENABLE(1),.FPS_DEFAULT_ON(1),.FPS_CLK_HZ(100000000),.STAGE_RATE_ENABLE(1),
  .H_ACT(1920),.H_FP(88),.H_SYNC(44),.H_BP(148),.V_ACT(1080),.V_FP(4),.V_SYNC(5),.V_BP(36),
  .INITIAL_BASE(32'h03000000))u_display(
  .clk_pix(pix_clk),.rst_pix_n(pix_rst_n),.clk_axi(clk),.rst_axi_n(rst_n),
  .camera_complete_count(camera_complete_count),.style_new_count(style_new_count),
  .stage_rate_x100_count(stage_rate_x100_count),.stage_rate_valid_count(stage_rate_valid_count),
  .publish_valid(publish_valid),.publish_ready(publish_ready),.publish_base(publish_base),.publish_fid(publish_fid),.publish_epoch(publish_epoch),
  .publish_guide_valid(1'b1),.publish_original_valid(1'b0),.publish_guide_fid(publish_fid),.publish_guide_slot(publish_slot),.guide_strength(2'd2),
  .fps_toggle(1'b0),.remote_fps_set_valid(publish_valid&&publish_ready),.remote_fps_set_value(publish_fps),
  .fps_requested_pe(),.remote_actual_state_pe(),.active_epoch(active_epoch),.frame_start_pix(),.frame_start_axi(),
  .comparison_publication_window_axi(),.first_scan_axi(),.first_scan_pix(),.scan_fid_axi(),.scan_epoch_axi(),.vblank_pix(),
  .active_base(active_base),.active_fid(active_fid),.active_valid(active_valid),
  .tmds_data0(tmds0),.tmds_data1(tmds1),.tmds_data2(tmds2),.tmds_clk(tmdsclk),
  .axi_arvalid(style_cmd[193]),.axi_araddr(style_cmd[225:194]),.axi_arlen(style_cmd[233:226]),.axi_arsize(style_cmd[236:234]),.axi_arburst(style_cmd[238:237]),
  .axi_arready(style_rsp[5]),.axi_rvalid(style_rsp[6]),.axi_rdata(style_rsp[134:7]),.axi_rlast(style_rsp[135]),.axi_rresp(style_rsp[137:136]),.axi_rready(style_cmd[239]),
  .guide_arvalid(guide_cmd[193]),.guide_araddr(guide_cmd[225:194]),.guide_arlen(guide_cmd[233:226]),.guide_arsize(guide_cmd[236:234]),.guide_arburst(guide_cmd[238:237]),
  .guide_arready(guide_rsp[5]),.guide_rvalid(guide_rsp[6]),.guide_rdata(guide_rsp[134:7]),.guide_rlast(guide_rsp[135]),.guide_rresp(guide_rsp[137:136]),.guide_rready(guide_cmd[239]),
  .rows_loaded(rows),.underrun(underrun),.axi_errors(axi_errors),.bad_rows(bad_rows),.starved(starved),
  .guide_faults(guide_faults),.guide_reader_axi_fault(guide_axi_fault),.guide_reader_protocol_fault(guide_protocol_fault),
  .guide_reader_outstanding(guide_outstanding),.guide_reader_slot(guide_slot),.source_protect_mask_axi(style_mask),
  .guide_reader_protect_mask_axi(guide_mask),
  .compare_enabled_pe(1'b0),.effect_enabled_pe(1'b0),.display_mode_pe(2'd0),.comparison_publish_valid(1'b0),
  .comparison_original_base(32'd0),.comparison_half_base(32'd0),.comparison_fid(32'd0),.comparison_epoch(32'd0),.comparison_slot(3'd0),
  .comparison_arready(1'b0),.comparison_rvalid(1'b0),.comparison_rdata(128'd0),.comparison_rlast(1'b0),.comparison_rresp(2'd0),
  .comparison_publish_ready(),.comparison_protect_mask_pe(),.comparison_araddr(),.comparison_arlen(),.comparison_arsize(),.comparison_arburst(),.comparison_arvalid(),.comparison_rready(),.comparison_faults(),.processing_fps(),
  .full_render_de(),.full_render_good(),.full_render_rgb(),.full_render_ax(),.full_render_ay(),.full_render_fid(),.full_render_epoch(),.full_render_guide_slot(),.full_render_slot_valid(),
  .visible_de(visible_de),.visible_good(visible_good),.visible_ax(visible_x),.visible_ay(visible_y),.visible_fid(visible_fid),.visible_epoch(visible_epoch));
 reg core_fault;(* async_reg="true" *)reg quality_s1,quality_s2;
 (* async_reg="true" *)reg pix_fault_s1,pix_fault_s2;
 wire pix_fault=guide_faults!=0||starved||underrun!=0||bad_rows!=0;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin core_fault<=0;pix_fault_s1<=0;pix_fault_s2<=0;end
  else begin
   pix_fault_s1<=pix_fault;pix_fault_s2<=pix_fault_s1;
   if(guide_axi_fault||guide_protocol_fault||axi_errors!=0)core_fault<=1;
  end
 always @(posedge pix_clk or negedge pix_rst_n)
  if(!pix_rst_n)begin quality_s1<=0;quality_s2<=0;end
  else begin quality_s1<=core_fault;quality_s2<=quality_s1;end
 assign fault=core_fault||pix_fault_s2;
 wire[31:0]complete_pix,distinct_pix,rejected_pix;
 d2_b_render_health u_health(.clk(pix_clk),.rst_n(pix_rst_n),.de(visible_de),.good(visible_good),.health_fault(pix_fault||quality_s2),
  .ax(visible_x),.ay(visible_y),.fid(visible_fid),.epoch(visible_epoch),.complete_frames(complete_pix),.distinct_frames(distinct_pix),.rejected_frames(rejected_pix),
  .repeated_frames(),.qualified_pulse(),.last_complete_fid(),.last_complete_epoch());
 d64_counter_gray_bridge u_complete(.src_clk(pix_clk),.src_rst_n(pix_rst_n),.src_count(complete_pix),.dst_clk(clk),.dst_rst_n(rst_n),.dst_count(complete_frames));
 d64_counter_gray_bridge u_distinct(.src_clk(pix_clk),.src_rst_n(pix_rst_n),.src_count(distinct_pix),.dst_clk(clk),.dst_rst_n(rst_n),.dst_count(distinct_frames));
 d64_counter_gray_bridge u_reject(.src_clk(pix_clk),.src_rst_n(pix_rst_n),.src_count(rejected_pix),.dst_clk(clk),.dst_rst_n(rst_n),.dst_count(rejected_frames));
endmodule
