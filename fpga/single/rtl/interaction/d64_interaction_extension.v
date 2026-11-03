`timescale 1ns/1ps
// Interactive peripherals attach to the existing AXI tree. Independent snapshot
// slots prevent phone backpressure from leasing the CNN's two output buffers.
module d64_interaction_extension #(
 parameter integer VOICE_ENABLE=1,GESTURE_ENABLE=1,COMPARISON_ENABLE=1,PREVIEW_ENABLE=1,REMOTE_DEBUG_ENABLE=0,
 parameter TEMPLATE_FILE="../../audio/data/voice_combined.mem"
)(
 input wire clk_pe,rst_pe_n,clk_pix,rst_pix_n,frame_tick,
 input wire comparison_publication_window,
 input wire spi_sclk,spi_cs_n,spi_mosi,output wire spi_miso,spi_ready,
 input wire compare_toggle,effect_trigger,
 output wire compare_enabled_pe,output reg [1:0] display_mode_pe,output wire effect_enabled_pe,
 input wire [15:0] remote_status,remote_active_epoch,input wire [1:0] remote_active_style,
 output wire remote_fps_set_valid,remote_fps_set_value,
 output wire external_style_valid,external_next,output wire [1:0] external_style,
 input wire cap_valid,cap_ready,cap_sof,cap_eof,input wire [383:0] cap_rgb,
 input wire [7:0] cap_x0,cap_y,input wire [31:0] cap_fid,input wire source_bad,
 input wire original_valid,input wire [15:0] original_rgb565,
 input wire [9:0] original_x,original_y,input wire [31:0] original_fid,
 input wire original_guide_valid,grant_valid,input wire [2:0] grant_slot,
 input wire guide_allocate,guide_commit,guide_failed,input wire [2:0] guide_commit_slot,
 input wire [31:0] guide_commit_fid,
 input wire [2:0] original_query_slot,input wire [31:0] original_query_fid,
 output wire original_query_valid,
 input wire full_render_de,full_render_good,input wire [23:0] full_render_rgb,
 input wire [11:0] full_render_ax,full_render_ay,input wire [31:0] full_render_fid,
 input wire [15:0] full_render_epoch,input wire [2:0] full_render_guide_slot,input wire full_render_slot_valid,
 input wire publish_event,input wire [31:0] publish_base,publish_fid,
 input wire [15:0] publish_epoch,input wire [1:0] publish_style,
 input wire overwrite_event,input wire [31:0] overwrite_base,input wire [7:0] processing_fps,
 output wire comparison_publish_valid,input wire comparison_publish_ready,
 output wire [31:0] comparison_original_base,comparison_half_base,comparison_fid,
 output wire [15:0] comparison_epoch,output wire [2:0] comparison_slot,
 input wire [4:0] comparison_protect_mask_pe,
 input wire [239:0] comparison_cmd,output wire [137:0] comparison_rsp,
 output wire [4:0] protected_slots,
 output wire [239:0] extra_cmd,input wire [137:0] extra_rsp,
 output wire video_fault,preview_fault,voice_fault
);
 localparam CAWVALID=0,CAWADDR=1,CAWLEN=33,CAWSIZE=41,CAWBURST=44,
 CWVALID=46,CWDATA=47,CWSTRB=175,CWLAST=191,CBREADY=192,
 CARVALID=193,CARADDR=194,CARLEN=226,CARSIZE=234,CARBURST=237,CRREADY=239;
 localparam RAWREADY=0,RWREADY=1,RBVALID=2,RBRESP=3,RARREADY=5,RRVALID=6,RRDATA=7,RRLAST=135,RRRESP=136;
 wire [239:0] original_cmd,half_cmd,snapshot_cmd,service_cmd,preview_cmd,stores_cmd,display_cmd;
 wire [137:0] original_rsp,half_rsp,snapshot_rsp,service_rsp,preview_rsp,stores_rsp,display_rsp;
 wire wave_trigger;reg [5:0] effect_frames;
 wire remote_control_valid;wire [2:0] remote_control_action;wire [1:0] remote_control_value;
 wire remote_mode_valid=REMOTE_DEBUG_ENABLE!=0&&remote_control_valid&&
     (remote_control_action==2||remote_control_action==3);
 wire [1:0] cycled_mode=(display_mode_pe==2)?2'd0:display_mode_pe+1'b1;
 wire [1:0] next_display_mode=remote_mode_valid?
     ((remote_control_action==2)?remote_control_value:cycled_mode):cycled_mode;
 wire mode_change=remote_mode_valid||compare_toggle;
 assign remote_fps_set_valid=REMOTE_DEBUG_ENABLE!=0&&remote_control_valid&&remote_control_action==1;
 assign remote_fps_set_value=remote_control_value[0];
 always @(posedge clk_pe or negedge rst_pe_n)begin
  if(!rst_pe_n)begin display_mode_pe<=0;effect_frames<=0;end
  else begin
   // KEY2 is already a one-cycle debounced event. Keep the legacy pair
   // reader enabled only in mode 1; mode 2 uses the full-size render path.
   if(mode_change)display_mode_pe<=next_display_mode;
   if(effect_trigger||wave_trigger||(REMOTE_DEBUG_ENABLE!=0&&remote_control_valid&&remote_control_action==4))effect_frames<=60;
   else if(frame_tick&&effect_frames!=0)effect_frames<=effect_frames-1'b1;
  end
 end
 assign compare_enabled_pe=display_mode_pe==2'd1;
 assign effect_enabled_pe=effect_frames!=0;
 generate if(GESTURE_ENABLE!=0)begin:g_wave
  d64_wave_detector u_wave(.clk(clk_pe),.rst_n(rst_pe_n),.cap_valid(cap_valid),.cap_ready(cap_ready),
   .cap_rgb(cap_rgb),.cap_x0(cap_x0),.cap_y(cap_y),.cap_fid(cap_fid),.cap_sof(cap_sof),.cap_eof(cap_eof),
   .source_bad(source_bad),.effect_trigger(wave_trigger),.primed(),.frame_good());
 end else begin:g_no_wave assign wave_trigger=0;end endgenerate
 wire pcm_valid,pcm_ready,pcm_abort;wire signed [15:0] pcm;
 wire voice_valid,voice_next,remote_valid,remote_next;wire [1:0] voice_style,remote_style;
 generate if(VOICE_ENABLE!=0)begin:g_voice
  d64_voice_pipeline #(.TEMPLATE_FILE(TEMPLATE_FILE)) u_voice(.clk(clk_pe),.rst_n(rst_pe_n),
   .abort_event(pcm_abort),.pcm_valid(pcm_valid),.pcm(pcm),.pcm_ready(pcm_ready),
   .voice_valid(voice_valid),.voice_style_id(voice_style),.voice_next(voice_next),.command_class(),
   .rejected(),.protocol_fault(voice_fault),.reject_reason(),.best_score(),.second_score());
 end else begin:g_no_voice assign pcm_ready=1;assign voice_valid=0;assign voice_next=0;
  assign voice_style=1;assign voice_fault=0;end endgenerate
 assign external_style_valid=remote_valid||voice_valid;
 assign external_next=remote_valid?remote_next:voice_next;
 assign external_style=remote_valid?remote_style:voice_style;
 wire original_capturing,original_commit,original_failed,original_started;
 reg original_ticket;wire [2:0] original_active_slot,original_commit_slot;
 wire [31:0] original_active_fid,original_commit_fid;wire original_overflow,original_protocol,original_axi;
 always @(posedge clk_pe or negedge rst_pe_n)begin
  if(!rst_pe_n)original_ticket<=0;
  else begin
   if(original_commit||original_failed)original_ticket<=0;
   if(original_started)original_ticket<=1;
  end
 end
 wire half_commit,half_failed;wire [2:0] half_commit_slot;wire [31:0] half_commit_fid;
 wire [15:0] half_commit_epoch;wire [4:0] half_ticket;wire half_axi,half_protocol,half_overflow;
 reg pair_pending;reg [31:0] pair_fid;reg [15:0] pair_epoch;reg [2:0] pair_slot;
 wire pair_valid;wire [4:0] complete_slots;
 reg [4:0] half_ticket_previous;
 // Admission excludes our own capture ticket: it is sampled only at PIX SOF.
 // A pending/current comparison keeps its immutable half plane protected.
 wire [4:0] comparison_admission_mask=comparison_protect_mask_pe|
  ((compare_enabled_pe&&pair_pending&&pair_valid)?(5'b1<<pair_slot):5'd0);
 (* async_reg="true" *) reg [4:0] admission_sync1,admission_sync2;
 always @(posedge clk_pix or negedge rst_pix_n)begin
  if(!rst_pix_n)begin admission_sync1<=0;admission_sync2<=0;end
  else begin admission_sync1<=comparison_admission_mask;admission_sync2<=admission_sync1;end
 end
 always @(posedge clk_pe or negedge rst_pe_n)begin
  if(!rst_pe_n)half_ticket_previous<=0;
  else half_ticket_previous<=half_ticket;
 end
 // Candidate: send the immutable completed pair during the synchronized
 // final vertical front porch. This lets the existing mailbox apply it at
 // the coming PIX boundary and preserves pending-mask settling before SOF.
 assign comparison_publish_valid=comparison_publication_window&&compare_enabled_pe&&pair_pending&&pair_valid&&!half_ticket[pair_slot];
 assign comparison_fid=pair_fid;assign comparison_epoch=pair_epoch;assign comparison_slot=pair_slot;
 wire [4:0] invalid_slots=(guide_allocate?(5'b1<<grant_slot):0)|
  (guide_failed?(5'b1<<guide_commit_slot):0)|
  ((original_failed&&original_ticket)?(5'b1<<original_commit_slot):0);
 // A half rewrite/failure does not destroy the still valid guide/original.
 wire [4:0] invalid_half_slots=(half_ticket&~half_ticket_previous)|
  (half_failed?(5'b1<<half_commit_slot):5'd0);
 always @(posedge clk_pe or negedge rst_pe_n)begin
  if(!rst_pe_n)begin pair_pending<=0;pair_fid<=0;pair_epoch<=0;pair_slot<=0;end
  else begin
   if(comparison_publish_valid&&comparison_publish_ready)pair_pending<=0;
   // Re-entering comparison can use the retained latest complete pair even
   // when no new CNN publication arrives (for example a stopped camera).
   if(mode_change&&next_display_mode==2'd1&&display_mode_pe!=2'd1&&pair_valid)pair_pending<=1;
   if(half_commit)begin pair_pending<=1;pair_fid<=half_commit_fid;pair_epoch<=half_commit_epoch;pair_slot<=half_commit_slot;end
   if(pair_pending&&invalid_slots[pair_slot])pair_pending<=0;
  end
 end
 assign protected_slots=comparison_protect_mask_pe|half_ticket|
  ((original_capturing||original_ticket)?(5'b1<<original_active_slot):0)|
  ((compare_enabled_pe&&pair_pending&&pair_valid)?(5'b1<<pair_slot):0);
 assign video_fault=original_overflow||original_protocol||original_axi||half_axi||half_protocol||half_overflow;

 generate if(COMPARISON_ENABLE!=0)begin:g_original
 d64_original_store_quad #(.FIFO_AW(10)) u_original(.clk(clk_pe),.rst_n(rst_pe_n),
 .in_valid(original_valid&&original_guide_valid),.in_rgb565(original_rgb565),
 .in_x({1'b0,original_x}),.in_y({1'b0,original_y}),.in_fid(original_fid),.frame_bad(source_bad),
 .frame_slot_valid(grant_valid),.frame_slot(grant_slot),.capture_started(original_started),.capturing(original_capturing),
 .active_slot(original_active_slot),.active_fid(original_active_fid),.committed(original_commit),.failed(original_failed),
 .committed_slot(original_commit_slot),.committed_fid(original_commit_fid),.in_ready(),
 .overflow_fault(original_overflow),.protocol_fault(original_protocol),.axi_fault(original_axi),
 .awaddr(original_cmd[CAWADDR+:32]),.awlen(original_cmd[CAWLEN+:8]),.awsize(original_cmd[CAWSIZE+:3]),.awburst(original_cmd[CAWBURST+:2]),
 .awvalid(original_cmd[CAWVALID]),.awready(original_rsp[RAWREADY]),.wdata(original_cmd[CWDATA+:128]),.wstrb(original_cmd[CWSTRB+:16]),
 .wlast(original_cmd[CWLAST]),.wvalid(original_cmd[CWVALID]),.wready(original_rsp[RWREADY]),.bvalid(original_rsp[RBVALID]),
 .bready(original_cmd[CBREADY]),.bresp(original_rsp[RBRESP+:2]));
 d64_half_preview_capture #(.ADMISSION_ENABLE(1)) u_half(.clk_pix(clk_pix),.rst_pix_n(rst_pix_n),.clk_pe(clk_pe),.rst_pe_n(rst_pe_n),
 .render_de(full_render_de),.render_good(full_render_good),.render_rgb(full_render_rgb),.render_ax(full_render_ax),.render_ay(full_render_ay),
 .render_fid(full_render_fid),.render_epoch(full_render_epoch),.guide_slot(full_render_guide_slot),.slot_valid(full_render_slot_valid),
 .admit_frame(!admission_sync2[full_render_guide_slot]),
 .busy_pix(),.ticket_mask_pix(),.skipped_frames_pix(),.overflow_fault_pix(),.busy_pe(),.ticket_mask_pe(half_ticket),
 .committed(half_commit),.failed(half_failed),.committed_slot(half_commit_slot),.committed_fid(half_commit_fid),.committed_epoch(half_commit_epoch),
 .writer_axi_fault(half_axi),.writer_protocol_fault(half_protocol),.writer_overflow_fault(half_overflow),
 .awaddr(half_cmd[CAWADDR+:32]),.awlen(half_cmd[CAWLEN+:8]),.awsize(half_cmd[CAWSIZE+:3]),.awburst(half_cmd[CAWBURST+:2]),
 .awvalid(half_cmd[CAWVALID]),.awready(half_rsp[RAWREADY]),.wdata(half_cmd[CWDATA+:128]),.wstrb(half_cmd[CWSTRB+:16]),
 .wlast(half_cmd[CWLAST]),.wvalid(half_cmd[CWVALID]),.wready(half_rsp[RWREADY]),.bvalid(half_rsp[RBVALID]),
 .bready(half_cmd[CBREADY]),.bresp(half_rsp[RBRESP+:2]));
 d64_comparison_pairs #(.HALF_INVALIDATION_ENABLE(1)) u_pairs(.clk(clk_pe),.rst_n(rst_pe_n),.invalidate_slots(invalid_slots),
 .invalidate_half_slots(invalid_half_slots),
 .guide_commit(guide_commit),.guide_slot(guide_commit_slot),.guide_fid(guide_commit_fid),
 .original_commit(original_commit),.original_slot(original_commit_slot),.original_fid(original_commit_fid),
 .half_commit(half_commit),.half_slot(half_commit_slot),.half_fid(half_commit_fid),.half_epoch(half_commit_epoch),
 .query_slot(pair_slot),.query_fid(pair_fid),.query_epoch(pair_epoch),.query_valid(pair_valid),
 .original_query_slot(original_query_slot),.original_query_fid(original_query_fid),.original_query_valid(original_query_valid),
 .query_original_base(comparison_original_base),.query_half_base(comparison_half_base),.complete_slots(complete_slots));
 end else begin:g_no_original
 assign original_cmd[191:0]=0;assign original_cmd[CBREADY]=1;assign half_cmd[191:0]=0;assign half_cmd[CBREADY]=1;
 assign original_capturing=0;assign original_started=0;assign original_commit=0;assign original_failed=0;assign original_active_slot=0;
 assign original_active_fid=0;assign original_commit_slot=0;assign original_commit_fid=0;
 assign original_overflow=0;assign original_protocol=0;assign original_axi=0;
 assign half_commit=0;assign half_failed=0;assign half_commit_slot=0;assign half_commit_fid=0;assign half_commit_epoch=0;
 assign half_ticket=0;assign half_axi=0;assign half_protocol=0;assign half_overflow=0;
 assign pair_valid=0;assign comparison_original_base=0;assign comparison_half_base=0;assign complete_slots=0;
 assign original_query_valid=0;
 end endgenerate
 assign original_cmd[238:193]=0;assign original_cmd[CRREADY]=1;
 assign half_cmd[238:193]=0;assign half_cmd[CRREADY]=1;
 wire latest_valid,latest_slot,lease_active,lease_slot;
 wire [31:0] latest_base,latest_fid;wire [15:0] latest_epoch;wire [1:0] latest_style;wire [7:0] latest_effect;
 wire snapshot_drop,snapshot_fault,service_protocol,service_ddr,mailbox_short,mailbox_busy,mailbox_tx;
 assign preview_fault=snapshot_fault||service_protocol||service_ddr||mailbox_short||mailbox_busy||mailbox_tx;
 generate if(PREVIEW_ENABLE!=0)begin:g_network
 d64_preview_snapshot #(.CAPTURE_DIV(1)) u_snapshot(.clk(clk_pe),.rst_n(rst_pe_n),.publish_event(publish_event),
 .publish_base(publish_base),.publish_fid(publish_fid),.publish_epoch(publish_epoch),.publish_style(publish_style),
 .publish_effect({7'd0,effect_enabled_pe}),.overwrite_event(overwrite_event),.overwrite_base(overwrite_base),
 .lease_active(lease_active),.lease_slot(lease_slot),.latest_valid(latest_valid),.latest_slot(latest_slot),
 .latest_fid(latest_fid),.latest_epoch(latest_epoch),.latest_style(latest_style),.latest_effect(latest_effect),
 .latest_base(latest_base),.busy(),.dropped(snapshot_drop),.fault(snapshot_fault),.araddr(snapshot_cmd[CARADDR+:32]),.arlen(snapshot_cmd[CARLEN+:8]),.arsize(snapshot_cmd[CARSIZE+:3]),.arburst(snapshot_cmd[CARBURST+:2]),
 .arvalid(snapshot_cmd[CARVALID]),.arready(snapshot_rsp[RARREADY]),.rdata(snapshot_rsp[RRDATA+:128]),.rvalid(snapshot_rsp[RRVALID]),
 .rready(snapshot_cmd[CRREADY]),.rlast(snapshot_rsp[RRLAST]),.rresp(snapshot_rsp[RRRESP+:2]),.awaddr(snapshot_cmd[CAWADDR+:32]),.awlen(snapshot_cmd[CAWLEN+:8]),.awsize(snapshot_cmd[CAWSIZE+:3]),.awburst(snapshot_cmd[CAWBURST+:2]),
 .awvalid(snapshot_cmd[CAWVALID]),.awready(snapshot_rsp[RAWREADY]),.wdata(snapshot_cmd[CWDATA+:128]),.wstrb(snapshot_cmd[CWSTRB+:16]),
 .wlast(snapshot_cmd[CWLAST]),.wvalid(snapshot_cmd[CWVALID]),.wready(snapshot_rsp[RWREADY]),.bvalid(snapshot_rsp[RBVALID]),
 .bready(snapshot_cmd[CBREADY]),.bresp(snapshot_rsp[RBRESP+:2]));
 wire rx_done,response_commit,tx_we;wire [8:0] rx_addr,tx_addr;wire [7:0] rx_data,tx_data;
 d64_spi_mailbox u_mailbox(.clk_pe(clk_pe),.rst_n(rst_pe_n),.spi_sclk(spi_sclk),.spi_cs_n(spi_cs_n),
 .spi_mosi(spi_mosi),.spi_miso(spi_miso),.ready(spi_ready),.rx_done(rx_done),.rx_rd_addr(rx_addr),.rx_rd_data(rx_data),
 .tx_we(tx_we),.tx_addr(tx_addr),.tx_data(tx_data),.response_commit(response_commit),
 .short_packet_fault(mailbox_short),.busy_packet_fault(mailbox_busy),.tx_overlap_fault(mailbox_tx));
 d64_preview_service #(.AUDIO_ENABLE(VOICE_ENABLE),.REMOTE_DEBUG_ENABLE(REMOTE_DEBUG_ENABLE)) u_service(.clk(clk_pe),.rst_n(rst_pe_n),.frame_tick(frame_tick),.rx_done(rx_done),
 .rx_addr(rx_addr),.rx_data(rx_data),.tx_we(tx_we),.tx_addr(tx_addr),.tx_data(tx_data),.response_commit(response_commit),
 .latest_valid(latest_valid),.latest_slot(latest_slot),.latest_base(latest_base),.latest_fid(latest_fid),
 // Effect is a live presentation control. ACQUIRE freezes it for the lease;
 // a later ACQUIRE/STATUS can update a retained picture after the effect ends.
 .latest_epoch(latest_epoch),.latest_style(latest_style),.latest_effect({7'd0,effect_enabled_pe}),.processing_fps(processing_fps),
 .remote_status(remote_status),.remote_active_epoch(remote_active_epoch),.remote_active_style(remote_active_style),
 .remote_control_valid(remote_control_valid),.remote_control_action(remote_control_action),.remote_control_value(remote_control_value),
 .lease_active(lease_active),.lease_slot(lease_slot),.pcm_valid(pcm_valid),.pcm(pcm),.pcm_ready(pcm_ready),.pcm_abort(pcm_abort),
 .style_valid(remote_valid),.style_next(remote_next),.style_id(remote_style),.protocol_fault(service_protocol),.ddr_fault(service_ddr),.araddr(service_cmd[CARADDR+:32]),.arlen(service_cmd[CARLEN+:8]),.arsize(service_cmd[CARSIZE+:3]),.arburst(service_cmd[CARBURST+:2]),
 .arvalid(service_cmd[CARVALID]),.arready(service_rsp[RARREADY]),.rdata(service_rsp[RRDATA+:128]),.rvalid(service_rsp[RRVALID]),
 .rready(service_cmd[CRREADY]),.rlast(service_rsp[RRLAST]),.rresp(service_rsp[RRRESP+:2]));
 end else begin:g_no_network
 assign snapshot_cmd=0;assign service_cmd[238:193]=0;assign service_cmd[CRREADY]=1;
 assign spi_miso=0;assign spi_ready=0;assign pcm_valid=0;assign pcm=0;assign pcm_abort=0;
 assign remote_valid=0;assign remote_next=0;assign remote_style=1;
 assign remote_control_valid=0;assign remote_control_action=0;assign remote_control_value=0;
 assign latest_valid=0;assign latest_slot=0;assign latest_base=0;assign latest_fid=0;assign latest_epoch=0;
 assign latest_style=1;assign latest_effect=0;assign lease_active=0;assign lease_slot=0;
 assign snapshot_drop=0;assign snapshot_fault=0;assign service_protocol=0;assign service_ddr=0;
 assign mailbox_short=0;assign mailbox_busy=0;assign mailbox_tx=0;
 end endgenerate
 assign service_cmd[191:0]=0;assign service_cmd[CBREADY]=1;
 d64_axi_join u_preview(.clk(clk_pe),.rst_n(rst_pe_n),.m0_cmd(snapshot_cmd),.m0_rsp(snapshot_rsp),.m1_cmd(service_cmd),.m1_rsp(service_rsp),.s_cmd(preview_cmd),.s_rsp(preview_rsp));
 d64_axi_join u_stores(.clk(clk_pe),.rst_n(rst_pe_n),.m0_cmd(original_cmd),.m0_rsp(original_rsp),.m1_cmd(half_cmd),.m1_rsp(half_rsp),.s_cmd(stores_cmd),.s_rsp(stores_rsp));
 d64_axi_join u_display(.clk(clk_pe),.rst_n(rst_pe_n),.m0_cmd(comparison_cmd),.m0_rsp(comparison_rsp),.m1_cmd(preview_cmd),.m1_rsp(preview_rsp),.s_cmd(display_cmd),.s_rsp(display_rsp));
 d64_axi_join u_extra(.clk(clk_pe),.rst_n(rst_pe_n),.m0_cmd(stores_cmd),.m0_rsp(stores_rsp),.m1_cmd(display_cmd),.m1_rsp(display_rsp),.s_cmd(extra_cmd),.s_rsp(extra_rsp));
endmodule
