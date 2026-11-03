`timescale 1ns/1ps
module d2_a_system(
 input wire clk,clk_fast,rst_n,ddr_ready,cam_clk,cam_rst_n,pix_clk,pix_rst_n,
 input wire capture_core_rst_n,capture_cam_rst_n,output wire capture_release_n,
 input wire valid,input wire [95:0] rgb4,input wire [10:0] x0,y,input wire [31:0] fid,
 input wire sof,eof,source_bad,frame_start,de,input wire [3:0] keys_n,
 input wire diag_packed_bad,diag_isp_bad,input wire [7:0] diag_bridge_code,
 input wire s3_uart_rx,output wire s3_uart_tx,
 input wire [3:0] i_spi0_io,output wire [3:0] o_spi0_io,oe_spi0_io,
 output wire o_spi_clk,oe_spi_clk,o_spi0_cs,oe_spi0_cs,
 output wire [23:0] rgb,output wire txd,output wire fault,
 output wire [239:0] cmd,input wire [137:0] rsp
);
 wire [239:0] mcmd[0:3],joincmd[0:2];wire [137:0] mrsp[0:3],joinrsp[0:2];
 d64_axi_join u_join0(.clk(clk),.rst_n(rst_n),.m0_cmd(mcmd[0]),.m0_rsp(mrsp[0]),.m1_cmd(mcmd[1]),.m1_rsp(mrsp[1]),.s_cmd(joincmd[0]),.s_rsp(joinrsp[0]));
 d64_axi_join u_join1(.clk(clk),.rst_n(rst_n),.m0_cmd(joincmd[0]),.m0_rsp(joinrsp[0]),.m1_cmd(mcmd[2]),.m1_rsp(mrsp[2]),.s_cmd(joincmd[1]),.s_rsp(joinrsp[1]));
 d64_axi_join u_join2(.clk(clk),.rst_n(rst_n),.m0_cmd(mcmd[3]),.m0_rsp(mrsp[3]),.m1_cmd(joincmd[1]),.m1_rsp(joinrsp[1]),.s_cmd(cmd),.s_rsp(rsp));
 wire cap_commit,release_valid,release_ok;wire [1:0] cap_slot,release_slot;wire [31:0] cap_fid,captured,dropped;
 wire capture_fault,capture_hard_fault,capture_quiescent,compute_fault,input_fault,tx_fault;
 wire recovering;wire [31:0] recovery_count,recovery_fault_count;
 camera_capture_recovery u_recovery(.clk(clk),.rst_n(rst_n),.fault(capture_fault&&!capture_hard_fault),.quiescent(capture_quiescent&&!capture_hard_fault),
 .pipeline_release_n(capture_release_n),.recovery_count(recovery_count),.fault_count(recovery_fault_count),.recovering(recovering));
 reg [3:0] owned_slots,recovery_owned_slots;reg capture_reset_seen;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin owned_slots<=0;recovery_owned_slots<=0;capture_reset_seen<=0;end
 else begin
 if(cap_commit)owned_slots[cap_slot]<=1;
 if(release_valid)owned_slots[release_slot]<=0;
 capture_reset_seen<=!capture_core_rst_n;
 if(!capture_core_rst_n&&!capture_reset_seen)recovery_owned_slots<=
   (owned_slots|(cap_commit?(4'b0001<<cap_slot):4'b0000))&
   ~(release_valid?(4'b0001<<release_slot):4'b0000);
 end
 wire [191:0] diag_service;wire diag_service_saturated;
 wire [191:0] diag_cap_core;wire [95:0] diag_cam,diag_tx_core;wire [127:0] diag_compute,diag_tx_pixel;wire diag_cam_valid,diag_tx_pixel_valid;
 reg local_fault;assign fault=local_fault||capture_hard_fault||compute_fault||input_fault||tx_fault;
 d2_a_capture u_capture(.cam_clk(cam_clk),.cam_rst_n(cam_rst_n),.clk(clk),.rst_n(rst_n),
 .pipeline_core_rst_n(capture_core_rst_n),.pipeline_cam_rst_n(capture_cam_rst_n),.recovering(recovering),.preserve_slots(recovery_owned_slots),
 .hard_fault(capture_hard_fault),.quiescent(capture_quiescent),.diag_service(diag_service),.diag_service_saturated(diag_service_saturated),
 .valid(valid),.rgb4(rgb4),.x0(x0),.y(y),.fid(fid),.sof(sof),.eof(eof),.source_bad(source_bad),
 .release_valid(release_valid),.release_slot(release_slot),.committed(cap_commit),.committed_slot(cap_slot),.committed_fid(cap_fid),
 .diag_packed_bad(diag_packed_bad),.diag_isp_bad(diag_isp_bad),.diag_bridge_code(diag_bridge_code),.diag_core(diag_cap_core),.diag_cam(diag_cam),.diag_cam_valid(diag_cam_valid),
 .fault(capture_fault),.captured(captured),.dropped(dropped),.cmd(mcmd[0]),.rsp(mrsp[0]));
 reg [2:0] state[0:3];reg [31:0] sfid[0:3],sepoch[0:3];reg [1:0] sstyle[0:3],slayout[0:3];reg sfps[0:3];reg [7:0] seffect[0:3];
 reg [1:0] ui_style,ui_layout;reg [31:0] ui_epoch;reg ui_fps;reg [5:0] effect_frames;
 wire [7:0] ui_effect={7'd0,effect_frames!=0};
 wire remote_control,voice_valid,voice_next,gesture_trigger,voice_fault;wire [7:0] remote_action,remote_value;wire [1:0] voice_style;
 wire remote_style=remote_control&&(remote_action==1||remote_action==2);
 wire [1:0] loaded_profile;wire model_loaded;reg [31:0] last_capture_fid,last_computed_fid,last_transmitted_fid;
 wire [31:0] uart_errors,uart_session;
 wire request;wire [1:0] request_style;wire toggle_fps,toggle_layout,trigger_effect;
 wire compute_ready,compute_done,input_start;reg compute_start,compute_busy;reg [1:0] compute_slot;
 wire [1:0] desired_style=compute_busy?sstyle[compute_slot]+1:ui_style;
 d64_ui_keys u_keys(.clk(clk),.rst_n(rst_n),.keys_n(keys_n),.active_style(desired_style),.model_valid(compute_ready),
 .external_style_valid(remote_style||voice_valid),.external_next(remote_style?remote_action==2:voice_next),.external_style(remote_style?remote_value[1:0]+1:voice_style),.request_valid(request),.request_style(request_style),
 .fps_toggle(toggle_fps),.compare_toggle(toggle_layout),.effect_trigger(trigger_effect),.target_style(),.target_pending());
 wire cap_valid,cap_ready,cap_sof,cap_eol,cap_eof,input_done;wire [383:0] cap_rgb;wire [31:0] input_fid;wire [7:0] cap_x,cap_y;
 wire [31:0] slot_base=32'h01000000+{8'd0,compute_slot,22'd0};
 d2_a_input_reader u_input(.clk(clk),.rst_n(rst_n),.start(input_start),.base(slot_base+32'h340000),.fid(sfid[compute_slot]),
 .out_valid(cap_valid),.out_ready(cap_ready),.out_rgb(cap_rgb),.out_fid(input_fid),.out_x(cap_x),.out_y(cap_y),.out_sof(cap_sof),.out_eol(cap_eol),.out_eof(cap_eof),
 .done(input_done),.failed(input_fault),.cmd(mcmd[1]),.rsp(mrsp[1]));
 d2_a_compute u_compute(.clk_pe(clk),.clk_weight_fast(clk_fast),.rst_pe_n(rst_n),.ddr_ready(ddr_ready),
 .requested_style(desired_style),.start(compute_start),.tensor_base(slot_base+32'h300000),.ready(compute_ready),.model_loaded(model_loaded),.active_profile(loaded_profile),.input_start(input_start),.done(compute_done),.failed(compute_fault),.diag_compute(diag_compute),
 .d64_valid(cap_valid),.d64_ready(cap_ready),.d64_rgb888(cap_rgb),.d64_frame_id(input_fid),.d64_x0(cap_x),.d64_y(cap_y),.d64_sof(cap_sof),.d64_eol(cap_eol),.d64_eof(cap_eof),
 .axi_cmd(mcmd[2]),.axi_rsp(mrsp[2]),.i_spi0_io(i_spi0_io),.o_spi0_io(o_spi0_io),.oe_spi0_io(oe_spi0_io),.o_spi_clk(o_spi_clk),.oe_spi_clk(oe_spi_clk),.o_spi0_cs(o_spi0_cs),.oe_spi0_cs(oe_spi0_cs));
 reg [1:0] tx_slot;reg tx_valid;wire tx_ready;wire [31:0] transmitted;
 d2_a_transport u_tx(.clk(clk),.rst_n(rst_n),.pix_clk(pix_clk),.pix_rst_n(pix_rst_n),.frame_start(frame_start),.de(de),
 .valid(tx_valid),.ready(tx_ready),.slot(tx_slot),.fid(sfid[tx_slot]),.epoch(sepoch[tx_slot]),.style(sstyle[tx_slot]),.layout(slayout[tx_slot]),.fps(sfps[tx_slot]),.effect(seffect[tx_slot]),
 .released(release_valid),.released_ok(release_ok),.released_slot(release_slot),.fault(tx_fault),.transmitted(transmitted),.diag_core(diag_tx_core),.diag_pixel(diag_tx_pixel),.diag_pixel_valid(diag_tx_pixel_valid),.rgb(rgb),.cmd(mcmd[3]),.rsp(mrsp[3]));
 d2_a_peripherals u_peripherals(.clk(clk),.rst_n(rst_n),.uart_rx(s3_uart_rx),.uart_tx(s3_uart_tx),
 .requested_style(ui_style-2'd1),.active_style(loaded_profile-2'd1),.layout(ui_layout),.fps(ui_fps),.effect(effect_frames!=0),.model_valid(model_loaded),.busy(compute_busy),
 .epoch(ui_epoch),.capture_fid(last_capture_fid),.computed_fid(last_computed_fid),.transmitted_fid(last_transmitted_fid),.faults({29'd0,uart_errors!=0,voice_fault,fault}),
 .cap_valid(cap_valid),.cap_ready(cap_ready),.cap_sof(cap_sof),.cap_eof(cap_eof),.cap_rgb(cap_rgb),.cap_x0(cap_x),.cap_y(cap_y),.cap_fid(input_fid),.source_bad(capture_fault||input_fault),
 .control_valid(remote_control),.control_action(remote_action),.control_value(remote_value),.voice_valid(voice_valid),.voice_next(voice_next),.voice_style(voice_style),.gesture_trigger(gesture_trigger),.voice_fault(voice_fault),.packet_errors(uart_errors),.session_id(uart_session));
 reg [31:0] computed;integer i;reg found;reg [1:0] pick;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 local_fault<=0;compute_busy<=0;compute_start<=0;compute_slot<=0;tx_slot<=0;tx_valid<=0;computed<=0;ui_style<=1;ui_layout<=0;ui_epoch<=0;ui_fps<=1;effect_frames<=0;last_capture_fid<=0;last_computed_fid<=0;last_transmitted_fid<=0;
 for(i=0;i<4;i=i+1)begin state[i]<=0;sfid[i]<=0;sepoch[i]<=0;sstyle[i]<=0;slayout[i]<=0;sfps[i]<=1;seffect[i]<=0;end end
 else begin
 compute_start<=0;tx_valid<=0;
 if(request)begin ui_style<=request_style;end
 if(toggle_fps)ui_fps<=!ui_fps;
 if(toggle_layout)ui_layout<=ui_layout==2?0:ui_layout+1;
 if(remote_control&&remote_action==3)ui_layout<=remote_value[1:0];
 if(remote_control&&remote_action==4)ui_layout<=ui_layout==2?0:ui_layout+1;
 if(remote_control&&remote_action==5)ui_fps<=remote_value[0];
 if(trigger_effect||gesture_trigger||(remote_control&&remote_action==6))effect_frames<=60;
 else if(cap_commit&&effect_frames!=0)effect_frames<=effect_frames-1;
 if((cap_commit&&effect_frames==1)||request||toggle_fps||toggle_layout||trigger_effect||gesture_trigger||(remote_control&&remote_action>=3))ui_epoch<=ui_epoch+1;
 if(cap_commit)begin last_capture_fid<=cap_fid;if(state[cap_slot]!=0)local_fault<=1;else begin state[cap_slot]<=1;sfid[cap_slot]<=cap_fid;end end
 if(!compute_busy&&compute_ready&&!fault)begin
 found=0;pick=0;for(i=0;i<4;i=i+1)if(state[i]==1&&(!found||$signed(sfid[i]-sfid[pick])<0))begin found=1;pick=i;end
 if(found)begin compute_slot<=pick;compute_busy<=1;compute_start<=1;state[pick]<=2;sepoch[pick]<=ui_epoch;sstyle[pick]<=ui_style-1;slayout[pick]<=ui_layout;sfps[pick]<=ui_fps;seffect[pick]<=ui_effect;end
 end
 if(compute_done&&compute_busy)begin state[compute_slot]<=3;compute_busy<=0;computed<=computed+1;last_computed_fid<=sfid[compute_slot];end
 if(tx_ready&&!tx_valid&&!fault)begin
 found=0;pick=0;for(i=0;i<4;i=i+1)if(state[i]==3&&(!found||$signed(sfid[i]-sfid[pick])<0))begin found=1;pick=i;end
 if(found)begin tx_slot<=pick;tx_valid<=1;state[pick]<=4;end
 end
 if(release_valid)begin if(release_ok)last_transmitted_fid<=sfid[release_slot];if(state[release_slot]!=4)local_fault<=1;state[release_slot]<=0;end
 end end
 wire [31:0] diag_faults={24'd0,recovering,capture_hard_fault,tx_fault,input_fault,compute_fault,capture_fault,local_fault,fault};
 reg [31:0] diag_schedule;reg [31:0] diag_cycles;
 always @*begin
 diag_schedule=0;diag_schedule[2:0]=state[0];diag_schedule[5:3]=state[1];diag_schedule[8:6]=state[2];diag_schedule[11:9]=state[3];
 diag_schedule[12]=compute_busy;diag_schedule[13]=compute_ready;diag_schedule[14]=model_loaded;
 diag_schedule[15]=cap_commit;diag_schedule[16]=tx_valid;diag_schedule[17]=tx_ready;diag_schedule[18]=release_valid;diag_schedule[19]=release_ok;
 diag_schedule[21:20]=compute_slot;diag_schedule[23:22]=tx_slot;diag_schedule[25:24]=loaded_profile;diag_schedule[27:26]=desired_style;
 diag_schedule[29:28]=ui_layout;diag_schedule[30]=ui_fps;diag_schedule[31]=fault;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)diag_cycles<=0;else diag_cycles<=diag_cycles+1;
 wire [1407:0] words={
 {28'd0,owned_slots},{28'd0,recovery_owned_slots},recovery_fault_count,recovery_count,
 32'h33474441,diag_service,
 diag_cycles,{29'd0,diag_service_saturated,diag_tx_pixel_valid,diag_cam_valid},diag_tx_pixel,diag_tx_core,diag_compute,
 diag_cam,diag_cap_core,last_transmitted_fid,last_computed_fid,last_capture_fid,diag_schedule,
 computed,transmitted,dropped,captured,diag_faults,32'h41324454,32'd3};
 d64_status_uart #(.CLK_HZ(100000000),.WORDS(44),.PACKET_MAGIC(32'h33474441)) u_status(.clk(clk),.rst_n(rst_n),.words(words),.tx(txd));
endmodule
