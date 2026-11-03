`timescale 1ns/1ps
`default_nettype none
// B367 segmented numerical core and services, Ethernet lease/compute shell.
// No camera, guide, preview, display, mobile, HDMI receiver or transmitter.
`include "d64_blk_beats.vh"
module d2e_b_compute_system_fast #(
 parameter integer ROWS=36,OUT_W=256,
 parameter WT_INIT_FILE="../../profiles/vangogh/weights.mem",
 parameter integer SAB_SHAPE_STALE=0
)(
 input wire clk,clk_weight_fast,rst_n,ddr_ready,session_valid,
 input wire[31:0]local_session,remote_session,
 input wire allocate_valid,output wire allocate_ready,allocate_ok,output wire[31:0]allocate_base,
 input wire[31:0]allocate_fid,allocate_epoch,input wire[1:0]allocate_style,allocate_layout,
 input wire allocate_fps,input wire[7:0]allocate_effect,
 input wire commit_valid,commit_cancel,output wire commit_ready,
 input wire[31:0]commit_base,commit_fid,commit_epoch,commit_crc,
 input wire[1:0]commit_style,commit_layout,input wire commit_fps,input wire[7:0]commit_effect,
 input wire abort_valid,input wire[31:0]abort_base,abort_fid,
 output wire job_valid,input wire job_ready,output wire[31:0]job_base,job_fid,job_epoch,
 output wire[1:0]job_style,job_layout,output wire job_fps,output wire[7:0]job_effect,
 input wire tx_done,tx_done_ok,input wire[31:0]tx_done_fid,
 output wire[239:0]axi_cmd,input wire[137:0]axi_rsp,
 input wire[3:0]i_spi0_io,output wire[3:0]o_spi0_io,oe_spi0_io,
 output wire o_spi_clk,oe_spi_clk,o_spi0_cs,oe_spi0_cs,
 output wire admission_ready,model_ready,output wire[3:0]assets_ready,
 output wire fault_sticky,output wire[255:0]status_words,output wire[31:0]status_flags
);
 localparam integer H=ROWS*4,BLK_BEATS=`D64_ROWS_TO_BEATS(ROWS);
 localparam [3:0] STAGE_EN_INIT=4'b1111;
 localparam [31:0] TRUNK_A_BASE=32'h001c0000,TRUNK_B_BASE=32'h00400000;
 localparam CAWVALID=0,CAWADDR=1,CAWLEN=33,CAWSIZE=41,CAWBURST=44,
  CWVVALID=46,CWVDATA=47,CWVSTRB=175,CWVLAST=191,CBREDY=192,
  CARVALID=193,CARADDR=194,CARLEN=226,CARSIZE=234,CARBURST=237,CRREDY=239;
 localparam RAWREADY=0,RWREDY=1,RBVALID=2,RBRESPO=3,RARREADY=5,RRVALID=6,RRDATA=7,RRLAST=135,RBRESPR=136;
 wire[239:0]m_cmd[0:7],chain_cmd[0:6];wire[137:0]m_rsp[0:7],chain_rsp[0:6];
 genvar g;
 generate for(g=0;g<7;g=g+1)begin:g_join
  wire[239:0]left_cmd;wire[137:0]left_rsp;
  if(g==0)begin assign left_cmd=m_cmd[0];assign m_rsp[0]=left_rsp;end
  else begin assign left_cmd=chain_cmd[g-1];assign chain_rsp[g-1]=left_rsp;end
  d64_axi_join u_join(.clk(clk),.rst_n(rst_n),.m0_cmd(left_cmd),.m1_cmd(m_cmd[g+1]),
   .m0_rsp(left_rsp),.m1_rsp(m_rsp[g+1]),.s_cmd(chain_cmd[g]),.s_rsp(chain_rsp[g]));
 end endgenerate
 assign axi_cmd=chain_cmd[6];assign chain_rsp[6]=axi_rsp;
 wire compute_release,job_start,job_slot,tensor_imported,core_idle;
 wire[2:0]job_input_slot;wire[31:0]job_frame_id;
 wire t_req_buf,t_src_buf,t_dst_buf;wire[31:0]tail_address;
 wire[1:0]active_style;wire[15:0]active_epoch;wire[9:0]profile_zero;
 wire model_valid,block_jobs,profile_publish;
 wire load_start,load_done,load_failed,load_invalidate,load_busy;
 wire[1:0]load_style;wire load_request,load_stream_valid,load_stream_ready,load_stream_error,style_reader_idle;
 wire[31:0]load_address;wire[15:0]load_beats;wire[127:0]load_data;
 wire cfg_we;wire[8:0]cfg_index;wire[63:0]cfg_data;
 wire dec_we=cfg_we&&cfg_index>=376;wire[4:0]dec_wa=cfg_index-376;
 wire[239:0]profile_cmd,flash_cmd;wire[137:0]profile_rsp;
 wire[3:0]asset_ready;wire flash_busy,flash_done,flash_failed;wire[19:0]flash_probe;
 reg flash_started;
 wire ddr_cal_done=ddr_ready,ddr_cal_pass=ddr_ready;
 wire tensor_start,tensor_idle,tensor_done,tensor_failed,tensor_fault;wire[31:0]tensor_words;
 wire style_request;wire[1:0]style_request_id;wire safe_to_load,result_profile_commit;
 wire[15:0]result_profile_epoch;
 wire[7:0]row_bnd_seen,row_bnd_busy;
 wire[31:0]row_rd_base_chg,row_rf_ovf;wire[2:0]row_axi_blk;wire[3:0]row_par_ones;
 wire row_par_ok,row_sel_mismatch;
 wire[31:0]row_rd_beats,row_wr_beats,row_rd_bursts,row_wr_bursts,row_axi_errors;
 assign assets_ready=asset_ready;assign model_ready=model_valid;

    wire         core_axi_arst_n;
    wire         core_pe_arst_n;
    wire         stage_finished, transport_retired, stage_success;
    wire         reset_req;
    wire         core_pe_busy, core_axi_busy;
    wire [31:0]  reset_pulses;
    wire [3:0]   reset_reason;
    wire [31:0]  admit_rejects, job_fails;
    wire [31:0]  owner_viol, bank_mode_viol;
    wire [31:0]  ddr_reads, ddr_fills, ddr_bursts, cache_stalls, cache_viol;
    wire [31:0]  ups0_words, ups0_beats, ups0_bursts, ups0_acks;
    wire [31:0]  ups0_bresp_errors, ups0_overflows, ups0_duplicates, ups0_oob;
    wire [31:0]  ups1_bresp_errors, ups1_overflows, ups1_duplicates, ups1_oob;
    wire [31:0]  u0_slots, u0_writes, u0_rq_overrun, u0_rq_wait, u0_stall_cyc;
    wire [31:0]  u1_slots, tl_pixels, tl_wr_responses;
    wire [31:0]  frame_cnt;
    wire [31:0]  cache_region_base, cache_region_span, cache_region_waits;
    wire         cache_region_req, cache_region_ack;
    wire [1:0]   dec_layer;
    wire         store_owner, dec_owns_store_seen;
    wire         core_failed;

    wire         out_valid, out_ready, out_sof, out_eol, out_eof;
    wire [7:0]   out_x, out_y;
    wire [1:0]   out_word;
    wire [79:0]  out_data;
    wire [31:0]  out_frame_id;

    wire [6:0]   x_req_row;
    wire         x_req_tog, x_ack_tog;
    wire         rf_wr_en, rf_wr_full;
    wire [127:0] rf_wr_data;
    wire         wf_rd_en, wf_rd_valid, wf_rd_empty;
    wire [127:0] wf_rd_data;
    wire [11:0] wf_rd_level;
    wire         wr_done_tog;

    wire [1:0]   u1_in_rd_en;
    wire [31:0]  u1_in_rd_addr;
    wire [5:0]   tl_in_rd_en;
    wire [95:0]  tl_in_rd_addr;

    wire         core_start;
    wire [3:0]   stage_en;
    wire [2:0]   stage;
    wire         core_busy, core_done;

    wire         d64_ready;
    wire         raw_sof_w, raw_sol_w, raw_eol_w, raw_eof_w;


    wire hs_valid, hs_ready, hs_sof, hs_eol, hs_eof, hs_commit;
    wire head_phase_ready;
    wire [7:0]  hs_x, hs_y;
    wire [63:0] hs_c8;
    wire [31:0] hs_fid;
    wire hr_start, hr_valid, hr_ready, hr_sof, hr_eol, hr_eof;
    wire [7:0]  hr_x, hr_y;
    wire [63:0] hr_c8;
    wire [31:0] hr_fid;
    wire d0s_valid, d0s_ready, d0s_sof, d0s_eol, d0s_eof, d0s_commit;
    wire [7:0]  d0s_x, d0s_y;
    wire [1:0]  d0s_word;
    wire [79:0] d0s_data;
    wire [31:0] d0s_fid;
    wire d0r_start, d0r_valid, d0r_ready, d0r_icg, d0r_sof, d0r_eol, d0r_eof;
    wire [7:0]  d0r_x, d0r_y;
    wire [63:0] d0r_c8;
    wire [31:0] d0r_fid;

    wire        trunk_committed, trunk_commit_axi;
    wire [31:0] trunk_store_commit_wait_cycles;
    wire [31:0] h_write_beats, h_read_beats, d0_write_beats, d0_read_beats;
    wire [31:0] trunk_write_beats;
    wire [31:0] h_write_bursts, h_read_bursts, d0_write_bursts, d0_read_bursts;
    wire [31:0] trunk_write_bursts;
    wire [31:0] mem_axi_response_errors, stream_pack_errors;
    wire [8:0]  write_fifo_max_occupancy, read_fifo_max_occupancy;
    wire [31:0] read_fifo_overflow, rready_drop_in_burst;

    reg [3:0] fence_cnt;
    wire fence_ok = &fence_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fence_cnt <= 4'd0;
        else if (trunk_committed && !(&fence_cnt)) fence_cnt <= fence_cnt + 4'd1;
    end
    wire trunk_committed_fenced = trunk_committed && fence_ok;

    // Active weights initialize in place; all runtime profiles come from DDR.
    wire wld_done=model_valid;
    wire ldr_run=!model_valid;
    wire wld_we,ldr_we;wire[8:0] wld_wa;wire[511:0] wld_wd;
    wire[8:0] ldr_wa;wire[63:0] ldr_dout;
    assign stage_en=STAGE_EN_INIT;
    assign core_start=job_start;
    wire         d64_valid, d64_sof, d64_eol, d64_eof;
    wire [31:0]  d64_frame_id;
    wire [7:0]   d64_x0, d64_y;
    wire [383:0] d64_rgb888;

    wire [31:0] cam_src_samples, cam_src_lines, cam_src_frames;
    wire [31:0] cam_demos_px, cam_out_px, cam_beats, cam_frames_out;
    wire [31:0] cam_dropped, cam_short_lines;
    wire        cam_geom_margin_ok;

    wire scalar_cap_valid,scalar_cap_sof,scalar_cap_eol,scalar_cap_eof;
    wire [31:0] scalar_cap_fid;
    wire [7:0] scalar_cap_x,scalar_cap_y;
    wire [383:0] scalar_cap_rgb;
    wire [31:0] scalar_samples,scalar_lines,scalar_frames,scalar_demos,scalar_outpx;
    wire [31:0] scalar_beats,scalar_frames_out,scalar_dropped,scalar_short;
    wire scalar_margin;
    wire scalar_guide_valid;
    wire [7:0] scalar_guide_gray;
    wire [15:0] scalar_guide_x,scalar_guide_y;
    wire [31:0] scalar_guide_fid;



    d64_network_core #(
        .ENABLE_ENCODER(0),.ENABLE_DECODER(1),.TRUNK_FIRST_BLOCK(0),.TRUNK_BLOCK_COUNT(5),
        .IMG_H(H), .IMG_W(OUT_W), .SAB_SHAPE_STALE(SAB_SHAPE_STALE),
        .WT_INIT_FILE(WT_INIT_FILE), .RUNTIME_PROFILE(1), .FAULT_STICKY(1),
        .TRUNK_CFG_BIAS_W(20), .DEC_CFG_BIAS_W(13), .CFG_STREAM_SRL(1),
        .HEAD_STORE_CREDIT_ENABLE(1),
        .TAIL_RESERVE_WRITE_BURST(1), .BQ_DEPTH(256)
    ) u_core (

        .clk_pe(clk), .clk_axi(clk), .arst_n(rst_n & compute_release),
        .clk_weight_fast(clk_weight_fast),
        .start(core_start), .stage_en(stage_en),
        .stage(stage), .busy(core_busy), .done(core_done), .failed(core_failed),
        .can_accept(core_idle), .profile_style(active_style), .profile_trunk_zero(profile_zero),
        .dec_cfg_we(dec_we), .dec_cfg_addr(dec_wa), .dec_cfg_data(cfg_data),

        .d64_valid(d64_valid), .d64_ready(d64_ready),
        .d64_frame_id(d64_frame_id), .d64_sof(d64_sof),
        .d64_x0(d64_x0), .d64_y(d64_y), .d64_rgb888(d64_rgb888),
        .d64_eol(d64_eol), .d64_eof(d64_eof),

        .wt_wr_en(wld_we), .wt_wr_addr(wld_wa), .wt_wr_data(wld_wd),
        .t_cfg_we(ldr_we), .t_cfg_wa(ldr_wa), .t_cfg_wd(ldr_dout),
        .blk_redo(1'b0), .t_cfg_selftest(!ldr_run),
        .trunk_store_committed(trunk_committed_fenced),

        .head_store_wr_valid(hs_valid), .head_store_wr_ready(hs_ready),
        .head_phase_ready(head_phase_ready),
        .head_store_wr_x(hs_x), .head_store_wr_y(hs_y),
        .head_store_wr_c8(hs_c8), .head_store_wr_frame_id(hs_fid),
        .head_store_wr_sof(hs_sof), .head_store_wr_eol(hs_eol),
        .head_store_wr_eof(hs_eof),
        .head_store_commit_ready(hs_commit),
        .head_read_start(hr_start), .head_read_valid(hr_valid),
        .head_read_ready(hr_ready), .head_read_x(hr_x), .head_read_y(hr_y),
        .head_read_c8(hr_c8), .head_read_frame_id(hr_fid),
        .head_read_sof(hr_sof), .head_read_eol(hr_eol), .head_read_eof(hr_eof),

        .down0_store_wr_valid(d0s_valid), .down0_store_wr_ready(d0s_ready),
        .down0_store_wr_x(d0s_x), .down0_store_wr_y(d0s_y),
        .down0_store_wr_word(d0s_word), .down0_store_wr_data(d0s_data),
        .down0_store_wr_frame_id(d0s_fid),
        .down0_store_wr_sof(d0s_sof), .down0_store_wr_eol(d0s_eol),
        .down0_store_wr_eof(d0s_eof),
        .down0_store_commit_ready(d0s_commit),
        .down0_read_start(d0r_start), .down0_read_valid(d0r_valid),
        .down0_read_ready(d0r_ready), .down0_read_x(d0r_x), .down0_read_y(d0r_y),
        .down0_read_icg(d0r_icg), .down0_read_c8(d0r_c8),
        .down0_read_frame_id(d0r_fid), .down0_read_sof(d0r_sof),
        .down0_read_eol(d0r_eol), .down0_read_eof(d0r_eof),

        .out_valid(out_valid), .out_ready(out_ready),
        .out_x(out_x), .out_y(out_y), .out_word(out_word),
        .out_data(out_data), .out_frame_id(out_frame_id),
        .out_sof(out_sof), .out_eol(out_eol), .out_eof(out_eof),

        .x_req_row(x_req_row), .x_req_tog(x_req_tog), .x_ack_tog(x_ack_tog),
        .t_req_buf(t_req_buf), .t_src_buf(t_src_buf), .t_dst_buf(t_dst_buf),
        .rf_wr_en(rf_wr_en), .rf_wr_data(rf_wr_data), .rf_wr_full(rf_wr_full),
        .wf_rd_en(wf_rd_en), .wf_rd_data(wf_rd_data),
        .wf_rd_valid(wf_rd_valid), .wf_rd_empty(wf_rd_empty), .wf_rd_level(wf_rd_level),
        .wr_done_tog(wr_done_tog),

        .u1_in_rd_en(u1_in_rd_en), .u1_in_rd_addr(u1_in_rd_addr),
        .u1_in_rd_data(128'd0), .u1_in_stall(1'b1),
        .tl_in_rd_en(tl_in_rd_en), .tl_in_rd_addr(tl_in_rd_addr),
        .tl_in_rd_data(384'd0), .tl_in_stall(1'b1),

        .cache_region_base(cache_region_base),
        .cache_region_span(cache_region_span),
        .cache_region_req(cache_region_req), .cache_region_ack(cache_region_ack),
        .cache_region_waits(cache_region_waits),
        .dec_layer(dec_layer), .store_owner(store_owner),

        .rd_axi_arvalid(m_cmd[0][CARVALID]),
        .rd_axi_arready(m_rsp[0][RARREADY]),
        .rd_axi_araddr(m_cmd[0][CARADDR +: 32]),
        .rd_axi_arlen(m_cmd[0][CARLEN +: 8]),
        .rd_axi_arsize(m_cmd[0][CARSIZE +: 3]),
        .rd_axi_arburst(m_cmd[0][CARBURST +: 2]),
        .rd_axi_rvalid(m_rsp[0][RRVALID]), .rd_axi_rready(m_cmd[0][CRREDY]),
        .rd_axi_rdata(m_rsp[0][RRDATA +: 128]),
        .rd_axi_rlast(m_rsp[0][RRLAST]),
        .rd_axi_rresp(m_rsp[0][RBRESPR +: 2]),

        .w0_axi_awvalid(m_cmd[1][CAWVALID]),
        .w0_axi_awready(m_rsp[1][RAWREADY]),
        .w0_axi_awaddr(m_cmd[1][CAWADDR +: 32]),
        .w0_axi_awlen(m_cmd[1][CAWLEN +: 8]),
        .w0_axi_awsize(m_cmd[1][CAWSIZE +: 3]),
        .w0_axi_awburst(m_cmd[1][CAWBURST +: 2]),
        .w0_axi_wvalid(m_cmd[1][CWVVALID]), .w0_axi_wready(m_rsp[1][RWREDY]),
        .w0_axi_wdata(m_cmd[1][CWVDATA +: 128]),
        .w0_axi_wstrb(m_cmd[1][CWVSTRB +: 16]),
        .w0_axi_wlast(m_cmd[1][CWVLAST]),
        .w0_axi_bvalid(m_rsp[1][RBVALID]), .w0_axi_bready(m_cmd[1][CBREDY]),
        .w0_axi_bresp(m_rsp[1][RBRESPO +: 2]),

        .w1_axi_awvalid(m_cmd[2][CAWVALID]),
        .w1_axi_awready(m_rsp[2][RAWREADY]),
        .w1_axi_awaddr(m_cmd[2][CAWADDR +: 32]),
        .w1_axi_awlen(m_cmd[2][CAWLEN +: 8]),
        .w1_axi_awsize(m_cmd[2][CAWSIZE +: 3]),
        .w1_axi_awburst(m_cmd[2][CAWBURST +: 2]),
        .w1_axi_wvalid(m_cmd[2][CWVVALID]), .w1_axi_wready(m_rsp[2][RWREDY]),
        .w1_axi_wdata(m_cmd[2][CWVDATA +: 128]),
        .w1_axi_wstrb(m_cmd[2][CWVSTRB +: 16]),
        .w1_axi_wlast(m_cmd[2][CWVLAST]),
        .w1_axi_bvalid(m_rsp[2][RBVALID]), .w1_axi_bready(m_cmd[2][CBREDY]),
        .w1_axi_bresp(m_rsp[2][RBRESPO +: 2]),

        .tw_axi_awvalid(m_cmd[3][CAWVALID]),
        .tw_axi_awready(m_rsp[3][RAWREADY]),
        .tw_axi_awaddr(tail_address),
        .tw_axi_awlen(m_cmd[3][CAWLEN +: 8]),
        .tw_axi_awsize(m_cmd[3][CAWSIZE +: 3]),
        .tw_axi_awburst(m_cmd[3][CAWBURST +: 2]),
        .tw_axi_wvalid(m_cmd[3][CWVVALID]), .tw_axi_wready(m_rsp[3][RWREDY]),
        .tw_axi_wdata(m_cmd[3][CWVDATA +: 128]),
        .tw_axi_wstrb(m_cmd[3][CWVSTRB +: 16]),
        .tw_axi_wlast(m_cmd[3][CWVLAST]),
        .tw_axi_bvalid(m_rsp[3][RBVALID]), .tw_axi_bready(m_cmd[3][CBREDY]),
        .tw_axi_bresp(m_rsp[3][RBRESPO +: 2]),

        .reset_req(reset_req), .pe_arst_n(core_pe_arst_n),
        .axi_arst_n(core_axi_arst_n),
        .pe_busy(core_pe_busy), .axi_busy(core_axi_busy),

        .frame_cnt(frame_cnt), .owner_viol(owner_viol),
        .bank_mode_viol(bank_mode_viol), .reset_pulses(reset_pulses),
        .dec_owns_store_seen(dec_owns_store_seen),
        .ddr_reads(ddr_reads), .ddr_fills(ddr_fills), .ddr_bursts(ddr_bursts),
        .cache_stalls(cache_stalls), .cache_viol(cache_viol),
        .ups0_words(ups0_words), .ups0_beats(ups0_beats),
        .ups0_bursts(ups0_bursts), .ups0_acks(ups0_acks),
        .ups0_bresp_errors(ups0_bresp_errors), .ups0_overflows(ups0_overflows),
        .u0_slots(u0_slots), .u0_writes(u0_writes),
        .u0_rq_overrun(u0_rq_overrun), .u0_rq_wait(u0_rq_wait),
        .u0_stall_cyc(u0_stall_cyc), .u1_slots(u1_slots),
        .tl_pixels(tl_pixels), .tl_wr_responses(tl_wr_responses),
        .stage_finished(stage_finished),
        .transport_retired(transport_retired),
        .stage_success(stage_success),
        .reset_reason(reset_reason), .admit_rejects(admit_rejects),
        .job_fails(job_fails),
        .ups0_duplicates(ups0_duplicates), .ups0_oob(ups0_oob),
        .ups1_bresp_errors(ups1_bresp_errors), .ups1_overflows(ups1_overflows),
        .ups1_duplicates(ups1_duplicates), .ups1_oob(ups1_oob)
    );

    d64_trunk_row_ddr #(.TRUNK_FIRST_BLOCK(0),.TRUNK_BLOCK_COUNT(5),.BLK_BEATS(BLK_BEATS),.RESERVE_WRITE_BURST(1),.WF_LEVEL_W(12)) u_row (
        .clk_pe(clk), .clk_axi(clk), .arst_n(rst_n & compute_release),
        .hold_blk(1'b0),
        .buf_a_base(TRUNK_A_BASE), .buf_b_base(TRUNK_B_BASE),
        .x_req_row(x_req_row),

        .x_req_buf(t_req_buf), .src_buf(t_src_buf), .dst_buf(t_dst_buf),
        .x_req_tog(x_req_tog), .x_ack_tog(x_ack_tog),
        .rd_base_chg(row_rd_base_chg),
        .rf_wr_en(rf_wr_en), .rf_wr_data(rf_wr_data), .rf_wr_full(rf_wr_full),
        .wf_rd_en(wf_rd_en), .wf_rd_data(wf_rd_data),
        .wf_rd_valid(wf_rd_valid), .wf_rd_empty(wf_rd_empty), .wf_rd_level(wf_rd_level),
        .wr_done_tog(wr_done_tog),
        .bnd_seen(row_bnd_seen), .bnd_busy(row_bnd_busy),
        .axi_blk_out(row_axi_blk), .par_ones_out(row_par_ones),
        .par_ok_out(row_par_ok), .sel_mismatch_out(row_sel_mismatch),
        .rf_ovf_out(row_rf_ovf),
        .axi_arvalid(m_cmd[5][CARVALID]), .axi_arready(m_rsp[5][RARREADY]),
        .axi_araddr(m_cmd[5][CARADDR +: 32]),
        .axi_arlen(m_cmd[5][CARLEN +: 8]),
        .axi_arsize(m_cmd[5][CARSIZE +: 3]),
        .axi_arburst(m_cmd[5][CARBURST +: 2]),
        .axi_rvalid(m_rsp[5][RRVALID]), .axi_rready(m_cmd[5][CRREDY]),
        .axi_rdata(m_rsp[5][RRDATA +: 128]),
        .axi_rlast(m_rsp[5][RRLAST]),
        .axi_rresp(m_rsp[5][RBRESPR +: 2]),
        .axi_awvalid(m_cmd[5][CAWVALID]), .axi_awready(m_rsp[5][RAWREADY]),
        .axi_awaddr(m_cmd[5][CAWADDR +: 32]),
        .axi_awlen(m_cmd[5][CAWLEN +: 8]),
        .axi_awsize(m_cmd[5][CAWSIZE +: 3]),
        .axi_awburst(m_cmd[5][CAWBURST +: 2]),
        .axi_wvalid(m_cmd[5][CWVVALID]), .axi_wready(m_rsp[5][RWREDY]),
        .axi_wdata(m_cmd[5][CWVDATA +: 128]),
        .axi_wstrb(m_cmd[5][CWVSTRB +: 16]),
        .axi_wlast(m_cmd[5][CWVLAST]),
        .axi_bvalid(m_rsp[5][RBVALID]), .axi_bready(m_cmd[5][CBREDY]),
        .axi_bresp(m_rsp[5][RBRESPO +: 2]),
        .rd_beats(row_rd_beats), .wr_beats(row_wr_beats),
        .rd_bursts(row_rd_bursts), .wr_bursts(row_wr_bursts),
        .axi_errors(row_axi_errors)
    );

    d64_style_loader u_style_loader(.clk(clk),.rst_n(rst_n),.start(load_start),.style(load_style),
        .request(load_request),.profile_addr(load_address),.profile_beats(load_beats),
        .stream_valid(load_stream_valid),.stream_ready(load_stream_ready),.stream_data(load_data),.stream_error(load_stream_error),
        .wt_we(wld_we),.wt_addr(wld_wa),.wt_data(wld_wd),.cfg_we(cfg_we),.cfg_index(cfg_index),.cfg_data(cfg_data),
        .profile_trunk_zero(profile_zero),.busy(load_busy),.done(load_done),.failed(load_failed),.invalidate(load_invalidate),.error_code());
    d64_style_ddr_reader #(.MAX_BURST_BEATS(1),.ABORT_EN(1),.RESERVE_AR_READY(1)) u_profile_reader(.clk(clk),.rst_n(rst_n),.abort(load_failed),
        .request(load_request),.base_addr(load_address),.beats(load_beats),.stream_valid(load_stream_valid),
        .stream_ready(load_stream_ready),.stream_data(load_data),.stream_error(load_stream_error),.idle(style_reader_idle),
        .arvalid(profile_cmd[CARVALID]),.arready(profile_rsp[RARREADY]),.araddr(profile_cmd[CARADDR+:32]),
        .arlen(profile_cmd[CARLEN+:8]),.arsize(profile_cmd[CARSIZE+:3]),.arburst(profile_cmd[CARBURST+:2]),
        .rvalid(profile_rsp[RRVALID]),.rready(profile_cmd[CRREDY]),.rdata(profile_rsp[RRDATA+:128]),
        .rlast(profile_rsp[RRLAST]),.rresp(profile_rsp[RBRESPR+:2]));
    assign profile_cmd[CAWVALID]=0;assign profile_cmd[CWVVALID]=0;assign profile_cmd[CBREDY]=1;
    assign profile_cmd[CAWADDR+:32]=0;assign profile_cmd[CAWLEN+:8]=0;
    assign profile_cmd[CAWSIZE+:3]=4;assign profile_cmd[CAWBURST+:2]=1;
    assign profile_cmd[CWVDATA+:128]=0;assign profile_cmd[CWVSTRB+:16]=0;
    assign profile_cmd[CWVLAST]=0;
    d64_flash_asset_boot u_flash(.clk(clk),.rst_n(rst_n),
        .start(!flash_started&&ddr_cal_done&&ddr_cal_pass),.spi_clk(o_spi_clk),.spi_cs_n(o_spi0_cs),
        .spi_mosi(o_spi0_io[0]),.spi_miso(i_spi0_io[1]),
        .asset_ready(asset_ready),.observe_boot(flash_probe),.busy(flash_busy),.done(flash_done),.failed(flash_failed),
        .awaddr(flash_cmd[CAWADDR+:32]),.awlen(flash_cmd[CAWLEN+:8]),.awsize(flash_cmd[CAWSIZE+:3]),
        .awburst(flash_cmd[CAWBURST+:2]),.awvalid(flash_cmd[CAWVALID]),.awready(m_rsp[7][RAWREADY]),
        .wdata(flash_cmd[CWVDATA+:128]),.wstrb(flash_cmd[CWVSTRB+:16]),.wlast(flash_cmd[CWVLAST]),
        .wvalid(flash_cmd[CWVVALID]),.wready(m_rsp[7][RWREDY]),.bvalid(m_rsp[7][RBVALID]),
        .bready(flash_cmd[CBREDY]),.bresp(m_rsp[7][RBRESPO+:2]));
    assign flash_cmd[CARVALID]=0;assign flash_cmd[CRREDY]=1;
    assign flash_cmd[CARADDR+:32]=0;assign flash_cmd[CARLEN+:8]=0;
    assign flash_cmd[CARSIZE+:3]=4;assign flash_cmd[CARBURST+:2]=1;
    assign o_spi0_io[3:1]=3'b110;assign oe_spi0_io=4'b1101;assign oe_spi_clk=1;assign oe_spi0_cs=1;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)flash_started<=0;else if(ddr_cal_done&&ddr_cal_pass)flash_started<=1;




 assign m_cmd[0][192:0]={1'b1,192'd0};
 assign m_cmd[1][239:193]={1'b1,46'd0};
 assign m_cmd[2][239:193]={1'b1,46'd0};
 assign m_cmd[3][239:193]={1'b1,46'd0};
 assign m_cmd[3][CAWADDR+:32]=tail_address+(job_slot?32'h00040000:0);
 assign m_cmd[6]=profile_cmd;assign profile_rsp=m_rsp[6];assign m_cmd[7]=flash_cmd;
 d2_b_tensor_copy #(.DEST_BASE(TRUNK_A_BASE)) u_import(.clk(clk),.rst_n(rst_n),.start(tensor_start),.slot(job_input_slot),
  .idle(tensor_idle),.done(tensor_done),.failed(tensor_failed),.fault_sticky(tensor_fault),.copied_words(tensor_words),
  .axi_cmd(m_cmd[4]),.axi_rsp(m_rsp[4]));
 assign trunk_committed=tensor_imported;
 assign d64_valid=0;assign d64_sof=0;assign d64_eol=0;assign d64_eof=0;
 assign d64_frame_id=job_frame_id;assign d64_x0=0;assign d64_y=0;assign d64_rgb888=0;
 assign hs_ready=0;assign hs_commit=0;assign head_phase_ready=0;
 assign hr_valid=0;assign hr_x=0;assign hr_y=0;assign hr_c8=0;assign hr_fid=0;assign hr_sof=0;assign hr_eol=0;assign hr_eof=0;
 assign d0s_ready=0;assign d0s_commit=0;assign d0r_valid=0;assign d0r_x=0;assign d0r_y=0;assign d0r_c8=0;assign d0r_fid=0;
 assign d0r_icg=0;assign d0r_sof=0;assign d0r_eol=0;assign d0r_eof=0;assign out_ready=1;
 assign ldr_we=cfg_we&&cfg_index>=56&&cfg_index<376;assign ldr_wa=cfg_index-56;assign ldr_dout=cfg_data;
 wire switch_done,switch_fault,deadline_missed;wire[31:0]switch_cycles,boot_total_cycles;
 wire boot_wait_source,boot_source_arm,boot_active;
 d64_style_switch #(.BOOT_VALID(0),.SWITCH_BUDGET_CYCLES(9800000))u_style_switch(
  .clk(clk),.rst_n(rst_n),.request_valid(style_request),.request_style(style_request_id),
  .request_is_boot(1'b0),.bootstrap_source_ready(1'b0),.asset_ready(asset_ready),.safe_to_load(safe_to_load),
  .load_start(load_start),.load_style(load_style),.load_done(load_done),.load_failed(load_failed),.load_invalidate(load_invalidate),
  .block_new_jobs(block_jobs),.model_valid(model_valid),.active_style(active_style),.active_epoch(active_epoch),
  .profile_publish(profile_publish),.display_commit(result_profile_commit),.display_epoch(result_profile_epoch),
  .switch_done(switch_done),.switch_fault(switch_fault),.deadline_missed(deadline_missed),.switch_cycles(switch_cycles),
  .boot_wait_source(boot_wait_source),.boot_source_arm(boot_source_arm),.boot_active(boot_active),.boot_total_cycles(boot_total_cycles));
 // Count accepted transactions through this shell's one external AXI service.
 // Session changes never reset these counters, arbiters or their bus owners.
 reg[4:0]write_pending,read_pending;reg bus_fault;
 wire aw_fire=axi_cmd[0]&&axi_rsp[0],b_fire=axi_cmd[192]&&axi_rsp[2];
 wire ar_fire=axi_cmd[193]&&axi_rsp[5],r_last=axi_cmd[239]&&axi_rsp[6]&&axi_rsp[135];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin write_pending<=0;read_pending<=0;bus_fault<=0;end
  else begin
   case({aw_fire,b_fire})2'b10:write_pending<=write_pending+1'b1;2'b01:write_pending<=write_pending-1'b1;endcase
   case({ar_fire,r_last})2'b10:read_pending<=read_pending+1'b1;2'b01:read_pending<=read_pending-1'b1;endcase
   if(b_fire&&(write_pending==0&&!aw_fire||axi_rsp[4:3]!=0))bus_fault<=1;
   if(axi_cmd[239]&&axi_rsp[6]&&(read_pending==0&&!ar_fire||axi_rsp[137:136]!=0))bus_fault<=1;
  end
 end
 wire bus_quiet=write_pending==0&&read_pending==0&&!axi_cmd[0]&&!axi_cmd[46]&&!axi_cmd[193];
 reg[15:0]tail_aw_count,tail_b_count;reg[31:0]tail_beats;reg tail_fault;
 wire tail_aw=m_cmd[3][0]&&m_rsp[3][0],tail_b=m_cmd[3][192]&&m_rsp[3][2];
 wire[31:0]selected_tail_base=job_slot?32'h00600000:32'h005c0000;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin tail_aw_count<=0;tail_b_count<=0;tail_beats<=0;tail_fault<=0;end
  else if(!compute_release)begin tail_aw_count<=0;tail_b_count<=0;tail_beats<=0;tail_fault<=0;end
  else begin
   if(tail_aw)begin
    tail_aw_count<=tail_aw_count+1'b1;tail_beats<=tail_beats+{24'd0,m_cmd[3][40:33]}+1'b1;
    if(m_cmd[3][32:1]<selected_tail_base||m_cmd[3][32:1]+(({24'd0,m_cmd[3][40:33]}+1)<<4)>selected_tail_base+H*OUT_W*4)tail_fault<=1;
   end
   if(tail_b)begin tail_b_count<=tail_b_count+1'b1;if(m_rsp[3][4:3]!=0)tail_fault<=1;end
  end
 end
 wire tail_bus_clean=!tail_fault&&tail_aw_count==H*OUT_W/256&&tail_b_count==H*OUT_W/256&&tail_beats==H*OUT_W/4;
 wire runtime_fault=flash_failed||load_failed||switch_fault||tensor_fault||core_failed||row_axi_errors!=0||bus_fault||tail_fault;
 assign admission_ready=ddr_ready&&((asset_ready&4'b1110)==4'b1110)&&!flash_failed&&!fault_sticky&&!runtime_fault;
 d2e_b_compute_control_fast #(.OUT_PIXELS(H*OUT_W),.TAIL_RESPONSES(H*OUT_W/256))u_control(
  .clk(clk),.rst_n(rst_n),.admission(admission_ready),.session_valid(session_valid),.local_session(local_session),.remote_session(remote_session),
  .allocate_valid(allocate_valid),.allocate_ready(allocate_ready),.allocate_ok(allocate_ok),.allocate_base(allocate_base),
  .allocate_fid(allocate_fid),.allocate_epoch(allocate_epoch),.allocate_style(allocate_style),.allocate_layout(allocate_layout),.allocate_fps(allocate_fps),.allocate_effect(allocate_effect),
  .commit_valid(commit_valid),.commit_cancel(commit_cancel),.commit_ready(commit_ready),.commit_base(commit_base),.commit_fid(commit_fid),.commit_epoch(commit_epoch),.commit_crc(commit_crc),
  .commit_style(commit_style),.commit_layout(commit_layout),.commit_fps(commit_fps),.commit_effect(commit_effect),.abort_valid(abort_valid),.abort_base(abort_base),.abort_fid(abort_fid),
  .job_valid(job_valid),.job_ready(job_ready),.job_base(job_base),.job_fid(job_fid),.job_epoch(job_epoch),.job_style(job_style),.job_layout(job_layout),.job_fps(job_fps),.job_effect(job_effect),
  .tx_done(tx_done),.tx_done_ok(tx_done_ok),.tx_done_fid(tx_done_fid),
  .model_valid(model_valid),.block_jobs(block_jobs),.load_busy(load_busy),.style_reader_idle(style_reader_idle),.active_style(active_style),.active_epoch(active_epoch),
  .style_request(style_request),.style_request_id(style_request_id),.safe_to_load(safe_to_load),.result_profile_commit(result_profile_commit),.result_profile_epoch(result_profile_epoch),
  .core_idle(core_idle),.core_done(core_done),.core_failed(core_failed),.core_transport_retired(transport_retired),.core_stage_success(stage_success),.core_stage(stage),
  .tail_pixels(tl_pixels),.tail_responses(tl_wr_responses),.copy_idle(tensor_idle),.copy_done(tensor_done),.copy_failed(tensor_failed),.bus_quiet(bus_quiet),.runtime_fault(runtime_fault),.tail_bus_clean(tail_bus_clean),
  .compute_release(compute_release),.core_start(job_start),.copy_start(tensor_start),.copy_slot(job_input_slot),.output_index(job_slot),.compute_fid(job_frame_id),.tensor_imported(tensor_imported),
  .status_words(status_words),.status_flags(status_flags),.fault_sticky(fault_sticky));
endmodule

// Ethernet lease/control shell. Numerical compute is outside this module.
module d2e_b_compute_control_fast #(
 parameter integer OUT_PIXELS=36864,TAIL_RESPONSES=144
)(
 input wire clk,rst_n,admission,session_valid,
 input wire[31:0]local_session,remote_session,
 input wire allocate_valid,output wire allocate_ready,allocate_ok,output wire[31:0]allocate_base,
 input wire[31:0]allocate_fid,allocate_epoch,input wire[1:0]allocate_style,allocate_layout,
 input wire allocate_fps,input wire[7:0]allocate_effect,
 input wire commit_valid,commit_cancel,output wire commit_ready,
 input wire[31:0]commit_base,commit_fid,commit_epoch,commit_crc,
 input wire[1:0]commit_style,commit_layout,input wire commit_fps,input wire[7:0]commit_effect,
 input wire abort_valid,input wire[31:0]abort_base,abort_fid,
 output wire job_valid,input wire job_ready,output wire[31:0]job_base,job_fid,job_epoch,
 output wire[1:0]job_style,job_layout,output wire job_fps,output wire[7:0]job_effect,
 input wire tx_done,tx_done_ok,input wire[31:0]tx_done_fid,
 input wire model_valid,block_jobs,load_busy,style_reader_idle,
 input wire[1:0]active_style,input wire[15:0]active_epoch,
 output reg style_request,output reg[1:0]style_request_id,output wire safe_to_load,
 output wire result_profile_commit,output wire[15:0]result_profile_epoch,
 input wire core_idle,core_done,core_failed,core_transport_retired,core_stage_success,
 input wire[2:0]core_stage,input wire[31:0]tail_pixels,tail_responses,
 input wire copy_idle,copy_done,copy_failed,bus_quiet,runtime_fault,tail_bus_clean,
 output reg compute_release,core_start,copy_start,
 output reg[2:0]copy_slot,output reg output_index,
 output reg[31:0]compute_fid,output reg tensor_imported,
 output wire[255:0]status_words,output wire[31:0]status_flags,
 output reg fault_sticky
);
 localparam FREE=0,RECEIVING=1,READY=2,COPYING=3;
 localparam R_FREE=0,R_COMPUTE=1,R_READY=2,R_TX=3,R_FAILED=4;
 localparam IDLE=0,ASSERT_CORE=1,COPY=2,WARM=3,RUN=4,FINISH=5,DRAIN=6,FAULT=7;
 reg[2:0]state,input_state[0:7],result_state[0:1];
 reg[31:0]in_fid[0:7],in_epoch[0:7],in_local[0:7],in_remote[0:7],in_crc[0:7],in_order[0:7];
 reg[1:0]in_style[0:7],in_layout[0:7];reg in_fps[0:7];reg[7:0]in_effect[0:7];
 reg[31:0]out_fid[0:1],out_epoch[0:1],out_local[0:1],out_remote[0:1],out_order[0:1];
 reg[1:0]out_style[0:1],out_layout[0:1];reg out_fps[0:1];reg[7:0]out_effect[0:1];
 reg[15:0]out_profile_epoch[0:1];
 reg[31:0]next_order,next_result_order;
 reg free_valid,result_free_valid,result_ready_valid;
 reg[2:0]free_index;reg free_result,ready_result;
 reg[31:0]best_order;reg duplicate_live;
 reg[7:0]input_owned;reg[1:0]result_owned;
 integer i;
 always @*begin
  free_valid=0;free_index=0;best_order=0;duplicate_live=0;input_owned=0;
  for(i=0;i<8;i=i+1)begin
   input_owned[i]=input_state[i]!=FREE;
   if(!free_valid&&input_state[i]==FREE)begin free_valid=1;free_index=i;end
   if(input_state[i]!=FREE&&in_local[i]==local_session&&in_remote[i]==remote_session&&in_fid[i]==allocate_fid&&in_epoch[i]==allocate_epoch)duplicate_live=1;
  end
  result_free_valid=0;result_ready_valid=0;free_result=0;ready_result=0;best_order=0;result_owned=0;
  for(i=0;i<2;i=i+1)begin
   result_owned[i]=result_state[i]!=R_FREE;
   if(!result_free_valid&&result_state[i]==R_FREE)begin result_free_valid=1;free_result=i;end
   if(result_state[i]==R_READY&&out_local[i]==local_session&&out_remote[i]==remote_session&&
      (!result_ready_valid||$signed(out_order[i]-best_order)<0))begin result_ready_valid=1;ready_result=i;best_order=out_order[i];end
   if(result_state[i]!=R_FREE&&out_local[i]==local_session&&out_remote[i]==remote_session&&out_fid[i]==allocate_fid&&out_epoch[i]==allocate_epoch)duplicate_live=1;
  end
 end
 assign allocate_ready=rst_n;
 assign allocate_ok=admission&&session_valid&&!fault_sticky&&free_valid&&!duplicate_live&&allocate_style<3&&allocate_layout<3;
 assign allocate_base=32'h04000000+({29'd0,free_index}<<18);
 wire commit_address_ok=commit_base>=32'h04000000&&commit_base<32'h04200000&&commit_base[17:0]==0;
 wire abort_address_ok=abort_base>=32'h04000000&&abort_base<32'h04200000&&abort_base[17:0]==0;
 wire[2:0]commit_slot=commit_base[20:18],abort_slot=abort_base[20:18];
 wire commit_lease=commit_address_ok&&input_state[commit_slot]==RECEIVING&&in_fid[commit_slot]==commit_fid;
 wire commit_meta=commit_lease&&in_epoch[commit_slot]==commit_epoch&&in_style[commit_slot]==commit_style&&
  in_layout[commit_slot]==commit_layout&&in_fps[commit_slot]==commit_fps&&in_effect[commit_slot]==commit_effect;
 wire commit_current=session_valid&&in_local[commit_slot]==local_session&&in_remote[commit_slot]==remote_session;
 // A cancel is consuming an existing lease, not publishing its data. Exact old
 // leases can retire even while their session is no longer current.
 assign commit_ready=rst_n&&commit_meta&&(commit_cancel||commit_current);
 reg tx_busy,tx_index;
 assign job_valid=rst_n&&session_valid&&!tx_busy&&result_ready_valid;
 assign job_base=ready_result?32'h00600000:32'h005c0000;
 assign job_fid=out_fid[ready_result];assign job_epoch=out_epoch[ready_result];
 assign job_style=out_style[ready_result];assign job_layout=out_layout[ready_result];
 assign job_fps=out_fps[ready_result];assign job_effect=out_effect[ready_result];
 assign result_profile_commit=job_valid&&job_ready;
 assign result_profile_epoch=out_profile_epoch[ready_result];
 assign safe_to_load=state==IDLE&&core_idle&&copy_idle&&bus_quiet&&compute_release&&style_reader_idle;
 reg[31:0]work_local,work_remote,work_epoch;reg[15:0]work_profile_epoch;
 reg[1:0]work_style,work_layout;reg work_fps;reg[7:0]work_effect;
 reg killed,tail_success_seen,style_wait,commit_bad_seen;reg[15:0]style_request_epoch;
 reg[5:0]wait_count;
 wire work_current=session_valid&&work_local==local_session&&work_remote==remote_session;
 wire completion_clean=tail_success_seen&&tail_pixels==OUT_PIXELS&&tail_responses==TAIL_RESPONSES&&
   tail_bus_clean&&!runtime_fault&&!core_failed&&work_profile_epoch==active_epoch&&active_style==work_style+1'b1;
 // One READY slot is examined per APP clock. A new END during a sweep
 // restarts it: otherwise a just-visited slot could commit earlier than a
 // not-yet-visited slot and violate FIFO. Commits after a held selection are
 // newer and do not preempt it. The original signed age comparison is kept.
 reg[2:0]scan_index,scan_best_index,ticket_index;
 reg scan_best_valid,ticket_valid;
 reg[31:0]scan_best_order,scan_local,scan_remote;
 reg[31:0]ticket_order,ticket_fid,ticket_epoch,ticket_local,ticket_remote;
 reg[1:0]ticket_style,ticket_layout;reg ticket_fps;reg[7:0]ticket_effect;
 wire scan_eligible=input_state[scan_index]==READY&&
   in_local[scan_index]==local_session&&in_remote[scan_index]==remote_session;
 wire scan_better=scan_eligible&&(!scan_best_valid||$signed(in_order[scan_index]-scan_best_order)<0);
 wire[2:0]scan_winner=scan_better?scan_index:scan_best_index;
 wire[2:0]ready_index=ticket_index;
 // Revalidate the selected lease, including its sequence stamp, on every
 // use. Selection owns no slot and cannot resurrect a reused lease.
 wire ready_valid=ticket_valid&&session_valid&&
   ticket_local==local_session&&ticket_remote==remote_session&&
   input_state[ticket_index]==READY&&in_order[ticket_index]==ticket_order&&
   in_fid[ticket_index]==ticket_fid&&in_epoch[ticket_index]==ticket_epoch&&
   in_local[ticket_index]==ticket_local&&in_remote[ticket_index]==ticket_remote&&
   in_style[ticket_index]==ticket_style&&in_layout[ticket_index]==ticket_layout&&
   in_fps[ticket_index]==ticket_fps&&in_effect[ticket_index]==ticket_effect;
 wire launch_now=state==IDLE&&ready_valid&&admission&&!fault_sticky&&!runtime_fault&&
   model_valid&&active_style==ticket_style+1'b1&&!block_jobs&&!load_busy&&
   core_idle&&copy_idle&&bus_quiet&&result_free_valid&&result_state[free_result]==R_FREE;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   scan_index<=0;scan_best_index<=0;scan_best_valid<=0;scan_best_order<=0;scan_local<=0;scan_remote<=0;
   ticket_valid<=0;ticket_index<=0;ticket_order<=0;ticket_fid<=0;ticket_epoch<=0;ticket_local<=0;ticket_remote<=0;
   ticket_style<=0;ticket_layout<=0;ticket_fps<=0;ticket_effect<=0;
  end else if(state!=IDLE||!session_valid||scan_local!=local_session||scan_remote!=remote_session||launch_now)begin
   scan_index<=0;scan_best_valid<=0;ticket_valid<=0;scan_local<=local_session;scan_remote<=remote_session;
  end else if(ticket_valid)begin
   if(!ready_valid)begin ticket_valid<=0;scan_index<=0;scan_best_valid<=0;end
  end else if(commit_valid&&commit_ready&&!commit_cancel)begin
   scan_index<=0;scan_best_valid<=0;
  end else begin
   if(scan_better)begin scan_best_valid<=1;scan_best_index<=scan_index;scan_best_order<=in_order[scan_index];end
   if(scan_index==7)begin
    scan_index<=0;scan_best_valid<=0;
    if(scan_best_valid||scan_better)begin
     ticket_valid<=1;ticket_index<=scan_winner;ticket_order<=in_order[scan_winner];
     ticket_fid<=in_fid[scan_winner];ticket_epoch<=in_epoch[scan_winner];
     ticket_local<=in_local[scan_winner];ticket_remote<=in_remote[scan_winner];
     ticket_style<=in_style[scan_winner];ticket_layout<=in_layout[scan_winner];
     ticket_fps<=in_fps[scan_winner];ticket_effect<=in_effect[scan_winner];
    end
   end else scan_index<=scan_index+1'b1;
  end
 end
 reg[31:0]allocated,committed,started,computed,returned,cancelled,protocol_errors,failed_jobs;
 assign status_words={failed_jobs,protocol_errors,cancelled,returned,computed,started,committed,allocated};
 assign status_flags={12'd0,result_owned,input_owned,active_style,model_valid,tx_busy,compute_release,killed,fault_sticky,state};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;next_order<=0;next_result_order<=0;tx_busy<=0;tx_index<=0;
   compute_release<=1;core_start<=0;copy_start<=0;copy_slot<=0;output_index<=0;compute_fid<=0;tensor_imported<=0;
   style_request<=0;style_request_id<=1;style_wait<=0;style_request_epoch<=0;wait_count<=0;killed<=0;tail_success_seen<=0;commit_bad_seen<=0;
   work_local<=0;work_remote<=0;work_epoch<=0;work_profile_epoch<=0;work_style<=0;work_layout<=0;work_fps<=0;work_effect<=0;
   allocated<=0;committed<=0;started<=0;computed<=0;returned<=0;cancelled<=0;protocol_errors<=0;failed_jobs<=0;fault_sticky<=0;
   for(i=0;i<8;i=i+1)begin
    input_state[i]<=FREE;in_fid[i]<=0;in_epoch[i]<=0;in_local[i]<=0;in_remote[i]<=0;in_crc[i]<=0;in_order[i]<=0;
    in_style[i]<=0;in_layout[i]<=0;in_fps[i]<=0;in_effect[i]<=0;
   end
   for(i=0;i<2;i=i+1)begin
    result_state[i]<=R_FREE;out_fid[i]<=0;out_epoch[i]<=0;out_local[i]<=0;out_remote[i]<=0;out_order[i]<=0;
    out_style[i]<=0;out_layout[i]<=0;out_fps[i]<=0;out_effect[i]<=0;out_profile_epoch[i]<=0;
   end
  end else begin
   core_start<=0;copy_start<=0;style_request<=0;
   if(!commit_valid)commit_bad_seen<=0;
   if(state!=IDLE&&!work_current)killed<=1;
   // READY data is already DDR-committed, with no active receiver writer. An
   // old RECEIVING lease remains owned until the receiver drains and aborts.
   for(i=0;i<8;i=i+1)if(input_state[i]==READY&&(!session_valid||in_local[i]!=local_session||in_remote[i]!=remote_session))input_state[i]<=FREE;
   for(i=0;i<2;i=i+1)if((result_state[i]==R_READY||result_state[i]==R_FAILED)&&
      (!session_valid||out_local[i]!=local_session||out_remote[i]!=remote_session))result_state[i]<=R_FREE;
   if(allocate_valid&&allocate_ready)begin
    if(allocate_ok)begin
     input_state[free_index]<=RECEIVING;in_fid[free_index]<=allocate_fid;in_epoch[free_index]<=allocate_epoch;
     in_style[free_index]<=allocate_style;in_layout[free_index]<=allocate_layout;in_fps[free_index]<=allocate_fps;in_effect[free_index]<=allocate_effect;
     in_local[free_index]<=local_session;in_remote[free_index]<=remote_session;in_crc[free_index]<=0;allocated<=allocated+1'b1;
    end
   end
   if(commit_valid)begin
    if(commit_ready)begin
     if(commit_cancel)begin input_state[commit_slot]<=FREE;cancelled<=cancelled+1'b1;end
     else begin input_state[commit_slot]<=READY;in_crc[commit_slot]<=commit_crc;in_order[commit_slot]<=next_order;next_order<=next_order+1'b1;committed<=committed+1'b1;end
    end else if(!commit_bad_seen)begin protocol_errors<=protocol_errors+1'b1;commit_bad_seen<=1;end
   end
   if(abort_valid)begin
    if(abort_address_ok&&in_fid[abort_slot]==abort_fid)begin
     if(input_state[abort_slot]==RECEIVING)begin input_state[abort_slot]<=FREE;cancelled<=cancelled+1'b1;end
     // commit_cancel is followed by an abort pulse in frame_rx. A matching
     // already-free tombstone is an idempotent cancellation, not a new owner.
     else if(input_state[abort_slot]!=FREE)protocol_errors<=protocol_errors+1'b1;
    end else protocol_errors<=protocol_errors+1'b1;
   end
   if(job_valid&&job_ready)begin result_state[ready_result]<=R_TX;tx_busy<=1;tx_index<=ready_result;end
   if(tx_done)begin
    if(tx_busy&&result_state[tx_index]==R_TX&&out_fid[tx_index]==tx_done_fid)begin
     tx_busy<=0;
     if(tx_done_ok)begin result_state[tx_index]<=R_FREE;returned<=returned+1'b1;end
     else if(!session_valid||out_local[tx_index]!=local_session||out_remote[tx_index]!=remote_session)begin result_state[tx_index]<=R_FREE;cancelled<=cancelled+1'b1;end
     // Failed transport has retired its AXI reader. Keep this same immutable
     // lease and offer it again; endpoint must gate job_ready during restart.
     // No numerical core/fabric reset or permanent compute fault is implied.
     else begin result_state[tx_index]<=R_READY;failed_jobs<=failed_jobs+1'b1;end
    end else protocol_errors<=protocol_errors+1'b1;
   end
   if(runtime_fault)fault_sticky<=1;
   case(state)
    IDLE:begin
     if(style_wait&&!load_busy&&model_valid&&(active_epoch!=style_request_epoch||
       (ready_valid&&active_style==ticket_style+1'b1)))style_wait<=0;
     if(ready_valid&&admission&&!fault_sticky&&!runtime_fault)begin
      if((!model_valid||active_style!=ticket_style+1'b1)&&!style_wait&&!load_busy&&safe_to_load)begin
       style_request<=1;style_request_id<=ticket_style+1'b1;style_wait<=1;style_request_epoch<=active_epoch;
      end
      if(launch_now)begin
       style_wait<=0;input_state[ready_index]<=COPYING;result_state[free_result]<=R_COMPUTE;
       copy_slot<=ready_index;output_index<=free_result;compute_fid<=ticket_fid;
       // Reserve output identity immediately, including the copy/compute phase,
       // so a duplicate FID cannot bypass the now reusable input source slot.
       out_fid[free_result]<=ticket_fid;out_epoch[free_result]<=ticket_epoch;
       out_local[free_result]<=ticket_local;out_remote[free_result]<=ticket_remote;
       work_local<=ticket_local;work_remote<=ticket_remote;work_epoch<=ticket_epoch;
       work_style<=ticket_style;work_layout<=ticket_layout;work_fps<=ticket_fps;work_effect<=ticket_effect;work_profile_epoch<=active_epoch;
       tensor_imported<=0;tail_success_seen<=0;killed<=0;compute_release<=0;wait_count<=0;state<=ASSERT_CORE;
      end
     end
    end
    ASSERT_CORE:if(runtime_fault)begin failed_jobs<=failed_jobs+1'b1;state<=DRAIN;end
     else if(wait_count==3)begin copy_start<=1;wait_count<=0;state<=COPY;end else wait_count<=wait_count+1'b1;
    COPY:if(copy_done)begin
     input_state[copy_slot]<=FREE;
     if(copy_failed||runtime_fault)begin fault_sticky<=1;failed_jobs<=failed_jobs+1'b1;state<=DRAIN;end
     else begin tensor_imported<=1;compute_release<=1;wait_count<=0;state<=WARM;end
    end
    WARM:if(wait_count==31&&core_idle&&bus_quiet)begin
     if(runtime_fault)begin failed_jobs<=failed_jobs+1'b1;state<=DRAIN;end
     else if(killed||!work_current)begin result_state[output_index]<=R_FREE;cancelled<=cancelled+1'b1;state<=IDLE;end
     else begin core_start<=1;started<=started+1'b1;state<=RUN;end
    end else if(wait_count!=31)wait_count<=wait_count+1'b1;
    RUN:begin
     if(core_stage==4&&core_transport_retired&&core_stage_success)tail_success_seen<=1;
     if(core_failed||runtime_fault)begin fault_sticky<=1;failed_jobs<=failed_jobs+1'b1;state<=DRAIN;end
     else if(core_done)state<=FINISH;
    end
    FINISH:if(bus_quiet&&core_idle&&copy_idle)begin
     if(!completion_clean)begin fault_sticky<=1;failed_jobs<=failed_jobs+1'b1;state<=DRAIN;end
     else if(killed||!work_current)begin result_state[output_index]<=R_FREE;cancelled<=cancelled+1'b1;state<=IDLE;end
     else begin
      result_state[output_index]<=R_READY;out_fid[output_index]<=compute_fid;out_epoch[output_index]<=work_epoch;
      out_style[output_index]<=work_style;out_layout[output_index]<=work_layout;out_fps[output_index]<=work_fps;out_effect[output_index]<=work_effect;
      out_local[output_index]<=work_local;out_remote[output_index]<=work_remote;out_profile_epoch[output_index]<=work_profile_epoch;
      out_order[output_index]<=next_result_order;next_result_order<=next_result_order+1'b1;computed<=computed+1'b1;state<=IDLE;
     end
    end
    DRAIN:if(bus_quiet&&copy_idle&&core_idle)begin
     if(input_state[copy_slot]==COPYING)input_state[copy_slot]<=FREE;
     result_state[output_index]<=R_FREE;state<=FAULT;
    end
    FAULT:begin end
    default:begin state<=FAULT;fault_sticky<=1;end
   endcase
  end
 end
endmodule

`default_nettype wire
