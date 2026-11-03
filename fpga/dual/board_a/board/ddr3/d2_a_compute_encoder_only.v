`timescale 1ns/1ps
`include "d64_blk_beats.vh"
module d2_a_compute_encoder_only(
 input wire clk_pe,clk_weight_fast,rst_pe_n,ddr_ready,
 input wire [1:0] requested_style,input wire start,input wire [31:0] tensor_base,
 output wire ready,output wire model_loaded,output wire [1:0] active_profile,output reg input_start,output reg done,output wire failed,output wire [127:0] diag_compute,
 input wire d64_valid,output wire d64_ready,input wire [383:0] d64_rgb888,
 input wire [31:0] d64_frame_id,input wire d64_sof,d64_eol,d64_eof,input wire [7:0] d64_x0,d64_y,
 output wire [239:0] axi_cmd,input wire [137:0] axi_rsp,
 input wire [3:0] i_spi0_io,output wire [3:0] o_spi0_io,oe_spi0_io,
 output wire o_spi_clk,oe_spi_clk,o_spi0_cs,oe_spi0_cs
);
 localparam integer ROWS=36,BLK_BEATS=`D64_ROWS_TO_BEATS(36),SAB_SHAPE_STALE=0;
 localparam WT_INIT_FILE="";
 localparam [31:0] H_BASE=32'h00100000,D0_BASE=32'h00180000,TRUNK_A_BASE=32'h001c0000;
 wire fabric_arst_n;
 wire [239:0] m_cmd[0:5];wire [137:0] m_rsp[0:5];
 wire [239:0] profile_cmd,flash_cmd;wire [137:0] profile_rsp,flash_rsp;
 wire [239:0] join_cmd[0:6];wire [137:0] join_rsp[0:6];
 genvar gi;generate for(gi=0;gi<7;gi=gi+1)begin:g_join
 wire [239:0] left=gi==0?m_cmd[0]:join_cmd[gi-1];wire [137:0] lr,rr;
 wire [239:0] right=gi<5?m_cmd[gi+1]:gi==5?profile_cmd:flash_cmd;
 if(gi==0)assign m_rsp[0]=lr;else assign join_rsp[gi-1]=lr;
 if(gi<5)assign m_rsp[gi+1]=rr;else if(gi==5)assign profile_rsp=rr;else assign flash_rsp=rr;
 d64_axi_join u_join(.clk(clk_pe),.rst_n(rst_pe_n),.m0_cmd(left),.m0_rsp(lr),.m1_cmd(right),.m1_rsp(rr),.s_cmd(join_cmd[gi]),.s_rsp(join_rsp[gi]));
 end endgenerate
 assign axi_cmd=join_cmd[6];assign join_rsp[6]=axi_rsp;
 wire [31:0] tail_address;
 assign m_cmd[3][32:1]=tail_address;
 wire [3:0] asset_ready;wire flash_busy,flash_done,flash_failed;reg flash_started;
 wire load_start,load_done,load_failed,load_invalidate,load_busy;
 wire [1:0] load_style;wire load_request,load_stream_valid,load_stream_ready,load_stream_error,style_reader_idle;
 wire [31:0] load_address;wire [15:0] load_beats;wire [127:0] load_data;
 wire cfg_we;wire [8:0] cfg_index;wire [63:0] cfg_data;
 wire dec_we=cfg_we&&cfg_index>=376;wire [4:0] dec_wa=cfg_index-376;
 wire [9:0] profile_zero;wire core_idle;
 reg loaded;reg [1:0] active_style,load_style_q;reg load_pulse;reg loading;reg failed_hold;
 wire model_valid=loaded;wire ddr_cal_done=ddr_ready,ddr_cal_pass=ddr_ready;
 assign load_style=load_style_q;assign load_start=load_pulse;
 reg compute_release,job_start;reg [2:0] js;reg [5:0] wait_count;
 assign model_loaded=loaded&&!failed_hold;assign active_profile=active_style;
 assign ready=loaded&&active_style==requested_style&&!loading&&js==0&&core_idle&&!failed_hold;
    localparam integer CAWVALID  = 0;
    localparam integer CAWADDR   = 1;
    localparam integer CAWLEN    = 33;
    localparam integer CAWSIZE   = 41;
    localparam integer CAWBURST  = 44;
    localparam integer CWVVALID  = 46;
    localparam integer CWVDATA   = 47;
    localparam integer CWVSTRB   = 175;
    localparam integer CWVLAST   = 191;
    localparam integer CBREDY    = 192;
    localparam integer CARVALID  = 193;
    localparam integer CARADDR   = 194;
    localparam integer CARLEN    = 226;
    localparam integer CARSIZE   = 234;
    localparam integer CARBURST  = 237;
    localparam integer CRREDY    = 239;
    localparam integer CMD_W     = 240;

    localparam integer RAWREADY  = 0;
    localparam integer RWREDY    = 1;
    localparam integer RBVALID   = 2;
    localparam integer RBRESPO   = 3;
    localparam integer RARREADY  = 5;
    localparam integer RRVALID   = 6;
    localparam integer RRDATA    = 7;
    localparam integer RRLAST    = 135;
    localparam integer RBRESPR   = 136;
    localparam integer RSP_W     = 138;

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


    wire         raw_sof_w, raw_sol_w, raw_eol_w, raw_eof_w;

    wire fabric_arst_n_w = rst_pe_n;
    assign fabric_arst_n = fabric_arst_n_w;

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
    always @(posedge clk_pe or negedge fabric_arst_n) begin
        if (!fabric_arst_n) fence_cnt <= 4'd0;
        else if (trunk_committed && !(&fence_cnt)) fence_cnt <= fence_cnt + 4'd1;
    end
    wire trunk_committed_fenced = trunk_committed && fence_ok;

    // Active weights initialize in place; all runtime profiles come from DDR.
    wire wld_done=model_valid;
    wire ldr_run=!model_valid;
    wire wld_we,ldr_we;wire[8:0] wld_wa;wire[511:0] wld_wd;
    wire[8:0] ldr_wa;wire[63:0] ldr_dout;
    assign stage_en=4'b1111;
    assign core_start=job_start;





    d64_network_core #(
        .IMG_H(144), .IMG_W(256), .ENABLE_ENCODER(1), .ENCODER_ONLY(1), .ENABLE_DECODER(0), .TRUNK_FIRST_BLOCK(0), .TRUNK_BLOCK_COUNT(0), .SAB_SHAPE_STALE(SAB_SHAPE_STALE),
        .WT_INIT_FILE(WT_INIT_FILE), .RUNTIME_PROFILE(1), .FAULT_STICKY(1),
        .TRUNK_CFG_BIAS_W(20), .DEC_CFG_BIAS_W(13), .CFG_STREAM_SRL(1),
        .HEAD_STORE_CREDIT_ENABLE(1),
        .TAIL_RESERVE_WRITE_BURST(1), .BQ_DEPTH(256)
    ) u_core (

        .clk_pe(clk_pe), .clk_axi(clk_pe), .arst_n(rst_pe_n & compute_release),
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

    assign m_cmd[0][CAWVALID] = 1'b0;
    assign m_cmd[0][191:1] = 191'd0;
    assign m_cmd[0][CBREDY]   = 1'b1;
    assign m_cmd[1][238:193] = 46'd0;
    assign m_cmd[1][CRREDY]   = 1'b1;
    assign m_cmd[2][238:193] = 46'd0;
    assign m_cmd[2][CRREDY]   = 1'b1;
    assign m_cmd[3][238:193] = 46'd0;
    assign m_cmd[3][CRREDY]   = 1'b1;

    d64_encoder_ddr_mem #(.FAULT_STICKY(1),.RESERVE_WRITE_BURST(1),
        .H_BASE(H_BASE), .D0_BASE(D0_BASE), .TRUNK_A_BASE(TRUNK_A_BASE),
        .ROWS(ROWS)
    ) u_mem (
        .clk_pe(clk_pe), .clk_axi(clk_pe), .arst_n(fabric_arst_n & compute_release),

        .pe_done(stage_finished),
        .head_store_valid(hs_valid), .head_store_ready(hs_ready),
        .head_phase_ready(head_phase_ready),
        .head_store_x(hs_x), .head_store_y(hs_y), .head_store_c8(hs_c8),
        .head_store_frame_id(hs_fid), .head_store_sof(hs_sof),
        .head_store_eol(hs_eol), .head_store_eof(hs_eof),
        .head_commit_ready(hs_commit),
        .head_read_start(hr_start), .head_read_valid(hr_valid),
        .head_read_ready(hr_ready), .head_read_x(hr_x), .head_read_y(hr_y),
        .head_read_c8(hr_c8), .head_read_frame_id(hr_fid),
        .head_read_sof(hr_sof), .head_read_eol(hr_eol), .head_read_eof(hr_eof),
        .d0_store_valid(d0s_valid), .d0_store_ready(d0s_ready),
        .d0_store_x(d0s_x), .d0_store_y(d0s_y), .d0_store_word(d0s_word),
        .d0_store_data(d0s_data), .d0_store_frame_id(d0s_fid),
        .d0_store_sof(d0s_sof), .d0_store_eol(d0s_eol), .d0_store_eof(d0s_eof),
        .d0_commit_ready(d0s_commit),
        .d0_read_start(d0r_start), .d0_read_valid(d0r_valid),
        .d0_read_ready(d0r_ready), .d0_read_x(d0r_x), .d0_read_y(d0r_y),
        .d0_read_icg(d0r_icg), .d0_read_c8(d0r_c8),
        .d0_read_frame_id(d0r_fid), .d0_read_sof(d0r_sof),
        .d0_read_eol(d0r_eol), .d0_read_eof(d0r_eof),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_x(out_x), .out_y(out_y), .out_word(out_word),
        .out_data(out_data), .out_frame_id(out_frame_id),
        .out_sof(out_sof), .out_eol(out_eol), .out_eof(out_eof),
        .axi_awvalid(m_cmd[4][CAWVALID]), .axi_awready(m_rsp[4][RAWREADY]),
        .axi_awaddr(m_cmd[4][CAWADDR +: 32]),
        .axi_awlen(m_cmd[4][CAWLEN +: 8]),
        .axi_awsize(m_cmd[4][CAWSIZE +: 3]),
        .axi_awburst(m_cmd[4][CAWBURST +: 2]),
        .axi_wvalid(m_cmd[4][CWVVALID]), .axi_wready(m_rsp[4][RWREDY]),
        .axi_wdata(m_cmd[4][CWVDATA +: 128]),
        .axi_wstrb(m_cmd[4][CWVSTRB +: 16]),
        .axi_wlast(m_cmd[4][CWVLAST]),
        .axi_bvalid(m_rsp[4][RBVALID]), .axi_bready(m_cmd[4][CBREDY]),
        .axi_bresp(m_rsp[4][RBRESPO +: 2]),
        .axi_arvalid(m_cmd[4][CARVALID]), .axi_arready(m_rsp[4][RARREADY]),
        .axi_araddr(m_cmd[4][CARADDR +: 32]),
        .axi_arlen(m_cmd[4][CARLEN +: 8]),
        .axi_arsize(m_cmd[4][CARSIZE +: 3]),
        .axi_arburst(m_cmd[4][CARBURST +: 2]),
        .axi_rvalid(m_rsp[4][RRVALID]), .axi_rready(m_cmd[4][CRREDY]),
        .axi_rdata(m_rsp[4][RRDATA +: 128]),
        .axi_rlast(m_rsp[4][RRLAST]),
        .axi_rresp(m_rsp[4][RBRESPR +: 2]),
        .trunk_done(trunk_committed), .trunk_commit_axi(trunk_commit_axi),
        .trunk_store_commit_wait_cycles(trunk_store_commit_wait_cycles),
        .h_write_beats(h_write_beats), .h_read_beats(h_read_beats),
        .d0_write_beats(d0_write_beats), .d0_read_beats(d0_read_beats),
        .trunk_write_beats(trunk_write_beats),
        .h_write_bursts(h_write_bursts), .h_read_bursts(h_read_bursts),
        .d0_write_bursts(d0_write_bursts), .d0_read_bursts(d0_read_bursts),
        .trunk_write_bursts(trunk_write_bursts),
        .axi_response_errors(mem_axi_response_errors),
        .stream_pack_errors(stream_pack_errors),
        .write_fifo_max_occupancy(write_fifo_max_occupancy),
        .read_fifo_max_occupancy(read_fifo_max_occupancy),
        .read_fifo_overflow(read_fifo_overflow),
        .rready_drop_in_burst(rready_drop_in_burst)
    );

    wire [7:0]  row_bnd_seen, row_bnd_busy;
    wire [31:0] row_rd_base_chg, row_rf_ovf;
    wire [2:0]  row_axi_blk;
    wire [3:0]  row_par_ones;
    wire        row_par_ok, row_sel_mismatch;
    wire [31:0] row_rd_beats, row_wr_beats, row_rd_bursts, row_wr_bursts;
    wire [31:0] row_axi_errors;

    // Encoder completion already includes every source DDR write response.
    // This real copy owns former row master5; completion waits its final BRESP.
    reg feature_copy_start;
    wire feature_copy_idle,feature_copy_done,feature_copy_failed,feature_copy_fault;
    wire[31:0]feature_copy_words;
    d2_a_feature_copy u_feature_copy(.clk(clk_pe),.rst_n(rst_pe_n),.start(feature_copy_start),.dest_base(tensor_base),
      .idle(feature_copy_idle),.done(feature_copy_done),.failed(feature_copy_failed),.fault_sticky(feature_copy_fault),
      .copied_words(feature_copy_words),.axi_cmd(m_cmd[5]),.axi_rsp(m_rsp[5]));
    assign x_ack_tog=0;assign rf_wr_en=0;assign rf_wr_data=0;assign wf_rd_en=0;assign wr_done_tog=0;
    assign row_rd_beats=feature_copy_words;assign row_wr_beats=feature_copy_words;
    assign row_rd_bursts=0;assign row_wr_bursts=0;assign row_axi_errors={31'd0,feature_copy_fault};

    d64_style_loader u_style_loader(.clk(clk_pe),.rst_n(rst_pe_n),.start(load_start),.style(load_style),
        .request(load_request),.profile_addr(load_address),.profile_beats(load_beats),
        .stream_valid(load_stream_valid),.stream_ready(load_stream_ready),.stream_data(load_data),.stream_error(load_stream_error),
        .wt_we(wld_we),.wt_addr(wld_wa),.wt_data(wld_wd),.cfg_we(cfg_we),.cfg_index(cfg_index),.cfg_data(cfg_data),
        .profile_trunk_zero(profile_zero),.busy(load_busy),.done(load_done),.failed(load_failed),.invalidate(load_invalidate),.error_code());
    d64_style_ddr_reader #(.MAX_BURST_BEATS(1),.ABORT_EN(1),.RESERVE_AR_READY(1)) u_profile_reader(.clk(clk_pe),.rst_n(fabric_arst_n),.abort(load_failed),
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
    d64_flash_asset_boot u_flash(.clk(clk_pe),.rst_n(rst_pe_n),
        .start(!flash_started&&ddr_cal_done&&ddr_cal_pass),.spi_clk(o_spi_clk),.spi_cs_n(o_spi0_cs),
        .spi_mosi(o_spi0_io[0]),.spi_miso(i_spi0_io[1]),
        .asset_ready(asset_ready),.busy(flash_busy),.done(flash_done),.failed(flash_failed),
        .awaddr(flash_cmd[CAWADDR+:32]),.awlen(flash_cmd[CAWLEN+:8]),.awsize(flash_cmd[CAWSIZE+:3]),
        .awburst(flash_cmd[CAWBURST+:2]),.awvalid(flash_cmd[CAWVALID]),.awready(flash_rsp[RAWREADY]),
        .wdata(flash_cmd[CWVDATA+:128]),.wstrb(flash_cmd[CWVSTRB+:16]),.wlast(flash_cmd[CWVLAST]),
        .wvalid(flash_cmd[CWVVALID]),.wready(flash_rsp[RWREDY]),.bvalid(flash_rsp[RBVALID]),
        .bready(flash_cmd[CBREDY]),.bresp(flash_rsp[RBRESPO+:2]));
    assign flash_cmd[CARVALID]=0;assign flash_cmd[CRREDY]=1;
    assign flash_cmd[CARADDR+:32]=0;assign flash_cmd[CARLEN+:8]=0;
    assign flash_cmd[CARSIZE+:3]=4;assign flash_cmd[CARBURST+:2]=1;
    assign o_spi0_io[3:1]=3'b110;assign oe_spi0_io=4'b1101;assign oe_spi_clk=1;assign oe_spi0_cs=1;
 reg [31:0] diag_compute_flags;reg [5:0] diag_compute_causes;
 always @(posedge clk_pe or negedge rst_pe_n)
 if(!rst_pe_n)diag_compute_causes<=0;
 else diag_compute_causes<=diag_compute_causes|{row_axi_errors!=0,stream_pack_errors!=0,mem_axi_response_errors!=0,core_failed,load_failed,flash_failed};
 always @*begin
 diag_compute_flags=0;
 diag_compute_flags[0]=failed_hold;diag_compute_flags[6:1]=diag_compute_causes;
 diag_compute_flags[7]=loaded;diag_compute_flags[8]=loading;diag_compute_flags[9]=flash_started;
 diag_compute_flags[10]=flash_busy;diag_compute_flags[11]=flash_done;diag_compute_flags[15:12]=asset_ready;
 diag_compute_flags[17:16]=active_style;diag_compute_flags[19:18]=requested_style;
 diag_compute_flags[22:20]=js;diag_compute_flags[23]=core_idle;diag_compute_flags[24]=core_busy;
 diag_compute_flags[25]=compute_release;diag_compute_flags[26]=load_busy;diag_compute_flags[27]=load_invalidate;
 end
 assign diag_compute={row_axi_errors,stream_pack_errors,mem_axi_response_errors,diag_compute_flags};
 assign failed=failed_hold;
 always @(posedge clk_pe or negedge rst_pe_n)begin
 if(!rst_pe_n)begin flash_started<=0;loaded<=0;active_style<=1;load_style_q<=1;failed_hold<=0;load_pulse<=0;loading<=0;compute_release<=1;job_start<=0;input_start<=0;done<=0;feature_copy_start<=0;js<=0;wait_count<=0;end
 else begin
 load_pulse<=0;job_start<=0;input_start<=0;done<=0;feature_copy_start<=0;
 if(ddr_ready)flash_started<=1;
 if(flash_failed||load_failed||core_failed||mem_axi_response_errors!=0||stream_pack_errors!=0||row_axi_errors!=0)failed_hold<=1;
 if(js==0&&!loading&&asset_ready[requested_style]&&(!loaded||active_style!=requested_style)&&core_idle&&!failed_hold)begin loading<=1;loaded<=0;load_style_q<=requested_style;load_pulse<=1;end
 if(loading&&load_done)begin active_style<=load_style_q;loaded<=1;loading<=0;end
 if(load_failed)begin loading<=0;loaded<=0;end
 case(js)
 0:if(start&&ready)begin js<=1;compute_release<=0;wait_count<=0;end
 1:if(wait_count==3)begin compute_release<=1;wait_count<=0;js<=2;end else wait_count<=wait_count+1;
 2:if(wait_count==31&&core_idle)begin job_start<=1;input_start<=1;js<=3;end else if(wait_count!=31)wait_count<=wait_count+1;
 3:if(core_done)begin
   if(core_failed||failed_hold||mem_axi_response_errors!=0||stream_pack_errors!=0)begin failed_hold<=1;js<=0;end
   else begin feature_copy_start<=1;js<=4;end
 end
 4:if(feature_copy_done)begin
   if(feature_copy_failed||feature_copy_fault||failed_hold||core_failed||mem_axi_response_errors!=0||stream_pack_errors!=0)failed_hold<=1;
   else done<=1;
   js<=0;
 end
 default:js<=0;
 endcase
 end end

 assign ldr_we=cfg_we&&cfg_index>=56&&cfg_index<376;
 assign ldr_wa=cfg_index-56;assign ldr_dout=cfg_data;
endmodule
