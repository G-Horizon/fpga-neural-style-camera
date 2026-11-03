`timescale 1ns/1ps
// Maintained camera/interaction integration; historical generator is guarded.
`include "rtl/ddr3_controller/ddr3_parameter.vh"

`include "d64_blk_beats.vh"

module d64_final_system #(

    parameter integer ROWS = 36,

    parameter [3:0] STAGE_EN_INIT = 4'b1111,
    parameter integer CLK_HZ = 25_000_000,
    parameter         WT_INIT_FILE = "../../profiles/vangogh/weights.mem",

    parameter integer CAM_W = 1920,
    parameter integer CAM_H = 1080,
    parameter [1:0]   CAM_CFA = 2'd0,
    parameter [15:0] CAM_WB_R_Q8 = 16'd256,
    parameter [15:0] CAM_WB_G_Q8 = 16'd256,
    parameter [15:0] CAM_WB_B_Q8 = 16'd256,
    parameter integer CAM_GAMMA_EN = 0,
    parameter integer CAM_PACKED = 0,
    parameter integer GUIDE_ENABLE = 0, COMPACT_UART = 0, STATUS_UART_ENABLE=1,
    parameter integer VOICE_ENABLE=1,GESTURE_ENABLE=1,COMPARISON_ENABLE=1,PREVIEW_ENABLE=1,REMOTE_DEBUG_ENABLE=0,
    parameter integer DISPLAY_SCALE = 1,
    parameter integer DISPLAY_BILINEAR = 0, DISPLAY_DST_W = 0, DISPLAY_DST_H = 0,
    parameter integer FPS_OSD_ENABLE = 1, FPS_DEFAULT_ON = 1,

    parameter integer OUT_W = 256,
    parameter integer OUT_H = 144,

    parameter integer SAB_SHAPE_STALE = 0,

    parameter integer H_ACT = 1280, H_FP = 1760, H_SYNC = 40, H_BP = 220,
    parameter integer V_ACT = 720,  V_FP = 5,    V_SYNC = 5,  V_BP = 20
) (

    input  wire        clk_25m,
    input wire clk_weight_fast,
    input  wire        pll_lock,
    input  wire        pll1_lock,
    input  wire        pe_pll_lock,
    output wire        ddr_pll_rstn,
    output wire        sys_pll_rstn,
    output wire        pe_pll_rstn,
    input  wire        clk_pe,
    input  wire        hdmi_tx_slow_clk,
    input  wire        hdmi_tx_fast_clk,
    input  wire        clk,
    input  wire        ddr_core_clk,
    input  wire        ddr_twd_clk,
    input  wire        ddr_tac_clk,
    input  wire        ddr_tdqss_clk,
    output wire        ddr_reset,
    output wire        ddr_cs,
    output wire        ddr_ras,
    output wire        ddr_cas,
    output wire        ddr_we,
    output wire        ddr_cke,
    output wire [15:0] ddr_addr,
    output wire [2:0]  ddr_ba,
    output wire        ddr_odt,
    output wire [1:0]  o_ddr_dm_hi,
    output wire [1:0]  o_ddr_dm_lo,
    input  wire [1:0]  i_ddr_dqs_hi,
    input  wire [1:0]  i_ddr_dqs_lo,
    input  wire [1:0]  i_ddr_dqs_n_hi,
    input  wire [1:0]  i_ddr_dqs_n_lo,
    output wire [1:0]  o_ddr_dqs_hi,
    output wire [1:0]  o_ddr_dqs_lo,
    output wire [1:0]  o_ddr_dqs_n_hi,
    output wire [1:0]  o_ddr_dqs_n_lo,
    output wire [1:0]  o_ddr_dqs_oe,
    output wire [1:0]  o_ddr_dqs_n_oe,
    input  wire [15:0] i_ddr_dq_hi,
    input  wire [15:0] i_ddr_dq_lo,
    output wire [15:0] o_ddr_dq_hi,
    output wire [15:0] o_ddr_dq_lo,
    output wire [15:0] o_ddr_dq_oe,
    output wire [2:0]  ddr_shift,
    output wire [4:0]  ddr_shift_sel,
    output wire        ddr_shift_ena,
    input  wire        verf0,
    input  wire        verf1,

    input  wire        cam_raw_valid,
    input  wire [9:0]  cam_raw10,
    input  wire        cam_sof,
    input  wire        cam_sol,
    input  wire        cam_eol,
    input  wire        cam_eof,
    input  wire [31:0] cam_frame_id,
    input  wire        io_cam_scl_IN,
    input  wire        io_cam_sda_IN,
    output wire        io_cam_scl_OUT,
    output wire        io_cam_scl_OE,
    output wire        io_cam_sda_OUT,
    output wire        io_cam_sda_OE,
    output wire        o_cam_rst_p,

    output wire [9:0]  tmds_data0_o,
    output wire [9:0]  tmds_data1_o,
    output wire [9:0]  tmds_data2_o,
    output wire [9:0]  tmds_clk_o,
    output wire        tmds_data0_TX_OE,
    output wire        tmds_data1_TX_OE,
    output wire        tmds_data2_TX_OE,
    output wire        tmds_clk_TX_OE,
    output wire        tmds_data0_TX_RST,
    output wire        tmds_data1_TX_RST,
    output wire        tmds_data2_TX_RST,

    output wire        tmds_clk_TX_RST,

    output wire [3:0]  led,
    output wire        txd,
    output wire        HPD_N,
    input wire [3:0] keys_n,
    input wire spi_preview_sclk,spi_preview_cs_n,spi_preview_mosi,
    output wire spi_preview_miso,spi_preview_ready,
    input wire [31:0] camera_status,camera_readback,
    input wire cam_packed_clk,cam_packed_rst_n,cam_packed_valid,
    input wire [39:0] cam_packed_raw4,
    input wire cam_packed_sof,cam_packed_sol,cam_packed_eol,cam_packed_eof,
    input wire [31:0] cam_packed_fid,
    input wire cam_packed_source_bad,
    output wire camera_accepting,
    input wire [3:0] i_spi0_io,
    output wire [3:0] o_spi0_io, oe_spi0_io,
    output wire o_spi_clk, oe_spi_clk, o_spi0_cs, oe_spi0_cs
);

    wire cap_valid,cap_ready,cap_sof,cap_eol,cap_eof;
    wire[383:0] cap_rgb888;wire[31:0] cap_frame_id;wire[7:0] cap_x0,cap_y;
    wire camera_available,camera_read_done,camera_read_failed;
    wire camera_ready_valid;wire [31:0] camera_ready_fid;
    wire[31:0] captured_frames,dropped_frames,capture_axi_errors,capture_protocol_errors;
    reg camera_read_start;
    wire core_idle;
    wire t_req_buf,t_src_buf,t_dst_buf;
    wire[31:0] tail_address;
    wire[1:0] active_style;wire[15:0] active_epoch;
    wire model_valid,block_jobs,profile_publish;
    wire[9:0] profile_zero;
    wire[3:0] asset_ready;
    wire flash_busy,flash_done,flash_failed;
    reg flash_started;
    wire user_request;wire[1:0] user_style;wire fps_toggle;
    wire rst_pe_n;
    wire [1:0] guide_strength=2'd2;
    wire compare_toggle,effect_trigger;
    wire voice_request_valid,voice_request_next;
    wire [1:0] voice_request_style;
    wire int_compare_enabled,int_effect_enabled;
    wire [1:0] int_display_mode;
    wire int_original_query_valid;
    wire remote_fps_set_valid,remote_fps_set_value,fps_requested_pe;
    wire [5:0] remote_actual_state_pe;wire [1:0] ui_target_style;wire ui_target_pending;
    wire [15:0] remote_status={remote_actual_state_pe[5:4],model_valid,remote_actual_state_pe[0],
      int_effect_enabled,remote_actual_state_pe[1],fps_requested_pe,remote_actual_state_pe[3:2],
      int_display_mode,ui_target_pending,ui_target_style,active_style};
    wire [239:0] int_extra_cmd,int_comparison_cmd,guide_profile_cmd;
    wire [137:0] int_extra_rsp,int_comparison_rsp,guide_profile_rsp;
    wire [4:0] int_protected_slots,int_comparison_protect;
    wire int_video_fault,int_preview_fault,int_voice_fault;
    wire [31:0] int_comparison_faults;
    wire [7:0] int_processing_fps;
    wire int_comp_publish,int_comp_ready;
    wire [31:0] int_comp_original_base,int_comp_half_base,int_comp_fid;
    wire [15:0] int_comp_epoch;wire [2:0] int_comp_slot;
    wire int_render_de,int_render_good,int_render_slot_valid;
    wire [23:0] int_render_rgb;wire [11:0] int_render_ax,int_render_ay;
    wire [31:0] int_render_fid;wire [15:0] int_render_epoch;wire [2:0] int_render_slot;
    wire int_grant_valid,int_guide_allocate,int_guide_commit,int_guide_failed;
    wire [2:0] int_grant_slot,int_guide_commit_slot;wire [31:0] int_guide_commit_fid;
    wire int_original_valid;wire [15:0] int_original_rgb565;
    wire [9:0] int_original_x,int_original_y;wire [31:0] int_original_fid;
    wire guide_scalar_valid,guide_scalar_fault;
    wire [7:0] guide_scalar_gray;
    wire [10:0] guide_scalar_x,guide_scalar_y;
    wire [31:0] guide_scalar_fid;
    wire [1:0] guide_result_valid;wire [2:0] guide_result_slot0,guide_result_slot1;
    wire [31:0] guide_display_faults;
    wire guide_reader_axi_fault,guide_reader_protocol_fault,guide_reader_outstanding;
    wire [2:0] guide_reader_slot;
    wire guide_lease_fault,guide_capture_fault;
    wire source_guide_allocated,camera_guide_ready,job_guide_slot_valid,job_guide_ready;
    wire [31:0] job_guide_fid;
    wire job_guided_ok;
    (* async_reg="true" *) reg [1:0] guide_display_fault_sync;
    always @(posedge clk_pe or negedge rst_pe_n)
        if(!rst_pe_n)guide_display_fault_sync<=0;
        else guide_display_fault_sync<={guide_display_fault_sync[0],guide_display_faults!=0};
    wire guide_any_fault=guide_lease_fault||guide_capture_fault||guide_scalar_fault||
        guide_display_fault_sync[1]||guide_reader_axi_fault||guide_reader_protocol_fault;
    reg boot_requested;
    wire boot_request=((asset_ready & 4'b1110)==4'b1110)&&!boot_requested;
    wire load_start,load_done,load_failed,load_invalidate,load_busy;
    wire[1:0] load_style;
    wire load_request,load_stream_valid,load_stream_ready,load_stream_error,style_reader_idle;
    wire[31:0] load_address;wire[15:0] load_beats;
    wire[127:0] load_data;
    wire cfg_we;wire[8:0] cfg_index;wire[63:0] cfg_data;
    wire dec_we=cfg_we && cfg_index>=376;
    wire[4:0] dec_wa=cfg_index-376;
    wire switch_done,switch_fault,deadline_missed;wire[31:0] switch_cycles;
    wire boot_wait_source,boot_source_arm,boot_active;wire [31:0] boot_total_cycles;
    wire publish_ready,display_frame,display_valid;
    wire display_scan;wire[31:0] display_scan_fid;
    wire comparison_publication_window;
    wire[31:0] display_base,display_fid;
    reg publish_valid;reg[31:0] publish_base,publish_fid;
    localparam J_IDLE=0,J_ASSERT=1,J_WARM=2,J_RUN=3,J_PUBLISH=4,J_FAULT=5;
    reg[2:0] job_state;
    reg compute_release,job_start,job_slot;
    reg[5:0] job_wait;
    reg[15:0] job_epoch;
    reg[1:0] job_style;
    reg[31:0] job_frame_id;
    assign job_guided_ok=GUIDE_ENABLE==0||
        (job_guide_slot_valid&&job_guide_ready&&job_guide_fid==job_frame_id);
    reg[31:0] raw_drop_base,raw_short_base;
    wire current_camera_bad;
    reg[31:0] presented_frames,published_frames,raw_samples,sticky_faults;
    reg[31:0] finished_pixels;
    wire[31:0] live_faults;
    reg display_underrun_pix;
    (* async_reg = "true" *) reg [1:0] display_underrun_sync;
    wire safe_to_load=(job_state==J_IDLE||job_state==J_PUBLISH)&&core_idle&&compute_release&&style_reader_idle;


    localparam integer H         = ROWS * 4;
    localparam integer BLK_BEATS = `D64_ROWS_TO_BEATS(ROWS);

    localparam [31:0] H_BASE       = 32'h0010_0000;
    localparam [31:0] D0_BASE      = 32'h0018_0000;
    localparam [31:0] TRUNK_A_BASE = 32'h001C_0000;
    localparam [31:0] TRUNK_B_BASE = 32'h0040_0000;
    localparam [31:0] TAIL_BASE    = 32'h005C_0000;

    assign ddr_pll_rstn = 1'b1;
    assign sys_pll_rstn = 1'b1;
    assign pe_pll_rstn  = 1'b1;

    reg [3:0] por = 4'd0;
    always @(posedge clk_25m) if (!(&por)) por <= por + 4'd1;

    reg [1:0] pll_locked_s;
    wire plls_locked = pll_lock && pll1_lock && pe_pll_lock;
    always @(posedge clk_25m)
        pll_locked_s <= {pll_locked_s[0], plls_locked};

    reg [7:0] lock_hold = 8'd0;
    always @(posedge clk_25m) begin
        if (!(&por) || !pll_locked_s[1]) lock_hold <= 8'd0;
        else if (!(&lock_hold))          lock_hold <= lock_hold + 8'd1;
    end

    reg started_q;
    always @(posedge clk_25m) started_q <= (&por) & (&lock_hold);

    wire rst_sw    = ~started_q;
    wire rst_axi_n = ~rst_sw;

    // Dedicated DDR release source; global/PE/display reset paths are intact.
    wire rst_ddr_n;
    d64_ddr_clk25_release u_ddr_release (
        .clk_25m(clk_25m),.global_rst_n(rst_axi_n),.ddr_release_n(rst_ddr_n));


    d64_reset_sync u_pe_reset (.clk(clk_pe), .arst_n(rst_axi_n), .rst_n(rst_pe_n));

    wire rst_disp_n;
    d64_reset_sync u_disp_reset (.clk(hdmi_tx_slow_clk), .arst_n(rst_axi_n),
                                 .rst_n(rst_disp_n));

    wire fabric_arst_n;

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

    wire [CMD_W-1:0] m_cmd [0:9];
    wire [RSP_W-1:0] m_rsp [0:9];
    wire [CMD_W-1:0] a_cmd [0:8];
    wire [RSP_W-1:0] a_rsp [0:8];
    wire [CMD_W-1:0] guide_rd_cmd,guide_wr_cmd,profile_cmd,flash_cmd;
    wire [RSP_W-1:0] profile_rsp,guide_rd_rsp;
    wire [CMD_W-1:0] br_cmd;
    wire [RSP_W-1:0] br_rsp;
    wire [CMD_W-1:0] hdmi_priority_cmd;
    wire [RSP_W-1:0] hdmi_priority_rsp;

    wire ddr_cal_done, ddr_cal_pass;
    wire [7:0] cal_fail_log;
    wire [2:0] cal_shift_val;

    wire [31:0] axi_aaddr;
    wire [7:0]  axi_alen;
    wire [2:0]  axi_asize;
    wire [1:0]  axi_aburst, axi_alock, axi_bresp, axi_rresp;
    wire        axi_avalid, axi_aready, axi_atype;
    wire [127:0] axi_wdata, axi_rdata;
    wire [15:0] axi_wstrb;
    wire        axi_wlast, axi_wvalid, axi_wready;
    wire        axi_bvalid, axi_bready, axi_rlast, axi_rvalid, axi_rready;

    wire [7:0] axi_aid = 8'd0;
    wire [7:0] axi_wid = 8'd0;
    wire [7:0] axi_rid, axi_bid;

`ifdef SIM_DDR
        d64_ddr_ip_model #(.ADDR_W(GUIDE_ENABLE!=0?22:19)) u_ddr3 (
            .clk(clk_pe), .core_clk(ddr_core_clk), .twd_clk(ddr_twd_clk),
            .tdqss_clk(ddr_tdqss_clk), .tac_clk(ddr_tac_clk),
            .reset_n(rst_ddr_n), .reset(ddr_reset), .cs(ddr_cs), .ras(ddr_ras),
            .cas(ddr_cas), .we(ddr_we), .cke(ddr_cke), .addr(ddr_addr),
            .ba(ddr_ba), .odt(ddr_odt), .shift(ddr_shift),
            .shift_sel(ddr_shift_sel), .shift_ena(ddr_shift_ena),
            .cal_ena(1'b1), .cal_done(ddr_cal_done), .cal_pass(ddr_cal_pass),
            .cal_shift_val(cal_shift_val), .cal_fail_log(cal_fail_log),
            .o_dm_hi(o_ddr_dm_hi), .o_dm_lo(o_ddr_dm_lo),
            .i_dqs_hi(i_ddr_dqs_hi), .i_dqs_lo(i_ddr_dqs_lo),
            .i_dqs_n_hi(i_ddr_dqs_n_hi), .i_dqs_n_lo(i_ddr_dqs_n_lo),
            .o_dqs_hi(o_ddr_dqs_hi), .o_dqs_lo(o_ddr_dqs_lo),
            .o_dqs_n_hi(o_ddr_dqs_n_hi), .o_dqs_n_lo(o_ddr_dqs_n_lo),
            .o_dqs_oe(o_ddr_dqs_oe), .o_dqs_n_oe(o_ddr_dqs_n_oe),
            .i_dq_hi(i_ddr_dq_hi), .i_dq_lo(i_ddr_dq_lo),
            .o_dq_hi(o_ddr_dq_hi), .o_dq_lo(o_ddr_dq_lo), .o_dq_oe(o_ddr_dq_oe),
            .axi_aid(axi_aid), .axi_aaddr(axi_aaddr), .axi_alen(axi_alen),
            .axi_asize(axi_asize), .axi_aburst(axi_aburst),
            .axi_alock(axi_alock),
            .axi_avalid(axi_avalid), .axi_aready(axi_aready),
            .axi_atype(axi_atype),
            .axi_wid(axi_wid), .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb),
            .axi_wlast(axi_wlast), .axi_wvalid(axi_wvalid),
            .axi_wready(axi_wready),
            .axi_rid(axi_rid), .axi_rdata(axi_rdata), .axi_rlast(axi_rlast),
            .axi_rvalid(axi_rvalid), .axi_rready(axi_rready),
            .axi_rresp(axi_rresp),
            .axi_bid(axi_bid), .axi_bresp(axi_bresp), .axi_bvalid(axi_bvalid),
            .axi_bready(axi_bready)
        );
`else
    DDR3IPCORE u_ddr3 (
        .clk(clk_pe), .core_clk(ddr_core_clk), .twd_clk(ddr_twd_clk),
        .tdqss_clk(ddr_tdqss_clk), .tac_clk(ddr_tac_clk),
        .reset_n(rst_ddr_n), .reset(ddr_reset), .cs(ddr_cs), .ras(ddr_ras),
        .cas(ddr_cas), .we(ddr_we), .cke(ddr_cke), .addr(ddr_addr),
        .ba(ddr_ba), .odt(ddr_odt), .shift(ddr_shift),
        .shift_sel(ddr_shift_sel), .shift_ena(ddr_shift_ena),
        .cal_ena(1'b1), .cal_done(ddr_cal_done), .cal_pass(ddr_cal_pass),
        .cal_shift_val(cal_shift_val), .cal_fail_log(cal_fail_log),
        .o_dm_hi(o_ddr_dm_hi), .o_dm_lo(o_ddr_dm_lo),
        .i_dqs_hi(i_ddr_dqs_hi), .i_dqs_lo(i_ddr_dqs_lo),
        .i_dqs_n_hi(i_ddr_dqs_n_hi), .i_dqs_n_lo(i_ddr_dqs_n_lo),
        .o_dqs_hi(o_ddr_dqs_hi), .o_dqs_lo(o_ddr_dqs_lo),
        .o_dqs_n_hi(o_ddr_dqs_n_hi), .o_dqs_n_lo(o_ddr_dqs_n_lo),
        .o_dqs_oe(o_ddr_dqs_oe), .o_dqs_n_oe(o_ddr_dqs_n_oe),
        .i_dq_hi(i_ddr_dq_hi), .i_dq_lo(i_ddr_dq_lo),
        .o_dq_hi(o_ddr_dq_hi), .o_dq_lo(o_ddr_dq_lo), .o_dq_oe(o_ddr_dq_oe),
        .axi_aid(axi_aid), .axi_aaddr(axi_aaddr), .axi_alen(axi_alen),
        .axi_asize(axi_asize), .axi_aburst(axi_aburst), .axi_alock(axi_alock),
        .axi_avalid(axi_avalid), .axi_aready(axi_aready), .axi_atype(axi_atype),
        .axi_wid(axi_wid), .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb),
        .axi_wlast(axi_wlast), .axi_wvalid(axi_wvalid), .axi_wready(axi_wready),
        .axi_rid(axi_rid), .axi_rdata(axi_rdata), .axi_rlast(axi_rlast),
        .axi_rvalid(axi_rvalid), .axi_rready(axi_rready), .axi_rresp(axi_rresp),
        .axi_bid(axi_bid), .axi_bresp(axi_bresp), .axi_bvalid(axi_bvalid),
        .axi_bready(axi_bready)
    );
`endif

    wire [31:0] bridge_a_addr;
    wire [7:0]  bridge_a_len;
    wire [2:0]  bridge_a_size;
    wire [1:0]  bridge_a_burst;
    wire        bridge_a_valid, bridge_a_type;
    wire [127:0] bridge_w_data;
    wire [15:0]  bridge_w_strb;
    wire        bridge_w_valid, bridge_w_last;
    wire        bridge_b_valid;
    wire [1:0]  bridge_b_resp;
    wire [127:0] bridge_r_data;
    wire        bridge_r_valid, bridge_r_last;
    wire [1:0]  bridge_r_resp;
    wire [31:0] bridge_dual_wait, bridge_aw_accepts, bridge_ar_accepts;

    d64_efx_ddr_axi_bridge u_axi_bridge (
        .clk(clk_pe), .rst_n(fabric_arst_n),
        .m_awvalid(br_cmd[CAWVALID]), .m_awready(br_rsp[RAWREADY]),
        .m_awaddr(br_cmd[CAWADDR +: 32]), .m_awlen(br_cmd[CAWLEN +: 8]),
        .m_awsize(br_cmd[CAWSIZE +: 3]), .m_awburst(br_cmd[CAWBURST +: 2]),
        .m_arvalid(br_cmd[CARVALID]), .m_arready(br_rsp[RARREADY]),
        .m_araddr(br_cmd[CARADDR +: 32]), .m_arlen(br_cmd[CARLEN +: 8]),
        .m_arsize(br_cmd[CARSIZE +: 3]), .m_arburst(br_cmd[CARBURST +: 2]),
        .ddr_aaddr(bridge_a_addr), .ddr_alen(bridge_a_len),
        .ddr_asize(bridge_a_size), .ddr_aburst(bridge_a_burst),
        .ddr_avalid(bridge_a_valid), .ddr_aready(axi_aready),
        .ddr_atype(bridge_a_type),
        .m_wdata(br_cmd[CWVDATA +: 128]), .m_wstrb(br_cmd[CWVSTRB +: 16]),
        .m_wlast(br_cmd[CWVLAST]), .m_wvalid(br_cmd[CWVVALID]),
        .m_wready(br_rsp[RWREDY]),
        .ddr_wdata(bridge_w_data), .ddr_wstrb(bridge_w_strb),
        .ddr_wlast(bridge_w_last), .ddr_wvalid(bridge_w_valid),
        .ddr_wready(axi_wready),
        .m_bvalid(br_rsp[RBVALID]), .m_bresp(br_rsp[RBRESPO +: 2]),
        .m_bready(br_cmd[CBREDY]),
        .ddr_bvalid(bridge_b_valid), .ddr_bresp(bridge_b_resp),
        .ddr_bready(axi_bready),
        .m_rdata(br_rsp[RRDATA +: 128]), .m_rlast(br_rsp[RRLAST]),
        .m_rresp(br_rsp[RBRESPR +: 2]), .m_rvalid(br_rsp[RRVALID]),
        .m_rready(br_cmd[CRREDY]),
        .ddr_rdata(bridge_r_data), .ddr_rlast(bridge_r_last),
        .ddr_rresp(bridge_r_resp), .ddr_rvalid(bridge_r_valid),
        .ddr_rready(axi_rready),
        .simultaneous_wait_cycles(bridge_dual_wait),
        .write_address_accepts(bridge_aw_accepts),
        .read_address_accepts(bridge_ar_accepts)
    );

    assign axi_avalid = bridge_a_valid;
    assign axi_atype  = bridge_a_type;
    assign axi_aaddr  = bridge_a_addr;
    assign axi_alen   = bridge_a_len;
    assign axi_asize  = bridge_a_size;
    assign axi_aburst = bridge_a_burst;

    assign axi_wvalid = bridge_w_valid;
    assign axi_wdata  = bridge_w_data;
    assign axi_wstrb  = bridge_w_strb;
    assign axi_wlast  = bridge_w_last;

    assign bridge_b_valid = axi_bvalid;
    assign bridge_b_resp  = axi_bresp;

    assign bridge_r_data  = axi_rdata;
    assign bridge_r_last  = axi_rlast;
    assign bridge_r_resp  = axi_rresp;
    assign bridge_r_valid = axi_rvalid;

    assign axi_alock = 2'b00;

    wire [31:0] arb_w_holds [0:8];
    wire [31:0] arb_r_holds [0:8];


    // Display m6 stays at the outermost read arbiter. Profile and guide reads
    // Guide/profile plus its writer sit outside all CNN branches; camera sits
    // outside guide. Flash lends its write leaf only after boot retirement.
    assign guide_rd_cmd[CAWVALID]=0;assign guide_rd_cmd[CWVVALID]=0;assign guide_rd_cmd[CBREDY]=1;
    assign guide_rd_cmd[CAWADDR+:32]=0;assign guide_rd_cmd[CAWLEN+:8]=0;
    assign guide_rd_cmd[CAWSIZE+:3]=4;assign guide_rd_cmd[CAWBURST+:2]=1;
    assign guide_rd_cmd[CWVDATA+:128]=0;assign guide_rd_cmd[CWVSTRB+:16]=0;
    assign guide_rd_cmd[CWVLAST]=0;
    assign guide_wr_cmd[CARVALID]=0;assign guide_wr_cmd[CRREDY]=1;
    assign guide_wr_cmd[CARADDR+:32]=0;assign guide_wr_cmd[CARLEN+:8]=0;
    assign guide_wr_cmd[CARSIZE+:3]=4;assign guide_wr_cmd[CARBURST+:2]=1;
    generate if(GUIDE_ENABLE!=0)begin:g_guide_bus
        d64_axi_arb2 u_read_arb(.clk(clk_pe),.rst_n(fabric_arst_n),
            .m0_awvalid(profile_cmd[CAWVALID]),
            .m0_awready(profile_rsp[RAWREADY]),
            .m0_awaddr(profile_cmd[CAWADDR +: 32]),
            .m0_awlen(profile_cmd[CAWLEN +: 8]),
            .m0_awsize(profile_cmd[CAWSIZE +: 3]),
            .m0_awburst(profile_cmd[CAWBURST +: 2]),
            .m0_wvalid(profile_cmd[CWVVALID]),
            .m0_wready(profile_rsp[RWREDY]),
            .m0_wdata(profile_cmd[CWVDATA +: 128]),
            .m0_wstrb(profile_cmd[CWVSTRB +: 16]),
            .m0_wlast(profile_cmd[CWVLAST]),
            .m0_bvalid(profile_rsp[RBVALID]),
            .m0_bready(profile_cmd[CBREDY]),
            .m0_bresp(profile_rsp[RBRESPO +: 2]),
            .m0_arvalid(profile_cmd[CARVALID]),
            .m0_arready(profile_rsp[RARREADY]),
            .m0_araddr(profile_cmd[CARADDR +: 32]),
            .m0_arlen(profile_cmd[CARLEN +: 8]),
            .m0_arsize(profile_cmd[CARSIZE +: 3]),
            .m0_arburst(profile_cmd[CARBURST +: 2]),
            .m0_rvalid(profile_rsp[RRVALID]),
            .m0_rready(profile_cmd[CRREDY]),
            .m0_rdata(profile_rsp[RRDATA +: 128]),
            .m0_rlast(profile_rsp[RRLAST]),
            .m0_rresp(profile_rsp[RBRESPR +: 2]),
            .m1_awvalid(guide_rd_cmd[CAWVALID]),
            .m1_awready(guide_rd_rsp[RAWREADY]),
            .m1_awaddr(guide_rd_cmd[CAWADDR +: 32]),
            .m1_awlen(guide_rd_cmd[CAWLEN +: 8]),
            .m1_awsize(guide_rd_cmd[CAWSIZE +: 3]),
            .m1_awburst(guide_rd_cmd[CAWBURST +: 2]),
            .m1_wvalid(guide_rd_cmd[CWVVALID]),
            .m1_wready(guide_rd_rsp[RWREDY]),
            .m1_wdata(guide_rd_cmd[CWVDATA +: 128]),
            .m1_wstrb(guide_rd_cmd[CWVSTRB +: 16]),
            .m1_wlast(guide_rd_cmd[CWVLAST]),
            .m1_bvalid(guide_rd_rsp[RBVALID]),
            .m1_bready(guide_rd_cmd[CBREDY]),
            .m1_bresp(guide_rd_rsp[RBRESPO +: 2]),
            .m1_arvalid(guide_rd_cmd[CARVALID]),
            .m1_arready(guide_rd_rsp[RARREADY]),
            .m1_araddr(guide_rd_cmd[CARADDR +: 32]),
            .m1_arlen(guide_rd_cmd[CARLEN +: 8]),
            .m1_arsize(guide_rd_cmd[CARSIZE +: 3]),
            .m1_arburst(guide_rd_cmd[CARBURST +: 2]),
            .m1_rvalid(guide_rd_rsp[RRVALID]),
            .m1_rready(guide_rd_cmd[CRREDY]),
            .m1_rdata(guide_rd_rsp[RRDATA +: 128]),
            .m1_rlast(guide_rd_rsp[RRLAST]),
            .m1_rresp(guide_rd_rsp[RBRESPR +: 2]),
            .s_awvalid(guide_profile_cmd[CAWVALID]),
            .s_awready(guide_profile_rsp[RAWREADY]),
            .s_awaddr(guide_profile_cmd[CAWADDR +: 32]),
            .s_awlen(guide_profile_cmd[CAWLEN +: 8]),
            .s_awsize(guide_profile_cmd[CAWSIZE +: 3]),
            .s_awburst(guide_profile_cmd[CAWBURST +: 2]),
            .s_wvalid(guide_profile_cmd[CWVVALID]),
            .s_wready(guide_profile_rsp[RWREDY]),
            .s_wdata(guide_profile_cmd[CWVDATA +: 128]),
            .s_wstrb(guide_profile_cmd[CWVSTRB +: 16]),
            .s_wlast(guide_profile_cmd[CWVLAST]),
            .s_bvalid(guide_profile_rsp[RBVALID]),
            .s_bready(guide_profile_cmd[CBREDY]),
            .s_bresp(guide_profile_rsp[RBRESPO +: 2]),
            .s_arvalid(guide_profile_cmd[CARVALID]),
            .s_arready(guide_profile_rsp[RARREADY]),
            .s_araddr(guide_profile_cmd[CARADDR +: 32]),
            .s_arlen(guide_profile_cmd[CARLEN +: 8]),
            .s_arsize(guide_profile_cmd[CARSIZE +: 3]),
            .s_arburst(guide_profile_cmd[CARBURST +: 2]),
            .s_rvalid(guide_profile_rsp[RRVALID]),
            .s_rready(guide_profile_cmd[CRREDY]),
            .s_rdata(guide_profile_rsp[RRDATA +: 128]),
            .s_rlast(guide_profile_rsp[RRLAST]),
            .s_rresp(guide_profile_rsp[RBRESPR +: 2]),.write_owner_holds(),.read_owner_holds());
        assign m_cmd[9]=(((asset_ready & 4'b1110)==4'b1110)&&!flash_busy&&!flash_failed)?guide_wr_cmd:flash_cmd;
    end else begin:g_no_guide_bus
        assign guide_profile_cmd=profile_cmd;assign profile_rsp=guide_profile_rsp;
        assign m_cmd[9]=flash_cmd;assign guide_rd_rsp=0;
    end endgenerate

    d64_axi_join u_interaction_bus(.clk(clk_pe),.rst_n(fabric_arst_n),
        .m0_cmd(guide_profile_cmd),.m0_rsp(guide_profile_rsp),
        .m1_cmd(int_extra_cmd),.m1_rsp(int_extra_rsp),.s_cmd(m_cmd[8]),.s_rsp(m_rsp[8]));
    // Comparison rows have a 960-pixel (about13us) deadline. Give this read
    // port priority before long-running camera/CNN traffic; the native source
    // row fetch has several repeated raster lines of prefetch slack. All
    // accepted bursts still retire under unchanged AXI ownership rules.
    d64_axi_join u_hdmi_priority(.clk(clk_pe),.rst_n(fabric_arst_n),
        .m0_cmd(int_comparison_cmd),.m0_rsp(int_comparison_rsp),
        .m1_cmd(m_cmd[6]),.m1_rsp(m_rsp[6]),.s_cmd(hdmi_priority_cmd),.s_rsp(hdmi_priority_rsp));
    genvar gi;
    generate
    for (gi = 0; gi < 9; gi = gi + 1) begin : g_arb
        // At gi3, source guide writes precede optional image-plane writes.
        // Its AR channel is inactive, so this swap leaves read priority intact.
        wire [CMD_W-1:0] m0c = (gi == 0) ? m_cmd[0] : (gi == 1) ? m_cmd[2] : (gi == 2) ? m_cmd[4] : (gi == 3) ? m_cmd[9] : (gi == 4) ? a_cmd[0] : (gi == 5) ? a_cmd[4] : (gi == 6) ? a_cmd[3] : (gi == 7) ? m_cmd[7] : hdmi_priority_cmd;
        wire [CMD_W-1:0] m1c = (gi == 0) ? m_cmd[1] : (gi == 1) ? m_cmd[3] : (gi == 2) ? m_cmd[5] : (gi == 3) ? m_cmd[8] : (gi == 4) ? a_cmd[1] : (gi == 5) ? a_cmd[2] : (gi == 6) ? a_cmd[5] : (gi == 7) ? a_cmd[6] : a_cmd[7];
        wire [RSP_W-1:0] m0r, m1r;
        if(gi==0) begin assign m_rsp[0]=m0r; assign m_rsp[1]=m1r; end
        if(gi==1) begin assign m_rsp[2]=m0r; assign m_rsp[3]=m1r; end
        if(gi==2) begin assign m_rsp[4]=m0r; assign m_rsp[5]=m1r; end
        if(gi==3) begin assign m_rsp[9]=m0r; assign m_rsp[8]=m1r; end
        if(gi==4) begin assign a_rsp[0]=m0r; assign a_rsp[1]=m1r; end
        if(gi==5) begin assign a_rsp[4]=m0r; assign a_rsp[2]=m1r; end
        if(gi==6) begin assign a_rsp[3]=m0r; assign a_rsp[5]=m1r; end
        if(gi==7) begin assign m_rsp[7]=m0r; assign a_rsp[6]=m1r; end
        if(gi==8) begin assign hdmi_priority_rsp=m0r; assign a_rsp[7]=m1r; end
        d64_axi_arb2 u_arb (
            .clk(clk_pe), .rst_n(fabric_arst_n),
            .m0_awvalid(m0c[CAWVALID]), .m0_awready(m0r[RAWREADY]),
            .m0_awaddr(m0c[CAWADDR +: 32]), .m0_awlen(m0c[CAWLEN +: 8]),
            .m0_awsize(m0c[CAWSIZE +: 3]), .m0_awburst(m0c[CAWBURST +: 2]),
            .m0_wvalid(m0c[CWVVALID]),   .m0_wready(m0r[RWREDY]),
            .m0_wdata(m0c[CWVDATA +: 128]), .m0_wstrb(m0c[CWVSTRB +: 16]),
            .m0_wlast(m0c[CWVLAST]),
            .m0_bvalid(m0r[RBVALID]), .m0_bready(m0c[CBREDY]),
            .m0_bresp(m0r[RBRESPO +: 2]),
            .m0_arvalid(m0c[CARVALID]), .m0_arready(m0r[RARREADY]),
            .m0_araddr(m0c[CARADDR +: 32]), .m0_arlen(m0c[CARLEN +: 8]),
            .m0_arsize(m0c[CARSIZE +: 3]), .m0_arburst(m0c[CARBURST +: 2]),
            .m0_rvalid(m0r[RRVALID]), .m0_rready(m0c[CRREDY]),
            .m0_rdata(m0r[RRDATA +: 128]), .m0_rlast(m0r[RRLAST]),
            .m0_rresp(m0r[RBRESPR +: 2]),

            .m1_awvalid(m1c[CAWVALID]), .m1_awready(m1r[RAWREADY]),
            .m1_awaddr(m1c[CAWADDR +: 32]), .m1_awlen(m1c[CAWLEN +: 8]),
            .m1_awsize(m1c[CAWSIZE +: 3]), .m1_awburst(m1c[CAWBURST +: 2]),
            .m1_wvalid(m1c[CWVVALID]),   .m1_wready(m1r[RWREDY]),
            .m1_wdata(m1c[CWVDATA +: 128]), .m1_wstrb(m1c[CWVSTRB +: 16]),
            .m1_wlast(m1c[CWVLAST]),
            .m1_bvalid(m1r[RBVALID]), .m1_bready(m1c[CBREDY]),
            .m1_bresp(m1r[RBRESPO +: 2]),
            .m1_arvalid(m1c[CARVALID]), .m1_arready(m1r[RARREADY]),
            .m1_araddr(m1c[CARADDR +: 32]), .m1_arlen(m1c[CARLEN +: 8]),
            .m1_arsize(m1c[CARSIZE +: 3]), .m1_arburst(m1c[CARBURST +: 2]),
            .m1_rvalid(m1r[RRVALID]), .m1_rready(m1c[CRREDY]),
            .m1_rdata(m1r[RRDATA +: 128]), .m1_rlast(m1r[RRLAST]),
            .m1_rresp(m1r[RBRESPR +: 2]),

            .s_awvalid(a_cmd[gi][CAWVALID]), .s_awready(a_rsp[gi][RAWREADY]),
            .s_awaddr(a_cmd[gi][CAWADDR +: 32]),
            .s_awlen(a_cmd[gi][CAWLEN +: 8]),
            .s_awsize(a_cmd[gi][CAWSIZE +: 3]),
            .s_awburst(a_cmd[gi][CAWBURST +: 2]),
            .s_wvalid(a_cmd[gi][CWVVALID]), .s_wready(a_rsp[gi][RWREDY]),
            .s_wdata(a_cmd[gi][CWVDATA +: 128]),
            .s_wstrb(a_cmd[gi][CWVSTRB +: 16]),
            .s_wlast(a_cmd[gi][CWVLAST]),
            .s_bvalid(a_rsp[gi][RBVALID]), .s_bready(a_cmd[gi][CBREDY]),
            .s_bresp(a_rsp[gi][RBRESPO +: 2]),
            .s_arvalid(a_cmd[gi][CARVALID]), .s_arready(a_rsp[gi][RARREADY]),
            .s_araddr(a_cmd[gi][CARADDR +: 32]),
            .s_arlen(a_cmd[gi][CARLEN +: 8]),
            .s_arsize(a_cmd[gi][CARSIZE +: 3]),
            .s_arburst(a_cmd[gi][CARBURST +: 2]),
            .s_rvalid(a_rsp[gi][RRVALID]), .s_rready(a_cmd[gi][CRREDY]),
            .s_rdata(a_rsp[gi][RRDATA +: 128]), .s_rlast(a_rsp[gi][RRLAST]),
            .s_rresp(a_rsp[gi][RBRESPR +: 2]),
            .write_owner_holds(arb_w_holds[gi]),
            .read_owner_holds(arb_r_holds[gi])
        );
    end
    endgenerate

    assign br_cmd = a_cmd[8];
    assign a_rsp[8] = br_rsp;

    wire [31:0] arb_w_holds_all = arb_w_holds[0] + arb_w_holds[1] +
                                  arb_w_holds[2] + arb_w_holds[3] +
                                  arb_w_holds[4] + arb_w_holds[5];
    wire [31:0] arb_r_holds_all = arb_r_holds[0] + arb_r_holds[1] +
                                  arb_r_holds[2] + arb_r_holds[3] +
                                  arb_r_holds[4] + arb_r_holds[5];

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

    d64_cam_rgb256_pixel_fifo #(
        .W_SRC(CAM_W), .H_SRC(CAM_H), .W_OUT(OUT_W), .H_OUT(OUT_H),
        .CFA(CAM_CFA), .BLACK(10'd64), .GAIN_Q8(16'd256), .FIFO_AW(5),
        .WB_R_Q8(CAM_WB_R_Q8), .WB_G_Q8(CAM_WB_G_Q8),
        .WB_B_Q8(CAM_WB_B_Q8), .GAMMA_EN(CAM_GAMMA_EN),
        .GUIDE_ENABLE(GUIDE_ENABLE && CAM_PACKED==0), .GUIDE_GAMMA_EN(0),
        .ORIGINAL_ENABLE(COMPARISON_ENABLE)
    ) u_cam (
        .clk(clk_pe), .rst_n(rst_pe_n && CAM_PACKED==0),
        .raw_valid(cam_raw_valid && CAM_PACKED==0), .raw10(cam_raw10),
        .raw_sof(cam_sof), .raw_sol(cam_sol),
        .raw_eol(cam_eol), .raw_eof(cam_eof), .raw_frame_id(cam_frame_id),
        .d64_valid(scalar_cap_valid), .d64_ready(cap_ready),
        .d64_frame_id(scalar_cap_fid), .d64_sof(scalar_cap_sof), .d64_x0(scalar_cap_x),
        .d64_y(scalar_cap_y), .d64_rgb888(scalar_cap_rgb),
        .d64_eol(scalar_cap_eol), .d64_eof(scalar_cap_eof),
        .src_samples(scalar_samples), .src_lines(scalar_lines),
        .src_frames(scalar_frames), .demos_px(scalar_demos),
        .out_px(scalar_outpx), .beats_out(scalar_beats),
        .frames_out(scalar_frames_out), .dropped_beats(scalar_dropped),
        .short_lines(scalar_short), .geom_margin_ok(scalar_margin)
        ,.guide_valid(scalar_guide_valid),.guide_g8(scalar_guide_gray),
        .guide_x(scalar_guide_x),.guide_y(scalar_guide_y),.guide_frame_id(scalar_guide_fid),
        .original_valid(int_original_valid),.original_rgb565(int_original_rgb565),
        .original_x(int_original_x),.original_y(int_original_y),.original_frame_id(int_original_fid)
    );

    wire packed_bad_sync;
    generate if(CAM_PACKED!=0)begin:g_packed_camera
        wire [31:0] native_beats,native_out,native_drop,native_markers;
        wire [31:0] crossed_beats,crossed_out,crossed_drop,crossed_fid;
        wire overflow_native;
        wire native_guide_valid,native_guide_sof,native_guide_eol,native_guide_eof,native_guide_fault;
        wire [31:0] native_guide4;
        (* async_reg="true" *) reg [1:0] source_bad_sync;
        always @(posedge clk_pe or negedge rst_pe_n)
            if(!rst_pe_n)source_bad_sync<=0;
            else source_bad_sync<={source_bad_sync[0],cam_packed_source_bad||overflow_native||(native_markers!=0)};
        assign packed_bad_sync=source_bad_sync[1];
        d64_packed_raw_to_rgb256 #(.W_SRC(CAM_W),.H_SRC(CAM_H),.W_OUT(OUT_W),.H_OUT(OUT_H),
            .CFA(CAM_CFA),.BLACK(64),.GAIN_Q8(256),.WB_R_Q8(CAM_WB_R_Q8),
            .WB_G_Q8(CAM_WB_G_Q8),.WB_B_Q8(CAM_WB_B_Q8),.GAMMA_EN(CAM_GAMMA_EN),
            .GUIDE_ENABLE(GUIDE_ENABLE),.GUIDE_GAMMA_EN(0)) u_packed(
            .clk_pix(cam_packed_clk),.rst_pix_n(cam_packed_rst_n),.raw_valid(cam_packed_valid),
            .raw4(cam_packed_raw4),.raw_sof(cam_packed_sof),.raw_sol(cam_packed_sol),
            .raw_eol(cam_packed_eol),.raw_eof(cam_packed_eof),.raw_frame_id(cam_packed_fid),
            .clk(clk_pe),.rst_n(rst_pe_n),.d64_valid(cap_valid),.d64_ready(cap_ready),
            .d64_frame_id(cap_frame_id),.d64_sof(cap_sof),.d64_x0(cap_x0),.d64_y(cap_y),
            .d64_rgb888(cap_rgb888),.d64_eol(cap_eol),.d64_eof(cap_eof),
            .src_beats(native_beats),.out_px(native_out),.dropped_beats(native_drop),
            .marker_errors(native_markers),.beats_out(cam_beats),.frames_out(cam_frames_out),
            .overflow_sticky(overflow_native),.guide4_valid(native_guide_valid),.guide4(native_guide4),
            .guide_x0(),.guide_y(),.guide_frame_id(),.guide_sof(native_guide_sof),.guide_sol(),
            .guide_eol(native_guide_eol),.guide_eof(native_guide_eof),.guide_fault_sticky(native_guide_fault));
        if(GUIDE_ENABLE!=0)begin:g_guide_decode
            d64_packed_guide_to_scalar u_decode(.clk_pix(cam_packed_clk),.rst_pix_n(cam_packed_rst_n),
                .frame_header(cam_packed_valid&&cam_packed_sof),.header_fid(cam_packed_fid),
                .guide_valid(native_guide_valid),.guide4(native_guide4),.guide_sof(native_guide_sof),
                .guide_eol(native_guide_eol),.guide_eof(native_guide_eof),.source_fault(native_guide_fault),
                .clk(clk_pe),.rst_n(fabric_arst_n),.out_valid(guide_scalar_valid),.out_gray(guide_scalar_gray),
                .out_x(guide_scalar_x),.out_y(guide_scalar_y),.out_fid(guide_scalar_fid),.fault(guide_scalar_fault));
        end else begin:g_no_guide_decode
            assign guide_scalar_valid=0;assign guide_scalar_gray=0;
            assign guide_scalar_x=0;assign guide_scalar_y=0;assign guide_scalar_fid=0;assign guide_scalar_fault=0;
        end
        d64_counter_gray_bridge u_count_beats(.src_clk(cam_packed_clk),.src_rst_n(cam_packed_rst_n),
            .src_count(native_beats),.dst_clk(clk_pe),.dst_rst_n(rst_pe_n),.dst_count(crossed_beats));
        d64_counter_gray_bridge u_count_out(.src_clk(cam_packed_clk),.src_rst_n(cam_packed_rst_n),
            .src_count(native_out),.dst_clk(clk_pe),.dst_rst_n(rst_pe_n),.dst_count(crossed_out));
        d64_counter_gray_bridge u_count_drop(.src_clk(cam_packed_clk),.src_rst_n(cam_packed_rst_n),
            .src_count(native_drop),.dst_clk(clk_pe),.dst_rst_n(rst_pe_n),.dst_count(crossed_drop));
        d64_counter_gray_bridge u_count_fid(.src_clk(cam_packed_clk),.src_rst_n(cam_packed_rst_n),
            .src_count(cam_packed_fid),.dst_clk(clk_pe),.dst_rst_n(rst_pe_n),.dst_count(crossed_fid));
        assign cam_src_samples=crossed_beats<<2;
        assign cam_src_lines=0;
        assign cam_src_frames=crossed_fid;
        assign cam_demos_px=crossed_out;
        assign cam_out_px=crossed_out;
        assign cam_dropped=crossed_drop;
        assign cam_short_lines={31'd0,packed_bad_sync};
        assign cam_geom_margin_ok=1;
    end else begin:g_scalar_camera_outputs
        if(GUIDE_ENABLE!=0)begin:g_scalar_guide_edges
            d64_scalar_guide_edge_extend #(.W(CAM_W),.H(CAM_H)) u_edges(
                .clk(clk_pe),.rst_n(fabric_arst_n),.in_valid(scalar_guide_valid),
                .in_gray(scalar_guide_gray),.in_x(scalar_guide_x),.in_y(scalar_guide_y),.in_fid(scalar_guide_fid),
                .out_valid(guide_scalar_valid),.out_gray(guide_scalar_gray),
                .out_x(guide_scalar_x),.out_y(guide_scalar_y),.out_fid(guide_scalar_fid),
                .out_sof(),.out_eol(),.out_eof(),.fault(guide_scalar_fault));
        end else begin:g_no_scalar_guide
            assign guide_scalar_valid=0;assign guide_scalar_gray=0;
            assign guide_scalar_x=0;assign guide_scalar_y=0;assign guide_scalar_fid=0;assign guide_scalar_fault=0;
        end
        assign packed_bad_sync=|camera_status[4:1];
        assign cap_valid=scalar_cap_valid;assign cap_frame_id=scalar_cap_fid;
        assign cap_sof=scalar_cap_sof;assign cap_x0=scalar_cap_x;assign cap_y=scalar_cap_y;
        assign cap_rgb888=scalar_cap_rgb;assign cap_eol=scalar_cap_eol;assign cap_eof=scalar_cap_eof;
        assign cam_src_samples=scalar_samples;assign cam_src_lines=scalar_lines;assign cam_src_frames=scalar_frames;
        assign cam_demos_px=scalar_demos;assign cam_out_px=scalar_outpx;
        assign cam_beats=scalar_beats;assign cam_frames_out=scalar_frames_out;
        assign cam_dropped=scalar_dropped;assign cam_short_lines=scalar_short;
        assign cam_geom_margin_ok=scalar_margin;
    end endgenerate

    assign io_cam_scl_OUT = 1'b1;
    assign io_cam_scl_OE  = 1'b0;
    assign io_cam_sda_OUT = 1'b1;
    assign io_cam_sda_OE  = 1'b0;
    assign o_cam_rst_p    = ~rst_pe_n;

    wire [31:0] fsb4_core_front_debug,fsb6_row_writer_debug;
    d64_network_core #(
        .IMG_H(H), .IMG_W(OUT_W), .SAB_SHAPE_STALE(SAB_SHAPE_STALE),
        .WT_INIT_FILE(WT_INIT_FILE), .RUNTIME_PROFILE(1), .FAULT_STICKY(1),
        .TRUNK_CFG_BIAS_W(20), .DEC_CFG_BIAS_W(13), .CFG_STREAM_SRL(1),
        .HEAD_STORE_CREDIT_ENABLE(1),
        .TAIL_RESERVE_WRITE_BURST(1), .BQ_DEPTH(256)
    ) u_core (
        .front_debug(fsb4_core_front_debug),
        .trunk_debug(),

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
    assign m_cmd[0][CWVVALID] = 1'b0;
    assign m_cmd[0][CBREDY]   = 1'b1;
    assign m_cmd[1][CARVALID] = 1'b0;
    assign m_cmd[1][CRREDY]   = 1'b1;
    assign m_cmd[2][CARVALID] = 1'b0;
    assign m_cmd[2][CRREDY]   = 1'b1;
    assign m_cmd[3][CARVALID] = 1'b0;
    assign m_cmd[3][CRREDY]   = 1'b1;

    d64_encoder_ddr_mem #(.FAULT_STICKY(1),.RESERVE_WRITE_BURST(1),
        .H_BASE(H_BASE), .D0_BASE(D0_BASE), .TRUNK_A_BASE(TRUNK_A_BASE),
        .ROWS(ROWS)
    ) u_mem (
        .writer_debug(),
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

    d64_trunk_row_ddr #(.BLK_BEATS(BLK_BEATS),.RESERVE_WRITE_BURST(1),.WF_LEVEL_W(12)) u_row (
        .row_writer_debug(fsb6_row_writer_debug),
        .clk_pe(clk_pe), .clk_axi(clk_pe), .arst_n(fabric_arst_n & compute_release),
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

    reg captured_lsb_d;
    always @(posedge clk_pe or negedge fabric_arst_n)
        if(!fabric_arst_n)captured_lsb_d<=0;else captured_lsb_d<=captured_frames[0];
    wire camera_complete_event=captured_frames[0]^captured_lsb_d;
    wire [9:0] tmds_data0,tmds_data1,tmds_data2,tmds_clk;
    wire [31:0] rows_loaded,underrun,axi_errors_v,bad_rows;
    wire starved;
    d64_native_display_chain #(.FAULT_STICKY(1),.IMG_W(OUT_W),.IMG_H(OUT_H),.SCALE(DISPLAY_SCALE),
        .GUIDE_ENABLE(GUIDE_ENABLE),.INTERACTION_ENABLE(1),.WIPE_ENABLE(COMPARISON_ENABLE),.REMOTE_DEBUG_ENABLE(REMOTE_DEBUG_ENABLE),
        .GUIDE_STORED_H(CAM_PACKED!=0?CAM_H:CAM_H-1),
        .BILINEAR_ENABLE(DISPLAY_BILINEAR),.DST_W(DISPLAY_DST_W),.DST_H(DISPLAY_DST_H),
        .FPS_OSD_ENABLE(FPS_OSD_ENABLE),.FPS_DEFAULT_ON(FPS_DEFAULT_ON),.FPS_CLK_HZ(100000000),
        .H_ACT(H_ACT),.H_FP(H_FP),.H_SYNC(H_SYNC),.H_BP(H_BP),
        .V_ACT(V_ACT),.V_FP(V_FP),.V_SYNC(V_SYNC),.V_BP(V_BP)) u_display(
        .clk_pix(hdmi_tx_slow_clk),.rst_pix_n(rst_disp_n),.clk_axi(clk_pe),.rst_axi_n(fabric_arst_n),
        .camera_complete_event(camera_complete_event),
        .remote_fps_set_valid(remote_fps_set_valid),.remote_fps_set_value(remote_fps_set_value),
        .fps_requested_pe(fps_requested_pe),.remote_actual_state_pe(remote_actual_state_pe),
        .publish_valid(publish_valid),.fps_toggle(fps_toggle),.publish_ready(publish_ready),.publish_base(publish_base),.publish_fid(publish_fid),
        .publish_guide_valid(guide_result_valid[job_slot]),
        .publish_guide_fid(job_frame_id),
        .publish_original_valid(int_original_query_valid),
        .publish_guide_slot(job_slot?guide_result_slot1:guide_result_slot0),.guide_strength(guide_strength),
        .active_base(display_base),.active_fid(display_fid),.active_valid(display_valid),
        .frame_start_axi(display_frame),.frame_start_pix(),.vblank_pix(),
        .comparison_publication_window_axi(comparison_publication_window),
        .first_scan_axi(display_scan),.scan_fid_axi(display_scan_fid),
        .tmds_data0(tmds_data0),.tmds_data1(tmds_data1),.tmds_data2(tmds_data2),.tmds_clk(tmds_clk),
        .rows_loaded(rows_loaded),.underrun(underrun),.axi_errors(axi_errors_v),.bad_rows(bad_rows),.starved(starved),
        .axi_arvalid(m_cmd[6][CARVALID]),.axi_arready(m_rsp[6][RARREADY]),.axi_araddr(m_cmd[6][CARADDR+:32]),
        .axi_arlen(m_cmd[6][CARLEN+:8]),.axi_arsize(m_cmd[6][CARSIZE+:3]),.axi_arburst(m_cmd[6][CARBURST+:2]),
        .axi_rvalid(m_rsp[6][RRVALID]),.axi_rready(m_cmd[6][CRREDY]),.axi_rdata(m_rsp[6][RRDATA+:128]),
        .axi_rlast(m_rsp[6][RRLAST]),.axi_rresp(m_rsp[6][RBRESPR+:2]),
        .guide_faults(guide_display_faults),.guide_reader_axi_fault(guide_reader_axi_fault),
        .guide_reader_protocol_fault(guide_reader_protocol_fault),.guide_reader_outstanding(guide_reader_outstanding),
        .guide_reader_slot(guide_reader_slot),.guide_araddr(guide_rd_cmd[CARADDR+:32]),
        .guide_arlen(guide_rd_cmd[CARLEN+:8]),.guide_arsize(guide_rd_cmd[CARSIZE+:3]),
        .guide_arburst(guide_rd_cmd[CARBURST+:2]),.guide_arvalid(guide_rd_cmd[CARVALID]),
        .guide_arready(guide_rd_rsp[RARREADY]),.guide_rdata(guide_rd_rsp[RRDATA+:128]),
        .guide_rvalid(guide_rd_rsp[RRVALID]),.guide_rready(guide_rd_cmd[CRREDY]),
        .guide_rlast(guide_rd_rsp[RRLAST]),.guide_rresp(guide_rd_rsp[RBRESPR+:2]),
        .compare_enabled_pe(int_compare_enabled),.effect_enabled_pe(int_effect_enabled),
        .display_mode_pe(int_display_mode),
        .comparison_publish_valid(int_comp_publish),.comparison_publish_ready(int_comp_ready),
        .comparison_original_base(int_comp_original_base),.comparison_half_base(int_comp_half_base),
        .comparison_fid(int_comp_fid),.comparison_epoch(int_comp_epoch),.comparison_slot(int_comp_slot),
        .comparison_protect_mask_pe(int_comparison_protect),.comparison_faults(int_comparison_faults),
        .processing_fps(int_processing_fps),
        .full_render_de(int_render_de),.full_render_good(int_render_good),.full_render_rgb(int_render_rgb),
        .full_render_ax(int_render_ax),.full_render_ay(int_render_ay),.full_render_fid(int_render_fid),
        .full_render_epoch(int_render_epoch),.full_render_guide_slot(int_render_slot),.full_render_slot_valid(int_render_slot_valid),
        .comparison_araddr(int_comparison_cmd[CARADDR+:32]),.comparison_arlen(int_comparison_cmd[CARLEN+:8]),
        .comparison_arsize(int_comparison_cmd[CARSIZE+:3]),.comparison_arburst(int_comparison_cmd[CARBURST+:2]),
        .comparison_arvalid(int_comparison_cmd[CARVALID]),.comparison_arready(int_comparison_rsp[RARREADY]),
        .comparison_rdata(int_comparison_rsp[RRDATA+:128]),.comparison_rvalid(int_comparison_rsp[RRVALID]),
        .comparison_rready(int_comparison_cmd[CRREDY]),.comparison_rlast(int_comparison_rsp[RRLAST]),
        .comparison_rresp(int_comparison_rsp[RBRESPR+:2]));
    assign int_comparison_cmd[191:0]=0;assign int_comparison_cmd[CBREDY]=1;
    assign m_cmd[6][CAWVALID]=0;assign m_cmd[6][CWVVALID]=0;assign m_cmd[6][CBREDY]=1;
    assign tmds_data0_TX_OE=1;assign tmds_data1_TX_OE=1;assign tmds_data2_TX_OE=1;assign tmds_clk_TX_OE=1;
    assign tmds_data0_TX_RST=0;assign tmds_data1_TX_RST=0;assign tmds_data2_TX_RST=0;assign tmds_clk_TX_RST=0;
    assign tmds_clk_o   = ~tmds_clk;
    assign tmds_data0_o = ~tmds_data0;
    assign tmds_data1_o = ~tmds_data1;
    assign tmds_data2_o = ~tmds_data2;

    assign led[0] = ddr_cal_done && ddr_cal_pass;
    assign led[1] = model_valid;
    assign led[2] = display_valid;
    assign led[3] = (underrun != 32'd0) | (axi_errors_v != 32'd0) |
                    (bad_rows != 32'd0) | (cam_dropped != 32'd0) |
                    (owner_viol != 32'd0) | (mem_axi_response_errors != 32'd0) |
                    (cam_short_lines != 32'd0) | (~cam_geom_margin_ok) |
                    core_failed | guide_any_fault | int_video_fault | int_preview_fault | (int_comparison_faults!=0);

    assign camera_accepting=ddr_cal_done&&ddr_cal_pass&&((asset_ready & 4'b1110)==4'b1110)&&!flash_failed;
    assign HPD_N = 1'b0;


    // Register the pixel-domain level before the two-flop status crossing.
    always @(posedge hdmi_tx_slow_clk or negedge rst_disp_n)begin
        if(!rst_disp_n)display_underrun_pix<=1'b0;
        else display_underrun_pix<=(underrun!=0)||starved;
    end
    always @(posedge clk_pe or negedge rst_pe_n)begin
        if(!rst_pe_n)display_underrun_sync<=0;
        else display_underrun_sync<={display_underrun_sync[0],display_underrun_pix};
    end
    assign current_camera_bad=packed_bad_sync||(cam_dropped!=raw_drop_base)||(cam_short_lines!=raw_short_base);
    assign live_faults={10'd0,int_voice_fault,(int_comparison_faults!=0),int_preview_fault,int_video_fault,guide_any_fault,camera_status[4],(owner_viol!=0),(bank_mode_viol!=0),
        (cam_short_lines!=0),(cam_dropped!=0),camera_status[3],camera_status[2],camera_status[1],
        flash_failed,deadline_missed,core_failed,display_underrun_sync[1],(axi_errors_v!=0),camera_read_failed,
        (capture_axi_errors!=0),(row_axi_errors!=0),(mem_axi_response_errors!=0)};
    always @(posedge clk_pe or negedge rst_pe_n)begin
        if(!rst_pe_n)begin presented_frames<=0;published_frames<=0;raw_samples<=0;sticky_faults<=0;finished_pixels<=0;end
        else begin
            sticky_faults<=sticky_faults|live_faults;
            if(CAM_PACKED!=0)raw_samples<=cam_src_samples;
            else if(cam_raw_valid)raw_samples<=raw_samples+1;
            if(display_scan&&display_valid)presented_frames<=presented_frames+1;
            if(publish_valid&&publish_ready)published_frames<=published_frames+1;
            if(job_state==J_RUN&&core_done)finished_pixels<=tl_pixels;
        end
    end
    wire [351:0] status_words={
        raw_samples,finished_pixels,presented_frames,camera_readback,
        {cam_short_lines[15:0],cam_dropped[15:0]},sticky_faults,switch_cycles,
        display_fid,published_frames,captured_frames,
        {active_epoch,6'd0,active_style,(job_state==J_FAULT),core_busy,model_valid,
         camera_status[3:0],(ddr_cal_done&&ddr_cal_pass)}};
    generate if(STATUS_UART_ENABLE!=0)begin:g_status_uart_enabled
    if(COMPACT_UART!=0)begin:g_compact_status
        d64_compact_status_uart #(.WORDS(4)) u_status(.clk(clk_pe),.rst_n(rst_pe_n),.words(status_words),.camera_frame_id(cam_src_frames),.tx(txd));
    end else begin:g_legacy_status
        d64_status_uart #(.WORDS(11),.PACKET_MAGIC(32'h33305346)) u_status(.clk(clk_pe),.rst_n(rst_pe_n),.tx(txd),.words(status_words));
    end
    end else begin:g_status_uart_disabled
        // FSB6 passive observation only, using explicit existing-signal outputs.
        wire [31:0] fsb_phase={m_cmd[5][CBREDY],m_rsp[5][RBVALID],m_rsp[5][RWREDY],m_cmd[5][CWVVALID],m_rsp[5][RAWREADY],m_cmd[5][CAWVALID],
            axi_wlast,(axi_avalid&&axi_atype),axi_bready,axi_bvalid,axi_wready,axi_wvalid,
            block_jobs,camera_read_failed,camera_available,model_valid,reset_req,core_axi_arst_n,core_pe_arst_n,compute_release,
            core_failed,core_done,core_busy,core_idle,dec_layer,stage,job_state};
        wire [159:0] fsb_words={fsb6_row_writer_debug,fsb4_core_front_debug,fsb_phase,
            {16'd0,published_frames[15:0]},{status_words[31:16],cam_src_frames[5:0],status_words[9:0]}};
        d64_status_uart #(.WORDS(5),.PACKET_MAGIC(32'h36425346),.DSP_REGISTER_ENABLE(1)) u_status(.clk(clk_pe),.rst_n(rst_pe_n),.words(fsb_words),.tx(txd));
    end endgenerate
    generate if(FPS_OSD_ENABLE!=0||GUIDE_ENABLE!=0) begin:g_ui_keys
        d64_ui_keys u_keys(.clk(clk_pe),.rst_n(rst_pe_n),.keys_n(keys_n),
            .active_style(active_style),.model_valid(model_valid),.target_style(ui_target_style),.target_pending(ui_target_pending),
            .request_valid(user_request),.request_style(user_style),.fps_toggle(fps_toggle),
            .compare_toggle(compare_toggle),.effect_trigger(effect_trigger),
            .external_style_valid(voice_request_valid),.external_style(voice_request_style),.external_next(voice_request_next));
    end else begin:g_style_keys
        d64_gpio_style_keys u_keys(.clk(clk_pe),.rst_n(rst_pe_n),.keys_n(keys_n),
            .request_valid(user_request),.request_style(user_style));
        assign fps_toggle=1'b0;assign compare_toggle=1'b0;assign effect_trigger=1'b0;
        assign ui_target_style=active_style;assign ui_target_pending=1'b0;
    end endgenerate
    d64_style_switch #(.BOOT_VALID(0),.SWITCH_BUDGET_CYCLES(9800000),.BOOT_SOURCE_ARM_ENABLE(1)) u_style_switch(
        .clk(clk_pe),.rst_n(rst_pe_n),.request_valid(user_request||boot_request),
        .request_style(user_request?user_style:2'd1),
        .request_is_boot(boot_request&&!user_request),
        .bootstrap_source_ready(camera_available&&(GUIDE_ENABLE==0||camera_guide_ready)),.asset_ready(asset_ready),.safe_to_load(safe_to_load),
        .load_start(load_start),.load_style(load_style),.load_done(load_done),.load_failed(load_failed),
        .load_invalidate(load_invalidate),.block_new_jobs(block_jobs),.model_valid(model_valid),
        .active_style(active_style),.active_epoch(active_epoch),.profile_publish(profile_publish),
        .display_commit(display_scan&&display_valid),.display_epoch(display_scan_fid[31:16]),
        .switch_done(switch_done),.switch_fault(switch_fault),.deadline_missed(deadline_missed),.switch_cycles(switch_cycles),.boot_wait_source(boot_wait_source),
        .boot_source_arm(boot_source_arm),.boot_active(boot_active),.boot_total_cycles(boot_total_cycles));
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
        .awburst(flash_cmd[CAWBURST+:2]),.awvalid(flash_cmd[CAWVALID]),.awready(m_rsp[9][RAWREADY]),
        .wdata(flash_cmd[CWVDATA+:128]),.wstrb(flash_cmd[CWVSTRB+:16]),.wlast(flash_cmd[CWVLAST]),
        .wvalid(flash_cmd[CWVVALID]),.wready(m_rsp[9][RWREDY]),.bvalid(m_rsp[9][RBVALID]),
        .bready(flash_cmd[CBREDY]),.bresp(m_rsp[9][RBRESPO+:2]));
    assign flash_cmd[CARVALID]=0;assign flash_cmd[CRREDY]=1;
    assign flash_cmd[CARADDR+:32]=0;assign flash_cmd[CARLEN+:8]=0;
    assign flash_cmd[CARSIZE+:3]=4;assign flash_cmd[CARBURST+:2]=1;
    assign o_spi0_io[3:1]=3'b110;assign oe_spi0_io=4'b1101;assign oe_spi_clk=1;assign oe_spi0_cs=1;
    always @(posedge clk_pe or negedge rst_pe_n)begin
        if(!rst_pe_n)begin flash_started<=0;boot_requested<=0;end
        else begin
            if(ddr_cal_done&&ddr_cal_pass)flash_started<=1;
            if(boot_request)boot_requested<=1;
        end
    end
    d64_camera_frame_store #(.FAULT_STICKY(1),.ADMISSION_ENABLE(1),.W(OUT_W),.H(OUT_H)) u_camera_store(
        .clk(clk_pe),.rst_n(fabric_arst_n),.frame_bad(current_camera_bad),.in_valid(cap_valid),.in_ready(cap_ready),
        .admit_frame(GUIDE_ENABLE==0||source_guide_allocated),
        .in_rgb(cap_rgb888),.in_frame_id(cap_frame_id),.in_sof(cap_sof),.in_eol(cap_eol),.in_eof(cap_eof),
        .in_x(cap_x0),.in_y(cap_y),.frame_available(camera_available),.read_start(camera_read_start),
        .ready_frame_valid(camera_ready_valid),.ready_frame_id(camera_ready_fid),
        .read_done(camera_read_done),.read_failed(camera_read_failed),.out_valid(d64_valid),.out_ready(d64_ready),
        .out_rgb(d64_rgb888),.out_frame_id(d64_frame_id),.out_sof(d64_sof),.out_eol(d64_eol),.out_eof(d64_eof),.out_x(d64_x0),.out_y(d64_y),
        .awaddr(m_cmd[7][CAWADDR+:32]),.awlen(m_cmd[7][CAWLEN+:8]),.awsize(m_cmd[7][CAWSIZE+:3]),
        .awburst(m_cmd[7][CAWBURST+:2]),.awvalid(m_cmd[7][CAWVALID]),.awready(m_rsp[7][RAWREADY]),
        .wdata(m_cmd[7][CWVDATA+:128]),.wstrb(m_cmd[7][CWVSTRB+:16]),.wlast(m_cmd[7][CWVLAST]),
        .wvalid(m_cmd[7][CWVVALID]),.wready(m_rsp[7][RWREDY]),.bvalid(m_rsp[7][RBVALID]),
        .bready(m_cmd[7][CBREDY]),.bresp(m_rsp[7][RBRESPO+:2]),
        .araddr(m_cmd[7][CARADDR+:32]),.arlen(m_cmd[7][CARLEN+:8]),.arsize(m_cmd[7][CARSIZE+:3]),
        .arburst(m_cmd[7][CARBURST+:2]),.arvalid(m_cmd[7][CARVALID]),.arready(m_rsp[7][RARREADY]),
        .rdata(m_rsp[7][RRDATA+:128]),.rvalid(m_rsp[7][RRVALID]),.rready(m_cmd[7][CRREDY]),
        .rlast(m_rsp[7][RRLAST]),.rresp(m_rsp[7][RBRESPR+:2]),
        .captured_frames(captured_frames),.dropped_frames(dropped_frames),.axi_errors(capture_axi_errors),.protocol_errors(capture_protocol_errors));

    generate if(GUIDE_ENABLE!=0)begin:g_guide_capture
        wire gcap,gcommit,gfailed;
        wire [2:0] active_slot,committed_slot,grant_slot;
        wire [31:0] active_fid,committed_fid;
        wire grant_valid,g_overflow,g_protocol,g_axi;
        wire [31:0] lease_drop,lease_protocol;
        wire first=guide_scalar_valid&&guide_scalar_x==0&&guide_scalar_y==0;
        wire accepted_start;
        reg capture_ticket;
        assign int_grant_valid=grant_valid;assign int_grant_slot=grant_slot;
        assign int_guide_allocate=accepted_start;assign int_guide_commit=gcommit&&capture_ticket;
        assign int_guide_failed=gfailed&&capture_ticket;assign int_guide_commit_slot=committed_slot;
        assign int_guide_commit_fid=committed_fid;
        always @(posedge clk_pe or negedge fabric_arst_n)
            if(!fabric_arst_n)capture_ticket<=0;
            else begin
                if(gcommit||gfailed)capture_ticket<=0;
                if(accepted_start)capture_ticket<=1;
            end
        wire publish_commit=(job_state==J_RUN)&&core_done&&!core_failed&&!camera_read_failed&&
            !packed_bad_sync&&!block_jobs&&!user_request&&(job_epoch==active_epoch)&&job_guided_ok;
        wire release_job=(job_state==J_RUN)&&
            (core_failed||camera_read_failed||packed_bad_sync||(core_done&&!publish_commit));
        wire display_index=(display_base==32'h00600000);
        wire [1:0] protected_results=(display_valid?(display_index?2'b10:2'b01):2'b00)|
            ((job_state==J_PUBLISH)?(job_slot?2'b10:2'b01):2'b00);
        d64_guide_slot_manager #(.EXTERNAL_PROTECTION(1),.SOURCE_QUERY_ENABLE(1)) u_leases(.clk(clk_pe),.rst_n(fabric_arst_n),
            .source_first(first),.source_fid(guide_scalar_fid),.capture_slot_valid(grant_valid),.capture_slot(grant_slot),
            .capture_started(accepted_start),.capture_started_slot(grant_slot),.capture_started_fid(guide_scalar_fid),
            .guide_committed(gcommit&&capture_ticket),.guide_committed_slot(committed_slot),.guide_committed_fid(committed_fid),
            .guide_failed(gfailed&&capture_ticket),.guide_failed_slot(committed_slot),.guide_failed_fid(committed_fid),
            .camera_ready_valid(camera_ready_valid),.camera_ready_fid(camera_ready_fid),
            .source_query_fid(cap_frame_id),.source_query_allocated(source_guide_allocated),
            .camera_ready_guide_ready(camera_guide_ready),
            .camera_claim(camera_read_start),.camera_claim_fid(camera_ready_fid),.job_release(release_job),
            .publish_commit(publish_commit),.publish_output_slot(job_slot),
            .retire_output_slot(camera_read_start),.retire_output_index(job_slot),.result_protect_mask(protected_results),.external_protected_slots(int_protected_slots),
            .display_valid(display_valid),.display_output_slot(display_index),
            .reader_outstanding(guide_reader_outstanding),.reader_guide_slot(guide_reader_slot),
            .job_guide_slot_valid(job_guide_slot_valid),.job_guide_slot(),.job_guide_fid(job_guide_fid),.job_guide_ready(job_guide_ready),
            .published_guide_valid(guide_result_valid),.published_guide_slot0(guide_result_slot0),
            .published_guide_slot1(guide_result_slot1),.published_guide_fid0(),.published_guide_fid1(),
            .protected_slots(),.committed_slots(),.drop_no_slot(lease_drop),.protocol_fault(lease_protocol));
        // Lack of a free protected slot is a controlled whole-frame drop.
        // Protocol/capture/AXI failures still feed the aggregate fault output.
        assign guide_lease_fault=(lease_protocol!=0);
        d64_raw1080_guide_store_quad #(.W(CAM_W),.H(CAM_PACKED!=0?CAM_H:CAM_H-1),
            .FIFO_AW(10),.BURST_BEATS(16),.SOURCE_COMPLETE(1)) u_writer(
            .clk(clk_pe),.rst_n(fabric_arst_n),.in_valid(guide_scalar_valid),.in_g(guide_scalar_gray),
            .in_x(guide_scalar_x),.in_y(guide_scalar_y),.in_fid(guide_scalar_fid),
            .frame_bad(guide_scalar_fault||packed_bad_sync),.frame_slot_valid(grant_valid),.frame_slot(grant_slot),
            .capture_started(accepted_start),
            .capturing(gcap),.active_slot(active_slot),.active_fid(active_fid),
            .committed(gcommit),.failed(gfailed),.committed_slot(committed_slot),.committed_fid(committed_fid),
            .overflow_fault(g_overflow),.protocol_fault(g_protocol),.axi_fault(g_axi),
            .awaddr(guide_wr_cmd[CAWADDR+:32]),.awlen(guide_wr_cmd[CAWLEN+:8]),.awsize(guide_wr_cmd[CAWSIZE+:3]),
            .awburst(guide_wr_cmd[CAWBURST+:2]),.awvalid(guide_wr_cmd[CAWVALID]),.awready(m_rsp[9][RAWREADY]),
            .wdata(guide_wr_cmd[CWVDATA+:128]),.wstrb(guide_wr_cmd[CWVSTRB+:16]),.wlast(guide_wr_cmd[CWVLAST]),
            .wvalid(guide_wr_cmd[CWVVALID]),.wready(m_rsp[9][RWREDY]),.bvalid(m_rsp[9][RBVALID]),
            .bready(guide_wr_cmd[CBREDY]),.bresp(m_rsp[9][RBRESPO+:2]));
        assign guide_capture_fault=g_overflow||g_protocol||g_axi;
    end else begin:g_no_guide_capture
        assign int_grant_valid=0;assign int_grant_slot=0;assign int_guide_allocate=0;
        assign int_guide_commit=0;assign int_guide_failed=0;assign int_guide_commit_slot=0;assign int_guide_commit_fid=0;
        assign source_guide_allocated=1;assign camera_guide_ready=1;
        assign job_guide_slot_valid=0;assign job_guide_ready=0;assign job_guide_fid=0;
        assign guide_result_valid=0;assign guide_result_slot0=0;assign guide_result_slot1=0;
        assign guide_wr_cmd[CAWADDR+:32]=0;assign guide_wr_cmd[CAWLEN+:8]=0;
        assign guide_wr_cmd[CAWSIZE+:3]=4;assign guide_wr_cmd[CAWBURST+:2]=1;
        assign guide_wr_cmd[CAWVALID]=0;assign guide_wr_cmd[CWVDATA+:128]=0;
        assign guide_wr_cmd[CWVSTRB+:16]=0;assign guide_wr_cmd[CWVLAST]=0;
        assign guide_wr_cmd[CWVVALID]=0;assign guide_wr_cmd[CBREDY]=1;
        assign guide_lease_fault=0;assign guide_capture_fault=0;
    end endgenerate
    assign m_cmd[3][CAWADDR+:32]=tail_address+(job_slot?32'h00040000:0);
    assign ldr_we=cfg_we && cfg_index>=56 && cfg_index<376;
    assign ldr_wa=cfg_index-56;
    assign ldr_dout=cfg_data;
    always @(posedge clk_pe or negedge rst_pe_n)begin
        if(!rst_pe_n)begin raw_drop_base<=0;raw_short_base<=0;end
        else if(cam_raw_valid&&cam_sof)begin raw_drop_base<=cam_dropped;raw_short_base<=cam_short_lines;end
    end
    always @(posedge clk_pe or negedge rst_pe_n)begin
        if(!rst_pe_n)begin
            job_state<=J_IDLE;compute_release<=1;job_start<=0;camera_read_start<=0;
            job_wait<=0;job_slot<=0;job_epoch<=0;job_style<=1;job_frame_id<=0;
            publish_valid<=0;publish_base<=32'h005c0000;publish_fid<=0;
        end else begin
            job_start<=0;camera_read_start<=0;
            if(publish_valid&&publish_ready)publish_valid<=0;
            case(job_state)
                J_IDLE:if(model_valid&&!block_jobs&&!load_busy&&camera_available&&core_idle&&
                    (GUIDE_ENABLE==0||camera_guide_ready))begin
                    compute_release<=0;job_wait<=0;job_state<=J_ASSERT;
                end
                J_ASSERT:if(job_wait==3)begin compute_release<=1;job_wait<=0;job_state<=J_WARM;end
                         else job_wait<=job_wait+1;
                J_WARM:if(job_wait==31)begin
                    if(block_jobs||!model_valid)job_state<=J_IDLE;
                    else if(core_idle&&camera_available&&(GUIDE_ENABLE==0||camera_guide_ready))begin
                        job_start<=1;camera_read_start<=1;job_epoch<=active_epoch;job_style<=active_style;
                        job_slot<=(display_base==32'h005c0000);job_state<=J_RUN;
                    end
                end else job_wait<=job_wait+1;
                J_RUN:begin
                    if(d64_valid&&d64_ready&&d64_sof)job_frame_id<=d64_frame_id;
                    if(core_failed||camera_read_failed||packed_bad_sync)job_state<=J_FAULT;
                    else if(core_done)begin
                        if(block_jobs||user_request||job_epoch!=active_epoch||!job_guided_ok)job_state<=J_IDLE;
                        else begin
                            publish_base<=job_slot?32'h00600000:32'h005c0000;
                            publish_fid<={job_epoch,job_frame_id[15:0]};publish_valid<=1;job_state<=J_PUBLISH;
                        end
                    end
                end
                J_PUBLISH:if(!publish_valid&&publish_ready&&display_valid&&display_fid==publish_fid)job_state<=J_IDLE;
                J_FAULT:begin end
                default:job_state<=J_FAULT;
            endcase
        end
    end


    // INTERACTION_EXTENSION_INTEGRATED
    d64_interaction_extension #(.VOICE_ENABLE(VOICE_ENABLE),.GESTURE_ENABLE(GESTURE_ENABLE),
        .COMPARISON_ENABLE(COMPARISON_ENABLE),.PREVIEW_ENABLE(PREVIEW_ENABLE),.REMOTE_DEBUG_ENABLE(REMOTE_DEBUG_ENABLE)) u_interaction(
        .clk_pe(clk_pe),.rst_pe_n(fabric_arst_n),.clk_pix(hdmi_tx_slow_clk),.rst_pix_n(rst_disp_n),.frame_tick(display_frame),
        .comparison_publication_window(comparison_publication_window),
        .spi_sclk(spi_preview_sclk),.spi_cs_n(spi_preview_cs_n),.spi_mosi(spi_preview_mosi),
        .spi_miso(spi_preview_miso),.spi_ready(spi_preview_ready),
        .compare_toggle(compare_toggle),.effect_trigger(effect_trigger),.compare_enabled_pe(int_compare_enabled),.effect_enabled_pe(int_effect_enabled),
        .display_mode_pe(int_display_mode),.original_query_valid(int_original_query_valid),
        .remote_status(remote_status),.remote_active_style(active_style),.remote_active_epoch(active_epoch),
        .remote_fps_set_valid(remote_fps_set_valid),.remote_fps_set_value(remote_fps_set_value),
        .original_query_slot(job_slot?guide_result_slot1:guide_result_slot0),.original_query_fid(job_frame_id),
        .external_style_valid(voice_request_valid),.external_next(voice_request_next),.external_style(voice_request_style),
        .cap_valid(cap_valid),.cap_ready(cap_ready),.cap_sof(cap_sof),.cap_eof(cap_eof),.cap_rgb(cap_rgb888),
        .cap_x0(cap_x0),.cap_y(cap_y),.cap_fid(cap_frame_id),.source_bad(current_camera_bad||guide_scalar_fault||packed_bad_sync),
        .original_valid(int_original_valid),.original_rgb565(int_original_rgb565),.original_x(int_original_x),.original_y(int_original_y),
        .original_fid(int_original_fid),.original_guide_valid(guide_scalar_valid),.grant_valid(int_grant_valid),.grant_slot(int_grant_slot),
        .guide_allocate(int_guide_allocate),.guide_commit(int_guide_commit),.guide_failed(int_guide_failed),
        .guide_commit_slot(int_guide_commit_slot),.guide_commit_fid(int_guide_commit_fid),
        .full_render_de(int_render_de),.full_render_good(int_render_good),.full_render_rgb(int_render_rgb),
        .full_render_ax(int_render_ax),.full_render_ay(int_render_ay),.full_render_fid(int_render_fid),
        .full_render_epoch(int_render_epoch),.full_render_guide_slot(int_render_slot),.full_render_slot_valid(int_render_slot_valid),
        .publish_event(publish_valid&&publish_ready),.publish_base(publish_base),.publish_fid(job_frame_id),
        .publish_epoch(job_epoch),.publish_style(job_style),.processing_fps(int_processing_fps),
        .overwrite_event(camera_read_start),.overwrite_base(job_slot?32'h00600000:32'h005c0000),
        .comparison_publish_valid(int_comp_publish),.comparison_publish_ready(int_comp_ready),
        .comparison_original_base(int_comp_original_base),.comparison_half_base(int_comp_half_base),
        .comparison_fid(int_comp_fid),.comparison_epoch(int_comp_epoch),.comparison_slot(int_comp_slot),
        // This port is serviced by u_hdmi_priority, outside the low-priority
        // preview/stores tree. The extension still owns frame/slot admission.
        .comparison_protect_mask_pe(int_comparison_protect),.comparison_cmd(240'd0),.comparison_rsp(),
        .protected_slots(int_protected_slots),.extra_cmd(int_extra_cmd),.extra_rsp(int_extra_rsp),
        .video_fault(int_video_fault),.preview_fault(int_preview_fault),.voice_fault(int_voice_fault));

endmodule
