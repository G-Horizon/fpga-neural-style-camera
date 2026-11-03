`timescale 1ns/1ps
`default_nettype none
// Independent full-native-resolution camera-to-HDMI diagnostic top.
module d2_board_a_top (
    input wire clk_25m,
    input wire clk_weight_fast,
    input wire camera_pixel_clk,
    input wire pll_lock,
    input wire pll1_lock,
    output wire ddr_pll_rstn,
    output wire sys_pll_rstn,
    input wire hdmi_tx_slow_clk,
    input wire hdmi_tx_fast_clk,
    input wire clk,
    input wire ddr_core_clk,
    input wire ddr_twd_clk,
    input wire ddr_tac_clk,
    input wire ddr_tdqss_clk,
    output wire ddr_reset,
    output wire ddr_cs,
    output wire ddr_ras,
    output wire ddr_cas,
    output wire ddr_we,
    output wire ddr_cke,
    output wire [15:0] ddr_addr,
    output wire [2:0] ddr_ba,
    output wire ddr_odt,
    output wire [1:0] o_ddr_dm_hi,
    output wire [1:0] o_ddr_dm_lo,
    input wire [1:0] i_ddr_dqs_hi,
    input wire [1:0] i_ddr_dqs_lo,
    input wire [1:0] i_ddr_dqs_n_hi,
    input wire [1:0] i_ddr_dqs_n_lo,
    output wire [1:0] o_ddr_dqs_hi,
    output wire [1:0] o_ddr_dqs_lo,
    output wire [1:0] o_ddr_dqs_oe,
    output wire [1:0] o_ddr_dqs_n_oe,
    input wire [15:0] i_ddr_dq_hi,
    input wire [15:0] i_ddr_dq_lo,
    output wire [15:0] o_ddr_dq_hi,
    output wire [15:0] o_ddr_dq_lo,
    output wire [15:0] o_ddr_dq_oe,
    output wire [2:0] ddr_shift,
    output wire [4:0] ddr_shift_sel,
    output wire ddr_shift_ena,
    input wire verf0,
    input wire verf1,
    output wire [9:0] tmds_data0_o,
    output wire [9:0] tmds_data1_o,
    output wire [9:0] tmds_data2_o,
    output wire [9:0] tmds_clk_o,
    output wire tmds_data0_TX_OE,
    output wire tmds_data1_TX_OE,
    output wire tmds_data2_TX_OE,
    output wire tmds_clk_TX_OE,
    output wire tmds_data0_TX_RST,
    output wire tmds_data1_TX_RST,
    output wire tmds_data2_TX_RST,
    output wire tmds_clk_TX_RST,
    output wire [3:0] led,
    output wire txd,
    output wire HPD_N,
    input wire [3:0] keys_n,
    input wire s3_uart_rx,output wire s3_uart_tx,
    input wire [3:0] i_spi0_io,
    output wire [3:0] o_spi0_io,
    output wire [3:0] oe_spi0_io,
    output wire o_spi_clk,
    output wire oe_spi_clk,
    output wire o_spi0_cs,
    output wire oe_spi0_cs,
    input wire io_cam_scl_IN,
    input wire io_cam_sda_IN,
    input wire i_cam_ck_CLKOUT,
    input wire i_cam_ck_LP_N_IN,
    input wire i_cam_ck_LP_P_IN,
    input wire cam_d2_FIFO_EMPTY,
    input wire [7:0] cam_d2_HS_IN,
    input wire cam_d2_LP_N_IN,
    input wire cam_d2_LP_P_IN,
    input wire cam_d3_FIFO_EMPTY,
    input wire [7:0] cam_d3_HS_IN,
    input wire cam_d3_LP_N_IN,
    input wire cam_d3_LP_P_IN,
    input wire cam_d0_FIFO_EMPTY,
    input wire [7:0] cam_d0_HS_IN,
    input wire cam_d0_LP_N_IN,
    input wire cam_d0_LP_P_IN,
    input wire cam_d1_FIFO_EMPTY,
    input wire [7:0] cam_d1_HS_IN,
    input wire cam_d1_LP_N_IN,
    input wire cam_d1_LP_P_IN,
    output wire io_cam_scl_OUT,
    output wire io_cam_scl_OE,
    output wire io_cam_sda_OUT,
    output wire io_cam_sda_OE,
    output wire o_cam_rst_p,
    output wire o_cam_ck_HS_ENA,
    output wire o_cam_ck_HS_TERM,
    output wire cam_d2_FIFO_RD,
    output wire cam_d2_HS_ENA,
    output wire cam_d2_HS_TERM,
    output wire cam_d2_RST,
    output wire cam_d3_FIFO_RD,
    output wire cam_d3_HS_ENA,
    output wire cam_d3_HS_TERM,
    output wire cam_d3_RST,
    output wire cam_d0_FIFO_RD,
    output wire cam_d0_HS_ENA,
    output wire cam_d0_HS_TERM,
    output wire cam_d0_RST,
    output wire cam_d1_FIFO_RD,
    output wire cam_d1_HS_ENA,
    output wire cam_d1_HS_TERM,
    output wire cam_d1_RST,
    input wire sys_clk_50mhz,
    input wire hdmi_tx_locked
);
    wire global_reset_n;
    wire ddr_cal_done,ddr_cal_pass;

    assign global_reset_n=pll_lock&&pll1_lock&&hdmi_tx_locked;
    wire camera_reset_n=global_reset_n;
    wire camera_accepting;
    wire rx_pix_valid_o;
    wire [63:0] rx_pix_data_o;
    wire [3:0] rx_pix_per_clk_o;
    wire rx_hsync_o;
    wire rx_vsync_o;
    wire rst_pix_n_o;
    wire config_done;
    wire config_error;
    wire [31:0] sensor_readback;
    wire [127:0] csi_status;wire [191:0] sensor_registers;wire [63:0] sensor_i2c_status;
    wire [5:0] rx_datatype_o;
    wire rx_irq_o;
    wire [15:0] rx_word_count_o;
    wire [15:0] rx_shortpkt_o;
    wire [1:0] rx_vc_o;
    wire raw_valid,raw_sof,raw_sol,raw_eol,raw_eof;
    wire[9:0] raw10;wire[31:0] raw_frame_id;
    wire raw_overflow,raw_ppc_bad;
    wire packed_valid,packed_sof,packed_sol,packed_eol,packed_eof;
    wire [39:0] packed_raw4;
    wire [31:0] packed_fid;
    wire packed_bad;
    wire official_adapter_bad;wire [31:0] official_good_frames,official_bad_frames;
    wire [359:0] bridge_byte_live;wire [7:0] bridge_fatal_code;
    reg [31:0] bridge_eof_count,bridge_word_count;

    wire [63:0] ppi_trace_sample;
    d64_camera_frontend_517 u_camera(
        .ppi_trace_o(ppi_trace_sample),
        .i_arstn(camera_reset_n),
        .CLK_25M(clk_25m),
        .core_clk(clk),
        .i_mipi_rx_pclk(camera_pixel_clk),
        .io_cam_scl_IN(io_cam_scl_IN),
        .io_cam_sda_IN(io_cam_sda_IN),
        .i_cam_ck_CLKOUT(i_cam_ck_CLKOUT),
        .i_cam_ck_LP_N_IN(i_cam_ck_LP_N_IN),
        .i_cam_ck_LP_P_IN(i_cam_ck_LP_P_IN),
        .cam_d2_FIFO_EMPTY(cam_d2_FIFO_EMPTY),
        .cam_d2_HS_IN(cam_d2_HS_IN),
        .cam_d2_LP_N_IN(cam_d2_LP_N_IN),
        .cam_d2_LP_P_IN(cam_d2_LP_P_IN),
        .cam_d3_FIFO_EMPTY(cam_d3_FIFO_EMPTY),
        .cam_d3_HS_IN(cam_d3_HS_IN),
        .cam_d3_LP_N_IN(cam_d3_LP_N_IN),
        .cam_d3_LP_P_IN(cam_d3_LP_P_IN),
        .cam_d0_FIFO_EMPTY(cam_d0_FIFO_EMPTY),
        .cam_d0_HS_IN(cam_d0_HS_IN),
        .cam_d0_LP_N_IN(cam_d0_LP_N_IN),
        .cam_d0_LP_P_IN(cam_d0_LP_P_IN),
        .cam_d1_FIFO_EMPTY(cam_d1_FIFO_EMPTY),
        .cam_d1_HS_IN(cam_d1_HS_IN),
        .cam_d1_LP_N_IN(cam_d1_LP_N_IN),
        .cam_d1_LP_P_IN(cam_d1_LP_P_IN),
        .io_cam_scl_OUT(io_cam_scl_OUT),
        .io_cam_scl_OE(io_cam_scl_OE),
        .io_cam_sda_OUT(io_cam_sda_OUT),
        .io_cam_sda_OE(io_cam_sda_OE),
        .o_cam_rst_p(o_cam_rst_p),
        .o_cam_ck_HS_ENA(o_cam_ck_HS_ENA),
        .o_cam_ck_HS_TERM(o_cam_ck_HS_TERM),
        .cam_d2_FIFO_RD(cam_d2_FIFO_RD),
        .cam_d2_HS_ENA(cam_d2_HS_ENA),
        .cam_d2_HS_TERM(cam_d2_HS_TERM),
        .cam_d2_RST(cam_d2_RST),
        .cam_d3_FIFO_RD(cam_d3_FIFO_RD),
        .cam_d3_HS_ENA(cam_d3_HS_ENA),
        .cam_d3_HS_TERM(cam_d3_HS_TERM),
        .cam_d3_RST(cam_d3_RST),
        .cam_d0_FIFO_RD(cam_d0_FIFO_RD),
        .cam_d0_HS_ENA(cam_d0_HS_ENA),
        .cam_d0_HS_TERM(cam_d0_HS_TERM),
        .cam_d0_RST(cam_d0_RST),
        .cam_d1_FIFO_RD(cam_d1_FIFO_RD),
        .cam_d1_HS_ENA(cam_d1_HS_ENA),
        .cam_d1_HS_TERM(cam_d1_HS_TERM),
        .cam_d1_RST(cam_d1_RST),
        .rx_pix_valid_o(rx_pix_valid_o),
        .rx_pix_data_o(rx_pix_data_o),
        .rx_pix_per_clk_o(rx_pix_per_clk_o),
        .rx_hsync_o(rx_hsync_o),
        .rx_vsync_o(rx_vsync_o),
        .rst_pix_n_o(rst_pix_n_o),
        .config_done(config_done),
        .config_error(config_error),
        .sensor_readback(sensor_readback),.csi_status(csi_status),.sensor_registers(sensor_registers),.i2c_status(sensor_i2c_status),
        .rx_datatype_o(rx_datatype_o),
        .rx_irq_o(rx_irq_o),
        .rx_word_count_o(rx_word_count_o),
        .rx_shortpkt_o(rx_shortpkt_o),
        .rx_vc_o(rx_vc_o)
    );

    wire rst_core_n,rst_display_n,rst_camera_n;
    d64_reset_sync u_core_reset(.clk(clk),.arst_n(global_reset_n),.rst_n(rst_core_n));
    d64_reset_sync u_pix_reset(.clk(hdmi_tx_slow_clk),.arst_n(global_reset_n),.rst_n(rst_display_n));
    wire rst_ddr_n;
    assign ddr_pll_rstn=1'b1;assign sys_pll_rstn=1'b1;
    assign HPD_N=1'b0;


    assign tmds_data0_TX_OE=1;assign tmds_data1_TX_OE=1;
    assign tmds_data2_TX_OE=1;assign tmds_clk_TX_OE=1;
    assign tmds_data0_TX_RST=!rst_display_n;assign tmds_data1_TX_RST=!rst_display_n;
    assign tmds_data2_TX_RST=!rst_display_n;assign tmds_clk_TX_RST=!rst_display_n;

    wire camera_ready,capture_camera_rst_n,system_core_rst_n,system_pix_rst_n;
    wire [1:0] ready_camera={2{camera_ready}};
    d2_a_reset_control u_reset_control(.clk_25m(clk_25m),.clk(clk),.cam_clk(camera_pixel_clk),.pix_clk(hdmi_tx_slow_clk),
      .global_reset_n(global_reset_n),.csi_reset_n(rst_pix_n_o),.config_done(config_done),.config_error(config_error),.ddr_cal_done(ddr_cal_done),.ddr_cal_pass(ddr_cal_pass),
      .rst_ddr_n(rst_ddr_n),.rst_camera_n(rst_camera_n),.capture_camera_rst_n(capture_camera_rst_n),.system_core_rst_n(system_core_rst_n),.system_pix_rst_n(system_pix_rst_n),.camera_ready(camera_ready));
    wire capture_release_n,capture_core_rst_n,capture_cam_rst_n;
    d64_reset_sync u_capture_core_reset(.clk(clk),.arst_n(system_core_rst_n&&capture_release_n),.rst_n(capture_core_rst_n));
    d64_reset_sync u_capture_cam_reset(.clk(camera_pixel_clk),.arst_n(capture_camera_rst_n&&capture_release_n),.rst_n(capture_cam_rst_n));
    reg [31:0] persistent_source_fid;
    always @(posedge camera_pixel_clk or negedge capture_camera_rst_n)
      if(!capture_camera_rst_n)persistent_source_fid<=0;
      else if(packed_valid&&packed_sof)persistent_source_fid<=persistent_source_fid+1'b1;
    wire [31:0] persistent_beat_fid=packed_sof?persistent_source_fid+1'b1:persistent_source_fid;
    wire rgb_valid,rgb_sof,rgb_sol,rgb_eol,rgb_eof,isp_bad;
    wire [95:0] rgb4;wire [10:0] rgb_x,rgb_y;wire [31:0] rgb_fid;
    camera_raw4_rgb888 u_isp(.clk(camera_pixel_clk),.rst_n(capture_cam_rst_n),
        .raw_valid(packed_valid),.raw4(packed_raw4),.raw_sof(packed_sof),.raw_sol(packed_sol),
        .raw_eol(packed_eol),.raw_eof(packed_eof),.raw_frame_id(persistent_beat_fid),
        .rgb4_valid(rgb_valid),.rgb4(rgb4),.rgb_x0(rgb_x),.rgb_y(rgb_y),.rgb_frame_id(rgb_fid),
        .rgb_sof(rgb_sof),.rgb_sol(rgb_sol),.rgb_eol(rgb_eol),.rgb_eof(rgb_eof),.fault_sticky(isp_bad));

    reg capture_run;
    always @(posedge camera_pixel_clk or negedge capture_cam_rst_n)
      if(!capture_cam_rst_n)capture_run<=0;else if(rgb_valid&&rgb_sof&&camera_ready)capture_run<=1;

 wire header_byte_rst_n;
 d64_reset_sync u_header_reset(.clk(i_cam_ck_CLKOUT),.arst_n(capture_cam_rst_n),.rst_n(header_byte_rst_n));
    validated_ppi_raw4_bridge u_ppi_bridge(
        .byte_clk(i_cam_ck_CLKOUT),.byte_rst_n(header_byte_rst_n),
        .ppi_data(ppi_trace_sample[31:0]),.ppi_valid(ppi_trace_sample[39:36]),
        .ppi_sync(ppi_trace_sample[43:40]),.ppi_stop(ppi_trace_sample[47:44]),.ppi_sot(ppi_trace_sample[51:48]),
        .pixel_clk(camera_pixel_clk),.pixel_rst_n(capture_cam_rst_n),
        .raw_valid(packed_valid),.raw4(packed_raw4),.raw_sof(packed_sof),.raw_sol(packed_sol),
        .raw_eol(packed_eol),.raw_eof(packed_eof),.raw_frame_id(packed_fid),
        .source_bad_sticky(packed_bad),.fatal_code(bridge_fatal_code),.emitted_rows(),.emitted_frames(),
        .accepted_rows(bridge_byte_live[31:0]),
        .accepted_frames(bridge_byte_live[63:32]),
        .startup_discard_packets(bridge_byte_live[95:64]),
        .buffer_overflows(bridge_byte_live[127:96]),
        .ecc_errors(bridge_byte_live[159:128]),
        .crc_errors(bridge_byte_live[191:160]),
        .geometry_errors(bridge_byte_live[223:192]),
        .boundary_errors(bridge_byte_live[255:224]),
        .short_packets(bridge_byte_live[287:256]),
        .missing_sync_transactions(bridge_byte_live[319:288]),
        .sequence_discontinuities(bridge_byte_live[351:320]),
        .fatal_code_byte(bridge_byte_live[359:352]),
        .sot_events(),.sync_events());

 wire [239:0] system_cmd;wire [137:0] system_rsp;
 wire [23:0] link_rgb;wire link_de,link_hs,link_vs,link_frame;wire system_fault;
 d64_video_timing #(.H_ACT(1920),.H_FP(88),.H_SYNC(44),.H_BP(148),.V_ACT(1080),.V_FP(4),.V_SYNC(5),.V_BP(36)) u_timing(
 .clk(hdmi_tx_slow_clk),.rst_n(rst_display_n),.hs(link_hs),.vs(link_vs),.de(link_de),.frame(link_frame),.x(),.y(),.ax(),.ay());
 d2_a_system u_system(.clk(clk),.clk_fast(clk_weight_fast),.rst_n(system_core_rst_n),.ddr_ready(ddr_cal_done&&ddr_cal_pass),
 .capture_release_n(capture_release_n),.capture_core_rst_n(capture_core_rst_n),.capture_cam_rst_n(capture_cam_rst_n),
 .cam_clk(camera_pixel_clk),.cam_rst_n(capture_camera_rst_n),.pix_clk(hdmi_tx_slow_clk),.pix_rst_n(system_pix_rst_n),
 .valid(rgb_valid&&ready_camera[1]&&(capture_run||rgb_sof)),.rgb4(rgb4),.x0(rgb_x),.y(rgb_y),.fid(rgb_fid),.sof(rgb_sof),.eof(rgb_eof),.source_bad(packed_bad||isp_bad),
 .frame_start(link_frame),.de(link_de),.keys_n(keys_n),.s3_uart_rx(s3_uart_rx),.s3_uart_tx(s3_uart_tx),.i_spi0_io(i_spi0_io),.o_spi0_io(o_spi0_io),.oe_spi0_io(oe_spi0_io),.o_spi_clk(o_spi_clk),.oe_spi_clk(oe_spi_clk),.o_spi0_cs(o_spi0_cs),.oe_spi0_cs(oe_spi0_cs),
 .diag_packed_bad(packed_bad),.diag_isp_bad(isp_bad),.diag_bridge_code(bridge_fatal_code),
 .rgb(link_rgb),.txd(txd),.fault(system_fault),.cmd(system_cmd),.rsp(system_rsp));
 reg [23:0] rgb_q;reg de_q,hs_q,vs_q;
 always @(posedge hdmi_tx_slow_clk or negedge rst_display_n)if(!rst_display_n)begin rgb_q<=0;de_q<=0;hs_q<=0;vs_q<=0;end else begin rgb_q<=link_rgb;de_q<=link_de;hs_q<=link_hs;vs_q<=link_vs;end
 wire [9:0] td0,td1,td2,tc;
 dvi_encoder u_tmds(.pixelclk(hdmi_tx_slow_clk),.rstin(!rst_display_n),.blue_din(rgb_q[7:0]),.green_din(rgb_q[15:8]),.red_din(rgb_q[23:16]),.hsync(hs_q),.vsync(vs_q),.de(de_q),.tmds_data0(td0),.tmds_data1(td1),.tmds_data2(td2),.tmds_clk(tc));
 assign tmds_data0_o=~td0;assign tmds_data1_o=~td1;assign tmds_data2_o=~td2;assign tmds_clk_o=~tc;
 assign led={!(system_fault||packed_bad||isp_bad),!config_error,config_done,!ddr_cal_pass};
 wire awvalid,awready,wvalid,wready,wlast,bvalid,bready;wire [31:0] awaddr;wire [7:0] awlen;wire [2:0] awsize;wire [1:0] awburst,bresp;wire [127:0] wdata;wire [15:0] wstrb;
 wire arvalid,arready,rvalid,rready,rlast;wire [31:0] araddr;wire [7:0] arlen;wire [2:0] arsize;wire [1:0] arburst,rresp;wire [127:0] rdata;
 assign awvalid=system_cmd[0];assign awaddr=system_cmd[32:1];assign awlen=system_cmd[40:33];assign awsize=system_cmd[43:41];assign awburst=system_cmd[45:44];
 assign wvalid=system_cmd[46];assign wdata=system_cmd[174:47];assign wstrb=system_cmd[190:175];assign wlast=system_cmd[191];assign bready=system_cmd[192];
 assign arvalid=system_cmd[193];assign araddr=system_cmd[225:194];assign arlen=system_cmd[233:226];assign arsize=system_cmd[236:234];assign arburst=system_cmd[238:237];assign rready=system_cmd[239];
 assign system_rsp={rresp,rlast,rdata,rvalid,arready,bresp,bvalid,wready,awready};
    wire [1:0] o_ddr_dqs_n_hi,o_ddr_dqs_n_lo;

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
        d64_ddr_ip_model #(.ADDR_W(25)) u_ddr3 (
            .clk(clk), .core_clk(ddr_core_clk), .twd_clk(ddr_twd_clk),
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
        .clk(clk), .core_clk(ddr_core_clk), .twd_clk(ddr_twd_clk),
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


    d64_efx_ddr_axi_bridge u_bridge(.clk(clk),.rst_n(system_core_rst_n),
        .m_awvalid(awvalid),.m_awready(awready),.m_awaddr(awaddr),.m_awlen(awlen),.m_awsize(awsize),.m_awburst(awburst),
        .m_arvalid(arvalid),.m_arready(arready),.m_araddr(araddr),.m_arlen(arlen),.m_arsize(arsize),.m_arburst(arburst),
        .ddr_aaddr(axi_aaddr),.ddr_alen(axi_alen),.ddr_asize(axi_asize),.ddr_aburst(axi_aburst),
        .ddr_avalid(axi_avalid),.ddr_aready(axi_aready),.ddr_atype(axi_atype),
        .m_wdata(wdata),.m_wstrb(wstrb),.m_wlast(wlast),.m_wvalid(wvalid),.m_wready(wready),
        .ddr_wdata(axi_wdata),.ddr_wstrb(axi_wstrb),.ddr_wlast(axi_wlast),.ddr_wvalid(axi_wvalid),.ddr_wready(axi_wready),
        .m_bvalid(bvalid),.m_bresp(bresp),.m_bready(bready),.ddr_bvalid(axi_bvalid),.ddr_bresp(axi_bresp),.ddr_bready(axi_bready),
        .m_rdata(rdata),.m_rlast(rlast),.m_rresp(rresp),.m_rvalid(rvalid),.m_rready(rready),
        .ddr_rdata(axi_rdata),.ddr_rlast(axi_rlast),.ddr_rresp(axi_rresp),.ddr_rvalid(axi_rvalid),.ddr_rready(axi_rready),
        .simultaneous_wait_cycles(),.write_address_accepts(),.read_address_accepts());
    assign axi_alock=0;
endmodule
`default_nettype wire

