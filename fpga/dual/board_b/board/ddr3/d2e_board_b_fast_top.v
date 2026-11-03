`timescale 1ns/1ps
// Product B: frozen compute + Ethernet. DDR/AXI/reset inherited from B367.
module d2e_board_b_fast_top(
  (* syn_peri_port = 0 *) input pll_inst1_CLKOUT4,
  (* syn_peri_port = 0 *) input pll1_lock,
  (* syn_peri_port = 0 *) input pll_lock,
  (* syn_peri_port = 0 *) input clk_weight_fast,
  (* syn_peri_port = 0 *) input clk,
  (* syn_peri_port = 0 *) input sys_clk_50mhz,
  (* syn_peri_port = 0 *) input ddr_core_clk,
  (* syn_peri_port = 0 *) input ddr_tdqss_clk,
  (* syn_peri_port = 0 *) input ddr_tac_clk,
  (* syn_peri_port = 0 *) input clk_25m,
  (* syn_peri_port = 0 *) input ddr_twd_clk,
  (* syn_peri_port = 0 *) input [15:0] i_ddr_dq_hi,
  (* syn_peri_port = 0 *) input [15:0] i_ddr_dq_lo,
  (* syn_peri_port = 0 *) input [1:0] i_ddr_dqs_hi,
  (* syn_peri_port = 0 *) input [1:0] i_ddr_dqs_lo,
  (* syn_peri_port = 0 *) input [3:0] i_spi0_io,
  (* syn_peri_port = 0 *) output txd,
  (* syn_peri_port = 0 *) output ddr_pll_rstn,
  (* syn_peri_port = 0 *) output [2:0] ddr_shift,
  (* syn_peri_port = 0 *) output ddr_shift_ena,
  (* syn_peri_port = 0 *) output [4:0] ddr_shift_sel,
  (* syn_peri_port = 0 *) output sys_pll_rstn,
  (* syn_peri_port = 0 *) output [15:0] ddr_addr,
  (* syn_peri_port = 0 *) output [2:0] ddr_ba,
  (* syn_peri_port = 0 *) output ddr_cas,
  (* syn_peri_port = 0 *) output ddr_cke,
  (* syn_peri_port = 0 *) output ddr_cs,
  (* syn_peri_port = 0 *) output [1:0] o_ddr_dm_hi,
  (* syn_peri_port = 0 *) output [1:0] o_ddr_dm_lo,
  (* syn_peri_port = 0 *) output [15:0] o_ddr_dq_hi,
  (* syn_peri_port = 0 *) output [15:0] o_ddr_dq_lo,
  (* syn_peri_port = 0 *) output [15:0] o_ddr_dq_oe,
  (* syn_peri_port = 0 *) output [1:0] o_ddr_dqs_hi,
  (* syn_peri_port = 0 *) output [1:0] o_ddr_dqs_lo,
  (* syn_peri_port = 0 *) output [1:0] o_ddr_dqs_oe,
  (* syn_peri_port = 0 *) output [1:0] o_ddr_dqs_n_oe,
  (* syn_peri_port = 0 *) output ddr_odt,
  (* syn_peri_port = 0 *) output ddr_ras,
  (* syn_peri_port = 0 *) output ddr_reset,
  (* syn_peri_port = 0 *) output ddr_we,
  (* syn_peri_port = 0 *) output [3:0] led,
  (* syn_peri_port = 0 *) output o_spi0_cs,
  (* syn_peri_port = 0 *) output oe_spi0_cs,
  (* syn_peri_port = 0 *) output [3:0] o_spi0_io,
  (* syn_peri_port = 0 *) output [3:0] oe_spi0_io,
  (* syn_peri_port = 0 *) output o_spi_clk,
  (* syn_peri_port = 0 *) output oe_spi_clk,
  (* syn_peri_port = 0 *) input tx_clk,
  (* syn_peri_port = 0 *) input rxc,
  (* syn_peri_port = 0 *) input rx_ref_clk,
  (* syn_peri_port = 0 *) input rx_sample_clk,
  (* syn_peri_port = 0 *) input rx_pll_LOCKED,
  (* syn_peri_port = 0 *) input [3:0] rxd_hi_i,
  (* syn_peri_port = 0 *) input [3:0] rxd_lo_i,
  (* syn_peri_port = 0 *) input rx_dv_HI,
  (* syn_peri_port = 0 *) input rx_dv_LO,
  (* syn_peri_port = 0 *) input mdio_i,
  (* syn_peri_port = 0 *) output [3:0] txd_hi_o,
  (* syn_peri_port = 0 *) output [3:0] txd_lo_o,
  (* syn_peri_port = 0 *) output tx_en_o_HI,
  (* syn_peri_port = 0 *) output tx_en_o_LO,
  (* syn_peri_port = 0 *) output txc_hi_o,
  (* syn_peri_port = 0 *) output txc_lo_o,
  (* syn_peri_port = 0 *) output mdc_o,
  (* syn_peri_port = 0 *) output mdio_o,
  (* syn_peri_port = 0 *) output mdio_oe,
  (* syn_peri_port = 0 *) output phy_rst_n,
  (* syn_peri_port = 0 *) output rx_pll_RSTN
);
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


    assign rx_pll_RSTN=1'b1;
    wire fabric_arst_n;
    wire [CMD_W-1:0] br_cmd;
    wire [RSP_W-1:0] br_rsp;
    wire [1:0] o_ddr_dqs_n_hi,o_ddr_dqs_n_lo;
    assign ddr_pll_rstn = 1'b1;
    assign sys_pll_rstn = 1'b1;

    reg [3:0] por = 4'd0;
    always @(posedge clk_25m) if (!(&por)) por <= por + 4'd1;

    reg [1:0] pll_locked_s = 2'b00;
    wire plls_locked = pll_lock && pll1_lock;
    always @(posedge clk_25m)
        pll_locked_s <= {pll_locked_s[0], plls_locked};

    reg [7:0] lock_hold = 8'd0;
    always @(posedge clk_25m) begin
        if (!(&por) || !pll_locked_s[1]) lock_hold <= 8'd0;
        else if (!(&lock_hold))          lock_hold <= lock_hold + 8'd1;
    end

    reg started_q = 1'b0;
    always @(posedge clk_25m) started_q <= (&por) & (&lock_hold);

    wire rst_sw    = ~started_q;
    wire rst_axi_n = ~rst_sw;

    // Dedicated DDR release source; global/PE/display reset paths are intact.
    wire rst_ddr_n;
    d64_ddr_clk25_release u_ddr_release (
        .clk_25m(clk_25m),.global_rst_n(rst_axi_n),.ddr_release_n(rst_ddr_n));


    d64_reset_sync u_app_reset (.clk(clk), .arst_n(rst_axi_n), .rst_n(fabric_arst_n));

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
        .i_dqs_n_hi(2'b00), .i_dqs_n_lo(2'b00),
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
        .clk(clk), .rst_n(fabric_arst_n),
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


    wire [7:0] app_tx_data,app_rx_data;
    wire app_tx_valid,app_tx_last,app_tx_ready,app_rx_valid,app_rx_last,app_rx_ready;
    wire [10:0] app_rx_length;wire [47:0] app_rx_source;
    wire qualified_app,app_fault;
    wire [415:0] eth_status_words;wire [31:0] eth_config_status;
    wire [15:0] eth_config_before,eth_config_desired,eth_config_readback;
    wire [255:0] eth_mac_status;wire [1:0] eth_mac_fresh;
    wire [31:0] eth_tx_queued,eth_tx_oversize,eth_rx_delivered;
    d2e_ethernet_port #(.ROLE(1))u_ethernet(
      .clk_25m(clk_25m),.app_clk(clk),.tx_clk(tx_clk),.rx_sample_clk(rx_sample_clk),
      .rx_pll_LOCKED(rx_pll_LOCKED),.arst_n(fabric_arst_n),
      .rxd_hi_i(rxd_hi_i),.rxd_lo_i(rxd_lo_i),.rx_dv_HI(rx_dv_HI),.rx_dv_LO(rx_dv_LO),.mdio_i(mdio_i),
      .txd_hi_o(txd_hi_o),.txd_lo_o(txd_lo_o),.tx_en_o_HI(tx_en_o_HI),.tx_en_o_LO(tx_en_o_LO),
      .txc_hi_o(txc_hi_o),.txc_lo_o(txc_lo_o),.mdc_o(mdc_o),.mdio_o(mdio_o),.mdio_oe(mdio_oe),.phy_rst_n(phy_rst_n),
      .s_tx_data(app_tx_data),.s_tx_valid(app_tx_valid),.s_tx_ready(app_tx_ready),.s_tx_last(app_tx_last),
      .m_rx_data(app_rx_data),.m_rx_valid(app_rx_valid),.m_rx_ready(app_rx_ready),.m_rx_last(app_rx_last),.m_rx_length(app_rx_length),.m_rx_src_mac(app_rx_source),
      .qualified_app(qualified_app),.status_words(eth_status_words),.config_status(eth_config_status),
      .config_before(eth_config_before),.config_desired(eth_config_desired),.config_readback(eth_config_readback),
      .stats_request(1'b0),.mac_status_words(eth_mac_status),.mac_status_fresh(eth_mac_fresh),
      .app_tx_queued(eth_tx_queued),.app_tx_oversize(eth_tx_oversize),.app_rx_delivered(eth_rx_delivered));
    d2e_b_application_fast u_application(
      .clk(clk),.clk_fast(clk_weight_fast),.rst_n(fabric_arst_n),.ddr_ready(ddr_cal_done&&ddr_cal_pass),.phy_qualified(qualified_app),
      .phy_status_words(eth_status_words),.phy_config_status(eth_config_status),.phy_config_before(eth_config_before),.phy_config_desired(eth_config_desired),.phy_config_readback(eth_config_readback),
      .mac_tx_data(app_tx_data),.mac_tx_valid(app_tx_valid),.mac_tx_ready(app_tx_ready),.mac_tx_last(app_tx_last),
      .mac_rx_data(app_rx_data),.mac_rx_valid(app_rx_valid),.mac_rx_ready(app_rx_ready),.mac_rx_last(app_rx_last),.mac_rx_length(app_rx_length),
      .i_spi0_io(i_spi0_io),.o_spi0_io(o_spi0_io),.oe_spi0_io(oe_spi0_io),.o_spi_clk(o_spi_clk),.oe_spi_clk(oe_spi_clk),.o_spi0_cs(o_spi0_cs),.oe_spi0_cs(oe_spi0_cs),
      .cmd(br_cmd),.rsp(br_rsp),.txd(txd),.fault(app_fault));
    assign led={app_fault,qualified_app,ddr_cal_pass,ddr_cal_done};
endmodule
