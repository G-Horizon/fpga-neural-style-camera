`timescale 1ns/1ps
// Reusable product port. GPIO DDIO/PLL are periphery responsibilities.
// TXC uses the same125MHz phase as TXD; only the restricted PHY startup RMW
// supplies the intended GE TX delay. All bank owners share arst_n assertion.
module d2e_ethernet_port #(
 parameter integer ROLE=0,CLOCK_HZ=100000000,MDC_HZ=500000,
 parameter[47:0]LOCAL_MAC=(ROLE==0?48'h02544936300a:48'h02544936300b),
 parameter[47:0]PEER_MAC=(ROLE==0?48'h02544936300b:48'h02544936300a),
 parameter[15:0]ETHERTYPE=16'h88b5
)(input wire clk_25m,app_clk,tx_clk,rx_sample_clk,rx_pll_LOCKED,arst_n,
 input wire[3:0]rxd_hi_i,rxd_lo_i,input wire rx_dv_HI,rx_dv_LO,mdio_i,
 output wire[3:0]txd_hi_o,txd_lo_o,
 output wire tx_en_o_HI,tx_en_o_LO,txc_hi_o,txc_lo_o,
 output wire mdc_o,mdio_o,mdio_oe,phy_rst_n,
 input wire[7:0]s_tx_data,input wire s_tx_valid,s_tx_last,output wire s_tx_ready,
 output wire[7:0]m_rx_data,output wire m_rx_valid,m_rx_last,input wire m_rx_ready,
 output wire[10:0]m_rx_length,output wire[47:0]m_rx_src_mac,
 output reg qualified_app,output wire[415:0]status_words,
 output wire[31:0]config_status,output wire[15:0]config_before,config_desired,config_readback,
 input wire stats_request,output wire[255:0]mac_status_words,output wire[1:0]mac_status_fresh,
 output wire[31:0]app_tx_queued,app_tx_oversize,app_rx_delivered);
 wire app_rst_n,tx_rst_n,rx_rst_n;
 eth_reset_release u_app_reset(app_clk,arst_n,app_rst_n);
 eth_reset_release u_tx_reset(tx_clk,arst_n,tx_rst_n);
 eth_reset_release u_rx_reset(rx_sample_clk,arst_n,rx_rst_n);
 assign phy_rst_n=1'b1;
 wire phy_qualified;
 reg session_seen,tx_app_inflight;
 wire mac_tx_ready;
 wire app_tx_allowed=qualified_app||tx_app_inflight;
 assign s_tx_ready=app_rst_n&&app_tx_allowed&&mac_tx_ready;
 always @(posedge app_clk or negedge app_rst_n)begin
  if(!app_rst_n)begin session_seen<=0;tx_app_inflight<=0;end
  else if(s_tx_valid&&s_tx_ready)begin session_seen<=1;tx_app_inflight<=!s_tx_last;end
 end
 d2e_phy_startup #(.CLOCK_HZ(CLOCK_HZ),.MDC_HZ(MDC_HZ),.ROLE(ROLE))u_phy(
  .clk(app_clk),.rst_n(app_rst_n),.session_active(session_seen),.mdio_i(mdio_i),.mdc(mdc_o),.mdio_o(mdio_o),.mdio_oe(mdio_oe),
  .qualified_phy(phy_qualified),.status_words(status_words),.config_status(config_status),
  .config_before(config_before),.config_desired(config_desired),.config_readback(config_readback));
 (* async_reg="true" *)reg rx_lock_s1,rx_lock_s2;
 reg[5:0]rx_lock_warm;
 always @(posedge app_clk or negedge app_rst_n)begin
  if(!app_rst_n)begin rx_lock_s1<=0;rx_lock_s2<=0;rx_lock_warm<=0;qualified_app<=0;end
  else begin
   rx_lock_s1<=rx_pll_LOCKED;rx_lock_s2<=rx_lock_s1;
   if(!rx_lock_s2)rx_lock_warm<=0;else if(!(&rx_lock_warm))rx_lock_warm<=rx_lock_warm+1'b1;
   qualified_app<=phy_qualified&&(&rx_lock_warm);
  end
 end
 (* async_reg="true" *)reg tx_enable_s1,tx_enable_s2,rx_enable_s1,rx_enable_s2;
 always @(posedge tx_clk or negedge tx_rst_n)
  if(!tx_rst_n)begin tx_enable_s1<=0;tx_enable_s2<=0;end
  else begin tx_enable_s1<=qualified_app;tx_enable_s2<=tx_enable_s1;end
 always @(posedge rx_sample_clk or negedge rx_rst_n)
  if(!rx_rst_n)begin rx_enable_s1<=0;rx_enable_s2<=0;end
  else begin rx_enable_s1<=qualified_app;rx_enable_s2<=rx_enable_s1;end
 reg[3:0]rx_rise_delayed;reg rx_ctl_rise_delayed;
 always @(posedge rx_sample_clk or negedge rx_rst_n)
  if(!rx_rst_n)begin rx_rise_delayed<=0;rx_ctl_rise_delayed<=0;end
  else begin rx_rise_delayed<=rxd_hi_i;rx_ctl_rise_delayed<=rx_dv_HI;end
 wire[7:0]gmii_txd,gmii_rxd;
 wire gmii_tx_en,gmii_tx_er,gmii_rx_dv,gmii_rx_er;
 eth_rgmii_codec u_codec(.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_tx_er(gmii_tx_er),
  .tx_rise(txd_hi_o),.tx_fall(txd_lo_o),.tx_ctl_rise(tx_en_o_HI),.tx_ctl_fall(tx_en_o_LO),.txc_rise(txc_hi_o),.txc_fall(txc_lo_o),
  .rx_rise(rx_rise_delayed),.rx_fall(rxd_lo_i),.rx_ctl_rise(rx_ctl_rise_delayed),.rx_ctl_fall(rx_dv_LO),
  .gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er));
 wire[31:0]tx_sent,rx_good,rx_bad_fcs,rx_bad_header,rx_bad_length,rx_phy_errors,rx_no_buffer,rx_bad_preamble;
 eth_packet_mac #(.MAX_APP_BYTES(1024),.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.ETHERTYPE(ETHERTYPE))u_mac(
  .app_clk(app_clk),.tx_clk(tx_clk),.rx_clk(rx_sample_clk),.rst_n(arst_n),
  .s_tx_data(s_tx_data),.s_tx_valid(s_tx_valid&&app_tx_allowed),.s_tx_last(s_tx_last),.s_tx_ready(mac_tx_ready),
  .m_rx_data(m_rx_data),.m_rx_valid(m_rx_valid),.m_rx_ready(m_rx_ready),.m_rx_last(m_rx_last),.m_rx_length(m_rx_length),.m_rx_src_mac(m_rx_src_mac),
  .tx_enable(tx_enable_s2),.rx_enable(rx_enable_s2),.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_tx_er(gmii_tx_er),
  .gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),
  .tx_queued_frames(app_tx_queued),.tx_oversize_frames(app_tx_oversize),.tx_sent_frames(tx_sent),
  .rx_good_frames(rx_good),.rx_bad_fcs(rx_bad_fcs),.rx_bad_header(rx_bad_header),.rx_bad_length(rx_bad_length),
  .rx_phy_errors(rx_phy_errors),.rx_no_buffer(rx_no_buffer),.rx_bad_preamble(rx_bad_preamble),.rx_delivered_frames(app_rx_delivered));
 wire[1:0]stats_busy;
 eth_diag_status_cdc u_stats(.app_clk(app_clk),.app_rst_n(app_rst_n),.request(stats_request),
  .tx_clk(tx_clk),.tx_rst_n(tx_rst_n),.tx_sent_frames(tx_sent),.rx_clk(rx_sample_clk),.rx_rst_n(rx_rst_n),
  .rx_counters({rx_bad_preamble,rx_no_buffer,rx_phy_errors,rx_bad_length,rx_bad_header,rx_bad_fcs,rx_good}),
  .app_mac_words(mac_status_words),.fresh_mask(mac_status_fresh),.busy_mask(stats_busy));
endmodule
