// GMII byte interface, 1 Gbit/s full duplex. 100/10 Mbit RGMII repetition is
// intentionally not implemented: wrapper must enable only resolved 1000/full.
// Shared rst_n resets all three domains; never reset one domain in isolation.
module eth_packet_mac #(
    parameter MAX_APP_BYTES=1024, parameter ADDR_WIDTH=(MAX_APP_BYTES<=1024?10:11),
    parameter [47:0] LOCAL_MAC=48'h020000000001,
    parameter [47:0] PEER_MAC=48'h020000000002,
    parameter [15:0] ETHERTYPE=16'h88b5,
    parameter ACCEPT_BROADCAST=0
)(
    input app_clk, tx_clk, rx_clk, rst_n,
    input [7:0] s_tx_data, input s_tx_valid,s_tx_last, output s_tx_ready,
    output [7:0] m_rx_data,output m_rx_valid,input m_rx_ready,
    output m_rx_last,output [10:0] m_rx_length,output [47:0] m_rx_src_mac,
    input tx_enable,rx_enable,
    output [7:0] gmii_txd,output gmii_tx_en,gmii_tx_er,
    input [7:0] gmii_rxd,input gmii_rx_dv,gmii_rx_er,
    output [31:0] tx_queued_frames,tx_oversize_frames,tx_sent_frames,
    output [31:0] rx_good_frames,rx_bad_fcs,rx_bad_header,rx_bad_length,
    output [31:0] rx_phy_errors,rx_no_buffer,rx_bad_preamble,rx_delivered_frames
);
    wire app_rst_n,tx_rst_n,rx_rst_n;
    eth_reset_release u_app_reset(app_clk,rst_n,app_rst_n);
    eth_reset_release u_tx_reset(tx_clk,rst_n,tx_rst_n);
    eth_reset_release u_rx_reset(rx_clk,rst_n,rx_rst_n);
    eth_packet_tx #(MAX_APP_BYTES,ADDR_WIDTH,LOCAL_MAC,PEER_MAC,ETHERTYPE) u_tx(
        .app_clk(app_clk),.app_rst_n(app_rst_n),.tx_clk(tx_clk),.tx_rst_n(tx_rst_n),
        .s_tx_data(s_tx_data),.s_tx_valid(s_tx_valid),.s_tx_last(s_tx_last),.s_tx_ready(s_tx_ready),
        .tx_enable(tx_enable),.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_tx_er(gmii_tx_er),
        .queued_frames(tx_queued_frames),.oversize_frames(tx_oversize_frames),.sent_frames(tx_sent_frames));
    eth_packet_rx #(MAX_APP_BYTES,ADDR_WIDTH,LOCAL_MAC,ETHERTYPE,ACCEPT_BROADCAST) u_rx(
        .app_clk(app_clk),.app_rst_n(app_rst_n),.rx_clk(rx_clk),.rx_rst_n(rx_rst_n),
        .gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),.rx_enable(rx_enable),
        .m_rx_data(m_rx_data),.m_rx_valid(m_rx_valid),.m_rx_ready(m_rx_ready),.m_rx_last(m_rx_last),
        .m_rx_length(m_rx_length),.m_rx_src_mac(m_rx_src_mac),
        .good_frames(rx_good_frames),.bad_fcs(rx_bad_fcs),.bad_header(rx_bad_header),.bad_length(rx_bad_length),
        .phy_errors(rx_phy_errors),.no_buffer(rx_no_buffer),.bad_preamble(rx_bad_preamble),.delivered_frames(rx_delivered_frames));
endmodule
