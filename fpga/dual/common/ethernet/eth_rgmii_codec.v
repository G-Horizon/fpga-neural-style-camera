// Logical DDR codec. *_rise/*_fall are one paired byte at the GPIO DDR boundary,
// NOT arbitrary fabric samples of pads. Wrapper owns IDDR/ODDR, phase and delays.
// For RGMII: rising edge low nibble, falling edge high nibble; CTL fall=DV^ER.
module eth_rgmii_codec(
    input [7:0] gmii_txd,input gmii_tx_en,gmii_tx_er,
    output [3:0] tx_rise,tx_fall,output tx_ctl_rise,tx_ctl_fall,
    output txc_rise,txc_fall,
    input [3:0] rx_rise,rx_fall,input rx_ctl_rise,rx_ctl_fall,
    output [7:0] gmii_rxd,output gmii_rx_dv,gmii_rx_er
);
    assign tx_rise=gmii_txd[3:0];assign tx_fall=gmii_txd[7:4];
    assign tx_ctl_rise=gmii_tx_en;assign tx_ctl_fall=gmii_tx_en^gmii_tx_er;
    assign txc_rise=1'b1;assign txc_fall=1'b0;
    assign gmii_rxd={rx_fall,rx_rise};assign gmii_rx_dv=rx_ctl_rise;
    assign gmii_rx_er=rx_ctl_rise^rx_ctl_fall;
endmodule
