# HDMI PLL BR0 is independent of the system/DDR PLLs. Payloads cross only
# through stable descriptor handshakes, Gray-pointer FIFOs or synchronized
# toggle notifications. Keep fast/slow/FB timing within the video group.
# A single asynchronous group cuts it from every other clock; it does not
# cut any system 100MHz <-> DDR/50MHz relationship.
set_clock_groups -asynchronous -group [get_clocks {hdmi_tx_slow_clk hdmi_tx_fast_clk FB}]

# CSI recovered byte clock is asynchronous to system and pixel clocks.
# 70MHz is the declared IP/board design target, not a measured frequency.
# The vendor efx_csi2/source_exp/ti60_loopback/mipi.sdc uses this separation.
set_clock_groups -asynchronous -group [get_clocks {i_cam_ck_CLKOUT}]

# ExternalSPI master clock is asynchronous to all local PLL clocks. Shared RAM
# windows and RX/TX descriptors are held through request/commit handshakes.
set_clock_groups -asynchronous -group [get_clocks {spi_preview_sclk}]
# READY is a level handshake polled asynchronously by ESP GPIO, not SPI data.
set_false_path -to [get_ports {spi_preview_ready}]
