`timescale 1ns/1ps
// Camera-only front end extracted from the proven receiver/I2C bring-up.
// No DDR, HDMI, DSI, UART telemetry, or extra PLL is instantiated here.
// clk_pix=50MHz, core_clk=100MHz, CLK_25M=25MHz; byte clock is recovered.
// External reset asserts asynchronously and releases on each free-running
// clock. Byte reset follows vendor raw-reset wiring so the link can start.
module d64_camera_frontend_517 #(parameter integer ROTATE_180 = 0) (
    input wire i_arstn, CLK_25M, core_clk, i_mipi_rx_pclk,
    input io_cam_scl_IN,
    input io_cam_sda_IN,
    input i_cam_ck_CLKOUT,
    input i_cam_ck_LP_N_IN,
    input i_cam_ck_LP_P_IN,
    input cam_d2_FIFO_EMPTY,
    input [7:0] cam_d2_HS_IN,
    input cam_d2_LP_N_IN,
    input cam_d2_LP_P_IN,
    input cam_d3_FIFO_EMPTY,
    input [7:0] cam_d3_HS_IN,
    input cam_d3_LP_N_IN,
    input cam_d3_LP_P_IN,
    input cam_d0_FIFO_EMPTY,
    input [7:0] cam_d0_HS_IN,
    input cam_d0_LP_N_IN,
    input cam_d0_LP_P_IN,
    input cam_d1_FIFO_EMPTY,
    input [7:0] cam_d1_HS_IN,
    input cam_d1_LP_N_IN,
    input cam_d1_LP_P_IN,
    output io_cam_scl_OUT,
    output io_cam_scl_OE,
    output io_cam_sda_OUT,
    output io_cam_sda_OE,
    output o_cam_rst_p,
    output o_cam_ck_HS_ENA,
    output o_cam_ck_HS_TERM,
    output cam_d2_FIFO_RD,
    output cam_d2_HS_ENA,
    output cam_d2_HS_TERM,
    output cam_d2_RST,
    output cam_d3_FIFO_RD,
    output cam_d3_HS_ENA,
    output cam_d3_HS_TERM,
    output cam_d3_RST,
    output cam_d0_FIFO_RD,
    output cam_d0_HS_ENA,
    output cam_d0_HS_TERM,
    output cam_d0_RST,
    output cam_d1_FIFO_RD,
    output cam_d1_HS_ENA,
    output cam_d1_HS_TERM,
    output cam_d1_RST,
    output        rx_pix_valid_o,
    output [63:0] rx_pix_data_o,
    output [3:0]  rx_pix_per_clk_o,
    output        rx_hsync_o,
    output        rx_vsync_o,
    output        rst_pix_n_o,
    output wire config_done, config_error,
    output wire [31:0] sensor_readback,
    output wire [7:0] sensor_orientation_readback,
    output wire [5:0] rx_datatype_o,
    output wire rx_irq_o,
    output wire [15:0] rx_word_count_o, rx_shortpkt_o,
    output wire [1:0] rx_vc_o
);
    wire clk_25m=CLK_25M, clk_pix=i_mipi_rx_pclk;
    wire arst_n=i_arstn;
    wire rst_pix_n, rst_25_n;
    d64_reset_sync u_pix_reset(.clk(clk_pix),.arst_n(arst_n),.rst_n(rst_pix_n));
    d64_reset_sync u_25_reset(.clk(clk_25m),.arst_n(arst_n),.rst_n(rst_25_n));
    localparam [7:0] N_REGS = 8'd162;

    reg  [7:0]  ld_k;
    reg         ld_run;
    reg  [15:0] nack_cnt, wr_cnt;
    reg  [7:0]  nack_first;
    reg  [7:0]  nack_last;
    wire [15:0] reg_a;
    wire [7:0]  reg_d;
    wire        i2c_busy, i2c_done, i2c_nack;

    localparam integer N_READS = ROTATE_180 != 0 ? 5 : 4;
    localparam [7:0] ORIENTATION_MASK = 8'h66;
    localparam [7:0] STREAM_PAIR_INDEX = 8'd144;
    reg [1:0] orientation_phase; // 0 normal, 1 read, 2 write, 3 verify
    reg orientation_checked, orientation_failed;
    reg [7:0] orientation_old, rd_st_3221;
    reg  [2:0]  rd_k;
    reg         rd_phase;      // 0 = writing the table, 1 = reading it back
    reg  [15:0] rd_addr;
    reg  [7:0]  rd_id_hi, rd_id_lo, rd_st_0100, rd_st_302c;
    reg  [7:0]  rd_ok;         // reads that came back without a nack
    reg  [7:0]  rd_nacks;
    reg         rd_done;

    always @* begin
        case (rd_k)
            3'd0:    rd_addr = 16'h3107;
            3'd1:    rd_addr = 16'h3108;
            3'd2:    rd_addr = 16'h0100;
            3'd3:    rd_addr = 16'h302c;
            default: rd_addr = 16'h3221;
        endcase
    end

    wire [15:0] i2c_addr = orientation_phase != 0 ? 16'h3221 : (rd_phase ? rd_addr : reg_a);
    wire        i2c_rd   = orientation_phase != 0 ? orientation_phase != 2 : rd_phase;
    wire [7:0]  i2c_wdata = orientation_phase == 2 ? (orientation_old | ORIENTATION_MASK) : reg_d;
    wire [7:0]  i2c_rdata;

    cam_sc431hai_regs u_regs (.k(ld_k), .a(reg_a), .d(reg_d));

    cam_i2c_master #(.CLK_HZ(25_000_000), .SCL_HZ(100_000)) u_i2c (
        .clk(clk_25m), .rst_n(rst_25_n),
        .start(ld_run), .addr(i2c_addr), .data(i2c_wdata),
        .rd(i2c_rd),
        .rdata(i2c_rdata),
        .busy(i2c_busy), .done(i2c_done), .nack(i2c_nack),
        .scl_i(io_cam_scl_IN), .scl_oe(io_cam_scl_OE),
        .sda_i(io_cam_sda_IN), .sda_oe(io_cam_sda_OE)
    );

    assign io_cam_scl_OUT = 1'b0;
    assign io_cam_sda_OUT = 1'b0;

    localparam [19:0] POR_END   = 20'h0FFFF;   // 2.6 ms with reset asserted
    localparam [19:0] SET_END   = 20'h3FFFF;   // 10.5 ms after it is released
    localparam [1:0]  MAX_RETRY = 2'd3;        // four attempts per entry

    reg [19:0] por_cnt;
    reg [19:0] set_cnt;
    reg        cam_rst_n;
    reg        boot_done;      // the settle has elapsed
    reg        need_send;      // an entry is waiting to go out
    reg  [1:0] retry;          // consecutive nacks on the current entry

    always @(posedge clk_25m or negedge rst_25_n) begin
        if (!rst_25_n) begin
            por_cnt <= 20'd0; set_cnt <= 20'd0; cam_rst_n <= 1'b0; boot_done <= 1'b0;
            ld_run <= 1'b0; need_send <= 1'b0; retry <= 2'd0;
            ld_k <= 8'd0; nack_cnt <= 16'd0; wr_cnt <= 16'd0;
            nack_first <= 8'hFF; nack_last <= 8'hFF;
            rd_phase <= 1'b0; rd_k <= 3'd0; rd_done <= 1'b0;
            orientation_phase <= 0; orientation_checked <= 0; orientation_failed <= 0;
            orientation_old <= 0; rd_st_3221 <= 0;
            rd_id_hi <= 8'h00; rd_id_lo <= 8'h00;
            rd_st_0100 <= 8'h00; rd_st_302c <= 8'h00;
            rd_ok <= 8'd0; rd_nacks <= 8'd0;
        end
        else begin
            ld_run <= 1'b0;
            if (!cam_rst_n) begin
                por_cnt <= por_cnt + 20'd1;
                if (por_cnt == POR_END) cam_rst_n <= 1'b1;
            end
            else begin
                if (!boot_done) begin
                    if (set_cnt == SET_END) begin
                        boot_done <= 1'b1;
                        need_send <= 1'b1;
                    end
                    else set_cnt <= set_cnt + 20'd1;
                end

                if (i2c_done) begin
                    if (orientation_phase != 0) begin
                        if (i2c_nack) begin
                            if (retry != MAX_RETRY) begin
                                retry <= retry + 2'd1;
                                need_send <= 1'b1;
                            end else begin
                                // Fail closed: the stream-on pair is not issued.
                                orientation_failed <= 1'b1;
                                orientation_phase <= 0;
                                orientation_checked <= 1'b1;
                                need_send <= 1'b0;
                            end
                        end else begin
                            retry <= 0;
                            if (orientation_phase == 1) begin
                                orientation_old <= i2c_rdata;
                                orientation_phase <= 2;
                                need_send <= 1'b1;
                            end else if (orientation_phase == 2) begin
                                orientation_phase <= 3;
                                need_send <= 1'b1;
                            end else begin
                                rd_st_3221 <= i2c_rdata;
                                orientation_phase <= 0;
                                orientation_checked <= 1'b1;
                                if (i2c_rdata == (orientation_old | ORIENTATION_MASK))
                                    need_send <= 1'b1;
                                else begin
                                    orientation_failed <= 1'b1;
                                    need_send <= 1'b0;
                                end
                            end
                        end
                    end else if (!rd_phase) begin
                        if (i2c_nack) begin
                            nack_cnt <= nack_cnt + 16'd1;
                            if (nack_first == 8'hFF) nack_first <= ld_k;
                            nack_last <= ld_k;
                        end
                        else wr_cnt <= wr_cnt + 16'd1;

                        if (i2c_nack && (retry != MAX_RETRY)) begin
                            retry     <= retry + 2'd1;
                            need_send <= 1'b1;                     // same entry again
                        end
                        else begin
                            retry <= 2'd0;
                            if (ld_k == N_REGS - 8'd1) begin
                                ld_k      <= N_REGS;
                                rd_phase  <= 1'b1;
                                rd_k      <= 3'd0;
                                need_send <= 1'b1;
                            end
                            else begin
                                ld_k      <= ld_k + 8'd1;
                                need_send <= 1'b1;
                            end
                        end
                    end
                    else begin
                        if (i2c_nack) begin
                            rd_nacks <= rd_nacks + 8'd1;
                            case (rd_k)
                                3'd0:    rd_id_hi   <= 8'hFF;
                                3'd1:    rd_id_lo   <= 8'hFF;
                                3'd2:    rd_st_0100 <= 8'hFF;
                                3'd3:    rd_st_302c <= 8'hFF;
                                default: rd_st_3221 <= 8'hFF;
                            endcase
                        end
                        else begin
                            rd_ok <= rd_ok + 8'd1;
                            case (rd_k)
                                3'd0:    rd_id_hi   <= i2c_rdata;
                                3'd1:    rd_id_lo   <= i2c_rdata;
                                3'd2:    rd_st_0100 <= i2c_rdata;
                                3'd3:    rd_st_302c <= i2c_rdata;
                                default: rd_st_3221 <= i2c_rdata;
                            endcase
                        end

                        if (i2c_nack && (retry != MAX_RETRY)) begin
                            retry     <= retry + 2'd1;
                            need_send <= 1'b1;                 // same register again
                        end
                        else begin
                            retry <= 2'd0;
                            if (rd_k == N_READS - 3'd1) rd_done <= 1'b1;
                            else begin
                                rd_k      <= rd_k + 3'd1;
                                need_send <= 1'b1;
                            end
                        end
                    end
                end
                else if (need_send && !i2c_busy) begin
                    if (ROTATE_180 != 0 && !rd_phase && !orientation_checked &&
                        orientation_phase == 0 && ld_k == STREAM_PAIR_INDEX) begin
                        // Insert before the original stream-on pair, preserving
                        // all LUT entries/order. Read its actual value.
                        orientation_phase <= 1;
                    end else begin
                        ld_run    <= 1'b1;
                        need_send <= 1'b0;
                    end
                end
            end
        end
    end

    assign o_cam_rst_p = ~cam_rst_n;
    wire [63:0] rx_pix_data;
    wire [3:0]  rx_pix_per_clk;
    wire [5:0]  rx_datatype;
    wire        rx_pix_valid, rx_hsync, rx_vsync, rx_irq;
    wire [15:0] ip_word_count, ip_shortpkt;
    wire [1:0]  ip_vc;
    wire        o_cam_ck_HS_ENA_w, o_cam_ck_HS_TERM_w;
    wire [3:0]  cam_HS_ENA_w, cam_HS_TERM_w, cam_FIFO_RD_w;
    wire [3:0] cam_FIFO_RST_w;
    wire [3:0]  cam_FIFO_EMPTY_w;

    csi_rx_controller u_rx (
        .reset_n            (arst_n),
        .clk                (core_clk),
        .reset_byte_HS_n    (arst_n),
        .clk_byte_HS        (i_cam_ck_CLKOUT),
        .reset_pixel_n      (rst_pix_n),
        .clk_pixel          (clk_pix),

        .Rx_LP_CLK_P        (i_cam_ck_LP_P_IN),
        .Rx_LP_CLK_N        (i_cam_ck_LP_N_IN),
        .Rx_HS_enable_C     (o_cam_ck_HS_ENA_w),
        .LVDS_termen_C      (o_cam_ck_HS_TERM_w),

        .Rx_LP_D_P          ({cam_d3_LP_P_IN, cam_d2_LP_P_IN,
                              cam_d1_LP_P_IN, cam_d0_LP_P_IN}),
        .Rx_LP_D_N          ({cam_d3_LP_N_IN, cam_d2_LP_N_IN,
                              cam_d1_LP_N_IN, cam_d0_LP_N_IN}),
        .Rx_HS_D_0          (cam_d0_HS_IN),
        .Rx_HS_D_1          (cam_d1_HS_IN),
        .Rx_HS_D_2          (cam_d2_HS_IN),
        .Rx_HS_D_3          (cam_d3_HS_IN),
        .Rx_HS_D_4          (),
        .Rx_HS_D_5          (),
        .Rx_HS_D_6          (),
        .Rx_HS_D_7          (),
        .Rx_HS_enable_D     (cam_HS_ENA_w),
        .LVDS_termen_D      (cam_HS_TERM_w),
        .fifo_rd_enable     (cam_FIFO_RD_w),
        
        .fifo_rd_empty      (cam_FIFO_EMPTY_w),
        .DLY_enable_D       (),
        .DLY_inc_D          (),
        .u_dly_enable_D     (),
        .u_dly_inc_D        (),

        .axi_clk            (1'b0),
        .axi_reset_n        (1'b0),
        .axi_awaddr         (6'b0),
        .axi_awvalid        (1'b0),
        .axi_awready        (),
        .axi_wdata          (32'b0),
        .axi_wvalid         (1'b0),
        .axi_wready         (),
        .axi_bvalid         (),
        .axi_bready         (1'b0),
        .axi_araddr         (6'b0),
        .axi_arvalid        (1'b0),
        .axi_arready        (),
        .axi_rdata          (),
        .axi_rvalid         (),
        .axi_rready         (1'b1),

        .hsync_vc0          (rx_hsync),
        .hsync_vc1          (), .hsync_vc2 (), .hsync_vc3 (),
        .vsync_vc0          (rx_vsync),
        .vsync_vc1          (), .vsync_vc2 (), .vsync_vc3 (),
        .vc                 (ip_vc),
        .word_count         (ip_word_count),
        .shortpkt_data_field(ip_shortpkt),
        .datatype           (rx_datatype),
        .pixel_per_clk      (rx_pix_per_clk),
        .pixel_data         (rx_pix_data),
        .pixel_data_valid   (rx_pix_valid),
        .irq                (rx_irq),
        .mipi_debug_in     (32'd0),
        .mipi_debug_out    ()
    );

    assign rx_pix_valid_o   = rx_pix_valid;
    assign rx_pix_data_o    = rx_pix_data;
    assign rx_pix_per_clk_o = rx_pix_per_clk;
    assign rx_hsync_o       = rx_hsync;
    assign rx_vsync_o       = rx_vsync;
    assign rst_pix_n_o      = rst_pix_n;

    assign o_cam_ck_HS_ENA  = o_cam_ck_HS_ENA_w;
    assign o_cam_ck_HS_TERM = o_cam_ck_HS_TERM_w;
    assign cam_d0_HS_ENA = cam_HS_ENA_w[0];  assign cam_d0_HS_TERM = cam_HS_TERM_w[0];
    assign cam_d1_HS_ENA = cam_HS_ENA_w[1];  assign cam_d1_HS_TERM = cam_HS_TERM_w[1];
    assign cam_d2_HS_ENA = cam_HS_ENA_w[2];  assign cam_d2_HS_TERM = cam_HS_TERM_w[2];
    assign cam_d3_HS_ENA = cam_HS_ENA_w[3];  assign cam_d3_HS_TERM = cam_HS_TERM_w[3];
    assign cam_d0_FIFO_RD = cam_FIFO_RD_w[0];
    assign cam_d1_FIFO_RD = cam_FIFO_RD_w[1];
    assign cam_d2_FIFO_RD = cam_FIFO_RD_w[2];
    assign cam_d3_FIFO_RD = cam_FIFO_RD_w[3];
    assign cam_FIFO_EMPTY_w = {cam_d3_FIFO_EMPTY, cam_d2_FIFO_EMPTY,
                               cam_d1_FIFO_EMPTY, cam_d0_FIFO_EMPTY};
    assign cam_d0_RST = 1'b0; assign cam_d1_RST = 1'b0;
    assign cam_d2_RST = 1'b0; assign cam_d3_RST = 1'b0;
    assign config_done=rd_done || orientation_failed;
    reg config_failed;
    always @(posedge clk_25m or negedge rst_25_n)
        if(!rst_25_n) config_failed<=0;
        else if(i2c_done && i2c_nack && retry==MAX_RETRY) config_failed<=1;
    // A recovered retry is diagnostic evidence, not a fatal configuration.
    assign config_error=config_failed || orientation_failed || (rd_done &&
        (({rd_id_hi,rd_id_lo,rd_st_0100,rd_st_302c}!=32'hcd6b0100) ||
         (ROTATE_180 != 0 && rd_st_3221 != (orientation_old | ORIENTATION_MASK))));
    assign sensor_orientation_readback=rd_st_3221;
    assign sensor_readback={rd_id_hi,rd_id_lo,rd_st_0100,rd_st_302c};
    assign rx_datatype_o=rx_datatype;
    assign rx_irq_o=rx_irq;
    assign rx_word_count_o=ip_word_count;
    assign rx_shortpkt_o=ip_shortpkt;
    assign rx_vc_o=ip_vc;
endmodule
