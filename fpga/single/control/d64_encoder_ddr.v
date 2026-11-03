`timescale 1ns/1ps

// D3c6-d: D3c6-c's one-bank encoder behind dual-clock, 2 KiB DDR streams.
// The AXI master is deliberately protocol-small: all feature transactions are
// INCR bursts of 128 x 128-bit beats. A board/service model or the eventual
// vendor controller connects to these ports; no memory is inferred here.
module d64_encoder_ddr #(
    parameter [31:0] H_BASE       = 32'h0000_0000,
    parameter [31:0] D0_BASE      = 32'h0008_0000,
    parameter [31:0] TRUNK_A_BASE = 32'h000C_0000,
    parameter WR_FIFO_AW = 8,       // 256 x 130 bits: >= two H rows of payload
    parameter RD_FIFO_AW = 8,       // 256 x 128-bit CDC staging beats
    parameter WT_INIT_FILE = "",
    parameter CFG_INIT_FILE = "",
    // **The shape, in the one form the board holds it.**  `ROWS` is the
    // residual block's row count and `H = 4*ROWS` the image height; both
    // children below are told, each in the form it needs -- the encoder in
    // `H`, the DDR machinery in `ROWS` -- so that this wrapper cannot be the
    // place where the two drift apart.  The default is the shipped 64-row
    // square.
    parameter integer RUNTIME_PROFILE = 0,
    parameter integer ROWS = 64,
    parameter integer W    = 256
) (
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire          clk_pe,
    input  wire          clk_axi,
    input  wire          arst_n,
    input  wire          start,

    input  wire          d64_valid,
    output wire          d64_ready,
    input  wire [31:0]   d64_frame_id,
    input  wire          d64_sof,
    input  wire [7:0]    d64_x0,
    input  wire [7:0]    d64_y,
    input  wire [383:0]  d64_rgb888,
    input  wire          d64_eol,
    input  wire          d64_eof,

    input  wire          wt_wr_en,
    input  wire [7:0]    wt_wr_addr,
    input  wire [1023:0] wt_wr_data,
    input  wire          cfg_wr_en,
    input  wire [5:0]    cfg_wr_addr,
    input  wire [63:0]   cfg_wr_data,

    output wire          busy,
    output wire          done,
    output wire [3:0]    state,
    output wire [1:0]    pe_mode,
    output wire          mac_mode_safe,
    output wire [31:0]   mode_viol,
    output wire [31:0]   layer_mode_viol,

    // Minimal AXI4 memory-master interface. All addresses are byte addresses.
    output wire          axi_awvalid,
    input  wire          axi_awready,
    output wire [31:0]   axi_awaddr,
    output wire [7:0]    axi_awlen,
    output wire [2:0]    axi_awsize,
    output wire [1:0]    axi_awburst,
    output wire          axi_wvalid,
    input  wire          axi_wready,
    output wire [127:0]  axi_wdata,
    output wire [15:0]   axi_wstrb,
    output wire          axi_wlast,
    input  wire          axi_bvalid,
    output wire          axi_bready,
    input  wire [1:0]    axi_bresp,

    output wire          axi_arvalid,
    input  wire          axi_arready,
    output wire [31:0]   axi_araddr,
    output wire [7:0]    axi_arlen,
    output wire [2:0]    axi_arsize,
    output wire [1:0]    axi_arburst,
    input  wire          axi_rvalid,
    output wire          axi_rready,
    input  wire [127:0]  axi_rdata,
    input  wire          axi_rlast,
    input  wire [1:0]    axi_rresp,

    // Frame-level and DDR-side evidence.
    output wire [31:0]   head_mac_slots,
    output wire [31:0]   down_mac_slots,
    output wire [31:0]   head_rq_overrun,
    output wire [31:0]   down_rq_overrun,
    output wire [31:0]   head_rq_wait,
    output wire [31:0]   down_rq_wait,
    output wire [31:0]   down_read_before_ready,
    output wire [31:0]   frame_id_violations,
    output wire [31:0]   head_store_overflow,
    output wire [31:0]   head_layer_cycles,
    output wire [31:0]   down0_layer_cycles,
    output wire [31:0]   down1_layer_cycles,
    output wire [31:0]   encoder_compute_cycles,
    output wire [31:0]   encoder_store_wait_cycles,
    output reg  [31:0]   encoder_wall_cycles,
    output wire  [31:0]   trunk_store_commit_wait_cycles,
    output wire [31:0]   head_accepted_beats,
    output wire [31:0]   head_accepted_rows,
    output wire [31:0]   head_marker_violations,
    output wire [31:0]   head_read_before_ready,
    output wire  [31:0]   h_write_beats,
    output wire  [31:0]   h_read_beats,
    output wire  [31:0]   d0_write_beats,
    output wire  [31:0]   d0_read_beats,
    output wire  [31:0]   trunk_write_beats,
    output wire  [31:0]   h_write_bursts,
    output wire  [31:0]   h_read_bursts,
    output wire  [31:0]   d0_write_bursts,
    output wire  [31:0]   d0_read_bursts,
    output wire  [31:0]   trunk_write_bursts,
    output wire [31:0]   axi_response_errors,
    output wire  [31:0]   stream_pack_errors,
    output wire  [8:0]    write_fifo_max_occupancy,
    output wire  [RD_FIFO_AW:0] read_fifo_max_occupancy,
    output wire [31:0] read_fifo_overflow,
    output wire [31:0] rready_drop_in_burst
);
    wire pe_rst_n, axi_rst_n;
    d64_reset_sync u_pe_reset  (.clk(clk_pe),  .arst_n(arst_n), .rst_n(pe_rst_n));
    d64_reset_sync u_axi_reset (.clk(clk_axi), .arst_n(arst_n), .rst_n(axi_rst_n));

    // ---- the encoder, and the boundary it used to be wired to in place -----
    wire hs_valid, hs_ready, hs_sof, hs_eol, hs_eof, hs_commit;
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

    wire o_valid, o_ready, o_sof, o_eol, o_eof;
    wire [7:0]  o_x, o_y;
    wire [1:0]  o_word;
    wire [79:0] o_data;
    wire [31:0] o_fid;

    wire core_done, trunk_done_w;
    wire [31:0] core_mode_viol, core_layer_mode_viol, core_encoder_wall_cycles;
    wire [31:0] core_head_store_overflow;
    assign mode_viol = core_mode_viol;
    assign layer_mode_viol = core_layer_mode_viol;
    assign head_store_overflow = core_head_store_overflow;

    d64_encoder #(.RUNTIME_PROFILE(RUNTIME_PROFILE), .WT_INIT_FILE(WT_INIT_FILE), .CFG_INIT_FILE(CFG_INIT_FILE),
                  .H(ROWS * 4), .W(W))
    u_encoder (
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .clk(clk_pe), .rst_n(pe_rst_n), .start(start),
        .d64_valid(d64_valid), .d64_ready(d64_ready),
        .d64_frame_id(d64_frame_id), .d64_sof(d64_sof),
        .d64_x0(d64_x0), .d64_y(d64_y), .d64_rgb888(d64_rgb888),
        .d64_eol(d64_eol), .d64_eof(d64_eof),
        .wt_wr_en(wt_wr_en), .wt_wr_addr(wt_wr_addr), .wt_wr_data(wt_wr_data),
        .cfg_wr_en(cfg_wr_en), .cfg_wr_addr(cfg_wr_addr), .cfg_wr_data(cfg_wr_data),
        .head_store_wr_valid(hs_valid), .head_store_wr_ready(hs_ready),
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
        .down0_read_frame_id(d0r_fid),
        .down0_read_sof(d0r_sof), .down0_read_eol(d0r_eol),
        .down0_read_eof(d0r_eof),
        .out_valid(o_valid), .out_ready(o_ready), .out_x(o_x), .out_y(o_y),
        .out_word(o_word), .out_data(o_data), .out_frame_id(o_fid),
        .out_sof(o_sof), .out_eol(o_eol), .out_eof(o_eof),
        .busy(busy), .done(core_done), .state(state), .pe_mode(pe_mode),
        .mac_mode_safe(mac_mode_safe), .mode_viol(core_mode_viol),
        .layer_mode_viol(core_layer_mode_viol),
        .frame_id_violations(frame_id_violations),
        .head_store_overflow(core_head_store_overflow),
        .head_mac_slots(head_mac_slots), .down_mac_slots(down_mac_slots),
        .head_rq_overrun(head_rq_overrun), .down_rq_overrun(down_rq_overrun),
        .head_rq_wait(head_rq_wait), .down_rq_wait(down_rq_wait),
        .down_read_before_ready(down_read_before_ready),
        .head_layer_cycles(head_layer_cycles), .down0_layer_cycles(down0_layer_cycles),
        .down1_layer_cycles(down1_layer_cycles),
        .head_compute_cycles(), .down0_compute_cycles(), .down1_compute_cycles(),
        .encoder_compute_cycles(encoder_compute_cycles),
        .encoder_store_wait_cycles(encoder_store_wait_cycles),
        .encoder_wall_cycles(core_encoder_wall_cycles),
        .head_accepted_beats(head_accepted_beats),
        .head_accepted_rows(head_accepted_rows),
        .head_marker_violations(head_marker_violations),
        .head_read_before_ready(head_read_before_ready)
    );

    d64_encoder_ddr_mem #(
        .H_BASE(H_BASE), .D0_BASE(D0_BASE), .TRUNK_A_BASE(TRUNK_A_BASE),
        .WR_FIFO_AW(WR_FIFO_AW), .RD_FIFO_AW(RD_FIFO_AW),
        .ROWS(ROWS), .W(W)
    ) u_mem (
        .clk_pe(clk_pe), .clk_axi(clk_axi), .arst_n(arst_n),
        .pe_done(core_done),
        .head_store_valid(hs_valid), .head_store_ready(hs_ready),
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
        .out_valid(o_valid), .out_ready(o_ready), .out_x(o_x), .out_y(o_y),
        .out_word(o_word), .out_data(o_data), .out_frame_id(o_fid),
        .out_sof(o_sof), .out_eol(o_eol), .out_eof(o_eof),
        .axi_awvalid(axi_awvalid), .axi_awready(axi_awready),
        .axi_awaddr(axi_awaddr), .axi_awlen(axi_awlen),
        .axi_awsize(axi_awsize), .axi_awburst(axi_awburst),
        .axi_wvalid(axi_wvalid), .axi_wready(axi_wready),
        .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb), .axi_wlast(axi_wlast),
        .axi_bvalid(axi_bvalid), .axi_bready(axi_bready), .axi_bresp(axi_bresp),
        .axi_arvalid(axi_arvalid), .axi_arready(axi_arready),
        .axi_araddr(axi_araddr), .axi_arlen(axi_arlen),
        .axi_arsize(axi_arsize), .axi_arburst(axi_arburst),
        .axi_rvalid(axi_rvalid), .axi_rready(axi_rready),
        .axi_rdata(axi_rdata), .axi_rlast(axi_rlast), .axi_rresp(axi_rresp),
        .trunk_done(trunk_done_w),
        .trunk_store_commit_wait_cycles(trunk_store_commit_wait_cycles),
        .h_write_beats(h_write_beats), .h_read_beats(h_read_beats),
        .d0_write_beats(d0_write_beats), .d0_read_beats(d0_read_beats),
        .trunk_write_beats(trunk_write_beats),
        .h_write_bursts(h_write_bursts), .h_read_bursts(h_read_bursts),
        .d0_write_bursts(d0_write_bursts), .d0_read_bursts(d0_read_bursts),
        .trunk_write_bursts(trunk_write_bursts),
        .axi_response_errors(axi_response_errors),
        .stream_pack_errors(stream_pack_errors),
        .write_fifo_max_occupancy(write_fifo_max_occupancy),
        .read_fifo_max_occupancy(read_fifo_max_occupancy),
        .read_fifo_overflow(read_fifo_overflow),
        .rready_drop_in_burst(rready_drop_in_burst)
    );

    // Both halves, and neither alone: the schedule finishing is not the last
    // trunk-A word reaching DDR, and the write response is not the arithmetic.
    assign done = core_done && trunk_done_w;

    // The board's wall, from the source's first beat to the final trunk-A
    // response.  It spans both modules, so it lives here.
    reg encoder_wall_active;
    wire source_sof_fire = d64_valid && d64_ready && d64_sof;

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            encoder_wall_cycles <= 0;
            encoder_wall_active <= 0;
        end else begin
            if (source_sof_fire) begin
                encoder_wall_cycles <= 1;
                encoder_wall_active <= 1;
            end else if (encoder_wall_active) begin
                encoder_wall_cycles <= encoder_wall_cycles + 1;
                if (done) encoder_wall_active <= 0;
            end
        end
    end

endmodule
