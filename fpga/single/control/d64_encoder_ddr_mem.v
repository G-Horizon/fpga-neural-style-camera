`timescale 1ns/1ps

// D3c6-e6 -- `d64_encoder_ddr` with the encoder lifted out.
//
// D3c6-d closed this design on the board: the encoder, the write FIFO, the AXI
// writer, the AXI reader and the read FIFO, with the layer stores and trunk-A
// in real DDR3.  D3c6-e6 needs exactly that machinery behind a *different*
// compute block -- `d64_encoder_trunk`, which owns one multiplier array and one
// weight store for the encoder and the trunk together -- and the choice was
// between copying it and extracting it.
//
// Copying was rejected for the reason D3c6-e5 exists: two copies of a thing
// that must stay in step is what this whole stage is about removing.  So the
// machinery moves here verbatim and `d64_encoder_ddr` becomes this module plus
// the encoder it always had.  The two testbenches that closed D3c6-d
// (`tb_d64_encoder_ddr` and the board-harness model) are the proof that the
// extraction changed nothing, and they run against both shapes.
//
// What is *not* here: no encoder, no `start`, no `d64_*` stream, no weight or
// requant load, and none of the encoder's own evidence.  The PE side is the
// layer boundary, port for port, and the names are the ones the machinery
// already used -- which is what makes this a move and not a rewrite.

module d64_encoder_ddr_mem #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    // The final multi-master root must not hold AW128 ownership while the
    // MAC is still producing the remainder. Legacy unit roots keep mode0.
    parameter integer RESERVE_WRITE_BURST = 0,
    parameter [31:0] H_BASE       = 32'h0000_0000,
    parameter [31:0] D0_BASE      = 32'h0008_0000,
    parameter [31:0] TRUNK_A_BASE = 32'h000C_0000,
    parameter WR_FIFO_AW = 8,
    parameter RD_FIFO_AW = 8,
    // **The shape, and this module was the other half of the same gap.**
    // `ROWS` is the residual block's row count -- the one number the board
    // holds -- and `H = 4*ROWS` is the image height, the graph's own
    // arithmetic rather than a convention: DOWN1 is the image after two
    // stride-2 downsamplings, which is why a residual block is `ROWS` rows and
    // the image is four times that.  `W` is the image width, carried because
    // the H store's pixel count needs it.
    //
    // Every literal this module carried is a function of those two, and each
    // one was a *completion* or an *end-of-frame* test -- the F09 shape of
    // fault: a counter that reaches the right total while the event that
    // should follow it never fires.  At `ROWS = 64` every expression below is
    // the literal it replaced (32768, 16384, 16384, 65535, 16383), which is
    // the arm that says nothing about the shipped shape moved.
    parameter integer ROWS = 64,
    parameter integer W    = 256
) (
    output wire [31:0] writer_debug,
    input  wire          clk_pe,
    input  wire          clk_axi,
    input  wire          arst_n,
    // The encoder's own `done`, which is the caller's signal.  This module only
    // needs it to count how long the far side took to commit what it was given.
    input  wire          pe_done,
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

    // ---- the PE side: `d64_encoder`'s layer boundary, port for port -------
    //
    // The names are the ones the machinery below already used, so this module
    // is the D3c6-d one with its encoder lifted out and the boundary turned
    // into ports.  That is the whole of the extraction: the write FIFO, the
    // packers, the AXI writer and the AXI reader are character for character
    // what the board ran.
    input  wire          head_store_valid,
    output wire          head_store_ready,
    output wire          head_phase_ready,
    input  wire [7:0]    head_store_x,
    input  wire [7:0]    head_store_y,
    input  wire [63:0]   head_store_c8,
    input  wire [31:0]   head_store_frame_id,
    input  wire          head_store_sof,
    input  wire          head_store_eol,
    input  wire          head_store_eof,
    output wire          head_commit_ready,

    input  wire          head_read_start,
    output wire          head_read_valid,
    input  wire          head_read_ready,
    output wire [7:0]    head_read_x,
    output wire [7:0]    head_read_y,
    output wire [63:0]   head_read_c8,
    output wire [31:0]   head_read_frame_id,
    output wire          head_read_sof,
    output wire          head_read_eol,
    output wire          head_read_eof,

    input  wire          d0_store_valid,
    output wire          d0_store_ready,
    input  wire [7:0]    d0_store_x,
    input  wire [7:0]    d0_store_y,
    input  wire [1:0]    d0_store_word,
    input  wire [79:0]   d0_store_data,
    input  wire [31:0]   d0_store_frame_id,
    input  wire          d0_store_sof,
    input  wire          d0_store_eol,
    input  wire          d0_store_eof,
    output wire          d0_commit_ready,

    input  wire          d0_read_start,
    output wire          d0_read_valid,
    input  wire          d0_read_ready,
    output wire [7:0]    d0_read_x,
    output wire [7:0]    d0_read_y,
    output wire          d0_read_icg,
    output wire [63:0]   d0_read_c8,
    output wire [31:0]   d0_read_frame_id,
    output wire          d0_read_sof,
    output wire          d0_read_eol,
    output wire          d0_read_eof,

    input  wire          out_valid,
    output wire          out_ready,
    input  wire [7:0]    out_x,
    input  wire [7:0]    out_y,
    input  wire [1:0]    out_word,
    input  wire [79:0]   out_data,
    input  wire [31:0]   out_frame_id,
    input  wire          out_sof,
    input  wire          out_eol,
    input  wire          out_eof,

    output wire          trunk_done,
    // **The same event, before it has crossed a clock.**  `trunk_done` above is
    // `clk_axi`'s toggle synchronised into `clk_pe`, and the ordering question
    // this answers -- "did the trunk's first read leave after the encoder's last
    // write response" -- lives entirely in `clk_axi`.  Measuring it through the
    // PE-domain copy means measuring it through two synchronisers and a domain
    // crossing, which is a different question.  This is the raw toggle.
    output wire          trunk_commit_axi,
    output reg  [31:0]   trunk_store_commit_wait_cycles,
    output reg  [31:0]   h_write_beats,
    output reg  [31:0]   h_read_beats,
    output reg  [31:0]   d0_write_beats,
    output reg  [31:0]   d0_read_beats,
    output reg  [31:0]   trunk_write_beats,
    output reg  [31:0]   h_write_bursts,
    output reg  [31:0]   h_read_bursts,
    output reg  [31:0]   d0_write_bursts,
    output reg  [31:0]   d0_read_bursts,
    output reg  [31:0]   trunk_write_bursts,
    output wire [31:0]   axi_response_errors,
    output reg  [31:0]   stream_pack_errors,
    output reg  [8:0]    write_fifo_max_occupancy,
    output reg  [RD_FIFO_AW:0] read_fifo_max_occupancy,
    output reg [31:0] read_fifo_overflow,
    output reg [31:0] rready_drop_in_burst
);

    localparam [1:0] KIND_H = 2'd0, KIND_D0 = 2'd1, KIND_TRUNK = 2'd2;
    localparam [1:0] W_IDLE = 2'd0, W_AW = 2'd1,
                     W_DATA = 2'd2, W_B = 2'd3;

    // ------------------------------------------------------------------ shape
    // One derivation, and every count below reads it.  **These are the same
    // numbers `d64_encoder_trunk_board_top.v` derives and compares the
    // measured counters against** (`HEAD_STORE_BEATS` = `(W*H)/2`,
    // `DOWN0_STORE_BEATS` = `(W*H)/4`, and the burst counts `H` and `H/2`);
    // stating them here is what makes them the design's rather than the
    // board's expectation.  The store geometry is fixed and the shape does not
    // move it: the H store is 8 bytes a pixel and the AXI beat is 16, so two
    // pixels a beat; DOWN0 emits one pixel a beat; a residual block is `ROWS`
    // rows of 256 128-bit beats (`D64_ROW_BEATS`, `board/ddr3/d64_blk_beats.vh`).
    //
    // **The bound on these counters is the reader's, not the map's.**  The
    // pixel index `rd_pixel_q` is 16 bits, so `W*H <= 65536` -- `ROWS <= 64` at
    // W = 256 -- and the map's own ceiling (`ROWS <= 128`, `d64_net_addr.v`)
    // is not reachable from here without widening it.  At `ROWS = 36` the image
    // is 36,864 pixels and the block 9,216 beats, both well inside.
    localparam integer H           = ROWS * 4;
    // `rd_pixel_q` is **two different counters wearing one name**: the H reader
    // steps it once per fire -- two pixels a 128-bit beat, so it indexes pixels
    // and ends on `H_PIXELS - 1` -- while the D0 reader steps it once a beat and
    // ends on the beat count.  Both end tests below are written against the
    // quantity that counter actually holds.
    localparam integer H_PIXELS    = W * H;          // 65536 at ROWS 64
    localparam integer H_BEATS     = H_PIXELS / 2;   // two 8-byte pixels a beat
    localparam integer D0_BEATS    = H_PIXELS / 4;   // two pixels a beat, half the rows
    localparam integer TRUNK_BEATS = ROWS * 256;     // one 16-byte pixel a beat

    wire pe_rst_n, axi_rst_n;
    d64_reset_sync u_pe_reset  (.clk(clk_pe), .arst_n(arst_n), .rst_n(pe_rst_n));
    d64_reset_sync u_axi_reset (.clk(clk_axi), .arst_n(arst_n), .rst_n(axi_rst_n));


    // ---------------------------------------------------------------- write FIFO
    // The packet tag shares the CDC entry with the beat. The address is derived
    // by the AXI-side writer from that tag and its committed beat count.
    wire [129:0] wf_rd_data;
    wire wf_full, wf_almost_full, wf_empty, wf_rd_valid;
    wire [WR_FIFO_AW:0] wf_wr_level, wf_rd_level;
    wire [RD_FIFO_AW:0] rf_wr_level, rf_rd_level;
    wire wf_rd_en;
    wire [127:0] wr_beat;
    wire [1:0] wr_kind;
    wire h_fire = head_store_valid && head_store_ready;
    wire d0_fire = d0_store_valid && d0_store_ready;
    wire out_fire = out_valid && out_ready;
    reg h_half_valid, d0_half_valid;
    reg [63:0] h_half_q, d0_half_q;
    reg [7:0] h_half_x_q, h_half_y_q, d0_half_x_q, d0_half_y_q;
    reg [31:0] h_half_frame_q, d0_half_frame_q;
    wire h_fifo_push = h_fire && h_half_valid;
    wire d0_fifo_push = d0_fire && d0_half_valid;
    wire d1_fifo_push = out_fire;
    reg [127:0] d1_u16_beat;
    integer d1_lane;

    always @* begin
        d1_u16_beat = 128'd0;
        for (d1_lane = 0; d1_lane < 8; d1_lane = d1_lane + 1)
            d1_u16_beat[d1_lane*16 +: 16] =
                {6'b0, out_data[d1_lane*10 +: 10]};
    end

    // Producer output remains a pulse. Keep its normal ready threshold, and
    // use separate phase admission credit so previously issued output is accepted.
    // HEAD needs nine taps per four pixels; its two metadata stages plus four
    // requant passes leave at most two phases (eight pixels/four beats) owed.
    // Sixteen beats leave room for that debt, a half-packed pixel, and margin.
    assign head_phase_ready = pe_rst_n && (wf_wr_level <= ((1 << WR_FIFO_AW) - 16));
    assign head_store_ready = !wf_almost_full;
    assign d0_store_ready = !wf_almost_full;
    assign out_ready = !wf_almost_full;
    assign wr_kind = h_fifo_push ? KIND_H :
                     d0_fifo_push ? KIND_D0 : KIND_TRUNK;
    assign wr_beat = h_fifo_push ? {head_store_c8, h_half_q} :
                     d0_fifo_push ? {d0_store_data[63:0], d0_half_q} :
                     d1_u16_beat;
    wire wf_wr_en = h_fifo_push || d0_fifo_push || d1_fifo_push;
    wire [129:0] wf_wr_data = {wr_kind, wr_beat};

    d64_async_fifo #(.DW(130), .AW(WR_FIFO_AW),
                     .ALMOST_FULL_LEVEL((1 << WR_FIFO_AW) - 2)) u_write_fifo (
        .wr_clk(clk_pe), .wr_rst_n(pe_rst_n), .wr_en(wf_wr_en),
        .wr_data(wf_wr_data), .wr_full(wf_full),
        .wr_almost_full(wf_almost_full), .wr_level(wf_wr_level),
        .rd_clk(clk_axi), .rd_rst_n(axi_rst_n), .rd_en(wf_rd_en),
        .rd_data(wf_rd_data), .rd_valid(wf_rd_valid), .rd_empty(wf_empty),
        .rd_level(wf_rd_level)
    );

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            h_half_valid <= 1'b0; h_half_q <= 64'd0;
            d0_half_valid <= 1'b0; d0_half_q <= 64'd0;
            h_half_x_q <= 0; h_half_y_q <= 0; h_half_frame_q <= 0;
            d0_half_x_q <= 0; d0_half_y_q <= 0; d0_half_frame_q <= 0;
            write_fifo_max_occupancy <= 9'd0;
            read_fifo_max_occupancy <= 0;
            stream_pack_errors <= 32'd0;
        end else begin
            if (wf_wr_level > write_fifo_max_occupancy)
                write_fifo_max_occupancy <= wf_wr_level[8:0];
            if (rf_rd_level > read_fifo_max_occupancy)
                read_fifo_max_occupancy <= rf_rd_level;

            if (head_store_valid && head_store_ready) begin
                if (!h_half_valid) begin
                    if (head_store_x[0]) stream_pack_errors <= stream_pack_errors + 1;
                    h_half_q <= head_store_c8;
                    h_half_x_q <= head_store_x;
                    h_half_y_q <= head_store_y;
                    h_half_frame_q <= head_store_frame_id;
                    h_half_valid <= 1'b1;
                end else begin
                    if (!head_store_x[0] || head_store_x != (h_half_x_q + 8'd1) ||
                        head_store_y != h_half_y_q ||
                        head_store_frame_id != h_half_frame_q)
                        stream_pack_errors <= stream_pack_errors + 1;
                    h_half_valid <= 1'b0;
                end
            end

            if (d0_store_valid && d0_store_ready) begin
                if (!d0_half_valid) begin
                    if (d0_store_word != 0) stream_pack_errors <= stream_pack_errors + 1;
                    d0_half_q <= d0_store_data[63:0];
                    d0_half_x_q <= d0_store_x; d0_half_y_q <= d0_store_y;
                    d0_half_frame_q <= d0_store_frame_id;
                    d0_half_valid <= 1'b1;
                end else begin
                    if (d0_store_word != 1 || d0_store_x != d0_half_x_q ||
                        d0_store_y != d0_half_y_q ||
                        d0_store_frame_id != d0_half_frame_q)
                        stream_pack_errors <= stream_pack_errors + 1;
                    d0_half_valid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------ AXI writer
    reg [1:0] wstate_q, wkind_q;
    reg [31:0] awaddr_q;
    reg [6:0] wbeat_q;
    reg [31:0] h_wcount_q, d0_wcount_q, trunk_wcount_q;
    reg h_done_tog_axi, d0_done_tog_axi, trunk_done_tog_axi;
    reg [31:0] writer_axi_errors, reader_axi_errors;
    assign axi_response_errors = writer_axi_errors + reader_axi_errors;
    reg [1:0] wq_count;
    reg [129:0] wq0, wq1;
    reg wf_rd_pending;
    wire wf_pop;
    wire wf_push = wf_rd_valid;
    wire wf_pop_int = (wstate_q == W_DATA) && axi_wvalid && axi_wready;
    wire [2:0] wq_after_pop = {1'b0, wq_count} - wf_pop_int;
    // rd_level excludes words already prefetched into the local two-entry
    // queue. Add those and the one registered read still in flight exactly
    // once. The synchronized writer pointer makes rd_level conservative.
    // All shipped stores have128-beat tails: H=ROWS*512 beats,
    // D0=ROWS*256 beats, trunk-A=ROWS*256 beats at W256/H4*ROWS.
    wire [WR_FIFO_AW+1:0] buffered_write_words =
        {1'b0, wf_rd_level} + {{WR_FIFO_AW{1'b0}}, wq_count} +
        {{(WR_FIFO_AW+1){1'b0}}, wf_rd_pending};
    wire write_burst_reserved = !RESERVE_WRITE_BURST ||
                               (buffered_write_words >= 128);
    assign wf_pop = wf_pop_int;
    assign wf_rd_en = !wf_empty &&
                      ((wq_after_pop + wf_rd_pending) < 2);

    function [31:0] write_base;
        input [1:0] kind;
        begin
            case (kind)
                KIND_H: write_base = H_BASE;
                KIND_D0: write_base = D0_BASE;
                default: write_base = TRUNK_A_BASE;
            endcase
        end
    endfunction

    function [31:0] write_count;
        input [1:0] kind;
        begin
            case (kind)
                KIND_H: write_count = h_wcount_q;
                KIND_D0: write_count = d0_wcount_q;
                default: write_count = trunk_wcount_q;
            endcase
        end
    endfunction

    assign axi_awvalid = (wstate_q == W_AW);
    assign axi_awaddr = awaddr_q;
    assign axi_awlen = 8'd127;
    assign axi_awsize = 3'd4;
    assign axi_awburst = 2'b01;
    assign axi_wvalid = (wstate_q == W_DATA) && (wq_count != 0);
    assign axi_wdata = wq0[127:0];
    assign axi_wstrb = 16'hffff;
    assign axi_wlast = (wbeat_q == 7'd127);
    assign axi_bready = (wstate_q == W_B);

    always @(posedge clk_axi or negedge axi_rst_n) begin
        if (!axi_rst_n) begin
            wq_count <= 0; wq0 <= 0; wq1 <= 0; wf_rd_pending <= 0;
        end else begin
            case ({wf_rd_en, wf_rd_valid})
                2'b10: wf_rd_pending <= 1'b1;
                2'b01: wf_rd_pending <= 1'b0;
                2'b11: wf_rd_pending <= 1'b1;
                default: begin end
            endcase
            case ({wf_push, wf_pop_int})
                2'b10: begin
                    if (wq_count == 0) wq0 <= wf_rd_data;
                    else if (wq_count == 1) wq1 <= wf_rd_data;
                    if (wq_count < 2) wq_count <= wq_count + 1'b1;
                end
                2'b01: begin
                    if (wq_count == 2) wq0 <= wq1;
                    if (wq_count != 0) wq_count <= wq_count - 1'b1;
                end
                2'b11: begin
                    if (wq_count == 1) wq0 <= wf_rd_data;
                    else if (wq_count == 2) begin
                        wq0 <= wq1;
                        wq1 <= wf_rd_data;
                    end else begin
                        wq0 <= wf_rd_data;
                        wq_count <= 1;
                    end
                end
                default: begin end
            endcase
        end
    end

    always @(posedge clk_axi or negedge axi_rst_n) begin
        if (!axi_rst_n) begin
            wstate_q <= W_IDLE; wkind_q <= KIND_H;
            awaddr_q <= 0; wbeat_q <= 0;
            h_wcount_q <= 0; d0_wcount_q <= 0; trunk_wcount_q <= 0;
            h_write_beats <= 0; d0_write_beats <= 0; trunk_write_beats <= 0;
            h_write_bursts <= 0; d0_write_bursts <= 0; trunk_write_bursts <= 0;
            h_done_tog_axi <= 0; d0_done_tog_axi <= 0; trunk_done_tog_axi <= 0;
            writer_axi_errors <= 0;
        end else begin
            case (wstate_q)
                W_IDLE: if ((wq_count != 0) && write_burst_reserved) begin
                    wkind_q <= wq0[129:128];
                    awaddr_q <= write_base(wq0[129:128]) +
                                 (write_count(wq0[129:128]) << 4);
                    wbeat_q <= 0;
                    wstate_q <= W_AW;
                end
                W_AW: if (axi_awvalid && axi_awready) begin
                    case (wkind_q)
                        KIND_H: h_write_bursts <= h_write_bursts + 1;
                        KIND_D0: d0_write_bursts <= d0_write_bursts + 1;
                        KIND_TRUNK: trunk_write_bursts <= trunk_write_bursts + 1;
                        default: writer_axi_errors <= FAULT_STICKY ? 32'd1 : writer_axi_errors + 32'd1;
                    endcase
                    wstate_q <= W_DATA;
                end
                W_DATA: if (axi_wvalid && axi_wready) begin
                    if (wq0[129:128] != wkind_q)
                        writer_axi_errors <= FAULT_STICKY ? 32'd1 : writer_axi_errors + 32'd1;
                    case (wkind_q)
                        KIND_H: begin h_wcount_q <= h_wcount_q + 1;
                                      h_write_beats <= h_write_beats + 1; end
                        KIND_D0: begin d0_wcount_q <= d0_wcount_q + 1;
                                       d0_write_beats <= d0_write_beats + 1; end
                        KIND_TRUNK: begin trunk_wcount_q <= trunk_wcount_q + 1;
                                          trunk_write_beats <= trunk_write_beats + 1; end
                    endcase
                    if (wbeat_q == 7'd127) begin
                        wstate_q <= W_B;
                        wbeat_q <= 0;
                    end else wbeat_q <= wbeat_q + 1'b1;
                end
                W_B: if (axi_bvalid && axi_bready) begin
                    if (axi_bresp != 0) writer_axi_errors <= FAULT_STICKY ? 32'd1 : writer_axi_errors + 32'd1;
                    else begin
                        case (wkind_q)
                            KIND_H: if (h_wcount_q == H_BEATS) h_done_tog_axi <= ~h_done_tog_axi;
                            KIND_D0: if (d0_wcount_q == D0_BEATS) d0_done_tog_axi <= ~d0_done_tog_axi;
                            KIND_TRUNK: if (trunk_wcount_q == TRUNK_BEATS)
                                            trunk_done_tog_axi <= ~trunk_done_tog_axi;
                            default: writer_axi_errors <= FAULT_STICKY ? 32'd1 : writer_axi_errors + 32'd1;
                        endcase
                    end
                    wstate_q <= W_IDLE;
                end
                default: wstate_q <= W_IDLE;
            endcase
        end
    end

    reg h_done_sync1, h_done_sync2;
    reg d0_done_sync1, d0_done_sync2;
    reg trunk_done_sync1, trunk_done_sync2;
    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            h_done_sync1 <= 0; h_done_sync2 <= 0;
            d0_done_sync1 <= 0; d0_done_sync2 <= 0;
            trunk_done_sync1 <= 0; trunk_done_sync2 <= 0;
        end else begin
            h_done_sync1 <= h_done_tog_axi; h_done_sync2 <= h_done_sync1;
            d0_done_sync1 <= d0_done_tog_axi; d0_done_sync2 <= d0_done_sync1;
            trunk_done_sync1 <= trunk_done_tog_axi;
            trunk_done_sync2 <= trunk_done_sync1;
        end
    end
    assign head_commit_ready = h_done_sync2;
    assign d0_commit_ready = d0_done_sync2;

    // The *encoder's* done is not this module's business any more; what it
    // knows is that the last trunk-A write response has crossed back.  The
    // caller combines the two, because it is the caller that owns both halves.
    assign trunk_done = trunk_done_sync2;
    assign trunk_commit_axi = trunk_done_tog_axi;

    // This wall ledger ends only when the final trunk-A write response has
    // crossed back from the AXI domain, not when the last beat merely entered
    // the write FIFO. The core's compute-only wall remains internal for debug.
    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) trunk_store_commit_wait_cycles <= 0;
        else if (pe_done && !trunk_done_sync2)
            trunk_store_commit_wait_cycles <= trunk_store_commit_wait_cycles + 1;
    end

    // ------------------------------------------------------------------ AXI reader
    wire [127:0] rf_rd_data;
    wire rf_full, rf_empty, rf_rd_valid;
    reg h_rd_start_tog_pe, d0_rd_start_tog_pe;
    reg [31:0] frame_id_pe_q;
    reg rd_have_beat;
    reg [127:0] rd_beat_q;
    reg rd_half_q, rd_active_pe, rd_kind_pe;
    reg [15:0] rd_pixel_q;
    wire h_read_fire = head_read_valid && head_read_ready;
    wire d0_read_fire = d0_read_valid && d0_read_ready;
    reg rd_pending_pe;
    wire rf_rd_en = rd_active_pe && !rd_have_beat && !rd_pending_pe && !rf_empty;

    d64_async_fifo #(.DW(128), .AW(RD_FIFO_AW),
                     .ALMOST_FULL_LEVEL((1 << RD_FIFO_AW) - 2)) u_read_fifo (
        .wr_clk(clk_axi), .wr_rst_n(axi_rst_n),
        .wr_en(axi_rvalid && axi_rready), .wr_data(axi_rdata),
        .wr_full(rf_full), .wr_almost_full(), .wr_level(rf_wr_level),
        .rd_clk(clk_pe), .rd_rst_n(pe_rst_n), .rd_en(rf_rd_en),
        .rd_data(rf_rd_data), .rd_valid(rf_rd_valid), .rd_empty(rf_empty),
        .rd_level(rf_rd_level)
    );

    assign head_read_valid = rd_active_pe && !rd_kind_pe && rd_have_beat;
    assign head_read_x = rd_pixel_q[7:0];
    assign head_read_y = rd_pixel_q[15:8];
    assign head_read_c8 = rd_half_q ? rd_beat_q[127:64] : rd_beat_q[63:0];
    assign head_read_frame_id = frame_id_pe_q;
    assign head_read_sof = head_read_valid && (rd_pixel_q == 0) && !rd_half_q;
    assign head_read_eol = head_read_valid && (rd_pixel_q[7:0] == 8'd255) && rd_half_q;
    assign head_read_eof = head_read_valid && (rd_pixel_q == H_PIXELS - 1) && rd_half_q;

    assign d0_read_valid = rd_active_pe && rd_kind_pe && rd_have_beat;
    assign d0_read_x = {1'b0, rd_pixel_q[6:0]};
    assign d0_read_y = rd_pixel_q[13:7];
    assign d0_read_icg = rd_half_q;
    assign d0_read_c8 = rd_half_q ? rd_beat_q[127:64] : rd_beat_q[63:0];
    assign d0_read_frame_id = frame_id_pe_q;
    assign d0_read_sof = d0_read_valid && (rd_pixel_q == 0) && !rd_half_q;
    assign d0_read_eol = d0_read_valid && (rd_pixel_q[6:0] == 7'd127) && rd_half_q;
    assign d0_read_eof = d0_read_valid && (rd_pixel_q == D0_BEATS - 1) && rd_half_q;

    always @(posedge clk_pe or negedge pe_rst_n) begin
        if (!pe_rst_n) begin
            h_rd_start_tog_pe <= 0; d0_rd_start_tog_pe <= 0;
            frame_id_pe_q <= 0; rd_have_beat <= 0; rd_beat_q <= 0;
            rd_half_q <= 0; rd_active_pe <= 0; rd_kind_pe <= 0;
            rd_pixel_q <= 0; rd_pending_pe <= 0;
        end else begin
            // The frame id is the one the encoder captured, which is the same
            // number the source carried and is reachable from the boundary.
            if (head_store_valid && head_store_ready && head_store_sof)
                frame_id_pe_q <= head_store_frame_id;
            if (head_read_start) h_rd_start_tog_pe <= ~h_rd_start_tog_pe;
            if (d0_read_start) d0_rd_start_tog_pe <= ~d0_rd_start_tog_pe;

            if (head_read_start || d0_read_start) begin
                rd_active_pe <= 1'b1;
                rd_kind_pe <= d0_read_start;
                rd_have_beat <= 1'b0; rd_half_q <= 1'b0;
                rd_pixel_q <= 16'd0; rd_pending_pe <= 1'b0;
            end else begin
                if (rf_rd_en) rd_pending_pe <= 1'b1;
                if (rf_rd_valid) begin
                    rd_pending_pe <= 1'b0;
                    rd_beat_q <= rf_rd_data;
                    rd_have_beat <= 1'b1;
                end
                if (h_read_fire) begin
                    if (!rd_half_q) begin
                        rd_half_q <= 1'b1;
                        rd_pixel_q <= rd_pixel_q + 1'b1;
                    end else begin
                        rd_half_q <= 1'b0;
                        rd_have_beat <= 1'b0;
                        if (rd_pixel_q == H_PIXELS - 1) rd_active_pe <= 1'b0;
                        else rd_pixel_q <= rd_pixel_q + 1'b1;
                    end
                end else if (d0_read_fire) begin
                    if (!rd_half_q) rd_half_q <= 1'b1;
                    else begin
                        rd_half_q <= 1'b0;
                        rd_have_beat <= 1'b0;
                        if (rd_pixel_q == D0_BEATS - 1) rd_active_pe <= 1'b0;
                        else rd_pixel_q <= rd_pixel_q + 1'b1;
                    end
                end
            end
        end
    end

    reg h_rd_sync1, h_rd_sync2, h_rd_seen;
    reg d0_rd_sync1, d0_rd_sync2, d0_rd_seen;
    reg rd_active_axi, rd_kind_axi, rd_burst_active_axi;
    reg [7:0] rd_burst_q;
    reg [15:0] rd_beats_axi;
    reg [31:0] rd_base_axi;
    wire [31:0] rd_burst_offset = {13'd0, rd_burst_q, 11'd0};
    wire [15:0] rd_total = rd_kind_axi ? D0_BEATS : H_BEATS;

    // Reserve a complete 2 KiB burst before issuing its address. The synchronized
    // remote read pointer makes wr_level conservative; four beats add CDC margin.
    localparam RD_DEPTH = (1 << RD_FIFO_AW);
    localparam RD_BURST = 128;
    localparam RD_MARGIN = 4;
    wire rd_space_ok = (RD_DEPTH >= RD_BURST + RD_MARGIN) &&
                       (rf_wr_level <= RD_DEPTH - RD_BURST - RD_MARGIN);
    assign axi_arvalid = rd_active_axi && !rd_burst_active_axi && rd_space_ok;
    assign axi_araddr = rd_base_axi + rd_burst_offset;
    assign axi_arlen = 8'd127;
    assign axi_arsize = 3'd4;
    assign axi_arburst = 2'b01;
    // Keep ready asserted also between reserved bursts of this layer. This
    // avoids unnecessary ready edges at the vendor interface. No new burst may
    // return until its AR has passed the same whole-burst space reservation.
    assign axi_rready = rd_active_axi;

    always @(posedge clk_axi or negedge axi_rst_n) begin
        if (!axi_rst_n) begin
            h_rd_sync1 <= 0; h_rd_sync2 <= 0; h_rd_seen <= 0;
            d0_rd_sync1 <= 0; d0_rd_sync2 <= 0; d0_rd_seen <= 0;
            rd_active_axi <= 0; rd_kind_axi <= 0;
            rd_burst_active_axi <= 0; rd_burst_q <= 0;
            rd_beats_axi <= 0; rd_base_axi <= H_BASE;
            h_read_beats <= 0; d0_read_beats <= 0;
            h_read_bursts <= 0; d0_read_bursts <= 0;
            reader_axi_errors <= 0;
            read_fifo_overflow <= 0;
            rready_drop_in_burst <= 0;
        end else begin
            if (axi_rvalid && axi_rready && rf_full)
                read_fifo_overflow <= read_fifo_overflow + 1;
            if (rd_burst_active_axi && !axi_rready)
                rready_drop_in_burst <= rready_drop_in_burst + 1;
            h_rd_sync1 <= h_rd_start_tog_pe; h_rd_sync2 <= h_rd_sync1;
            d0_rd_sync1 <= d0_rd_start_tog_pe; d0_rd_sync2 <= d0_rd_sync1;
            if (h_rd_sync2 != h_rd_seen) begin
                h_rd_seen <= h_rd_sync2;
                rd_active_axi <= 1'b1; rd_kind_axi <= 1'b0;
                rd_burst_active_axi <= 1'b0; rd_burst_q <= 0;
                rd_beats_axi <= 0; rd_base_axi <= H_BASE;
            end else if (d0_rd_sync2 != d0_rd_seen) begin
                d0_rd_seen <= d0_rd_sync2;
                rd_active_axi <= 1'b1; rd_kind_axi <= 1'b1;
                rd_burst_active_axi <= 1'b0; rd_burst_q <= 0;
                rd_beats_axi <= 0; rd_base_axi <= D0_BASE;
            end

            if (axi_arvalid && axi_arready) begin
                if ((axi_araddr[10:0] != 0) || axi_arlen != 8'd127 ||
                    axi_arsize != 3'd4 || axi_arburst != 2'b01)
                    reader_axi_errors <= FAULT_STICKY ? 32'd1 : reader_axi_errors + 32'd1;
                rd_burst_active_axi <= 1'b1;
                if (rd_kind_axi) d0_read_bursts <= d0_read_bursts + 1;
                else h_read_bursts <= h_read_bursts + 1;
            end
            if (axi_rvalid && axi_rready) begin
                if (axi_rresp != 0 || (!rd_burst_active_axi &&
                    !(axi_arvalid && axi_arready)))
                    reader_axi_errors <= FAULT_STICKY ? 32'd1 : reader_axi_errors + 32'd1;
                if (rd_kind_axi) d0_read_beats <= d0_read_beats + 1;
                else h_read_beats <= h_read_beats + 1;
                rd_beats_axi <= rd_beats_axi + 1;
                if (axi_rlast) begin
                    rd_burst_active_axi <= 1'b0;
                    if (rd_beats_axi + 1 >= rd_total) rd_active_axi <= 1'b0;
                    else rd_burst_q <= rd_burst_q + 1'b1;
                end
            end
        end
    end

    // The one-frame wrapper deliberately holds the synchronized write-done
    // toggles as completion levels after their first edge. Reset re-arms a frame.
    // FSB4 passive same-physical-clock APP/AXI writer diagnostic.
    wire [31:0] fsb4_write_count = (wkind_q == KIND_H) ? h_wcount_q : (wkind_q == KIND_D0) ? d0_wcount_q : trunk_wcount_q;
    assign writer_debug = {axi_bresp,axi_bready,axi_bvalid,axi_wready,axi_wvalid,
        axi_awready,axi_awvalid,wf_empty,write_burst_reserved,
        trunk_done_sync2,d0_done_sync2,h_done_sync2,fsb4_write_count[14:0],wkind_q,wstate_q};
endmodule
