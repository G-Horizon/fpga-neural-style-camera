`timescale 1ns/1ps

// Integration of the frozen RGB256 stream, prepacked HEAD window, shared
// weight image, scheduler and exactly one universal MAC/requant bank.
module d64_head_top #(
    parameter WT_TILES = 247,
    parameter WT_WORD_W = 1024,
    parameter WT_ADDR_W = 8,
    // **The shape, threaded rather than assumed.**  The head's two boundary
    // modules -- the window that accepts the source and the controller that
    // schedules it -- have read `H` since `ce5bfc9`; this integration wrapper
    // was the one place above them that still passed nothing, so both sat on
    // their `H = 256` defaults no matter what the caller wanted.  `H = 4*ROWS`
    // is the graph's own arithmetic (DOWN1 is the image after two stride-2
    // downs), and `W` is carried because the controller's output boundary is a
    // width.  At `H = 256`, `W = 256` -- the defaults -- nothing changes.
    parameter integer H = 256,
    parameter integer W = 256
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [1:0]   pe_mode,

    input  wire         d64_valid,
    output wire         d64_ready,
    input  wire [31:0]  d64_frame_id,
    input  wire         d64_sof,
    input  wire [7:0]   d64_x0,
    input  wire [7:0]   d64_y,
    input  wire [383:0] d64_rgb888,
    input  wire         d64_eol,
    input  wire         d64_eof,

    input  wire                     wt_wr_en,
    input  wire [WT_ADDR_W-1:0]     wt_wr_addr,
    input  wire [WT_WORD_W-1:0]     wt_wr_data,
    // Resource-root audit read. Tie low in the deployed datapath; it lets the
    // integrated PNR root observe every bit and address of the full image.
    input  wire                     wt_audit_en,
    input  wire [WT_ADDR_W-1:0]     wt_audit_addr,
    output wire [WT_WORD_W-1:0]     wt_audit_data,

    output wire         head_out_valid,
    output wire [7:0]   head_out_x,
    output wire [7:0]   head_out_y,
    output wire [63:0]  head_out_c8,
    output wire [31:0]  head_out_frame_id,
    output wire         head_out_sof,
    output wire         head_out_eol,
    output wire         head_out_eof,

    output wire [31:0]  mac_slots,
    output wire [31:0]  head_compute_cycles,
    output reg  [31:0]  head_frame_cycles,
    output wire [31:0]  rq_wait,
    output wire [31:0]  rq_overrun,
    output wire [31:0]  mode_viol,
    output wire         done,

    output wire [2:0]   row_ready,
    output wire [23:0]  row_tags,
    output wire [31:0]  accepted_beats,
    output wire [31:0]  accepted_rows,
    output wire [31:0]  marker_violations,
    output wire [31:0]  read_before_ready
);

    wire [7:0] next_input_y;
    wire [1:0] next_input_slot;
    wire row_permit;
    wire [431:0] win_rd_data;   // {B, G, R}
    wire win_rd_en, win_rd_valid;
    wire [7:0] win_rd_row;
    wire [3:0] win_rd_group;
    wire [1:0] win_rd_sub;
    wire [7:0] ctrl_wt_rd_addr;
    wire ctrl_wt_rd_en;
    wire [WT_WORD_W-1:0] wt_rd_data;
    wire [WT_WORD_W-1:0] wt_store_data;
    wire [7:0] wt_store_addr = wt_audit_en ? wt_audit_addr : ctrl_wt_rd_addr;
    wire wt_store_rd_en = wt_audit_en | ctrl_wt_rd_en;
    assign wt_rd_data = wt_audit_en ? {WT_WORD_W{1'b0}} : wt_store_data;
    assign wt_audit_data = wt_store_data;
    wire [31:0] captured_frame_id;
    wire captured_frame_id_valid;
    reg frame_measure_active;

    // Frame latency starts when the frozen source handshake accepts SOF and
    // ends at the cycle in which the final EOF pulse is emitted. EOF is visible
    // after its producing edge, so the counter holds its just-counted value
    // when that pulse is sampled on the following edge.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_measure_active <= 1'b0;
            head_frame_cycles <= 32'd0;
        end else if (d64_valid && d64_ready && d64_sof) begin
            frame_measure_active <= 1'b1;
            head_frame_cycles <= 32'd1;
        end else if (frame_measure_active) begin
            if (head_out_valid && head_out_eof)
                frame_measure_active <= 1'b0;
            else
                head_frame_cycles <= head_frame_cycles + 32'd1;
        end
    end

    d64_head_window #(.H(H), .W(W)) u_window (
        .clk(clk), .rst_n(rst_n),
        .d64_valid(d64_valid), .d64_ready(d64_ready),
        .d64_frame_id(d64_frame_id), .d64_sof(d64_sof),
        .d64_x0(d64_x0), .d64_y(d64_y), .d64_rgb888(d64_rgb888),
        .d64_eol(d64_eol), .d64_eof(d64_eof),
        .row_permit(row_permit), .next_input_y(next_input_y),
        .next_input_slot(next_input_slot), .row_ready(row_ready),
        .row_tags(row_tags), .frame_id(captured_frame_id),
        .frame_id_valid(captured_frame_id_valid),
        .accepted_beats(accepted_beats), .accepted_rows(accepted_rows),
        .marker_violations(marker_violations),
        .read_before_ready(read_before_ready),
        .rd_en(win_rd_en), .rd_row(win_rd_row), .rd_group(win_rd_group), .rd_sub(win_rd_sub),
        .rd_data(win_rd_data),
        .rd_valid(win_rd_valid)
    );

    d64_weight_store #(
        .TILES(WT_TILES), .WORD_W(WT_WORD_W), .ADDR_W(WT_ADDR_W)
    ) u_weights (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wt_wr_en), .wr_addr(wt_wr_addr), .wr_data(wt_wr_data),
        .rd_en(wt_store_rd_en), .rd_addr(wt_store_addr), .rd_data(wt_store_data)
    );

    d64_head_ctrl #(.H(H), .W(W)) u_ctrl (
        .clk(clk), .rst_n(rst_n), .pe_mode(pe_mode),
        .row_ready(row_ready), .row_tags(row_tags),
        .next_input_y(next_input_y), .next_input_slot(next_input_slot),
        .row_permit(row_permit),
        .win_rd_en(win_rd_en), .win_rd_row(win_rd_row),
        .win_rd_group(win_rd_group), .win_rd_sub(win_rd_sub),
        .win_rd_data(win_rd_data), .win_rd_valid(win_rd_valid),
        .wt_rd_en(ctrl_wt_rd_en), .wt_rd_addr(ctrl_wt_rd_addr),
        .wt_rd_data(wt_rd_data),
        .frame_id(captured_frame_id), .frame_id_valid(captured_frame_id_valid),
        .head_out_valid(head_out_valid), .head_out_x(head_out_x),
        .head_out_y(head_out_y), .head_out_c8(head_out_c8),
        .head_out_frame_id(head_out_frame_id), .head_out_sof(head_out_sof),
        .head_out_eol(head_out_eol), .head_out_eof(head_out_eof),
        .mac_slots(mac_slots), .head_compute_cycles(head_compute_cycles),
        .rq_wait(rq_wait), .rq_overrun(rq_overrun),
        .mode_viol(mode_viol), .done(done)
    );
endmodule
