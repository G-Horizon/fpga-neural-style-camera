`timescale 1ns/1ps

// D3c6-c: HEAD, down0, and down1 are clients of one physical d64_mac128.
// Full-frame layer boundaries are explicit stream/store interfaces so the
// functional testbench can materialize H and D0 without inferring on-chip frame
// memories or binding this controller to a particular DDR implementation.
module d64_encoder #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter WT_INIT_FILE = "",
    parameter CFG_INIT_FILE = "",
    // D3c6-e4.  When set, this encoder owns neither its multiplier array
    // nor its weight store: both come in over the seam below and are shared
    // with every other scheduler in the network.  At 0 -- the default, and
    // the configuration the board closed on -- nothing about this module
    // changes and the seam is inert.
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0,
    parameter integer HEAD_STORE_CREDIT_ENABLE = 0,
    // **The shape of the image this encoder is driven at.**  `H = 4*ROWS`,
    // `ROWS` being the residual block's row count: DOWN1 is the image after
    // two stride-2 downs, so a `ROWS`-row residual block is a `4*ROWS`-row
    // image.  Every count this module's front half produces is a function of
    // it -- the head accepts `H` rows, the head store is `(W*H)/2` 128-bit
    // beats over `H` bursts, DOWN0's is `(W*H)/4` over `H/2` -- and until now
    // both boundary modules below sat on their 256 defaults while the board's
    // gate compared the *measured* counters against expressions in `H`.
    // The parameters are inherited by name, so the default is the shipped
    // 256-row square and nothing moves at it.
    parameter integer H = 256,
    parameter integer W = 256
) (
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,

    // Frozen camera/CNN boundary: 16 adjacent RGB888 pixels per beat.
    input  wire          d64_valid,
    output wire          d64_ready,
    input  wire [31:0]   d64_frame_id,
    input  wire          d64_sof,
    input  wire [7:0]    d64_x0,
    input  wire [7:0]    d64_y,
    input  wire [383:0]  d64_rgb888,
    input  wire          d64_eol,
    input  wire          d64_eof,

    // One 247-tile weight image and one downs bias/requant table.
    input  wire          wt_wr_en,
    input  wire [7:0]    wt_wr_addr,
    input  wire [1023:0] wt_wr_data,
    input  wire          cfg_wr_en,
    input  wire [5:0]    cfg_wr_addr,
    // ---- D3c6-e4: the shared-PE seam, `EXTERNAL_PE` only -------------------
    // HEAD and down0/down1 are **already arbitrated inside this module**: the
    // owner mux below picks which client's operands reach the array and which
    // client reads its responses.  So what crosses this port is that mux's
    // output, and not two client interfaces.  The network does not need to
    // know this module has two clients; it needs to know it has one array's
    // worth of work in it, which is what this is.
    //
    // `pe_mode` stays an ordinary output.  A layer is a shape and a shape is a
    // mode, so whoever owns the array drives it -- and this module is still the
    // one that knows which of its three layers is running.
    output wire                pe_req_slot_valid,
    output wire [1407:0]       pe_req_act,
    output wire [1023:0]       pe_req_wgt,
    output wire                pe_req_pix_start,
    output wire [4095:0]       pe_req_bias,
    output wire                pe_req_rq_start,
    output wire [255:0]        pe_req_rq_m,
    output wire [79:0]         pe_req_rq_shift,
    output wire [9:0]          pe_req_rq_zero,
    output wire [9:0]          pe_req_rq_qmin,
    output wire [9:0]          pe_req_rq_qmax,
    input  wire                pe_rsp_rq_ready,
    input  wire                pe_rsp_rq_ready_n,
    input  wire [79:0]         pe_rsp_y,
    input  wire [7:0]          pe_rsp_y_valid,
    input  wire                pe_rsp_rq_busy,
    input  wire                pe_rsp_rq_done,
    input  wire                pe_rsp_rq_overrun,
    input  wire [31:0]         pe_rsp_mode_viol,
    input  wire [4095:0]       pe_rsp_acc,
    input  wire [4095:0]       pe_rsp_snapshot,
    output wire                pe_wt_rd_en,
    output wire [7:0]          pe_wt_rd_addr,
    input  wire [1023:0]       pe_wt_rd_data,
    input  wire [63:0]   cfg_wr_data,

    // H feature-store write port. HEAD emits one complete 8-channel pixel per
    // valid cycle and is intentionally not backpressured by a layer store.
    output wire          head_store_wr_valid,
    input  wire          head_store_wr_ready,
    input  wire          head_phase_ready,
    output wire [7:0]    head_store_wr_x,
    output wire [7:0]    head_store_wr_y,
    output wire [63:0]   head_store_wr_c8,
    output wire [31:0]   head_store_wr_frame_id,
    output wire          head_store_wr_sof,
    output wire          head_store_wr_eol,
    output wire          head_store_wr_eof,
    input  wire          head_store_commit_ready,

    // Materialized H reader, started only after HEAD output/PE drain and store
    // commit. One U8 feature pixel per beat.
    output wire          head_read_start,
    input  wire          head_read_valid,
    output wire          head_read_ready,
    input  wire [7:0]    head_read_x,
    input  wire [7:0]    head_read_y,
    input  wire [63:0]   head_read_c8,
    input  wire [31:0]   head_read_frame_id,
    input  wire          head_read_sof,
    input  wire          head_read_eol,
    input  wire          head_read_eof,

    // D0 feature-store write/commit port. The down engine's own output queue
    // supplies backpressure until the behavioral or future DDR store accepts.
    output wire          down0_store_wr_valid,
    input  wire          down0_store_wr_ready,
    output wire [7:0]    down0_store_wr_x,
    output wire [7:0]    down0_store_wr_y,
    output wire [1:0]    down0_store_wr_word,
    output wire [79:0]   down0_store_wr_data,
    output wire [31:0]   down0_store_wr_frame_id,
    output wire          down0_store_wr_sof,
    output wire          down0_store_wr_eol,
    output wire          down0_store_wr_eof,
    input  wire          down0_store_commit_ready,

    // Materialized D0 reader, started after the D0 store commit.
    output wire          down0_read_start,
    input  wire          down0_read_valid,
    output wire          down0_read_ready,
    input  wire [7:0]    down0_read_x,
    input  wire [7:0]    down0_read_y,
    input  wire          down0_read_icg,
    input  wire [63:0]   down0_read_c8,
    input  wire [31:0]   down0_read_frame_id,
    input  wire          down0_read_sof,
    input  wire          down0_read_eol,
    input  wire          down0_read_eof,

    // Final D1 tensor stream. Format is 64x64x32 U10, four 8-channel words per
    // pixel, directly matching the frozen trunk block0 input.
    output wire          out_valid,
    input  wire          out_ready,
    output wire [7:0]    out_x,
    output wire [7:0]    out_y,
    output wire [1:0]    out_word,
    output wire [79:0]   out_data,
    output wire [31:0]   out_frame_id,
    output wire          out_sof,
    output wire          out_eol,
    output wire          out_eof,

    output wire          busy,
    output wire          done,
    output wire [3:0]    state,
    output wire [1:0]    pe_mode,
    output wire          mac_mode_safe,
    output wire [31:0]   mode_viol,
    output reg  [31:0]   layer_mode_viol,
    output reg  [31:0]   frame_id_violations,
    output reg  [31:0]   head_store_overflow,
    output wire [31:0]   head_mac_slots,
    output wire [31:0]   down_mac_slots,
    output wire [31:0]   head_rq_overrun,
    output wire [31:0]   down_rq_overrun,
    output wire [31:0]   head_rq_wait,
    output wire [31:0]   down_rq_wait,
    output wire [31:0]   down_read_before_ready,
    output reg  [31:0]   head_layer_cycles,
    output reg  [31:0]   down0_layer_cycles,
    output reg  [31:0]   down1_layer_cycles,
    output reg  [31:0]   head_compute_cycles,
    output reg  [31:0]   down0_compute_cycles,
    output reg  [31:0]   down1_compute_cycles,
    output reg  [31:0]   encoder_compute_cycles,
    output reg  [31:0]   encoder_store_wait_cycles,
    output reg  [31:0]   encoder_wall_cycles,
    output wire [31:0]   head_accepted_beats,
    output wire [31:0]   head_accepted_rows,
    output wire [31:0]   head_marker_violations,
    output wire [31:0]   head_read_before_ready
);
    localparam [3:0] ST_IDLE              = 4'd0,
                     ST_HEAD              = 4'd1,
                     ST_HEAD_DRAIN        = 4'd2,
                     ST_HEAD_STORE_COMMIT = 4'd3,
                     ST_MODE_SWITCH       = 4'd4,
                     ST_DOWN0_START       = 4'd5,
                     ST_DOWN0             = 4'd6,
                     ST_DOWN0_DRAIN       = 4'd7,
                     ST_D0_STORE_COMMIT   = 4'd8,
                     ST_DOWN1_START       = 4'd9,
                     ST_DOWN1             = 4'd10,
                     ST_DOWN1_DRAIN       = 4'd11,
                     ST_DONE              = 4'd12;

    reg [3:0] state_q;
    reg head_owner_q,down_owner_q;
    reg [31:0] frame_id_q;
    reg frame_id_seen;
    reg wall_active;
    reg down0_active, down1_active;
    reg [31:0] head_compute_prev, down_compute_prev;
    assign state = state_q;
    assign busy = (state_q != ST_IDLE) && (state_q != ST_DONE);
    assign done = (state_q == ST_DONE);
    assign pe_mode=head_owner_q ? 2'd3 : 2'd0;

    // Parallel state bits update on precisely the same transition edge as
    // state_q. Wide PE fields no longer traverse its binary-state decoder.
    wire head_owner=head_owner_q;
    wire down_owner=down_owner_q;

    wire [2:0] row_ready;
    wire [23:0] row_tags;
    wire row_permit;
    wire [7:0] next_input_y;
    wire [1:0] next_input_slot;
    wire [431:0] win_rd_data;   // {B, G, R}
    wire win_rd_en, win_rd_valid;
    wire [7:0] win_rd_row;
    wire [3:0] win_rd_group;
    wire [1:0] win_rd_sub;
    wire [31:0] captured_frame_id;
    wire captured_frame_id_valid;
    wire [31:0] head_mac_compute;
    wire head_done, head_mac_safe;
    wire [7:0] head_x, head_y;
    wire [63:0] head_c8;
    wire [31:0] head_frame_id;
    wire head_out_valid, head_sof, head_eol, head_eof;
    wire [31:0] head_mode_viol;
    wire head_wt_rd_en;
    wire [7:0] head_wt_rd_addr;
    wire [1023:0] head_wt_rd_data;
    wire [1407:0] h_pe_act;
    wire [1023:0] h_pe_wgt;
    wire h_pe_slot, h_pe_pix_start, h_pe_rq_start;
    wire [4095:0] h_pe_bias, h_pe_acc, h_pe_snapshot;
    wire [255:0] h_pe_m;
    wire [79:0] h_pe_shift, h_pe_y;
    wire [9:0] h_pe_zero, h_pe_qmin, h_pe_qmax;
    wire [7:0] h_pe_y_valid;
    wire h_pe_rq_ready, h_pe_rq_ready_n, h_pe_rq_busy, h_pe_rq_done;
    wire h_pe_rq_overrun;
    wire [31:0] h_pe_mode_viol;

    wire down_busy, down_done, down_mac_safe;
    wire down_in_ready, down_out_valid;
    wire [7:0] down_out_x, down_out_y;
    wire [1:0] down_out_word;
    wire [79:0] down_out_data;
    wire [31:0] down_out_frame_id;
    wire down_out_sof, down_out_eol, down_out_eof;
    wire [31:0] down_mode_viol, down_layer_mode_viol;
    wire [31:0] down_slots, down_rq_accepts, down_compute, down_frame;
    wire [31:0] down_read_ready_count, down_rq_wait_count;
    wire [31:0] down_overrun;
    wire [31:0] down_output_order_violations;
    wire [31:0] down_input_marker_violations;
    wire [31:0] down_input_words, down_input_rows;
    wire [31:0] down_row_wait, down_credit_wait, down_no_slot_wait;
    wire [511:0] down_snapshot;
    wire [4:0] down_row_ready;
    wire [39:0] down_row_tags;
    wire [1407:0] d_pe_act;
    wire [1023:0] d_pe_wgt;
    wire d_pe_slot, d_pe_pix_start, d_pe_rq_start;
    wire [4095:0] d_pe_bias, d_pe_acc, d_pe_snapshot;
    wire [255:0] d_pe_m;
    wire [79:0] d_pe_shift, d_pe_y;
    wire [9:0] d_pe_zero, d_pe_qmin, d_pe_qmax;
    wire [7:0] d_pe_y_valid;
    wire d_pe_rq_ready, d_pe_rq_ready_n, d_pe_rq_busy, d_pe_rq_done;
    wire d_pe_rq_overrun;
    wire [31:0] d_pe_mode_viol;
    wire down_wt_rd_en;
    wire [7:0] down_wt_rd_addr;
    wire [1023:0] down_wt_rd_data;

    wire [7:0] wt_rd_addr = head_owner ? head_wt_rd_addr : down_wt_rd_addr;
    wire wt_rd_en = (head_owner && head_wt_rd_en) ||
                    (down_owner && down_wt_rd_en);
    wire [1023:0] wt_store_q;
    wire [1023:0] wt_store_rd_data = EXTERNAL_PE ? wt_store_q : (wt_rd_en ? wt_store_q : 1024'd0);
    assign head_wt_rd_data = EXTERNAL_PE ? wt_store_rd_data : (head_owner ? wt_store_rd_data : 1024'd0);
    assign down_wt_rd_data = EXTERNAL_PE ? wt_store_rd_data : (down_owner ? wt_store_rd_data : 1024'd0);

    wire window_d64_ready;
    d64_head_window #(.H(H), .W(W)) u_head_window (
        .clk(clk), .rst_n(rst_n),
        .d64_valid(d64_valid && (state_q == ST_HEAD)), .d64_ready(window_d64_ready),
        .d64_frame_id(d64_frame_id), .d64_sof(d64_sof),
        .d64_x0(d64_x0), .d64_y(d64_y), .d64_rgb888(d64_rgb888),
        .d64_eol(d64_eol), .d64_eof(d64_eof),
        .row_permit(row_permit), .next_input_y(next_input_y),
        .next_input_slot(next_input_slot), .row_ready(row_ready),
        .row_tags(row_tags), .frame_id(captured_frame_id),
        .frame_id_valid(captured_frame_id_valid),
        .accepted_beats(head_accepted_beats), .accepted_rows(head_accepted_rows),
        .marker_violations(head_marker_violations),
        .read_before_ready(head_read_before_ready),
        .rd_en(win_rd_en), .rd_row(win_rd_row), .rd_group(win_rd_group), .rd_sub(win_rd_sub),
        .rd_data(win_rd_data),
        .rd_valid(win_rd_valid)
    );

    assign d64_ready = (state_q == ST_HEAD) && window_d64_ready;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_weight_store
            // `d64_weight_store`'s read is `rd_en ? q : 0`, and the store out
            // there applies the same enable to the same address, so the raw
            // output is already qualified and nothing has to be gated twice.
            assign wt_store_q = pe_wt_rd_data;
        end else begin : g_local_weight_store
            d64_weight_store #(.TILES(247), .WORD_W(1024), .ADDR_W(8),
                               .INIT_FILE(WT_INIT_FILE)) u_weight_store (
                .clk(clk), .rst_n(rst_n),
                .wr_en(wt_wr_en), .wr_addr(wt_wr_addr), .wr_data(wt_wr_data),
                .rd_en(wt_rd_en), .rd_addr(wt_rd_addr), .rd_data(wt_store_q)
            );
        end
    endgenerate

    d64_head_ctrl #(.RUNTIME_PROFILE(RUNTIME_PROFILE), .HEAD_STORE_CREDIT_ENABLE(HEAD_STORE_CREDIT_ENABLE), .EXTERNAL_PE(1), .H(H), .W(W)) u_head_client (
        .head_phase_ready(head_phase_ready),
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .clk(clk), .rst_n(rst_n), .pe_mode(pe_mode),
        .pe_req_slot_valid(h_pe_slot), .pe_req_act(h_pe_act),
        .pe_req_wgt(h_pe_wgt), .pe_req_pix_start(h_pe_pix_start),
        .pe_req_bias(h_pe_bias), .pe_req_rq_start(h_pe_rq_start),
        .pe_req_rq_m(h_pe_m), .pe_req_rq_shift(h_pe_shift),
        .pe_req_rq_zero(h_pe_zero), .pe_req_rq_qmin(h_pe_qmin),
        .pe_req_rq_qmax(h_pe_qmax),
        .pe_rsp_rq_ready(h_pe_rq_ready), .pe_rsp_rq_ready_n(h_pe_rq_ready_n),
        .pe_rsp_y(h_pe_y), .pe_rsp_y_valid(h_pe_y_valid),
        .pe_rsp_rq_busy(h_pe_rq_busy), .pe_rsp_rq_done(h_pe_rq_done),
        .pe_rsp_rq_overrun(h_pe_rq_overrun), .pe_rsp_mode_viol(h_pe_mode_viol),
        .pe_rsp_acc(h_pe_acc), .pe_rsp_snapshot(h_pe_snapshot),
        .mac_mode_safe(head_mac_safe),
        .row_ready(row_ready), .row_tags(row_tags),
        .next_input_y(next_input_y), .next_input_slot(next_input_slot),
        .row_permit(row_permit),
        .win_rd_en(win_rd_en), .win_rd_row(win_rd_row),
        .win_rd_group(win_rd_group), .win_rd_sub(win_rd_sub),
        .win_rd_data(win_rd_data), .win_rd_valid(win_rd_valid),
        .wt_rd_en(head_wt_rd_en), .wt_rd_addr(head_wt_rd_addr),
        .wt_rd_data(head_wt_rd_data),
        .frame_id(captured_frame_id), .frame_id_valid(captured_frame_id_valid),
        .head_out_valid(head_out_valid), .head_out_x(head_x), .head_out_y(head_y),
        .head_out_c8(head_c8), .head_out_frame_id(head_frame_id),
        .head_out_sof(head_sof), .head_out_eol(head_eol), .head_out_eof(head_eof),
        .mac_slots(head_mac_slots), .head_compute_cycles(head_mac_compute),
        .rq_wait(head_rq_wait), .rq_overrun(head_rq_overrun),
        .mode_viol(head_mode_viol), .done(head_done)
    );

    // d64_head_window owns the source-ready signal; keep the state gate outside
    // the module so a repeated frame cannot start while the previous encoder is
    // still materializing its layer boundaries.
    assign head_store_wr_valid = head_owner && head_out_valid;
    assign head_store_wr_x = head_x;
    assign head_store_wr_y = head_y;
    assign head_store_wr_c8 = head_c8;
    assign head_store_wr_frame_id = head_frame_id;
    assign head_store_wr_sof = head_sof;
    assign head_store_wr_eol = head_eol;
    assign head_store_wr_eof = head_eof;

    wire down_start = (state_q == ST_DOWN0_START) || (state_q == ST_DOWN1_START);
    wire layer_down1 = (state_q == ST_DOWN1_START) || (state_q == ST_DOWN1) ||
                       (state_q == ST_DOWN1_DRAIN);
    wire down_in_valid = ((state_q == ST_DOWN0) || (state_q == ST_DOWN0_START)) ?
                         head_read_valid :
                         (((state_q == ST_DOWN1) || (state_q == ST_DOWN1_START)) ?
                          down0_read_valid : 1'b0);
    wire [7:0] down_in_x = layer_down1 ? down0_read_x : head_read_x;
    wire [7:0] down_in_y = layer_down1 ? down0_read_y : head_read_y;
    wire down_in_icg = layer_down1 ? down0_read_icg : 1'b0;
    wire [63:0] down_in_c8 = layer_down1 ? down0_read_c8 : head_read_c8;
    wire [31:0] down_in_frame_id = layer_down1 ? down0_read_frame_id :
                                                    head_read_frame_id;
    wire down_in_sof = layer_down1 ? down0_read_sof : head_read_sof;
    wire down_in_eol = layer_down1 ? down0_read_eol : head_read_eol;
    wire down_in_eof = layer_down1 ? down0_read_eof : head_read_eof;
    assign head_read_start = (state_q == ST_DOWN0_START);
    assign down0_read_start = (state_q == ST_DOWN1_START);
    assign head_read_ready = ((state_q == ST_DOWN0) ||
                              (state_q == ST_DOWN0_START)) && down_in_ready;
    assign down0_read_ready = ((state_q == ST_DOWN1) ||
                               (state_q == ST_DOWN1_START)) && down_in_ready;
    wire down0_store_active = (state_q == ST_DOWN0) ||
                               (state_q == ST_DOWN0_DRAIN);
    wire final_stream_active = (state_q == ST_DOWN1) ||
                               (state_q == ST_DOWN1_DRAIN);
    wire down_output_ready = down0_store_active ? down0_store_wr_ready :
                             final_stream_active ? out_ready : 1'b1;
    assign down0_store_wr_valid = down0_store_active && down_out_valid;
    assign down0_store_wr_x = down_out_x;
    assign down0_store_wr_y = down_out_y;
    assign down0_store_wr_word = down_out_word;
    assign down0_store_wr_data = down_out_data;
    assign down0_store_wr_frame_id = down_out_frame_id;
    assign down0_store_wr_sof = down_out_sof;
    assign down0_store_wr_eol = down_out_eol;
    assign down0_store_wr_eof = down_out_eof;
    assign out_valid = final_stream_active && down_out_valid;
    assign out_x = down_out_x;
    assign out_y = down_out_y;
    assign out_word = down_out_word;
    assign out_data = down_out_data;
    assign out_frame_id = down_out_frame_id;
    assign out_sof = down_out_sof;
    assign out_eol = down_out_eol;
    assign out_eof = down_out_eof;

    d64_down_engine #(.RUNTIME_PROFILE(RUNTIME_PROFILE), .EXTERNAL_PE(1), .H(H), .W_TILES(247), .W_WORD_W(1024),
                      .W_ADDR_W(8), .CFG_WORDS(48), .FIFO_DEPTH(8),
                      .CFG_INIT_FILE(CFG_INIT_FILE))
    u_down_client (
        .profile_style(profile_style), .profile_trunk_zero(profile_trunk_zero),
        .clk(clk), .rst_n(rst_n), .start(down_start),
        .layer_down1(layer_down1), .pe_mode(pe_mode),
        .pe_req_slot_valid(d_pe_slot), .pe_req_act(d_pe_act),
        .pe_req_wgt(d_pe_wgt), .pe_req_pix_start(d_pe_pix_start),
        .pe_req_bias(d_pe_bias), .pe_req_rq_start(d_pe_rq_start),
        .pe_req_rq_m(d_pe_m), .pe_req_rq_shift(d_pe_shift),
        .pe_req_rq_zero(d_pe_zero), .pe_req_rq_qmin(d_pe_qmin),
        .pe_req_rq_qmax(d_pe_qmax),
        .pe_rsp_rq_ready(d_pe_rq_ready), .pe_rsp_rq_ready_n(d_pe_rq_ready_n),
        .pe_rsp_y(d_pe_y), .pe_rsp_y_valid(d_pe_y_valid),
        .pe_rsp_rq_busy(d_pe_rq_busy), .pe_rsp_rq_done(d_pe_rq_done),
        .pe_rsp_rq_overrun(d_pe_rq_overrun), .pe_rsp_mode_viol(d_pe_mode_viol),
        .pe_rsp_acc(d_pe_acc), .pe_rsp_snapshot(d_pe_snapshot),
        .pe_wt_rd_en(down_wt_rd_en), .pe_wt_rd_addr(down_wt_rd_addr),
        .pe_wt_rd_data(down_wt_rd_data),
        .busy(down_busy), .done(down_done), .mac_mode_safe(down_mac_safe),
        .wt_we(1'b0), .wt_wa(8'd0), .wt_wd(1024'd0),
        .cfg_we(cfg_wr_en), .cfg_wa(cfg_wr_addr), .cfg_wd(cfg_wr_data),
        .in_valid(down_in_valid), .in_ready(down_in_ready),
        .in_x(down_in_x), .in_y(down_in_y), .in_icg(down_in_icg),
        .in_c8(down_in_c8), .in_sof(down_in_sof), .in_eol(down_in_eol),
        .in_eof(down_in_eof), .in_frame_id(down_in_frame_id),
        .out_valid(down_out_valid), .out_ready(down_output_ready),
        .out_x(down_out_x), .out_y(down_out_y), .out_word(down_out_word),
        .out_data(down_out_data), .out_frame_id(down_out_frame_id),
        .out_sof(down_out_sof), .out_eol(down_out_eol), .out_eof(down_out_eof),
        .input_words(down_input_words), .input_rows(down_input_rows),
        .input_marker_violations(down_input_marker_violations),
        .output_order_violations(down_output_order_violations),
        .output_words(), .mac_slots(down_slots), .rq_accepts(down_rq_accepts),
        .rq_wait(down_rq_wait_count), .rq_overrun(down_overrun),
        .mode_viol(down_mode_viol), .layer_mode_viol(down_layer_mode_viol),
        .read_before_ready(down_read_ready_count),
        .engine_compute_cycles(down_compute), .engine_frame_cycles(down_frame),
        .row_start_wait_cycles(down_row_wait), .rq_wait_cycles(down_rq_wait),
        .output_credit_wait_cycles(down_credit_wait),
        .input_no_free_slot_cycles(down_no_slot_wait),
        .debug_rq_accept_event(), .debug_mac_snapshot(down_snapshot),
        .row_ready(down_row_ready), .row_tags(down_row_tags)
    );

    wire pe_slot_valid = head_owner ? h_pe_slot : (down_owner ? d_pe_slot : 1'b0);
    wire [1407:0] pe_act = head_owner ? h_pe_act :
                              (down_owner ? d_pe_act : 1408'd0);
    wire [1023:0] pe_wgt = head_owner ? h_pe_wgt :
                               (down_owner ? d_pe_wgt : 1024'd0);
    wire pe_pix_start = head_owner ? h_pe_pix_start :
                          (down_owner ? d_pe_pix_start : 1'b0);
    wire [4095:0] pe_bias = head_owner ? h_pe_bias :
                                (down_owner ? d_pe_bias : 4096'd0);
    wire pe_rq_start = head_owner ? h_pe_rq_start :
                         (down_owner ? d_pe_rq_start : 1'b0);
    wire [255:0] pe_rq_m = head_owner ? h_pe_m :
                               (down_owner ? d_pe_m : 256'd0);
    wire [79:0] pe_rq_shift = head_owner ? h_pe_shift :
                                  (down_owner ? d_pe_shift : 80'd0);
    wire [9:0] pe_rq_zero = head_owner ? h_pe_zero :
                                (down_owner ? d_pe_zero : 10'd0);
    wire [9:0] pe_rq_qmin = head_owner ? h_pe_qmin :
                                (down_owner ? d_pe_qmin : 10'd0);
    wire [9:0] pe_rq_qmax = head_owner ? h_pe_qmax :
                                (down_owner ? d_pe_qmax : 10'd0);
    wire pe_rq_ready, pe_rq_ready_n, pe_rq_busy, pe_rq_done, pe_rq_overrun;
    wire [79:0] pe_y;
    wire [7:0] pe_y_valid;
    wire [4095:0] pe_acc, pe_snapshot;
    wire [31:0] pe_mode_viol;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_pe
            // The whole response side, straight from the seam.  `pe_mode` is
            // not here: it is this module's own output either way.
            assign pe_rq_ready    = pe_rsp_rq_ready;
            assign pe_rq_ready_n  = pe_rsp_rq_ready_n;
            assign pe_y           = pe_rsp_y;
            assign pe_y_valid     = pe_rsp_y_valid;
            assign pe_rq_busy     = pe_rsp_rq_busy;
            assign pe_rq_done     = pe_rsp_rq_done;
            assign pe_rq_overrun  = pe_rsp_rq_overrun;
            assign pe_mode_viol   = pe_rsp_mode_viol;
            assign pe_acc         = pe_rsp_acc;
            assign pe_snapshot    = pe_rsp_snapshot;
        end else begin : g_local_pe
            d64_mac128 #(.ACT_W(11), .WGT_W(8), .ACC_W(32), .M_W(16),
                         .SHIFT_W(5), .OUT_W(10), .RQ_LANES(8), .LANES(128),
                         .OC_MAX(16), .RQ_LOOKAHEAD(3)) u_shared_mac (
                .clk(clk), .rst_n(rst_n), .mode(pe_mode),
                .slot_valid(pe_slot_valid), .act(pe_act), .wgt(pe_wgt),
                .pix_start(pe_pix_start), .bias(pe_bias),
                .rq_start(pe_rq_start), .rq_ready(pe_rq_ready),
                .rq_ready_n(pe_rq_ready_n), .rq_m(pe_rq_m),
                .rq_shift(pe_rq_shift), .rq_zero(pe_rq_zero),
                .rq_qmin(pe_rq_qmin), .rq_qmax(pe_rq_qmax),
                .y(pe_y), .y_valid(pe_y_valid), .rq_busy(pe_rq_busy),
                .rq_done(pe_rq_done), .rq_overrun(pe_rq_overrun),
                .mode_viol(pe_mode_viol), .acc_o(pe_acc), .rq_snap_o(pe_snapshot)
            );
        end
    endgenerate

    // ---- the request side, either way --------------------------------------
    // Driven from the owner mux in both configurations, because the mux is this
    // module's and stays this module's: what `EXTERNAL_PE` moves is the array,
    // not the decision about which client is using it.
    assign pe_req_slot_valid = pe_slot_valid;
    assign pe_req_act        = pe_act;
    assign pe_req_wgt        = pe_wgt;
    assign pe_req_pix_start  = pe_pix_start;
    assign pe_req_bias       = pe_bias;
    assign pe_req_rq_start   = pe_rq_start;
    assign pe_req_rq_m       = pe_rq_m;
    assign pe_req_rq_shift   = pe_rq_shift;
    assign pe_req_rq_zero    = pe_rq_zero;
    assign pe_req_rq_qmin    = pe_rq_qmin;
    assign pe_req_rq_qmax    = pe_rq_qmax;
    assign pe_wt_rd_en       = wt_rd_en;
    assign pe_wt_rd_addr     = wt_rd_addr;

    assign h_pe_rq_ready = head_owner ? pe_rq_ready : 1'b1;
    assign h_pe_rq_ready_n = head_owner ? pe_rq_ready_n : 1'b1;
    assign h_pe_y = head_owner ? pe_y : 80'd0;
    assign h_pe_y_valid = head_owner ? pe_y_valid : 8'd0;
    assign h_pe_rq_busy = head_owner && pe_rq_busy;
    assign h_pe_rq_done = head_owner && pe_rq_done;
    assign h_pe_rq_overrun = head_owner && pe_rq_overrun;
    assign h_pe_mode_viol = head_owner ? pe_mode_viol : 32'd0;
    assign h_pe_acc = head_owner ? pe_acc : 4096'd0;
    assign h_pe_snapshot = head_owner ? pe_snapshot : 4096'd0;
    assign d_pe_rq_ready = down_owner ? pe_rq_ready : 1'b1;
    assign d_pe_rq_ready_n = down_owner ? pe_rq_ready_n : 1'b1;
    assign d_pe_y = down_owner ? pe_y : 80'd0;
    assign d_pe_y_valid = down_owner ? pe_y_valid : 8'd0;
    assign d_pe_rq_busy = down_owner && pe_rq_busy;
    assign d_pe_rq_done = down_owner && pe_rq_done;
    assign d_pe_rq_overrun = down_owner && pe_rq_overrun;
    assign d_pe_mode_viol = down_owner ? pe_mode_viol : 32'd0;
    assign d_pe_acc = down_owner ? pe_acc : 4096'd0;
    assign d_pe_snapshot = down_owner ? pe_snapshot : 4096'd0;
    assign mac_mode_safe = !pe_slot_valid && !pe_rq_busy &&
                           head_mac_safe && down_mac_safe;
    assign mode_viol = pe_mode_viol;
    assign down_rq_wait = down_rq_wait_count;
    assign down_read_before_ready = down_read_ready_count;
    assign down_mac_slots = down_slots;
    assign down_rq_overrun = down_overrun;

    reg sof_seen;
    reg final_eof_seen;
    wire d64_fire = d64_valid && d64_ready;
    wire head_final_store = head_store_wr_valid && head_store_wr_ready &&
                            head_store_wr_eof;
    wire down0_final_store = down0_store_wr_valid && down0_store_wr_ready &&
                             down0_store_wr_eof;
    wire out_final_fire = out_valid && out_ready && out_eof;
    wire head_first_sof = d64_fire && d64_sof;
    wire [31:0] down_frame_id_seen = (state_q == ST_DOWN0) ?
        head_read_frame_id : down0_read_frame_id;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            begin state_q <= ST_IDLE; head_owner_q <= 1'b0; down_owner_q <= 1'b0; end
            frame_id_q <= 32'd0;
            frame_id_seen <= 1'b0;
            sof_seen <= 1'b0;
            wall_active <= 1'b0;
            final_eof_seen <= 1'b0;
            down0_active <= 1'b0;
            down1_active <= 1'b0;
            head_store_overflow <= 32'd0;
            layer_mode_viol <= 32'd0;
            frame_id_violations <= 32'd0;
            head_layer_cycles <= 32'd0;
            down0_layer_cycles <= 32'd0;
            down1_layer_cycles <= 32'd0;
            head_compute_cycles <= 32'd0;
            down0_compute_cycles <= 32'd0;
            down1_compute_cycles <= 32'd0;
            encoder_compute_cycles <= 32'd0;
            encoder_store_wait_cycles <= 32'd0;
            encoder_wall_cycles <= 32'd0;
            head_compute_prev <= 32'd0;
            down_compute_prev <= 32'd0;
        end else begin
            if (((state_q == ST_HEAD) || (state_q == ST_HEAD_DRAIN) ||
                 (state_q == ST_HEAD_STORE_COMMIT)) && (pe_mode != 2'd3))
                layer_mode_viol <= layer_mode_viol + 32'd1;
            if (((state_q == ST_DOWN0_START) || (state_q == ST_DOWN0) ||
                 (state_q == ST_DOWN0_DRAIN) || (state_q == ST_DOWN1_START) ||
                 (state_q == ST_DOWN1) || (state_q == ST_DOWN1_DRAIN)) &&
                (pe_mode != 2'd0))
                layer_mode_viol <= layer_mode_viol + 32'd1;

            if (head_store_wr_valid && !head_store_wr_ready)
                head_store_overflow <= FAULT_STICKY ? 32'd1 : head_store_overflow + 32'd1;
            if (d64_fire && d64_sof) begin
                frame_id_q <= d64_frame_id;
                frame_id_seen <= 1'b1;
                encoder_wall_cycles <= 32'd1;
                wall_active <= 1'b1;
                sof_seen <= 1'b1;
                head_layer_cycles <= 32'd1;
            end else if (wall_active) begin
                encoder_wall_cycles <= encoder_wall_cycles + 32'd1;
            end

            if (head_first_sof)
                head_layer_cycles <= 32'd1;
            else if (sof_seen && !head_final_store)
                head_layer_cycles <= head_layer_cycles + 32'd1;
            if (head_final_store)
                sof_seen <= 1'b0;

            if (head_compute_prev != head_mac_compute) begin
                if (head_mac_compute > head_compute_prev) begin
                    head_compute_cycles <= head_mac_compute;
                    encoder_compute_cycles <= encoder_compute_cycles + 32'd1;
                end
                head_compute_prev <= head_mac_compute;
            end
            if (down_compute_prev != down_compute) begin
                if (down_compute > down_compute_prev) begin
                    if (state_q == ST_DOWN0)
                        down0_compute_cycles <= down_compute;
                    else if (state_q == ST_DOWN1)
                        down1_compute_cycles <= down_compute;
                    encoder_compute_cycles <= encoder_compute_cycles + 32'd1;
                end
                down_compute_prev <= down_compute;
            end

            if ((state_q == ST_HEAD_STORE_COMMIT) && !head_store_commit_ready)
                encoder_store_wait_cycles <= encoder_store_wait_cycles + 32'd1;
            if ((state_q == ST_D0_STORE_COMMIT) && !down0_store_commit_ready)
                encoder_store_wait_cycles <= encoder_store_wait_cycles + 32'd1;

            if (head_read_start) begin
                down0_active <= 1'b1;
                down0_layer_cycles <= 32'd1;
            end else if (down0_active && !down0_final_store) begin
                down0_layer_cycles <= down0_layer_cycles + 32'd1;
            end
            if (down0_final_store)
                down0_active <= 1'b0;

            if (down0_read_start) begin
                down1_active <= 1'b1;
                down1_layer_cycles <= 32'd1;
            end else if (down1_active && !out_final_fire) begin
                down1_layer_cycles <= down1_layer_cycles + 32'd1;
            end
            if (out_final_fire) begin
                down1_active <= 1'b0;
                wall_active <= 1'b0;
                final_eof_seen <= 1'b1;
            end

            if (head_read_valid && head_read_ready &&
                (!frame_id_seen || head_read_frame_id != frame_id_q))
                frame_id_violations <= frame_id_violations + 32'd1;
            if (down0_read_valid && down0_read_ready &&
                (!frame_id_seen || down0_read_frame_id != frame_id_q))
                frame_id_violations <= frame_id_violations + 32'd1;
            if (down0_store_wr_valid && down0_store_wr_ready &&
                down0_store_wr_frame_id != frame_id_q)
                frame_id_violations <= frame_id_violations + 32'd1;
            if (out_valid && out_ready && out_frame_id != frame_id_q)
                frame_id_violations <= frame_id_violations + 32'd1;

            case (state_q)
                ST_IDLE: if (start) begin state_q <= ST_HEAD; head_owner_q <= 1'b1; down_owner_q <= 1'b0; end
                ST_HEAD: if (d64_fire && d64_eof) begin state_q <= ST_HEAD_DRAIN; head_owner_q <= 1'b1; down_owner_q <= 1'b0; end
                ST_HEAD_DRAIN: if (head_done) begin state_q <= ST_HEAD_STORE_COMMIT; head_owner_q <= 1'b1; down_owner_q <= 1'b0; end
                ST_HEAD_STORE_COMMIT:
                    if (head_store_commit_ready && (head_store_overflow == 0))
                        begin state_q <= ST_MODE_SWITCH; head_owner_q <= 1'b0; down_owner_q <= 1'b0; end
                ST_MODE_SWITCH:
                    if (mac_mode_safe) begin state_q <= ST_DOWN0_START; head_owner_q <= 1'b0; down_owner_q <= 1'b1; end
                ST_DOWN0_START: begin state_q <= ST_DOWN0; head_owner_q <= 1'b0; down_owner_q <= 1'b1; end
                ST_DOWN0: if (down_done) begin state_q <= ST_DOWN0_DRAIN; head_owner_q <= 1'b0; down_owner_q <= 1'b1; end
                ST_DOWN0_DRAIN: begin state_q <= ST_D0_STORE_COMMIT; head_owner_q <= 1'b0; down_owner_q <= 1'b0; end
                ST_D0_STORE_COMMIT:
                    if (down0_store_commit_ready) begin state_q <= ST_DOWN1_START; head_owner_q <= 1'b0; down_owner_q <= 1'b1; end
                ST_DOWN1_START: begin state_q <= ST_DOWN1; head_owner_q <= 1'b0; down_owner_q <= 1'b1; end
                ST_DOWN1: if (down_done) begin state_q <= ST_DOWN1_DRAIN; head_owner_q <= 1'b0; down_owner_q <= 1'b1; end
                ST_DOWN1_DRAIN: if (final_eof_seen) begin state_q <= ST_DONE; head_owner_q <= 1'b0; down_owner_q <= 1'b0; end
                ST_DONE: begin state_q <= ST_DONE; head_owner_q <= 1'b0; down_owner_q <= 1'b0; end
                default: begin state_q <= ST_IDLE; head_owner_q <= 1'b0; down_owner_q <= 1'b0; end
            endcase
        end
    end
endmodule
