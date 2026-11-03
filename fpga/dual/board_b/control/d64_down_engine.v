`timescale 1ns/1ps

// D3c6-b2: one stride-2 engine for downs.0 and downs.1.
//
// The layer selector chooses only the frozen layer shape and quant format:
//   0: 256x256x8 U8 -> 128x128x16 U8, 1 IC group, 1 OC group, WBASE=9
//   1: 128x128x16 U8 -> 64x64x32 U10, 2 IC groups, 2 OC groups, WBASE=18
// Both layers use one P1/I8/O16 d64_mac128 and the same 5x256x64 row ring.
// Input beats are row-major, x-major, IC-group-inner. Output beats are
// row-major, x-major, OC-group then 8-channel requant pass.
module d64_down_engine #(
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0,
    parameter integer W_TILES = 247,
    parameter integer W_WORD_W = 1024,
    parameter integer W_ADDR_W = 8,
    parameter integer CFG_WORDS = 48,
    parameter integer FIFO_DEPTH = 8,
    parameter CFG_INIT_FILE = "",
    // **The image height, and the last place the front half still typed it.**
    // `layer_q` chose a frozen *layer shape*, and two of the four numbers that
    // make up a layer are heights: DOWN0 reads `H` rows and writes `H/2`, DOWN1
    // reads `H/2` and writes `H/4`.  Both were typed as 256 / 128 / 64, so at
    // any other image height the engine was told it had an input it never got:
    // `out_eof` never fires (`out_y == out_h_q - 1` is a row the downsampler
    // will not reach once the source runs out), `final_output_seen` stays low,
    // `done` stays low, and `d64_encoder` sits in `ST_DOWN0` forever.  It is not
    // a wrong number on a wire; it is a round that never ends.
    //
    // The two *widths* stay: this change moves height, and the widths at both
    // layers are 256 and 128 whether the image is square or not.  At
    // `H = 256` -- the default, and the shipped shape -- all four numbers are
    // the literals they replaced.
    parameter integer H = 256
) (
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire          layer_down1,
    input  wire [1:0]    pe_mode,
    // The default standalone wrapper owns a local weight store and MAC. The
    // encoder sets EXTERNAL_PE=1 and routes this seam to its single PE bank.
    output wire          pe_req_slot_valid,
    output wire [1407:0] pe_req_act,
    output wire [1023:0] pe_req_wgt,
    output wire          pe_req_pix_start,
    output wire [4095:0] pe_req_bias,
    output wire          pe_req_rq_start,
    output wire [255:0]  pe_req_rq_m,
    output wire [79:0]   pe_req_rq_shift,
    output wire [9:0]    pe_req_rq_zero,
    output wire [9:0]    pe_req_rq_qmin,
    output wire [9:0]    pe_req_rq_qmax,
    input  wire          pe_rsp_rq_ready,
    input  wire          pe_rsp_rq_ready_n,
    input  wire [79:0]   pe_rsp_y,
    input  wire [7:0]    pe_rsp_y_valid,
    input  wire          pe_rsp_rq_busy,
    input  wire          pe_rsp_rq_done,
    input  wire          pe_rsp_rq_overrun,
    input  wire [31:0]   pe_rsp_mode_viol,
    input  wire [4095:0] pe_rsp_acc,
    input  wire [4095:0] pe_rsp_snapshot,
    output wire          pe_wt_rd_en,
    output wire [7:0]    pe_wt_rd_addr,
    input  wire [1023:0] pe_wt_rd_data,
    output reg           busy,
    output reg           done,
    output wire          mac_mode_safe,

    // Shared network weight image and per-channel bias/requant table.
    input  wire          wt_we,
    input  wire [7:0]    wt_wa,
    input  wire [1023:0] wt_wd,
    input  wire          cfg_we,
    input  wire [5:0]    cfg_wa,
    input  wire [63:0]   cfg_wd,

    // Canonical feature word: one pixel and one 8-channel group per beat.
    input  wire          in_valid,
    output wire          in_ready,
    input  wire [7:0]    in_x,
    input  wire [7:0]    in_y,
    input  wire          in_icg,
    input  wire [63:0]   in_c8,
    input  wire          in_sof,
    input  wire          in_eol,
    input  wire          in_eof,
    input  wire [31:0]   in_frame_id,

    // One output word contains eight adjacent channels. U8 uses 64 low bits;
    // U10 uses all 80 bits. `out_word` orders the groups/passes for a pixel.
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

    // Regression and system-ledger observability.
    output reg  [31:0]   input_words,
    output reg  [31:0]   input_rows,
    output reg  [31:0]   input_marker_violations,
    output reg  [31:0]   output_order_violations,
    output reg  [31:0]   output_words,
    output reg  [31:0]   mac_slots,
    output reg  [31:0]   rq_accepts,
    output reg  [31:0]   rq_wait,
    output reg  [31:0]   rq_overrun,
    output wire [31:0]   mode_viol,
    output reg  [31:0]   layer_mode_viol,
    output reg  [31:0]   read_before_ready,
    output reg  [31:0]   engine_compute_cycles,
    output reg  [31:0]   engine_frame_cycles,
    output reg  [31:0]   row_start_wait_cycles,
    output wire [31:0]   rq_wait_cycles,
    output reg  [31:0]   output_credit_wait_cycles,
    output reg  [31:0]   input_no_free_slot_cycles,
    output reg           debug_rq_accept_event,
    output wire [511:0]  debug_mac_snapshot,
    output wire [4:0]    row_ready,
    output wire [39:0]   row_tags
);
    localparam [1:0] MAC_P1_I8_O16 = 2'd0;

    reg layer_q;
    reg [8:0] in_w_q, in_h_q;
    reg [7:0] out_w_q, out_h_q;
    reg [1:0] ic_groups_q, oc_groups_q;
    reg [7:0] wbase_q;
    reg [9:0] rq_zero_q, rq_qmin_q, rq_qmax_q;
    reg out_u8_q;

    wire [8:0] in_w_cfg = layer_down1 ? 9'd128 : 9'd256;
    wire [8:0] in_h_cfg = layer_down1 ? (H / 2) : H;
    wire [7:0] out_w_cfg = layer_down1 ? 8'd64 : 8'd128;
    wire [7:0] out_h_cfg = layer_down1 ? (H / 4) : (H / 2);
    wire [1:0] ic_groups_cfg = layer_down1 ? 2'd2 : 2'd1;
    wire [1:0] oc_groups_cfg = layer_down1 ? 2'd2 : 2'd1;
    wire [7:0] wbase_cfg = layer_down1 ? 8'd18 : 8'd9;
    wire [9:0] rq_zero_cfg = layer_down1 ? (RUNTIME_PROFILE ? profile_trunk_zero : 10'd413) : 10'd0;
    wire [9:0] rq_qmin_cfg = layer_down1 ? (RUNTIME_PROFILE ? profile_trunk_zero : 10'd413) : 10'd0;
    wire [9:0] rq_qmax_cfg = layer_down1 ? 10'd1023 : 10'd255;

    // Five slots hold a three-row active window plus two lookahead rows.
    reg [4:0] ring_ready_q;
    reg [4:0] ring_reserved_q;
    reg [7:0] ring_tag [0:4];
    reg input_frame_done;
    reg input_row_active;
    reg [2:0] input_row_slot_q;
    assign row_ready = ring_ready_q;
    assign row_tags = {ring_tag[4], ring_tag[3], ring_tag[2],
                       ring_tag[1], ring_tag[0]};

    reg free_slot_found;
    reg [2:0] free_slot_idx;
    integer free_scan;
    always @* begin
        free_slot_found = 1'b0;
        free_slot_idx = 3'd0;
        for (free_scan = 0; free_scan < 5; free_scan = free_scan + 1) begin
            if (!free_slot_found && !ring_ready_q[free_scan] &&
                !ring_reserved_q[free_scan]) begin
                free_slot_found = 1'b1;
                free_slot_idx = free_scan[2:0];
            end
        end
    end
    wire input_new_row = (in_x == 0) && (in_icg == 0);
    wire [2:0] in_slot = input_row_active ? input_row_slot_q : free_slot_idx;
    wire in_slot_available = input_row_active ?
        (ring_reserved_q[input_row_slot_q] &&
         (ring_tag[input_row_slot_q] == in_y)) :
        (input_new_row && free_slot_found);
    assign in_ready = busy && !input_frame_done && in_slot_available;
    wire in_fire = in_valid && in_ready;
    wire [7:0] in_word_ix = layer_q ? {in_x[6:0], 1'b0} + {7'd0, in_icg}
                                    : in_x;
    wire ring_wr_en = in_fire;

    reg [31:0] frame_id_q;
    reg frame_id_valid_q;
    reg frame_active;

    reg [7:0] sched_y, sched_x;
    reg [1:0] sched_ocg;
    reg [1:0] sched_ky, sched_kx;
    reg sched_icg;
    reg sched_active, sched_done;
    reg req_valid, req_first, req_last, req_final;
    reg [7:0] req_y, req_x;
    reg [1:0] req_ocg;
    reg op_valid, op_first, op_last, op_final;
    reg [7:0] op_y, op_x;
    reg [1:0] op_ocg;
    reg [63:0] op_act;
    reg [1023:0] op_wgt;
    reg [1407:0] mac_act;
    wire issue_slot;

    function [7:0] reflect_coord;
        input signed [10:0] pos;
        input [8:0] width;
        begin
            if (pos < 0)
                reflect_coord = 8'd1;
            else if (pos >= $signed({2'b00, width}))
                reflect_coord = width - 9'd2;
            else
                reflect_coord = pos[7:0];
        end
    endfunction

    function [7:0] last_use_y;
        input [7:0] source_y;
        reg [8:0] candidate;
        begin
            candidate = ({1'b0, source_y} + 9'd1) >> 1;
            if (candidate >= out_h_q)
                last_use_y = out_h_q - 8'd1;
            else
                last_use_y = candidate[7:0];
        end
    endfunction

    wire signed [10:0] sched_y_pos = {2'b00, sched_y, 1'b0};
    wire signed [10:0] sched_x_pos = {2'b00, sched_x, 1'b0};
    wire signed [10:0] sched_ky_pos = {9'd0, sched_ky};
    wire signed [10:0] sched_kx_pos = {9'd0, sched_kx};
    wire signed [10:0] rd_y_pos = sched_y_pos + sched_ky_pos - 11'sd1;
    wire signed [10:0] rd_x_pos = sched_x_pos + sched_kx_pos - 11'sd1;
    wire [7:0] rd_y = reflect_coord(rd_y_pos, in_h_q);
    wire [7:0] rd_x = reflect_coord(rd_x_pos, in_w_q);
    wire [7:0] rd_word_ix = layer_q ? {rd_x[6:0], 1'b0} + {7'd0, sched_icg}
                                    : rd_x;
    reg rd_slot_found;
    reg [2:0] rd_slot;
    integer read_scan;
    always @* begin
        rd_slot_found = 1'b0;
        rd_slot = 3'd0;
        for (read_scan = 0; read_scan < 5; read_scan = read_scan + 1) begin
            if (!rd_slot_found && ring_ready_q[read_scan] &&
                (ring_tag[read_scan] == rd_y)) begin
                rd_slot_found = 1'b1;
                rd_slot = read_scan[2:0];
            end
        end
    end
    wire [10:0] rd_addr = {rd_slot, rd_word_ix};
    wire [3:0] weight_tap = sched_ky * 4'd3 + sched_kx;
    wire [7:0] wt_offset = layer_q ?
        ({4'd0, weight_tap} << 2) + ({7'd0, sched_icg} << 1) + sched_ocg :
        {4'd0, weight_tap};
    wire [7:0] wt_rd_addr = wbase_q + wt_offset;
    wire [63:0] ring_rd_data;
    wire [1023:0] wt_rd_data;
    wire wt_rd_en;
    wire ring_rd_en;

    d64_down_row_ring u_row_ring (
        .clk(clk), .wr_en(ring_wr_en), .wr_slot(in_slot),
        .wr_word(in_word_ix), .wr_data(in_c8),
        .rd_en(ring_rd_en), .rd_addr(rd_addr), .rd_data(ring_rd_data)
    );

    assign rq_wait_cycles = rq_wait;

    assign pe_wt_rd_en = wt_rd_en;
    assign pe_wt_rd_addr = wt_rd_addr;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_weight_store
            assign wt_rd_data = pe_wt_rd_data;
        end else begin : g_local_weight_store
            d64_weight_store #(.TILES(W_TILES), .WORD_W(W_WORD_W), .ADDR_W(W_ADDR_W))
            u_weight_store (
                .clk(clk), .rst_n(rst_n),
                .wr_en(wt_we), .wr_addr(wt_wa), .wr_data(wt_wd),
                .rd_en(wt_rd_en), .rd_addr(wt_rd_addr), .rd_data(wt_rd_data)
            );
        end
    endgenerate

    wire [1:0] sched_ic_groups_last = ic_groups_q - 2'd1;
    wire sched_icg_last = ({1'b0, sched_icg} == sched_ic_groups_last);
    wire sched_phase_last = (sched_ky == 2'd2) && (sched_kx == 2'd2) &&
                            sched_icg_last;
    wire sched_output_row_end = sched_phase_last &&
            (sched_ocg == (oc_groups_q - 2'd1)) &&
            (sched_x == (out_w_q - 8'd1));
    wire sched_frame_end = sched_output_row_end &&
                           (sched_y == (out_h_q - 8'd1));
    wire req_row_ready = rd_slot_found;

    // Reserve two queue entries at phase accept so the two requant passes can
    // always be committed even if DDR/output backpressure starts immediately.
    reg [3:0] out_count;
    reg [2:0] out_head, out_tail;
    reg [1:0] reserved_words;
    reg [79:0] out_fifo_data [0:FIFO_DEPTH-1];
    reg [7:0] out_fifo_x [0:FIFO_DEPTH-1];
    reg [7:0] out_fifo_y [0:FIFO_DEPTH-1];
    reg [1:0] out_fifo_word [0:FIFO_DEPTH-1];
    reg [31:0] out_fifo_frame_id [0:FIFO_DEPTH-1];
    wire [4:0] credit_used = {1'b0, out_count} + {3'b000, reserved_words};
    wire output_credit_ok = (credit_used <= (FIFO_DEPTH - 2));

    wire [7:0] mac_y_valid;
    wire [79:0] mac_y;
    wire mac_rq_ready, mac_rq_ready_n, mac_rq_busy, mac_rq_done;
    wire mac_rq_overrun;
    wire [4095:0] mac_acc, mac_snapshot;
    wire [255:0] rq_m_now, rq_m_hold;
    wire [79:0] rq_shift_now, rq_shift_hold;
    reg [255:0] rq_m_hold_q;
    reg [79:0] rq_shift_hold_q;
    wire mac_rq_accept = op_valid && op_last && mac_rq_ready;
    wire mac_rq_start = mac_rq_accept;
    wire mac_pix_start = (start && !busy) ||
                         (op_valid && op_last && !op_final);
    wire [31:0] mac_mode_viol;
    wire mac_slot_valid = op_valid;

    reg [7:0] phase_x_q, phase_y_q;
    reg [1:0] phase_ocg_q;
    reg phase_pass_q;
    reg [31:0] phase_frame_id_q;
    wire [4095:0] bias_bus;
    wire [1:0] bias_ocg = (start && !busy) ? 2'd0 :
                         (op_ocg == (oc_groups_q - 2'd1)) ? 2'd0 :
                                                               (op_ocg + 2'd1);

    // **D3c6-d1b: the deployment config is structure, not a memory.**
    //
    // This was a 48 x 64 array with a write port, filled by `initial $readmemh`.
    // The board ties the write port off, but its presence is enough to make the
    // array a read/write memory; the read is asynchronous, so the mapper placed
    // it in logic memory, and the image did not survive -- post-map the requant
    // multiplier register kept 4 of its 256 bits and the shift register 3 of 80.
    // Every downs channel ran on a don't-care fold of the layer-shape registers.
    //
    // `d64_down_cfg_rom` is a case over literals generated from the frozen image
    // by `tools/d64_down_cfg_rtl.py`.  There is no initial state to mistrust and
    // no don't-care to exploit.  The port list is unchanged -- `cfg_we`,
    // `cfg_wa` and `cfg_wd` are still accepted and simply have nothing to write
    // to, which is what the board was already doing.
    wire [31:0]   cfg_image_crc;
    wire          cfg_page_oob, cfg_page_oob_m, cfg_page_oob_b;

    // **Two selects, because the original really had two index rules and they
    // are not the same page.**  `cfg_base_now` and `bias_base_now` agree for
    // down0 and agree for down1 once its output group settles, but they differ
    // whenever `op_ocg` and `bias_ocg` do, and at a layer's first cycle the bias
    // rule pins page 1 while the M/shift rule already tracks `op_ocg`.  Reading
    // them as one page would have been a silent semantic change dressed up as a
    // simplification; two constant lookups cost nothing, so both are carried.
    //
    // The base addresses are kept exactly as they were, out-of-range case
    // included: the old read of `cfg_mem[base + mi]` past entry 47 is an x in
    // simulation and a don't-care in synthesis, so anything at or past entry 48
    // is routed to the ROM's not-a-page arm and reported.
    wire [5:0] cfg_base_now  = layer_q ? (6'd16 + {op_ocg, 4'b0}) : 6'd0;
    wire [5:0] bias_base_now = (layer_down1 && (start && !busy)) ? 6'd16
                             : layer_q ? (6'd16 + {bias_ocg, 4'b0}) : 6'd0;

    function [1:0] page_of;
        input [5:0] base;
        begin
            page_of = (base > 6'd47) ? 2'd3 : base[5:4];
        end
    endfunction

    wire [1023:0] cfg_page_m, cfg_page_b;

    generate if (RUNTIME_PROFILE != 0) begin : g_profile_down
        wire [1:0] page_m = page_of(cfg_base_now);
        wire [1:0] page_b = page_of(bias_base_now);
        d64_front_profile_rom u_profile_m (
            .style(profile_style), .page(page_m + 2'd1), .words(cfg_page_m));
        d64_front_profile_rom u_profile_b (
            .style(profile_style), .page(page_b + 2'd1), .words(cfg_page_b));
        assign cfg_image_crc = 32'd0;
        assign cfg_page_oob_m = (page_m == 2'd3);
        assign cfg_page_oob_b = (page_b == 2'd3);
    end else begin : g_frozen_down
        d64_down_cfg_rom u_cfg_rom_m (
            .page(page_of(cfg_base_now)), .words(cfg_page_m),
            .image_crc(cfg_image_crc), .page_oob(cfg_page_oob_m));
        d64_down_cfg_rom u_cfg_rom_b (
            .page(page_of(bias_base_now)), .words(cfg_page_b),
            .image_crc(), .page_oob(cfg_page_oob_b));
    end endgenerate
    assign cfg_page_oob = cfg_page_oob_m | cfg_page_oob_b;

    // Kept, and reported: a select outside the image used to read off the end of
    // the array, which is an x in simulation and a don't-care in synthesis.  The
    // ROM answers zero and says so.
    reg cfg_oob_seen;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) cfg_oob_seen <= 1'b0;
        else if (cfg_page_oob) cfg_oob_seen <= 1'b1;


    integer bi;
    integer mi;
    integer sh_i;
    reg [255:0] rq_m_comb;
    reg [79:0] rq_shift_comb;
    reg [4095:0] bias_comb;
    always @* begin
        rq_m_comb = 256'd0;
        rq_shift_comb = 80'd0;
        bias_comb = 4096'd0;
        for (mi = 0; mi < 16; mi = mi + 1) begin
            rq_m_comb[mi*16 +: 16] = cfg_page_m[mi*64 + 32 +: 16];
            rq_shift_comb[mi*5 +: 5] = cfg_page_m[mi*64 + 48 +: 5];
            bias_comb[mi*32 +: 32] = cfg_page_b[mi*64 + 0 +: 32];
        end
    end
    assign rq_m_now = rq_m_comb;
    assign rq_shift_now = rq_shift_comb;
    assign rq_m_hold = rq_m_hold_q;
    assign rq_shift_hold = rq_shift_hold_q;
    assign bias_bus = bias_comb;

    wire [255:0] rq_m_bus = mac_rq_accept ? rq_m_now : rq_m_hold;
    wire [79:0] rq_shift_bus = mac_rq_accept ? rq_shift_now : rq_shift_hold;

    assign pe_req_slot_valid = mac_slot_valid;
    assign pe_req_act = mac_act;
    assign pe_req_wgt = op_wgt;
    assign pe_req_pix_start = mac_pix_start;
    assign pe_req_bias = bias_bus;
    assign pe_req_rq_start = mac_rq_start;
    assign pe_req_rq_m = rq_m_bus;
    assign pe_req_rq_shift = rq_shift_bus;
    assign pe_req_rq_zero = rq_zero_q;
    assign pe_req_rq_qmin = rq_qmin_q;
    assign pe_req_rq_qmax = rq_qmax_q;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_pe
            assign mac_rq_ready = pe_rsp_rq_ready;
            assign mac_rq_ready_n = pe_rsp_rq_ready_n;
            assign mac_rq_busy = pe_rsp_rq_busy;
            assign mac_rq_done = pe_rsp_rq_done;
            assign mac_rq_overrun = pe_rsp_rq_overrun;
            assign mac_mode_viol = pe_rsp_mode_viol;
            assign mac_acc = pe_rsp_acc;
            assign mac_snapshot = pe_rsp_snapshot;
            assign mac_y = pe_rsp_y;
            assign mac_y_valid = pe_rsp_y_valid;
        end else begin : g_local_pe
            d64_mac128 #(.ACT_W(11), .WGT_W(8), .ACC_W(32), .M_W(16),
                         .SHIFT_W(5), .OUT_W(10), .RQ_LANES(8), .LANES(128),
                         .OC_MAX(16), .RQ_LOOKAHEAD(3)) u_mac (
                .clk(clk), .rst_n(rst_n), .mode(pe_mode),
                .slot_valid(mac_slot_valid), .act(mac_act), .wgt(op_wgt),
                .pix_start(mac_pix_start), .bias(bias_bus),
                .rq_start(mac_rq_start), .rq_ready(mac_rq_ready),
                .rq_ready_n(mac_rq_ready_n), .rq_m(rq_m_bus),
                .rq_shift(rq_shift_bus), .rq_zero(rq_zero_q),
                .rq_qmin(rq_qmin_q), .rq_qmax(rq_qmax_q),
                .y(mac_y), .y_valid(mac_y_valid), .rq_busy(mac_rq_busy),
                .rq_done(mac_rq_done), .rq_overrun(mac_rq_overrun),
                .mode_viol(mac_mode_viol), .acc_o(mac_acc), .rq_snap_o(mac_snapshot)
            );
        end
    endgenerate

    integer lane;
    integer ic;
    always @* begin
        mac_act = 1408'd0;
        for (lane = 0; lane < 128; lane = lane + 1) begin
            ic = lane % 8;
            mac_act[lane*11 +: 11] = {3'b000, (EXTERNAL_PE ? ring_rd_data[ic*8 +: 8] : op_act[ic*8 +: 8])};
        end
    end

    assign mode_viol = mac_mode_viol;
    assign debug_mac_snapshot = mac_snapshot[511:0];
    assign mac_mode_safe = !mac_slot_valid && !req_valid && !op_valid &&
                           !mac_rq_busy;
    assign wt_rd_en = issue_slot || req_valid;
    assign ring_rd_en = issue_slot || req_valid;

    reg [79:0] packed_y;
    integer py;
    always @* begin
        packed_y = 80'd0;
        if (out_u8_q) begin
            for (py = 0; py < 8; py = py + 1)
                packed_y[py*8 +: 8] = mac_y[py*10 +: 8];
        end else begin
            for (py = 0; py < 8; py = py + 1)
                packed_y[py*10 +: 10] = mac_y[py*10 +: 10];
        end
    end

    assign out_valid = (out_count != 0);
    assign out_x = out_fifo_x[out_head];
    assign out_y = out_fifo_y[out_head];
    assign out_word = out_fifo_word[out_head];
    assign out_data = out_fifo_data[out_head];
    assign out_frame_id = out_fifo_frame_id[out_head];
    assign out_sof = out_valid && (out_x == 0) && (out_y == 0) && (out_word == 0);
    assign out_eol = out_valid && (out_x == (out_w_q - 8'd1)) &&
                     (out_word == ((oc_groups_q * 2) - 1));
    assign out_eof = out_eol && (out_y == (out_h_q - 8'd1));
    wire out_pop = out_valid && out_ready;
    wire out_push = mac_y_valid[0];
    wire out_final_pop = out_pop && out_eof;

    // Per-cycle request + registered operand/control stages.  The ring/weight
    // RAM outputs never feed the multiplier directly.
    wire phase_credit_ok = !sched_phase_last || output_credit_ok;
    wire phase_ready_ok = !sched_phase_last || mac_rq_ready_n;
    wire req_row_ok = req_row_ready;
    assign issue_slot = busy && sched_active && !sched_done && req_row_ok &&
                        phase_credit_ok && phase_ready_ok;
    wire sched_phase_final = sched_phase_last && (sched_ocg == (oc_groups_q - 2'd1)) &&
                             (sched_x == (out_w_q - 8'd1)) &&
                             (sched_y == (out_h_q - 8'd1));
    wire sched_phase_row_last = sched_phase_last &&
                                (sched_ocg == (oc_groups_q - 2'd1)) &&
                                (sched_x == (out_w_q - 8'd1));
    wire [7:0] triplet_top = reflect_coord(sched_y_pos - 11'sd1, in_h_q);
    wire [7:0] triplet_mid = reflect_coord(sched_y_pos, in_h_q);
    wire [7:0] triplet_bot = reflect_coord(sched_y_pos + 11'sd1, in_h_q);
    reg triplet_top_ready, triplet_mid_ready, triplet_bot_ready;
    integer triplet_scan;
    always @* begin
        triplet_top_ready = 1'b0;
        triplet_mid_ready = 1'b0;
        triplet_bot_ready = 1'b0;
        for (triplet_scan = 0; triplet_scan < 5; triplet_scan = triplet_scan + 1) begin
            if (ring_ready_q[triplet_scan]) begin
                if (ring_tag[triplet_scan] == triplet_top)
                    triplet_top_ready = 1'b1;
                if (ring_tag[triplet_scan] == triplet_mid)
                    triplet_mid_ready = 1'b1;
                if (ring_tag[triplet_scan] == triplet_bot)
                    triplet_bot_ready = 1'b1;
            end
        end
    end
    wire triplet_ready = triplet_top_ready && triplet_mid_ready && triplet_bot_ready;

    reg [7:0] expected_output_x, expected_output_y;
    reg [1:0] expected_output_word;
    reg final_output_seen;
    reg compute_active;
    reg [7:0] expected_input_x, expected_input_y;
    reg expected_input_icg;
    integer ri;
    integer free_y;
    integer next_ocg_tmp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            layer_q <= 1'b0;
            in_w_q <= 8'd0; in_h_q <= 8'd0;
            out_w_q <= 8'd0; out_h_q <= 8'd0;
            ic_groups_q <= 2'd0; oc_groups_q <= 2'd0;
            wbase_q <= 8'd0;
            rq_zero_q <= 10'd0; rq_qmin_q <= 10'd0; rq_qmax_q <= 10'd255;
            out_u8_q <= 1'b1;
            busy <= 1'b0; done <= 1'b0;
            ring_ready_q <= 5'd0; ring_reserved_q <= 5'd0;
            for (ri = 0; ri < 5; ri = ri + 1) ring_tag[ri] <= 8'd0;
            frame_id_q <= 32'd0; frame_id_valid_q <= 1'b0;
            input_frame_done <= 1'b0; input_row_active <= 1'b0;
            input_row_slot_q <= 3'd0; frame_active <= 1'b0;
            sched_y <= 8'd0; sched_x <= 8'd0; sched_ocg <= 2'd0;
            sched_ky <= 2'd0; sched_kx <= 2'd0; sched_icg <= 1'b0;
            sched_active <= 1'b0; sched_done <= 1'b0;
            req_valid <= 1'b0; req_first <= 1'b0; req_last <= 1'b0; req_final <= 1'b0;
            req_y <= 8'd0; req_x <= 8'd0; req_ocg <= 2'd0;
            op_valid <= 1'b0; op_first <= 1'b0; op_last <= 1'b0; op_final <= 1'b0;
            op_y <= 8'd0; op_x <= 8'd0; op_ocg <= 2'd0;
            op_act <= 64'd0; if(EXTERNAL_PE==0) op_wgt <= 1024'd0;
            phase_x_q <= 8'd0; phase_y_q <= 8'd0; phase_ocg_q <= 2'd0;
            phase_pass_q <= 1'b0; phase_frame_id_q <= 32'd0;
            rq_m_hold_q <= 256'd0; rq_shift_hold_q <= 80'd0;
            for (ri = 0; ri < FIFO_DEPTH; ri = ri + 1) begin
                out_fifo_data[ri] <= 80'd0;
                out_fifo_x[ri] <= 8'd0; out_fifo_y[ri] <= 8'd0;
                out_fifo_word[ri] <= 2'd0; out_fifo_frame_id[ri] <= 32'd0;
            end
            out_count <= 4'd0; out_head <= 3'd0; out_tail <= 3'd0;
            reserved_words <= 2'd0;
            final_output_seen <= 1'b0; compute_active <= 1'b0;
            expected_output_x <= 8'd0; expected_output_y <= 8'd0;
            expected_output_word <= 2'd0;
            expected_input_x <= 8'd0; expected_input_y <= 8'd0;
            expected_input_icg <= 1'b0;
            input_words <= 32'd0; input_rows <= 32'd0;
            input_marker_violations <= 32'd0; output_order_violations <= 32'd0;
            output_words <= 32'd0; mac_slots <= 32'd0; rq_accepts <= 32'd0;
            rq_wait <= 32'd0; rq_overrun <= 32'd0; layer_mode_viol <= 32'd0;
            read_before_ready <= 32'd0;
            engine_compute_cycles <= 32'd0; engine_frame_cycles <= 32'd0;
            row_start_wait_cycles <= 32'd0;
            output_credit_wait_cycles <= 32'd0;
            input_no_free_slot_cycles <= 32'd0;
            debug_rq_accept_event <= 1'b0;
        end else begin
            debug_rq_accept_event <= mac_rq_accept;

            if (start && !busy) begin
                layer_q <= layer_down1;
                in_w_q <= in_w_cfg; in_h_q <= in_h_cfg;
                out_w_q <= out_w_cfg; out_h_q <= out_h_cfg;
                ic_groups_q <= ic_groups_cfg; oc_groups_q <= oc_groups_cfg;
                wbase_q <= wbase_cfg;
                rq_zero_q <= rq_zero_cfg; rq_qmin_q <= rq_qmin_cfg;
                rq_qmax_q <= rq_qmax_cfg; out_u8_q <= !layer_down1;
                busy <= 1'b1; done <= 1'b0;
                ring_ready_q <= 5'd0; ring_reserved_q <= 5'd0;
                for (ri = 0; ri < 5; ri = ri + 1) ring_tag[ri] <= 8'd0;
                frame_id_q <= 32'd0; frame_id_valid_q <= 1'b0;
                input_frame_done <= 1'b0; input_row_active <= 1'b0;
                input_row_slot_q <= 3'd0; frame_active <= 1'b0;
                sched_y <= 8'd0; sched_x <= 8'd0; sched_ocg <= 2'd0;
                sched_ky <= 2'd0; sched_kx <= 2'd0; sched_icg <= 1'b0;
                sched_active <= 1'b0; sched_done <= 1'b0;
                req_valid <= 1'b0; req_first <= 1'b0; req_last <= 1'b0; req_final <= 1'b0;
                op_valid <= 1'b0; op_first <= 1'b0; op_last <= 1'b0; op_final <= 1'b0;
                out_count <= 4'd0; out_head <= 3'd0; out_tail <= 3'd0;
                reserved_words <= 2'd0;
                final_output_seen <= 1'b0; compute_active <= 1'b0;
                expected_output_x <= 8'd0; expected_output_y <= 8'd0;
                expected_output_word <= 2'd0;
                expected_input_x <= 8'd0; expected_input_y <= 8'd0;
                expected_input_icg <= 1'b0;
                input_words <= 32'd0; input_rows <= 32'd0;
                input_marker_violations <= 32'd0; output_order_violations <= 32'd0;
                output_words <= 32'd0; mac_slots <= 32'd0; rq_accepts <= 32'd0;
                rq_wait <= 32'd0; rq_overrun <= 32'd0; layer_mode_viol <= 32'd0;
                read_before_ready <= 32'd0;
                engine_compute_cycles <= 32'd0; engine_frame_cycles <= 32'd0;
                row_start_wait_cycles <= 32'd0;
                output_credit_wait_cycles <= 32'd0;
                input_no_free_slot_cycles <= 32'd0;
            end else begin
                if (busy && (pe_mode != MAC_P1_I8_O16))
                    layer_mode_viol <= layer_mode_viol + 32'd1;

                if (in_fire) begin
                    if (in_x != expected_input_x || in_y != expected_input_y ||
                        in_icg != expected_input_icg ||
                        in_sof != (input_words == 0) ||
                        in_eol != ((in_x == (in_w_q - 8'd1)) &&
                                   (in_icg == (ic_groups_q - 2'd1))) ||
                        in_eof != ((in_x == (in_w_q - 8'd1)) &&
                                   (in_y == (in_h_q - 8'd1)) &&
                                   (in_icg == (ic_groups_q - 2'd1))))
                        input_marker_violations <= input_marker_violations + 32'd1;
                    if (input_words == 0) begin
                        frame_id_q <= in_frame_id;
                        frame_id_valid_q <= 1'b1;
                    end else if (in_frame_id != frame_id_q)
                        input_marker_violations <= input_marker_violations + 32'd1;
                    input_words <= input_words + 32'd1;
                    if (in_eol) input_rows <= input_rows + 32'd1;
                    if (in_eof) input_frame_done <= 1'b1;

                    if (expected_input_icg != (ic_groups_q - 2'd1))
                        expected_input_icg <= 1'b1;
                    else begin
                        expected_input_icg <= 1'b0;
                        if (expected_input_x != (in_w_q - 8'd1))
                            expected_input_x <= expected_input_x + 8'd1;
                        else begin
                            expected_input_x <= 8'd0;
                            if (expected_input_y != (in_h_q - 8'd1))
                                expected_input_y <= expected_input_y + 8'd1;
                        end
                    end

                    if (input_new_row) begin
                        ring_tag[in_slot] <= in_y;
                        ring_reserved_q[in_slot] <= 1'b1;
                        ring_ready_q[in_slot] <= 1'b0;
                        input_row_active <= 1'b1;
                        input_row_slot_q <= in_slot;
                    end
                    if ((in_x == (in_w_q - 8'd1)) &&
                        (in_icg == (ic_groups_q - 2'd1))) begin
                        ring_reserved_q[in_slot] <= 1'b0;
                        ring_ready_q[in_slot] <= 1'b1;
                        input_row_active <= 1'b0;
                    end
                    if (in_sof) begin
                        frame_active <= 1'b1;
                        engine_frame_cycles <= 32'd1;
                    end
                end

                if (frame_active)
                    engine_frame_cycles <= engine_frame_cycles + 32'd1;
                if (out_final_pop)
                    frame_active <= 1'b0;

                if (!sched_active && !sched_done && busy && triplet_ready)
                    sched_active <= 1'b1;

                if (busy && !sched_done && !sched_active && !triplet_ready)
                    row_start_wait_cycles <= row_start_wait_cycles + 32'd1;
                if (sched_active && !sched_done && sched_phase_last &&
                    !output_credit_ok)
                    output_credit_wait_cycles <= output_credit_wait_cycles + 32'd1;
                if (in_valid && busy && !input_frame_done && !in_slot_available)
                    input_no_free_slot_cycles <= input_no_free_slot_cycles + 32'd1;

                if (sched_active && !sched_done && !req_row_ok)
                    read_before_ready <= read_before_ready + 32'd1;

                if (sched_active && sched_phase_last && !mac_rq_ready_n)
                    rq_wait <= rq_wait + 32'd1;

                if (issue_slot) begin
                    req_valid <= 1'b1;
                    req_first <= (sched_ky == 0) && (sched_kx == 0) &&
                                 (sched_icg == 0);
                    req_last <= sched_phase_last;
                    req_final <= sched_phase_final;
                    req_y <= sched_y; req_x <= sched_x; req_ocg <= sched_ocg;

                    if (sched_icg_last) begin
                        sched_icg <= 1'b0;
                        if (sched_kx != 2'd2) sched_kx <= sched_kx + 2'd1;
                        else begin
                            sched_kx <= 2'd0;
                            if (sched_ky != 2'd2) sched_ky <= sched_ky + 2'd1;
                            else begin
                                sched_ky <= 2'd0;
                                if (sched_ocg != (oc_groups_q - 2'd1))
                                    sched_ocg <= sched_ocg + 2'd1;
                                else begin
                                    sched_ocg <= 2'd0;
                                    if (sched_x != (out_w_q - 8'd1))
                                        sched_x <= sched_x + 8'd1;
                                    else begin
                                        sched_x <= 8'd0;
                                        if (sched_y != (out_h_q - 8'd1))
                                            sched_y <= sched_y + 8'd1;
                                        else begin
                                            sched_active <= 1'b0;
                                            sched_done <= 1'b1;
                                        end
                                    end
                                end
                            end
                        end
                    end else
                        sched_icg <= 1'b1;

                    if (sched_output_row_end) begin
                        sched_active <= 1'b0;
                        if (!sched_frame_end)
                            sched_y <= sched_y + 8'd1;
                        for (ri = 0; ri < 5; ri = ri + 1) begin
                            if ((ring_ready_q[ri] &&
                                 (last_use_y(ring_tag[ri]) == sched_y)) ||
                                (in_fire && (in_slot == ri[2:0]) &&
                                 (in_x == (in_w_q - 8'd1)) &&
                                 (in_icg == (ic_groups_q - 2'd1)) &&
                                 (last_use_y(in_y) == sched_y))) begin
                                ring_ready_q[ri] <= 1'b0;
                                ring_reserved_q[ri] <= 1'b0;
                            end
                        end
                    end
                end else if (!issue_slot) begin
                    req_valid <= 1'b0;
                    req_first <= 1'b0; req_last <= 1'b0; req_final <= 1'b0;
                end

                // Synchronous RAM -> registered operand/control -> universal PE.
                op_valid <= req_valid;
                op_first <= req_valid && req_first;
                op_last <= req_valid && req_last;
                op_final <= req_valid && req_final;
                if (req_valid) begin
                    op_y <= req_y; op_x <= req_x; op_ocg <= req_ocg;
                    op_act <= ring_rd_data;
                    if(EXTERNAL_PE==0) op_wgt <= wt_rd_data;
                end

                if (mac_slot_valid)
                    mac_slots <= mac_slots + 32'd1;
                if (mac_slot_valid && !compute_active) begin
                    compute_active <= 1'b1;
                    engine_compute_cycles <= 32'd1;
                end else if (compute_active)
                    engine_compute_cycles <= engine_compute_cycles + 32'd1;
                if (mac_slot_valid && op_final && op_last)
                    compute_active <= 1'b0;

                if (mac_rq_accept) begin
                    rq_accepts <= rq_accepts + 32'd1;
                    phase_x_q <= op_x; phase_y_q <= op_y; phase_ocg_q <= op_ocg;
                    phase_pass_q <= 1'b0;
                    phase_frame_id_q <= frame_id_q;
                end
                if (mac_rq_accept)
                    begin
                        rq_m_hold_q <= rq_m_now;
                        rq_shift_hold_q <= rq_shift_now;
                    end
                if (mac_rq_overrun)
                    rq_overrun <= rq_overrun + 32'd1;

                if (out_push) begin
                    out_fifo_data[out_tail] <= packed_y;
                    out_fifo_x[out_tail] <= phase_x_q;
                    out_fifo_y[out_tail] <= phase_y_q;
                    out_fifo_word[out_tail] <= {phase_ocg_q[0], phase_pass_q};
                    out_fifo_frame_id[out_tail] <= phase_frame_id_q;
                    out_tail <= out_tail + 3'd1;
                    phase_pass_q <= ~phase_pass_q;
                end
                if (out_pop)
                    out_head <= out_head + 3'd1;
                case ({out_push, out_pop})
                    2'b10: out_count <= out_count + 4'd1;
                    2'b01: out_count <= out_count - 4'd1;
                    default: out_count <= out_count;
                endcase
                case ({mac_rq_accept, out_push})
                    2'b10: reserved_words <= reserved_words + 2'd2;
                    2'b01: reserved_words <= reserved_words - 2'd1;
                    2'b11: reserved_words <= reserved_words + 2'd1;
                    default: reserved_words <= reserved_words;
                endcase

                if (out_pop) begin
                    if (out_word != expected_output_word ||
                        out_x != expected_output_x ||
                        out_y != expected_output_y ||
                        out_frame_id != frame_id_q)
                        output_order_violations <= output_order_violations + 32'd1;
                    output_words <= output_words + 32'd1;
                    if (out_eof) final_output_seen <= 1'b1;
                    if (expected_output_word == ((oc_groups_q * 2) - 1)) begin
                        expected_output_word <= 2'd0;
                        if (expected_output_x == (out_w_q - 8'd1)) begin
                            expected_output_x <= 8'd0;
                            if (expected_output_y != (out_h_q - 8'd1))
                                expected_output_y <= expected_output_y + 8'd1;
                        end else
                            expected_output_x <= expected_output_x + 8'd1;
                    end else
                        expected_output_word <= expected_output_word + 2'd1;
                end

                if (sched_done && final_output_seen && mac_mode_safe &&
                    (reserved_words == 0) && (out_count == 0)) begin
                    done <= 1'b1;
                    busy <= 1'b0;
                end
            end
        end
    end

    // External bank consumes weights only with its valid slot. Sampling the
    // common response every clock, with no owner-specific mask/reset, allows
    // equivalent raw response bits to share FFs across mutually exclusive engines.
    // The standalone backend retains its original reset and capture contract.
    generate if(EXTERNAL_PE!=0)begin:g_shared_weight_capture
      always @(posedge clk) op_wgt <= wt_rd_data;
    end endgenerate

    // synthesis translate_off
    wire [1407:0] act_registered_ghost;
    genvar ga;generate for(ga=0;ga<128;ga=ga+1)begin:g_act_ghost
      assign act_registered_ghost[ga*11+:11]={3'b0,op_act[(ga%8)*8+:8]};
    end endgenerate
    // synthesis translate_on
endmodule
