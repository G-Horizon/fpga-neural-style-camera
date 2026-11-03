`timescale 1ns/1ps

// D3c6-b1: HEAD scheduler and one-bank PE seam. The RAM outputs are expanded
// into the 128-lane P16/I1/O8 operands in a dedicated register stage before the
// MAC. Request metadata is registered with the synchronous RAM address; the
// first request of a group advances its bias preload by one stage so it can
// coincide with the previous group's final slot/requant accept without losing
// either contribution.
module d64_head_ctrl #(
    parameter integer EXTERNAL_PE = 0,
    parameter integer RUNTIME_PROFILE = 0,
    parameter integer HEAD_STORE_CREDIT_ENABLE = 0,
    // The shape.  **Height is what the four boundaries in this file are
    // about**: each `255` that used to sit below is `H-1` only for as long as
    // H is 256.  Width is carried too because `head_out_eol` is the same kind
    // of literal one axis over, and `W-1` is bit-identical to the 255 it
    // replaces at W = 256.
    parameter integer H = 256,
    parameter integer W = 256
) (
    // Stable for an entire job; the root changes these only at idle.
    input wire [1:0] profile_style,
    input wire [9:0] profile_trunk_zero,
    input  wire         head_phase_ready,
    input  wire         clk,
    input  wire         rst_n,
    input  wire [1:0]   pe_mode,

    // One-bank encoder seam. The default standalone path retains its local
    // MAC; the encoder sets EXTERNAL_PE=1 and arbitrates this request on the
    // shared d64_mac128.
    output wire         pe_req_slot_valid,
    output wire [1407:0] pe_req_act,
    output wire [1023:0] pe_req_wgt,
    output wire         pe_req_pix_start,
    output wire [4095:0] pe_req_bias,
    output wire         pe_req_rq_start,
    output wire [255:0] pe_req_rq_m,
    output wire [79:0]  pe_req_rq_shift,
    output wire [9:0]   pe_req_rq_zero,
    output wire [9:0]   pe_req_rq_qmin,
    output wire [9:0]   pe_req_rq_qmax,
    input  wire         pe_rsp_rq_ready,
    input  wire         pe_rsp_rq_ready_n,
    input  wire [79:0]  pe_rsp_y,
    input  wire [7:0]   pe_rsp_y_valid,
    input  wire         pe_rsp_rq_busy,
    input  wire         pe_rsp_rq_done,
    input  wire         pe_rsp_rq_overrun,
    input  wire [31:0]  pe_rsp_mode_viol,
    input  wire [4095:0] pe_rsp_acc,
    input  wire [4095:0] pe_rsp_snapshot,
    output wire         mac_mode_safe,

    input  wire [2:0]   row_ready,
    input  wire [23:0]  row_tags,
    input  wire [7:0]   next_input_y,
    input  wire [1:0]   next_input_slot,
    output reg          row_permit,

    output wire         win_rd_en,
    output wire [7:0]   win_rd_row,
    output wire [3:0]   win_rd_group,
    output wire [1:0]   win_rd_sub,
    input  wire [431:0] win_rd_data,   // {B, G, R} of the same 18-byte word
    input  wire         win_rd_valid,

    output wire         wt_rd_en,
    output wire [7:0]   wt_rd_addr,
    input  wire [1023:0] wt_rd_data,

    input  wire [31:0]  frame_id,
    input  wire         frame_id_valid,
    output wire         head_out_valid,
    output wire [7:0]   head_out_x,
    output wire [7:0]   head_out_y,
    output wire [63:0]  head_out_c8,
    output wire [31:0]  head_out_frame_id,
    output wire         head_out_sof,
    output wire         head_out_eol,
    output wire         head_out_eof,

    output reg  [31:0]  mac_slots,
    output reg  [31:0]  head_compute_cycles,
    output reg  [31:0]  rq_wait,
    output wire [31:0]  rq_overrun,
    output wire [31:0]  mode_viol,
    output reg          done
);

    reg [7:0] sched_y;
    reg [3:0] sched_group;
    reg [1:0] sched_ky, sched_kx;
    // The four-pixel sub-group inside the sixteen-pixel window word.  It
    // replaces `sched_ci`, which existed only because the old shape spent a
    // whole slot on one input channel; the three channels now arrive
    // together, so the slot that walked them walks pixels instead.
    reg [1:0] sched_sub;
    reg row_active;
    reg sched_done;
    reg final_eof_seen;
    reg compute_active;

    function [7:0] reflect_y;
        input signed [9:0] y;
        begin
            // The bound is the integer form `H - 1` and not an 8-bit
            // localparam, because `y` is `signed`: comparing it against an
            // unsigned value would make -1 read as a large positive number and
            // the top-edge reflection would silently become the bottom one.
            // The result truncates into the 8 bits the row fields carry.
            if (y < 0)          reflect_y = 8'd1;
            else if (y > H - 1) reflect_y = H - 2;
            else                reflect_y = y[7:0];
        end
    endfunction

    function row_present;
        input [7:0] wanted;
        input [2:0] valid;
        input [23:0] tags;
        begin
            row_present = (valid[0] && (tags[7:0] == wanted)) ||
                          (valid[1] && (tags[15:8] == wanted)) ||
                          (valid[2] && (tags[23:16] == wanted));
        end
    endfunction

    wire signed [9:0] sched_y_ext = {2'b00, sched_y};
    wire signed [9:0] sched_ky_ext = {8'b0, sched_ky};

    // A slot is reusable only after the last output row that reads its old
    // source row has issued the final ky=0 tap for group 15. This exposes the
    // 18 remaining MAC cycles for the incoming row's 16 beats plus final word.
    wire old_slot_present = row_ready[next_input_slot] &&
                            (row_tags[next_input_slot*8 +: 8] != next_input_y);
    wire [7:0] old_row_tag = row_tags[next_input_slot*8 +: 8];
    always @* begin
        row_permit = !old_slot_present;
        if (old_slot_present) begin
            if (row_active && (sched_y > (old_row_tag + 8'd1)))
                row_permit = 1'b1;
            // **`sched_sub` is load-bearing here.**  The old order walked
            // `ci` innermost and `ky` outside `group`, so `ky != 0` at group
            // 15 meant the row was finished.  `ky` now sits *inside* `sub`
            // and cycles four times a group, so the same test fired at sub 0
            // and let the incoming row overwrite a slot that sub-groups 1 to 3
            // had not read yet.  The last read of `old_row_tag` is at group 15,
            // sub 3, ky 0 -- so the permit has to wait for sub 3.
            else if (row_active && (sched_y == (old_row_tag + 8'd1)) &&
                     (sched_group == 4'd15) && (sched_sub == 2'd3) &&
                     (sched_ky != 0))
                row_permit = 1'b1;
            else if (!row_active && (sched_y > (old_row_tag + 8'd1)))
                row_permit = 1'b1;
        end
    end

    reg req_valid;
    wire issue_slot = row_active;
    wire [3:0] weight_tap = (sched_ky * 4'd3) + sched_kx;
    assign win_rd_en = issue_slot;
    assign win_rd_row = reflect_y(sched_y_ext + sched_ky_ext - 10'sd1);
    assign win_rd_group = sched_group;
    assign win_rd_sub = sched_sub;
    // d64_weight_store gates its registered output with rd_en. Keep that mask
    // open for the cycle in which the synchronous RAM result is captured.
    assign wt_rd_en = issue_slot || req_valid;
    assign wt_rd_addr = {4'd0, weight_tap}; // frozen HEAD weight base = 0

    reg req_first;
    reg [7:0] req_y;
    reg [3:0] req_group;
    reg [1:0] req_ky, req_kx, req_sub;

    reg op_valid;
    reg op_last;
    reg [7:0] op_y;
    reg [3:0] op_group;
    reg [1:0] op_ky, op_kx, op_sub;
    reg [1407:0] op_act;
    reg [1023:0] op_wgt;
    reg [4095:0] bias_bus;
    reg [255:0] rq_m_bus;
    reg [79:0] rq_shift_bus;

    function [31:0] head_bias;
        input [2:0] channel;
        begin
            case (channel)
                0: head_bias = 32'sd369;
                1: head_bias = 32'sd782;
                2: head_bias = 32'sd464;
                3: head_bias = 32'sd408;
                4: head_bias = 32'sd396;
                5: head_bias = -32'sd544;
                6: head_bias = 32'sd5;
                default: head_bias = 32'sd632;
            endcase
        end
    endfunction

    function [15:0] head_mult;
        input [2:0] channel;
        begin
            case (channel)
                0: head_mult = 16'd53472;
                1: head_mult = 16'd43076;
                2: head_mult = 16'd41533;
                3: head_mult = 16'd50218;
                4: head_mult = 16'd36485;
                5: head_mult = 16'd40402;
                6: head_mult = 16'd33803;
                default: head_mult = 16'd64697;
            endcase
        end
    endfunction

    function [4:0] head_shift;
        input [2:0] channel;
        begin
            head_shift = (channel == 3'd7) ? 5'd25 : 5'd24;
        end
    endfunction

    wire [1023:0] profile_head_words;
    generate if (RUNTIME_PROFILE != 0) begin : g_profile_head
        d64_front_profile_rom u_profile (
            .style(profile_style), .page(2'd0), .words(profile_head_words));
    end else begin : g_frozen_head
        assign profile_head_words = 1024'd0;
    end endgenerate

    integer lane;
    integer e_pix;                 // pixel inside the four-pixel group
    integer e_oc;                  // output channel
    integer e_ic;                  // input channel; 3 is the zero pad
    integer e_byte;
    reg [143:0] e_word;
    reg [1407:0] expanded_act;
    reg [1023:0] expanded_wgt;
    // **The lane number is `(pixel*OC + oc)*IC + ic` and the accumulator is
    // `pixel*OC + oc`** -- the bank's own numbering, reduction innermost.  That
    // is where the whole saving comes from: four pixels against eight output
    // channels is 32 accumulators, not 128, because the four input channels go
    // through the reduction tree instead of one slot each.
    //
    // **`IC 3` is why the fourth lane is a zero.**  Head reads R, G and B; the
    // shared heap has levels at 1, 2, 4 and 8 and none at three, so the slot is
    // filled to four and the fourth product is zeroed on **both** sides -- the
    // activation and the weight.  Zeroing only one of them would leave the two
    // sides agreeing by the accident that the other is zero.
    //
    // The window word covers x = 16*group-1 .. 16*group+16, so output pixel
    // `16*group + 4*sub + e_pix` at tap `kx` reads byte `4*sub + e_pix + kx`.
    // That index peaks at 3*4+3+2 = 17 and the word is 18 bytes, which is
    // exactly what the two-pixel halo was for.
    always @* begin
        expanded_act = {1408{1'b0}};
        expanded_wgt = {1024{1'b0}};
        for (lane = 0; lane < 128; lane = lane + 1) begin
            e_pix  = lane / 32;
            e_oc   = (lane / 4) % 8;
            e_ic   = lane % 4;
            e_byte = e_pix + req_kx;
            if (e_ic < 3) begin
                e_word = win_rd_data[e_ic*144 +: 144];
                expanded_act[lane*11 +: 11] = {3'b000, e_word[e_byte*8 +: 8]};
                expanded_wgt[lane*8 +: 8] =
                    wt_rd_data[((e_oc * 3 + e_ic) * 8) +: 8];
            end
        end
    end

    wire [79:0] mac_y10;
    reg [63:0] mac_y8;
    wire [7:0] mac_y_valid;
    wire mac_rq_ready, mac_rq_ready_n, mac_rq_busy, mac_rq_done;
    wire mac_rq_overrun;
    wire [31:0] mac_mode_viol;
    wire [4095:0] acc_bus, snap_bus;
    wire mac_rq_start = op_valid && op_last && mac_rq_ready;
    wire mac_pix_start = req_valid && req_first;
    wire [1407:0] mac_act = EXTERNAL_PE ? expanded_act : op_act;
    wire [1023:0] mac_wgt = op_wgt;
    wire mac_slot_valid = op_valid;
    wire last_frame_mac_slot = mac_slot_valid && op_last &&
                               (op_y == H - 1) && (op_group == 4'd15) &&
                               (op_sub == 2'd3);
    // After the final EOF has been observed, rq_accept has cleared acc_dirty.
    // The explicit pipeline/busy terms make the safe mode-change boundary
    // visible at this layer seam.
    wire [9:0] mac_rq_zero = 10'd0;
    wire [9:0] mac_rq_qmin = 10'd0;
    wire [9:0] mac_rq_qmax = 10'd255;
    assign pe_req_slot_valid = mac_slot_valid;
    assign pe_req_act = mac_act;
    assign pe_req_wgt = mac_wgt;
    assign pe_req_pix_start = mac_pix_start;
    assign pe_req_bias = bias_bus;
    assign pe_req_rq_start = mac_rq_start;
    assign pe_req_rq_m = rq_m_bus;
    assign pe_req_rq_shift = rq_shift_bus;
    assign pe_req_rq_zero = mac_rq_zero;
    assign pe_req_rq_qmin = mac_rq_qmin;
    assign pe_req_rq_qmax = mac_rq_qmax;
    assign mac_mode_safe = !mac_slot_valid && !op_valid && !req_valid &&
                           !mac_rq_busy;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_pe
            assign mac_rq_ready = pe_rsp_rq_ready;
            assign mac_rq_ready_n = pe_rsp_rq_ready_n;
            assign mac_rq_busy = pe_rsp_rq_busy;
            assign mac_rq_done = pe_rsp_rq_done;
            assign mac_rq_overrun = pe_rsp_rq_overrun;
            assign mac_mode_viol = pe_rsp_mode_viol;
            assign acc_bus = pe_rsp_acc;
            assign snap_bus = pe_rsp_snapshot;
            assign mac_y10 = pe_rsp_y;
            assign mac_y_valid = pe_rsp_y_valid;
        end else begin : g_local_pe
            d64_mac128 #(.ACT_W(11), .WGT_W(8), .ACC_W(32), .M_W(16),
                         .SHIFT_W(5), .OUT_W(10), .RQ_LANES(8), .LANES(128),
                         .OC_MAX(16), .RQ_LOOKAHEAD(3)) u_mac (
                .clk(clk), .rst_n(rst_n), .mode(pe_mode),
                .slot_valid(mac_slot_valid), .act(mac_act), .wgt(mac_wgt),
                .pix_start(mac_pix_start), .bias(bias_bus),
                .rq_start(mac_rq_start), .rq_ready(mac_rq_ready),
                .rq_ready_n(mac_rq_ready_n), .rq_m(rq_m_bus),
                .rq_shift(rq_shift_bus), .rq_zero(mac_rq_zero),
                .rq_qmin(mac_rq_qmin), .rq_qmax(mac_rq_qmax),
                .y(mac_y10), .y_valid(mac_y_valid), .rq_busy(mac_rq_busy),
                .rq_done(mac_rq_done), .rq_overrun(mac_rq_overrun),
                .mode_viol(mac_mode_viol), .acc_o(acc_bus), .rq_snap_o(snap_bus)
            );
        end
    endgenerate

    integer out_lane;
    always @* begin
        mac_y8 = 64'd0;
        for (out_lane = 0; out_lane < 8; out_lane = out_lane + 1)
            mac_y8[out_lane*8 +: 8] = mac_y10[out_lane*10 +: 8];
    end

    reg [7:0] out_y_q;
    reg [7:0] out_x0_q;
    reg [4:0] out_pixel_index;
    reg [31:0] out_frame_id_q;
    reg out_frame_id_valid_q;
    assign head_out_valid = mac_y_valid[0];
    assign head_out_x = out_x0_q + {3'b000, out_pixel_index};
    assign head_out_y = out_y_q;
    assign head_out_c8 = mac_y8[63:0];
    assign head_out_frame_id = out_frame_id_q;
    assign head_out_sof = head_out_valid && (head_out_x == 0) &&
                          (head_out_y == 0);
    // EOL is a width and EOF is a height.  Width does not move; both are
    // written as functions of the shape so the next change cannot leave a
    // boundary behind.
    assign head_out_eol = head_out_valid && (head_out_x == W - 1);
    assign head_out_eof = head_out_eol && (head_out_y == H - 1);

    assign rq_overrun = mac_rq_overrun ? 32'd1 : 32'd0;
    assign mode_viol = mac_mode_viol;

    integer p, ch;
    reg [7:0] ysrc0, ysrc1, ysrc2;
    reg triplet_now;
    always @* begin
        ysrc0 = reflect_y(sched_y_ext - 10'sd1);
        ysrc1 = sched_y;
        ysrc2 = reflect_y(sched_y_ext + 10'sd1);
        triplet_now = row_present(ysrc0, row_ready, row_tags) &&
                      row_present(ysrc1, row_ready, row_tags) &&
                      row_present(ysrc2, row_ready, row_tags);
    end

    // Nine slots a sub-group, not twenty-seven: the `ci` walk folded into the
    // reduction tree.  The layer goes from 110,592 slots to 147,456, which is
    // the 3/4 the padding costs and nothing else.
    wire slot_last = (sched_ky == 2'd2) && (sched_kx == 2'd2);
    wire group_last = (sched_group == 4'd15) && (sched_sub == 2'd3) && slot_last;
    // Admit a complete nine-tap/four-pixel phase only with downstream credit.
    // Once started, keep its taps together; req/op/RQ outputs already in flight
    // are covered by the store margin and keep their original capture latency.
    wire phase_first = (sched_ky==0) && (sched_kx==0);
    wire can_issue = row_active && (sched_ky < 3) && (sched_kx < 3) &&
                     (!HEAD_STORE_CREDIT_ENABLE || !phase_first || head_phase_ready) &&
                     (!slot_last || mac_rq_ready_n);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sched_y <= 8'd0;
            sched_group <= 4'd0;
            sched_ky <= 2'd0;
            sched_kx <= 2'd0;
            sched_sub <= 2'd0;
            row_active <= 1'b0;
            sched_done <= 1'b0;
            final_eof_seen <= 1'b0;
            done <= 1'b0;
            compute_active <= 1'b0;
            req_valid <= 1'b0;
            req_first <= 1'b0;
            req_y <= 8'd0;
            req_group <= 4'd0;
            req_ky <= 2'd0;
            req_kx <= 2'd0;
            req_sub <= 2'd0;
            op_valid <= 1'b0;
            op_last <= 1'b0;
            op_y <= 8'd0;
            op_group <= 4'd0;
            op_ky <= 2'd0;
            op_kx <= 2'd0;
            op_sub <= 2'd0;
            op_act <= {1408{1'b0}};
            if(EXTERNAL_PE==0) op_wgt <= {1024{1'b0}};
            bias_bus <= {4096{1'b0}};
            rq_m_bus <= {256{1'b0}};
            rq_shift_bus <= {80{1'b0}};
            out_y_q <= 8'd0;
            out_x0_q <= 8'd0;
            out_pixel_index <= 5'd0;
            out_frame_id_q <= 32'd0;
            out_frame_id_valid_q <= 1'b0;
            mac_slots <= 32'd0;
            head_compute_cycles <= 32'd0;
            rq_wait <= 32'd0;
        end else begin
            req_valid <= can_issue;
            // Once a sub-group, not once a group.  `ky` and `kx` both reset
            // when `sub` advances, so this is the first slot of a sub-group
            // and the accumulator takes its bias there.
            req_first <= can_issue && (sched_ky == 0) && (sched_kx == 0);
            if (can_issue) begin
                req_y <= sched_y;
                req_group <= sched_group;
                req_ky <= sched_ky;
                req_kx <= sched_kx;
                req_sub <= sched_sub;
            end

            // RAM output -> this registered operand/control fabric -> MAC.
            op_valid <= req_valid && win_rd_valid;
            op_last <= req_valid && win_rd_valid &&
                       (req_ky == 2'd2) && (req_kx == 2'd2);
            if (req_valid && win_rd_valid) begin
                op_y <= req_y;
                op_group <= req_group;
                op_ky <= req_ky;
                op_kx <= req_kx;
                op_sub <= req_sub;
                op_act <= expanded_act;
                if(EXTERNAL_PE==0) op_wgt <= expanded_wgt;
            end

            // Build the frozen bias and requant configuration once after reset.
            if (!out_frame_id_valid_q && frame_id_valid) begin
                out_frame_id_q <= frame_id;
                out_frame_id_valid_q <= 1'b1;
                for (p = 0; p < 16; p = p + 1)
                    for (ch = 0; ch < 8; ch = ch + 1)
                        bias_bus[(p*8+ch)*32 +: 32] <= RUNTIME_PROFILE ?
                            profile_head_words[ch*64 +: 32] : head_bias(ch[2:0]);
                for (ch = 0; ch < 8; ch = ch + 1) begin
                    rq_m_bus[ch*16 +: 16] <= RUNTIME_PROFILE ?
                        profile_head_words[ch*64+32 +: 16] : head_mult(ch[2:0]);
                    rq_shift_bus[ch*5 +: 5] <= RUNTIME_PROFILE ?
                        profile_head_words[ch*64+48 +: 5] : head_shift(ch[2:0]);
                end
            end

            if (!row_active && !sched_done && triplet_now)
                row_active <= 1'b1;

            if (head_out_eof)
                final_eof_seen <= 1'b1;
            if (sched_done && final_eof_seen && mac_mode_safe)
                done <= 1'b1;

            if (row_active && slot_last && !mac_rq_ready_n)
                rq_wait <= rq_wait + 32'd1;

            // `kx` innermost, then `ky`, then four-pixel `sub`. The native
            // window uses sub in its RAM address. Each returned color region
            // carries six samples, so the sample index is pixel+kx.
            if (can_issue) begin
                if (sched_kx != 2'd2) sched_kx <= sched_kx + 2'd1;
                else begin
                    sched_kx <= 2'd0;
                    if (sched_ky != 2'd2) sched_ky <= sched_ky + 2'd1;
                    else begin
                        sched_ky <= 2'd0;
                        if (sched_sub != 2'd3) sched_sub <= sched_sub + 2'd1;
                        else begin
                            sched_sub <= 2'd0;
                            if (sched_group != 4'd15)
                                sched_group <= sched_group + 4'd1;
                            else begin
                                sched_group <= 4'd0;
                                row_active <= 1'b0;
                                if (sched_y == H - 1) sched_done <= 1'b1;
                                else sched_y <= sched_y + 8'd1;
                            end
                        end
                    end
                end
            end

            if (mac_slot_valid)
                mac_slots <= mac_slots + 32'd1;

            if (mac_slot_valid && !compute_active) begin
                compute_active <= 1'b1;
                head_compute_cycles <= 32'd1;
            end else if (compute_active) begin
                head_compute_cycles <= head_compute_cycles + 32'd1;
            end
            if (last_frame_mac_slot) begin
                compute_active <= 1'b0;
            end

            if (mac_rq_start) begin
                out_y_q <= op_y;
                // Four requant passes a sub-group and one pixel a pass, so the
                // pass index walks 0..3 and the sub-group supplies the low two
                // bits of x.
                out_x0_q <= {op_group, 4'b0000} + {op_sub, 2'b00};
                out_pixel_index <= 5'd0;
            end else if (head_out_valid && (out_pixel_index < 5'd4)) begin
                out_pixel_index <= out_pixel_index + 5'd1;
            end
        end
    end
    // External bank consumes weights only with its valid slot. Sampling the
    // common response every clock, with no owner-specific mask/reset, allows
    // equivalent raw response bits to share FFs across mutually exclusive engines.
    // The standalone backend retains its original reset and capture contract.
    generate if(EXTERNAL_PE!=0)begin:g_shared_weight_capture
      always @(posedge clk) op_wgt <= expanded_wgt;
    end endgenerate

    // synthesis translate_off
    wire [1407:0] act_registered_ghost=op_act;
    // synthesis translate_on
endmodule
