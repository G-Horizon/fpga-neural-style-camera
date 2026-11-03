// Two-row RAW candidate. Correctness requires same-address READ_FIRST in actual MAP.
`timescale 1ns/1ps
// ============================================================================
// d64_cam_rgb256 -- contract A in, contract B out.  The camera's missing half.
// ============================================================================
//
// Date 2026-10-08, branch `debug/integrate`, owner: camera/display integration.
//
// **Why this module has to exist.**  `docs/D3C6B0_CAMERA_INGRESS.md` fixes two
// boundaries.  Contract A is one unpacked RAW10 sensor sample a cycle with
// raster markers; contract B is the network's frozen input -- sixteen RGB888
// pixels per beat, one beat a row-chunk, `x0` in steps of 16.  The camera
// project (`board/cam/`) implements **neither side of the conversion**: it
// receives CSI-2 and reports numbers over the UART.  `grep -riE
// 'demosaic|bayer|cfa|isp'` over the RTL tree returns no module at all.  So the
// camera half that "already works" is the receive-and-validate half, and the
// produce-an-RGB-frame half does not exist at any commit.  This file is that
// half, and it is new.
//
// What is in it, in order, and what is deliberately not
// -----------------------------------------------------
//
//   1. black-level subtract and one programmable gain, clamped to 10 bits,
//      then the top eight bits. Post-demosaic WB_R/G/B_Q8 and optional
//      gamma2.2 are combinational, with no added frame/marker cycles. Defaults
//      are unity and gamma-off, preserving the original numeric contract.
//      Automatic white balance and a colour-correction matrix remain absent;
//      per-room parameter trials are not a calibrated sensor colour profile.
//   2. **bilinear Bayer demosaic at source resolution**, CFA-programmable, with
//      no default phase assumed.  This is the order the geometry decision
//      requires (`CAMERA_FULL_FOV_DECISION_2026-10-07.md` step 4): demosaic
//      first, resample after.  Demosaicing *after* a non-integer decimation is
//      not available to this design, because a 7.5:1 decimation of a 1920-wide
//      line samples a CFA grid whose phase alternates -- see "the geometry this
//      cannot do" below.
//   3. a **rational** 2:15 horizontal and vertical nearest-sample resample, by
//      Bresenham accumulator, so 1920x1080 becomes 256x144 exactly with no
//      line-rate assumption and no gcd computed at runtime.
//   4. the sixteen-pixel beat packer and a small beat FIFO.
//
// The geometry this cannot do, and it is not a rounding detail
// -----------------------------------------------------------
//
// The obvious cheap pipeline -- take one RAW10 sample in every 7.5 in both axes,
// then demosaic -- **is not legal and this module does not do it**.  A CFA grid
// survives only a decimation whose step and origin are both even, because only
// then does every output sample keep the colour its neighbours expect.  At
// 7.5:1 the sampled grid alternates phase between output columns and there is no
// Bayer pattern to demosaic.  That is the same rule
// `docs/CAM_FRONTEND_GEOMETRY_2026-10-07.md` states and calls load-bearing, and
// it is why the demosaic is here at full resolution where it costs a 3-row line
// buffer and buys an unconditional correctness argument.
//
// The one-column lag, and the check that it costs nothing
// ------------------------------------------------------
//
// A pixel is demosaiced with the sample that arrives one column later, because
// its right-hand neighbour in the row above is not written until then.  The
// source column `cy`'s last output pixel is therefore taken from source column
// `floor((W_OUT-1) * W_SRC / W_OUT)`, and this module can only produce up to
// source column `W_SRC-2`.  At 1920 -> 256 that last index is 1912 and the
// margin is seven columns; `geom_margin_ok` is that comparison, computed from
// the parameters, printed and checked by the testbench, and **a geometry whose
// last output column falls on `W_SRC-1` must not be used.**
//
// Evidence, and what it is not
// ---------------------------
//
// The counters are the module's own and the testbench checks them against an
// expectation computed in the testbench.  **Nothing here is synthesis,
// place-and-route, a board, a sensor, a demosaiced photograph, or a colour
// claim**: no sensor has ever been attached to this module, the CFA phase of the
// SC431HAI is not established (the contract says so in as many words), and the
// frame the gate feeds it is synthetic.
// ============================================================================
module d64_cam_rgb256_pixel_fifo #(
    // The sensor mode.  1920 x 1080 is the mode the board measured --
    // `board/cam/rtl/cam_csi2_top.v`'s own `cam_frame_valid` instance is
    // `H_ACT(1920), V_ACT(1080), PPC(4)` -- and not the 2560 x 1440 array size
    // the contract's hardware table quotes.  **Those two numbers disagree in
    // this tree and this module is written to the measured one.**
    parameter integer W_SRC   = 1920,
    parameter integer H_SRC   = 1080,
    // The network's compute grid.  H_OUT is the residual trunk's shape times
    // four; 144 is the full-field-of-view 16:9 choice.
    parameter integer W_OUT   = 256,
    parameter integer H_OUT   = 144,
    // The CFA phase at source (0,0).  0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR.  **No
    // phase is assumed to be right**: the contract requires a runtime-programmed
    // phase and this parameter is the shell's copy of it.
    parameter [1:0]   CFA     = 2'd0,
    // Black level and gain, both bring-up settings and neither a calibration.
    parameter [9:0]   BLACK   = 10'd64,
    // Q8: 256 is unity.
    parameter [15:0]  GAIN_Q8 = 16'd256,

    // Optional post-demosaic colour processing. Q8 unity=256; supported0..1024.
    // Default unity/gamma-off preserves the original contract bit for bit.
    parameter [15:0] WB_R_Q8 = 16'd256,
    parameter [15:0] WB_G_Q8 = 16'd256,
    parameter [15:0] WB_B_Q8 = 16'd256,
    parameter integer GAMMA_EN = 0,

    // The beat FIFO is `1 << FIFO_AW` deep and its pointers are FIFO_AW bits.
    // **They were eight bits with an eight-deep array, which reads out of range
    // and therefore X from the ninth beat on** -- the gate reported exactly
    // that, `0xxxxxx` on beat 8 of a 32-beat frame, and every count still
    // right, because a FIFO that returns X returns it for a full beat.
    parameter integer FIFO_AW = 3,
    // Verification only, folded away at 0.  At 1 the beat packer puts the
    // colour into the lane its *neighbour* should have -- every address and
    // every marker still right, one pixel's worth of shift.  The gate requires
    // a FAIL on this arm, so that a green run is paired with a red one.
    parameter integer SAB_LANE_SHIFT = 0,
    parameter integer ORIGINAL_ENABLE = 0,
    parameter integer GUIDE_ENABLE = 0,
    parameter integer GUIDE_GAMMA_EN = GAMMA_EN
) (
    input  wire         clk,
    input  wire         rst_n,

    // ---- contract A: the canonical RAW10 pixel stream, one sample a cycle ----
    // `raw_x`/`raw_y` are the receiver's counters and are not inputs here: the
    // adapter that owns them (`csi_pixel_adapter`, contract A) does not exist in
    // this tree, so this module derives position from the markers alone.
    input  wire         raw_valid,
    input  wire [9:0]   raw10,
    input  wire         raw_sof,       // first active pixel of the frame
    input  wire         raw_sol,       // first active pixel of the line
    input  wire         raw_eol,       // last active pixel of the line
    input  wire         raw_eof,       // last active pixel of the frame
    input  wire [31:0]  raw_frame_id,

    // ---- contract B: the frozen 256 x H_OUT RGB888 stream ----
    output wire         d64_valid,
    input  wire         d64_ready,
    output wire [31:0]  d64_frame_id,
    output wire         d64_sof,
    output wire [7:0]   d64_x0,
    output wire [7:0]   d64_y,
    output wire [383:0] d64_rgb888,
    output wire         d64_eol,
    output wire         d64_eof,

    // Full sensor-width de-Bayered green proxy. Rows 0..H_SRC-2 and
    // columns 0..W_SRC-2; the guide writer explicitly extends the two edges.
    output wire         guide_valid,
    output wire [15:0]  guide_x, guide_y,
    output wire [31:0]  guide_frame_id,
    output wire [7:0]   guide_g8,
    // Original colour from full-resolution ISP, even coordinates sampled 2x2.
    // 1920x1080 reaches every 960x540 pixel without edge extrapolation.
    output wire original_valid,
    output wire [15:0] original_rgb565,
    output wire [9:0] original_x,original_y,
    output wire [31:0] original_frame_id,

    // ---- evidence ----
    output reg  [31:0]  src_samples,
    output reg  [31:0]  src_lines,
    output reg  [31:0]  src_frames,
    output reg  [31:0]  demos_px,
    output reg  [31:0]  out_px,
    output reg  [31:0]  beats_out,
    output reg  [31:0]  frames_out,
    output reg  [31:0]  dropped_beats,
    output reg  [31:0]  short_lines,   // a line that ended before W_SRC samples
    output wire         geom_margin_ok
);
    // A single synchronous read from each row bank supplies its RIGHT tap.
    // Two sample registers per bank provide CENTER and LEFT taps. This is
    // three 1920x10 RAMs, each with one read port and at most one write port.
    // It replaces six asynchronous reads from a combined 5760x10 array.
    reg raw_valid_p, raw_sof_p, raw_sol_p, raw_eol_p, raw_eof_p;
    reg [9:0] raw10_p;
    reg [31:0] raw_frame_id_p;
    reg [15:0] input_x;
    reg [1:0] input_head;
    (* ram_style="block" *) reg [9:0] line0 [0:W_SRC-1];
    (* ram_style="block" *) reg [9:0] line1 [0:W_SRC-1];
    reg [9:0] right0, right1;
    reg [9:0] center0, center1;
    reg [9:0] left0, left1;
    wire [15:0] input_addr=raw_sof ? 16'd0 : input_x;
    wire [1:0] input_bank=raw_sof ? 2'd0 : input_head;
    always @(posedge clk) begin
        if(raw_valid) begin
            if(input_bank==0) line0[input_addr]<=raw10;
            if(input_bank==1) line1[input_addr]<=raw10;
            right0<=line0[input_addr];
            right1<=line1[input_addr];
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            raw_valid_p<=0; raw_sof_p<=0; raw_sol_p<=0;
            raw_eol_p<=0; raw_eof_p<=0; raw10_p<=0; raw_frame_id_p<=0;
            input_x<=0; input_head<=0;
            center0<=0; center1<=0;
            left0<=0; left1<=0;
        end else begin
            raw_valid_p<=raw_valid;
            if(raw_valid) begin
                raw_sof_p<=raw_sof; raw_sol_p<=raw_sol;
                raw_eol_p<=raw_eol; raw_eof_p<=raw_eof;
                raw10_p<=raw10; raw_frame_id_p<=raw_frame_id;
                if(raw_eol) begin
                    input_x<=0;
                    input_head<=raw_eof ? 0 :
                        (input_bank^2'd1);
                end else begin
                    input_x<=input_addr+1'b1;
                    input_head<=input_bank;
                end
            end
            if(raw_valid_p) begin
                left0<=center0; left1<=center1;
                center0<=right0; center1<=right1;
            end
        end
    end


    // ------------------------------------------------------------------
    // The geometry's own two conditions, derived and not guessed
    // ------------------------------------------------------------------
    // The resampler this file uses is centred (round-to-nearest): the k-th
    // output sample is taken at source index
    //
    //     i_k = ceil( (W_SRC/2 + k*W_SRC) / W_OUT ) - 1
    //
    // and the demosaic can only produce source indices up to `W_SRC-2` (the
    // one-column lag).  The last output column is `k = W_OUT-1`, and
    // `i_{W_OUT-1} <= W_SRC-2` rearranges exactly to `2*W_OUT <= W_SRC`.
    // **So it is the same condition on both axes and it is a number, not a
    // hope**: at 1920 -> 256 the last index is 1916 against a reachable 1918.
    // A geometry that fails this would silently drop its right-hand column and
    // this wire is what the gate checks.
    assign geom_margin_ok = (2*W_OUT <= W_SRC) && (2*H_OUT <= H_SRC);

    // ------------------------------------------------------------------
    // the CFA decode
    // ------------------------------------------------------------------
    // 0 = R, 1 = G, 2 = B, from the pattern and the pixel's parity.  Written as
    // a function of the *source grid's* parity, which is the whole reason the
    // demosaic runs before the resampler.
    function [1:0] cfa_col;
        input [1:0] pat;
        input       xp;
        input       yp;
        reg   [1:0] c;
        begin
            case (pat)
            2'd0:    c = xp ? (yp ? 2'd2 : 2'd1) : (yp ? 2'd1 : 2'd0); // RGGB
            2'd1:    c = xp ? (yp ? 2'd1 : 2'd0) : (yp ? 2'd2 : 2'd1); // GRBG
            2'd2:    c = xp ? (yp ? 2'd1 : 2'd2) : (yp ? 2'd0 : 2'd1); // GBRG
            default: c = xp ? (yp ? 2'd0 : 2'd1) : (yp ? 2'd1 : 2'd2); // BGGR
            endcase
            cfa_col = c;
        end
    endfunction

    localparam [1:0] COL_R = 2'd0, COL_G = 2'd1, COL_B = 2'd2;

    // ------------------------------------------------------------------
    // the frame's own state
    // ------------------------------------------------------------------
    reg [15:0] sx;              // source column being written
    reg [15:0] sy;              // source row being written
    reg [1:0]  head;            // the 3-row buffer row y is written into
    reg [9:0]  cur_xm1, cur_xm2;// row `sy`, columns sx-1 and sx-2
    reg        keep_row;        // the row being demosaiced is an output row
    reg [15:0] ox;              // output column, 0..W_OUT-1
    reg [15:0] oy;              // output row, 0..H_OUT-1
    reg [31:0] fid_lat;

    // **Row `y` lives in slot `y mod 3`**, because the row slots advance on
    // `raw_eol_p` and the slot written is always `head`.  So row `sy-1` is at
    // `(head+2) mod 3` and row `sy-2` at `(head+1) mod 3` -- and the first
    // version of this file had those two expressions the other way round, so
    // the demosaic read row `sy-2` where it wanted the centre and row `sy-1`
    // where it wanted the row above.  The gate caught it as "every pixel
    // wrong, and wrong in the way a one-row shift looks".
    wire [1:0] head_prev = head^2'd1; // row sy-1
    wire [1:0] head_prev2 = head; // row sy-2

    // ---- the three source rows ------------------------------------------
    // One array, three rows of `W_SRC`, indexed by the physical buffer and the
    // column.  Row `sy` is the one being written; `sy-1` and `sy-2` are read.
    // Row storage is the synchronous line0/1/2 banks above.

    reg signed [31:0] h_acc;    // centred, `W_OUT` per demosaiced column
    reg signed [31:0] v_acc;    // centred, `H_OUT` per source row

    wire signed [31:0] v_step = v_acc + H_OUT;
    wire               v_keep = (v_step >= 0);
    wire signed [31:0] v_nacc = v_keep ? (v_step - H_SRC) : v_step;

    // ---- the demosaic window, combinationally ---------------------------
    // The centre is (cx, cy) = (sx-1, sy-1): the sample that arrives one column
    // later is what makes its right-hand neighbour in the row above available.
    wire [15:0] cx = sx - 16'd1;
    wire [15:0] cx_l = (cx == 16'd0) ? 16'd0 : (cx - 16'd1);
    wire [15:0] cx_r = (cx == 16'd0) ? 16'd1 : (cx + 16'd1);


    wire [9:0] s_c = head_prev==0 ? center0 : center1;
    wire [9:0] s_l0 = head_prev==0 ? left0 : left1;
    wire [9:0] s_l = cx==0 ? s_c : s_l0;
    wire [9:0] s_r = head_prev==0 ? right0 : right1;
    wire [9:0] s_u = head_prev2==0 ? center0 : center1;
    wire [9:0] s_ul0 = head_prev2==0 ? left0 : left1;
    wire [9:0] s_ul = cx==0 ? s_u : s_ul0;
    wire [9:0] s_ur = head_prev2==0 ? right0 : right1;
    wire [9:0] s_d  = cur_xm2;      // row sy, column cx-1
    wire [9:0] s_dr = raw10_p;        // row sy, column cx+1

    // ---- black level, gain, clamp, and the top eight bits ---------------
    // One instance of the photometry, used for all eight taps, so a change to
    // it cannot reach one tap and not another.
    function [7:0] atap;
        input [9:0] v;
        reg   [25:0] t;
        begin
            t = (v > BLACK) ? ((v - BLACK) * GAIN_Q8) : 26'd0;
            t = t >> 8;
            atap = (t > 26'd1023) ? 8'hFF : t[9:2];
        end
    endfunction

    // ---- the bilinear demosaic ------------------------------------------
    // Sums are widened explicitly: in Verilog the width of `a + b` is
    // `max(width a, width b)`, so four eight-bit taps added at eight bits would
    // overflow and the mapper would not warn.
    wire [7:0] t_c  = atap(s_c);
    wire [7:0] t_u  = atap(s_u);
    wire [7:0] t_d  = atap(cur_xm1);   // row sy, column cx
    wire [7:0] t_l  = atap(s_l);
    wire [7:0] t_r  = atap(s_r);
    wire [7:0] t_ul = atap(s_ul);
    wire [7:0] t_ur = atap(s_ur);
    wire [7:0] t_dl = atap(cur_xm2);   // row sy, column cx-1
    wire [7:0] t_dr = atap(s_dr);      // row sy, column cx+1

    // The centred rational grid cannot emit cx/cy zero when these strict
    // integer-half conditions hold. SAB shifts a neighbour into an output,
    // so it deliberately keeps the legacy independent colour arithmetic.
    wire [15:0] cy = sy - 16'd1;
    localparam SHARED_ORIGINAL_RGB=(ORIGINAL_ENABLE!=0) &&
        (W_SRC/2>W_OUT) && (H_SRC/2>H_OUT) && (SAB_LANE_SHIFT==0);
    wire [7:0] dm_u=SHARED_ORIGINAL_RGB && cy==0 ? t_c : t_u;
    wire [7:0] dm_ul=SHARED_ORIGINAL_RGB && cy==0 ? t_l : t_ul;
    wire [7:0] dm_ur=SHARED_ORIGINAL_RGB && cy==0 ? t_r : t_ur;
    wire [7:0] dm_dl=SHARED_ORIGINAL_RGB && cx==0 ? t_d : t_dl;
    wire [9:0] g4s = {2'b0,dm_u} + {2'b0,t_d} + {2'b0,t_l} + {2'b0,t_r};
    wire [9:0] d4s = {2'b0,dm_ul} + {2'b0,dm_ur} + {2'b0,dm_dl} + {2'b0,t_dr};
    wire [8:0] h2s = {1'b0,t_l} + {1'b0,t_r};
    wire [8:0] v2s = {1'b0,dm_u} + {1'b0,t_d};
    // Floors, not rounds, and the testbench's reference floors the same way.
    // A rounding rule that lived in one place and a reference that assumed the
    // other is how a demosaic gate measures nothing.
    wire [7:0] g4 = g4s[9:2];
    wire [7:0] d4 = d4s[9:2];
    wire [7:0] h2 = h2s[8:1];
    wire [7:0] v2 = v2s[8:1];

    wire [1:0]  col_c = cfa_col(CFA, cx[0], cy[0]);
    wire [1:0]  col_u = cfa_col(CFA, cx[0], ~cy[0]);
    wire [1:0]  col_l = cfa_col(CFA, ~cx[0], cy[0]);

    reg [7:0] dm_r, dm_g, dm_b;
    always @* begin
        case (col_c)
        COL_R:   begin dm_r = t_c; dm_g = g4; dm_b = d4; end
        COL_B:   begin dm_b = t_c; dm_g = g4; dm_r = d4; end
        default: begin
            dm_g = t_c;
            if (col_u == COL_R) begin dm_r = v2; dm_b = h2; end
            else                begin dm_b = v2; dm_r = h2; end
        end
        endcase
    end

    // ---- the resample accumulators --------------------------------------
    wire signed [31:0] h_step = h_acc + W_OUT;
    wire               h_emit = (h_step >= 0);
    wire signed [31:0] h_nacc = h_emit ? (h_step - W_SRC) : h_step;

    // ---- the beat packer ------------------------------------------------
    reg  [383:0] beat_px;
    reg  [23:0]  last_px;

    // ISP_COLOUR_FUNCTIONS_BEGIN -- generated by tests/prepare_room_isp_colour.py
    // Fixed gains use shifts/adds, never runtime multipliers. Extend BEFORE
    // shifting, retain saturation bits, round positive values half-up.
    function [7:0] isp_wb_q8;
        input [7:0] pixel;
        input [15:0] gain_q8;
        reg [23:0] wide,sum,rounded;
        integer wb_bit;
        begin
            wide={16'd0,pixel};sum=24'd0;
            for(wb_bit=0;wb_bit<16;wb_bit=wb_bit+1)
                if(gain_q8[wb_bit])sum=sum+(wide<<wb_bit);
            rounded=(sum+24'd128)>>8;
            isp_wb_q8=(rounded>24'd255)?8'd255:rounded[7:0];
        end
    endfunction

    // Exact integer8 gamma2.2 table: round(255*(x/255)^(1/2.2)).
    // No files, memory init or run-time pow; full case has a defined default.
    function [7:0] isp_gamma8;
        input [7:0] pixel;
        begin
            case(pixel)
                8'd0:isp_gamma8=8'd0;
                8'd1:isp_gamma8=8'd21;
                8'd2:isp_gamma8=8'd28;
                8'd3:isp_gamma8=8'd34;
                8'd4:isp_gamma8=8'd39;
                8'd5:isp_gamma8=8'd43;
                8'd6:isp_gamma8=8'd46;
                8'd7:isp_gamma8=8'd50;
                8'd8:isp_gamma8=8'd53;
                8'd9:isp_gamma8=8'd56;
                8'd10:isp_gamma8=8'd59;
                8'd11:isp_gamma8=8'd61;
                8'd12:isp_gamma8=8'd64;
                8'd13:isp_gamma8=8'd66;
                8'd14:isp_gamma8=8'd68;
                8'd15:isp_gamma8=8'd70;
                8'd16:isp_gamma8=8'd72;
                8'd17:isp_gamma8=8'd74;
                8'd18:isp_gamma8=8'd76;
                8'd19:isp_gamma8=8'd78;
                8'd20:isp_gamma8=8'd80;
                8'd21:isp_gamma8=8'd82;
                8'd22:isp_gamma8=8'd84;
                8'd23:isp_gamma8=8'd85;
                8'd24:isp_gamma8=8'd87;
                8'd25:isp_gamma8=8'd89;
                8'd26:isp_gamma8=8'd90;
                8'd27:isp_gamma8=8'd92;
                8'd28:isp_gamma8=8'd93;
                8'd29:isp_gamma8=8'd95;
                8'd30:isp_gamma8=8'd96;
                8'd31:isp_gamma8=8'd98;
                8'd32:isp_gamma8=8'd99;
                8'd33:isp_gamma8=8'd101;
                8'd34:isp_gamma8=8'd102;
                8'd35:isp_gamma8=8'd103;
                8'd36:isp_gamma8=8'd105;
                8'd37:isp_gamma8=8'd106;
                8'd38:isp_gamma8=8'd107;
                8'd39:isp_gamma8=8'd109;
                8'd40:isp_gamma8=8'd110;
                8'd41:isp_gamma8=8'd111;
                8'd42:isp_gamma8=8'd112;
                8'd43:isp_gamma8=8'd114;
                8'd44:isp_gamma8=8'd115;
                8'd45:isp_gamma8=8'd116;
                8'd46:isp_gamma8=8'd117;
                8'd47:isp_gamma8=8'd118;
                8'd48:isp_gamma8=8'd119;
                8'd49:isp_gamma8=8'd120;
                8'd50:isp_gamma8=8'd122;
                8'd51:isp_gamma8=8'd123;
                8'd52:isp_gamma8=8'd124;
                8'd53:isp_gamma8=8'd125;
                8'd54:isp_gamma8=8'd126;
                8'd55:isp_gamma8=8'd127;
                8'd56:isp_gamma8=8'd128;
                8'd57:isp_gamma8=8'd129;
                8'd58:isp_gamma8=8'd130;
                8'd59:isp_gamma8=8'd131;
                8'd60:isp_gamma8=8'd132;
                8'd61:isp_gamma8=8'd133;
                8'd62:isp_gamma8=8'd134;
                8'd63:isp_gamma8=8'd135;
                8'd64:isp_gamma8=8'd136;
                8'd65:isp_gamma8=8'd137;
                8'd66:isp_gamma8=8'd138;
                8'd67:isp_gamma8=8'd139;
                8'd68:isp_gamma8=8'd140;
                8'd69:isp_gamma8=8'd141;
                8'd70:isp_gamma8=8'd142;
                8'd71:isp_gamma8=8'd143;
                8'd72:isp_gamma8=8'd144;
                8'd73:isp_gamma8=8'd144;
                8'd74:isp_gamma8=8'd145;
                8'd75:isp_gamma8=8'd146;
                8'd76:isp_gamma8=8'd147;
                8'd77:isp_gamma8=8'd148;
                8'd78:isp_gamma8=8'd149;
                8'd79:isp_gamma8=8'd150;
                8'd80:isp_gamma8=8'd151;
                8'd81:isp_gamma8=8'd151;
                8'd82:isp_gamma8=8'd152;
                8'd83:isp_gamma8=8'd153;
                8'd84:isp_gamma8=8'd154;
                8'd85:isp_gamma8=8'd155;
                8'd86:isp_gamma8=8'd156;
                8'd87:isp_gamma8=8'd156;
                8'd88:isp_gamma8=8'd157;
                8'd89:isp_gamma8=8'd158;
                8'd90:isp_gamma8=8'd159;
                8'd91:isp_gamma8=8'd160;
                8'd92:isp_gamma8=8'd160;
                8'd93:isp_gamma8=8'd161;
                8'd94:isp_gamma8=8'd162;
                8'd95:isp_gamma8=8'd163;
                8'd96:isp_gamma8=8'd164;
                8'd97:isp_gamma8=8'd164;
                8'd98:isp_gamma8=8'd165;
                8'd99:isp_gamma8=8'd166;
                8'd100:isp_gamma8=8'd167;
                8'd101:isp_gamma8=8'd167;
                8'd102:isp_gamma8=8'd168;
                8'd103:isp_gamma8=8'd169;
                8'd104:isp_gamma8=8'd170;
                8'd105:isp_gamma8=8'd170;
                8'd106:isp_gamma8=8'd171;
                8'd107:isp_gamma8=8'd172;
                8'd108:isp_gamma8=8'd173;
                8'd109:isp_gamma8=8'd173;
                8'd110:isp_gamma8=8'd174;
                8'd111:isp_gamma8=8'd175;
                8'd112:isp_gamma8=8'd175;
                8'd113:isp_gamma8=8'd176;
                8'd114:isp_gamma8=8'd177;
                8'd115:isp_gamma8=8'd178;
                8'd116:isp_gamma8=8'd178;
                8'd117:isp_gamma8=8'd179;
                8'd118:isp_gamma8=8'd180;
                8'd119:isp_gamma8=8'd180;
                8'd120:isp_gamma8=8'd181;
                8'd121:isp_gamma8=8'd182;
                8'd122:isp_gamma8=8'd182;
                8'd123:isp_gamma8=8'd183;
                8'd124:isp_gamma8=8'd184;
                8'd125:isp_gamma8=8'd184;
                8'd126:isp_gamma8=8'd185;
                8'd127:isp_gamma8=8'd186;
                8'd128:isp_gamma8=8'd186;
                8'd129:isp_gamma8=8'd187;
                8'd130:isp_gamma8=8'd188;
                8'd131:isp_gamma8=8'd188;
                8'd132:isp_gamma8=8'd189;
                8'd133:isp_gamma8=8'd190;
                8'd134:isp_gamma8=8'd190;
                8'd135:isp_gamma8=8'd191;
                8'd136:isp_gamma8=8'd192;
                8'd137:isp_gamma8=8'd192;
                8'd138:isp_gamma8=8'd193;
                8'd139:isp_gamma8=8'd194;
                8'd140:isp_gamma8=8'd194;
                8'd141:isp_gamma8=8'd195;
                8'd142:isp_gamma8=8'd195;
                8'd143:isp_gamma8=8'd196;
                8'd144:isp_gamma8=8'd197;
                8'd145:isp_gamma8=8'd197;
                8'd146:isp_gamma8=8'd198;
                8'd147:isp_gamma8=8'd199;
                8'd148:isp_gamma8=8'd199;
                8'd149:isp_gamma8=8'd200;
                8'd150:isp_gamma8=8'd200;
                8'd151:isp_gamma8=8'd201;
                8'd152:isp_gamma8=8'd202;
                8'd153:isp_gamma8=8'd202;
                8'd154:isp_gamma8=8'd203;
                8'd155:isp_gamma8=8'd203;
                8'd156:isp_gamma8=8'd204;
                8'd157:isp_gamma8=8'd205;
                8'd158:isp_gamma8=8'd205;
                8'd159:isp_gamma8=8'd206;
                8'd160:isp_gamma8=8'd206;
                8'd161:isp_gamma8=8'd207;
                8'd162:isp_gamma8=8'd207;
                8'd163:isp_gamma8=8'd208;
                8'd164:isp_gamma8=8'd209;
                8'd165:isp_gamma8=8'd209;
                8'd166:isp_gamma8=8'd210;
                8'd167:isp_gamma8=8'd210;
                8'd168:isp_gamma8=8'd211;
                8'd169:isp_gamma8=8'd212;
                8'd170:isp_gamma8=8'd212;
                8'd171:isp_gamma8=8'd213;
                8'd172:isp_gamma8=8'd213;
                8'd173:isp_gamma8=8'd214;
                8'd174:isp_gamma8=8'd214;
                8'd175:isp_gamma8=8'd215;
                8'd176:isp_gamma8=8'd215;
                8'd177:isp_gamma8=8'd216;
                8'd178:isp_gamma8=8'd217;
                8'd179:isp_gamma8=8'd217;
                8'd180:isp_gamma8=8'd218;
                8'd181:isp_gamma8=8'd218;
                8'd182:isp_gamma8=8'd219;
                8'd183:isp_gamma8=8'd219;
                8'd184:isp_gamma8=8'd220;
                8'd185:isp_gamma8=8'd220;
                8'd186:isp_gamma8=8'd221;
                8'd187:isp_gamma8=8'd221;
                8'd188:isp_gamma8=8'd222;
                8'd189:isp_gamma8=8'd223;
                8'd190:isp_gamma8=8'd223;
                8'd191:isp_gamma8=8'd224;
                8'd192:isp_gamma8=8'd224;
                8'd193:isp_gamma8=8'd225;
                8'd194:isp_gamma8=8'd225;
                8'd195:isp_gamma8=8'd226;
                8'd196:isp_gamma8=8'd226;
                8'd197:isp_gamma8=8'd227;
                8'd198:isp_gamma8=8'd227;
                8'd199:isp_gamma8=8'd228;
                8'd200:isp_gamma8=8'd228;
                8'd201:isp_gamma8=8'd229;
                8'd202:isp_gamma8=8'd229;
                8'd203:isp_gamma8=8'd230;
                8'd204:isp_gamma8=8'd230;
                8'd205:isp_gamma8=8'd231;
                8'd206:isp_gamma8=8'd231;
                8'd207:isp_gamma8=8'd232;
                8'd208:isp_gamma8=8'd232;
                8'd209:isp_gamma8=8'd233;
                8'd210:isp_gamma8=8'd233;
                8'd211:isp_gamma8=8'd234;
                8'd212:isp_gamma8=8'd234;
                8'd213:isp_gamma8=8'd235;
                8'd214:isp_gamma8=8'd235;
                8'd215:isp_gamma8=8'd236;
                8'd216:isp_gamma8=8'd236;
                8'd217:isp_gamma8=8'd237;
                8'd218:isp_gamma8=8'd237;
                8'd219:isp_gamma8=8'd238;
                8'd220:isp_gamma8=8'd238;
                8'd221:isp_gamma8=8'd239;
                8'd222:isp_gamma8=8'd239;
                8'd223:isp_gamma8=8'd240;
                8'd224:isp_gamma8=8'd240;
                8'd225:isp_gamma8=8'd241;
                8'd226:isp_gamma8=8'd241;
                8'd227:isp_gamma8=8'd242;
                8'd228:isp_gamma8=8'd242;
                8'd229:isp_gamma8=8'd243;
                8'd230:isp_gamma8=8'd243;
                8'd231:isp_gamma8=8'd244;
                8'd232:isp_gamma8=8'd244;
                8'd233:isp_gamma8=8'd245;
                8'd234:isp_gamma8=8'd245;
                8'd235:isp_gamma8=8'd246;
                8'd236:isp_gamma8=8'd246;
                8'd237:isp_gamma8=8'd247;
                8'd238:isp_gamma8=8'd247;
                8'd239:isp_gamma8=8'd248;
                8'd240:isp_gamma8=8'd248;
                8'd241:isp_gamma8=8'd249;
                8'd242:isp_gamma8=8'd249;
                8'd243:isp_gamma8=8'd249;
                8'd244:isp_gamma8=8'd250;
                8'd245:isp_gamma8=8'd250;
                8'd246:isp_gamma8=8'd251;
                8'd247:isp_gamma8=8'd251;
                8'd248:isp_gamma8=8'd252;
                8'd249:isp_gamma8=8'd252;
                8'd250:isp_gamma8=8'd253;
                8'd251:isp_gamma8=8'd253;
                8'd252:isp_gamma8=8'd254;
                8'd253:isp_gamma8=8'd254;
                8'd254:isp_gamma8=8'd255;
                8'd255:isp_gamma8=8'd255;
                default:isp_gamma8=8'd0;
            endcase
        end
    endfunction
    // Simulation application-contract check only. EFX_MAP does not execute
    // this guard. The root must validate static JSON/parameters before freeze.
    // 24-bit arithmetic itself covers the entire16bit unsigned gain range.
    // synthesis translate_off
    initial begin
        if(WB_R_Q8>16'd1024 || WB_G_Q8>16'd1024 || WB_B_Q8>16'd1024)
            $fatal(1,"INVALID_ISP_WB_Q8_SUPPORTED_RANGE_0_TO_1024");
        if(GAMMA_EN!=0 && GAMMA_EN!=1)
            $fatal(1,"INVALID_ISP_GAMMA_EN_MUST_BE_0_OR_1");
    end
    // synthesis translate_on
    // ISP_COLOUR_FUNCTIONS_END

    wire [7:0] wb_r = isp_wb_q8(dm_r,WB_R_Q8);
    wire [7:0] wb_g = isp_wb_q8(dm_g,WB_G_Q8);
    wire [7:0] wb_b = isp_wb_q8(dm_b,WB_B_Q8);
    wire [23:0] px_dm = GAMMA_EN ?
        {isp_gamma8(wb_r),isp_gamma8(wb_g),isp_gamma8(wb_b)} : {wb_r,wb_g,wb_b};
    wire [23:0] px_in = SAB_LANE_SHIFT ? last_px : px_dm;

    wire [7:0] guide_upper=(cy==16'd0)?t_c:t_u;
    wire [9:0] guide_gsum={2'd0,guide_upper}+{2'd0,t_d}+{2'd0,t_l}+{2'd0,t_r};
    wire [7:0] guide_green=(col_c==COL_G)?t_c:guide_gsum[9:2];
    wire [7:0] guide_wb=isp_wb_q8(guide_green,WB_G_Q8);
    assign guide_valid=(GUIDE_ENABLE!=0)&&raw_valid_p&&sx>=16'd1&&sx<W_SRC&&sy>=16'd1&&sy<H_SRC;
    assign guide_x=cx;
    assign guide_y=cy;
    assign guide_frame_id=fid_lat;
    assign guide_g8=GUIDE_GAMMA_EN?isp_gamma8(guide_wb):guide_wb;
    assign original_valid=(ORIGINAL_ENABLE!=0)&&raw_valid_p&&
        sx>=16'd1&&sx<W_SRC&&sy>=16'd1&&sy<H_SRC&&!cx[0]&&!cy[0];
    assign original_x=cx[10:1];assign original_y=cy[10:1];
    assign original_frame_id=fid_lat;
    // The CNN never samples source row0/column0. Its legacy taps remain
    // unchanged; original colour independently replicates missing top/left
    // neighbours so unwritten row RAM cannot enter the visible first row.
    wire [7:0] original_u=cy==0?t_c:t_u;
    wire [7:0] original_ul=cy==0?t_l:t_ul;
    wire [7:0] original_ur=cy==0?t_r:t_ur;
    wire [7:0] original_dl=cx==0?t_d:t_dl;
    wire [9:0] original_gsum={2'd0,original_u}+{2'd0,t_d}+{2'd0,t_l}+{2'd0,t_r};
    wire [9:0] original_dsum={2'd0,original_ul}+{2'd0,original_ur}+{2'd0,original_dl}+{2'd0,t_dr};
    wire [8:0] original_vsum={1'b0,original_u}+{1'b0,t_d};
    reg [7:0] original_r,original_g,original_b;
    always @* begin
        case(col_c)
            COL_R:begin original_r=t_c;original_g=original_gsum[9:2];original_b=original_dsum[9:2];end
            COL_B:begin original_b=t_c;original_g=original_gsum[9:2];original_r=original_dsum[9:2];end
            default:begin
                original_g=t_c;
                if(col_u==COL_R)begin original_r=original_vsum[8:1];original_b=h2;end
                else begin original_b=original_vsum[8:1];original_r=h2;end
            end
        endcase
    end
    wire [7:0] original_wr=isp_wb_q8(original_r,WB_R_Q8);
    wire [7:0] original_wg=isp_wb_q8(original_g,WB_G_Q8);
    wire [7:0] original_wb=isp_wb_q8(original_b,WB_B_Q8);
    wire [23:0] original_colour=SHARED_ORIGINAL_RGB ? px_dm : GAMMA_EN?
        {isp_gamma8(original_wr),isp_gamma8(original_wg),isp_gamma8(original_wb)}:
        {original_wr,original_wg,original_wb};
    assign original_rgb565={original_colour[23:19],original_colour[15:10],original_colour[7:3]};

    // **`sx` is the column of the sample presented on this cycle**, and the
    // line and row slots advance on `raw_eol_p` rather than on the next
    // `raw_sol_p` -- see the sequencer.  `dm_fire` therefore sits inside the
    // "an ordinary column of the line" arm and its bounds are the ones the
    // closed form in the testbench assumes.
    wire dm_fire   = raw_valid_p && (sx >= 16'd1) && (sx < W_SRC)
                     && (sy >= 16'd1) && keep_row;
    wire pix_fire  = dm_fire && h_emit;
    wire beat_push = pix_fire && (ox[3:0] == 4'd15);

    wire [7:0]  beat_x0 = {ox[7:4], 4'b0000};
    wire        beat_sof = (oy == 16'd0) && (ox[7:4] == 12'd0);
    // The `x` field is not stored: every beat of a row carries `eol` and the
    // row's last beat is the one whose `x0` is `W_OUT-16`, which is the same
    // thing as `ox == W_OUT-1`.  **One `1'b1` for the whole field was the second
    // version's error**: at 256 wide a row is sixteen beats so every beat *is*
    // the row's last, and the mistake is invisible there; at the gate's 32-wide
    // geometry a row is two beats and the first one was marked `eol`.
    wire beat_eol = (ox == W_OUT-1);

    wire        beat_eof = (oy == H_OUT-1) && (ox == W_OUT-1);

    wire [402:0] beat_word = { beat_eof, beat_eol, beat_sof, beat_x0,
                               oy[7:0], px_in, beat_px[359:0] };

    // ---- the beat FIFO --------------------------------------------------
    // Eight beats.  Contract A says the sensor is never stalled by network
    // backpressure, so a full FIFO is a lost beat and a counter -- not a stall.
    // The contract's answer to sustained backpressure is the capture-side
    // ping-pong in DDR (`d64_rgb256_frame_source`), which is **not built**;
    // this FIFO is what a build without it has.

    // Candidate only: keep pixels in a synchronous 71-bit FIFO instead of
    // 403-bit asynchronous-read beat FIFO. Reserve 16 entries at packet start
    // so overflow discards a WHOLE beat, never half of one.
    localparam integer PAW=FIFO_AW+4;
    localparam integer PIXEL_DEPTH=1<<PAW;
    (* ram_style="block" *) reg [70:0] pixel_mem [0:PIXEL_DEPTH-1];
    reg [PAW-1:0] pixel_wp,pixel_rp;
    reg [PAW:0] pixel_count;
    reg packet_drop;
    wire packet_room=pixel_count<=PIXEL_DEPTH-16;
    wire this_drop=(ox[3:0]==0) ? !packet_room : packet_drop;
    wire pixel_push=pix_fire && !this_drop;
    wire [70:0] pixel_wd={fid_lat,oy[7:0],ox[7:4],beat_eof,beat_eol,
                        (oy==0 && ox==0),px_in};
    reg [70:0] pixel_rd;
    reg read_pending;
    reg output_valid;
    reg [383:0] output_pixels;
    reg [31:0] output_fid;
    reg output_sof,output_eol,output_eof,assembly_sof;
    reg [7:0] output_x0,output_y,receive_x,receive_y;
    reg [3:0] assembly_i;
    wire pixel_read=(pixel_count!=0) && !read_pending && !output_valid;
    assign d64_valid=output_valid;
    assign d64_rgb888=output_pixels;
    assign d64_frame_id=output_fid;
    assign d64_sof=output_sof;
    assign d64_eol=output_eol;
    assign d64_eof=output_eof;
    assign d64_x0=output_x0;
    assign d64_y=output_y;
    always @(posedge clk) begin
        if(pixel_push) pixel_mem[pixel_wp]<=pixel_wd;
        if(pixel_read) pixel_rd<=pixel_mem[pixel_rp];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pixel_rp<=0;read_pending<=0;output_valid<=0;
            output_pixels<=0;output_fid<=0;
            output_sof<=0;output_eol<=0;output_eof<=0;
            output_x0<=0;output_y<=0;receive_x<=0;receive_y<=0;
            assembly_i<=0;assembly_sof<=0;
            beats_out<=0;frames_out<=0;
        end else begin
            read_pending<=pixel_read;
            if(pixel_read) pixel_rp<=pixel_rp+1'b1;
            if(output_valid && d64_ready) begin
                output_valid<=0;
                beats_out<=beats_out+1;
                if(output_eof) frames_out<=frames_out+1;
            end
            if(read_pending) begin
                output_pixels[assembly_i*24+:24]<=pixel_rd[23:0];
                if(assembly_i==0) assembly_sof<=pixel_rd[24];
                if(assembly_i==15) begin
                    output_fid<=pixel_rd[70:39];
                    output_sof<=assembly_sof;
                    output_eol<=pixel_rd[25];output_eof<=pixel_rd[26];
                    output_x0<={pixel_rd[30:27],4'd0};
                    output_y<=pixel_rd[38:31];
                    output_valid<=1;assembly_i<=0;
                end else begin
                    assembly_i<=assembly_i+1'b1;
                end
                if(pixel_rd[24]) begin
                    receive_x<=1;receive_y<=0;
                end else if(pixel_rd[25]) begin
                    receive_x<=0;receive_y<=receive_y+1'b1;
                end else receive_x<=receive_x+1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // the sequencer -- one always block, so no register has two drivers
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sx <= 16'd0; sy <= 16'd0; head <= 2'd0;
            cur_xm1 <= 10'd0; cur_xm2 <= 10'd0;
            keep_row <= 1'b0; ox <= 16'd0; oy <= 16'd0;
            h_acc <= -(W_SRC/2); v_acc <= -(H_SRC/2);
            fid_lat <= 32'd0;
            beat_px <= 384'd0; last_px <= 24'd0;
            pixel_wp<=0;pixel_count<=0;packet_drop<=0;
            src_samples <= 32'd0; src_lines <= 32'd0; src_frames <= 32'd0;
            demos_px <= 32'd0; out_px <= 32'd0;
            dropped_beats <= 32'd0; short_lines <= 32'd0;
        end
        else begin
            case({pixel_push,pixel_read})
                2'b10:pixel_count<=pixel_count+1'b1;
                2'b01:pixel_count<=pixel_count-1'b1;
                default:pixel_count<=pixel_count;
            endcase
            if(pixel_push) pixel_wp<=pixel_wp+1'b1;
            if(pix_fire && ox[3:0]==0) packet_drop<=!packet_room;
            // ---- the producer side --------------------------------------
            if (raw_valid_p) begin
                src_samples <= src_samples + 32'd1;


                if (raw_sof_p) begin
                    // The frame's own state.  The row slot is already 0: the
                    // previous frame's `raw_eof_p` put it there, so the write of
                    // column 0 of row 0 lands in row slot 0 on this very cycle
                    // and needs no help.
                    oy <= 16'd0; ox <= 16'd0;
                    h_acc <= -(W_SRC/2); v_acc <= -(H_SRC/2);
                    keep_row <= 1'b0;
                    fid_lat <= raw_frame_id_p;
                    // **Column 0 arrives with `raw_sof_p`, so the next cycle is
                    // column 1 -- the same advance the `raw_sol_p` arm makes.**
                    // Without it the frame's first line ran one sample short
                    // (measured: `raw_eol_p` at `sx = 126` against an expected 127,
                    // twice, once a frame) and `short_lines` was 2.
                    sx <= 16'd1;
                    src_frames <= src_frames + 32'd1;
                    src_lines  <= src_lines + 32'd1;
                end
                else if (raw_sol_p) begin
                    // **No row or column bookkeeping here, and that is the
                    // correction this file needed.**  `sy` and `head` were
                    // advanced on `raw_sol_p`, which is one cycle too late: the
                    // column-0 sample of the new row would be written into the
                    // *previous* row's slot, and from the second column on the
                    // demosaic would read the wrong rows -- measured, the gate
                    // reported 1024 of 1024 pixels wrong with the colours of
                    // the taps exchanged, which is what a one-row shift looks
                    // like.  They advance on `raw_eol_p` now, so at this cycle
                    // `sy` is already this row and `sx` is already 0.
                    //
                    // What belongs here is the once-a-row decision: the
                    // vertical keep test for the row about to be demosaiced
                    // (cy = sy-1), and the horizontal accumulator's restart.
                    v_acc <= v_nacc;
                    keep_row <= v_keep;
                    h_acc <= -(W_SRC/2);
                    ox <= 16'd0;
                    // **`sx` advances here too, and missing that is what the
                    // first corrected version got wrong.**  `sx` is the column
                    // of the sample on this cycle and `raw_sol_p` arrives with
                    // column 0, so the next cycle is column 1.  Leaving `sx`
                    // alone in this branch stalled it at 0 for one cycle, which
                    // shifted every write in the line buffer one column left and
                    // dropped the line's last column.  The gate reported it as
                    // "the centre tap holds the sample of the column to its
                    // right", which is exactly what it was.
                    sx <= 16'd1;
                    src_lines <= src_lines + 32'd1;
                end
                else begin
                    // ---- the demosaic, one column behind the sample ----------
                    if (dm_fire) begin
                        demos_px <= demos_px + 32'd1;
                        h_acc <= h_nacc;
                        if (h_emit) begin
                            out_px <= out_px + 32'd1;
                            ox <= ox + 16'd1;
                            if (beat_push) begin
                                if(this_drop) dropped_beats<=dropped_beats+1;
                            end
                            else begin
                                beat_px[ox[3:0]*24 +: 24] <= px_in;
                            end
                            last_px <= px_in;
                        end
                    end

                    if (raw_eol_p) begin
                        // **A line shorter than the geometry claims is a fault
                        // and not a gap**, and it is counted rather than
                        // absorbed: `raw_eol_p` on a column below the expected
                        // one puts every pixel of that line at the wrong place
                        // in the beats.
                        if (sx + 16'd1 != W_SRC)
                            short_lines <= short_lines + 32'd1;
                        // The row's last pixel is demosaiced on this very
                        // cycle -- the lag is one *column*, not one line -- so
                        // `oy` advances after it, not before.
                        if (keep_row) oy <= oy + 16'd1;
                        // The next sample belongs to a new line, so the column
                        // restarts here and the row slots advance here.
                        sx <= 16'd0;
                        if (raw_eof_p) begin
                            sy   <= 16'd0;
                            head <= 2'd0;
                        end
                        else begin
                            sy   <= sy + 16'd1;
                            head <= head^2'd1;
                        end
                    end
                    else begin
                        sx <= sx + 16'd1;
                    end
                end

                cur_xm2 <= cur_xm1;
                cur_xm1 <= raw10_p;
            end
        end
    end

endmodule
