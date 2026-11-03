`timescale 1ns/1ps

// ============================================================================
// What the sensor actually sent, per frame, and a verdict a buffer can latch.
// ============================================================================
//
// ----------------------------------------------------------------------------
// What the anchors are, and why they are these
// ----------------------------------------------------------------------------
//
// CSI-2 sends **packets**, not a raster.  A frame is a Frame Start short packet,
// then one long packet per line, then a Frame End short packet.  There is no
// sync pulse in the pixel clock at all; the physical layer goes back to LP-11
// between packets.  So the protocol's own markers are
//
//   * **a line**   -- a long packet whose header carries its payload length.
//                     At 1920 pixels of RAW10 that is `1920 * 10 / 8` = 2400
//                     bytes, and there is no line number: order is identity.
//   * **a frame**  -- a short packet, and `vsync_vc0` is the receiver telling
//                     you one is starting.  The short packet carries the
//                     **frame number**, which is the sensor's own statement of
//                     which frame this is.
//
// `csi_rx_controller` exposes all of it: `word_count` and
// `shortpkt_data_field` are two names for the same 16-bit packet field -- the
// payload length on a long packet, the frame number on a short one -- beside
// `datatype`, `vc`, `hsync_vc0` and `vsync_vc0`.
//
// **`hsync_vc0` is a data-valid gate, not a line marker.**  That is not an
// inference: the vendor's own project for this sensor and this IP consumes it as
//
//     .i_vs (w_mipi_rx_vs),
//     .i_de (w_mipi_rx_de & w_mipi_rx_hs),          // 10_Ti60f225_sc431hai2hdmi
//
// -- the pixel is taken when it is valid *and* `hsync_vc0` is high -- and no
// vendor demo anywhere counts lines from its rising edges.  Their reference
// testbench generates VGA-style syncs (`H_SyncPulse` 96, `H_BackPorch` 48,
// `H_ActivePix` 640) with the sync pulse and the data window as separate spans,
// which is a shape in which "the interval between two rising edges is one line"
// happens to hold and is not a promise.
//
// This module used to judge the frame by that interval.  On a fixed sensor it
// held for 1079 intervals out of 1080 and split one, every frame, at the frame
// boundary -- and the split was invisible to the version that shipped because it
// skipped exactly that interval.  The anchors below are the protocol's, so the
// question does not arise.  `hsync` is still measured, extensively, and none of
// its measurements is a verdict input.
//
// ----------------------------------------------------------------------------
// The verdict
// ----------------------------------------------------------------------------
//
//   * **`frame_beats == V_ACT * BEATS_LINE`** -- the payload domain, counted
//     from `pixel_data_valid` alone.  One comparison against a constant, and it
//     is the whole geometric claim: 518400 beats is 1080 lines of 480, and any
//     other division of it is a configuration the sensor cannot be in.
//     **It is also the only place this design can see a dropped beat**, which
//     matters because the pixel-side FIFO has no overflow output at all -- see
//     the note further down.
//   * **the frame number advanced by exactly one.**  The sensor says which frame
//     this is, so a lost or duplicated frame is *reported* rather than inferred.
//     Nothing in the pixel domain can give that.
//   * **the source faults are zero**: datatype, `pixel_per_clk`, the unused top
//     half of the beat, the CSI interrupt.
//   * **`word_count == WCOUNT_EXP` at a line marker** -- every long packet is
//     one line long.  The board reads 0x960 at every marker of every frame.
//
// What is deliberately *not* a verdict input is anything derived from the
// spacing of `hsync_vc0`: `lines_short`, `lines_long`, `lines_zero`,
// `lines_exact`, the interval histogram and `lb_min`/`lb_max` are all still
// measured and published, because a capture should be able to show what the
// gate is doing, and none of them can refuse a frame.
//
// ----------------------------------------------------------------------------
// The frame event
// ----------------------------------------------------------------------------
//
// The plan asks for a completion event a buffer can latch: `P2` freezes one
// frame into DDR and only hands it to the display if that frame was good, and
// cumulative counters cannot express that because by the time a reader looks the
// counter has moved on.  So every frame boundary pulses `frame_done` for one
// cycle, and on that cycle
//
//   `frame_id`   the frame that just closed, counting from 0
//   `frame_ok`   the verdict for *that* frame
//   `frame_reason` its own reason bits
//   `fs_num`     the frame number the sensor put on it
//   `crc_frame`, `crc_beats`, `beats_last`, `lines_seen_last`
//
// are all valid together and all describe the same frame.  A bad frame still
// raises `frame_done` with `frame_ok = 0`: a frame that ended and was bad is a
// different fact from no frame having ended, and a buffer that could not tell
// them apart would hold the previous frame for ever without saying why.
//
// ----------------------------------------------------------------------------
// What is deliberately NOT here
// ----------------------------------------------------------------------------
//
// **No pixel-FIFO overflow counter.**  The plan asks for one and this interface
// does not have one: `csi_rx_controller` exposes `fifo_rd_empty[3:0]` on the
// D-PHY lane side and nothing on the pixel side, and the lane FIFO's EMPTY is
// not a pixel-FIFO level.  Inventing a counter from a signal that does not mean
// that would be the same mistake as reading `irq_sticky = 0` as "all ECC, CRC
// and FIFO errors are zero", which the plan names explicitly.  `crc_beats` is
// the honest substitute for the part of this design can see: a FIFO that
// overflowed lost beats, and a frame that lost beats has the wrong total.
//
// **No claim about `shortpkt_data_field` beyond the frame number.**  On a long
// packet it reads the same as `word_count`, because it *is* the same register;
// on a short packet it is the field, and at the frame boundary it counts frames
// -- which the board shows directly (`wc_at_vsync` and `sp_at_vsync` running
// 0x1e..0x272 across a capture, in step with `frame_id`).
//
// ----------------------------------------------------------------------------
// The geometry is a parameter, and the first board run is a calibration
// ----------------------------------------------------------------------------
//
// `H_ACT`/`V_ACT`/`PPC` set what is *expected*.  A build that expects 1920x1080
// and gets 2560x1440 reports every frame as bad, which is a correct report of a
// real disagreement -- but it is not yet a judgement about the sensor.  The
// counters are therefore absolute and separately readable, so a capture says
// what arrived even when every frame is bad.
module cam_frame_valid #(
    parameter integer H_ACT = 1920,
    parameter integer V_ACT = 1080,
    parameter integer PPC   = 4,                    // pixels per beat
    parameter [5:0]   DT_RAW10 = 6'h2B,
    // The CSI-2 long-packet word count of one `H_ACT`-pixel RAW10 line:
    // `H_ACT * 10 / 8` bytes, which is 2400 at 1920.
    parameter integer WCOUNT_EXP = (H_ACT * 10) / 8,
    // Set to 0 to drop the packet-length test out of the verdict while leaving
    // the counter that measures it.  The verdict that excludes it is
    // `frames_geom`, which is published beside `frames_good` so a capture can
    // separate "is the sensor right" from "is this field the one I think it is".
    parameter integer CHECK_WCOUNT = 1
) (
    input  wire        clk,                         // clk_pix
    input  wire        rst_n,

    // ---- the receiver's pixel port -------------------------------------
    input  wire        pix_valid,
    input  wire [63:0] pix_data,
    input  wire [3:0]  pix_per_clk,
    input  wire [5:0]  datatype,
    input  wire        hsync,                       // vc0 -- a data gate
    input  wire        vsync,                       // vc0 -- the frame marker
    input  wire        irq,

    // ---- the packet field, published verbatim ---------------------------
    input  wire [15:0] word_count,
    input  wire [15:0] shortpkt,
    input  wire [1:0]  vc,

    // ---- the frame verdict --------------------------------------------
    output reg  [31:0] frames_good,   // frames that passed every condition
    output reg  [31:0] frames_bad,    // ... and frames that did not
    output reg  [31:0] frames_geom,   // passed everything except the packet-length test
    output reg  [7:0]  bad_reason,    // OR of the reasons over every bad frame

    // ---- the frame event, one cycle, all of it the same frame ----------
    output reg  [31:0] frame_id,      // counting from 0 at the first boundary
    output reg         frame_done,
    output reg         frame_ok,
    output reg  [7:0]  frame_reason,  // the closing frame's own reason bits
    output reg  [15:0] fs_num,        // the sensor's frame number for it
    output reg  [31:0] fs_bad,        // frames whose number did not advance by 1

    // ---- what arrived, whatever the expectation ------------------------
    output reg  [31:0] beats_last,       // beats in the last complete frame
    output reg  [15:0] lines_seen_last,  // line markers in the last complete frame
    output reg  [31:0] line_beats,       // beats since the last line marker, live

    // ---- faults that are not geometry ---------------------------------
    output reg  [31:0] dt_bad,        // beats whose datatype was not RAW10
    output reg  [31:0] ppc_bad,       // beats whose pixel_per_clk was not PPC
    output reg  [31:0] hi_bad,        // beats with pix_data[63:40] non-zero
    output reg  [31:0] irq_count,
    output reg         irq_sticky,

    // ---- the frame fingerprint ----------------------------------------
    output reg  [31:0] crc_frame,     // CRC32 of the last complete frame, final-xored
    output reg  [31:0] crc_frames,    // how many frames it covers
    output reg  [31:0] crc_beats,     // beats folded into that frame

    // ---- the packet field and the packet-length test --------------------
    output reg  [15:0] obs_word_count,
    output reg  [15:0] obs_shortpkt,
    output reg  [1:0]  obs_vc,
    output reg  [31:0] wc_bad,        // line markers whose packet was not one line

    // ---- the line-marker observations, none of which is a verdict ------
    //
    // **These are published and they are not judged.**  `hsync_vc0` is a
    // data-valid gate, so the spacing between its rising edges is a property of
    // the receiver, not a promise about the sensor; the board shows one split
    // interval per frame for that reason.  A capture should be able to see it,
    // and no frame is refused for it.
    //
    // The histogram exists because the first board capture's two readings could
    // not both be true and no arithmetic was going to say which was being
    // misread -- 29,347 intervals shorter than a line and none longer, over a
    // run whose frame totals were exact.  Twelve buckets, eight bits each
    // saturating, packed three to a word.  Bucket 0 is the frame boundary and
    // bucket 10 is a line.
    output reg  [31:0] lines_short,   // intervals shorter than a line but not empty
    output reg  [31:0] lines_long,    // ... and longer
    output reg  [31:0] lines_missing, // expected markers that never arrived
    output reg  [31:0] lines_extra,   // markers beyond the expected count
    output reg  [31:0] lines_zero,    // intervals of no beats at all
    output reg  [31:0] hist_lo,       // buckets 0..3
    output reg  [31:0] hist_mid,      // buckets 4..7
    output reg  [31:0] hist_hi,       // buckets 8..11
    output reg  [15:0] lb_min,        // shortest interval in the last frame
    output reg  [15:0] lb_max,        // longest, saturated at 65535
    output reg  [31:0] lines_exact,   // intervals of exactly one line, absolute
    // A line counted every BEATS_LINE beats, which does not use the marker at
    // all.  Absolute, like every other counter here, and **sixteen bits**, so it
    // wraps every 65536 lines -- about sixty frames.  It is a shape check, not a
    // running total; `crc_beats` is the total.
    output reg  [15:0] lines_by_beats,

    // ---- the packet field, latched at a line marker ---------------------
    output reg  [15:0] wc_marker, sp_marker,
    output reg  [15:0] wc_vsync,  sp_vsync,
    output reg  [15:0] wc_min,    wc_max,
    output reg  [31:0] sp_marks       // markers at which the packet field was non-zero
);

    localparam integer BEATS_LINE = H_ACT / PPC;    // 480
    // **Verilog-2001 has no bit-select on an integer parameter**, so every width
    // a comparison needs is its own constant.  `d64_video_row_ddr.v` carries the
    // same note for the same reason, and a `V_ACT[15:0]` here compiles into an
    // implicit-width mess rather than the comparison it looks like.
    localparam [15:0] LINES_EXP = V_ACT;
    localparam [15:0] BEATS_EXP = BEATS_LINE;
    localparam [3:0]  PPC_B     = PPC;
    localparam [15:0] WC_EXP    = WCOUNT_EXP;
    // The payload domain's whole-frame expectation.  Written as a constant
    // rather than as a product in the comparison so the width is explicit:
    // 518400 needs 20 bits and a 16-bit product would wrap silently.
    localparam [31:0] FRAME_EXP = V_ACT * BEATS_LINE;

    // Bucket edges.  See the port comment for why they are these.
    //
    // **`== line` and `> line` are tested first, and the first version did not.**
    // It ran the powers-of-two ladder up to 255 and only then asked whether the
    // interval was a line -- which is right at the build's 480 beats a line and
    // silently wrong at any geometry where a line is shorter than 256.  The
    // bench's 8-beat lines put every interval in the `<= 255` bucket, so the
    // `= line` bucket read zero and the bench caught it; a build at a smaller
    // resolution would have read the same way with nobody the wiser.
    function [3:0] bucket;
        input [31:0] v;
        begin
            if      (v == {16'd0, BEATS_EXP}) bucket = 4'd10;
            else if (v >  {16'd0, BEATS_EXP}) bucket = 4'd11;
            else if (v == 0)          bucket = 4'd0;
            else if (v <= 1)          bucket = 4'd1;
            else if (v <= 3)          bucket = 4'd2;
            else if (v <= 7)          bucket = 4'd3;
            else if (v <= 15)         bucket = 4'd4;
            else if (v <= 31)         bucket = 4'd5;
            else if (v <= 63)         bucket = 4'd6;
            else if (v <= 127)        bucket = 4'd7;
            else if (v <= 255)        bucket = 4'd8;
            else                      bucket = 4'd9;
        end
    endfunction

    // The verdict's reason bits, and every one of them is a protocol condition.
    localparam [7:0] R_STRANGE = 8'h01;   // datatype, pixel_per_clk, top half
    localparam [7:0] R_IRQ     = 8'h02;   // the CSI interrupt
    localparam [7:0] R_TOTAL   = 8'h04;   // the payload was not one whole frame
    localparam [7:0] R_FS      = 8'h08;   // the frame number did not advance by one
    localparam [7:0] R_WC      = 8'h10;   // a line marker's packet was not one line

    integer bi;

    // ---- the CRC: one 64-bit beat per cycle ---------------------------
    //
    // Generated by `tools/d64_chunk_crc32_gen.py --width 64` and verified there
    // against `zlib.crc32`.  The state is raw, so it starts at 32'hFFFFFFFF and
    // the result is complemented when a frame is latched -- which is exactly what
    // `zlib.crc32` does, and what makes a board capture comparable with a PC.
    wire [31:0] crc_o;
    reg  [31:0] crc;

    cam_crc32_64 u_crc (.crc_i(crc), .pkt(pix_data), .crc_o(crc_o));

    // ---- edge detection, and the interval measurement ------------------
    reg hs_q, vs_q;
    wire hs_rise = hsync & ~hs_q;
    wire vs_rise = vsync & ~vs_q;

    reg [15:0] lines_seen;
    reg [31:0] frame_beats;
    reg [7:0]  f_reason;            // the source faults seen in this frame
    reg [31:0] wc_frame_bad;        // packet-length failures in this frame
    reg [31:0] frame_seq;
    reg [15:0] fs_prev;
    reg        fs_seen;             // a previous frame number exists

    reg [7:0]  hist [0:11];
    reg [15:0] lb_min_r, lb_max_r;
    reg [15:0] wc_min_r, wc_max_r;
    reg [11:0] beat_ph;             // beats since the last counted line
    reg [3:0]  bk;

    // A beat count wider than 16 bits is published saturated, because the
    // alternative is a wrapped small number that reads as a short interval --
    // the one reading that must not be possible.
    function [15:0] sat16;
        input [31:0] v;
        begin sat16 = (v > 32'd65535) ? 16'hFFFF : v[15:0]; end
    endfunction

    // A beat is "strange" in three separable ways and they get three counters,
    // because a datatype that changed and a bus that grew a non-zero top half
    // are different faults with different next steps.
    wire beat_dt_bad  = pix_valid && (datatype    != DT_RAW10);
    wire beat_ppc_bad = pix_valid && (pix_per_clk != PPC_B);
    wire beat_hi_bad  = pix_valid && (pix_data[63:40]);

    // ==================================================================
    // The interval and the frame, settled before the clock edge
    // ==================================================================
    //
    // **The interval is measured, not judged.**  Every closing edge closes
    // exactly one interval -- a line marker, the frame marker, or both on one
    // cycle -- and a beat on that cycle belongs to the interval it closes, so
    // `iv` counts it once and the next interval starts at zero.  That is the
    // rule the shipped module never settled, and the asymmetry is where its last
    // board bug came from: at a line marker it seeded the next interval with the
    // coincident beat and measured the closed one from the pre-edge value, so
    // the two cancelled for every line but the first -- and the first was
    // skipped, which is why every interval read 480 while the frame's own first
    // line did not.
    wire        iv_edge   = hs_rise | vs_rise;
    wire [31:0] iv        = line_beats + (pix_valid ? 32'd1 : 0);
    wire [15:0] marks_iv  = lines_seen  + (hs_rise   ? 16'd1 : 0);
    wire [31:0] beats_iv  = frame_beats + (pix_valid ? 32'd1 : 0);

    // **The interval that closes on this cycle is folded into the frame's
    // extremes too.**  Latching `lb_min_r`/`lb_max_r` alone at the frame marker
    // reads them one interval stale, and which interval is missed depends on
    // where the boundary interval falls -- it is the frame's *first* under a
    // start-of-line marker and its *last* under an end-of-line one, so the same
    // code reads 0 in one mode and 8 in the other.  Folding it here makes the
    // reading the same in all three.
    wire [15:0] lb_min_iv = (iv_edge && (iv < {16'd0, lb_min_r})) ? iv[15:0] : lb_min_r;
    wire [15:0] lb_max_iv = (iv_edge && (iv > {16'd0, lb_max_r})) ? sat16(iv) : lb_max_r;

    wire iv_zero  = iv_edge && (iv == 32'd0);
    wire iv_short = iv_edge && (iv != 32'd0) && (iv < {16'd0, BEATS_EXP});
    wire iv_long  = iv_edge && (iv > {16'd0, BEATS_EXP});

    // **The packet-length test is at every line marker**, not only at one that
    // closed a whole line.  `hsync_vc0` gates the data and every one of its
    // markers presented a 2400-byte packet on the board (`wc_min` and `wc_max`
    // are both 0x960 over a frame), so there is nothing to exclude -- and the
    // exclusion the earlier version made was one more interval it declined to
    // look at.  At the frame boundary the same register holds the frame number
    // instead, which is why the test is on `hs_rise` and not on `vs_rise`.
    // **Not on a cycle that is also the frame marker.**  In CSI-2 a short packet
    // occupies the same 16-bit field a long packet uses for its length, so at the
    // frame boundary the register holds the frame number -- the board reads
    // 0x01b..0x272 there against 0x960 at every line marker.  A build with the
    // two markers coincident would otherwise fail every frame on a field that is
    // not a length.
    wire wc_check = CHECK_WCOUNT && hs_rise && !vs_rise;
    wire wc_wrong = wc_check && (word_count != WC_EXP);

    // The sensor's own frame number, and whether this frame followed the last.
    // **A frame with no predecessor is not a mismatch** -- that is the first
    // frame after a reset, and it has nothing to be consecutive with.
    wire [15:0] fs_next  = fs_prev + 16'd1;
    wire        fs_ok    = !fs_seen || (shortpkt == fs_next);

    // The verdict, as wires over the current registers.  `f_reason` and
    // `lines_seen` are both updated with `<=` below, so a test written after
    // those assignments in the same block would read the pre-update value and
    // the verdict would depend on statement order.
    wire [15:0] lines_closed  = marks_iv;
    wire [31:0] beats_closed  = beats_iv;
    wire [7:0]  reason_closed = f_reason
                              | (fs_ok ? 8'd0 : R_FS)
                              | ((beats_closed != FRAME_EXP) ? R_TOTAL : 8'd0);
    wire        wc_frame_clean = (wc_frame_bad == 32'd0) && !wc_wrong;

    // Geometry first, so it can be read on its own.  `frames_geom` differing
    // from `frames_good` is the packet-length test and nothing else, which is
    // what makes a capture able to answer "is the sensor right" separately from
    // "is this field the one I think it is".
    wire geom_ok = (reason_closed == 8'd0);
    wire ok_closed = geom_ok && wc_frame_clean;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_q <= 1'b0; vs_q <= 1'b0;
            crc <= 32'hFFFFFFFF;
            lines_seen <= 16'd0; frame_beats <= 32'd0;
            f_reason <= 8'd0; wc_frame_bad <= 32'd0; frame_seq <= 32'd0;
            fs_prev <= 16'd0; fs_seen <= 1'b0;
            line_beats <= 32'd0;
            frames_good <= 32'd0; frames_bad <= 32'd0; frames_geom <= 32'd0;
            lines_short <= 32'd0; lines_long <= 32'd0;
            lines_missing <= 32'd0; lines_extra <= 32'd0;
            bad_reason <= 8'd0;
            frame_id <= 32'd0; frame_done <= 1'b0; frame_ok <= 1'b0;
            frame_reason <= 8'd0; fs_num <= 16'd0; fs_bad <= 32'd0;
            beats_last <= 32'd0; lines_seen_last <= 16'd0;
            dt_bad <= 32'd0; ppc_bad <= 32'd0; hi_bad <= 32'd0;
            irq_count <= 32'd0; irq_sticky <= 1'b0;
            crc_frame <= 32'd0; crc_frames <= 32'd0; crc_beats <= 32'd0;
            for (bi = 0; bi < 12; bi = bi + 1) hist[bi] <= 8'd0;
            lb_min_r <= 16'hFFFF; lb_max_r <= 16'd0; beat_ph <= 12'd0; bk <= 4'd0;
            lines_exact <= 32'd0; lines_by_beats <= 16'd0; lines_zero <= 32'd0;
            wc_marker <= 16'd0; sp_marker <= 16'd0;
            wc_vsync <= 16'd0; sp_vsync <= 16'd0;
            wc_min <= 16'hFFFF; wc_max <= 16'd0;
            wc_min_r <= 16'hFFFF; wc_max_r <= 16'd0; sp_marks <= 32'd0;
            obs_word_count <= 16'd0; obs_shortpkt <= 16'd0; obs_vc <= 2'd0;
            wc_bad <= 32'd0;
        end
        else begin
            hs_q <= hsync;
            vs_q <= vsync;

            // The event pulse is one cycle.  Clearing it here rather than in the
            // frame branch keeps the branch about the frame and this line about
            // the pulse's width.
            frame_done <= 1'b0;

            // Published live rather than latched: a reader wants to see these
            // move, and a latch would need a trigger whose meaning nobody has
            // established yet.
            obs_word_count <= word_count;
            obs_shortpkt   <= shortpkt;
            obs_vc         <= vc;

            if (irq) begin
                irq_count  <= irq_count + 32'd1;
                irq_sticky <= 1'b1;
                f_reason   <= f_reason | R_IRQ;
            end

            // ---- the pixels -------------------------------------------
            if (pix_valid) begin
                crc         <= crc_o;
                frame_beats <= frame_beats + 32'd1;
                line_beats  <= line_beats + 32'd1;
                // **A line counted by beats, not by the marker.**  This is the
                // counter that says whether the payload divides into lines
                // without reference to `hsync_vc0` at all.
                if (beat_ph == {4'd0, BEATS_EXP} - 12'd1) begin
                    beat_ph        <= 12'd0;
                    lines_by_beats <= lines_by_beats + 16'd1;
                end
                else beat_ph <= beat_ph + 12'd1;
                if (beat_dt_bad)  begin dt_bad  <= dt_bad  + 32'd1;
                                         f_reason <= f_reason | R_STRANGE; end
                if (beat_ppc_bad) begin ppc_bad <= ppc_bad + 32'd1;
                                         f_reason <= f_reason | R_STRANGE; end
                if (beat_hi_bad)  begin hi_bad  <= hi_bad  + 32'd1;
                                         f_reason <= f_reason | R_STRANGE; end
            end

            // ---- an interval ended -------------------------------------
            //
            // **Measured and published; no branch here touches the verdict.**
            // The counters exist so that a capture can show what `hsync_vc0` is
            // doing -- and on this receiver what it does is split one interval
            // per frame -- without any of it refusing a frame.
            //
            // `line_beats <= 32'd0` has to be the *last* assignment to
            // `line_beats` in this block, which it is, because this branch is
            // after the pixel branch.
            if (iv_edge) begin
                bk = bucket(iv);
                if (hist[bk] != 8'hFF) hist[bk] <= hist[bk] + 8'd1;
                if (iv < {16'd0, lb_min_r}) lb_min_r <= iv[15:0];
                if (iv > {16'd0, lb_max_r}) lb_max_r <= sat16(iv);
                if (iv == {16'd0, BEATS_EXP}) lines_exact <= lines_exact + 32'd1;
                if (iv_zero)  lines_zero  <= lines_zero  + 32'd1;
                if (iv_short) lines_short <= lines_short + 32'd1;
                else if (iv_long) lines_long <= lines_long + 32'd1;
                // The packet field, latched here so page 5 can pair it with the
                // interval it belongs to.
                wc_marker <= word_count;
                sp_marker <= shortpkt;
                if (shortpkt != 16'd0) sp_marks <= sp_marks + 32'd1;
                if (word_count < wc_min_r) wc_min_r <= word_count;
                if (word_count > wc_max_r) wc_max_r <= word_count;
                if (wc_wrong) begin
                    wc_bad       <= wc_bad + 32'd1;
                    wc_frame_bad <= wc_frame_bad + 32'd1;
                end

                // The next interval starts empty: a beat on this cycle has just
                // been counted into `iv`.
                line_beats <= 32'd0;
                if (hs_rise) lines_seen <= lines_seen + 16'd1;
            end

            // ---- a frame ended -----------------------------------------
            //
            // **After the interval branch, so a shared cycle is one interval and
            // one frame, not two.**  `marks_iv` already counted the line marker
            // this cycle may carry.
            if (vs_rise) begin
                if (lines_closed < LINES_EXP)
                    lines_missing <= lines_missing + {16'd0, LINES_EXP} - {16'd0, lines_closed};
                else if (lines_closed > LINES_EXP)
                    lines_extra   <= lines_extra + {16'd0, lines_closed} - {16'd0, LINES_EXP};

                if (geom_ok)   frames_geom <= frames_geom + 32'd1;
                if (ok_closed) frames_good <= frames_good + 32'd1;
                else begin
                    frames_bad  <= frames_bad + 32'd1;
                    // **Both bytes' worth of reason, in one place.**  The packet
                    // length is a verdict input but it is not part of
                    // `reason_closed` -- that is computed as a wire before this
                    // cycle's failures are counted -- so it has to be ORed in
                    // here or a frame refused only by it would raise an event
                    // with no reason bit set, which is the one reading a reader
                    // must never get.
                    bad_reason  <= bad_reason | reason_closed
                                 | (wc_frame_clean ? 8'd0 : R_WC);
                end

                // The sensor's own frame number: latched, compared with the one
                // before it, and counted when it does not follow.
                if (!fs_ok) fs_bad <= fs_bad + 32'd1;
                fs_prev <= shortpkt;
                fs_seen <= 1'b1;

                // **The event.**  All of it describes the frame that is ending,
                // and all of it is valid on the same cycle as `frame_done`.
                frame_id      <= frame_seq;
                frame_seq     <= frame_seq + 32'd1;
                frame_done    <= 1'b1;
                frame_ok      <= ok_closed;
                frame_reason  <= reason_closed | (wc_frame_clean ? 8'd0 : R_WC);
                fs_num        <= shortpkt;

                // Complemented here rather than in the engine, so the state in
                // `crc` is the raw running value and a board capture is
                // comparable with `zlib.crc32` directly -- the same convention
                // `d64_crc32_128` uses and states.
                //
                // **A beat on this cycle is folded into the frame that is
                // closing**, which is the same rule as the interval above and
                // the reason this is not simply `~crc`: otherwise `crc_beats`
                // would claim a beat the CRC had not covered, and the whole point
                // of publishing `crc_beats` beside the CRC is that the two cannot
                // disagree.
                crc_frame       <= pix_valid ? ~crc_o : ~crc;
                crc_frames      <= crc_frames + 32'd1;
                crc_beats       <= beats_closed;
                beats_last      <= beats_closed;
                lines_seen_last <= lines_closed;

                lb_min      <= lb_min_iv;
                lb_max      <= lb_max_iv;
                wc_vsync    <= word_count;
                sp_vsync    <= shortpkt;
                wc_min      <= wc_min_r;
                wc_max      <= wc_max_r;
                wc_min_r    <= 16'hFFFF;
                wc_max_r    <= 16'd0;
                lb_min_r    <= 16'hFFFF;
                lb_max_r    <= 16'd0;
                beat_ph     <= 12'd0;

                // The next frame starts empty, including on a cycle where a beat
                // arrived: that beat closed the frame above.
                crc          <= 32'hFFFFFFFF;
                frame_beats  <= 32'd0;
                lines_seen   <= 16'd0;
                line_beats   <= 32'd0;
                f_reason     <= 8'd0;
                wc_frame_bad <= 32'd0;
            end
        end
    end

    // Packed on every cycle rather than latched: a reader wants to watch these
    // move, and the snapshot handshake is what decides *when* they are read.
    always @* begin
        hist_lo  = {hist[3], hist[2], hist[1], hist[0]};
        hist_mid = {hist[7], hist[6], hist[5], hist[4]};
        hist_hi  = {hist[11], hist[10], hist[9], hist[8]};
    end

endmodule
