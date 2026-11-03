`timescale 1ns/1ps

// ============================================================================
// D64 video row loader -- the scaler's loader port, answered from DDR.
// ============================================================================
//
// `d64_video_scaler` streams its 256x256 source through two 24x256 row banks
// (4 RAM10) and asks for a row with `load_req`/`load_row`; it takes the pixels
// back on `load_we`/`load_wi`/`load_wd` and raises `row_ready` as a level.  This
// module is that producer, with the picture coming from DDR rather than from a
// computed pattern, and it is a drop-in for `d64_video_test_src` at the same
// pins -- which is what the loader port was shaped for in the first place.
//
// A row is 256 pixels x 4 bytes = 1 KiB, which is 64 AXI beats of 128 bits.
//
// ----------------------------------------------------------------------------
// The descriptor, and how it crosses
// ----------------------------------------------------------------------------
//
// The scaler runs on the pixel clock (`clk_pe`, 74.375 MHz on this board) and
// the AXI port runs on `clk`.  What crosses *forward* is a descriptor
//
//     { bank, frame_id[1:0], epoch[8:0], row[7:0], src_base[31:0] }
//
// and what crosses *back* is the row of pixels plus the same tag, through an
// asynchronous FIFO.
//
// `d64_encoder_trunk_board_top.v:493` carries `x_req_row`/`src_buf`/`dst_buf`
// beside a one-bit `x_req_tog`, the receiver edge-detects the toggle
// (`d64_trunk_row_ddr.v:266`) and samples the payload on the synchronised edge.
// **That shape is copied here, and it is the right shape**, because the payload
// is stable for a whole display line -- thousands of `clk` cycles -- so bit
// skew across the payload is not a race, it is noise a synchroniser has already
// finished with.  What is *added* is the thing the trunk's copy does not have:
//
//   * **the payload cannot move until the far side has acknowledged it.**
//     `req_tog` is not released until `ack_tog` has come back through the
//     display domain's own two flops, and only then may a new request latch a
//     new payload.  The trunk's sender has no such gate: `x_req_row` follows
//     `d64_trunk_blocks`' block index and the receiver's sampling of it is a
//     race the board is trusted to lose gracefully.  Gating it costs one
//     `d_busy` flop and removes the race entirely.
//
//   * **the payload is constrained.**  `board/ddr3/d64_encoder_trunk_board.sdc`
//     has no `set_max_delay` on the `x_req_*` datapath, so static timing has
//     nothing to say about it while the two clocks are declared asynchronous --
//     a bundled-data crossing left unconstrained is a crossing whose skew is
//     whatever the router felt like.  `docs/H3_HDMI_ROW_LOADER.md` carries the
//     two lines that belong in `hdmi_tx.sdc` if this is ever integrated.
//
// So: same scheme, plus a full handshake and a constraint.  The handshake makes
// the payload's stability a property of the design rather than of the sender's
// pace, and the constraint makes the skew bounded.
//
// ----------------------------------------------------------------------------
// One burst at a time, and a FIFO the size of the burst
// ----------------------------------------------------------------------------
//
// The read staging FIFO is **exactly one burst deep**: 64 words of 128 bits
// plus a 12-bit tag, so 140 bits.  That is 7 RAM10 -- the same 7 a 128-bit
// word costs, because 6 columns of RAM10 is 120 bits and 128 does not fit; the
// tag rides in the slack.  A burst is started only into an empty FIFO
// (`rf_wr_level == 0`) and only one burst is ever in flight, so **no beat can
// be dropped and no `rready` backpressure is needed.**
//
// That is the deliberate difference from `d64_trunk_row_ddr`: there the read
// FIFO is 32 beats against a 128-beat burst, `rf_wr_en` does not look at
// `rf_wr_full`, and holding `rready` low while full deadlocks it in simulation
// (`d64_trunk_row_ddr.v:300-334`), so it counts overflows instead of preventing
// them.  Sizing the storage for the burst removes the question.  `rready` is
// still held low on `rf_wr_full` as a second line of defence, and it cannot
// deadlock here because the drain is a different clock domain that keeps
// running -- but it is not expected to ever assert.
//
// ----------------------------------------------------------------------------
// Epochs, late returns, and the stale word
// ----------------------------------------------------------------------------
//
// `live_epoch` counts display frames.  Every descriptor carries the epoch it was
// made under, and the tag comes back through the FIFO with the pixels, so the
// display side can tell `the row I asked for` from `the row I still want`.
//
// A word is handled by three rules, and they are the whole correctness
// argument:
//
//   tag does not match      a leftover from an abandoned burst.  Discarded and
//                           skipped; the fetch keeps going.  Consecutive
//                           descriptors always differ in `bank` -- it toggles on
//                           every fetch -- so a leftover can never match.
//   tag matches, frame gone the row belongs to a frame the display has left.
//                           The fetch is **superseded**: the rest of the row is
//                           counted stale and the same row is re-requested under
//                           the current epoch, from the current base.  Nothing
//                           from the old frame is written into the new one.
//   tag matches, frame live  the pixel.
//
// `row_ready` rises only after a whole row has landed **and** its epoch and
// frame_id still match, so a row is never presented half old and half new.
//
// ----------------------------------------------------------------------------
// underrun
// ----------------------------------------------------------------------------
//
// A fetch that has not delivered its row within `TIMEOUT` display clocks is
// given up on: the target bank is filled with 256 black pixels, `underrun`
// counts the event, and `row_ready` rises so the scaler is released.  The
// display therefore always emits a defined line and the raster never stops --
// and because `TIMEOUT + SRC_W < one line` (1024 + 256 < 1650) the row is back
// before the scaler samples `row_ready` at the next `line_0`, so the scaler's
// own sticky `starved` never asserts.  A blocked supply therefore shows as a
// black line, an unbroken raster, and a counter that moved.
//
// The row that was abandoned is left in the FIFO and skipped by tag when it
// finally arrives, so the failure does not have to be waited out before the
// next row can be fetched.
//
// ----------------------------------------------------------------------------
// A row the memory got wrong, which is not the same as a row that was late
// ----------------------------------------------------------------------------
//
// `underrun` says the supply did not arrive. It says nothing about whether what
// arrived was *right*, and until this was added nothing did: a beat with a bad
// `RRESP` was counted in `axi_errors` and written into the row bank anyway, an
// `RLAST` before the sixty-fourth beat was not noticed at all, and a final beat
// with no `RLAST` was counted and published. So a memory returning errors
// produced `axi_errors != 0` on the telemetry and a perfectly plausible picture
// on the screen, which is the worst of the two available failures.
//
// A word now carries a verdict bit, set by the AXI side and travelling in the
// FIFO beside the pixels. The bit is the OR of three conditions, and each is a
// separate way the row can be unearnable:
//
//   bad `RRESP`     the slave answered, and the answer was an error. It does
//                   **not** end the burst -- AXI allows SLVERR on one beat and
//                   more beats after it -- so the condemnation has to persist
//                   across beats, which is what `burst_bad` is for.
//   early `RLAST`   the slave ended the burst before the sixty-fourth beat. No
//                   sixty-fourth beat will come, so no word can carry the news
//                   unless the beat it *did* send carries it. It does.
//   missing `RLAST` the sixty-fourth beat arrived without an end marker. The
//                   word being written on that cycle is the last one, and it
//                   carries the verdict for its own arrival.
//
// The display blanks the line the moment it sees an accepted word with the bit
// set -- not at the end of the row -- and counts it in `bad_rows`, separately
// from `underrun`, because "the memory answered and was wrong" and "the memory
// did not answer" are different faults and a number that merged them could not
// say which happened. `first_bad_row` names the first one after reset, so a run
// with many failures can say where they started rather than only how many.
//
// The bit costs one RAM10 block on the staging FIFO: 140 bits was exactly seven
// (7 x 20) and 141 needs eight. That is the price of the verdict being unable to
// separate from its word, and it was preferred to a synchronised level because
// the level would land within about one `clk_pe` cycle of the publish decision
// and the race would then be settled by the clock ratio.
//
// The early-`RLAST` case has a second half that is not about the picture at all:
// the burst used to end only at `sent == LAST_BEAT`, so a short burst left
// `burst_live` set for ever, `a_idle` never returned, and every later fetch took
// the `S_WAIT` path into a black line. The display showed black for the rest of
// the run and nothing in it could tell why, because the display cannot see the
// bus. The master now believes the `RLAST` it is given.
//
// ----------------------------------------------------------------------------
// The H3 gap in d64_video_scaler.v:49-66, and whether this depends on it
// ----------------------------------------------------------------------------
//
// **It does not.**  The scaler re-requests on every window line including the
// `stepy == 0` lines, so rows are asked for 720 times a frame and only 256 of
// those are a new source row.  A one-pass tail cannot answer that because it
// produces each row once; DDR can, because a repeated row is just another
// 64-beat burst.  So H1's loader answers whatever it is asked and the picture
// is right either way -- what H3 buys here is **bandwidth, not correctness**:
// 720 rows/frame is 44.2 MB/s and 256 is 15.7 MB/s.  The per-row service
// latency the acceptance cares about is unchanged by H3; only the count is.
//
// ----------------------------------------------------------------------------
// What the top would need, and the I3 question
// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
// The 4 KiB rule, which this loader did not have (F3 6.18)
// ----------------------------------------------------------------------------
//
// A row is one burst of 64 beats of 128 bits: 1024 bytes at
// `src_base + (row << 10)`.  **Nothing here looked at where that burst lands
// inside its 4 KiB page.**  `board/ddr3/d64_axi_read_burst.v` (F08, F3 6.11)
// refuses such a request and `d64_burst_prefetch.v` inherits the rule; this
// reader did not, and AXI's own rule is that an INCR burst must not cross a 4
// KiB boundary -- 4 KiB is the smallest region a slave is required to own as a
// unit.  An interconnect may answer a crossing burst by splitting it, by
// returning an error, or by **answering it**, which puts bytes from the wrong
// page in a row whose transfer looks correct from end to end.  The third is the
// one that costs a picture and nothing else.
//
// **`src_base` is a port at runtime**, so this is not a property of a constant
// this file could check once.  With rows on a 1024-byte stride the page offset
// of a fetch is `(src_base[11:0] + row*0x400) & 0xFFF`, so a base of
// `0x0000_0100` puts one row in four across the boundary and a base of
// `0x0000_0C00` puts none -- the rule has to be evaluated per fetch, which is
// what it now is.
//
//     ar_byte[11:0] + ROW_BYTES  >  4096      ->  refused
//
// `ar_byte` is the address the burst would be issued at, the same expression the
// AXI block below forms; `ROW_BYTES` is the whole burst.  The comparison is `>`
// and not `>=`: a burst whose last byte is byte 4095 of its page is legal, and
// `tools/run_row_page_guard.sh`'s arm 2 is exactly that case, accepted and
// correct with `page_refusals` still zero, while arm 1 is the same page one row
// later and refused.
//
// **What a refusal does.**  It happens in the display domain, where the address
// is known one cycle before the descriptor crosses, and that placement is the
// whole of the design: nothing reaches the bus (no `ARVALID` is raised), the
// row is **blanked** -- 256 black pixels and `row_ready`, exactly as an underrun
// is -- and the event is counted in `page_refusals` and **nowhere else**.  A
// refusal delivered after the descriptor had crossed could only end as the
// `S_FETCH` timeout, and a timeout is counted as `underrun`: a number that said
// the supply was slow about a fetch that was never asked for.  The module's own
// header argues at length that `underrun` and `bad_rows` are separate faults;
// "we did not ask, because the address was illegal" is a third, and it has a
// third counter.
//
// ----------------------------------------------------------------------------
// Integration is `hdmi_top.v` replacing the `u_src` instance with this module
// and bringing the AXI port out of the peri; that also means a second clock at
// the top and a `set_clock_groups -async` between it and the pixel clock.  Not
// done here on purpose -- this is the design and its model, not the bitstream.
//
// The I3 question: `hdmi_tx.sdc` was generated at timing model **C4** ("Device:
// Ti60F225 / Timing Model: C4 (final)") and the part on the board is
// **Ti60F225I3**, grade 3.  C4 is a commercial-speed model, so every path the
// vendor DVI stage and the PLL close at C4 has an unknown margin at I3.  To
// re-verify them under I3 the change is in the build, not in the RTL: set the
// project's timing model to `I3` (project XML / Interface Designer regenerate)
// so `outflow/hdmi_tx.pt.sdc` is emitted against I3 and the vendor primitives
// are re-timed; the pixel clock is declared in that same generated file, and
// the only number in `hdmi_top.v` that could move as a result is the PLL
// setting -- 74.375 MHz is a peri-declared output and is not a top-level
// parameter.  Nothing in the RTL above changes.
// ============================================================================

module d64_video_row_ddr #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter integer SRC_W    = 256,               // pixels in a source row
    parameter integer PX_BYTES = 4,                 // RGBX
    parameter integer AW       = 6,                 // 2^AW = 64 words = one burst
    parameter integer TIMEOUT  = 1024,              // clk_pe cycles before a row is written off
    parameter [31:0]  SRC_BASE = 32'h0028_0000,
    // F3 6.18.  Verification only, folded away at 0.  At 1 the span test is
    // removed and the illegal burst is issued -- which is what this module did
    // before the rule existed -- so the arm must fail **on data** and not on a
    // counter.  `tools/run_row_page_guard.sh` requires it.
    parameter integer SAB_NO_PAGE_GUARD = 0
) (
    // ---- display domain (clk_pe): the scaler's loader port ----
    input  wire         clk_pe,
    input  wire         rst_pe_n,
    input  wire         load_req,
    input  wire [7:0]   load_row,
    output reg          row_ready,     // level: the requested row is whole and current
    output reg          row_good,      // whole/current successful fetch; blank rows are not good
    output reg          load_we,
    output reg  [7:0]   load_wi,
    output reg  [23:0]  load_wd,

    // ---- display domain: which source frame ----
    input  wire [1:0]   frame_id,      // the caller's current source buffer
    input  wire [31:0]  src_base,      // ... and its DDR base
    input  wire         frame_tick,    // one cycle at the display frame's first pixel

    // ---- AXI domain (clk_axi): a read master, one burst at a time ----
    input  wire         clk_axi,
    input  wire         arst_axi_n,
    output reg          axi_arvalid,
    input  wire         axi_arready,
    output reg  [31:0]  axi_araddr,
    output wire [7:0]   axi_arlen,
    output wire [2:0]   axi_arsize,
    output wire [1:0]   axi_arburst,
    input  wire         axi_rvalid,
    output reg          axi_rready,
    input  wire [127:0] axi_rdata,
    input  wire         axi_rlast,
    input  wire [1:0]   axi_rresp,

    // ---- evidence ----
    output reg  [31:0]  rows_loaded,   // rows that landed whole and current
    output reg  [31:0]  stale_beats,   // 128-bit words discarded by tag or by epoch
    output reg  [31:0]  underrun,      // rows written off because the supply was slow
    output reg  [31:0]  axi_errors,    // bad RRESP, a missing RLAST, or a short burst
    // **A blanked row and a counted error are not the same claim**, and the two
    // counters are separate because of it: `underrun` says the supply did not
    // arrive in time and `bad_rows` says it arrived and was wrong.  A number
    // that merged them could not tell a memory fault from a scheduling one.
    output reg  [31:0]  bad_rows,      // rows blanked because a word carried the verdict
    output reg  [7:0]   first_bad_row, // the row of the first one, and it does not move
    // F3 6.18.  How many fetches were refused because their burst would have
    // crossed a 4 KiB boundary.  **A refusal is a third fault and it gets its
    // own number**: counting it in `underrun` would claim the supply was slow
    // and counting it in `bad_rows` would claim the memory answered and was
    // wrong.  Neither happened, and a merged number could not say so.
    output reg  [31:0]  page_refusals
);

    localparam integer ROW_BYTES   = SRC_W * PX_BYTES;      // 1024
    localparam integer BEATS       = ROW_BYTES / 16;        // 64
    localparam integer PX_PER_BEAT = 16 / PX_BYTES;         // 4
    localparam integer RSH         = $clog2(ROW_BYTES);     // 10
    localparam integer TAGW        = 12;                    // {bank, fid[1:0], epoch[8:0]}
    // **One bit more than the tag needs, and that bit is the verdict on the
    // word.**  140 bits is exactly 7 RAM10 (7 x 20); the 141st costs an eighth
    // block and it buys the property that a word's verdict cannot be separated
    // from the word.  The alternative -- a level synchronised beside the data --
    // does not hold, and it was checked on paper before this was written:
    // `axi_errors` is decided at the last beat and the display publishes the row
    // three or four `clk_pe` cycles later, so a level crossing would land within
    // about one cycle of the publish and the race would be settled by the clock
    // ratio rather than by the design.
    localparam integer DATA_LSB    = 0;
    localparam integer TAG_LSB     = 128;
    localparam integer BAD_LSB     = 128 + TAGW;            // 140
    localparam integer FIFO_DW     = 128 + TAGW + 1;        // 141 = 8 x 20
    localparam [6:0]   LAST_BEAT   = BEATS - 1;
    // Verilog-2001 has no bit-select on an integer parameter, so the widths the
    // comparisons need are their own constants.
    localparam [7:0]   BEATS_B     = BEATS;
    localparam [7:0]   LAST_B      = BEATS - 1;
    localparam [7:0]   SRW_B       = SRC_W;
    localparam [15:0]  TMO_MAX     = TIMEOUT;

    assign axi_arlen   = LAST_BEAT;
    assign axi_arsize  = 3'd4;          // 16 bytes, as the rest of this design uses
    assign axi_arburst = 2'b01;         // INCR

    // ------------------------------------------------------------------
    // The display-domain crossing registers, declared *here* rather than
    // beside the block that drives them.  They are read by the AXI block
    // above, and a signal used before its declaration is an implicit
    // one-bit net in this flow -- a trap this project has already paid for.
    // ------------------------------------------------------------------

    reg  [8:0]  live_epoch;     // display frames seen
    reg         req_tog;        // the crossing's toggle
    reg  [8:0]  d_epoch;        // the descriptor, held stable across it
    reg  [1:0]  d_fid;
    reg  [7:0]  d_row;
    reg         d_bank;
    reg  [31:0] d_base;
    reg         d_busy;         // issued and not yet acknowledged

    // ------------------------------------------------------------------
    // the read staging FIFO: exactly one burst
    // ------------------------------------------------------------------

    wire [FIFO_DW-1:0] rf_rd_data;
    wire               rf_rd_valid, rf_rd_empty, rf_wr_full;
    wire [AW:0]        rf_wr_level;
    wire               rf_rd_en;
    reg                rf_wr_en;
    reg  [FIFO_DW-1:0] rf_wr_data;

    d64_async_fifo #(.DW(FIFO_DW), .AW(AW)) u_rf (
        .wr_clk(clk_axi), .wr_rst_n(arst_axi_n),
        .wr_en(rf_wr_en), .wr_data(rf_wr_data), .wr_full(rf_wr_full),
        .wr_almost_full(), .wr_level(rf_wr_level),
        .rd_clk(clk_pe), .rd_rst_n(rst_pe_n),
        .rd_en(rf_rd_en), .rd_data(rf_rd_data), .rd_valid(rf_rd_valid),
        .rd_empty(rf_rd_empty), .rd_level()
    );

    // ==================================================================
    // AXI domain: one descriptor in, one burst out
    // ==================================================================

    reg  [1:0]  rtog_s;
    reg         rtog_seen;
    wire        rq_edge = rtog_s[1] ^ rtog_seen;

    // `n_*` is the descriptor as it arrived; `c_*` is the one the live burst
    // is reading.  **Both the address and `c_*` are latched on the cycle the
    // AR goes out**, not one at each end of it: latching `c_*` at the
    // handshake let a request landing in between move the tag without
    // moving the address, and since the tag names the frame and the bank
    // but not the row, the display accepted one row's pixels as another's.
    reg  [8:0]  n_epoch; reg [1:0] n_fid; reg [7:0] n_row; reg n_bank; reg [31:0] n_base;
    reg  [8:0]  c_epoch; reg [1:0] c_fid; reg [7:0] c_row; reg c_bank;

    reg         pending, burst_live, ack_tog;
    reg  [6:0]  sent;
    // The verdict on the burst in flight, and it is per-burst rather than
    // per-beat because a bad RRESP in the middle does not end a burst -- AXI
    // lets a slave return SLVERR for one beat and carry on -- so a row has to
    // stay condemned from the first bad beat all the way to the last word,
    // which is the only word the display looks at when it decides to publish.
    reg         burst_bad;

    wire [TAGW-1:0] c_tag = {c_bank, c_fid, c_epoch};

    // `beat_err` is this beat's fault; `beat_bad` is what the word travels with.
    // **All three ways a read can be untrustworthy are here**, and the third is
    // the one that used to be invisible: an RLAST arriving before the
    // sixty-fourth beat means the slave has ended the burst short, and the row
    // cannot be completed at all.
    wire r_err    = (axi_rresp != 2'd0);
    wire r_short  = axi_rlast && (sent != LAST_BEAT);    // RLAST before beat 64
    wire r_nolast = !axi_rlast && (sent == LAST_BEAT);   // beat 64 with no RLAST
    wire beat_err = r_err || r_short || r_nolast;
    wire beat_bad = burst_bad || beat_err;

    // **A fetch may only start against a quiet AXI side.**  Without this the
    // display ran several fetches while one blocked burst was still in flight;
    // the burst belonged to a fetch *two or more back*, and since `bank`
    // alternates, an even number of fetches apart carries the same tag -- so a
    // leftover was accepted as the current row's data.  Nothing about reading
    // tags can fix that, because the tag would then have to name the row and
    // the row repeats.  The fix is that there is never more than one
    // descriptor outstanding, so there is never more than one burst's worth of
    // words in the FIFO and no leftover to mis-attribute.
    wire a_idle = !burst_live && !pending && !axi_arvalid && (rf_wr_level == 0);

    always @(posedge clk_axi or negedge arst_axi_n) begin
        if (!arst_axi_n) begin
            rtog_s <= 2'd0; rtog_seen <= 1'b0; ack_tog <= 1'b0;
            n_epoch <= 9'd0; n_fid <= 2'd0; n_row <= 8'd0; n_bank <= 1'b0; n_base <= SRC_BASE;
            c_epoch <= 9'd0; c_fid <= 2'd0; c_row <= 8'd0; c_bank <= 1'b0;
            pending <= 1'b0; burst_live <= 1'b0; sent <= 7'd0; burst_bad <= 1'b0;
            axi_arvalid <= 1'b0; axi_araddr <= 32'd0; axi_rready <= 1'b0;
            rf_wr_en <= 1'b0; rf_wr_data <= {FIFO_DW{1'b0}}; axi_errors <= 32'd0;
        end
        else begin
            rf_wr_en <= 1'b0;
            rtog_s   <= {rtog_s[0], req_tog};

            // The toggle is the crossing; the payload beside it has been
            // stable since before the toggle and stays stable until the ack
            // below has been seen on the other side.
            if (rq_edge) begin
                rtog_seen <= rtog_s[1];
                ack_tog   <= ~ack_tog;
                n_epoch   <= d_epoch; n_fid <= d_fid; n_row <= d_row;
                n_bank    <= d_bank;  n_base <= d_base;
                pending   <= 1'b1;
            end

            // One burst at a time, and only into an empty FIFO -- the FIFO is
            // exactly one burst deep, so a beat cannot be lost.
            // **The address and the tag are latched together, at the AR, and
            // not one at each end of it.**  They were: the address came from
            // `n_*` on the cycle AR went high and `c_*` was copied on the
            // cycle it was accepted, and a request landing between the two
            // moved the tag without moving the address.  The tag does not
            // carry the row, so the display happily accepted a row's worth of
            // one row's pixels as another's -- which is what the source check
            // caught, eight columns of it.
            if (axi_arvalid) begin
                if (axi_arready) begin
                    axi_arvalid <= 1'b0;
                    burst_live  <= 1'b1;
                    sent        <= 7'd0;
                    burst_bad   <= 1'b0;    // a new burst, a new verdict
                end
            end
            else if (pending && !burst_live && (rf_wr_level == 0)) begin
                axi_arvalid <= 1'b1;
                axi_araddr  <= n_base + ({24'd0, n_row} << RSH);
                c_epoch     <= n_epoch; c_fid <= n_fid; c_row <= n_row; c_bank <= n_bank;
                pending     <= 1'b0;
            end

            // Backpressure is expressed, not relied on: if the FIFO were ever
            // full the beat is held on the bus rather than dropped.  The drain
            // is another clock domain and keeps running, so this cannot
            // deadlock the way the same statement does in d64_trunk_row_ddr.
            axi_rready <= burst_live && !rf_wr_full;

            if (axi_rvalid && axi_rready) begin
                rf_wr_en   <= 1'b1;
                rf_wr_data <= {beat_bad, c_tag, axi_rdata};
                if (beat_err) begin
                    axi_errors <= FAULT_STICKY ? 32'd1 : axi_errors + 32'd1;
                    burst_bad  <= 1'b1;
                end
                // **The burst ends at whichever comes first, and that is the
                // whole of the short-burst fix.**  It used to end only at
                // `sent == LAST_BEAT`, so an early RLAST left `burst_live` set
                // for ever: `a_idle` never came back, every later fetch took the
                // `S_WAIT` path, and after `WAIT_MAX` the display wrote black
                // lines for the rest of the run.  Nothing in the display
                // domain can recover from that, because nothing there can see
                // the bus -- so the master has to believe the RLAST it is given.
                if ((sent == LAST_BEAT) || axi_rlast) begin
                    burst_live <= 1'b0;
                    sent       <= 7'd0;
                end
                else sent <= sent + 7'd1;
            end
        end
    end

    // ==================================================================
    // display domain: request, drain, supersede, blank
    // ==================================================================

    reg  [1:0]  ack_s;          // ack_tog, synchronised
    // Reset **high**, because the AXI side is idle out of reset and a
    // synchroniser that starts low would race the frame's prime request --
    // which is issued two cycles after reset and would be blanked.
    reg  [2:0]  idle_s;
    wire        a_idle_s = idle_s[2];

    reg  [1:0]  st;
    reg         fill_bank;      // the bank tag this fetch carries (the scaler routes it)
    reg  [7:0]  wc;             // words accepted into the row so far
    reg  [7:0]  wi;             // pixels emitted into the row so far
    reg  [7:0]  px_i;           // lane within the word being emitted
    reg         have_word;
    reg  [FIFO_DW-1:0] wq;
    reg  [15:0] tmo;
    reg  [7:0]  bcol;
    reg         pend;
    reg  [7:0]  pend_row;
    reg         rd_pend;
    reg  [7:0]  wait_row;
    reg  [15:0] wt;
    // `blank_bad` says *why* the line is being blanked, so `S_BLANK` can count
    // the fault it actually was.  Without it every failed row lands in
    // `underrun` and "the memory answered with an error" is indistinguishable
    // from "the memory did not answer in time".
    reg         blank_bad;
    // F3 6.18.  Why *this* blank, separately from `blank_bad`: a refused fetch
    // is neither slow nor wrong, and the blank it produces must be counted on
    // the rotation that says so.
    reg         blank_refused;

    localparam [1:0] S_IDLE = 2'd0, S_FETCH = 2'd1, S_BLANK = 2'd2, S_WAIT = 2'd3;
    localparam [15:0] WAIT_MAX = 16'd256;

    wire [TAGW-1:0] d_tag  = {d_bank, d_fid, d_epoch};
    wire [TAGW-1:0] v_tag  = rf_rd_data[TAG_LSB +: TAGW];
    // **The verdict arrives with the word it is about.**  It is only meaningful
    // for a word whose tag matches -- a leftover from an abandoned burst is not
    // ours to judge -- which is why every use of it below is guarded by `v_acc`.
    wire            v_bad  = rf_rd_data[BAD_LSB];
    wire            v_tok  = (v_tag == d_tag);
    wire            v_live = (d_epoch == live_epoch) && (d_fid == frame_id);
    wire            v_acc  = v_tok && v_live;

    wire            word_end = have_word && (px_i == PX_PER_BEAT-1);
    // Popped in idle as well as in a fetch.  In idle the word is dropped --
    // the capture below is gated on S_FETCH -- and draining is what keeps
    // the FIFO quiet between fetches, which is what `a_idle` needs to see.
    wire            pop      = (st != S_BLANK) && !have_word
                             && !rd_pend && !rf_rd_empty;
    assign          rf_rd_en = pop;

    wire [23:0] lane0 = wq[23:0],  lane1 = wq[55:32];
    wire [23:0] lane2 = wq[87:64], lane3 = wq[119:96];
    reg  [23:0] pxw;
    always @(*) case (px_i[1:0])
        2'd0:    pxw = lane0;
        2'd1:    pxw = lane1;
        2'd2:    pxw = lane2;
        default: pxw = lane3;
    endcase

    always @(posedge clk_pe or negedge rst_pe_n) begin
        if (!rst_pe_n) begin
            rd_pend <= 1'b0;
            ack_s   <= 2'd0;
            idle_s  <= 3'b111;
        end
        else begin
            ack_s  <= {ack_s[0], ack_tog};
            idle_s <= {idle_s[1:0], a_idle};
            if (rf_rd_valid) rd_pend <= 1'b0;
            else if (pop)    rd_pend <= 1'b1;
        end
    end

    // The pixel window is latched when the word is taken, so a word's four
    // pixels are all emitted under the same verdict.
    always @(posedge clk_pe or negedge rst_pe_n) begin
        if (!rst_pe_n) begin
            have_word <= 1'b0; wq <= {FIFO_DW{1'b0}}; px_i <= 8'd0;
        end
        // **`st == S_FETCH` is load-bearing.**  Without it a word still in
        // the FIFO when a fetch was blanked was captured here, sat in
        // `have_word` through the idle cycle, and was then emitted under the
        // *next* fetch's descriptor -- one row's pixels written into another
        // row's bank slot, which the source check caught as eight columns of
        // it.  A word belongs to a fetch only while that fetch is running.
        // **A word that carries the verdict is not captured.**  It is drained --
        // the FSM blanks on the same cycle -- and `wc` never counts it, so no
        // pixel of a failed burst can reach the row bank's emitter.  Letting it
        // be captured and relying on `S_BLANK` to overwrite it also works, since
        // `S_BLANK` drives `load_we` in preference to the fetch; but it leaves
        // `have_word` set through the blank and the two paths then disagree
        // about whose word it is, and that is not a thing to rest on a priority.
        else if ((st == S_FETCH) && rf_rd_valid && v_acc && !v_bad) begin
            wq <= rf_rd_data; have_word <= 1'b1; px_i <= 8'd0;
        end
        else if (have_word) begin
            if (word_end) have_word <= 1'b0;
            else          px_i <= px_i + 8'd1;
        end
    end

    always @(posedge clk_pe or negedge rst_pe_n) begin
        if (!rst_pe_n) begin
            load_we <= 1'b0; load_wi <= 8'd0; load_wd <= 24'd0;
        end
        else begin
            load_we <= 1'b0;
            if (st == S_BLANK) begin
                load_we <= 1'b1; load_wi <= bcol; load_wd <= 24'h000000;
            end
            else if ((st == S_FETCH) && have_word) begin
                load_we <= 1'b1; load_wi <= wi; load_wd <= pxw;
            end
        end
    end

    // ------------------------------------------------------------------
    // the request/response FSM
    // ------------------------------------------------------------------

    wire go     = (st == S_IDLE) && !d_busy && !have_word && !rd_pend &&
                  rf_rd_empty && (load_req || pend);
    // **Eight bits, written down.**  A `wire` with no range is one bit,
    // and the ternary's context would have truncated the row to its low
    // bit -- which is a picture that shows rows 0 and 1 and nothing else,
    // and looks plausible enough that the pixels are what caught it.
    wire [7:0] go_row = load_req ? load_row : pend_row;

    // ------------------------------------------------------------------
    // F3 6.18: the 4 KiB rule, in the shape `d64_axi_read_burst.v` states it
    // ------------------------------------------------------------------
    //
    // **The same expression the AXI block below uses to form its address**
    // (`n_base + ({24'd0, n_row} << RSH)`), evaluated one domain earlier so the
    // refusal can have a defined ending.  Declared here and not above, because
    // a signal used before its declaration is an implicit one-bit net in this
    // flow -- a trap this project has already paid for, and the reason the
    // display-domain crossing registers are declared at the top of the AXI
    // block rather than beside the logic that drives them.
    localparam [31:0] BURST_BYTES = ROW_BYTES;
    wire [31:0] go_addr   = src_base + ({24'd0, go_row} << RSH);
    wire [31:0] page_span = {20'd0, go_addr[11:0]} + BURST_BYTES;
    wire        ar_cross4k = (page_span > 32'd4096) & (SAB_NO_PAGE_GUARD == 0);

    always @(posedge clk_pe or negedge rst_pe_n) begin
        if (!rst_pe_n) begin
            st          <= S_IDLE;
            row_ready   <= 1'b1;
            row_good    <= 1'b0;
            fill_bank   <= 1'b0;
            req_tog     <= 1'b0;
            d_busy      <= 1'b0;
            live_epoch  <= 9'd0;
            d_epoch     <= 9'd0; d_fid <= 2'd0; d_row <= 8'd0;
            d_bank      <= 1'b0; d_base <= SRC_BASE;
            wc          <= 8'd0;  wi  <= 8'd0;  tmo <= 16'd0; bcol <= 8'd0;
            wt          <= 16'd0; wait_row <= 8'd0;
            pend        <= 1'b0;  pend_row <= 8'd0;
            rows_loaded <= 32'd0; stale_beats <= 32'd0; underrun <= 32'd0;
            bad_rows    <= 32'd0; first_bad_row <= 8'd0; blank_bad <= 1'b0;
            blank_refused <= 1'b0; page_refusals <= 32'd0;
        end
        else begin
            // ---- the frame counter the tags are made of --------------
            if (frame_tick) live_epoch <= live_epoch + 9'd1;

            // ---- the acknowledge, which releases the payload ---------
            if (ack_s[1] ^ ack_s[0]) d_busy <= 1'b0;

            // ---- the timer -------------------------------------------
            if (st == S_FETCH) tmo <= tmo + 16'd1;

            // ---- word accounting -------------------------------------
            if (word_end) begin
                wc <= wc + 8'd1;
                wi <= wi + 8'd1;
            end
            else if ((st == S_FETCH) && have_word) wi <= wi + 8'd1;

            if (st == S_FETCH) begin
                // the fetch is over the moment its last word is emitted,
                // but only if the frame it was made for is still the frame
                // wanted.  Otherwise the row is re-requested, not shown.
                if (word_end && (wc == LAST_B)) begin
                    if (v_live) begin
                        rows_loaded <= rows_loaded + 32'd1;
                        row_ready   <= 1'b1;
                        row_good    <= 1'b1;
                        st          <= S_IDLE;
                    end
                    else begin
                        st <= S_IDLE;
                        if (!pend) begin pend <= 1'b1; pend_row <= d_row; end
                        stale_beats <= stale_beats + {24'd0, BEATS_B} - {24'd0, wc};
                    end
                end

                // a word that is ours but belongs to a frame we have left:
                // stop now, count what is left of the row, and re-request it
                if (rf_rd_valid && v_tok && !v_live) begin
                    st <= S_IDLE;
                    if (!pend) begin pend <= 1'b1; pend_row <= d_row; end
                    stale_beats <= stale_beats + {24'd0, BEATS_B} - {24'd0, wc};
                end

                // a word that is not ours at all is a leftover from an
                // abandoned burst; skip it and keep going
                if (rf_rd_valid && !v_tok)
                    stale_beats <= stale_beats + 32'd1;

                // ---- the write-off -----------------------------------
                if (!have_word && !rd_pend && (tmo >= TMO_MAX)) begin
                    st            <= S_BLANK;
                    bcol          <= 8'd0;
                    blank_bad     <= 1'b0;      // slow, not wrong
                    blank_refused <= 1'b0;
                end

                // ---- a word the memory said was bad ------------------
                //
                // **Last in this block on purpose**, because it has to override
                // the publish above: a bad beat can be any of the sixty-four,
                // including ones already emitted into the bank, so the only
                // verdict that can be earned is "this row is not trustworthy".
                // `S_BLANK` then writes 256 black pixels over the whole bank
                // before `row_ready`, so nothing half-good is ever read and the
                // retroactive part costs nothing.
                if (rf_rd_valid && v_acc && v_bad) begin
                    st            <= S_BLANK;
                    bcol          <= 8'd0;
                    blank_bad     <= 1'b1;
                    blank_refused <= 1'b0;
                    if (bad_rows == 32'd0) first_bad_row <= d_row;
                    bad_rows <= FAULT_STICKY ? 32'd1 : bad_rows + 32'd1;
                end
            end

            if (st == S_WAIT) begin
                wt <= wt + 16'd1;
                if (a_idle_s) begin
                    st   <= S_IDLE;
                    pend <= 1'b1; pend_row <= wait_row;
                end
                else if (wt >= WAIT_MAX) begin
                    st            <= S_BLANK;
                    bcol          <= 8'd0;
                    blank_bad     <= 1'b0;      // blocked, not wrong
                    blank_refused <= 1'b0;
                end
            end

            if (st == S_BLANK) begin
                row_good<=1'b0;
                if (bcol == SRW_B - 8'd1) begin
                    // **One counter or the other, never both, and now three.**
                    // Which one was set where the blank was entered; the fault
                    // that caused it is the fault that gets counted, and a row
                    // that was both slow and wrong is reported as wrong, which
                    // is the reading that asks for a different investigation.
                    // A refusal moves `page_refusals` and neither of the others.
                    if      (blank_refused) page_refusals <= page_refusals + 32'd1;
                    else if (!blank_bad)    underrun <= FAULT_STICKY ? 32'd1 : underrun + 32'd1;
                    row_ready <= 1'b1;
                    st        <= S_IDLE;
                end
                else bcol <= bcol + 8'd1;
            end

            // ---- a new fetch -----------------------------------------
            if (go) begin
                row_good<=1'b0;
                pend <= 1'b0;
                if (ar_cross4k) begin
                    // F3 6.18.  **Refused, and nothing reaches the bus.**  The
                    // address this fetch would have used spans two 4 KiB pages,
                    // so the slave is not required to answer it and an
                    // interconnect that does answer may answer from the wrong
                    // page.  There is no version of this row that is right, so
                    // the row is blanked -- 256 black pixels and `row_ready`,
                    // the same ending an underrun has -- and the event is
                    // counted in `page_refusals` alone.  `S_BLANK` counts it.
                    // `d_*` and `req_tog` do not move, so no descriptor is ever
                    // made and `d_busy` is never set; the loader is idle again
                    // the moment the blank is written.
                    st            <= S_BLANK;
                    bcol          <= 8'd0;
                    blank_bad     <= 1'b0;
                    blank_refused <= 1'b1;
                    row_ready     <= 1'b0;
                end
                else if (a_idle_s) begin
                    d_epoch    <= live_epoch;
                    d_fid      <= frame_id;
                    d_row      <= go_row;
                    d_bank     <= fill_bank;
                    d_base     <= src_base;
                    fill_bank  <= ~fill_bank;
                    req_tog    <= ~req_tog;
                    d_busy     <= 1'b1;
                    row_ready  <= 1'b0;
                    st         <= S_FETCH;
                    wc         <= 8'd0; wi <= 8'd0; tmo <= 16'd0;
                end
                else begin
                    // The AXI side is not quiet.  Usually that is the few
                    // cycles after a supersede, while `burst_live` and the
                    // synchronised read pointer catch up with a drain that
                    // has already finished -- so wait, briefly, and then
                    // fetch for real.  If it is still not quiet after
                    // `WAIT_MAX`, the supply is genuinely blocked: write the
                    // line black, and a supply blocked for N lines is N black
                    // lines and N counted underruns.  `row_ready` never drops
                    // either way, so the scaler is not held up by a memory it
                    // cannot see.
                    st       <= S_WAIT;
                    wait_row <= go_row;
                    wt       <= 16'd0;
                end
            end
            else if (load_req) begin
                // a request that could not be served now, remembered.  Last
                // one wins, which is right: the scaler asks for the row the
                // *next* line will show, so the newest request is the one.
                pend     <= 1'b1;
                pend_row <= load_row;
            end
            // A publication/frame boundary invalidates any previous ready
            // bank verdict, including a completion coincident with this edge.
            if(frame_tick)row_good<=1'b0;
        end
    end

endmodule
