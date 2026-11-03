`timescale 1ns/1ps

// ============================================================================
// d64_rgbx_beat_writer -- four RGBX pixels in, one 128-bit DDR beat out.
// ============================================================================
//
// **There is no straddle here and there is no repacking.**
//
// `d64_tail_engine` emits one 32-bit `0x00RRGGBB` word a pixel, one pixel a
// cycle, on `out_wr_en` / `out_wr_addr[17:0]` / `out_wr_data[31:0]` (that
// module's port list, `:190`).  The board's DDR moves 128-bit beats.  Four
// 32-bit pixels **are** 128 bits, so four pixels are exactly one beat, and the
// pixel's byte offset and the beat's byte offset are the same count of bytes
// seen from two different ends:
//
//     beat addr  =  TAIL_BASE + pixel_index * 4        (a pixel)
//                =  TAIL_BASE + beat_index  * 16       (a beat)
//     beat_index =  pixel_index / 4,  lane = pixel_index % 4
//
// Nothing is split across a beat, nothing is shifted by a partial amount, and
// no repacker exists between this module and the engine.  That is said first
// and at length because this project has already spent a day on an imaginary
// 80-to-128 straddle -- `d64_h0_block.v:17` ("the 80/128 straddle does not
// exist") and the implementation at `:400` -- and the way that day is not
// repeated is by the module that would have had to care saying that it does
// not.  `docs/F3_INTERFACES_2026-10-07.md` section 1 lists it as settled:
// **do NOT reopen these.**
//
// The region, and where its number comes from
// -------------------------------------------
//
// 256 x 256 x 4 = **262144 B**, which is the TAIL region `d64_net_addr` already
// defines -- `TAIL_BASE = 32'h005C_0000`, `TAIL_W = 256`, `TAIL_H = 256`,
// `TAIL_S_W = TAIL_C * TAIL_ELEM_B = 4` bytes a pixel, `TAIL_SIZE = 262144`.
// The map's own header calls that number exact ("TAIL 256*256*4 = 262144 B =
// 256 KiB exactly -- nothing was rounded"), so this module takes it as a
// parameter rather than retyping it, and the testbench reads `sel_base` and
// `sel_size` back out of an instantiated `d64_net_addr` and requires the two to
// agree.  A constant retyped on both sides of an interface is a constant that
// can be wrong on both sides.
//
// Where a byte goes, exactly
// --------------------------
//
// `d64_net_addr`'s TAIL note settles the byte order and is quoted rather than
// re-derived: "lane 0 is B, 1 is G, 2 is R, 3 is X for a little-endian
// 0x00RRGGBB".  So pixel `p`'s four bytes land at image offsets `4p .. 4p+3` as
// `B, G, R, X`, and inside a beat the pixel at lane `L` occupies
// `beat_data[32*L +: 32]` with the same three low bytes carrying RGB.  That is
// bit-for-bit `rgbx_frame_writer`'s map -- its header's
//
//     lane0 = wq[23:0]  pixel 0   -- "one 128-bit word is four pixels and the
//     lane1 = wq[55:32] pixel 1      RGB is the low 24 bits of each 32-bit
//     lane2 = wq[87:64] pixel 2      slot -- the top byte is RGBX's X"
//     lane3 = wq[119:96] pixel 3
//
// -- and that module is board-verified, which is why this one is written to the
// same map rather than to a second one that merely agrees with it today.
//
// **The lane comes from the address and not from an arrival count.**  A pixel's
// lane is its own `out_wr_addr[1:0]`, so a beat is right even if its four
// pixels arrive out of order, and the whole of the packing is one indexed part
// select on `out_wr_data`.  `cnt` counts arrivals only to know when the beat is
// complete; it never selects a lane.  A build whose lane came from the arrival
// order would be a build that is wrong for any producer that reorders a phase,
// and the difference is not observable from an in-order stream -- so the
// testbench drives a second run with each beat's four pixels permuted,
// `tools/run_rgbx_beat_writer.sh`'s `shuffled` arm.
//
// The beat boundary, in full
// --------------------------
//
// A beat is pushed to the bus queue on the cycle the **fourth** pixel of that
// beat arrives (`cnt == LPC_LAST`), and at no other time.  A partial beat is
// never written: there is no path from a pixel to the queue that does not pass
// through that comparison, so a frame that ends mid-beat leaves the last beat
// unassembled rather than writing three quarters of it.  The testbench requires
// both halves of that -- 16384 beats and never 16385 -- and separately requires
// that no beat is handed to the bus before the cycle in which its own last
// pixel was driven.
//
// The bus: raw AXI4 write master
// ------------------------------
//
// `docs/F3_INTERFACES_2026-10-07.md` section 3.2 fixes the board's DDR as **raw
// AXI** and names `d64_video_row_ddr.v` as the working precedent, and this
// module's sibling on the read side, `board/ddr3/d64_axi_read_burst.v`, uses
// exactly that port set.  The write half is therefore spelled the same way --
// `axi_awvalid/awready/awaddr/awlen/awsize/awburst`, `axi_wvalid/wready/
// wdata/wstrb/wlast`, `axi_bvalid/bready/bresp` -- which is also the master-side
// port of `d64_axi_arb2.v`, the arbiter this design's masters hang off.
//
// **`rgbx_frame_writer`'s precedent is the payload and the discipline; the
// delta is the address channel's spelling.**  That module drives the EFX DDR
// IP's shared A/ATYPE port (`avalid/aready/aaddr/alen/asize/aburst/atype` plus
// `aid`/`awlock`); the W, B and R channels of the two are identical, signal for
// signal.  What is taken from it here, and why each one:
//
//   `wstrb = 16'hFFFF`   every byte of every beat is written.  There is no
//                        partial beat to strobe, which is the same statement as
//                        "no straddle" made in the write-mask's language.
//   `awsize = 3'd4`      sixteen bytes, the full 128-bit bus; `awburst` INCR.
//                        The same two values `d64_trunk_row_ddr.v:127` uses
//                        against this controller.
//   `wdata`/`wlast` are **combinational off the queue head**, never registered.
//                        `rgbx_frame_writer`'s header records the bug that made
//                        this a rule: registering them shipped the *previous*
//                        beat's data, and the last beat rode with WLAST low.
//                        Row 0 of its test frame was all border and hid it.
//   one burst a row      256 pixels / 4 = 64 beats = 1024 B, and 256 rows x 64
//                        = 16384 beats, which is the whole region with nothing
//                        left over.  The region is byte-contiguous across rows
//                        (`TAIL_S_H = 1024 = TAIL_ROW_B`), so a 64-beat burst
//                        is 64 consecutive addresses of one image whatever the
//                        row boundary does.
//   the response is real  `W_RESP` and `bresp` are the same states
//                        `d64_trunk_row_ddr.v:494` retires a block on, and the
//                        reason is in `d64_done_tracker.v`'s header: a frame is
//                        ready when the memory has said it took it.
//
// Backpressure: the committed row, and what is not claimed
// -------------------------------------------------------
//
// **`d64_tail_engine` has no output stall input.**  Its port list is
// `out_wr_en`/`out_wr_addr`/`out_wr_data` and nothing else, so this module
// cannot ask it to wait and does not invent a signal the engine would not
// answer.  A pixel the writer does not take in the cycle it is offered is gone,
// so **the buffer is the whole of the protection** and this section is about
// how deep it has to be.
//
// The rate, from the engine's own schedule: a phase is four pixels and twelve
// cycles (`d64_tail_engine.v`, "the schedule, in cycles from the start of a
// phase"), so the engine offers **one beat every twelve cycles** while the bus
// takes one a cycle.  `docs/TAIL_OUTPUT_CHAIN_2026-10-07.md` measures the same
// ratio from the far end: 16384 beats a frame with 176,640 `stall_cycles` at 64
// beats a burst.  That is why the queue covers the bus's latency and not the
// engine's rate -- and it is also why an overflow is a **bus** event and not a
// producer event.  The bus's wait is the entire input to the depth.
//
// **The depth is the burst.**  The writer takes an AXI commitment of
// `BURST_BEATS` beats the moment it raises `axi_awvalid`, so a buffer that
// cannot hold the burst it has already committed to is a buffer that a bus wait
// can empty while the writer still owes beats of a burst it opened.  `BQ_DEPTH`
// is the caller's floor and **`BQ_N` is what is built**:
//
//     BQ_N = max(BQ_DEPTH, BURST_BEATS)
//
// -- the committed-row buffer, in the review's phrase (`REVIEW_FINDINGS.md` G02:
// "issue before credit reservation, or a committed-row buffer").  The writer's own
// defaults move from 4 beats to 64, and that is the difference between absorbing
// a 48-cycle bus wait and absorbing a 768-cycle one, because the engine makes a
// beat every twelve cycles:
//
//     BQ_N =   4     4 x 12 =   48 cycles
//     BQ_N =  64    64 x 12 =  768 cycles
//
// **That number is a bound and it is stated as one.**  `BQ_N` beats of engine
// production is the longest single wait this writer absorbs without losing a
// word.  Past it a beat is lost, `queue_overflows` counts it and `frame_fault`
// latches: the beat is written no more, the frame is reported and is **not
// publishable**, where before the same event was a frame that was wrong in one
// place and said nothing.  **No writer-only change can do better than a bound.**
// The engine cannot be asked to wait, so an unbounded wait needs an unbounded
// buffer, and an unbounded buffer is not a design.  A guarantee for an
// *unbounded* wait would require an `out_ready` on the engine's write port --
// a contract amendment of the kind `docs/F3_INTERFACES_2026-10-07.md` section
// 6.9 had to make for the read side.  **This change does not make it and does
// not invent the port**, and section 0's rule is why it says so here instead of
// quietly adding a signal the engine would not answer.
//
// Two numbers say the bound is real, and both are measured:
//
//   `queue_peak`     the high-water mark of the occupancy.  **1** on the quiet
//                    arm of `tools/run_tail_write_backpressure.sh`, where the
//                    bus keeps up: the queue is empty at the start of every
//                    wait, so all of `BQ_N` is headroom for the bus.  The same
//                    gate's 600-cycle window reaches **51** -- the 50 beats the
//                    engine makes in 600 cycles plus the one already queued,
//                    which is the arithmetic the bound above is made of.
//   `bus_wait_max`   the longest single run of cycles in which the writer had a
//                    beat to hand over and the bus would not take it -- the AW
//                    of the open burst refused, the W beat refused, or the
//                    response not returned.  This is the number `BQ_N` must
//                    cover: **0** on the quiet arm, **601** against the window.
//
// `stall_cycles` is the other direction's count and stays: the cycles a burst
// was open with no beat to send.  It is reported rather than required to be
// zero, because in this direction "the bus is waiting for the engine" is the
// honest description and not a fault.
//
// A response that is an error, and what it does not mean -- F03, 2026-10-07
// -------------------------------------------------------------------------
// The root review injected `BRESP = SLVERR` on this writer's response and
// measured the pair: `bresp_errors = 1`, and downstream the tracker raising
// `done = 1` with `reason = COUNT`.  Both numbers are true, and together they
// are the defect: "every response arrived" was being read as "the frame may be
// published", and those are two different claims.
//
// So `wr_ack` is **unchanged** -- a response retires its transaction whether or
// not it carried an error, and withholding the pulse would convert a reported
// failure into a tracker waiting for an answer that has already come -- and the
// verdict is separated out instead: `wr_ok` for a successful commit, and the
// sticky `frame_fault` for the frame.  A faulted frame is **still written**:
// the beats keep going to the memory and the responses keep retiring, because
// stopping would leave a burst half-sent.  What the fault buys is the claim:
// `d64_done_tracker.v`'s `publishable` must not rise for it, and the display
// keeps the previous frame.
//
// Teeth
// -----
//
// Two builds, each required to fail, and each aimed at one claim above:
//
//   `SWAP_BYTES_SABOTAGE`  byte lanes 0 and 1 of the assembled beat exchange
//                          places.  Lanes 0 and 1 of the low 32-bit lane are
//                          that pixel's B and G.  Every address, every beat
//                          count and every burst stays right and only the byte
//                          *position* moves, so this is the tooth that says the
//                          byte-position check is load-bearing and not a check
//                          that passes on anything.
//   `PACK3_SABOTAGE`       a beat is completed after **three** pixels.  This is
//                          the tooth for the beat boundary: the fourth pixel of
//                          each group of three starts the next beat, whose
//                          address is the same 16-byte slot as its own group's,
//                          so duplicates, gaps and a partial last beat all
//                          appear at once.
// Credit, successful commit, and drained -- the writer's obligation, named
// ---------------------------------------------------------------------
// Three things a parent needs and used to have to infer, stated in full in
// `docs/F3_INTERFACES_2026-10-07.md` section 6.20, one line each:
//
//   **credit**            how many beats the writer may have in flight:
//                         `BQ_N = max(BQ_DEPTH, max(BURST_MAX, CREDIT_BEATS))`,
//                         reported on `credit_beats` / `credit_free`.  A beat
//                         this writer accepts is a beat it had credit for.
//   **successful commit** a beat is written when the memory's response says
//                         `OKAY`.  A beat *accepted* is not a beat committed,
//                         and a response is one per **burst**: `wr_ack` retires
//                         (any response, error or not -- F03) and `wr_ok`
//                         commits.
//   **drained**           the queue empty **and** no burst open **and** no beat
//                         completing into the queue this cycle.  A response says
//                         nothing about the queue, so "everything was
//                         acknowledged" is a different claim; `drained` is the
//                         one "the frame is finished" has to read.
//
// **`CREDIT_BEATS` is the fix for issue #109 and it is a bound, not a
// guarantee.**  `d64_tail_output_chain` ships `BURST_BEATS = 1` with
// `BQ_DEPTH = 4`, so the credit was **four beats** -- 48 cycles of the tail
// engine's rate -- and a bus that refused AW, W or B for longer than that wrote
// the queue full and lost the word.  A lost word is not a small loss: the frame's
// burst arithmetic is then an integer short, the last burst is never completed,
// its response never comes, and **the frame never drains**.  An *unbounded*
// guarantee needs an `out_ready` on the engine's write port, which is a contract
// amendment this change does not make.
//
// The 4 KiB rule is a **refusal**: a base that is not page-aligned cannot be
// served by an interface with no end-of-transfer input, so it is refused and
// counted (`page_refusals`) with `frame_fault` latched -- never crossed and never
// silently split.  `BURST_BEATS` above a page is clamped to the page and counted
// (`burst_clamps`), and the clamped length is on `burst_beats`.
//
//   `BQ_FLOOR_SABOTAGE`    the queue floor is removed: the writer is built
//                          with the caller's `BQ_DEPTH` instead of
//                          `max(BQ_DEPTH, BURST_BEATS, CREDIT_BEATS)`, so it
//                          commits to a `BURST_BEATS`-beat burst with a
//                          `BQ_DEPTH`-beat buffer.  Nothing else moves.  Against
//                          a long bus wait it overflows, drops beats, and the
//                          frame's addresses shift from the first drop on -- the
//                          tooth that says the credit is load-bearing and not a
//                          number.
//   `PAGE_SPLIT_SABOTAGE`  the page rule is removed: the writer opens a burst
//                          for a base it cannot serve, so the burst crosses a
//                          4 KiB boundary.  Every address, count and byte stays
//                          right and only the *span* is illegal -- which is the
//                          point, because a real interconnect may split or reject
//                          that burst and the failure would surface as wrong data
//                          rather than as a refusal.
//
// **The credit's tooth is the named parameter itself, not a sabotage**: an arm
// built at `CREDIT_BEATS = 8` against a 600-cycle window loses 43 words, which is
// the honest statement that the number is load-bearing.  And the *pre-fix* build
// is the tooth for the rest of the change: a base 512 bytes off a page boundary
// makes the unclamped writer cross 64 of 64 bursts, on data, which is
// `tools/run_tail_writer_page.sh`'s `PAGE_SPLIT_SABOTAGE` arm.
//
// What is verified and what is not
// --------------------------------
//
// Verified by `tools/run_rgbx_beat_writer.sh` and nothing else: that every byte
// of every one of the 16384 beats holds the right byte of the right pixel in the
// right lane, that the 16384 beat addresses are distinct, in-region and without
// a gap, that the region this writer fills is the region `d64_net_addr` defines,
// that no beat is written before its own last pixel has arrived, and that the
// two sabotages above fail.  See `docs/RGBX_BEAT_WRITER_2026-10-07.md` for the
// counts.
//
// **The depth's claim is verified by `tools/run_tail_write_backpressure.sh`**,
// which is not this module's own gate: a slave that refuses AW, W and B for a
// long window leaves `queue_overflows` at zero and every one of the 262,144
// bytes right, and the same window with `BQ_FLOOR_SABOTAGE` set overflows and
// loses words.  See `docs/TAIL_WRITE_BACKPRESSURE_2026-10-07.md`.
//
// `wr_ok`, `wr_oks` and `frame_fault` are verified by
// `tools/run_axi_error_drain.sh` (F03), which drives this writer and
// `d64_done_tracker.v` together with a response that carries `SLVERR` and
// requires the retire and the verdict to be separable -- and nothing in *this*
// module's own gate reads them.
//
// **Not verified:** anything about the board.  Nothing here has run against a
// DDR controller, and there is no claim about the full network -- `F3` section 4
// item 5 is explicit that these are pieces and the network does not exist yet.
// Nor is the reverse direction checked: this module writes, and the reader that
// will consume TAIL is not written yet, so "the display will see this frame" is
// a claim nobody has made.
// ============================================================================
module d64_rgbx_beat_writer #(
    // The TAIL region's base, as `d64_net_addr` parameterises it.  A parameter
    // and not a literal: the map owns the number, this module asks for it, and
    // the testbench compares the two rather than trusting either.
    // **Moved by `afa71bd` when UPS0/UPS1 became U16 images and the map's
    // regions shifted.**  This default was `0050_0000` and the module's own
    // gate hardcoded the same number, so both were wrong together -- which is
    // exactly the failure that gate's own comment warns about ("a constant
    // retyped on both sides ... can be wrong on both sides"), and it went
    // unnoticed for seven minutes of commits until another gate compared
    // against an instantiated `d64_net_addr` and disagreed.
    parameter [31:0]  TAIL_BASE  = 32'h005C_0000,

    // The pixel index is 18 bits because that is what the engine's port is --
    // `{2'b00, pk_r, 8'b0} + {10'b0, pk_g, 2'b00} + {16'b0, pk_i}` -- even
    // though 256*256 = 65536 pixels is a 16-bit number.  Kept at 18 so the port
    // joins `out_wr_addr` without a slice, which is the same reason
    // `d64_net_addr.tail_word()` is 18 bits wide.
    parameter integer PIX_W      = 18,

    // Pixels a beat.  Four, and it is not a tuning knob: four 32-bit pixels are
    // 128 bits and the bus is 128 bits.  `PACK3_SABOTAGE` moves it, which is
    // the point of that parameter and the only reason it is not a localparam.
    parameter integer PIX_PER_BEAT = 4,

    // One AXI INCR burst covers one image row.  See the header.
    parameter integer BURST_BEATS = 64,

    // Reserve the complete produced burst before claiming AW. The caller
    // must retain spare queue credit for production while AW/W/B wait.
    parameter integer RESERVE_WRITE_BURST = 0,

    // Beats buffered between the packer and the bus, as the caller's floor.
    // See the header: the engine has no stall input, so this is the whole of
    // what absorbs a busy memory.  **The queue is never shallower than the bus
    // commitment**: `BQ_N`, the depth actually built, is
    // `max(BQ_DEPTH, BURST_BEATS)`.  A caller that asks for less than one burst
    // is given one and told so -- `BQ_N` is what the header and the gate quote.
    parameter integer BQ_DEPTH   = 4,

    // **The credit: how many beats the writer may have in flight.**  A named
    // number and not a consequence of the two above, because those two are a
    // floor and a burst and neither of them answers the question.  The writer's
    // credit is the largest of the three: a caller that asks for a deeper queue
    // gets one, and a caller that asks for a short burst or a shallow floor is
    // still given a credit that covers the bus wait the TAIL chain meets.
    //
    // The default is 64 beats, which is 768 cycles of the tail engine's own
    // production (four pixels in twelve cycles, one beat every twelve), and
    // `docs/TAIL_WRITE_BACKPRESSURE_2026-10-07.md` measures a 600-cycle window
    // absorbed with it against the same window losing 47 words without it.
    // **At the geometry `d64_tail_output_chain` shipped with -- `BURST_BEATS =
    // 1`, `BQ_DEPTH = 4` -- the credit was four beats, 48 cycles, and a bus that
    // stalled for longer lost a write.**  That is issue #109 and this parameter
    // is the fix, stated as a bound.
    parameter integer CREDIT_BEATS = 64,

    // ---- verification only, folded away at 0 ------------------------------
    parameter integer SWAP_BYTES_SABOTAGE = 0,
    parameter integer PACK3_SABOTAGE      = 0,
    // Removes the floor above, and nothing else: `BQ_N` becomes `BQ_DEPTH`.
    // See the header's teeth.
    parameter integer BQ_FLOOR_SABOTAGE   = 0,
    // The page clamp removed: the writer asks the memory for the burst it would
    // have asked for before this change, which crosses a page for any base that
    // is not page-aligned.
    parameter integer PAGE_SPLIT_SABOTAGE = 0,

    // **The read pipeline's tooth.**  At 0 the queue's read address is the head
    // the *next* cycle will want -- it advances with the pop -- which absorbs
    // the block RAM's one cycle of read latency.  At 1 the address is the head
    // as it stands *now*, which is the scratch diagnostic's shape: the beat
    // handed over is the one the read issued a cycle ago, so every beat of a
    // burst is its predecessor, repeated.  Both builds declare the same length,
    // drive the same number of beats and put WLAST on the same beat; only the
    // bytes move.
    parameter integer RDPIPE_SABOTAGE     = 0,

    // **The admission's second reading**, and it is a measurement rather than a
    // change.  At 0 the admission reads the occupancy as it stood at the start
    // of the cycle, so a beat landing in the cycle a pop lands is refused
    // although the pop frees the very cell it would write.  At 1 the pop is
    // credited and that beat is taken.
    //
    // At 0 it costs at most one word per saturation -- the freed slot is used by
    // the next beat -- and section 6.20 of
    // `docs/F3_INTERFACES_2026-10-07.md` records that an arm built to separate
    // the two **could not**, so the change was not made.  This parameter exists
    // so the separation can be asked for again on a bus that provokes it rather
    // than argued about: `tb_d64_tail_writer_page.v`'s `TB_SYNC`, a bus that
    // accepts exactly one beat a beat period, in the cycle the beat is produced.
    parameter integer ADMIT_POP           = 0,

    // **The bite's tooth: the bypass removed.**  At 0 a read that lands on the
    // cell a push is writing this cycle is answered with the pushed word, which
    // is what the bus must see.  At 1 it is answered with the memory's own
    // output -- the **old** word -- which is what the RAM alone would give.  The
    // two occupancies where a push and a read share a cell are the empty queue
    // (the first beat of a frame, and the beat after any drain) and the queue
    // holding exactly one beat (a bus rate-matched to the engine, and the
    // configuration `d64_tail_output_chain` ships), so this tooth fires on the
    // first beat of a frame and on the steady state rather than on a corner.
    parameter integer BYPASS_SABOTAGE     = 0
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- the tail engine's output, wired straight in ----------------------
    // The names are the engine's own, so a wiring mistake is a name that does
    // not resolve rather than a name that resolves to the wrong thing.  This is
    // the same choice `d64_axi_read_burst.v` makes for `d64_burst_prefetch`'s
    // `rd_*` port.
    input  wire                     out_wr_en,
    input  wire [PIX_W-1:0]         out_wr_addr,
    input  wire [31:0]              out_wr_data,

    // ---- AXI4 write master, 128-bit beats --------------------------------
    output reg  [31:0]              axi_awaddr,
    output reg  [7:0]               axi_awlen,     // beats MINUS ONE, as AXI4
    output wire [2:0]               axi_awsize,
    output wire [1:0]               axi_awburst,
    output reg                      axi_awvalid,
    input  wire                     axi_awready,
    output wire                     axi_wvalid,
    input  wire                     axi_wready,
    output wire [127:0]             axi_wdata,
    output wire [15:0]              axi_wstrb,
    output wire                     axi_wlast,
    input  wire                     axi_bvalid,
    output wire                     axi_bready,
    input  wire [1:0]               axi_bresp,

    // ---- evidence ---------------------------------------------------------
    output reg  [31:0]              pixels_in,
    output reg  [31:0]              beats_out,
    output reg  [31:0]              bursts_out,
    // A committed write response.  `wr_ack` is the pulse, one cycle wide, and
    // `wr_acks` is the count; the pair is what `d64_done_tracker.v` consumes.
    // **A TAIL frame is 16384 of these carrying 65536 pixels**, not 65536 --
    // the beat is the unit the memory answers in.
    //
    // **`wr_ack` pulses for every response, including an error response, and
    // that is deliberate** (F03, 2026-10-07).  Suppressing the acknowledgement
    // of a failed write would not report the failure, it would convert it into
    // a tracker that waits for a response that has already come -- a hang in
    // place of a diagnosis.  A response **retires** its transaction whether or
    // not it carried an error; what must not happen is a frame carrying an
    // error being published.  So the two claims are separated here instead of
    // conflated: `wr_ack` is "the memory answered", `wr_ok` is "and the answer
    // was not an error", and `frame_fault` is the sticky verdict on the frame.
    output reg                      wr_ack,
    output reg  [31:0]              wr_acks,
    // The **successful** commits: one pulse and one count, the same shape as
    // `wr_ack`/`wr_acks` and for the same reason.  A frame whose `wr_ok` count
    // is short of its `wr_ack` count carried a failed write.
    output reg                      wr_ok,
    output reg  [31:0]              wr_oks,
    output reg  [31:0]              bresp_errors,
    // **The frame fault, latched.**  Set by any of the three ways this module
    // can learn that the frame it is writing is not the frame the tail
    // computed: a write response that is not `OKAY`, a beat that could not be
    // queued (`queue_overflows`, which is a dropped beat), and a pixel outside
    // the region (`oob_pixels`, which is a write to somebody else's memory).
    // Cleared by `rst_n` and by the first pixel of the next frame -- the arrival
    // of `out_wr_addr == 0` with `out_wr_en`, which is where the tail's walk
    // starts and the only frame boundary this interface has.  A producer that
    // never sends pixel 0 keeps the fault latched, which is the safe direction
    // for a claim about publication.
    //
    // A faulted frame is **still written**: the beats keep going to the memory
    // and the responses still retire, because stopping would leave a burst
    // half-sent and a tracker owed a response.  What the fault buys is that
    // nothing downstream may call the frame good -- `d64_done_tracker.v`'s
    // `publishable` is the state that must not rise, and `DEBUG_CATCHUP`'s
    // `DISPLAY` keeps the previous frame.
    output reg                      frame_fault,
    // Pixel indices at or above the 65536 the region holds.  The engine cannot
    // produce one -- its address is three slices of 8, 6 and 2 bits -- so this
    // is a check on the producer and it is free.
    output reg  [31:0]              oob_pixels,
    // Pixels that did not arrive in lane order.  Not an error: see the header,
    // the lane comes from the address and the shuffled arm of the gate is what
    // says so.  Reported because a producer that reorders is worth knowing
    // about even when the data is right.
    output reg  [31:0]              disorder_pixels,
    // A pixel of a different beat arriving inside an open beat.  A producer
    // contract violation, reported and not repaired.
    output reg  [31:0]              misgrouped_pixels,
    output reg  [31:0]              queue_overflows,
    output reg  [31:0]              stall_cycles,
    // ---- the depth's evidence, measured and not asserted ------------------
    // The high-water mark of the queue's occupancy: how much of `BQ_N` a bus
    // wait actually gets to use.  On the writer's own clean arm it is 1.
    output reg  [31:0]              queue_peak,
    // The bus's wait, as cycles and as a record.  One cycle of this is a cycle
    // the writer had a beat to hand over and the bus would not take it: the AW
    // of the open burst not accepted, the W beat not accepted, or the response
    // not returned.  `bus_wait_max` is the longest single run of those, and it
    // is the number `BQ_N` has to cover -- the bound in the header, measured
    // rather than claimed.
    output reg  [31:0]              bus_wait_cycles,
    output reg  [31:0]              bus_wait_max,

    // ---- the credit, the commit and the drain, named ---------------------
    // **The credit**, as the number of beats the writer may have in flight.
    // `credit_free` is what is left of it.  A beat this writer accepts is a beat
    // it had credit for; a beat it refuses is one it did not.
    output wire [31:0]              credit_beats,
    output wire [31:0]              credit_free,
    // **A successful commit is a burst whose response was `OKAY`.**  Accepting a
    // beat into the queue is not committing it -- the memory commits, and it
    // answers once per burst -- so `wr_ack`/`wr_acks` (the retire) and
    // `wr_ok`/`wr_oks` (the commit) are two claims and are counted apart above.
    //
    // **`drained` is the third and it is not either of those.**  A response
    // retires a burst and says nothing about the queue, so "everything was
    // acknowledged" can be true while a beat the writer accepted is still
    // waiting to go.  `drained` is the queue empty **and** no burst open **and**
    // no beat completing into the queue this cycle.  A parent that means "the
    // frame is finished" reads this and not a response count.
    output wire                     drained,
    // **A burst the writer refused to open because it would have crossed a 4 KiB
    // boundary, and a burst it could not give the length that was asked for.**
    // Both are counted per burst and not restated from a parameter.  A refusal
    // is not a repair: see the header for why this interface refuses a base it
    // cannot serve instead of splitting a burst at a page boundary.
    // **Cycles the writer spent refusing a burst it would have crossed a page
    // with**, and bursts it could not give the length that was asked for.  Both
    // are counted and not restated from a parameter.  A refusal is not a repair:
    // see the header for why this interface refuses a base it cannot serve
    // rather than splitting a burst at a page boundary.
    output reg  [31:0]              page_refusals,
    output reg  [31:0]              burst_clamps,
    // **The burst length this writer will actually declare**, after the page
    // clamp.  A caller that derives a response count from the geometry reads it
    // here rather than restating the clamp -- a constant retyped on both sides of
    // an interface is a constant that can be wrong on both sides (`b74c9d4`).
    output wire [15:0]               burst_beats,
    // The base this writer was given is one it can serve: page-aligned, so a
    // burst no longer than a page cannot cross one.  Reported rather than
    // assumed, because it is a property of a number in `d64_net_addr`.
    output wire                     base_aligned
);

    // -----------------------------------------------------------------------
    // Shape.  Every number here is a `localparam` and none of them is on the
    // port list, because a port list cannot see one -- the trap
    // `docs/F2_INTERFACES_2026-10-06.md` section 0 names -- so nothing on this
    // interface depends on any of them.
    // -----------------------------------------------------------------------

    // Pixels a beat, and the value of `cnt` that completes one.  The lane count
    // and the push comparison are the same expression, so a three-pixel beat is
    // a beat and not a four-pixel beat pushed early.
    localparam [1:0] LPC_LAST = PACK3_SABOTAGE ? 2'd2 : (PIX_PER_BEAT - 1);

    // -----------------------------------------------------------------------
    // **The burst's length, and the page it may not leave.**
    //
    // An AXI INCR burst may not cross a 4 KiB boundary -- the rule
    // `docs/F3_INTERFACES_2026-10-07.md` section 6.11 states for the read side
    // and `d64_burst_prefetch` and `d64_ups_u16_writer` implement.  This module
    // did not implement it.  It held only because 256 beats of 16 bytes is
    // exactly one 4 KiB page and the shipping base is page-aligned, which is a
    // coincidence of the beat size and of one number in `d64_net_addr` and not
    // a rule: move the base 512 bytes and every third burst crosses a page.
    //
    // Two clamps, and they are different things:
    //
    //   `BURST_MAX`  the *static* one.  A page holds `PAGE_BEATS` beats and
    //                `axi_awlen` is eight bits, so a requested burst longer than
    //                a page is clamped here, once, and **counted** (`burst_clamps`).
    //                Before this it was truncated silently by the port width:
    //                a caller asked for 512 beats and was given 256 with no port
    //                and no count saying so.
    // **And a base this writer cannot serve is refused, not split.**  The
    // obvious repair for a base that is not page-aligned is to cut the burst at
    // the boundary -- and it does not work, for the reason the rest of this
    // header is about: the writer has **no end-of-transfer input**.  It learns a
    // frame has begun from `out_wr_addr == 0` and never learns that it has
    // ended, so a burst cut at a page boundary in the middle of a region leaves
    // the next burst declaring a full page with fewer beats behind it, the
    // response for that burst never comes, and the frame never drains.  A split
    // moves the failure; it does not remove it.  So the rule is a *refusal*, in
    // the language section 6.11 uses for the read adapter: refused and counted,
    // not silently split and not silently truncated.
    //
    // The consequence a caller must respect: **the region's base is
    // page-aligned**.  `d64_net_addr`'s TAIL base is, and this module reports it
    // on `base_aligned` so that a caller which moves it finds out from a port
    // rather than from a frame that never finishes.
    //
    // Both are expressed as the number of beats -- 1 that AXI4's `awlen` is --
    // and **the declared length and the driven length are the same wire**, so
    // the address channel and the data channel cannot come to describe two
    // different transactions.
    // -----------------------------------------------------------------------
    localparam integer BEAT_B      = 16;                  // 128 bits
    localparam integer PAGE_B      = 4096;                // the 4 KiB rule
    localparam integer PAGE_BEATS  = PAGE_B / BEAT_B;     // 256
    localparam integer BURST_REQ   = (BURST_BEATS < 1) ? 1 : BURST_BEATS;
    localparam integer BURST_MAX   = (BURST_REQ > PAGE_BEATS) ? PAGE_BEATS : BURST_REQ;
    localparam integer BURST_CLAMP = (BURST_REQ > PAGE_BEATS) ? 1 : 0;
    localparam [7:0] AWLEN    = BURST_MAX - 1;     // AXI4: beats minus one

    // A burst that starts on a page boundary and is no longer than a page
    // cannot cross one.  This is the whole of the 4 KiB rule for this writer.
    assign base_aligned = (TAIL_BASE[11:0] == 12'd0);
    assign burst_beats  = BURST_MAX[15:0];

    // The pixel index the region holds: 256 x 256.  Declared here and not below
    // the always block that uses it -- `vlog` resolves a name against what is
    // above it, which `rgbx_frame_writer.v` records having been bitten by.
    localparam [PIX_W-1:0] PIX_N = 256 * 256;

    // **The depth that is built, and it is never shallower than the bus
    // commitment.**  See the header: the writer raises `axi_awvalid` for
    // `BURST_BEATS` beats, so a buffer that cannot hold them is a buffer a bus
    // wait can empty while beats are still owed.  `BQ_DEPTH` is the caller's
    // floor; this is what is built, and it is what every count below is sized
    // from.
    // **The credit, and it is the largest of the three.**  `BQ_FLOOR_SABOTAGE`
    // removes the whole of it -- the burst's floor and the named credit both --
    // so the sabotaged build is the caller's `BQ_DEPTH` against a `BURST_MAX`
    // commitment, which is the configuration issue #109 was found in.
    localparam integer BQ_CREDIT = (BURST_MAX > CREDIT_BEATS) ? BURST_MAX : CREDIT_BEATS;
    localparam integer BQ_FLOOR  = (BQ_FLOOR_SABOTAGE != 0) ? 1 : BQ_CREDIT;
    localparam integer BQ_N     = (BQ_DEPTH < BQ_FLOOR) ? BQ_FLOOR : BQ_DEPTH;

    // The queue's pointer width.  Two bits more than the depth is enough that
    // the pointers' difference is the occupancy and never wraps past it.
    // `$clog2` of the depth, floored at 1 so that a one-beat queue still has a
    // pointer -- a zero-width part select is not a part select.
    localparam integer BQ_AW  = ($clog2(BQ_N) < 1) ? 1 : $clog2(BQ_N);
    localparam integer BQ_PW  = BQ_AW + 2;

    localparam [1:0] W_IDLE = 2'd0,
                     W_ADDR = 2'd1,
                     W_DATA = 2'd2,
                     W_RESP = 2'd3;

    // -----------------------------------------------------------------------
    // State.  Every `reg` below is written from exactly one `always` block.
    // -----------------------------------------------------------------------

    // ---- the packer's -----------------------------------------------------
    reg [127:0]     acc;                    // the beat being packed
    reg [1:0]       cnt;                    // pixels packed into it
    reg [PIX_W-1:0] beat_pix;               // its pixel 0, from its first arrival

    // **The queue is a block RAM, and it was not one.**  Mapped as its own root
    // on Efinity 2026.1.132.4.5 it came back as `Mapping into logic memory block
    // 'bq' (10240 bits) ... because read port is not synchronous` -- 7,740 LUT4
    // and 10,054 FF for a 10,240-bit queue.  The shape that makes it a memory is
    // `memory/d64_tc_ram.v`'s and is the same shape `1a4c768` (F3-n) applied to
    // the burst cache's line RAM and the same one `d64_ups_u16_writer.v` now
    // uses: `ram_style = "block"`, a synchronous read in the memory's own
    // clocked block, and the read **unconditional** -- an enable on the address
    // is what the mapper answers with logic memory.
    //
    // `bq_ra` is the address being read and `bq_q` the memory's output register;
    // both are owned by the block below and by nothing else.  The block also
    // owns the push, because a second write to the same array from the packer's
    // block would be the double-driver shape
    // `tools/check_double_drivers.py` refuses; moving the push is the whole of
    // what that costs, and the push's condition and payload are unchanged wires.
    (* ram_style = "block" *) reg [159:0] bq [0:BQ_N-1];  // {byte addr, 128-bit beat}
    reg [159:0]     bq_q;                   // the memory's own output register
    reg             bq_hz;                  // the read hit a cell being written
    reg [159:0]     bq_hd;                  // ... and the word that wrote it
    reg [BQ_PW-1:0] bq_wp, bq_rp;           // push pointer, pop pointer
    wire [159:0]    bq_out;                 // the head: the memory, or the bypass

    // ---- the bus's --------------------------------------------------------
    reg [1:0]       wst;
    reg [7:0]       b_rlen;                 // the length this burst declared
    reg [7:0]       b_wcnt;                 // W beats handed over in this burst
    reg [31:0]      bus_wait_run;           // the current run of bus_wait cycles

    // **The occupancy is a wire and not a register.**  A count written by the
    // push and by the pop is one register with two drivers, which ModelSim
    // accepts, `efx_map` refuses, and `tools/check_double_drivers.py` reports --
    // the asymmetry that cost this project two board builds.  Each pointer
    // below is owned by one block and the occupancy follows from the two.
    wire [BQ_PW-1:0] bq_cnt = bq_wp - bq_rp;

    // -----------------------------------------------------------------------
    // The pixel, taken apart.  Nothing below reads `out_wr_data` or the low
    // bits of the address except these five wires.
    // -----------------------------------------------------------------------

    // The lane shift: 0, 32, 64 or 96.  Widened to 128 bits first, so
    // `{96'b0, word} << sh` is a 128-bit shift; a 32-bit left operand would
    // shift the word out of existence and produce a 32-bit result.
    wire [6:0]   lane_sh  = {out_wr_addr[1:0], 5'b00000};
    wire [127:0] lane_val = {96'b0, out_wr_data} << lane_sh;
    wire [127:0] lane_msk = {96'b0, 32'hFFFF_FFFF} << lane_sh;
    wire [127:0] acc_nxt  = out_wr_en ? ((acc & ~lane_msk) | lane_val) : acc;

    // The beat's byte address: `TAIL_BASE + pixel_index * 4` with the pixel
    // index taken down to its beat.  A function of the pixel and not of how many
    // beats happened to come before it -- which is what makes the PACK3
    // sabotage fail on duplicate addresses rather than on a plausible sequence
    // of addresses for the wrong pixels.
    wire [31:0]  beat_addr = TAIL_BASE + {beat_pix[PIX_W-1:2], 4'b0000};

    // The head of the queue: the beat the next burst will start at.  Read for the
    // AW address and nothing else -- the *length* is `AWLEN`, decided by the
    // shape above and reported by `burst_clamps`.  It is the memory's registered
    // output and not `bq[bq_rp]`, and `bq_ra` below is what makes that the head.
    wire [31:0]  head_addr  = bq_out[159:128];
    // `PAGE_SPLIT_SABOTAGE` keeps the name it was built under: what it removes
    // is the page rule, which for this module is the refusal below.
    wire         page_ok    = PAGE_SPLIT_SABOTAGE ? 1'b1 : base_aligned;

    // The assembled beat, and the sabotaged variant of it.  At
    // `SWAP_BYTES_SABOTAGE = 0` the ternary folds to `acc_nxt` and the
    // concatenation is not built.
    wire [127:0] beat_data = SWAP_BYTES_SABOTAGE
                           ? {acc_nxt[127:16], acc_nxt[7:0], acc_nxt[15:8]}
                           : acc_nxt;

    // **The open beat's base, as a wire and not as the register.**  `beat_pix`
    // is loaded on the cycle the beat's first pixel arrives, so a comparison
    // against the register on that same cycle reads the *previous* beat's base
    // and flags the first pixel of every beat -- which is exactly what this
    // check did before it was written this way, and the gate found it
    // (`misgrouped_pixels` = 16,383: every beat but the first).
    wire [PIX_W-1:0] open_base = (cnt == 2'd0)
                               ? {out_wr_addr[PIX_W-1:2], 2'b00}
                               : beat_pix;

    wire         beat_full = out_wr_en && (cnt == LPC_LAST);
    wire         bq_empty  = (bq_cnt == 2'd0);
    wire         bq_pop    = (wst == W_DATA) && !bq_empty && axi_wready;
    // **The admission reads the occupancy as it was at the start of the cycle**,
    // so a beat that lands in the same cycle as a pop is refused even though the
    // pop frees a slot in that cycle.  The cost is bounded -- the slot is used by
    // the next beat, so the queue settles one below the credit and at most one
    // word is lost per saturation -- and it is **recorded rather than changed**:
    // an arm that separates this admission from one that credits the pop was
    // built (`tb_d64_tail_writer_page.v`, the rate-matched bus) and it did not
    // separate them, and this project does not ship a change no gate can see.
    // The measurement and its raw output are in `docs/F3_INTERFACES_2026-10-07.md`
    // section 6.18.
    // The admission, with the two readings of the occupancy.  `ADMIT_POP` is
    // the second: the cell the pop frees this cycle is a cell the push may use
    // this cycle, because the push writes `bq_wp` and the pop reads `bq_rp` and
    // those are the same cell exactly when the queue is full -- the one case the
    // old reading refuses.  A read and a write of one cell in one cycle returns
    // the **old** word, which is the word the pop is taking, so crediting the
    // pop is safe as well as permissive.
    wire         bq_full   = (bq_cnt >= BQ_N) && !(ADMIT_POP && bq_pop);
    wire         push      = beat_full && !bq_full;

    // **`drained`.**  The queue empty, no burst open, and no beat completing
    // into the queue this cycle.  "Everything was acknowledged" is a different
    // claim: a response retires a burst and says nothing about what is behind
    // it.  See the port list.
    assign credit_beats = BQ_N;
    assign credit_free  = BQ_N - bq_cnt;
    assign drained      = bq_empty && (wst == W_IDLE) && !beat_full;

    // ---- F03: what makes a frame unpublishable -----------------------------
    // Four conditions, all wires because the register they set is owned by one
    // block.  `disorder_pixels` is deliberately **not** among them: the lane
    // comes from the address, so a producer that reorders a beat still writes
    // the right frame, and the shuffled arm of `tools/run_rgbx_beat_writer.sh`
    // is what says so.
    wire oob_now      = out_wr_en && (out_wr_addr >= PIX_N);
    wire misg_now     = out_wr_en && (out_wr_addr[PIX_W-1:2] != open_base[PIX_W-1:2]);
    wire overflow_now = beat_full && bq_full;              // a beat that is lost
    wire resp_err_now = (wst == W_RESP) && axi_bvalid && (axi_bresp != 2'd0);
    // The only frame boundary this interface has: the tail's walk starts at
    // pixel 0 of row 0.
    wire new_frame    = out_wr_en && (out_wr_addr == {PIX_W{1'b0}});

    // -----------------------------------------------------------------------
    // The packer.  One `always` block owns the beat under assembly and both
    // queue pointers, so no register here has two drivers.  The queue's array
    // is the memory block's, below, for the same reason.
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixels_in         <= 32'd0;
            beats_out         <= 32'd0;
            queue_overflows   <= 32'd0;
            disorder_pixels   <= 32'd0;
            misgrouped_pixels <= 32'd0;
            oob_pixels        <= 32'd0;
            cnt               <= 2'd0;
            beat_pix          <= {PIX_W{1'b0}};
            acc               <= 128'd0;
            bq_wp             <= {BQ_PW{1'b0}};
            bq_rp             <= {BQ_PW{1'b0}};
            queue_peak        <= 32'd0;
        end
        else begin
            if (out_wr_en) begin
                pixels_in <= pixels_in + 32'd1;

                // The beat this pixel belongs to, from the pixel's own index.
                if (cnt == 2'd0)
                    beat_pix <= {out_wr_addr[PIX_W-1:2], 2'b00};

                // Evidence, in the order a producer can be wrong: outside the
                // region, inside an open beat that is not its own, not in lane
                // order.
                if (out_wr_addr >= PIX_N)
                    oob_pixels <= oob_pixels + 32'd1;
                if (out_wr_addr[PIX_W-1:2] != open_base[PIX_W-1:2])
                    misgrouped_pixels <= misgrouped_pixels + 32'd1;
                if (out_wr_addr[1:0] != cnt)
                    disorder_pixels <= disorder_pixels + 32'd1;

                // The beat under assembly, with this pixel in it.  Cleared on
                // the completing pixel: the next beat writes all four lanes
                // before it is pushed, and a stale lane is a lane nobody would
                // ever see -- which is exactly the kind of lane that leaks.
                acc <= (cnt == LPC_LAST) ? 128'd0 : acc_nxt;

                if (cnt == LPC_LAST) cnt <= 2'd0;
                else                 cnt <= cnt + 2'd1;
            end

            if (push) begin
                // **The array itself is written by the memory block below**, so
                // that no array has two writers.  The pointer and the count are
                // this block's, as they are the packer's own events.
                bq_wp     <= bq_wp + 1'b1;
                beats_out <= beats_out + 32'd1;
            end
            else if (beat_full && bq_full) begin
                // The engine cannot be asked to wait, so this is the number
                // that says the buffer was too small for the memory it met.
                queue_overflows <= queue_overflows + 32'd1;
            end

            if (bq_pop) bq_rp <= bq_rp + 1'b1;

            // The occupancy's high-water mark.  `bq_cnt` is the wire the push
            // and the pop produce, so this reads the depth that was actually
            // used and not a number restated from the parameter.
            if (bq_cnt > queue_peak) queue_peak <= bq_cnt;
        end
    end

    // -----------------------------------------------------------------------
    // The queue's memory, its read address and its output
    // -----------------------------------------------------------------------
    // The queue is a **block RAM** now, so its read is synchronous: `bq_q` holds
    // `bq[bq_ra]` as it stood on the previous edge.  Everything the bus uses --
    // the W payload and the AW address -- comes off `bq_out`, and `bq_ra` is
    // what makes the word the memory holds the word `bq_rp` names.
    //
    // **`bq_ra` is a wire and not a register, and that is the whole of it.**
    // `bq_q(T) = bq[bq_ra(T-1)]`, so the address on the wire in cycle T is the
    // one that has to be right in cycle T+1 -- and the pointer that is right in
    // cycle T+1 is `bq_rp` **plus the pop that happens in cycle T**:
    //
    //     bq_ra(T) = bq_rp(T) + bq_pop(T)      bq_q(T+1) = bq[bq_rp(T+1)]
    //
    // A *registered* `bq_ra` was built first and is one cycle short for exactly
    // the reason `d64_ups_u16_writer.v` gives at length: a register cannot be
    // asked what next cycle's pointer will be.  The wire can, because the pop
    // is this cycle's own event.
    //
    // In `W_IDLE`, `W_ADDR` and `W_RESP` nothing pops, so `bq_ra` is the head
    // and stays the head while the pointer does -- which is what lets `W_IDLE`
    // open the burst at `head_addr` and lets `W_DATA` hand over the beat with no
    // bubble while the bus is stalled.
    //
    // **The hazard, and why the bypass is needed.**  A push writes `bq_wp` and
    // the read is at `bq_ra`, and the two are the same cell whenever
    // `bq_wp - bq_ra` is a multiple of `BQ_N` -- which happens at the two
    // occupancies the queue actually runs at:
    //
    //   * the queue is **empty** and a beat is pushed: `bq_rp == bq_wp` and
    //     nothing pops, so `bq_ra == bq_wp`.  This is the first beat of a frame
    //     and the beat after any drain.
    //   * the queue holds **exactly one** beat, that beat pops and a beat is
    //     pushed: `bq_ra = bq_rp + 1` and `bq_wp = bq_rp + 1`.  This is the
    //     steady state of a bus rate-matched to the engine, and it is the
    //     configuration `d64_tail_output_chain` ships.
    //
    // A read and a write of one cell in one cycle returns the **old** word --
    // the RAM's own semantics, and the same in one `always` block or two -- so
    // without the bypass the writer would hand the bus a stale beat and open the
    // next burst at a stale address.  `bq_hz`/`bq_hd` are that word and its
    // flag, one cycle behind, and `bq_out` selects them **after** the memory's
    // output register.  That position is the point: a select in *front* of a
    // memory's output register is the shape `1a4c768` names as the one an
    // inference pattern cannot match.
    // The tooth (`RDPIPE_SABOTAGE`): the pop is not counted, so the address is
    // the head as it stands *now* rather than as it will stand next cycle, and
    // every beat handed over is its own predecessor.
    wire [BQ_PW-1:0] bq_ra = bq_rp
                           + ((bq_pop && (RDPIPE_SABOTAGE == 0)) ? 1'b1 : 1'b0);
    wire        bq_hazard = push && (bq_wp[BQ_AW-1:0] == bq_ra[BQ_AW-1:0]);

    always @(posedge clk)
        // Unconditional, and in the memory's own clocked block: this is the
        // read the mapper needs to see as synchronous.  No reset here -- the
        // push and the read are one clocked block the way `memory/d64_tc_ram.v`
        // writes them, and neither register needs one: `bq_hz` is cleared
        // below, and `bq_q` is only read while `bq_hz` or the queue is
        // non-empty.
        begin
            if (push) bq[bq_wp[BQ_AW-1:0]] <= {beat_addr, beat_data};
            bq_q  <= bq[bq_ra[BQ_AW-1:0]];
            bq_hd <= {beat_addr, beat_data};
        end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) bq_hz <= 1'b0;
        else        bq_hz <= bq_hazard;

    // The select, **after** the memory's output register.  `BYPASS_SABOTAGE`
    // removes it and leaves the memory's own answer, which is the stale one.
    assign bq_out = (bq_hz && (BYPASS_SABOTAGE == 0)) ? bq_hd : bq_q;

    // -----------------------------------------------------------------------
    // The bus.  One burst at a time, one `always` block, and the payload off
    // the queue head -- which is now the memory's **registered** read plus the
    // bypass, `bq_out`, and not a combinational index into `bq`.  The register
    // matters and is not the bug `rgbx_frame_writer`'s header records: there the
    // *handshake* was a cycle out of step with a register on `wdata`/`wlast`, so
    // the previous beat was shipped and WLAST moved.  Here the handshake is
    // unchanged and `bq_ra` is chosen so that the word the memory holds is the
    // word `bq_rp` names -- and `wlast` is still `b_wcnt`'s own comparison.
    // -----------------------------------------------------------------------

    assign axi_awsize  = 3'd4;              // 16 bytes: the whole 128-bit bus
    assign axi_awburst = 2'b01;             // INCR
    assign axi_wstrb   = 16'hFFFF;          // every byte of every beat is real
    assign axi_wvalid  = (wst == W_DATA) && !bq_empty;
    assign axi_wdata   = bq_out[127:0];
    assign axi_wlast   = (b_wcnt == b_rlen);
    assign axi_bready  = (wst == W_RESP);

    // **The bus's wait, as a wire.**  A cycle in which the writer had a beat to
    // hand over and the bus would not take it, on any of the three channels a
    // write needs.  This is the whole input to `BQ_N`: see the header.  It is
    // deliberately *not* `stall_cycles`, which counts the opposite condition --
    // an open burst with nothing to send -- and is the engine's number.
    wire         aw_wait  = axi_awvalid && !axi_awready;
    wire         w_wait   = axi_wvalid  && !axi_wready;
    wire         b_wait   = (wst == W_RESP) && !axi_bvalid;
    wire         bus_wait = aw_wait | w_wait | b_wait;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wst          <= W_IDLE;
            axi_awvalid  <= 1'b0;
            axi_awaddr   <= 32'd0;
            axi_awlen    <= 8'd0;
            b_rlen       <= 8'd0;
            b_wcnt       <= 8'd0;
            bursts_out   <= 32'd0;
            page_refusals <= 32'd0;
            burst_clamps  <= 32'd0;
            wr_ack       <= 1'b0;
            wr_acks      <= 32'd0;
            wr_ok        <= 1'b0;
            wr_oks       <= 32'd0;
            frame_fault  <= 1'b0;
            bresp_errors <= 32'd0;
            stall_cycles <= 32'd0;
            bus_wait_cycles <= 32'd0;
            bus_wait_max    <= 32'd0;
            bus_wait_run    <= 32'd0;
        end
        else begin
            // One cycle wide, and it goes away unless a response lands.
            wr_ack <= 1'b0;
            wr_ok  <= 1'b0;

            // The frame's verdict, in this block and this block only -- one
            // register, one driver, which is the property
            // `tools/check_double_drivers.py` exists for.  The clear is
            // written first so that a fault landing in the same cycle a new
            // frame starts is a fault and not a memory of one.
            // The bus's wait, as a count and as a record.  The record is
            // taken on every cycle of the run rather than only at its end, so
            // the number is the longest wait the frame actually met.
            if (bus_wait) begin
                bus_wait_cycles <= bus_wait_cycles + 32'd1;
                bus_wait_run    <= bus_wait_run + 32'd1;
                if ((bus_wait_run + 32'd1) > bus_wait_max)
                    bus_wait_max <= bus_wait_run + 32'd1;
            end
            else begin
                bus_wait_run <= 32'd0;
            end

            if (new_frame) frame_fault <= 1'b0;
            if (oob_now | misg_now | overflow_now | resp_err_now)
                frame_fault <= 1'b1;

            case (wst)
                // **Not until there is a beat to write.**  `d64_trunk_row_ddr.v`
                // learned this on the board: an AW presented with nothing behind
                // it took the shared write channel and held it for the whole
                // run, and the first capture was all zeros with no transaction
                // issued.
                // **Nothing is opened that would cross a page.**  The base
                // decides it, once, and a base this writer cannot serve is
                // refused here and counted rather than crossed.  The length is
                // `AWLEN`: a page holds `PAGE_BEATS` beats, `BURST_MAX` is never
                // more than that, and `axi_awlen` is eight bits -- so the
                // declared length is inside a page by construction, and the
                // declared and the driven length are one number (`b_rlen`).
                // **One `W_IDLE:` item and not two.**  A case with a duplicated
                // item compiles, and `vlog` keeps the first and drops the rest,
                // so the refusal below was written, compiled, and never executed
                // -- `page_refusals` read zero on the arm built for it.  The
                // `if/else` is the fix and the reason it is written out.
                W_IDLE: if (!bq_empty
                            && (!RESERVE_WRITE_BURST || bq_cnt >= BURST_MAX)) begin
                    if (page_ok) begin
                        axi_awaddr  <= head_addr;
                        axi_awlen   <= AWLEN;
                        b_rlen      <= AWLEN;
                        axi_awvalid <= 1'b1;
                        b_wcnt      <= 8'd0;
                        if (BURST_CLAMP != 0)
                            burst_clamps <= burst_clamps + 32'd1;
                        wst         <= W_ADDR;
                    end
                    // **The refusal, and it is a fault and not a quiet stall.**
                    // A base that is not page-aligned cannot be served by this
                    // interface, so the frame is reported rather than written
                    // wrong: the count is per burst and the fault is sticky, so
                    // `drained` never rises for it.
                    else begin
                        page_refusals <= page_refusals + 32'd1;
                        frame_fault   <= 1'b1;
                    end
                end

                W_ADDR: if (axi_awvalid && axi_awready) begin
                    axi_awvalid <= 1'b0;
                    bursts_out  <= bursts_out + 32'd1;
                    wst         <= W_DATA;
                end

                W_DATA: if (!bq_empty) begin
                    if (axi_wready) begin
                        if (b_wcnt == b_rlen) wst   <= W_RESP;
                        else                  b_wcnt <= b_wcnt + 8'd1;
                    end
                end
                else begin
                    // Open burst, nothing to send: the bus waiting for the
                    // engine.  Large here, and that is the honest reading of
                    // this direction's ratio -- see the header.
                    stall_cycles <= stall_cycles + 32'd1;
                end

                W_RESP: if (axi_bvalid) begin
                    // **The response retires whatever it says.**  `wr_ack` is
                    // the retire; the verdict travels beside it (see the port
                    // list).  Suppressing this pulse on an error would leave
                    // the tracker waiting for an answer that has already come.
                    wr_acks <= wr_acks + 32'd1;
                    wr_ack  <= 1'b1;
                    if (axi_bresp != 2'd0)
                        bresp_errors <= bresp_errors + 32'd1;
                    else begin
                        wr_ok  <= 1'b1;
                        wr_oks <= wr_oks + 32'd1;
                    end
                    wst <= W_IDLE;
                end

                default: wst <= W_IDLE;
            endcase
        end
    end

endmodule
