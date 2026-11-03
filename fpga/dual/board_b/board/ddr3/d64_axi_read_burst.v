`timescale 1ns/1ps

// ============================================================================
// d64_axi_read_burst -- the cache's abstract DDR read port, answered from AXI.
// ============================================================================
//
// `d64_burst_prefetch` does not speak AXI and must not: its business is which
// line to fetch, not how a memory transaction is shaped.  What it presents is
// an abstract read port --
//
//     rd_req, rd_addr[31:0], rd_len[15:0]   (beats MINUS ONE, as an `arlen`)
//       -> rd_ack, then rd_data[127:0] with rd_valid, ending at rd_last
//
// -- and the board's DDR is raw AXI.  This module is the adapter, and the shape
// of the adapter is fixed first by what it is **not**:
//
//   * it is not a repacker.  A beat is 128 bits on both sides -- eight sixteen
//     bit containers, which `d64_burst_prefetch` unpacks straight into its
//     eighty-bit store word -- so one R beat in is one beat out and there is no
//     bit to move.  `d64_h0_block.v:17` says the same of the HEAD layer's
//     reader and `:400` implements it; that layer is board-verified.
//   * it is not a scheduler.  One burst at a time: a request is accepted, run to
//     its end, and the port is quiet until the next.  AXI without ID
//     interleaving has the same rule, and the cache, which fills one line at a
//     time, asks for one burst at a time anyway.
//
// What it *is* is the four things the abstract port does not say.
//
// 1. The address unit
// -------------------
// `rd_addr` addresses a **beat**; AXI addresses a **byte**.  `arsize = 3'd4` is
// sixteen bytes -- the rest of this design's transfer size, `d64_video_row_ddr.v:276`
// -- so the conversion is `araddr = rd_addr << BEAT_LSB` with `BEAT_LSB = 4`,
// and the low four bits are zero by construction, which is what AXI requires of
// an address for a 16-byte transfer.  The shift is written as a concatenation so
// that the top four bits are dropped deliberately and not by the width of an
// expression.
//
// 2. The length
// -------------
// `rd_len` is sixteen bits and AXI4's `arlen` is eight, so a request is issuable
// only up to 256 beats.  The cache's burst is 256 words and its `rd_len` is 255,
// which fits exactly.  A longer request is **refused and counted**, never
// truncated: a truncated burst is a correct-looking transfer of the wrong
// length, and nothing downstream could tell.
//
// 3. What a memory that answers wrongly looks like
// ------------------------------------------------
// The abstract port has no error and no abort, and this module does not invent
// one -- an interface agreed after the fact is an interface that does not exist.
// Every fault is reported on this module's own evidence outputs (`axi_errors`,
// the sticky `rd_err`) and the burst is **ended**; the three faults are the
// three `d64_video_row_ddr.v:348` names, and they are the same three:
//
//   bad `RRESP`     the slave answered, and the answer was an error.  AXI
//                   allows `SLVERR` on one beat with good beats after it, so
//                   this does **not** end the burst -- the beat is delivered
//                   like any other, and the verdict is sticky across the rest
//                   of it.  **It does end the burst's *cleanliness***, which is
//                   a different thing and was missing until 2026-10-07: a burst
//                   that carried an error response closes with `rd_abort` and
//                   not with `rd_last`, because AXI does not define the data
//                   beside an error response and a caller that installed the
//                   transfer would be holding an undefined word behind a valid
//                   address.  See section 5.
//   early `RLAST`   the slave ended the burst before the beat the request asked
//                   for.  `RLAST` **is** the end of the transaction -- it is
//                   what releases the arbiter's ownership of the read channel
//                   (`d64_axi_arb2.v`: `if (s_rvalid && s_rready && s_rlast)
//                   r_busy <= 1'b0`) -- so the burst does close here, and what
//                   this check owes the caller is that `rd_last` is **not**
//                   raised: the beats that came are delivered and the burst is
//                   counted faulty, because a short burst reported as a
//                   finished transfer is the failure this check exists to
//                   prevent, and it is not detectable one beat later because
//                   there is no later beat.
//   missing `RLAST` the requested last beat arrived with no end marker.  The
//                   count is complete and the transaction is **not** closed, so
//                   the burst stays open -- see section 4 -- the fault is
//                   counted, and `rd_last` is not raised.
//
// It is the **agreement** between the count the request asked for and the
// `RLAST` the slave sent that makes a burst clean, and that agreement is the
// whole of the short- and long-burst checks:
//
//     clean  <=>  (the beat is the requested last one) AND `RLAST`
//
// 4. A timeout is not a cancellation -- F02, 2026-10-07
// ----------------------------------------------------
// A rendezvous that never happens used to be treated as if the transaction had
// been called off.  It had not.  With the real adapter and the real arbiter
// (`d64_axi_arb2.v`), the root-project review of 2026-10-07 measured both
// halves of the cost:
//
//   * after this module returned to `IDLE`, the **arbiter was still busy** --
//     it releases read ownership on `RLAST` and on nothing else -- so the next
//     legal request could never be granted;
//   * and with `ARVALID` withdrawn before `ARREADY`, the arbiter's held grant
//     (`r_ar_held`) kept presenting the **stale address** on `s_ar*`, and the
//     DDR side then accepted the old AR against a request this module had
//     already forgotten.
//
// So the two rendezvous have two different answers, and neither of them is
// "forget it":
//
//   the address is owed   once `ARVALID` is asserted it is **held**, because
//                         AXI says VALID stands until READY and because a
//                         withdrawn VALID is a grant the arbiter cannot
//                         release.  The watchdog still fires and still
//                         reports (`timeouts`, `rd_timeout`, `rd_abort`) --
//                         and then the port waits for the handshake it is
//                         owed.  When the late accept comes, the burst it
//                         starts is **quarantined**: `RREADY` is held, every
//                         beat is discarded and counted on `late_beats`, and
//                         nothing is delivered, because the caller was told
//                         this request failed and a beat arriving after that
//                         is stale by construction.  `RLAST` then closes the
//                         read transaction and the arbiter's ownership with
//                         it.  This is the recovery, and it is a recovery and
//                         not a refusal: the port is back in service and the
//                         next request is served in full.
//
//   the beat stream is owed  a burst whose count is complete and whose
//                         `RLAST` is missing is **not** ended on the count.
//                         It stays open: `RREADY` stays high, further beats
//                         are discarded (they are past the length that was
//                         asked for) and counted on `late_beats`, and the
//                         burst closes when the slave closes it.  `rd_last` is
//                         not raised and `rd_abort` says the fill is not
//                         usable.
//
// **What a watchdog expiry does not do is pretend.**  Nothing here returns the
// port to "idle" while a transaction is still live behind it.  Where the bus
// cannot be brought back -- the address channel that never accepts, the beat
// stream that stops with the transaction open -- the port says so: the sticky
// `rd_fatal` is raised and **every further request is refused** until `rst_n`.
// A single client cannot close another block's transaction, so the recovery is
// the parent's: a coordinated reset of the whole AXI domain.  What `rd_fatal`
// buys is that the parent is told, in bounded time, instead of a port that
// reports itself idle and then fails every request that follows.
//
// **And a request presented while the port is draining is not consumed.**
// The caller re-arms a failed request as a one-cycle edge three cycles after
// the abort, and the state that raised the abort is not always `IDLE` by then:
// `S_ARQ` is still waiting for an address channel that timed out, and `S_DRN`
// and `S_DRQ` are draining a transaction nobody will receive.  An edge
// presented there was invisible -- this module looked for a rise only in
// `S_IDLE` -- so a burst was never issued for it, and the caller, which had
// been told its request was *taken*, waited for a fill that could not come.
// One bit carries the edge across the drain and `S_IDLE` spends it; the
// alternative -- the caller re-arming on a window instead of an edge -- is a
// change to the pair's contract rather than to this module.  `S_FATAL` spends
// nothing: a port that is out of service refuses, and it says so on
// `rd_fatal`.
//
// `rd_timeout` and `rd_fatal` are deliberately different claims.  A timeout is
// "a rendezvous took longer than the board allows" and is reportable while the
// port is still working; `rd_fatal` is "this port is out of service".  The
// first is a number a run can watch, the second is a state a run must act on.
//
// 5. What the caller is told, and the one new obligation
// ------------------------------------------------------
// `rd_last` still means exactly one thing: the burst closed cleanly, every
// requested beat was delivered, and the fill is complete.  **Every other
// ending raises `rd_abort`** for one cycle -- the refusals, the timeouts, the
// short burst, the `RLAST`-missing drain, the **error response**, and the
// fatal -- so that a caller
// which has to decide whether to install the line it was filling has one
// signal to read and not a set of cases to reproduce.  The cache's fill must
// be invalidated on `rd_abort`; it is the same statement `rd_last`'s absence
// makes, said once, in the caller's language.
//
// `rd_req` is taken on its **rising edge** and not as a level, and it stays
// that way.  A level would make every request that was refused or timed out an
// unbounded retry behind the caller's back -- the caller holds `rd_req` until
// `rd_ack`, and after a timeout that `rd_ack` is never coming.
//
// 6. A burst may not cross a 4 KiB boundary -- F08, 2026-10-07
// ----------------------------------------------------------
// AXI4 forbids an INCR burst that crosses a 4 KiB boundary, and the reason is
// the shape of a slave's decode: 4 KiB is the smallest region a slave is
// required to own as a unit, so a burst that spans two of them asks one slave
// for bytes it was never given.  A real interconnect may answer that by
// splitting the burst, by returning an error, or -- the case that matters here
// -- by answering it, which puts bytes from the wrong page in a fill whose
// transfer looks correct from end to end.  The cache computes a fill's length
// from the line it is filling, so a wrong length is not a hypothesis about a
// future bug; it is the shape this adapter has already been handed once.  This
// check is the same class as section 2's, and it was missing.
//
// A request whose **span** crosses a 4 KiB boundary is therefore refused before
// anything reaches the bus, counted on `page_refusals`, and does not move: the
// state machine stays in `S_IDLE`, `ARVALID` is never raised, and nothing is
// split and nothing is truncated.  The span is the request's first byte
// **inside its page** plus the bytes of the whole burst --
//
//     araddr[11:0] + ((rd_len + 1) << BEAT_LSB)  >  4096
//
// -- and the comparison is `>` and not `>=`, which is the whole of the
// off-by-one in both directions.  A request that ends **exactly on** the
// boundary is legal, because its last byte is byte 4095 of the page, and it is
// accepted; one beat past that is refused.  `tools/run_axi_page_guard.sh` arms
// both sides of that line from a **single start address**: `rd_len` 15 at
// `araddr` 0x0F00 is accepted and `rd_len` 16 from the same address is refused,
// and the gate also carries the root-project review's own case -- `araddr`
// 0x0FF0 with two beats, `page_end` 4112 -- which is the request the review
// measured the adapter accepting.
//
// The length rule of section 2 is checked first, so a request that is both too
// long for `arlen` and crossing a boundary is refused as too long and counted
// there.  The two counters say which rule fired: `axi_errors` moves for every
// fault this module sees, both refusals included; `page_refusals` moves for the
// 4 KiB rule alone.
//
// The refusal is an ending, and section 5 already says what an ending that is
// not a clean `rd_last` is worth: `rd_err` sticky, `rd_abort` for one cycle,
// and no `rd_ack`.  **What it leaves behind is that the port is idle** -- which
// is the second thing a refusal can get wrong, so it is checked rather than
// assumed.  `S_IDLE` was never left, `ARVALID` is low, `RREADY` follows
// `axi_rvalid` and is therefore low on a quiet bus, and
// `tools/run_axi_page_guard.sh` requires a later request to be served **in
// full** rather than merely requiring a number to be printed.  What the caller
// is left holding is `rd_req` with no `rd_ack` -- the same standing obligation
// section 5 names for the timeout, and the reason `rd_abort` is the signal to
// read.  A caller that keeps `rd_req` high will not be served twice: the edge is
// consumed, deliberately, exactly as the too-long refusal has consumed it since
// it was written.

// What is deliberately absent
// ---------------------------
// No address/control pipeline, no outstanding-burst count, no `RID`/`BID`
// checking: with one burst in flight there is nothing to match against.  No
// data buffering either -- `RREADY` is held high for the whole read phase,
// because the abstract port has no backpressure to express and inventing one
// would be inventing the interface this module exists to honour.  The beat is
// presented on the cycle **after** the R handshake, so nothing combinational
// crosses from the bus into the cache.
//
// The whole machine is one `always` block: no register here is written from
// two, which is the property `tools/check_double_drivers.py` exists for and is
// not one ModelSim can see.
//
// What is verified, and what is not
// ---------------------------------
// `tools/run_axi_read_burst.sh` (unmodified, its own arms and its own numbers),
// `tools/run_axi_page_guard.sh` (F08's guard: both sides of the boundary, and
// the length rule beside it as the control) and `tools/run_axi_error_drain.sh`
// (the join with the real arbiter, the real writer and the real tracker).
// **Not verified:** anything on the board.  No AXI protocol checker has been
// run over this module -- the rules it is written
// to are stated in the comments above and checked by the arms and not by a
// checker -- and the `late_response` hazard of section 4 in the window between
// a new AR being issued and the old transaction's last beat is stated and not
// closed.
module d64_axi_read_burst #(
    // Bits of AXI byte address below one beat.  `arsize = 3'd4` is sixteen
    // bytes and 16 == 1 << BEAT_LSB, so these two must agree; they are stated
    // apart only because `arsize` is a wire and a port list cannot see a
    // `localparam` (the trap `docs/F2_INTERFACES_2026-10-06.md` section 0
    // names).
    parameter integer BEAT_LSB = 4,

    // Cycles of no progress before a rendezvous is **reported**.  A parameter
    // rather than a constant because the gate sets it small; a board build may
    // set it large.  It is the same window for both channels and for a drain,
    // because it is one question asked in four places: has the bus stopped?
    parameter integer TIMEOUT  = 4096,

    // Verification only, all folded away at 0.
    //   SAB_NO_TIMEOUT   the watchdog never fires, so a stalled burst waits
    //                    for ever.  `tools/run_axi_read_burst.sh` builds it and
    //                    requires the stall arm to fail.
    //   SAB_TRUST_RLAST  the slave's `RLAST` is taken as the end of the burst
    //                    and the beat count is not checked, so a short burst is
    //                    indistinguishable from a clean one.  The gate builds it
    //                    and requires the short-burst arm to fail.
    //   SAB_END_ON_COUNT **the pre-F02 rule, restored verbatim.**
    //                    `tools/run_axi_error_drain.sh` builds it and requires
    //                    the `MISSING_LAST` arm to fail.  What it restores is
    //                    the line this module carried until 2026-10-07:
    //
    //                        wire r_end = r_hs & (axi_rlast | r_final);
    //
    //                    -- the count alone ending the burst, with no RLAST
    //                    behind it, so the port returned to `IDLE` while the
    //                    arbiter was still busy and the next request timed out.
    //   SAB_DROP_ARVALID **the pre-F02 rule on the other channel, verbatim.**
    //                    The same script builds it and requires the `AR_CANCEL`
    //                    arm to fail.  What it restores is the timeout arm that
    //                    withdrew the address:
    //
    //                        axi_arvalid <= 1'b0;  ...  st <= S_IDLE;
    //
    //                    -- VALID dropping with no READY, which the arbiter
    //                    answers by keeping its held grant and presenting the
    //                    stale address, and which the DDR side then accepted.
    parameter integer SAB_NO_TIMEOUT   = 0,
    parameter integer SAB_TRUST_RLAST  = 0,
    parameter integer SAB_END_ON_COUNT = 0,
    parameter integer SAB_DROP_ARVALID = 0,
    //   SAB_NO_PAGE_GUARD **the pre-F08 rule, restored**: the 4 KiB span check
    //                    is removed, so a request that crosses a boundary is
    //                    issued like any other and the memory answers it -- the
    //                    failure the guard exists to prevent, which surfaces as
    //                    a completed transfer of the wrong bytes.
    //                    `tools/run_axi_page_guard.sh` builds it and requires
    //                    the crossing arm to fail.
    parameter integer SAB_NO_PAGE_GUARD = 0,
    //   SAB_TRUST_RRESP  **the pre-fix reading of an error response,
    //                    restored**: a bad `RRESP` is counted and nothing else,
    //                    so the burst ends with a clean `rd_last` and the line
    //                    it was filling installs.  The two paths look identical
    //                    from this module's outputs except for this one signal,
    //                    which is why the arm that removes it is run **against
    //                    the real cache** in `tools/run_cache_abort.sh` and
    //                    required to fail **on data**: the poison the faulted
    //                    burst delivered becomes a line and the next read hits
    //                    it.
    parameter integer SAB_TRUST_RRESP  = 0,
    //   SAB_DROP_PEND    **the pre-fix reading of the drain window, restored**:
    //                    a request presented while the port is draining is
    //                    consumed and no burst is ever issued for it.  The
    //                    caller waits for ever, so
    //                    `tools/run_cache_abort.sh`'s mode 6 builds it and
    //                    requires the walk **not to finish**.
    parameter integer SAB_DROP_PEND    = 0
) (
    input  wire          clk,
    input  wire          rst_n,

    // ---- the cache's abstract DDR read port --------------------------------
    // `d64_burst_prefetch`'s, port for port.  `rd_req` is held with `rd_addr`
    // and `rd_len` until `rd_ack`; the beats follow, one `rd_valid` a beat,
    // ending at exactly one `rd_last` on beat `rd_len + 1`.
    input  wire          rd_req,
    input  wire [31:0]   rd_addr,
    input  wire [15:0]   rd_len,
    output reg           rd_ack,
    output reg  [127:0]  rd_data,
    output reg           rd_valid,
    output reg           rd_last,

    // ---- raw AXI -----------------------------------------------------------
    output reg           axi_arvalid,
    input  wire          axi_arready,
    output reg  [31:0]   axi_araddr,
    output wire [7:0]    axi_arlen,
    output wire [2:0]    axi_arsize,
    output wire [1:0]    axi_arburst,
    input  wire          axi_rvalid,
    output reg           axi_rready,
    input  wire [127:0]  axi_rdata,
    input  wire          axi_rlast,
    input  wire [1:0]    axi_rresp,

    // ---- evidence ----------------------------------------------------------
    output reg  [31:0]   bursts,       // AR handshakes
    output reg  [31:0]   beats,        // R beats delivered to the cache
    output reg  [31:0]   axi_errors,   // bad RRESP, short burst, long burst, or
                                       // a request too long to issue
    output reg  [31:0]   timeouts,     // bursts abandoned by the watchdog
    output reg           rd_err,       // sticky: a fault has been seen
    output reg           rd_timeout,   // sticky: a burst has been abandoned

    // F02, 2026-10-07.  The two claims a timeout used to conflate.
    output reg           rd_abort,     // pulse: the burst did not close cleanly,
                                       // so the cache's current fill must be
                                       // invalidated.  **Every ending that is
                                       // not a clean `rd_last` raises this.**
    output reg           rd_fatal,     // sticky: this port is OUT OF SERVICE and
                                       // every further request is refused until
                                       // `rst_n`.  The parent's recovery is a
                                       // coordinated reset of the AXI domain.
    output reg  [31:0]   late_beats,   // beats the cache was NOT given: the
                                       // quarantine of section 4.  A number and
                                       // not a hang -- a late return that is
                                       // large is a memory answering something
                                       // nobody is waiting for.

    // F08, 2026-10-07.  How many requests were turned away because the burst
    // they asked for would have crossed a 4 KiB boundary (section 6).  A number
    // and not a hang, for the same reason `late_beats` is one: a wrong length
    // a caller computes is a thing that happens, and what this port owes the
    // run is the count of how often it happened.
    output reg  [31:0]   page_refusals
);

    // AXI's constants, and they are the tree's: `d64_video_row_ddr.v:276` is
    // the same two assignments.
    localparam [2:0] ARSIZE  = 3'd4;   // 16 bytes a beat == 1 << BEAT_LSB
    localparam [1:0] ARBURST = 2'b01;  // INCR

    localparam [2:0] S_IDLE  = 3'd0,   // nothing owed
                     S_AR    = 3'd1,   // AR on the bus, not taken yet
                     S_R     = 3'd2,   // the beat stream
                     S_ARQ   = 3'd3,   // the AR watchdog fired: VALID is HELD
                     S_DRN   = 3'd4,   // the count without RLAST: drain to RLAST
                     S_DRQ   = 3'd5,   // a late AR was accepted: drain the dead
                                       // burst, deliver nothing
                     S_FATAL = 3'd6;   // out of service until rst_n

    // Verilog-2001 has no bit-select on an integer parameter, and the
    // comparisons need the width.  `d64_video_row_ddr.v:266` does the same.
    localparam [31:0] TMO_MAX = TIMEOUT;

    // `rd_addr` is a beat address and `araddr` is a byte address.  Written as a
    // concatenation so the four bits that fall off the top fall off on purpose.
    wire [31:0] ar_byte = {rd_addr[31-BEAT_LSB:0], {BEAT_LSB{1'b0}}};

    // A request AXI cannot name.  Refused rather than truncated: see the
    // header, section 2.
    wire ar_too_long = |rd_len[15:8];

    // F08.  A request whose span crosses a 4 KiB boundary -- refused rather
    // than issued, for the same reason and in the same place: see the header,
    // section 6.  `ar_byte[11:0]` is the request's first byte inside the page
    // it starts in; the second term is the whole burst.  The comparison is `>`
    // and not `>=`: a span that ends exactly on byte 4096 is legal, and that
    // one byte is the whole of the off-by-one in both directions.
    wire [31:0] burst_bytes = ({16'd0, rd_len} + 32'd1) << BEAT_LSB;
    wire [31:0] page_span   = {20'd0, ar_byte[11:0]} + burst_bytes;
    wire        ar_cross4k  = (page_span > 32'd4096) & (SAB_NO_PAGE_GUARD == 0);

    // **Declared before the assignments that read them.**  A signal used above
    // its declaration is an implicit one-bit net in this flow -- a trap this
    // project has already paid for, and the reason `d64_video_row_ddr.v`
    // declares its display-domain crossing registers at the top of the AXI
    // block rather than beside the logic that drives them.
    reg  [2:0]  st;
    reg  [7:0]  alen_q;    // `arlen`, latched with the request
    reg  [15:0] last_q;    // index of the requested last beat == `rd_len`
    reg  [15:0] bidx_q;    // index of the beat now on the R channel
    reg  [31:0] tmo_q;     // cycles without progress
    reg         rd_req_q;  // for the rising edge
    reg         bbad_q;    // this burst has a fault
    reg         rresp_seen;// this burst carried a bad `RRESP`
    reg         req_pend;  // an edge presented where this port could not serve

    assign axi_arsize  = ARSIZE;
    assign axi_arburst = ARBURST;
    assign axi_arlen   = alen_q;

    wire rd_req_rise = rd_req & ~rd_req_q;

    // **An edge must not be consumed by a state that cannot serve it.**  The
    // caller's re-arm is a rise; only `S_IDLE` can spend one, so every other
    // state latches it here and `S_IDLE` spends it.  `S_FATAL` is the
    // exception and clears it -- a port that is out of service refuses the
    // request and says so on `rd_fatal`, and `rd_req` is driven low by the
    // caller that reads it.
    wire req_ask = rd_req_rise | (req_pend & (SAB_DROP_PEND == 0));

    // The R channel's verdicts.  `r_final` is what the *request* asked for;
    // `axi_rlast` is what the *slave* says.  A burst is clean only where the
    // two agree.
    wire r_hs      = (st == S_R) & axi_rvalid;
    wire r_final   = (bidx_q == last_q);
    wire r_short   = r_hs &  axi_rlast & ~r_final;   // RLAST before the count
    wire r_long    = r_hs & ~axi_rlast &  r_final;   // the count with no RLAST
    wire r_badresp = r_hs & (axi_rresp != 2'd0);
    wire r_good    = r_hs &  axi_rlast & r_final;

    // `SAB_TRUST_RLAST` is "the slave's `RLAST` is the truth": the count is not
    // checked, so a short burst cannot be told from a clean one.  A bad `RRESP`
    // is still counted -- it is a different question and the sabotage is about
    // the count.
    wire det_count = (SAB_TRUST_RLAST == 0);
    wire r_fault   = r_badresp | (det_count & (r_short | r_long));

    // **A bad `RRESP` ends the burst's *cleanliness*.**  The beat is delivered
    // and the burst runs to its own end -- that is what keeps the arbiter's
    // release and the drain rules of section 4 intact -- and what it may not do
    // is close as a transfer the caller can install.  AXI leaves the data
    // beside an error response undefined, so a fill built from the burst would
    // hold an undefined word behind a valid address: it is the short burst's
    // failure, and the same remedy.  `rresp_seen` carries the fault from the
    // beat it arrived on to the beat that ends the burst, which is where the
    // verdict is taken.
    wire rresp_fault = (rresp_seen | r_badresp) & (SAB_TRUST_RRESP == 0);

    // ---- the end of a burst, and the pre-F02 end of a burst ----------------
    // Section 4.  `RLAST` closes the transaction; the count does not, because
    // the count is this module's own bookkeeping and the arbiter is released
    // by the slave's marker and by nothing else.
    wire r_end = (SAB_END_ON_COUNT != 0) ? (r_hs & (axi_rlast | r_final))
                                         : (r_hs &  axi_rlast);

    // What `rd_last` would carry on this beat, and therefore what this burst's
    // ending is worth to the caller: anything short of this raises `rd_abort`.
    // and a bad `RRESP` anywhere in the burst takes it away.  `rd_last`
    // carries this same value -- see `S_R` -- so `rd_last` and `rd_abort` are
    // complements by construction and not by two expressions kept in step by
    // hand.
    wire close_ok = ((SAB_TRUST_RLAST != 0) ? axi_rlast : r_good)
                    & ~rresp_fault;

    // The watchdog ticks in exactly the cycles the burst is waiting on the
    // memory.  On the address channel any progress clears it; in the beat
    // stream and in a **drain** an accepted beat clears it too -- a beat is the
    // bus working, and a drain of a long burst would otherwise be declared dead
    // halfway through it.  What the drain is really waiting for is `RLAST`, so
    // the window it measures is **silence**: once the slave's own burst ends --
    // and AXI bounds one burst at 256 beats -- the last beat is the last
    // progress there will be, and the watchdog then measures the gap that
    // follows it.
    wire wait_ar  = (st == S_AR) | (st == S_ARQ);
    wire wait_r   = (st == S_R)  | (st == S_DRN) | (st == S_DRQ);
    wire prog_ar  = wait_ar & axi_arready;
    wire prog_r   = (st == S_R) | (st == S_DRN) | (st == S_DRQ);
    wire prog     = prog_ar | (prog_r & axi_rvalid);
    wire tmo_tick = (wait_ar | wait_r) & ~prog;
    wire tmo_exp  = (SAB_NO_TIMEOUT == 0) & (tmo_q == (TMO_MAX - 32'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            axi_arvalid <= 1'b0;
            axi_araddr  <= 32'd0;
            axi_rready  <= 1'b0;
            rd_ack      <= 1'b0;
            rd_data     <= 128'd0;
            rd_valid    <= 1'b0;
            rd_last     <= 1'b0;
            alen_q      <= 8'd0;
            last_q      <= 16'd0;
            bidx_q      <= 16'd0;
            tmo_q       <= 32'd0;
            rd_req_q    <= 1'b0;
            req_pend    <= 1'b0;
            bbad_q      <= 1'b0;
            rresp_seen  <= 1'b0;
            bursts      <= 32'd0;
            beats       <= 32'd0;
            axi_errors  <= 32'd0;
            timeouts    <= 32'd0;
            rd_err      <= 1'b0;
            rd_timeout  <= 1'b0;
            rd_abort    <= 1'b0;
            rd_fatal    <= 1'b0;
            late_beats  <= 32'd0;
            page_refusals <= 32'd0;
        end
        else begin
            // The abstract side's two handshakes are one cycle wide.
            rd_ack   <= 1'b0;
            rd_valid <= 1'b0;
            rd_last  <= 1'b0;
            rd_abort <= 1'b0;
            rd_req_q <= rd_req;
            // The latch, and the one thing that clears it besides `S_IDLE`: a
            // request the caller has taken back is not served.  It is a promise
            // to spend the edge, not a licence to hold a request nobody wants.
            if (rd_req_rise) req_pend <= 1'b1;
            if (!rd_req)     req_pend <= 1'b0;

            // One counter, armed in the states that owe the bus something,
            // cleared by progress and by leaving them.  The case below may
            // override it -- and each state that overrides a timeout **clears**
            // it, because the counter sits at its ceiling in exactly the cycle
            // the state is left and would otherwise fire again immediately.
            tmo_q <= tmo_tick ? (tmo_q + 32'd1) : 32'd0;

            case (st)

                // ---- nothing owed: wait for a request's rising edge --------
                // The read channel is drained here too, and that is the
                // quarantine: after a burst was abandoned, a beat belonging to
                // it can still arrive, and consuming it -- counted on
                // `late_beats` and never delivered -- is what releases the
                // arbiter's ownership instead of leaving it to be misattributed
                // to whatever runs next.
                S_IDLE: begin
                    axi_rready <= axi_rvalid;
                    if (axi_rvalid) late_beats <= late_beats + 32'd1;
                    if (req_ask) begin
                        req_pend <= 1'b0;
                        if (ar_too_long) begin
                            // Not issuable.  Counted, refused, and the edge is
                            // consumed so that a held `rd_req` cannot become a
                            // retry loop.
                            axi_errors <= axi_errors + 32'd1;
                            rd_err     <= 1'b1;
                            rd_abort   <= 1'b1;
                        end
                        else if (ar_cross4k) begin
                            // F08.  Not issuable either, and refused exactly the
                            // same way: nothing reaches the bus, the state does
                            // not move -- so the port is idle on the next cycle
                            // and stays idle -- and the edge is consumed so that
                            // a held `rd_req` cannot become a retry loop.
                            // `page_refusals` is the number that says it was
                            // this rule and not the length rule.  See the
                            // header, section 6.
                            axi_errors    <= axi_errors + 32'd1;
                            page_refusals <= page_refusals + 32'd1;
                            rd_err        <= 1'b1;
                            rd_abort      <= 1'b1;
                        end
                        else begin
                            alen_q      <= rd_len[7:0];
                            last_q      <= rd_len;
                            bidx_q      <= 16'd0;
                            bbad_q      <= 1'b0;
                            rresp_seen  <= 1'b0;
                            axi_araddr  <= ar_byte;
                            axi_arvalid <= 1'b1;
                            st          <= S_AR;
                        end
                    end
                end

                // ---- the address is on the bus; hold it until it is taken ---
                // The read channel is drained here too: no beat can belong to
                // this request before its address is taken, so anything on the
                // R channel is a late return from a burst that was abandoned,
                // and consuming it is how it closes against this module and not
                // against the burst that follows.
                S_AR: begin
                    axi_rready <= axi_rvalid;
                    if (axi_rvalid) late_beats <= late_beats + 32'd1;
                    if (prog_ar) begin
                        axi_arvalid <= 1'b0;
                        axi_rready  <= 1'b1;
                        rd_ack      <= 1'b1;
                        bursts      <= bursts + 32'd1;
                        st          <= S_R;
                    end
                    else if (tmo_exp) begin
                        tmo_q      <= 32'd0;
                        timeouts   <= timeouts + 32'd1;
                        rd_timeout <= 1'b1;
                        rd_err     <= 1'b1;
                        rd_abort   <= 1'b1;
                        if (SAB_DROP_ARVALID != 0) begin
                            // the pre-F02 line, restored: VALID withdrawn with
                            // no READY, and the port back in `IDLE`.  The
                            // arbiter's held grant outlives it.
                            axi_arvalid <= 1'b0;
                            st          <= S_IDLE;
                        end
                        else begin
                            // **`ARVALID` and the address do not move.**  AXI:
                            // VALID stands until READY.  The handshake this
                            // port is owed is still coming.
                            st <= S_ARQ;
                        end
                    end
                end

                // ---- the AR watchdog fired and the address is still owed ----
                // The request has been reported as failed; the bus has not been
                // told anything, and must not be.  This state waits for the
                // handshake, and the burst it starts is drained rather than
                // delivered (section 4).
                S_ARQ: begin
                    axi_rready <= axi_rvalid;   // a stray beat is still drained
                    if (axi_rvalid) late_beats <= late_beats + 32'd1;
                    if (prog_ar) begin
                        axi_arvalid <= 1'b0;
                        axi_rready  <= 1'b1;
                        bursts      <= bursts + 32'd1;
                        st          <= S_DRQ;
                    end
                    else if (tmo_exp) begin
                        // The address channel is dead.  There is nothing left
                        // to wait for and nothing this port can repair: only a
                        // reset of the AXI domain clears the arbiter's grant.
                        axi_arvalid <= 1'b0;
                        rd_fatal    <= 1'b1;
                        rd_abort    <= 1'b1;
                        st          <= S_FATAL;
                    end
                end

                // ---- one beat a cycle, until the count and the marker agree -
                S_R: begin
                    if (r_hs) begin
                        // `RREADY` is high for the whole read phase, so this is
                        // the beat the memory put on the bus in the cycle just
                        // ended.  One cycle of latency, and no window in which
                        // a beat could be dropped -- the abstract port has no
                        // backpressure to offer.
                        rd_data  <= axi_rdata;
                        rd_valid <= 1'b1;
                        rd_last  <= close_ok;
                        beats    <= beats + 32'd1;
                        if (r_fault) begin
                            axi_errors <= axi_errors + 32'd1;
                            bbad_q     <= 1'b1;
                        end
                        // The per-burst half of the response verdict: the
                        // fault may have arrived many beats before the one that
                        // ends the burst, and `close_ok` reads it there.
                        if (r_badresp) rresp_seen <= 1'b1;
                        if (r_end) begin
                            axi_rready <= 1'b0;
                            st         <= S_IDLE;
                            if (bbad_q | r_fault) rd_err <= 1'b1;
                            // `rd_last` carries `close_ok`; anything else is a
                            // fill the cache cannot install.
                            if (!close_ok) rd_abort <= 1'b1;
                        end
                        else begin
                            bidx_q <= bidx_q + 16'd1;
                            if ((SAB_END_ON_COUNT == 0) & r_long) begin
                                // The count is complete and the slave has not
                                // closed the burst.  The transaction is open,
                                // so the burst stays open.
                                rd_err <= 1'b1;
                                st     <= S_DRN;
                            end
                        end
                    end
                    else if (tmo_exp) begin
                        // The memory stopped answering mid-burst.  The beats
                        // that did arrive have been delivered; the rest never
                        // come, and no `rd_last` is invented for them.  The
                        // read channel is drained from here on (S_IDLE), so a
                        // late return closes against this module and not
                        // against the next request.
                        tmo_q      <= 32'd0;
                        axi_rready <= 1'b0;
                        timeouts   <= timeouts + 32'd1;
                        rd_timeout <= 1'b1;
                        rd_err     <= 1'b1;
                        rd_abort   <= 1'b1;
                        st         <= S_IDLE;
                    end
                end

                // ---- the count is in and the slave has not closed the burst --
                // Section 4.  `RLAST` is what releases ownership, so the burst
                // is kept open: `RREADY` stays high, anything further is
                // discarded -- it is past the length that was asked for -- and
                // nothing is delivered.  `rd_last` was spent on the count and
                // was not raised.
                S_DRN: begin
                    if (axi_rvalid) begin
                        late_beats <= late_beats + 32'd1;
                        if (axi_rlast) begin
                            axi_rready <= 1'b0;
                            rd_abort   <= 1'b1;   // the fill was already unusable
                            st         <= S_IDLE;
                        end
                    end
                    else if (tmo_exp) begin
                        // The slave owes `RLAST` and is not coming.  The
                        // transaction is open and this client cannot close it.
                        tmo_q      <= 32'd0;
                        axi_rready <= 1'b0;
                        timeouts   <= timeouts + 32'd1;
                        rd_timeout <= 1'b1;
                        rd_err     <= 1'b1;
                        rd_abort   <= 1'b1;
                        rd_fatal   <= 1'b1;
                        st         <= S_FATAL;
                    end
                end

                // ---- a late AR was accepted, for a request already failed ----
                // The memory will send a burst the cache asked for and must not
                // receive: every beat is discarded and counted, `RREADY` stays
                // high, and `RLAST` closes the read transaction so the arbiter
                // is released and the port is back in service.  This is the
                // recovery, and it is exercised by
                // `tools/run_axi_error_drain.sh`'s `AR_LATE` arm.
                S_DRQ: begin
                    if (axi_rvalid) begin
                        late_beats <= late_beats + 32'd1;
                        if (axi_rlast) begin
                            axi_rready <= 1'b0;
                            st         <= S_IDLE;
                        end
                    end
                    else if (tmo_exp) begin
                        tmo_q      <= 32'd0;
                        axi_rready <= 1'b0;
                        timeouts   <= timeouts + 32'd1;
                        rd_timeout <= 1'b1;
                        rd_fatal   <= 1'b1;
                        st         <= S_FATAL;
                    end
                end

                // ---- out of service ----------------------------------------
                // No request is accepted, no address is presented, and the
                // read channel is still drained -- the one thing this state can
                // do for the domain is consume a beat that finally arrives.
                // `axi_rready` follows `axi_rvalid` so that the port is quiet
                // when the bus is.
                S_FATAL: begin
                    axi_arvalid <= 1'b0;
                    axi_rready  <= axi_rvalid;
                    // Out of service: nothing is issued, and a request held
                    // across the reset is refused rather than remembered.
                    req_pend    <= 1'b0;
                    if (axi_rvalid) late_beats <= late_beats + 32'd1;
                end

                default: st <= S_IDLE;

            endcase
        end
    end

endmodule
