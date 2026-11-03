`timescale 1ns/1ps

// ============================================================================
// d64_done_tracker -- what the network's final DONE actually waits for.
// ============================================================================
//
// The plan is explicit and this module is the whole of it: the final DONE waits
// for **all TAIL pixels and all DDR write responses committed**, not for the
// last PE valid cycle.  Those are two different events, and conflating them is
// how a network reports success with its last row still in flight -- the write
// was handed to the memory, the memory has not answered, and the frame is
// announced as finished anyway.
//
// The two events are two ports here, and they are kept apart all the way down:
//
//   `ack`          a DDR write response has been **committed**.  This is the
//                  thing DONE waits for.  `board/ddr3/d64_trunk_row_ddr.v:494`
//                  is the precedent: `W_RESP` retires a block at the `B`
//                  handshake, and that file's own header says a barrier built on
//                  the handover instead of the answer counted 16,383 words
//                  where 16,384 were written.
//   `last_valid`   the PE's last valid cycle.  It is an input to this module for
//                  exactly one reason: so that the gate can show, on the same
//                  build, that DONE does **not** follow it.  In a correct build
//                  it changes nothing but the `last_valids` evidence counter.
//
// 1. When DONE is raised, and once only
// -------------------------------------
// DONE is raised when the number of counted acknowledgements reaches the
// expected write count, and at no other time:
//
//     done  <=  armed AND (exp_count != 0) AND (acks == exp_count)
//
// `acks` saturates at `exp_count` and is not incremented at all once DONE is
// up, so `acks` can never exceed the target, `outstanding` can never wrap to a
// large number, and DONE cannot be raised a second time.  The target is `!= 0`
// because a zero-count frame is a mis-programmed frame: it is reported as a
// stall (below) rather than announced as a finished one.
//
// 2. An acknowledgement is a **rising edge**
// ------------------------------------------
// `ack` is counted on its rising edge and not as a level.  A response the
// parent holds for several cycles is one response, and a level-counting tracker
// would count it once per cycle and reach its target early -- which is the
// duplicate-acknowledgement failure `tools/run_done_tracker.sh` builds a tooth
// for.  This is the same rule `board/ddr3/d64_axi_read_burst.v` applies to
// `rd_req`, and for the same reason: the parent's handshake is an event, and a
// level makes one event into as many as the clock gives it.
//
// An acknowledgement that commits nothing -- one that arrives before the
// tracker is armed, or after the count is already met -- is **counted
// separately** on `extra_acks`, because a duplicate that is merely invisible is
// a duplicate nobody can report.
//
// 3. A stall is a number, not a hang
// ----------------------------------
// `outstanding` is `exp_count - acks`: the writes that were expected and have
// not been answered.  On a correct run it reaches zero and DONE rises.  On a
// stuck run it sits at a non-zero number, and that number is the diagnosis --
// "five writes are unanswered" and not "the frame rate is zero and no counter
// is moving".
//
// `stall_cycles` counts the cycles in which the tracker is armed, is not done,
// and **no acknowledgement arrived**; it is cleared by every acknowledgement.
// At `IDLE_TIMEOUT` the sticky `stall_timeout` is raised.  That is a **report
// and not an abort**: this module never gives up, never raises DONE because it
// timed out, and never stops counting -- the acknowledgement count is still the
// truth about what committed, and abandoning it would throw that away.  What
// the timeout buys is that a tracker nobody is feeding says so, on a port, in
// bounded time, instead of being waited on for ever.
//
// 4. The reason travels with DONE
// -------------------------------
// A DONE with no reason attached is not evidence, so the reason is its own
// port.  The encoding is written out here as well as in the `localparam`s
// below, because a port list cannot see a `localparam` declared in the module
// body -- the trap section 0 of `docs/F3_INTERFACES_2026-10-07.md` names.
//
//     2'd0  DONE_NONE       no DONE has been raised
//     2'd1  DONE_COUNT      the acknowledgement count was met and nothing is
//                           outstanding.  The only reason a correct build
//                           ever carries.
//     2'd2  DONE_LASTVALID  DONE was raised by the PE's last valid cycle and
//                           not by the count.  Reachable only with
//                           `SAB_DONE_ON_LAST_VALID`, so in a board build it is
//                           a reason that cannot occur -- which is what makes a
//                           board capture carrying it an unambiguous defect.
//     2'd3  DONE_STALL      **reserved, and this module never raises it.**  A
//                           DONE raised by a timeout is precisely the false
//                           success this module exists to prevent; the stall is
//                           reported on `stall_timeout`, beside the number, and
//                           deliberately not as a DONE.
//
// 5. F03: the arrival claim and the publish claim are two states
// -------------------------------------------------------------
// The root-project review of 2026-10-07 injected `BRESP = SLVERR` into the
// writer this module counts and measured `bresp_errors = 1` **and** `done = 1`
// with `reason = COUNT`.  Both are true and the pair is the defect: a DONE read
// as "the frame may be displayed" was published over a frame whose writes had
// failed.  The plan's own words are section 6.8's -- "the engine stopped", "the
// responses arrived" and "the frame may be displayed" are three separate
// states -- and this module was carrying two of them on one wire.
//
// So it carries them on two:
//
//   `done`         every expected response has retired.  Unchanged, and still
//                  the only thing the acknowledgement count drives.  An error
//                  response still retires its transaction, and a tracker that
//                  refused to count it would wait for an answer that had
//                  already come -- a hang in place of a diagnosis.
//   `publishable`  the count is met **and** no fault is standing.  Rises once
//                  per arm and is cleared by `start`, the same rule as `done`.
//                  A faulted frame ends with `done` up and `publishable` down.
//   `fault_seen`   the latched fault, so that "the frame faulted" and "the
//                  frame never finished" are different readings of a low
//                  `publishable` rather than one.
//
// The fault is an input (`fault`) and not something this module derives: it
// counts acknowledgements, and a response that carried an error looks exactly
// like one that did not from here.  Who knows is the parent -- the writer that
// saw the `BRESP`, the counter that saw a dropped beat -- and this module's job
// is to couple that knowledge to the claim.
//
// What is deliberately absent
// ---------------------------
// No write IDs and no per-write bookkeeping.  The interface the plan settled is
// an expected count and a stream of acknowledgements, and a tracker that
// demanded more would be inventing the interface it exists to honour.  The
// consequence is stated rather than hidden: with no ID, two acknowledgements
// that arrive as two rising edges are two committed writes as far as this
// module can see, and it is the **parent's** job to present one response as one
// edge.  No clock domain crossing either: this module belongs to the DDR
// write-response domain (`clk_axi` in `d64_trunk_row_ddr.v`) and every input is
// expected to be synchronous to `clk`.
//
// The whole machine is one `always` block.  No register here is written from
// two, which is the property `tools/check_double_drivers.py` exists for and is
// not one ModelSim can see.
module d64_done_tracker #(
    // Cycles the tracker may be armed with nothing happening -- no
    // acknowledgement and no DONE -- before the stall is reported.  Must be at
    // least 1.  A parameter rather than a constant because the gate sets it
    // small; a board build may set it large.
    parameter integer IDLE_TIMEOUT = 256,

    // Verification only, both folded away at 0.
    //   SAB_DONE_ON_LAST_VALID  DONE is raised by the PE's last valid cycle
    //                           instead of by the acknowledgement count.  This
    //                           is the defect the module exists to prevent, and
    //                           `tools/run_done_tracker.sh` builds it and
    //                           requires the hold-back arm to fail.
    //   SAB_NO_TIMEOUT          the stall is never reported, so a tracker that
    //                           is owed writes waits in silence for ever --
    //                           exactly the hang `outstanding` exists to make
    //                           visible.  The gate builds it and requires the
    //                           timeout arm to fail.
    parameter integer SAB_DONE_ON_LAST_VALID = 0,
    parameter integer SAB_NO_TIMEOUT         = 0
) (
    input  wire          clk,
    input  wire          rst_n,

    // ---- arming -------------------------------------------------------------
    // `start` is taken on its rising edge: it latches `exp_count`, clears the
    // count, clears DONE and clears the stall monitor.  A `start` held high is
    // one arm and not one per cycle -- the same rule as `ack`, for the same
    // reason.  An acknowledgement presented in the same cycle as `start` is
    // dropped: arming wins, and the parent is expected not to do it.
    input  wire          start,
    input  wire [31:0]   exp_count,   // writes expected; a TAIL frame is 65536

    // ---- the two events that must not be conflated --------------------------
    input  wire          ack,         // a DDR write response has committed
    input  wire          last_valid,  // the PE's last valid cycle -- not a commit

    // ---- F03, 2026-10-07: the third event -----------------------------------
    // A **frame fault** from the parent: a write response that carried an error,
    // a beat that was dropped, a write outside the region -- whatever the
    // parent knows that makes this frame's data wrong.  It is a level and it is
    // latched here; the parent's own latch may clear it between frames, which is
    // why this module latches it too (a fault that is high on the cycle the
    // count is met must not be missed).
    //
    // **An unconnected `fault` reads `x`, and this module then refuses to
    // publish.**  That is the safe direction -- see `publishable` below -- and
    // it is stated rather than left to a default.
    input  wire          fault,

    // ---- what DONE actually waits for ---------------------------------------
    output reg           done,        // sticky; rises exactly once per arm
    output reg  [1:0]    done_reason, // the encoding is in the header, 0..3
    // **The publish claim, and the reason this module has two state outputs
    // instead of one.**  `done` means "every expected response has retired";
    // `publishable` means "and nothing says the frame is wrong".  They are
    // separate states because they are separate claims, and the root-project
    // review of 2026-10-07 measured the cost of conflating them: with
    // `BRESP = SLVERR` injected, `done` rose with `reason = COUNT` over a frame
    // whose `bresp_errors` read 1.  A DONE that is read as "display this" makes
    // that a wrong frame on the screen; nothing here can stop a parent reading
    // the wrong output, but a parent that reads `publishable` cannot get it.
    //
    // `publishable` rises once per arm, on the same claim as `done` and one
    // condition stronger: the count is met **and** no fault has been seen.  It
    // is cleared by `start`.  A fault is never repaired here -- this module has
    // no way to know the frame became right -- so a faulted arm ends with
    // `done` up and `publishable` down, which is the whole of the fix.
    output reg           publishable,
    // The latched frame fault, as this module saw it: `publishable`'s "no"
    // said on a port, so that a run can tell "the frame faulted" from "the
    // frame never completed".
    output reg           fault_seen,
    output wire [31:0]   outstanding, // exp_count - acks; a stall, as a number
    output reg  [31:0]   acks,        // acknowledgements counted, saturating
    output reg  [31:0]   extra_acks,  // acknowledgements that committed nothing
    output reg  [31:0]   last_valids, // the PE's last valid cycles -- evidence
    output reg  [31:0]   stall_cycles,// armed and not done and no ack arrived
    output reg           stall_timeout// sticky: the stall was reported
);

    // ---- the reason DONE carries -------------------------------------------
    localparam [1:0] DONE_NONE      = 2'd0,
                     DONE_COUNT     = 2'd1,
                     DONE_LASTVALID = 2'd2,
                     DONE_STALL     = 2'd3;

    // Verilog-2001 has no bit-select on an integer parameter and the
    // comparisons need the width; `d64_axi_read_burst.v` does the same for its
    // watchdog.
    localparam [31:0] TMO = IDLE_TIMEOUT;

    // ---- state -------------------------------------------------------------
    // **Declared before the assignments that read them.**  A signal used above
    // its declaration is an implicit one-bit net in this flow -- a trap this
    // project has already paid for.
    reg        armed_q;
    reg  [31:0] exp_q;
    reg        start_q;   // for `start`'s rising edge
    reg        ack_q;     // for `ack`'s rising edge
    reg        lv_q;      // for `last_valid`'s rising edge

    // ---- the two edges -----------------------------------------------------
    wire start_rise = start & ~start_q;
    wire ack_rise   = ack   & ~ack_q;
    wire lv_rise    = last_valid & ~lv_q;

    // An acknowledgement that actually commits a write: the tracker is armed,
    // DONE is not yet up, and something is still owed.  Everything else is an
    // `extra_acks`.
    wire ack_counted = armed_q && !done && ack_rise && (acks != exp_q);

    // The count is met -- and `exp_count != 0`, because a zero-count frame is a
    // mis-programmed frame and must be reported, not announced.
    wire count_met = armed_q && (exp_q != 32'd0) && (acks == exp_q);

    // The event DONE is wired to, and the one place the sabotage lives.
    wire done_event = (SAB_DONE_ON_LAST_VALID != 0) ? (armed_q && lv_rise)
                                                     : count_met;

    // Writes expected and not yet answered.  `acks` saturates at `exp_q`, so
    // this is never negative and never wraps.
    assign outstanding = exp_q - acks;

    // The publish claim's two conditions.  `fault` is read **only inside an
    // `if` and inside `&&`**, never as data, so the reading of an undriven
    // `fault` is Verilog's own: an ambiguous value takes the else branch.  A
    // testbench that does not connect `fault` therefore gets `fault_seen` clear
    // and `publishable` unraised -- fail closed, which is the right direction
    // for the module whose job is to say "this may be displayed".
    wire pub_event = count_met && !fault_seen && !fault;

    // Cycles with nothing happening.  `done` and `!armed_q` clear it, and so
    // does any acknowledgement -- the monitor measures silence, not elapsed
    // time.
    wire idle_now = armed_q && !done && !ack_rise;

    // ------------------------------------------------------------------------
    // the tracker
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            armed_q <= 1'b0; exp_q <= 32'd0;
            start_q <= 1'b0; ack_q <= 1'b0; lv_q <= 1'b0;
            done <= 1'b0; done_reason <= DONE_NONE;
            publishable <= 1'b0; fault_seen <= 1'b0;
            acks <= 32'd0; extra_acks <= 32'd0; last_valids <= 32'd0;
            stall_cycles <= 32'd0; stall_timeout <= 1'b0;
        end
        else begin
            start_q <= start;
            ack_q   <= ack;
            lv_q    <= last_valid;

            if (start_rise) begin
                // Arm.  Everything a previous arm left behind is cleared: the
                // count, the verdict, and the stall monitor.  `last_valids` is
                // deliberately **not** cleared -- it is evidence about the PE,
                // and a second arm does not un-happen the first PE's last
                // valid cycle.
                armed_q       <= 1'b1;
                exp_q         <= exp_count;
                acks          <= 32'd0;
                extra_acks    <= 32'd0;
                done          <= 1'b0;
                done_reason   <= DONE_NONE;
                // A fresh arm is a clean slate for the publish claim too: the
                // fault belongs to the frame, and a new frame is a new frame.
                publishable   <= 1'b0;
                fault_seen    <= 1'b0;
                stall_cycles  <= 32'd0;
                stall_timeout <= 1'b0;
            end
            else begin
                // ---- the count --------------------------------------------------
                if (ack_counted)
                    acks <= acks + 32'd1;

                if (ack_rise && !ack_counted)
                    extra_acks <= extra_acks + 32'd1;

                if (lv_rise)
                    last_valids <= last_valids + 32'd1;

                // ---- DONE -------------------------------------------------------
                if (!done && done_event) begin
                    done        <= 1'b1;
                    done_reason <= (SAB_DONE_ON_LAST_VALID != 0) ? DONE_LASTVALID
                                                                 : DONE_COUNT;
                end

                // ---- the frame fault, and the publish claim --------------------
                // F03.  A fault is latched and never repaired -- this module
                // cannot know that the frame became right, and a claim that can
                // rise again after falling is not a claim.  `publishable` is
                // set once, on the count with no fault seen and none standing,
                // and the two can therefore disagree, which is the point:
                // `done` over a frame fault is "every response arrived", and
                // `publishable` down over it is "and this frame is not to be
                // displayed".
                if (fault) fault_seen <= 1'b1;

                if (!publishable && pub_event) publishable <= 1'b1;

                // ---- the stall monitor ------------------------------------------
                if (!idle_now)
                    stall_cycles <= 32'd0;
                else if (stall_cycles != TMO)
                    stall_cycles <= stall_cycles + 32'd1;

                if ((SAB_NO_TIMEOUT == 0) && idle_now && (stall_cycles >= TMO))
                    stall_timeout <= 1'b1;
            end
        end
    end

    // `DONE_STALL` is never assigned.  It is named because the encoding is the
    // interface and an encoding with a hole in it is one a reader has to guess
    // at; see section 4 of the header for why the hole is the point.

endmodule
