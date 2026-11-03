`timescale 1ns/1ps

// ============================================================================
// The coordinated reset of the AXI domain, and the order it has to happen in
// ============================================================================
//
// F3 6.3 and 6.12 both name this piece and neither has it: `rd_fatal` -- the
// sticky "this port is OUT OF SERVICE" that `board/ddr3/d64_axi_read_burst.v`
// raises when an address channel never accepts or a beat stream stops with the
// transaction open -- asks for "a coordinated reset of the whole AXI domain",
// and `control/d64_burst_prefetch.v` says the same in its own header.  A single
// client cannot close another block's transaction, so the recovery is the
// parent's, and until now there was no parent and no reset.
//
// What exists already, and why it is not this
// -------------------------------------------
//
// `memory/d64_reset_sync.v` is one **asynchronous-assert, synchronous-release**
// synchroniser per domain, and every module that owns two domains instantiates
// one per domain off a **common** `arst_n` (`control/d64_encoder_ddr.v:113`,
// `control/d64_trunk_blocks.v:295`, the two board tops).  That is the right
// shape for power-on and it already gets the assertion right: both chains clear
// from the same falling `arst_n`, so both domains enter reset together.  What it
// cannot do is **re-assert** -- a reset synchroniser has no request input, and
// the only way to ask it for a reset is to drop `arst_n` for the whole chip.
//
// So this module is that request, plus the ordering.  It is small on purpose:
// two flops a domain and a chain.
//
// The required ordering, and why
// ------------------------------
//
// **1. The assertion must be simultaneous.**  `axi_arst_n` and `pe_arst_n` are
// both combinational functions of the same asynchronously-set demand, so they
// fall in the same instant with no reference to either clock.  This is not
// fastidiousness; it is the one thing `memory/d64_async_fifo.v` says it cannot
// survive, at line 30 of its header:
//
//     "Both sides reset together.  Resetting one domain while the other runs is
//      not supported: the pointers would have to be re-agreed across a crossing
//      that is by definition not listening, and no amount of synchroniser stages
//      makes that safe."
//
// Staggering the *assertion* -- adding a synchroniser to it, the obvious
// mistake, because that is exactly what the release path wants -- clears one
// domain's pointers while the other's still describe the transaction that was
// in flight.  The read side then believes it has words, reads the RAM at a
// pointer the writer has already abandoned, and **delivers a word from the
// transaction that was cancelled**.  That is the same shape as F3 6.12: data
// that is wrong and a transfer that looks correct.  `SAB_STAGGER_ASSERT` is this
// mistake, and `tools/run_axi_reset_ctl.sh` requires it to fail **on data**.
//
// **2. The release is staggered, and the AXI domain goes first.**  With the AXI
// domain out of reset and idle, nothing a producer can do is lost; with the
// producer out of reset first, every request it makes in the window lands on a
// consumer that is not listening.  A **level** request channel recovers from
// that by itself -- the synchroniser sees the level when it wakes up -- but a
// request that is a **pulse** does not, and a pulse is a legal shape here:
// `d64_axi_read_burst`'s `rd_req` is consumed on its rising edge
// (`docs/CACHE_ABORT_2026-10-07.md`: "a level held across a failure is a request
// that is never made again"), and the cache's one-cycle re-arm is a pulse.
// Releasing the consumer first removes the question for every protocol instead
// of for the protocols someone remembered to check.
//
// The stagger is `PE_HOLD` `clk_pe` cycles **after `axi_arst_n` has risen**, not
// after the request: the producer is held out of reset until the bus it will
// talk to has been up and idle for that long.  That is the whole of it.
//
// The synchroniser depth, and why it is that number
// -------------------------------------------------
//
// **`SYNC_STAGES` = 2, and there is one of them, on `pe_up`.**  The only signal
// that crosses is the release level, and it is a **single bit with no payload
// beside it**, so the depth here is a metastability question and not a skew one:
// two flops is the standard minimum that makes the unresolved-window probability
// negligible at these clocks, and it is the same number `memory/d64_reset_sync.v`
// uses.  A third stage would buy nothing, because nothing downstream of the
// release depends on *which* cycle inside a metastable window the release
// landed -- the producer's first cycle of work is defined either way, and the
// `PE_HOLD` cycles of margin are what absorbs the synchroniser's uncertainty.
// A wider number would mean a payload crossing, and there is none.
//
// The demand itself is deliberately **not** synchronised: it is the assertion,
// and it has to reach both domains at once.  `req` may be a pulse or a level
// from either domain; a pulse of any width asserts the reset, because the
// assertion is asynchronous.
//
// What this module is not
// -----------------------
//
// It is not a watchdog and it does not decide *when* a reset is needed -- it
// does not read `rd_fatal` or `fatal` for you.  It sequences a request it is
// given.
//
// **It is wired now, and the joint arm is where.**  No module in this tree
// instantiated it, so `fatal -> reset -> recovery` was unproven; the arm is
// `tb_d64_axi_read_burst_recover`, run by `tools/run_axi_reset_ctl.sh` beside
// the sequencing arms, and it puts the **real cache** and the **real adapter**
// in the loop: the port raises `rd_fatal` by itself, `req` follows it, the
// domain is reset and the walk afterwards is compared word for word against the
// memory.
//
// **And the requester owes the request's *withdrawal* -- that is the finding.**
// The release runs on `req_n = arst_n & ~req`, in both domain blocks, so a
// domain cannot come back while the request that took it down is still
// standing.  Three shapes were built and measured against the real pair:
//
//   a **one-cycle pulse**, set on the fault's rising edge: it recovers, and the
//     reset is as wide as the request plus `SYNC_STAGES` cycles.  This is the
//     shape to wire.
//   the **fatal level itself** driven onto `req`: it recovers -- the assertion
//     is asynchronous -- but the request is **0 ns wide**, because the level
//     being asked with is the level this reset clears.  Measured, in that arm's
//     `request:` line, and the whole of the reset's width is then the chain's.
//   a latch **held until the sequence has finished**: it **deadlocks**.  `req`
//     high holds both resets, the sequence never finishes, and `req` is never
//     cleared.  This is the latch a reader would write first, which is why the
//     arm exists; a board-level root must count its resets or time the request
//     out rather than wait for the sequence it is holding.
module d64_axi_reset_ctl #(
    // Release synchroniser depth, per domain.  **2 is the value and the reason
    // is in the header**: a single-bit level with no payload, so this is
    // metastability and not skew.  Must be >= 2 for the shift chain below.
    parameter integer SYNC_STAGES = 2,
    // `clk_pe` cycles the producer domain is held past the AXI domain's release.
    // Must be >= 2.  This is the stagger and it is the point of the module.
    parameter integer PE_HOLD = 8,
    // ---- verification only, folded away at 0 -------------------------------
    // 1: the assertion is staggered -- the producer's reset goes through a
    //    four-deep synchroniser.  The obvious mistake, and the one the async
    //    FIFO's header says it cannot survive.  The gate must fail **on data**.
    parameter integer SAB_STAGGER_ASSERT = 0,
    // 1: the release order is reversed -- the producer domain is released on
    //    its own and the AXI domain is released a full `PE_HOLD` after it.
    //    The gate must fail **on the order**.
    parameter integer SAB_RESET_ORDER = 0
) (
    input  wire clk_pe,
    input  wire clk_axi,

    // The board's asynchronous reset, active low.  Asserting it is exactly a
    // permanent request; nothing here overrides it.
    input  wire arst_n,
    // The request: a pulse or a level, from either domain.  Asserted
    // asynchronously, so the width does not matter and neither does the clock
    // it came from.
    input  wire req,

    // The two domain resets.  Asynchronous assert, synchronous release.
    output wire pe_arst_n,      // the producer domain -- released LAST
    output wire axi_arst_n,     // the AXI domain      -- released FIRST

    // Readable state, one per domain and not mixed: `pe_busy` outlasts
    // `axi_busy` by construction, and a caller that wants "the sequence is
    // done" wants `pe_busy`.  A caller that wants "the bus is idle" wants
    // `axi_busy`, which is also the -"am I still holding the producers off"-
    // answer.  Neither is a combinational mix of the two domains.
    output wire pe_busy,
    output wire axi_busy
);

    // ------------------------------------------------------------------
    // the demand: one asynchronous function of the request, in both domains
    // ------------------------------------------------------------------
    //
    // **`req_n` is the assertion and it is not synchronised.**  Both domain
    // resets are derived from it in the same delta, which is the property
    // requirement 1 needs and the only way to get it.
    wire req_n = arst_n & ~req;

    // ------------------------------------------------------------------
    // SAB_STAGGER_ASSERT: a synchroniser in the assert path
    // ------------------------------------------------------------------
    //
    // The mistake this models is a real one and an easy one: the release path
    // wants a synchroniser, so a synchroniser is put on the reset and the
    // assertion inherits it.  `stg` has no asynchronous reset on purpose -- the
    // point of the sabotage is that the producer's reset *is* a synchronised
    // `req_n` and therefore asserts four `clk_pe` cycles late.
    reg [3:0] stg;
    always @(posedge clk_pe)
        stg <= {stg[2:0], req_n};

    wire req_n_pe = SAB_STAGGER_ASSERT ? stg[3] : req_n;

    // ------------------------------------------------------------------
    // the AXI domain: released first, after `SYNC_STAGES` of its own clock
    // ------------------------------------------------------------------
    //
    // No synchroniser on this release: it is a chain in the domain that is
    // being released, so there is nothing to synchronise against.
    reg axi_hold;
    // **Declared here and not in the producer block below**, because the AXI
    // block reads it: a signal used before its declaration is an implicit
    // one-bit net in this flow, which is a trap this project has already paid
    // for.
    reg pe_hold;
    reg [SYNC_STAGES-1:0] axi_chain;
    reg [SYNC_STAGES-1:0] pe_up_s;      // the producer's release, seen from clk_axi

    // The reversed order's release condition.  At 0 this is a constant 1 and
    // the AXI domain releases on its own chain, which is the design.
    wire axi_go = (SAB_RESET_ORDER != 0) ? pe_up_s[SYNC_STAGES-1] : 1'b1;

    always @(posedge clk_axi or negedge req_n)
        if (!req_n) begin
            axi_hold  <= 1'b1;
            axi_chain <= {SYNC_STAGES{1'b0}};
        end
        else begin
            axi_chain <= {axi_chain[SYNC_STAGES-2:0], axi_go};
            if (&axi_chain) axi_hold <= 1'b0;
        end

    always @(posedge clk_axi or negedge req_n)
        if (!req_n) pe_up_s <= {SYNC_STAGES{1'b0}};
        else        pe_up_s <= {pe_up_s[SYNC_STAGES-2:0], ~pe_hold};

    // ------------------------------------------------------------------
    // the producer domain: released last, and only after the bus is up
    // ------------------------------------------------------------------
    //
    // `axi_up_s` is the synchroniser: the AXI domain's release level crossing
    // into `clk_pe`, `SYNC_STAGES` deep.  **The `pe_chain` that follows is not a
    // synchroniser** -- it is the margin, in `clk_pe` cycles, between "the bus
    // has been out of reset for a while" and "the producers may run".  Reading
    // those two things as one register would make the stagger an accident of
    // the clock ratio, which is how a margin turns into a race.
    reg [SYNC_STAGES-1:0] axi_up_s;
    reg [PE_HOLD-1:0]     pe_chain;

    wire pe_go = (SAB_RESET_ORDER != 0) ? req_n : axi_up_s[SYNC_STAGES-1];

    always @(posedge clk_pe or negedge req_n_pe)
        if (!req_n_pe) begin
            pe_hold  <= 1'b1;
            pe_chain <= {PE_HOLD{1'b0}};
        end
        else begin
            pe_chain <= {pe_chain[PE_HOLD-2:0], pe_go};
            if (&pe_chain) pe_hold <= 1'b0;
        end

    always @(posedge clk_pe or negedge req_n_pe)
        if (!req_n_pe) axi_up_s <= {SYNC_STAGES{1'b0}};
        else           axi_up_s <= {axi_up_s[SYNC_STAGES-2:0], ~axi_hold};

    // ------------------------------------------------------------------
    // the resets, and they assert together
    // ------------------------------------------------------------------
    assign axi_arst_n = arst_n & ~axi_hold;
    assign pe_arst_n  = arst_n & ~pe_hold;

    assign axi_busy = axi_hold;
    assign pe_busy  = pe_hold;

endmodule
