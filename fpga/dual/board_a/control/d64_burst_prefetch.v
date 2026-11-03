`timescale 1ns/1ps

// F2 -- the decoder's intermediate tensors fetched in bursts, read from a cache.
//
// The rule this module exists for
// ------------------------------
//
// **A tap must never become a DDR read.**  The decoder walks a 3x3 window over a
// feature map, so one intermediate word is wanted by up to nine taps of one
// output pixel and by the taps of the neighbouring output pixels as well.  A
// read per tap puts the DDR controller on the path of every MAC slot and the
// frame rate dies.  So the intermediate is fetched from DDR in bursts into an
// on-chip cache, and the engine's input store reads the cache.
//
// What "the engine's input store" is
// ----------------------------------
//
// `d64_tconv_engine` addresses its input store in WORDS -- `in_rd_addr` is
// `(pixel * IC_GROUPS) + icg`, sixteen bits -- and the store answers with
// `IC_PAR * IN_W` bits a word, **registered**: the address is on the port in one
// cycle and the word is on `in_rd_data` in the next.  This module presents
// exactly that, same names, same widths, same one-cycle latency.
//
// The one port this module adds, and why it is unavoidable
// --------------------------------------------------------
//
// `in_stall`.  The engine's store port has no ready, because a store made of RAM
// always answers; a cache can miss, and a miss has to be told to somebody.  A
// cache that cannot stall is a cache that returns the wrong word quietly -- the
// failure this module exists to prevent, one level down.  So:
//
//   the engine presents `in_rd_en` with `in_rd_addr` and holds both while
//   `in_stall` is high; the read is **transferred** in the cycle in which
//   `in_stall` is low, and the word for that address is on `in_rd_data` in the
//   **following** cycle.  `in_rd_en` drops after the transfer, as it does in any
//   ready/valid handshake.
//
// That is the whole contract.  Integration is one OR gate: this module's
// `in_stall` folds into `d64_tconv_engine`'s scheduler `stall`, which already
// holds `r`/`c`/`phase`/`tap`/`icg` and already clears `in_rd_en` while it is
// high -- a stalled scheduler re-presents the same slot, which is the hold this
// contract asks for.  `d64_tconv_engine.v` is not edited in this stage.
//
// The numbers, and why they are those numbers
// -------------------------------------------
//
// **Burst length: 256 beats**, one beat a store word.  Chosen from the region's
// shape, not from a bandwidth budget.  The decoder's intermediates are
// RES4 64x64x32, UPS0 128x128x16 and UPS1 256x256x8, and the engine's word
// address is `pixel*IC_GROUPS + icg`; the three multiply out as
//
//     64 x 4  =  128 x 2  =  256 x 1  =  **256 words in an image row**
//
// in every one of them.  So a 256-word line aligned to 256 is exactly one image
// row in all three tensors -- no per-tensor parameter, no half-row remainder.
// A burst that were not a row would either split a row across two transactions
// in one tensor and not another, or leave a tail burst per line; both make the
// transaction count stop being a property of the walk.
//
// **Cache line: 256 words -- the same number, on purpose.**  A fill is then
// exactly one DDR burst, so the transaction count is exactly `lines touched`,
// and "span / burst length" is not an approximation of the count but the count.
// `BURST_WORDS` divides `LINE_WORDS`; set smaller, a fill becomes
// `LINE_WORDS/BURST_WORDS` bursts and the count scales with it, which is the
// only sense in which the two numbers are separable.
//
// **Lines: 3.  Associativity: fully associative (3 ways).**  Three is not a
// guess, and it is set by the deeper of the two consumers of this store.
//
//   the transposed-conv decoder (`d64_tconv_scheduler`) reads a 2x2 input
//   block an input step -- source rows `r` and `r+1`, and the taps of a phase
//   all read one pixel of it -- so **two rows** are live at a time and two
//   lines hold the whole working set.
//
//   the last layer (`d64_tail_engine`) reads a 3x3 stride-1 window with
//   reflection: source rows `reflect_y(r-1)`, `r`, `reflect_y(r+1)`, and its
//   taps visit them `r-1 x3, r x3, r+1 x3`.  That is **three rows**, and a
//   two-line cache evicts the row it is about to want on every output pixel --
//   roughly three fills an output pixel, which is the DDR hammering this module
//   exists to stop.  The gate measures exactly that: with two lines the burst
//   count passes the ideal for the whole image after **3,073 of 589,824 reads**,
//   and the same two-line build passes the transposed-conv walk with the same
//   64 transactions three lines give.  The third line is bought for the 3x3
//   window and for nothing else.
//
// Four lines buy nothing either walk uses and cost 2560 bytes.  Three ways is
// also enough that on either row-major walk each row is fetched exactly once --
// the minimum, since every row is read.
//
// **Replacement: exact LRU**, a 2-bit age a line ("stack" ages: on a use the
// line goes to 3 and every line older than it shifts down one).  Exact, and
// exactness costs nothing at three ways.  Ties cannot arise while all lines are
// valid, because the ages are distinct by construction.  One rule here is not a
// policy but a correctness rule: **a fresh fill never evicts a line that an
// accepted read is reading in the same cycle** -- the victim scan skips any line
// whose tag is on a live port, so the line RAM's read side and write side are
// never the same address, and the module does not depend on which one a RAM
// would prefer.
//
// **Storage: 3 lines x 256 words x 80 bits = 61,440 bits, and it is EBR.**
// That was the one thing in this header left unverified -- "no synthesis was
// run" -- and it has since been run, the expensive way, and fixed.  Mapped with
// itself as the root on Efinity 2026.1.132.4.5 the module came back **0 RAM10,
// 49,398 LUT4 and 61,962 FF**, which is more flip-flops than the Ti60F225 has,
// with the mapper's own line:
//
//     Mapping into logic memory block 'line_mem' (61440 bits) ... because read
//     port is not synchronous. [EFX-0657]
//
// Two things were wrong and both are fixed.  There was **no `ram_style`
// attribute**, and the read was **asynchronous into the port's register with
// the bypass/violation select in front of it** -- a select ahead of a memory's
// output register is the one shape an inference pattern cannot match.  The
// array is now the shape `memory/d64_tc_ram.v` states and that the trunk's A
// and B rings map with:
//
//     (* ram_style = "block" *) reg [WORD_W-1:0] line_mem [0:MEM_N-1];
//     always @(posedge clk) begin
//         if (line_we) line_mem[line_wa] <= beat_word;
//         ram_out[pp] <= line_mem[line_ra[pp]];
//     end
//
// the read and the write in **one** clocked block, the read **unconditional**
// (an enable on the address is what the mapper answers with logic memory), and
// the select applied **after** the RAM's own output register, against the
// control of the cycle the read was accepted in.  One register follows the
// select and holds its answer, so `in_rd_data` is the word for the accepted
// read and then stays there until the next one -- which is what the register
// this replaced did, and what both benches sample.  The handshake, the line
// count, the replacement rule and the region rule are all untouched: what moved
// is the select's position and nothing about what it selects.
//
// One instance, three tensors -- the region at run time
// -----------------------------------------------------
//
// The storage is 61,440 bits and there are three intermediates to serve (RES4
// 64x64x32, UPS0 128x128x16, UPS1 256x256x8).  Three instances would be 184,320
// bits out of a RAM10 budget with about thirty free, so the tensor is a
// **run-time** input here and one instance is re-pointed between the three:
// `region_base`/`region_span` carry the two numbers the parameters above hold,
// and the parameters stay as their reset value.  A build that never presents a
// request is the build this module was before the ports existed, bit for bit
// (`tools/run_burst_prefetch.sh` is that build, unmodified).
//
// The rule that makes a re-point safe, stated before it is enforced
// ----------------------------------------------------------------
//
// The engine's word address is `pixel * IC_GROUPS + icg` -- **relative to the
// region** -- so a line of one tensor carries exactly the tag a read of the
// next one looks for, and nothing in that read distinguishes them.  A region
// that moved while a fill was in flight, or while a line of the old region was
// live, would answer the new tensor's read out of the old tensor's data, **as
// a hit**, and the only sign would be a wrong number.
//
// So a re-point is refused unless the module is quiescent **in the same cycle**
// the request is presented, and quiescence is all four of:
//
//   1. the fill machine is in `F_IDLE` -- no burst issued, none acknowledged,
//      no beat outstanding.  A fill taken across the move installs a line of
//      the old region under a new region's tag: the tag is one the new
//      region's walk will ask for, and the word behind it is the other
//      tensor's.
//   2. **no port presents `in_rd_en`, including a port held by `in_stall`.**  A
//      miss presented in the accept cycle stalls, and the fill it starts on
//      the next cycle would take its address from the new region and its tag
//      from the old -- the same failure one cycle later.
//   3. `rd_req` is low: no transaction outstanding on the DDR port.
//   4. the bypass machine is idle (`bst == B_IDLE`), which is where the
//      `BYPASS` build keeps its one outstanding beat.
//
// On acceptance **every line is invalidated in the same cycle**, and that is
// the second half of the rule, not a convenience.  A surviving line is a line
// of the old region under a tag the new region's walk will ask for, and the
// read that hits it looks exactly like a read that hits.  That the engine's
// address is region-relative is what makes this a hazard rather than a tidy
// one, and it is why the invalidation belongs to the handshake.
//
// What the requester must do: present `region_base` and `region_span` with
// `region_req` high, hold all three until `region_ack` pulses one cycle, and
// read nothing -- of either region -- between the request and the acknowledge.
// `region_base` must be a multiple of `LINE_WORDS`, exactly as the parameter
// had to be.  `region_waits` counts the cycles a request was held off, so a
// re-point asked of a busy cache is a number and not a wait; a build that
// accepts one in the cycle it arrives is a build whose guard has stopped
// guarding, and the counter is where that shows.
//
// `SAB_REPOINT` is the sabotaged build of the two halves of that rule,
// verification only and folded away at 0: 1 removes the quiescence test and 2
// removes the invalidation.  **Both fail on data and neither fails on a
// count** -- a tooth that only moved a counter would mean the guard was
// decoration -- and `tools/run_burst_prefetch_shared.sh` requires both to fire.
//
// The two reports from the adapter, and what this module does with them
// ---------------------------------------------------------------------
//
// `board/ddr3/d64_axi_read_burst.v` gained two reports on 2026-10-07
// (`docs/AXI_ERROR_DRAIN_2026-10-07.md`; section 6.3 and item 6.12 of
// `docs/F3_INTERFACES_2026-10-07.md`): **`rd_abort`**, a pulse meaning the burst
// did not close cleanly, and **`rd_fatal`**, sticky, meaning the port is out of
// service until `rst_n`.  Until this change this module read neither, and that
// is the **third** instance in this project of one shape -- *a signal that is
// correct for the cycle it is looked at and wrong for the cycle it is not*.  The
// arbiter's `bank_idle` margin and this module's own re-point rule are the other
// two.
//
// **`rd_abort` ends the fill, and no line is installed.**  The fill machine
// returns to `F_IDLE` where it stands and `rd_req` drops.  `valid_r[f_line]` --
// which a fill sets at one place and one place only, on a clean `rd_last` -- is
// not set.  The words the abandoned burst did deliver are not written to
// `line_mem` either: an abandoned transaction's data behind an invalid tag is
// unreachable, and leaving it in the RAM is a fact about a transaction this
// module has already given up on.  `aborts` counts the pulse.
//
// `rd_req` drops for **one cycle** and the fill is re-issued from `F_IDLE`,
// because `d64_axi_read_burst` takes a request on its **rising edge** -- a level
// held across a failure is a request that is never made again.  One cycle of low
// is a fresh rising edge, and the read the engine is still presenting (held by
// `in_stall`, which is what makes the one-cycle re-arm safe) is served from the
// refilled line.  That is the whole of "the next read is correct": not "no fault
// was reported" but a word compared against the memory.
//
// **The one-cycle re-arm has a window, and it is closed on the port's side.**
// The re-arm is a rising edge, and the adapter's **draining** states (`S_ARQ`,
// `S_DRN`, `S_DRQ`) could not act on an edge: they tracked `rd_req` with
// `rd_req_q` and acted on it only in `S_IDLE`, so a rise presented while the
// port was draining was **consumed and no burst was ever issued for it** --
// the fill waited for ever.  It is reachable from an **address-channel**
// rendezvous that times out and then completes
// (`S_AR -> S_ARQ -> S_DRQ -> S_IDLE`); it is *not* reachable from a mid-burst
// data timeout, which ends in `S_IDLE` in the same cycle it raises `rd_abort`,
// and that is the shape section 6.12 names and the shape the arms here were
// built on.  **This module is unchanged by the fix and needs to be**: the edge
// is the pair's contract, and the party that could not honour it was the port.
// `board/ddr3/d64_axi_read_burst.v` now latches an edge it cannot serve in one
// bit (`req_pend`) and spends it in `S_IDLE`; `tools/run_cache_abort.sh`'s mode
// 6 is the arm, and it is a **hang** before the fix and a clean walk after it.
//
// **`rd_fatal` stops the cache.**  `fatal` latches it, and `in_stall` stands
// high in **every** cycle from then on, so no read is ever transferred again;
// the fill machine returns to `F_IDLE`, no burst is issued, and `rd_req` is
// driven low.  Nothing clears it but `rst_n` -- not a later good burst, not a
// re-point.  `aborts` and `fatal` are the two readable states, and a cycle that
// raises `rd_abort` and `rd_fatal` together is counted as fatal and not as an
// abort.
//
// What a caller must do with `fatal`
// ----------------------------------
//
// **This module cannot recover and will not pretend to.**  A single client
// cannot close another block's transaction: the arbiter that granted this port
// the read channel releases it on `RLAST` and on nothing else, and a port that
// is out of service is not going to produce that `RLAST`.  A reset of this cache
// alone would leave the arbiter holding a grant for a port that no longer
// exists.  So:
//
//   1. **Read `fatal`.**  It is sticky and it is an output, so a parent samples
//      it once an epoch; `aborts` here, and the adapter's `timeouts`,
//      `rd_timeout`, `late_beats` and `axi_errors`, are what say which fault it
//      was.  This is the only thing this module can honestly offer.
//   2. **Stop presenting reads, and treat the frame in flight as dead.**  They
//      will never be served -- `in_stall` is high for ever -- so a producer that
//      keeps asking stalls for ever, and a frame that was being computed when
//      the port died must **not** reach a publish claim.  That claim is
//      `d64_done_tracker`'s `publishable` and not this module's, and nothing
//      here raises it or lowers it.
//   3. **Reset the domain, not this module.**  This cache, the adapter and the
//      arbiter together, from one reset, in one epoch, with the producers held
//      off across it.
//
// **That reset does not exist yet.**  A coordinated reset of the AXI domain is a
// root-level piece -- section 6.3 names it as one and it is written nowhere in
// this tree -- and this module does not invent one in order to give `fatal`
// somewhere to go.  Exposing the state and saying so is the honest half of the
// pair, and it is the half that is here.
//
// `SAB_ABORT` is the tooth, verification only and folded away at 0.  **1** reads
// `rd_abort` as the end of the fill, so the words the abandoned burst delivered
// become a line under a valid tag and the engine's next read **hits** it -- a
// wrong word and not a counter.  **2** ignores the pulse entirely, which is what
// this module did before the ports existed: the fill waits in `F_DATA` for an
// `rd_last` a faulted burst never raises, and the walk does not finish.
// `tools/run_cache_abort.sh` requires 1 to fail **on data** and shows 2 as the
// pre-fix reading.
//
// The access pattern assumed, and what would break it
// ---------------------------------------------------
//
// Assumed: the reader walks the intermediate **row-major**, one row at a time,
// with a window at most three rows tall.  Then a whole-row line is never
// partially used, the live set is two or three rows, and each row is fetched
// once.  Measured on both walks (see `docs/BURST_PREFETCH_2026-10-06.md`) the
// fill count equals the theoretical minimum and not merely the ideal.
//
// What breaks the assumption, said out loud so that it is somebody's problem if
// it happens:
//   (a) a **column-major or tiled walk**, or any consumer that revisits a row
//       two rows after leaving it, would evict a live line at every step.  The
//       row size and the line size would be right and the count would still be
//       one per tap;
//   (b) a window **taller than three rows** -- 5x5, or dilated -- needs five
//       lines and three would thrash;
//   (c) an intermediate whose **row is not 256 words** -- a layer with a
//       different channel count or width -- stops being one line a row, and
//       `LINE_WORDS`/`BURST_WORDS` have to be re-chosen with the tensor.  That
//       is why the region's shape is parameters here and not constants.
//
// No prefetcher, deliberately
// ---------------------------
//
// A next-line prefetcher was written and then removed.  With three lines and a
// row-major walk the demand misses already equal the minimum, so it bought no
// transaction -- and a sequential prefetcher triggered on "this line was filled"
// walks the entire region ahead of the engine, because every fill makes the next
// line look wanted.  A prefetch that fetches the whole tensor is a worse number
// than no prefetch, and a policy that must be told when to stop is a policy with
// a state machine that can get it wrong.
//
// What is here
// ------------
//
// one read port to the engine, one burst port to DDR, a tag/valid/age array and
// a line RAM, one fill state machine (issue, accept, beat, install), and
// counters -- because "it never misses" is a claim and a counter is evidence.
//
// `BYPASS = 1` is the sabotaged build: **every** engine read becomes its own
// one-beat DDR transaction.  The words are still right; the transaction count is
// one per tap.  That is the tooth, and `tools/run_burst_prefetch.sh` requires it
// to fail the count check and to fail nothing else.
module d64_burst_prefetch #(
    // ---- what a store word is ----------------------------------------------
    // The same three numbers `d64_tconv_engine` builds its store with.  The word
    // is the engine's word, not this module's invention.
    parameter integer IC_PAR     = 8,        // channels in a word
    parameter integer IN_W       = 10,       // bits in a stored activation
    parameter integer IN_PORTS   = 1,        // engine read ports
    parameter integer IN_WORD_W  = 16,       // engine's word-address width

    // Bits in one DDR transport container.  The trunk writes the **U16 image**,
    // so a channel rides in sixteen bits and the container is what a beat is
    // built from.  **A parameter and not a localparam**: the port list needs it,
    // and a port cannot see a localparam declared in the module body -- the
    // trap `docs/F2_INTERFACES_2026-10-06.md` section 0 names.
    //
    // **`IN_W` must not exceed it.**  The unpack slices `IN_W` bits out of
    // each container, so a channel wider than its container reads into the
    // neighbour and corrupts it -- silently, because the slice is still a
    // legal slice.  `IN_W = 8` and `IN_W = 10` in a 16-bit container are the
    // two the decoder uses.
    parameter integer DDR_LANE_W = 16,

    // Verification only, folded away at 0.  Takes the **top** `IN_W` bits of
    // the container instead of the bottom ones -- the one way an unpack like
    // this goes wrong, and the reason the testbench's container is not
    // zero-extended above each channel.  `tools/run_burst_prefetch.sh` builds
    // it and requires it to fail.
    parameter integer SAB_HIGH_LANE = 0,

    // ---- the region ---------------------------------------------------------
    // **Explicit, and not derived from `d64_net_addr`.**  That module is being
    // written beside this one and neither may lean on the other, so the region
    // arrives as two numbers: where the tensor's word 0 is, and how many words
    // of it there are.  `BASE_ADDR` is a WORD address, added unchanged to the
    // line offset, so a region is addressable here only if its base is a
    // multiple of `LINE_WORDS` -- which an aligned allocation gives.
    //
    // **These two are now the reset value of a run-time region and not the
    // region itself.**  One instance serves three tensors -- see "One instance,
    // three tensors" in the header -- so the live pair is the registers
    // `cur_base`/`cur_span`, which take `region_base`/`region_span` at
    // `region_ack`.  The parameters stay so that a build which presents no
    // request never moves them, which is every build that existed before the
    // ports did.
    parameter [31:0]  BASE_ADDR  = 32'h0000_0000,
    parameter integer SPAN_WORDS = 65536,    // 256x256x1 groups == UPS1

    // ---- the shape of the walk ----------------------------------------------
    parameter integer LINE_WORDS  = 256,     // words in a line == words in a row
    parameter integer LINES       = 3,       // lines in the cache (1..4)
    parameter integer BURST_WORDS = 256,     // beats in one DDR burst

    // Verification only, folded away at 0.
    parameter integer BYPASS = 0,            // every read goes to DDR

    // Verification only, folded away at 0.  The sabotaged builds of the region
    // handshake; the rule they break is stated in the header.  **1** removes
    // the quiescence test, so a re-point asked for while a fill is in flight is
    // taken in the cycle it is asked for.  **2** removes the invalidation, so
    // the lines of the old region stay live across the move.  Both are data
    // failures, and `tools/run_burst_prefetch_shared.sh` requires both to fire.
    parameter integer SAB_REPOINT = 0,

    // Verification only, folded away at 0.  **The three readings of
    // `rd_abort`.**  0 is the guard: the fill is abandoned and nothing is
    // installed.  1 is the tooth: the pulse is read as the **end of the fill**,
    // so the words the abandoned burst delivered install under a valid tag and
    // the next read hits them -- a failure on data and not on a count, which is
    // what makes the guard load-bearing rather than decorative.  2 ignores the
    // pulse, which is the pre-fix module: the fill waits for an `rd_last` that
    // never comes.  The header states all three;
    // `tools/run_cache_abort.sh` requires 1 to fail on data and shows 2.
    parameter integer SAB_ABORT = 0
) (
    input  wire                           clk,
    input  wire                           rst_n,

    // ---- the engine's input store, read side --------------------------------
    // `d64_tconv_engine`, port for port; the registered read is the same
    // registered read.  `in_stall` is the one addition -- see the header.
    input  wire [IN_PORTS-1:0]                 in_rd_en,
    input  wire [IN_PORTS*IN_WORD_W-1:0]       in_rd_addr,
    output wire [IN_PORTS*IC_PAR*IN_W-1:0]     in_rd_data,
    output wire                                in_stall,

    // ---- the DDR read port --------------------------------------------------
    // The same shape the board's other readers use: a request held until it is
    // acknowledged, then a beat stream that ends at `rd_last`.  `rd_len` is
    // beats MINUS ONE, as an AXI `arlen` is, so a one-beat burst is a zero and
    // not a special case.  Addresses are **words of this store**, and a word is
    // a beat -- see the unpack below.
    //
    // **`rd_data` is the DDR's 16-byte beat, not the store word.**  The DDR
    // holds the U16 transport image, so eight channels are 8 x 16 = 128 bits,
    // which is exactly one AXI beat, and the store word is exactly that group
    // of eight.  **One beat is one word, 1:1**, and there is no repacker
    // between them because none is needed.  `d64_h0_block.v:17` says the same
    // for the HEAD layer's reader and `:400` implements it, and that layer is
    // board-verified -- this is the same mechanism, on the read path of a
    // different tensor.
    output reg                            rd_req,
    output reg  [31:0]                    rd_addr,
    output reg  [15:0]                    rd_len,
    input  wire                           rd_ack,
    input  wire [IC_PAR*DDR_LANE_W-1:0]   rd_data,
    input  wire                           rd_valid,
    input  wire                           rd_last,

    // ---- the DDR port's fault side ------------------------------------------
    // The two reports `board/ddr3/d64_axi_read_burst.v` raises and this module
    // did not read until 2026-10-07.  `rd_abort` is a **pulse**: the burst did
    // not close cleanly, so the fill in flight is not installable.  `rd_fatal`
    // is **sticky**: the port is out of service until `rst_n`, and the reset it
    // asks for belongs to the AXI domain and not to this module -- see "What a
    // caller must do with `fatal`" in the header.
    //
    // Both are read the way `region_req` is: in a clocked `if`, or through a
    // conditional, where an unconnected input (`z`) is **false** and not `x`.
    // A build that predates these ports therefore keeps working and keeps its
    // meaning, and `board/ddr3/d64_future_memory_fill.v`'s instance -- which
    // leaves both unconnected -- is exactly that build.
    input  wire                           rd_abort,
    input  wire                           rd_fatal,

    // ---- the region, at run time --------------------------------------------
    // The tensor this instance is serving.  `region_base` is the word address of
    // its word 0 and `region_span` its length in words -- the two numbers the
    // parameters above hold, moved to ports so that one cache serves three
    // tensors instead of three caches costing 61,440 bits each.
    //
    // The handshake: present `region_base`/`region_span` with `region_req` high
    // and hold all three until `region_ack` pulses in answer, then read the new
    // region.  A request is **refused** -- the old region keeps answering -- for
    // as long as the module is not quiescent, and `region_waits` counts the
    // cycles of that.  The header states what quiescence is and why each part
    // of it is there, and why every line is invalidated on the move.
    //
    // **`region_base` must be a multiple of `LINE_WORDS`**, as the parameter
    // was; an unaligned base is not a region.
    //
    // An input left unconnected is `z`, and `z` is not a request: the handshake
    // reads `region_req` in a clocked `if`, where `z` -- like `x` -- is false.
    // So a build that predates this port keeps its parameter region, and there
    // is no path from a floating pin into the address arithmetic.
    input  wire                           region_req,
    input  wire [31:0]                    region_base,
    input  wire [31:0]                    region_span,
    output reg                            region_ack,    // one cycle, on the move
    output reg  [31:0]                    region_waits,  // cycles held off

    // ---- evidence -----------------------------------------------------------
    output reg  [31:0]  reads,          // engine reads transferred
    output reg  [31:0]  hit_count,      // ... served without DDR
    output reg  [31:0]  fill_count,     // lines fetched from DDR
    output reg  [31:0]  ddr_bursts,     // DDR read transactions issued
    output reg  [31:0]  ddr_beats,
    output reg  [31:0]  stall_cycles,   // cycles a read was presented and held
    output reg  [31:0]  addr_viol,      // a read outside the region

    // ---- the fault side, readable ------------------------------------------
    // `aborts` counts the pulses that ended a fill with the port still in
    // service, and `fatal` is the sticky out-of-service state.  Both are outputs
    // because section 6.12 asks for the state to be **readable**, and because
    // there is nowhere else for it to go yet: the coordinated AXI-domain reset
    // `fatal` names is a root-level piece.  Nothing here clears `fatal`; `rst_n`
    // does, and the header says what the caller owes.
    output reg  [31:0]  aborts,
    output reg          fatal
);

    localparam integer WORD_W   = IC_PAR * IN_W;         // 80
    // One DDR beat: eight channels in U16 containers is 128 bits, which is the
    // sixteen bytes `axi_arsize = 3'd4` names.
    localparam integer BEAT_W   = IC_PAR * DDR_LANE_W;   // 128
    localparam integer OFF_W    = $clog2(LINE_WORDS);    // 8
    localparam integer TAG_W    = IN_WORD_W - OFF_W;     // 8
    localparam integer IDX_W    = 2;                     // LINES is 1..4
    localparam integer MEM_N    = LINES * LINE_WORDS;    // 768
    // The line RAM's own address width.  `MEM_N` is 768 at the default three
    // lines -- not a power of two -- so the array is addressed with this and
    // `$clog2` names it rather than a hand count that a parameter change would
    // leave behind.
    localparam integer RA_W     = $clog2(MEM_N);         // 10
    localparam integer NBURSTS  = LINE_WORDS / BURST_WORDS;

    // ------------------------------------------------------------------
    // the arrays: a tag, an age and a valid bit a line; a word a
    // (line, offset); an output word a port
    //
    // `line_mem` is a **block RAM**, and three things make it one: `ram_style`,
    // which is an instruction here and not a hint; a read that is registered
    // inside the memory's own clocked block and is **unconditional**, because
    // an enable on the address is the shape the mapper answers with logic
    // memory; and the select between the RAM, a zero and the bypass beat
    // sitting **after** the RAM's output register instead of in front of it.
    //
    // `ram_out` is that output register and `rd_word_q` is the answer held
    // from it.  The select's control is the accepted cycle's, one cycle behind
    // (`acc_d`, `viol_d`, `bp_word_d`), because the word it is applied to is
    // the one that read produced -- and `sel_now` is a function of those
    // registers only, so it is stable for the whole cycle it is read in.
    //
    // `line_ra` is a port's read address, combinational from the address the
    // engine is presenting, and it is always a real word of the array: `p_idx`
    // is a line and `p_off` an offset inside one, so the sum is below `MEM_N`
    // by construction.  That is what lets the read be unconditional.
    // ------------------------------------------------------------------
    reg [TAG_W-1:0]  tag_r   [0:LINES-1];
    reg [1:0]        age_r   [0:LINES-1];
    reg              valid_r [0:LINES-1];
    (* ram_style = "block" *) reg [WORD_W-1:0] line_mem[0:MEM_N-1];
    reg [WORD_W-1:0] ram_out [0:IN_PORTS-1];   // the RAM's own output register
    reg [RA_W-1:0]   line_ra [0:IN_PORTS-1];   // one read address a port
    reg [WORD_W-1:0] sel_now [0:IN_PORTS-1];   // the select, on that control
    reg [WORD_W-1:0] rd_word_q[0:IN_PORTS-1];  // the answer, held
    reg [IN_PORTS-1:0]  acc_d;                 // a transfer, one cycle on
    reg [IN_PORTS-1:0]  viol_d;
    reg [WORD_W-1:0]    bp_word_d[0:IN_PORTS-1];

    genvar gi;
    generate
        for (gi = 0; gi < IN_PORTS; gi = gi + 1) begin : g_data
            // The answer: the select while a transfer is in flight, and the
            // last one **held** otherwise.  The hold is not a convenience --
            // `tb_d64_burst_prefetch_shared.v` samples `in_rd_data` a delta
            // after the edge that ends the transfer's cycle, and so reads this
            // register where `tb_d64_burst_prefetch.v` reads the cycle before
            // it.  Both benches must see the same word, which is what the
            // single register this replaced gave them by holding.
            assign in_rd_data[gi*WORD_W +: WORD_W] =
                acc_d[gi] ? sel_now[gi] : rd_word_q[gi];
        end
    endgenerate

    // ------------------------------------------------------------------
    // one DDR beat -> one store word
    //
    // Eight channels of `IN_W` bits, each in its own `DDR_LANE_W`-bit
    // container, and nothing else: **no shifter and no crossing**, because the
    // beat order is the word order and the two are the same eight channels.
    // `d64_h0_block.v:400` is the same eight assignments on HEAD's reader.
    //
    // **The `begin`/`end` is not decoration.** `d64_h0_block.v` records what
    // happens without it: efx_map warns that a generate-for needs one, then
    // elaborates a build where the assignment is dropped entirely -- the ring,
    // the write FIFO, the weight store and all 144 DSP came back as zero behind
    // a map that reported success.  The unpack is exactly that shape, so after
    // any build this is the site to check against `MAP_EQUIVALENCE_NOTES.md`.
    // ------------------------------------------------------------------
    wire [WORD_W-1:0] beat_word;
    genvar bi;
    generate
        for (bi = 0; bi < IC_PAR; bi = bi + 1) begin : g_unpack
            assign beat_word[bi*IN_W +: IN_W] =
                SAB_HIGH_LANE
                ? rd_data[bi*DDR_LANE_W + (DDR_LANE_W-IN_W) +: IN_W]
                : rd_data[bi*DDR_LANE_W +: IN_W];
        end
    endgenerate

    // ------------------------------------------------------------------
    // the fault side: the two reports from the adapter
    // ------------------------------------------------------------------
    // **Read in a clocked `if`, never in a combinational expression.**  An
    // input a caller may leave unconnected is `z`.  A **clocked `if` takes `z`
    // as false** -- the rule `region_req` relies on just below, and measured --
    // while a combinational `always @*` in this simulator propagates it as `x`,
    // also measured: an `x` in `abort_q` aborted **every** fill, which is how
    // the first version of this change failed.  So the pulse is registered one
    // cycle before it is acted on.  That costs nothing here: the adapter raises
    // `rd_abort` only once its burst has stopped delivering, and a fill ended
    // one cycle after the pulse is a fill ended with nothing installed, which is
    // the whole of what the rule asks.
    reg abort_q;    // the abort, one cycle behind the adapter's pulse

    // The guard and the tooth.  `abort_ends` is the rule -- the fill is
    // abandoned where it stands -- and `abort_is_end` is `SAB_ABORT = 1`, the
    // reading that installs.  At `SAB_ABORT = 2` both are low and the pulse is
    // invisible, which is the pre-fix module.
    wire abort_ends   = abort_q && (SAB_ABORT == 0);
    wire abort_is_end = abort_q && (SAB_ABORT == 1);
    // The end of **one burst of the fill**: a clean `rd_last`, and -- folded
    // away at 0 -- the tooth's reading of the abort.  `fill_end` is that end
    // carrying a beat, or the tooth's abort arriving on a cycle with no beat at
    // all, which is the shape a timeout has.
    wire fill_last = rd_last | abort_is_end;
    wire fill_end  = (rd_valid | abort_is_end) & fill_last;

    // ------------------------------------------------------------------
    // the region at run time
    // ------------------------------------------------------------------
    // The live region and the request latch.  **Declared here, above the
    // lookup, because the violation test reads the span and the fill's address
    // reads the base**: after a re-point those registers are the only
    // statement about the tensor the lines hold and about the address a refill
    // is for.
    //
    // `req_seen` makes the handshake take a request **once**: a requester that
    // holds `region_req` until the acknowledge it was promised gets one move
    // and not one a cycle.
    reg [31:0]      cur_base;
    reg [31:0]      cur_span;
    reg             req_seen;

    // ------------------------------------------------------------------
    // the lookup, per port
    // ------------------------------------------------------------------
    //
    // The tag is the address above the line offset and the offset is what is
    // left.  A tag is compared against every line at once: at three lines a
    // priority encoder is cheaper than an index and there is no set to select.
    reg [TAG_W-1:0]  p_tag [0:IN_PORTS-1];
    reg [OFF_W-1:0]  p_off [0:IN_PORTS-1];
    reg              p_hit [0:IN_PORTS-1];
    reg              p_viol[0:IN_PORTS-1];
    reg [IDX_W-1:0]  p_idx [0:IN_PORTS-1];
    reg [IN_PORTS-1:0] p_need;
    reg [IN_PORTS-1:0] p_acc;
    reg [IDX_W-1:0]  victim;
    reg              have_victim;
    reg [TAG_W-1:0]  need_tag;
    reg              need_any;
    reg [31:0]       need_base;
    reg [4:0]        acc_n, acc_hit_n, acc_viol_n;
    reg              line_in_use [0:LINES-1];
    reg              hit_any;
    reg [IDX_W-1:0]  hit_idx;

    integer pp, li;
    reg [IN_WORD_W-1:0] ta;
    reg                 v_found;
    reg [IDX_W-1:0]     v_sel;
    reg [1:0]           v_age;

    always @* begin
        for (pp = 0; pp < IN_PORTS; pp = pp + 1) begin
            ta = in_rd_addr[pp*IN_WORD_W +: IN_WORD_W];
            p_tag[pp]  = ta[IN_WORD_W-1:OFF_W];
            p_off[pp]  = ta[OFF_W-1:0];
            // Outside the region is answered with zeros and counted, never
            // fetched.  The walk is not supposed to leave the region -- the
            // masking in `d64_tconv_scheduler` keeps `src_row`/`src_col` on the
            // image -- so a non-zero count here is a walk defect, and a cache
            // that fetched it instead would have hidden one.
            //
            // Against the **live** span and not the parameter: after a re-point
            // this is the only span that is a statement about the data in the
            // lines.
            p_viol[pp] = (ta >= cur_span);
            p_hit[pp]  = 1'b0;
            p_idx[pp]  = {IDX_W{1'b0}};
            for (li = 0; li < LINES; li = li + 1)
                if (valid_r[li] && (tag_r[li] == ta[IN_WORD_W-1:OFF_W])) begin
                    p_hit[pp] = 1'b1;
                    p_idx[pp] = li[IDX_W-1:0];
                end
            // The port's read address, wide enough for the array and no wider.
            // This is the address the memory registers, not the engine's word
            // address, and it is computed from the lookup and not from the
            // enable: a read that is asked for whether or not it is wanted is
            // the whole of what "unconditional" means here.
            line_ra[pp] = p_idx[pp]*LINE_WORDS + p_off[pp];
        end

        // in_stall is a function of the presented address, so it is a function
        // of the presented cycle and of nothing registered.
        for (pp = 0; pp < IN_PORTS; pp = pp + 1) begin
            p_need[pp] = in_rd_en[pp] && !p_hit[pp] && !p_viol[pp];
            p_acc[pp]  = in_rd_en[pp] && !in_stall;
        end

        // the tag the fill is for: the lowest-index port that needs one.
        // Looping down so that port 0 wins.  With IN_PORTS > 1 the ports are
        // adjacent pixels of one row and land in one line, so "lowest index" is
        // the working line and not an arbitrary pick.
        need_any  = 1'b0;
        need_tag  = {TAG_W{1'b0}};
        for (pp = IN_PORTS-1; pp >= 0; pp = pp - 1)
            if (p_need[pp]) begin need_any = 1'b1; need_tag = p_tag[pp]; end
        // The **live** base, so this is the address of the word in DDR and not
        // the address the build was elaborated for.
        need_base = cur_base + (need_tag << OFF_W);

        // the oldest accepted hit, for the age update.  Any accepted read has
        // hit (a miss stalls and is not accepted), so this is the line to
        // freshen.  Only one line an access, and only in F_IDLE -- during a fill
        // nothing is accepted, so the two age writers cannot collide.
        hit_any = 1'b0;
        hit_idx = {IDX_W{1'b0}};
        for (pp = IN_PORTS-1; pp >= 0; pp = pp - 1)
            if (p_acc[pp] && p_hit[pp]) begin hit_any = 1'b1; hit_idx = p_idx[pp]; end

        // ---- the eviction ----------------------------------------------------
        // A line an accepted read is reading this cycle is not a candidate: the
        // line RAM's read and the fill's write would otherwise be the same
        // address in the same cycle, and "the read wins" is a RAM mode and not
        // a rule this module may depend on.  Nothing else is excluded, and
        // something always survives the exclusion: a line is only in use
        // because some live port hits it, and a port that needs a fill does
        // not hit.
        for (li = 0; li < LINES; li = li + 1) begin
            line_in_use[li] = 1'b0;
            for (pp = 0; pp < IN_PORTS; pp = pp + 1)
                if (in_rd_en[pp] && p_hit[pp] && (p_idx[pp] == li[IDX_W-1:0]))
                    line_in_use[li] = 1'b1;
        end

        // an invalid line first, then the oldest age
        v_found = 1'b0;
        v_sel   = {IDX_W{1'b0}};
        for (li = 0; li < LINES; li = li + 1)
            if (!v_found && !valid_r[li] && !line_in_use[li]) begin
                v_found = 1'b1;
                v_sel   = li[IDX_W-1:0];
            end
        if (!v_found) begin
            v_age = 2'd3;
            for (li = 0; li < LINES; li = li + 1)
                if (!line_in_use[li] && (age_r[li] <= v_age)) begin
                    v_age   = age_r[li];
                    v_found = 1'b1;
                    v_sel   = li[IDX_W-1:0];
                end
        end
        victim      = v_sel;
        have_victim = v_found;

        // ---- the per-cycle tallies -------------------------------------------
        acc_n = 5'd0; acc_hit_n = 5'd0; acc_viol_n = 5'd0;
        for (pp = 0; pp < IN_PORTS; pp = pp + 1) begin
            if (p_acc[pp]) begin
                acc_n = acc_n + 5'd1;
                if (p_hit[pp]) acc_hit_n = acc_hit_n + 5'd1;
            end
            if (in_rd_en[pp] && p_viol[pp]) acc_viol_n = acc_viol_n + 5'd1;
        end

        // The select, one per port.  **A function of registered signals only** --
        // the RAM's output register and the three controls that came with it --
        // so it is stable for the whole cycle it is read in and never depends
        // on the address the engine happens to be presenting.  Its position,
        // after the memory's own register and not in front of it, is the whole
        // of what this file's resource history turns on.
        for (pp = 0; pp < IN_PORTS; pp = pp + 1)
            sel_now[pp] = BYPASS ? bp_word_d[pp]
                                 : (viol_d[pp] ? {WORD_W{1'b0}} : ram_out[pp]);
    end

    // ------------------------------------------------------------------
    // the fill: issue, accept, beat, install
    // ------------------------------------------------------------------
    localparam [1:0] F_IDLE = 2'd0, F_REQ = 2'd1, F_DATA = 2'd2;
    localparam [1:0] B_IDLE = 2'd0, B_REQ = 2'd1, B_DATA = 2'd2;

    reg [1:0]       fst;
    reg [IDX_W-1:0] f_line;
    reg [TAG_W-1:0] f_tag;
    reg [15:0]      f_burst;
    reg [15:0]      f_word;

    reg [1:0]       bst;

    // ------------------------------------------------------------------
    // the line RAM: one clocked block, a registered and unconditional read
    // ------------------------------------------------------------------
    // `memory/d64_tc_ram.v`'s shape, which is the primitive the trunk's A and B
    // rings map with and which exists for exactly this reason.  **The read and
    // the write are in the same clocked block and the read is unconditional**:
    // those two are what the mapper's inference pattern matches, and an enable
    // on the address is what it answers with logic memory -- which cost 61,962
    // flip-flops against a device that has 60,800.
    //
    // The write enable is the condition the fill used to write `line_mem`
    // under, moved out of the fill's `case` and named, so that the memory has
    // one writer in one place and the fill's state machine does not carry a
    // write into an array declared a hundred lines above it.  `!BYPASS` is in
    // it although the fill machine never leaves `F_IDLE` in that build: the
    // write is gated by a machine only that build runs, and saying so once is
    // cheaper than proving it twice.
    wire [RA_W-1:0] line_wa = f_line*LINE_WORDS + f_word;
    // **`!abort_ends` is in the write enable and not only in the state
    // machine.**  On the cycle the abort is acted on, a beat that arrives with
    // it belongs to the transaction being abandoned and is not written: the
    // line it was for is not installable.  The words the abandoned burst
    // delivered **before** that cycle are in the RAM and stay there -- behind a
    // tag that is never validated, and overwritten word for word by the next
    // fill of that line, which always covers it.  The tooth's build writes them
    // all, and that is what makes the tooth's line wrong.
    wire            line_we = rst_n && !BYPASS && (fst == F_DATA) && rd_valid
                              && !abort_ends;

    always @(posedge clk) begin
        if (line_we) line_mem[line_wa] <= beat_word;
        for (pp = 0; pp < IN_PORTS; pp = pp + 1)
            ram_out[pp] <= line_mem[line_ra[pp]];
    end

    // ------------------------------------------------------------------
    // the region at run time: the rule
    // ------------------------------------------------------------------
    // The live region and the request latch are declared above, with the
    // lookup, because the violation test reads `cur_span`.

    // **The rule.**  A region may move only in a cycle in which all four of
    // these hold, and they are read in the same cycle the request is presented:
    //
    //   F_IDLE       no fill in flight.  A fill taken across the move would
    //                install a line of the old region under a tag of the new
    //                one -- and because the engine's address is region-relative
    //                that tag is one the new region's walk will ask for.  Wrong
    //                tensor, right-looking read, no counter moved.
    //   B_IDLE       the bypass build's outstanding beat.  Folded away at
    //                `BYPASS = 0`, where `bst` never leaves B_IDLE.
    //   no in_rd_en  nothing presented, a port held by `in_stall` included.  A
    //                miss presented in the accept cycle stalls, and the fill it
    //                starts next cycle would take its address from the new
    //                region and its tag from the old.
    //   no rd_req    no transaction outstanding on the DDR port.
    //
    // On acceptance every line is invalidated in the same cycle, which is the
    // other half of the rule and is argued in the header.
    wire region_idle = (fst == F_IDLE) && (bst == B_IDLE)
                    && !(|in_rd_en) && !rd_req;
    wire region_ready = (SAB_REPOINT == 1) ? 1'b1 : region_idle;

    // The bypass's leading edge: a beat on `rd_data` this cycle means the word
    // is in `rd_word_q` next cycle, which is the latency the engine's store
    // expects.  So `in_stall` is low in exactly the cycle the beat lands and
    // `rd_data` is latched straight into `rd_word_q` at that edge.
    // The bypass build's one outstanding beat is abandoned with the same
    // override the fill machine uses, so the two cannot answer an abort
    // differently: a beat that arrives on the abandoning cycle is not a beat the
    // engine may accept, or the bypass build would answer with the abandoned
    // transaction's word while the fill build refused to.
    wire bp_beat = (bst == B_DATA) && rd_valid && !abort_ends;

    // **A fatal port serves nothing.**  `in_stall` stands high in every cycle,
    // so the engine is held rather than answered -- the only safe ending for a
    // cache that cannot fetch.  It is the miss rule one level up: a cache that
    // answers without a line is the failure this module exists to prevent, and a
    // port that is out of service has no line to answer with and never will.
    assign in_stall = fatal ? 1'b1 : (BYPASS ? ~bp_beat : (|p_need));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fst <= F_IDLE; bst <= B_IDLE;
            rd_req <= 1'b0; rd_addr <= 32'd0; rd_len <= 16'd0;
            f_line <= {IDX_W{1'b0}}; f_tag <= {TAG_W{1'b0}};
            f_burst <= 16'd0; f_word <= 16'd0;
            reads <= 32'd0; hit_count <= 32'd0; fill_count <= 32'd0;
            ddr_bursts <= 32'd0; ddr_beats <= 32'd0; stall_cycles <= 32'd0;
            addr_viol <= 32'd0;
            aborts <= 32'd0;
            fatal  <= 1'b0;
            // The region starts as the parameters, so a build that presents no
            // request is the build this module was before the ports existed.
            cur_base <= BASE_ADDR; cur_span <= SPAN_WORDS;
            req_seen <= 1'b0; region_ack <= 1'b0; region_waits <= 32'd0;
            for (li = 0; li < LINES; li = li + 1) begin
                valid_r[li] <= 1'b0;
                tag_r[li]   <= {TAG_W{1'b0}};
                // distinct ages at reset, so that "the oldest" is defined
                // before anything has been read
                age_r[li]   <= li[1:0];
            end
            // ---- the memory carries no reset -------------------------------
            // `line_mem` and `ram_out`, the register it reads into, are the
            // line RAM's, and a memory is not a register: a reset clause on a
            // memory's output register is one more thing standing between the
            // mapper and the block it is asked to infer.  Everything the
            // **answer** is made of is reset, so `in_rd_data` is a defined
            // zero before the first read rather than an `x`.
            acc_d  <= {IN_PORTS{1'b0}};
            viol_d <= {IN_PORTS{1'b0}};
            for (pp = 0; pp < IN_PORTS; pp = pp + 1) begin
                bp_word_d[pp] <= {WORD_W{1'b0}};
                rd_word_q[pp] <= {WORD_W{1'b0}};
            end
        end
        else begin
            // ---- the region handshake -----------------------------------------
            // `region_req` is read in a clocked `if` and not as a term in a wire
            // expression: an unconnected input is `z`, and `z` is false here,
            // where `z` in a comparison would have been `x` and would have walked
            // into the address arithmetic.
            // ---- the DDR port's two reports --------------------------------
            // The only place either is read, and both are read the way
            // `region_req` is: an unconnected input is `z`, `z` in a clocked
            // `if` is false, and neither signal therefore has a path from a
            // floating pin into the fill machine.  `rd_fatal` is sampled and
            // **latched**; `rd_abort` is a pulse and is re-armed every cycle.
            abort_q <= 1'b0;
            if (rd_abort) abort_q <= 1'b1;
            if (rd_fatal) fatal  <= 1'b1;

            region_ack <= 1'b0;                  // a pulse, one cycle wide
            if (!region_req) req_seen <= 1'b0;   // withdrawn: arm the next one
            if (region_req && !req_seen) begin
                if (region_ready) begin
                    cur_base   <= region_base;
                    cur_span   <= region_span;
                    region_ack <= 1'b1;
                    req_seen   <= 1'b1;
                    // **the invalidation is the other half of the rule.**  The
                    // engine's address is region-relative, so a line of the old
                    // tensor carries exactly the tag a read of the new one asks
                    // for; a surviving line answers that read out of the other
                    // tensor, as a hit, and nothing about it looks wrong.
                    if (SAB_REPOINT != 2)
                        for (li = 0; li < LINES; li = li + 1)
                            valid_r[li] <= 1'b0;
                end
                else
                    region_waits <= region_waits + 32'd1;
            end

            // ---- the read side: the select, against the accepted cycle ------
            // The word itself was registered by the line RAM's own block, at
            // the accept edge, from the address that cycle presented -- so the
            // word for an accepted read is on `ram_out` **now**, and the three
            // things that decide what this module answers with are the ones
            // that read was accepted under.  The select is combinational and
            // the answer is held in the register after it, which is what keeps
            // `in_rd_data` in the cycle the header promises it in.
            //
            // `line_in_use`, above, is unchanged and still load-bearing: the
            // memory's read and the fill's write are both evaluated at the same
            // edge, and a fill must not take a line an accepted read is reading
            // in that cycle.  Where the read registers does not change that
            // rule -- it is why the rule is there.
            acc_d  <= p_acc;
            for (pp = 0; pp < IN_PORTS; pp = pp + 1) begin
                // `p_viol` is an unpacked array of scalars and `viol_d`
                // is a vector, so this is not one assignment.
                viol_d[pp]    <= p_viol[pp];
                bp_word_d[pp] <= beat_word;
                // The hold, loaded with the answer `in_rd_data` is showing
                // combinationally in this cycle, so that it survives into the
                // next.  `acc_d` and `sel_now` here are **last** cycle's
                // transfer and last cycle's select, which is exactly the pair
                // that selected the word being held.
                if (acc_d[pp]) rd_word_q[pp] <= sel_now[pp];
            end

            // ---- the tallies --------------------------------------------------
            if (acc_n != 5'd0) begin
                reads     <= reads + {{27{1'b0}}, acc_n};
                hit_count <= hit_count + {{27{1'b0}}, acc_hit_n};
            end
            if (acc_viol_n != 5'd0) addr_viol <= addr_viol + {{27{1'b0}}, acc_viol_n};
            if ((|in_rd_en) && in_stall) stall_cycles <= stall_cycles + 32'd1;

            // ---- the ages -----------------------------------------------------
            if (hit_any && (fst == F_IDLE)) begin
                age_r[hit_idx] <= 2'd3;
                for (li = 0; li < LINES; li = li + 1)
                    if ((li[IDX_W-1:0] != hit_idx)
                        && (age_r[li] > age_r[hit_idx]))
                        age_r[li] <= age_r[li] - 2'd1;
            end

            // ---- the fault side: the two overrides ---------------------------
            // `rd_fatal` and `rd_abort` are acted on **before** either machine,
            // in one place, so that the fill and the bypass build cannot answer
            // them differently.  `rd_fatal` is first: a fatal port is out of
            // service, and the `rd_abort` of a port that is giving up is not a
            // second event -- the adapter raises both over the same failure, and
            // `fatal` latches on the pulse where `abort_q` is one cycle behind
            // it, so the cycle that carries both counts as fatal and not as an
            // abort.
            if (fatal) begin
                fst    <= F_IDLE;
                bst    <= B_IDLE;
                rd_req <= 1'b0;
            end
            else if (abort_ends) begin
                // **The rule: the line is not installed.**  No `valid_r`, no
                // `tag_r`, no `line_we` -- the three places a fill touches the
                // cache all pass this cycle by.  The machine goes back to
                // `F_IDLE`, one cycle later it re-issues the same fill for the
                // same tag, and the read the engine has been holding on
                // `in_stall` is served by that second burst.
                fst    <= F_IDLE;
                bst    <= B_IDLE;
                rd_req <= 1'b0;
                aborts <= aborts + 32'd1;
            end
            else if (!BYPASS) begin
                // ---- the demand fill ------------------------------------------
                case (fst)
                    F_IDLE: begin
                        if (need_any && have_victim) begin
                            f_tag   <= need_tag;
                            f_line  <= victim;
                            f_burst <= 16'd0;
                            f_word  <= 16'd0;
                            rd_addr <= need_base;
                            rd_len  <= BURST_WORDS - 1;
                            rd_req  <= 1'b1;
                            fill_count <= fill_count + 32'd1;
                            fst     <= F_REQ;
                        end
                    end
                    F_REQ: begin
                        if (rd_ack) begin
                            rd_req <= 1'b0;
                            ddr_bursts <= ddr_bursts + 32'd1;
                            fst <= F_DATA;
                        end
                    end
                    F_DATA: begin
                        if (rd_valid) begin
                            // the write to `line_mem` is in the line RAM's own
                            // clocked block, beside its read, so that the
                            // mapper sees one memory with a registered read
                            // port -- see `line_we` above.
                            ddr_beats <= ddr_beats + 32'd1;
                            f_word <= f_word + 16'd1;
                        end
                        // **The end of this burst**, which in the guarded build
                        // is a clean `rd_last` and in the tooth is that or an
                        // `rd_abort`.  A guarded build that is aborted never
                        // reaches here: the override above returns the machine
                        // to `F_IDLE` on the cycle the pulse is registered on.
                        if (fill_end) begin
                            if (f_burst == (NBURSTS - 1)) begin
                                // install: the line becomes the tag, and the
                                // newest age
                                valid_r[f_line] <= 1'b1;
                                tag_r[f_line]   <= f_tag;
                                age_r[f_line]   <= 2'd3;
                                for (li = 0; li < LINES; li = li + 1)
                                    if ((li[IDX_W-1:0] != f_line)
                                        && (age_r[li] > age_r[f_line]))
                                        age_r[li] <= age_r[li] - 2'd1;
                                fst <= F_IDLE;
                            end
                            else begin
                                f_burst <= f_burst + 16'd1;
                                // from `f_tag`, not from the live port: the
                                // burst after the first must not depend on what
                                // the engine happens to be presenting.
                                rd_addr <= cur_base + (f_tag << OFF_W)
                                         + ((f_burst + 16'd1) * BURST_WORDS);
                                rd_req <= 1'b1;
                                fst <= F_REQ;
                            end
                        end
                    end
                endcase
            end
            else begin
                // ---- the sabotaged build: one transaction a tap -------------
                // Verification only.  The address is the engine's, so the words
                // are still right; the count is the point.  Nothing is served
                // from the cache, so `in_stall` is high until the beat lands.
                case (bst)
                    B_IDLE: begin
                        // started by `in_rd_en` and not by a transfer, because
                        // in this build nothing is ever servable on the cycle it
                        // is asked for.  The engine holds the address while
                        // `in_stall` is high, which is the whole fetch.
                        if (in_rd_en[0]) begin
                            rd_addr <= cur_base + in_rd_addr[IN_WORD_W-1:0];
                            rd_len  <= 16'd0;
                            rd_req  <= 1'b1;
                            bst     <= B_REQ;
                        end
                    end
                    B_REQ: if (rd_ack) begin
                        rd_req <= 1'b0;
                        ddr_bursts <= ddr_bursts + 32'd1;
                        bst <= B_DATA;
                    end
                    B_DATA: if (rd_valid) begin
                        ddr_beats <= ddr_beats + 32'd1;
                        bst <= B_IDLE;
                    end
                endcase
            end
        end
    end

endmodule
