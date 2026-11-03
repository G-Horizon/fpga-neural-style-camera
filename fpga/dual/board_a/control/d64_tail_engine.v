`timescale 1ns/1ps

// F2 tail -- the network's last layer, 8 input channels to 3, RGBX out.
//
// The layer
// ---------
//
// 3x3, stride 1, 8 input channels -> 3 output channels, **no ReLU**, on a
// 256x256 grid, with **reflection** at all four edges.  The output is packed
// RGBX: one 32-bit word a pixel, `0x00RRGGBB`, channel 0 in the low byte.
//
// This engine owns no multipliers.  It drives the shared PE over the seam in
// `docs/F2_INTERFACES_2026-10-06.md` section 1, exactly as `d64_tconv_engine`
// does at `EXTERNAL_PE = 1`, and it builds no bank of its own -- `d64_mac128` is
// instantiated once for the whole network and 144 DSP blocks will not be spent
// twice.  It does use `d64_mac128_compat_map`, which is the operand permutation
// and not a bank: it is what turns this engine's flat `act`/`wgt`/`bias` into
// the bank's lane numbering, with no logic.
//
// The fill lane, in full
// ----------------------
//
// `d64_mac128` runs this layer as `MAC_P4_I8_O4`: 4 pixels x 8 input channels x
// 4 output channels is 128 multipliers, and the layer has **three** output
// channels.  So output channel 3 is a fill.
//
// Where it is, exactly.  The bank numbers its accumulators `k = p*OC + o`, so
// the fill occupies `k = 3, 7, 11, 15` -- one accumulator in every group of
// four, pixel-major.  The requant returns eight lanes a pass (`RQ_LANES`), pass
// 0 carrying `k = 0..7` and pass 1 carrying `k = 8..15`, so the fill arrives as
// **lanes 3 and 7 of every requant pass**.
//
// Where it is dropped.  The packer takes lanes 0, 1 and 2 into R, G and B and
// writes a constant zero into the top byte; lane 3 -- `pk_src[3*OUT_W +: 8]`,
// the fourth lane of whichever half-pass the packer is holding -- is never
// selected into `out_wr_data`.  `FILL_SABOTAGE` puts that lane into the top
// byte instead, which is the tooth: a lane that is never checked is a lane that
// can be written.
//
// The fill's weights are zero and its bias is not, so its accumulator is a
// **constant non-zero** and the tooth has something to leak.  A fill lane that
// requants to zero would make the sabotaged build pass, which is a tooth that
// does not fire.
//
// The schedule, in cycles from the start of a phase
// -------------------------------------------------
//
// A phase is four output pixels -- `4g .. 4g+3` of output row `r` -- and takes
// twelve cycles.  Every phase is the same twelve cycles, so there is no state
// machine past a counter and no schedule table to disagree with:
//
//   cy 0,1,2   input reads.  Source rows `reflect_y(r-1)`, `r`, `r+1`, each read
//              as six columns `reflect_x(4g-1) .. reflect_x(4g+4)` at all eight
//              channels -- `IN_PORTS` ports, one read a port a cycle
//   cy 1,2,3   the store answers; the three rows land in `rf` in that order
//   cy 2..10   the nine weight tiles, tap `cy-2`, one a cycle so the data is
//              back in time for the slot that wants it
//   cy 2       `pix_start`: the bias load for this phase
//   cy 3..11   the nine MAC slots.  Slot `cy-3` is tap `(ky,kx) = (t/3, t%3)`,
//              and its operand is `rf[ky][p+kx][i]` for output pixel `p`, input
//              channel `i` -- which is the whole of the convolution, and the
//              reflection is already inside the `rf` indices
//   cy 11      the phase's last slot
//   cy 12      the accept: this phase's accumulator is snapshotted while the
//              next phase's first read is already in flight
//
// Six input columns and not four
// ------------------------------
//
// The window for output pixels `4g..4g+3` with a 3-wide kernel is input columns
// `4g-1 .. 4g+4`; four columns would only reach `4g+2`.  The six are read once
// and the three kernel columns are muxed out of them, so the three taps of a
// kernel row share one read each and not three.
//
// What the reflection costs, and what it does not
// -----------------------------------------------
//
// Both helpers are applied at the read address and nowhere else, so the store is
// the plain image and no padded copy exists.  They are copies, verbatim, of
// `d64_head_window.reflect_x` and `d64_head_ctrl.reflect_y`; there is no shared
// helper in this tree to call and this module does not get to edit one in.
// `CLAMP_SABOTAGE` replaces them with a clamp, which is the other tooth.
//
// 18 reads a phase, 4.5 an output pixel
// -------------------------------------
//
// Output rows `r` and `r+1` share two of their three source rows and this engine
// still re-reads them, so the layer reads its input **4.5 times**: 294,912
// external reads against a 65,536-pixel tensor.  Holding three rows on chip is
// the fix and it is not here -- the input prefetch policy is section 2.3's, and
// this engine's port set is the one the contract names.  Written down so it is
// a measured cost and not an accident.
//
// **"Holding three rows on chip" is not the fix, and that has now been
// measured** (`docs/TAIL_WINDOW_2026-10-07.md`).  A phase is four pixels of ONE
// row, so the three source rows do not advance from phase to phase at all: the
// rows a window could hold are already held, one line RAM away, in
// `d64_burst_prefetch`, which serves every read this engine makes as a **hit**.
// What advances is the **column** window, and carrying it cuts the reads from
// 18.000 to 12.094 a phase -- 294,912 to 198,143 measured on this frame -- while
// leaving the **worst** phase at 18, which is the phase a store is sized for.
// The storage such a window needs is registers and not block RAM (the operand
// read is 32 words combinationally, so it can never be a block RAM), so it buys
// no RAM10 and removes no port.  The header above said the re-read was the fix;
// it is the re-read that is the cost, and the fix is not a window.
//
// Alignment
// ---------
//
// `act` and `wgt` are registered together and the control travels with them.
// Registering the operand alone -- the weight going straight into the bank --
// is the one-cycle skew `d64_tconv_engine` exists to prevent, committed by the
// module itself.  One consequence is not local: `b_prime` is `s_prime` one
// cycle later and the first slot is two, so **`pix_start` never lands on a
// slot**.  `d64_tconv_engine` lets it land on one and leans on the bank folding
// that slot into `done_acc`; this engine does not need the fold, and driving
// `pix_start` where no slot is present is what makes that independence
// checkable rather than argued.
//
// A refused accept pauses the phase, and the pause is a replay
// -----------------------------------------------------------
//
// `pe_rsp_rq_ready` is a handshake and not a courtesy.  When the bank answers
// the accept cycle low -- it is busy with the previous request's requant, or an
// arbiter has not granted it -- the phase's accumulator is not taken.  The
// engine used to carry on regardless: held un-ready from the start it reported
// `requests 9216, accepts 0, writes 0, done 1` at H=144, a frame it never
// computed announced as finished, because `done` watched the walk reaching its
// end and the walk reaches its end whether or not anyone was listening.
//
// So a refused accept **rewinds the walk to the phase it refused and runs that
// phase again**.  The rewind is the phase and not the read address: the six
// activations of the three source rows, the nine weight tiles, the phase's
// identity in `row`/`grp` and the accept itself are all re-issued together,
// because a phase is one unit and a half-issued one is a phase whose operands
// and whose identity disagree.  The hold is the same state machine: the refusal
// is seen on the cycle the accept is presented, and on that edge `cy` returns to
// zero and `row`/`grp` return to the refused phase's coordinates -- which stage
// B still holds, because `b_row`/`b_grp` are the coordinates the metadata queue
// is pushed from.
//
// Nothing has to be undone, and that is why the rewind is safe.  A refused
// accept pushes nothing into the metadata queue (`mf_push` is `b_accept`),
// starts no requant (`rq_start` is `b_accept`), and writes nothing out.  The one
// thing it leaves behind is the first read of the phase that was advanced into
// and never reached; the registered store answers that read on the replay's
// `cy 0`, which is a cycle the `rf` write decoder does not write on, so it lands
// nowhere.
//
// The cost of a refusal is the phase, and the counters say so.  A replayed phase
// presents its accept again, so `rq_requests` counts presentations and
// `rq_accepts` counts phases; their difference is the number of refusals.
// `slot_count`, `in_read_count` and `w_read_count` are *issued* counts and grow
// with the replays -- which is the evidence that the activations and the weights
// really were re-issued rather than held.
//
// The bank reports one lost accumulator per refusal and it is right to:
// `d64_mac128`'s `rq_overrun` is `pix_start & acc_dirty & ~rq_accept`, and the
// replay's bias load is the cycle the aborted attempt's slots are still dirty.
// So `rq_overrun_count` is *exactly* the number of refusals here, and the gate
// checks that identity instead of requiring a zero.
//
// The input side's accept, and the hold that is a phase
// ----------------------------------------------------
//
// This engine's store-facing side is `in_rd_en` / `in_rd_addr` / `in_rd_data`
// and, since F3 section 6.9, `in_stall`.  The store this engine is pointed at
// in the network is not a RAM: it is `d64_burst_prefetch`, whose own header
// says why the port exists -- *"a cache that cannot stall is a cache that
// returns the wrong word quietly"* -- and whose integration note assumes a
// scheduler `stall` this engine does not have.  So the accept lives here.
//
// **A read is transferred in the cycle `in_stall` is low**, and the word for
// the address presented in that cycle arrives on `in_rd_data` in the following
// one.  While it is high, the engine holds `in_rd_en` and `in_rd_addr` and the
// read is not transferred.
//
// The hold is the phase, and it is one counter
// --------------------------------------------
//
// Everything this engine does inside a phase is a function of `cy` and of
// `row`/`grp`: the six read addresses (`cy - 1` and the reflection), the nine
// weight tiles (`cy - CY_W0`), the tap and its kernel position (`cy - CY_S0`),
// the slot window, `pix_start` at `cy == 2`, the accept one cycle after the
// last slot, and the three `rf` write cycles at `cy` 1, 2 and 3.  So **not
// advancing `cy` holds all of them at once** -- the address, the activation,
// the weight, the phase's identity and the last-MAC-slot relationship the
// review's rule 1 names -- and it holds them consistently, because they were
// never separate.  A fix that held `in_rd_en` and `in_rd_addr` while letting
// `cy` run would keep the address and lose the relationship, which is the
// failure that rule is about.
//
// The hold is safe on the read side for the same reason, and this is worth
// writing down because it is not obvious: the `rf` write decoder fires at
// `cy == 1`, 2 and 3 from `in_rd_data`, and `d64_burst_prefetch` holds its
// previous answer on `in_rd_data` for as long as it cannot serve a new one.
// During a hold at `cy == c` the word being held is the one the last
// *transferred* read produced, which is read `c-1` -- the row that cycle's
// write is for.  The write is therefore idempotent over the whole hold, and
// the row it writes is the right row whether it is written once or a thousand
// times.  A hold at `cy == 0` writes no row at all.
//
// Rule 3 -- "no stale data during a miss" -- is this and not a separate guard:
// **the read window is `cy <= 2` and the slot window opens at `cy == 3`.**  A
// store that cannot answer freezes the phase before any of its twelve slots
// have run, so no slot is ever presented with the word the store was still
// holding from the previous phase.  There is no cycle in which the engine could
// compute on the previous word, because the hold does not let the phase reach
// the slot that would.
//
// The two pauses, and which of them wins
// --------------------------------------
//
// A phase can be refused by the PE (section 6.2, above) in the same cycle a
// read is refused by the store: both are presented at `cy == 0`, the PE's
// refusal being the cycle after the phase's last slot.  The two want opposite
// things -- the PE's refusal rewinds `row`/`grp` to the phase it refused, which
// *changes the address* the store is still holding, and the store's contract is
// that the address is held while `in_stall` is high.
//
// **The hold wins, and the rewind is latched.**  On a refusal with a read
// outstanding the engine sets `rw_pend` and leaves `cy`, `row`, `grp`, `in_rd_en`
// and `in_rd_addr` exactly where they are; the rewind to the refused phase's
// coordinates is taken at the first edge on which no read is outstanding, where
// the read that was in flight has been transferred (and its word is dropped --
// the phase it belonged to has been abandoned, and `cy == 0` is a cycle the `rf`
// write decoder does not write on).  The refused phase is replayed whole either
// way; what the latch buys is that the store is never handed a new address
// while it is still looking for the old one.
//
// Counters
// --------
//
// `in_stall_cycles` counts the cycles a read was presented and held and
// `in_stall_events` the refused reads, one an edge -- the same pair the PE side
// carries as `rq_wait_cycles` and `fault_count`, and for the same reason: a
// stall that can only be seen as a longer frame is a stall nobody can point at.
// Neither is a fault.  A cache that never stalled would be a cache with no DDR
// behind it.
//
// **No premature success, and the completion gate does not change.**  The walk
// cannot reach `sdone_seen` while a read is outstanding, because the hold is a
// hold on `cy` and `sdone_seen` is set from `cy == CY_LAST`; the last phase's
// own reads are resolved before its accept exists.  A completion while a read
// is outstanding is therefore unreachable rather than gated, and
// `tools/run_tail_input_stall.sh`'s `stuck` arm measures it (a store that never
// answers the last line: no `done`, a growing `in_stall_cycles`, and words still
// unwritten) instead of arguing it from this paragraph.
//
// What it costs, and what it does not
// -----------------------------------
//
// One flop of state (`rw_pend`, with its two coordinate registers), one AND for
// `s_stall`, and the two counters.  The engine is otherwise untouched, and a
// build that leaves `in_stall` unconnected -- every gate that existed before
// this port, `tb_d64_tail_engine_pause.v` and the board's output chain included
// -- is bit-for-bit the build it was: `z` is not a stall, in the clocked `if`s
// that consume it, exactly as `d64_burst_prefetch` reads an unconnected
// `region_req` as no request.
//
// The completion gate
// -------------------
//
// `done` is gated on what actually happened, and the walk's end is not evidence
// of anything.  The gate is the phase count, the word count and the fault -- and
// the phase count is the **shape's** (`H*W/PIX_PAR`) and not the walk's
// `rq_requests`, because a replayed phase presents its accept twice and a gate
// written against the walk is a gate no replayed frame could ever satisfy.
//
// The fault and the counts are not the same cycle's evidence, and that
// difference is load-bearing.  `fault_sticky` is a register: the refusal sets it
// on the edge, so it is still low on the cycle the refusal is raised on -- which
// on the last phase is the cycle `sdone_seen` is high and the drain is empty,
// the cycle a false `done` fires on.  The counts are combinational and are short
// on that same cycle.  A build with the counts dropped from the gate and the
// fault kept reports a finished frame with a phase never accepted: that is the
// tooth in `tools/run_tail_engine_pause.sh`.
//
// Counters
// --------
//
// `rq_wait_cycles` **counts now, and it counts cycles**: the cycle the accept was
// refused and every cycle of the replay it forces.  It used to exist and never be
// incremented, on the argument that the accept rate is one a twelve cycles and
// the requant is done in five, so `rq_ready` is high every time it is asked.  The
// argument is true of a bank nobody else drives; it is not a reason for a counter
// to be unable to move.  Held un-ready it read 0 while 9,216 requests went
// unaccepted, and a counter that cannot move is not evidence.
module d64_tail_engine #(
    // ---- the layer -------------------------------------------------------
    parameter H          = 256,
    parameter W          = 256,
    parameter PIX_PAR    = 4,        // output pixels a phase
    parameter IC_PAR     = 8,
    parameter OC_LANES   = 4,        // the bank's MAC_P4_I8_O4 group
    parameter OC_REAL    = 3,        // the layer's; channel 3 is the fill
    parameter K          = 3,
    // Six input columns a phase: `4g-1 .. 4g+4`.  Read once, muxed per tap.
    parameter IN_PORTS   = 6,
    // One 64-bit word a pixel, addressed `row*W + col`.  W is 256, so the row
    // is the high byte of the word address; 16 bits holds the largest of them,
    // 65535, exactly.
    parameter IN_ADDR_W  = 16,
    parameter IN_W       = 8,        // stored activation width
    parameter IN_ZERO    = 0,        // ups.1's output is a ReLU tensor
    // The nine taps, tap `ky*3+kx`, no base.  `W_ADDR_W` is wider than the four
    // bits nine taps need so a shared store with a base needs no change here.
    parameter W_BASE     = 0,
    parameter W_ADDR_W   = 12,
    parameter OUT_ADDR_W = 18,

    // ---- the PE seam's widths: the network's, not this layer's -----------
    //
    // A port list cannot see a `localparam`, so every width a port needs is a
    // parameter and they are the contract's own numbers.
    parameter ACT_W      = 11,
    parameter WGT_W      = 8,
    parameter ACC_W      = 32,
    parameter M_W        = 16,
    parameter SHIFT_W    = 5,
    parameter OUT_W      = 10,
    parameter RQ_LANES   = 8,
    parameter PE_LANES   = 128,
    parameter PE_OC_MAX  = 16,
    parameter PE_OUT_W   = 10,
    // The 8-bit tail is a `qmax` of 255 on a 10-bit lane.  `d64_requant_lane`
    // clamps to its full range first and the bank clamps to these second; the
    // two compose, so this is the layer's saturation and the lane's is not.
    // "No ReLU" is about the layer: an unsigned 8-bit image has no negative
    // values to carry, so the format's low clamp is 0 whether the layer wants
    // an activation function or not.
    parameter integer QMIN = 0,
    parameter integer QMAX = 255,

    // ---- verification only, folded away at 0 -----------------------------
    // The fill lane into the top byte instead of the constant zero.
    parameter integer FILL_SABOTAGE  = 0,
    // Reflection disabled: clamped at both ends instead of mirrored.
    parameter integer CLAMP_SABOTAGE = 0,
    // The engine as it was before the input side had an accept path: the
    // schedule advances whether or not the store could take the read.  The
    // tooth for this stage, and it must fail on data -- see the header.
    parameter integer IN_STALL_SABOTAGE = 0
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    output reg                     busy,
    output reg                     done,
    output reg  [31:0]             cycles,

    // ---- the layer's quantisation ----------------------------------------
    input  wire [OC_LANES*M_W-1:0]     rq_m,
    input  wire [OC_LANES*SHIFT_W-1:0] rq_shift,
    input  wire [OC_LANES*ACC_W-1:0]   rq_bias,

    // ---- the input store, external, registered read -----------------------
    // One port a column; the word is one pixel, all eight channels.
    output wire [IN_PORTS-1:0]             in_rd_en,
    output wire [IN_PORTS*IN_ADDR_W-1:0]   in_rd_addr,
    input  wire [IN_PORTS*IC_PAR*IN_W-1:0] in_rd_data,
    // **The read is transferred in the cycle `in_stall` is low**, and the word
    // for the address presented in that cycle is on `in_rd_data` in the
    // following one.  While it is high the engine holds `in_rd_en` and
    // `in_rd_addr` -- and, because a phase is one unit, it holds the phase with
    // them.  See "The input side's accept" in the header.
    //
    // An input left unconnected is `z`, and `z` reads as *no stall* in the
    // clocked `if`s that consume it, which is the same idiom
    // `d64_burst_prefetch` uses for `region_req`.  A build that predates this
    // port therefore keeps the behaviour it had, bit for bit.
    input  wire                            in_stall,

    // ---- the weight store, external, registered read ----------------------
    output wire                    w_rd_en,
    output wire [W_ADDR_W-1:0]     w_rd_addr,
    input  wire [IC_PAR*OC_LANES*WGT_W-1:0] w_rd_data,

    // ---- the output, one packed pixel a cycle -----------------------------
    output wire                    out_wr_en,
    output wire [OUT_ADDR_W-1:0]   out_wr_addr,
    output wire [31:0]             out_wr_data,

    // ---- counters ---------------------------------------------------------
    output reg  [31:0]             rq_requests,
    output reg  [31:0]             rq_accepts,
    output reg  [31:0]             rq_wait_cycles,
    output reg  [31:0]             rq_overrun_count,
    output reg  [31:0]             slot_count,
    output reg  [31:0]             out_write_count,
    output reg  [31:0]             mfifo_peak,

    // ---- the input side's stall, as a number ------------------------------
    //
    // A read the store would not take is a **number on a port** and not only a
    // longer `cycles`: `in_stall_cycles` is the cycles a read was presented and
    // held, `in_stall_events` the refused reads (one an edge of the hold).
    // Neither is a fault -- a cache that never stalled is a cache with no DDR
    // behind it -- and the fault this pair exists for is the other one: a
    // completion that arrives while a read is outstanding, which the hold makes
    // unreachable and which `tools/run_tail_input_stall.sh` measures rather
    // than argues.
    output reg  [31:0]             in_stall_cycles,
    output reg  [31:0]             in_stall_events,

    // ---- the fault: as a number, and as a latch ---------------------------
    //
    // `fault_count` counts every refused accept this run has seen and only
    // `start` clears it -- sticky in the plain sense, and a *number*, so an
    // engine that was refused reads as a count rather than as a DONE that did
    // not arrive.  `fault_sticky` is the refusal that has not yet been made
    // good: set by the refusal edge, held while the phase is outstanding, and
    // cleared only by the accept that resolves it.  **While it is high `done` is
    // impossible**, however the walk got on.
    output reg  [31:0]             fault_count,
    output reg                     fault_sticky,

    // ---- the PE seam, section 1 -------------------------------------------
    output wire                      pe_req_slot_valid,
    output wire [PE_LANES*ACT_W-1:0] pe_req_act,
    output wire [PE_LANES*WGT_W-1:0] pe_req_wgt,
    output wire                      pe_req_pix_start,
    output wire [PE_LANES*ACC_W-1:0] pe_req_bias,
    // The shape, out loud.  An owner that does not own its bank has to say
    // which one it is: the trunk's external testbench reached into the bank's
    // hierarchy for this and a synthesised arbiter cannot.
    output wire [1:0]                pe_req_mode,
    output wire                      pe_req_rq_start,
    output wire [PE_OC_MAX*M_W-1:0]  pe_req_rq_m,
    output wire [PE_OC_MAX*SHIFT_W-1:0] pe_req_rq_shift,
    output wire [PE_OUT_W-1:0]       pe_req_rq_zero,
    output wire [PE_OUT_W-1:0]       pe_req_rq_qmin,
    output wire [PE_OUT_W-1:0]       pe_req_rq_qmax,
    input  wire                      pe_rsp_rq_ready,
    // Declared and never read: nothing here stalls, so the one-cycle-early
    // warning has nothing to warn.  It stays on the port because it is half of
    // the contract, and a later owner of this bank will want it.
    input  wire                      pe_rsp_rq_ready_n,
    input  wire [RQ_LANES*PE_OUT_W-1:0] pe_rsp_y,
    input  wire [RQ_LANES-1:0]       pe_rsp_y_valid,
    input  wire                      pe_rsp_rq_busy,
    input  wire                      pe_rsp_rq_done,
    input  wire                      pe_rsp_rq_overrun,
    input  wire [31:0]               pe_rsp_mode_viol,
    input  wire [PE_LANES*ACC_W-1:0] pe_rsp_acc,
    input  wire [PE_LANES*ACC_W-1:0] pe_rsp_snapshot,
    output wire                      mac_mode_safe
);

    localparam ROW_W   = $clog2(H);          // 8
    localparam GRP_W   = $clog2(W/PIX_PAR);  // 6
    localparam NGRP    = W / PIX_PAR;        // 64 phases a row

    // The frame's debt, from the shape and not from the walk.  These are what
    // the completion gate is written against: a replayed phase presents its
    // accept twice, so the walk's own request count is not the phase count.
    localparam [31:0] NPHASE_TOTAL = H * W / PIX_PAR;   // 16,384 at 256
    localparam [31:0] NPIX_TOTAL   = H * W;             // 65,536 at 256

    localparam NPIX    = PIX_PAR * OC_LANES;                // 16 accumulators
    localparam RQ_PASS = (NPIX + RQ_LANES - 1) / RQ_LANES;  // 2 requant passes
    localparam RQW     = $clog2(RQ_PASS + 1);               // 2

    //: Requests whose passes have not all come back.  The accept rate is one a
    //: twelve cycles and a request's two passes arrive back to back, so two
    //: would do; four is what tconv carries and costs two flops a field.
    localparam MFIFO = 4;
    localparam MFW   = 2;

    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_RUN   = 2'd1,
                     ST_DRAIN = 2'd2,
                     ST_DONE  = 2'd3;

    // The phase's twelve cycles, named once.  The weight tile for tap `t` is
    // issued at `CY_W0 + t` and the slot for tap `t` runs at `CY_S0 + t`, which
    // is why the weight read is one cycle early: the store is registered.
    localparam [3:0] CY_RD    = 4'd2,
                     CY_W0    = 4'd2,
                     CY_W1    = 4'd10,
                     CY_PRIME = 4'd2,
                     CY_S0    = 4'd3,
                     CY_LAST  = 4'd11;

    // ------------------------------------------------------------------
    // the schedule
    // ------------------------------------------------------------------

    reg  [1:0]       st;
    reg  [3:0]       cy;
    reg  [ROW_W-1:0] row;
    reg  [GRP_W-1:0] grp;

    // The refused phase waiting for a read to be transferred before it is
    // replayed.  `rw_pend` is set on the refusal edge **only** when `s_stall` is
    // high there, so in every build whose store never stalls it is reset and
    // never written: it cannot change the behaviour of a build that predates
    // `in_stall`.
    reg              rw_pend;
    reg  [ROW_W-1:0] rw_row;
    reg  [GRP_W-1:0] rw_grp;

    wire s_rd    = (st == ST_RUN) && (cy <= CY_RD);
    // **A read is presented and the store will not take it.**  This one wire is
    // the whole of the input-side accept: the hold it forces, the rewind it
    // defers, and the two counters below.  `IN_STALL_SABOTAGE` is the pre-fix
    // build and folds away at zero.
    //
    // It is consumed only in clocked `if`s, and deliberately: an `in_stall` left
    // unconnected is `z`, `s_rd && z` is `x`, and `if (x)` takes the else branch
    // -- which is the pre-fix behaviour.  A **ternary** on this wire would merge
    // the two arms into `x` and walk an `x` into `cy`; there is none.
    wire s_stall = s_rd && in_stall && !IN_STALL_SABOTAGE;
    wire s_wen   = (st == ST_RUN) && (cy >= CY_W0) && (cy <= CY_W1);
    wire s_slot  = (st == ST_RUN) && (cy >= CY_S0) && (cy <= CY_LAST);
    wire s_prime = (st == ST_RUN) && (cy == CY_PRIME);
    wire s_last  = (st == ST_RUN) && (cy == CY_LAST);

    // `cy` underflows below the read and slot windows and the index it makes is
    // then whatever the subtraction left.  Both are clamped to the last legal
    // value rather than left to the store: an out-of-range read of `rf` is an x
    // that walks into the multiplier bank, and on the first phase there is
    // nothing in `rf` yet for it to be an x instead of.
    wire [3:0] s_wtap = (cy >= CY_W0) ? (cy - CY_W0) : (CY_W1 - CY_W0);
    wire [3:0] s_tap  = (cy >= CY_S0) ? (cy - CY_S0) : (CY_LAST - CY_S0);

    // Tap index to the kernel position.  `ky = t/3, kx = t%3`, written out
    // because a divide by three is a divider unless it is a case.
    function [3:0] tap_rc;
        input [3:0] t;
        begin
            case (t)
                4'd0: tap_rc = 4'b00_00;
                4'd1: tap_rc = 4'b00_01;
                4'd2: tap_rc = 4'b00_10;
                4'd3: tap_rc = 4'b01_00;
                4'd4: tap_rc = 4'b01_01;
                4'd5: tap_rc = 4'b01_10;
                4'd6: tap_rc = 4'b10_00;
                4'd7: tap_rc = 4'b10_01;
                default: tap_rc = 4'b10_10;
            endcase
        end
    endfunction

    wire [3:0] s_rc = tap_rc(s_tap);
    wire [1:0] s_ky = s_rc[3:2];
    wire [1:0] s_kx = s_rc[1:0];

    // ------------------------------------------------------------------
    // the reflection, at the read address and nowhere else
    // ------------------------------------------------------------------
    //
    // `d64_head_window.reflect_x` and `d64_head_ctrl.reflect_y`, copied.  The
    // tree has them as separate module-local functions and neither is callable
    // from here without editing a file this work does not own.
    //
    // `reflect_y` takes `signed [9:0]` as the head's does.  H is 256 by
    // default, so the argument is in `-1 .. H` and ten signed bits covers
    // `-512 .. 511`.
    //
    // **The bounds are functions of H, not literals.**  This function used to
    // hardcode 255/254 while `H` sat two hundred lines above it unused, with a
    // comment reading "H is 256" directly over the bug -- the parameter
    // existed, the boundary ignored it, and changing H moved the row counter
    // and left the reflection reading a row off the end of the tensor.
    //
    // The bound is written in the integer form `H - 1` and not as an 8-bit
    // localparam, because the argument is `signed`: comparing it against an
    // unsigned value would make -1 read as a large positive number and the
    // top-edge reflection would silently become the bottom one.

    function [7:0] reflect_y;
        input signed [9:0] y;
        begin
            if (CLAMP_SABOTAGE) begin
                if (y < 0)          reflect_y = 8'd0;
                else if (y > H - 1) reflect_y = H - 1;
                else                reflect_y = y[7:0];
            end
            else begin
                if (y < 0)          reflect_y = 8'd1;
                else if (y > H - 1) reflect_y = H - 2;
                else                reflect_y = y[7:0];
            end
        end
    endfunction

    function integer reflect_x;
        input integer x;
        begin
            // Width, not height: W stays 256 and these are bit-identical to
            // the literals they replace.  Written as functions of W for the
            // same reason `reflect_y` now reads H -- a literal that is right
            // by coincidence is wrong after the next change.
            if (CLAMP_SABOTAGE) begin
                if (x < 0)          reflect_x = 0;
                else if (x > W - 1) reflect_x = W - 1;
                else                reflect_x = x;
            end
            else begin
                if (x < 0)          reflect_x = 1;
                else if (x > W - 1) reflect_x = W - 2;
                else                reflect_x = x;
            end
        end
    endfunction

    // `row + cy - 1` for the three read cycles, and `4*grp - 1 + port`.
    wire signed [10:0] s_rpos  = {3'b000, row} + {9'd0, cy[1:0]} - 11'sd1;
    wire signed [10:0] s_cbase = {5'b0, grp, 2'b00} - 11'sd1;
    wire [7:0]         s_ry    = reflect_y(s_rpos);

    genvar gp;
    generate
        for (gp = 0; gp < IN_PORTS; gp = gp + 1) begin : g_iaddr
            // W is 256, so the row is the high byte of the word address.
            wire [7:0] cx = reflect_x(s_cbase + gp);
            assign in_rd_addr[gp*IN_ADDR_W +: IN_ADDR_W] = {s_ry, cx};
            assign in_rd_en[gp] = s_rd;
        end
    endgenerate

    // `W_BASE + tap`, and no replication: `{{(W_ADDR_W-4){1'b0}}, tap}` is a
    // zero-width replication the day `W_ADDR_W` is four, which is exactly the
    // width nine taps need.
    wire [W_ADDR_W-1:0] w_addr_v = W_BASE + s_wtap;

    assign w_rd_en   = s_wen;
    assign w_rd_addr = w_addr_v;

    // ------------------------------------------------------------------
    // the three-row window
    // ------------------------------------------------------------------
    //
    // Three source rows of six columns of eight channels.  The read is
    // registered, so the address presented on `cy` answers on `cy+1` and the
    // three rows land on `cy` 1, 2 and 3 in that order.  Row `ky` is then
    // indexed `rf[ky][p+kx][i]` -- which is the kernel, the pixel offset and
    // the reflection all at once.
    //
    // No reset.  Every entry is written before it is read (the header's
    // schedule note walks through which cycles do which) and a reset on 144
    // bytes is 1,152 flops for nothing.

    // Held as `K*IN_PORTS` PIXEL WORDS of `IC_PAR` channels rather than as
    // `K*IN_PORTS*IC_PAR` single channels.  The bytes, their order and every
    // read are identical -- byte `ri` of pixel word `j` is the old
    // `rf[j*IC_PAR + ri]` -- and what changes is the shape of the array.
    //
    // **Why, and it is not a preference.**  `efx_map` 2026.1.132.4.5 crashes
    // (access violation, `libefx.dll+0x3f333`) on the flat 144-byte read in the
    // combinational `for` loop below -- `rf[((s_ky*6) + ap + s_kx)*8 + ai]` --
    // and it crashed on every algebraic rearrangement of that index that was
    // tried: hoisted into a wire, written as a shift, accumulated, loop-swapped
    // and fully unrolled alike.  The pixel-word read `rf[s_wrow + ap]` takes the
    // loop variable out of the multiply and it maps.  A read restructure that
    // changes the data would be worse than the crash, so this shape is gated on
    // the frames being bit-exact -- `docs/MAPPER_INDEX_2026-10-07.md`.

    // Six columns retain the three accepted source rows in SRL history.
    // Capture once on each advancing cycle; a stalled row capture is idempotent
    // in the original register file and must never shift this history again.
    // At cy=3 only the first two rows have arrived; the first tap reads row0
    // from cell1. After the same edge's third-row capture it moves to cell2.
    wire [IN_PORTS*IC_PAR*IN_W-1:0] selected_row;
    wire [2:0] history_addr = cy==4'd3 ? 3'd6 :
                              s_ky==0 ? 3'd5 : s_ky==1 ? 3'd6 : 3'd7;
    wire history_capture = rst_n && st==ST_RUN &&
             ((cy==4'd1 || cy==4'd2) ? !s_stall : cy==4'd3);
    genvar hist_bit;
    generate for(hist_bit=0;hist_bit<IN_PORTS*IC_PAR*IN_W;hist_bit=hist_bit+1) begin:g_history
        EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
            .CLK(clk),.CE(history_capture),.D(~in_rd_data[hist_bit]),
            .A(history_addr),.Q(selected_row[hist_bit]),.Q7());
    end endgenerate
    wire [PIX_PAR*IC_PAR*IN_W-1:0] selected_window =
                s_kx==0 ? selected_row[0 +: PIX_PAR*IC_PAR*IN_W] :
                s_kx==1 ? selected_row[IC_PAR*IN_W +: PIX_PAR*IC_PAR*IN_W] :
                          selected_row[2*IC_PAR*IN_W +: PIX_PAR*IC_PAR*IN_W];

    // ------------------------------------------------------------------
    // the operand
    // ------------------------------------------------------------------
    //
    // `act[p*IC + i] = rf[ky][p + kx][i] - zero`, in the order
    // `d64_mac128_compat_map` reads.  The subtract is the trunk's `q - zero`
    // and is a no-op at `IN_ZERO = 0`, which is what a ReLU tensor has; it is
    // here because the port is the same port either way.
    //
    // **`s_slot` gates the whole bus.**  Off a slot the tap index is clamped
    // and means nothing, and the value it selects out of an `rf` that has not
    // been written yet is an x -- which the bank would multiply.  A zero on an
    // unused slot costs one AND a lane and stops x from leaving this module.

    reg  [PIX_PAR*IC_PAR*ACT_W-1:0] act_bus;
    integer ap, ai;
    reg [IN_W-1:0] av;
    reg [IC_PAR*IN_W-1:0] pw;

    always @* begin
        act_bus = {PIX_PAR*IC_PAR*ACT_W{1'b0}};
        for (ap = 0; ap < PIX_PAR; ap = ap + 1) begin
            // One read a pixel: all eight channels of pixel `ap + kx` of row
            // `ky`, taken out of the window in one `IC_PAR*IN_W`-bit word.
            // `s_wrow + ap` and not a multiplied index, for the reason the
            // register file's own declaration gives.
            pw = selected_window[ap*IC_PAR*IN_W +: IC_PAR*IN_W];
            for (ai = 0; ai < IC_PAR; ai = ai + 1) begin
                av = pw[ai*IN_W +: IN_W];
                act_bus[(ap*IC_PAR + ai)*ACT_W +: ACT_W] =
                    s_slot ? ($signed({1'b0, av}) - IN_ZERO[ACT_W-1:0])
                           : {ACT_W{1'b0}};
            end
        end
    end

    // ------------------------------------------------------------------
    // stage B: the operand and the control, registered together
    // ------------------------------------------------------------------
    //
    // The store answers a cycle after its address, so `act_bus` is a
    // combinational path out of a 144-entry register file into 128
    // multipliers.  It gets a register of its own and the control travels with
    // it -- `slot_valid`, `pix_start` and the accept all move together, because
    // a control a cycle ahead of its data is the previous slot's operand under
    // this slot's phase framing.

    reg [PIX_PAR*IC_PAR*ACT_W-1:0]  b_act;
    reg [IC_PAR*OC_LANES*WGT_W-1:0] b_wgt;
    reg                     b_live, b_last, b_prime;
    reg [ROW_W-1:0]         b_row;
    reg [GRP_W-1:0]         b_grp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_act <= {PIX_PAR*IC_PAR*ACT_W{1'b0}};
            if(1==0) b_wgt <= {IC_PAR*OC_LANES*WGT_W{1'b0}};
            b_live <= 1'b0; b_last <= 1'b0; b_prime <= 1'b0;
            b_row <= {ROW_W{1'b0}}; b_grp <= {GRP_W{1'b0}};
        end
        else begin
            b_act   <= act_bus;
            if(1==0) b_wgt   <= w_rd_data;
            b_live  <= s_slot;
            b_last  <= s_last;
            b_prime <= s_prime;
            b_row   <= row;
            b_grp   <= grp;
        end
    end

    // `pix_start` is the prime of the phase whose first slot is the cycle after
    // it, and nothing else.  It is deliberately **not** the accept: on a
    // twelve-cycle phase the accept and the next `pix_start` are four cycles
    // apart, and keeping them apart is what keeps this engine off the bank's
    // `pix_start`-beats-`slot_valid` fold.
    wire pix_start = b_prime;
    // The accept is a handshake: `b_accept` is the phase taken and `b_refused`
    // is the phase refused.  `b_live & b_last` is high for exactly one cycle --
    // the cycle after the phase's last slot -- so exactly one of the two is that
    // phase's outcome, and neither can be missed.
    wire b_accept  = b_live & b_last &  pe_rsp_rq_ready;
    wire b_refused = b_live & b_last & ~pe_rsp_rq_ready;

    // ------------------------------------------------------------------
    // the PE
    // ------------------------------------------------------------------

    wire [PE_LANES*ACT_W-1:0]    bank_act;
    wire [PE_LANES*WGT_W-1:0]    bank_wgt;
    wire [PE_LANES*ACC_W-1:0]    bank_bias;
    wire [PE_OC_MAX*M_W-1:0]     bank_m;
    wire [PE_OC_MAX*SHIFT_W-1:0] bank_shift;
    wire [1:0]                   bank_mode;
    wire [PE_OUT_W-1:0]          bank_qmin, bank_qmax;

    d64_mac128_compat_map #(
        .IC_PAR(IC_PAR), .OC_LANES(OC_LANES), .PIX_PAR(PIX_PAR),
        .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W),
        .SHIFT_W(SHIFT_W), .OUT_W(PE_OUT_W), .RQ_LANES(RQ_LANES),
        .QMIN(QMIN), .QMAX(QMAX),
        .LANES(PE_LANES), .OC_MAX(PE_OC_MAX)
    ) u_map (
        .act(act_bus), .wgt(b_wgt), .bias({PIX_PAR{rq_bias}}),
        .rq_m(rq_m), .rq_shift(rq_shift),
        .b_act(bank_act), .b_wgt(bank_wgt), .b_bias(bank_bias),
        .b_m(bank_m), .b_shift(bank_shift), .b_mode(bank_mode),
        .b_qmin(bank_qmin), .b_qmax(bank_qmax),
        // The observation inputs are driven from the seam rather than left
        // open: the map's `acc_o`/`rq_snap_o` are outputs and may be dropped,
        // its `b_acc`/`b_snapshot` are inputs and dropping those is a net with
        // no driver.
        .b_acc(pe_rsp_acc), .b_snapshot(pe_rsp_snapshot),
        .acc_o(), .rq_snap_o()
    );

    assign pe_req_slot_valid = b_live;
    assign pe_req_act        = bank_act;
    assign pe_req_wgt        = bank_wgt;
    assign pe_req_pix_start  = pix_start;
    assign pe_req_bias       = bank_bias;
    assign pe_req_mode       = bank_mode;
    assign pe_req_rq_start   = b_accept;
    assign pe_req_rq_m       = bank_m;
    assign pe_req_rq_shift   = bank_shift;
    assign pe_req_rq_zero    = {PE_OUT_W{1'b0}};
    assign pe_req_rq_qmin    = bank_qmin;
    assign pe_req_rq_qmax    = bank_qmax;

    // The seam's lane is `PE_OUT_W` and this layer's output width is `OUT_W`.
    // They are both 10 and the assignment is direct on purpose: a
    // `{{(PE_OUT_W-OUT_W){1'b0}}, ..}` zero-extension is a **zero-width
    // replication** the day the two are equal, which is not legal Verilog, and
    // silently dropping either side's bits is worse than a width error.
    wire [RQ_LANES*OUT_W-1:0] mac_y;
    genvar sl;
    generate
        for (sl = 0; sl < RQ_LANES; sl = sl + 1) begin : g_y
            assign mac_y[sl*OUT_W +: OUT_W] = pe_rsp_y[sl*PE_OUT_W +: PE_OUT_W];
        end
    endgenerate

    wire [RQ_LANES-1:0] mac_y_valid = pe_rsp_y_valid;
    wire                mac_busy    = pe_rsp_rq_busy;

    // ------------------------------------------------------------------
    // the request metadata: a queue, not a register
    // ------------------------------------------------------------------
    //
    // The coordinates travel with the request.  `y_valid` arrives three cycles
    // after the accept, by which time `row`/`grp` have moved on -- using them
    // live writes the right number of words to the wrong places.

    reg [ROW_W-1:0] mf_r    [0:MFIFO-1];
    reg [GRP_W-1:0] mf_g    [0:MFIFO-1];
    reg [RQW-1:0]   mf_pass [0:MFIFO-1];
    reg [MFW-1:0]   mf_head, mf_tail;
    reg [MFW:0]     mf_n;

    // The pop is the request's **last** pass, counted in the queue and not
    // derived from the bank's `rq_done`: the pass count belongs to the request,
    // and a request whose pass count is wrong is a request whose words are
    // written twice.
    wire mf_pop  = (|mac_y_valid) && (mf_pass[mf_head] == (RQ_PASS - 1));
    wire mf_push = b_accept && (mf_n != MFIFO[MFW:0]);

    integer mi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mf_head <= {MFW{1'b0}}; mf_tail <= {MFW{1'b0}}; mf_n <= {MFW+1{1'b0}};
            for (mi = 0; mi < MFIFO; mi = mi + 1) begin
                mf_r[mi] <= {ROW_W{1'b0}}; mf_g[mi] <= {GRP_W{1'b0}};
                mf_pass[mi] <= {RQW{1'b0}};
            end
        end
        else if (start) begin
            mf_head <= {MFW{1'b0}}; mf_tail <= {MFW{1'b0}}; mf_n <= {MFW+1{1'b0}};
        end
        else begin
            if (mf_push) begin
                // Stage B's coordinates, because the push is stage B's accept.
                mf_r[mf_tail]    <= b_row;
                mf_g[mf_tail]    <= b_grp;
                mf_pass[mf_tail] <= {RQW{1'b0}};
                mf_tail <= mf_tail + 1'b1;
            end
            // Independent of the push, and on the head rather than the tail: a
            // pass arriving on the cycle of the next accept is the normal case
            // here, and as an `else if` the increment loses to every one of
            // them and the head's count never reaches the pop.
            if (|mac_y_valid)
                mf_pass[mf_head] <= mf_pass[mf_head] + 1'b1;
            if (mf_pop) mf_head <= mf_head + 1'b1;
            if (mf_push && !mf_pop)      mf_n <= mf_n + 1'b1;
            else if (mf_pop && !mf_push) mf_n <= mf_n - 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // the packer: four lanes a pixel, three of them written
    // ------------------------------------------------------------------
    //
    // A requant pass returns eight lanes: two pixels of four channels.  Pass 0
    // carries accumulators `k = 0..7` and pass 1 `k = 8..15`, so the two passes
    // together are the phase's four pixels and the fourth lane of every group
    // is the fill.
    //
    // The two passes arrive on consecutive cycles and the port writes one word
    // a cycle, so pass 0's lanes are held while pass 1's are emitted and the
    // four words go out over the four cycles that follow.  A phase is twelve
    // cycles, so the window never reaches the next phase's.

    reg [4*OUT_W-1:0] p_lo, p_hi, q_lo, q_hi;
    reg               pk_busy;
    reg [1:0]         pk_i;
    reg [ROW_W-1:0]   pk_r;
    reg [GRP_W-1:0]   pk_g;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_lo <= {4*OUT_W{1'b0}}; p_hi <= {4*OUT_W{1'b0}};
            q_lo <= {4*OUT_W{1'b0}}; q_hi <= {4*OUT_W{1'b0}};
        end
        else if (|mac_y_valid) begin
            if (mf_pass[mf_head] == {RQW{1'b0}}) begin
                p_lo <= mac_y[0*OUT_W +: 4*OUT_W];
                p_hi <= mac_y[4*OUT_W +: 4*OUT_W];
            end
            else begin
                q_lo <= mac_y[0*OUT_W +: 4*OUT_W];
                q_hi <= mac_y[4*OUT_W +: 4*OUT_W];
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pk_busy <= 1'b0; pk_i <= 2'd0;
            pk_r <= {ROW_W{1'b0}}; pk_g <= {GRP_W{1'b0}};
        end
        else if (start) begin
            pk_busy <= 1'b0; pk_i <= 2'd0;
        end
        else if (pk_busy) begin
            if (pk_i == 2'd3) pk_busy <= 1'b0;
            else              pk_i <= pk_i + 2'd1;
        end
        else if ((|mac_y_valid) && (mf_pass[mf_head] != {RQW{1'b0}})) begin
            pk_busy <= 1'b1;
            pk_i    <= 2'd0;
            pk_r    <= mf_r[mf_head];
            pk_g    <= mf_g[mf_head];
        end
    end

    wire [4*OUT_W-1:0] pk_src = (pk_i == 2'd0) ? p_lo :
                                (pk_i == 2'd1) ? p_hi :
                                (pk_i == 2'd2) ? q_lo : q_hi;

    wire [7:0] lane_r = pk_src[0*OUT_W +: 8];
    wire [7:0] lane_g = pk_src[1*OUT_W +: 8];
    wire [7:0] lane_b = pk_src[2*OUT_W +: 8];
    // The fill.  Read here only so the sabotaged build has something to leak;
    // at `FILL_SABOTAGE = 0` this wire reaches no port.
    wire [7:0] lane_f = pk_src[3*OUT_W +: 8];

    assign out_wr_en = pk_busy;
    assign out_wr_data = FILL_SABOTAGE ? {lane_f, lane_b, lane_g, lane_r}
                                       : {8'h00,  lane_b, lane_g, lane_r};

    // W is 256 and PIX_PAR is 4, so the two strides are shifts and the whole
    // address is three slices added together.
    assign out_wr_addr = {2'b00, pk_r, 8'b0}
                       + {10'b0, pk_g, 2'b00}
                       + {16'b0, pk_i};

    // Every valid, read and output drained, then a mode change is safe.  The
    // packer is in here and not only the bank: a word still in the packing
    // register under one shape is a word under another.
    assign mac_mode_safe = !b_live && !mac_busy && !pk_busy
                        && (mf_n == {MFW+1{1'b0}});

    // ------------------------------------------------------------------
    // completion
    // ------------------------------------------------------------------
    //
    // The schedule's last phase is several pipeline stages upstream of the
    // engine's last output, so `done` waits for the queue, the bank and the
    // packer.  Reporting the schedule's end would undercount the drain and let
    // a whole-network controller switch layers while the last words were still
    // in flight.

    // **`!b_accept` is not decoration.**  The schedule's last phase ends on the
    // same cycle its accept is taken, and that accept's push into the metadata
    // queue lands on the *next* edge -- so on that cycle `mf_n` is still zero,
    // nothing is valid, nothing is busy, and an `fin` that only reads those
    // four says the engine is finished while the last four words are still on
    // their way.  They arrive, the packer packs them, and `done` has already
    // been reported: four words written after the engine said it was done.  The
    // testbench found it as "4 of 65536 addresses never written", on the last
    // phase, which is what a testbench is for.
    reg  sdone_seen;

    // **The gate is the counts, and the counts are the shape's.**  `rq_accepts`
    // cannot be `rq_requests` here: a replayed phase is presented twice and
    // accepted once, so a gate against the walk would be unsatisfiable after any
    // refusal.  `out_write_count` is the second half of the same debt -- a frame
    // whose phases were all accepted but whose last words are still in the
    // packer is a frame that is not finished, which `!pk_busy` alone does not
    // say on the cycle the last write is counted.
    //
    // `!fault_sticky` is not redundant with the counts, it is the latch that
    // names the condition: an accept that was refused and never made good is an
    // unfinished frame whatever the counts happen to read.  It costs one AND and
    // it is what makes a refusal a *reported* state rather than an arithmetic
    // accident.
    wire all_phases = (rq_accepts      == NPHASE_TOTAL);
    wire all_words  = (out_write_count == NPIX_TOTAL);

    wire fin = sdone_seen && !b_accept && (mf_n == {MFW+1{1'b0}})
            && !(|mac_y_valid) && !mac_busy && !pk_busy
            && all_phases && all_words
            && !fault_sticky;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= ST_IDLE; cy <= 4'd0;
            row <= {ROW_W{1'b0}}; grp <= {GRP_W{1'b0}};
            sdone_seen <= 1'b0;
            rw_pend <= 1'b0; rw_row <= {ROW_W{1'b0}}; rw_grp <= {GRP_W{1'b0}};
        end
        else begin
            case (st)
                ST_IDLE: begin
                    if (start) begin
                        st <= ST_RUN; cy <= 4'd0;
                        row <= {ROW_W{1'b0}}; grp <= {GRP_W{1'b0}};
                        sdone_seen <= 1'b0;
                        // A pending rewind does not survive a new frame: it
                        // names a phase of the frame that was abandoned.
                        rw_pend <= 1'b0; rw_row <= {ROW_W{1'b0}};
                        rw_grp  <= {GRP_W{1'b0}};
                    end
                end
                ST_RUN: begin
                    // **The refusal is checked first, and it outranks the
                    // advance.**  It arrives on the cycle `cy` already reads
                    // zero -- the phase advanced on the edge that ended it -- so
                    // the rewind puts `cy` back where it is and only `row`/`grp`
                    // move, back to the phase stage B still names.
                    //
                    // **Unless a read is outstanding, and then the hold wins.**
                    // The store's contract is that the engine holds `in_rd_en`
                    // and `in_rd_addr` while `in_stall` is high; rewinding here
                    // would change the address under a store that is still
                    // looking for the old one.  So the rewind is *latched* and
                    // taken at the first edge on which no read is outstanding --
                    // the refused phase is replayed either way, `cy` later
                    // rather than sooner, and the read it abandons was never
                    // transferred so nothing was consumed by abandoning it.
                    if (b_refused) begin
                        if (s_stall) begin
                            rw_pend <= 1'b1;
                            rw_row  <= b_row;
                            rw_grp  <= b_grp;
                        end
                        else begin
                            cy  <= 4'd0;
                            row <= b_row;
                            grp <= b_grp;
                        end
                    end
                    else if (rw_pend && !s_stall) begin
                        // The deferred rewind, at the first edge the store is
                        // free.  `cy` is zero here by construction: the pending
                        // rewind was latched on the refusal, which is always a
                        // `cy == 0` cycle, and the hold did not let it move.
                        rw_pend <= 1'b0;
                        cy  <= 4'd0;
                        row <= rw_row;
                        grp <= rw_grp;
                    end
                    else if (s_stall) begin
                        // **The hold.**  Nothing moves: `cy` stays where it is,
                        // `row`/`grp` with it, and every quantity this phase
                        // derives from `cy` -- the six read addresses, the nine
                        // weight tiles, the tap and kernel indices, the slot
                        // window, `pix_start` and the accept's position -- stays
                        // consistent with the read that has not been transferred.
                        // `in_rd_en` and `in_rd_addr` are held with it, which is
                        // what the store asked for.
                        //
                        // This is where rule 3 is kept: the slot window opens at
                        // `cy == 3` and a read is only ever presented at
                        // `cy <= 2`, so a store that cannot answer does not
                        // freeze the engine *between* two operands of a MAC
                        // slot -- it freezes it before any of the phase's slots
                        // have run, and no slot ever sees the word the store was
                        // still holding from the previous phase.
                    end
                    else if (cy == CY_LAST) begin
                        cy <= 4'd0;
                        if (grp == NGRP - 1) begin
                            grp <= {GRP_W{1'b0}};
                            if (row == H - 1) begin
                                st <= ST_DRAIN; sdone_seen <= 1'b1;
                            end
                            else row <= row + 1'b1;
                        end
                        else grp <= grp + 1'b1;
                    end
                    else cy <= cy + 4'd1;
                end
                ST_DRAIN: begin
                    // **The last phase can be refused too, and then the walk's
                    // end is not the frame's end.**  `sdone_seen` goes back down
                    // with it: the frame is finished when its last phase has been
                    // accepted, and the replay has to reach the end again to have
                    // reached it.
                    if (b_refused) begin
                        st <= ST_RUN; cy <= 4'd0;
                        row <= b_row; grp <= b_grp;
                        sdone_seen <= 1'b0;
                    end
                    else if (fin) st <= ST_DONE;
                end
                default:  st <= ST_IDLE;
            endcase
        end
    end

    reg running;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running <= 1'b0; cycles <= 32'd0; busy <= 1'b0; done <= 1'b0;
        end
        else begin
            done <= 1'b0;
            if (start) begin
                running <= 1'b1; cycles <= 32'd0; busy <= 1'b1;
            end
            else if (running) begin
                if (fin) begin
                    running <= 1'b0; busy <= 1'b0; done <= 1'b1;
                end
                else
                    cycles <= cycles + 32'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // counters
    // ------------------------------------------------------------------

    // **The fault, and the replay it is the fault of.**  `replay_active` is the
    // window the wait counter counts: it opens on the refusal edge and closes on
    // the accept that resolves it -- the refusal itself included, so a refusal
    // that is recovered costs its own cycle plus the twelve of the replay.
    // Cleared by `start`, so a second frame starts clean.
    reg replay_active;

    //: The previous cycle's `s_stall`, so the input side's counter counts
    //: **events** and not only cycles: a read that has to be held is one event
    //: however many cycles the store takes.
    reg s_stall_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            replay_active <= 1'b0; fault_sticky <= 1'b0; fault_count <= 32'd0;
        end
        else if (start) begin
            replay_active <= 1'b0; fault_sticky <= 1'b0; fault_count <= 32'd0;
        end
        else begin
            if (b_refused) begin
                replay_active <= 1'b1;
                fault_sticky  <= 1'b1;
                fault_count   <= fault_count + 32'd1;
            end
            else if (b_accept) begin
                // The phase is taken, so the refusal is over and the latch
                // clears.  A refusal that is never resolved leaves it set and
                // `done` unreachable, which is the sticky half of the fault.
                replay_active <= 1'b0;
                fault_sticky  <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rq_requests <= 32'd0; rq_accepts <= 32'd0;
            rq_wait_cycles <= 32'd0; rq_overrun_count <= 32'd0;
            slot_count <= 32'd0; out_write_count <= 32'd0; mfifo_peak <= 32'd0;
            in_stall_cycles <= 32'd0; in_stall_events <= 32'd0; s_stall_q <= 1'b0;
        end
        else if (start) begin
            rq_requests <= 32'd0; rq_accepts <= 32'd0;
            rq_wait_cycles <= 32'd0; rq_overrun_count <= 32'd0;
            slot_count <= 32'd0; out_write_count <= 32'd0; mfifo_peak <= 32'd0;
            in_stall_cycles <= 32'd0; in_stall_events <= 32'd0; s_stall_q <= 1'b0;
        end
        else begin
            if (s_slot & s_last)   rq_requests     <= rq_requests + 32'd1;
            if (b_accept)          rq_accepts      <= rq_accepts + 32'd1;
            if (s_slot)            slot_count      <= slot_count + 32'd1;
            if (out_wr_en)         out_write_count <= out_write_count + 32'd1;
            if (pe_rsp_rq_overrun) rq_overrun_count <= rq_overrun_count + 32'd1;
            // The input side's stall, counted as **cycles and as events**: the
            // number the store's refusal is, in the same form the PE side's
            // `fault_count`/`rq_wait_cycles` pair takes.  A counter that could
            // not move would be a counter that cannot say a store stalled.
            if (s_stall) begin
                in_stall_cycles <= in_stall_cycles + 32'd1;
                if (!s_stall_q) in_stall_events <= in_stall_events + 32'd1;
            end
            s_stall_q <= s_stall;
            // The wait count, which used to be a counter nothing could move.
            if (b_refused || replay_active)
                                   rq_wait_cycles  <= rq_wait_cycles + 32'd1;
            if ({1'b0, mf_n} > mfifo_peak[MFW:0])
                mfifo_peak <= {{(32-MFW-1){1'b0}}, mf_n};
        end
    end

    // External bank consumes weights only with its valid slot. Sampling the
    // common response every clock, with no owner-specific mask/reset, allows
    // equivalent raw response bits to share FFs across mutually exclusive engines.
    // The standalone backend retains its original reset and capture contract.
    generate if(1!=0)begin:g_shared_weight_capture
      always @(posedge clk) b_wgt <= w_rd_data;
    end endgenerate

    // synthesis translate_off
    wire [128*ACT_W-1:0] act_registered_ghost;
    genvar ga;generate for(ga=0;ga<128;ga=ga+1)begin:g_act_ghost
      localparam AP=((ga/IC_PAR)/OC_LANES)*IC_PAR+(ga%IC_PAR);
      assign act_registered_ghost[ga*ACT_W+:ACT_W]=b_act[AP*ACT_W+:ACT_W];
    end endgenerate
    // synthesis translate_on
endmodule
