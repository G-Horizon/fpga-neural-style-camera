`timescale 1ns/1ps

// D3c4b -- the transposed convolution's memory schedule, one layer at a time.
//
// D3c4a proved the arithmetic: four phases, 1/2/2/4 taps, no kernel flip, and a
// tap whose input index leaves the image simply does not exist.  What it did not
// do is say whether either layer can actually be fed.  This does, with no
// multiplier anywhere -- the MAC and the requant are a token pipeline, because the
// question is where every read and write goes on every cycle.
//
// The two layers are the same circuit at different shapes:
//
//             in        out       PIX  IC  OC   IC groups  cycles/pixel
//   ups.0   64x64x32  128x128x16   1    8  16       4      1+2+2+4 taps x 4
//   ups.1  128x128x16 256x256x8    2    8   8       2      ... x 2, per pair
//
// so the fixed slot is 36 cycles per input pixel for ups.0 and 18 per input pixel
// pair for ups.1, both 147,456 for the layer.
//
// The row-pair order
// ------------------
//
// The phases could be run whole-frame: all EE, then all EO.  That makes the output
// addresses stride by two, which is the wrong thing to hand an external memory.
// Instead the loop is `for r, for c, for phase`.  One input row therefore produces
// exactly output rows 2r and 2r+1, and the two-row scratch absorbs the phase
// interleaving so what leaves for off chip is two contiguous rows.
//
// The scratch is the same 32,768 bits in both layers -- 2 x 128 x 16 x 8 for
// ups.0 and 2 x 256 x 8 x 8 for ups.1 -- which is also exactly the encoder's
// worst-case line buffer.  Four RAM10, reused rather than added to M1.
//
// Border slots are spent, not skipped
// -----------------------------------
//
// The fixed schedule gives every tap its slot, including the taps that fall off
// the image at the bottom row and right column.  Those slots cost 2,299 cycles
// across the two layers, 1.6% of the pair, and recovering them would mean a second
// border-specialised schedule for 0.8%.  What they do NOT do is read: an invalid
// tap issues no input read and no weight read, which is what the testbench checks.

// The store's refusal is not a freeze
// -----------------------------------
//
// `stall` freezes the schedule; `in_stall` says the slot already on the wires has
// not been taken.  They are two different things and the second is the one a
// cache needs (`control/d64_burst_prefetch.v` can miss).  While `in_stall` is
// high the schedule holds -- and it holds **the whole slot**: the enable, the
// address, the phase, the tap, the group, the coordinates and the port mask,
// because those are what turn a word into the right operand and a re-presented
// address would otherwise take them from the slot that follows it.  The slot is
// replayed when the store finally takes it, and `cur_issue` is what says so: it
// is combinational and means "the store took the slot on the wires this cycle",
// which is the edge the engine registers the slot on.
//
// Measured against the real cache at one and two ports, with the tooth that
// fails on data when the hold is dropped: `docs/TCONV_INPUT_STALL_2026-10-07.md`.
module d64_tconv_scheduler #(
    parameter H          = 64,
    parameter W          = 64,
    parameter PIX_PAR    = 1,        // input columns consumed per step
    parameter IC_GROUPS  = 4,
    parameter OC_GROUPS  = 1,
    parameter OC_LANES   = 16,
    parameter IN_PORTS   = 1,        // input words read per cycle
    parameter WORD_CH    = 8,        // channels per input and output word
    parameter IN_ADDR_W  = 14,
    parameter W_BASE     = 184,      // this layer's tile base in the weight store
    parameter W_ADDR_W   = 8,
    parameter OUT_W      = 128,
    parameter OUT_ADDR_W = 16,
    parameter REQ_LANES  = 8
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    output reg                     busy,
    output reg                     done,
    output reg  [31:0]             cycles,

    // input feature reads.  ``in_rd_valid`` is the tap existing at all; a slot
    // whose tap is off the image asserts neither.
    output reg  [IN_PORTS-1:0]     in_rd_en,
    output reg  [IN_PORTS-1:0]     in_rd_valid,
    output reg  [IN_PORTS*IN_ADDR_W-1:0] in_rd_addr,

    // one weight tile per MAC cycle
    output reg                     w_rd_en,
    output reg  [W_ADDR_W-1:0]     w_rd_addr,

    // the two-row output scratch: at most one word per cycle
    output reg                     out_wr_en,
    output reg  [OUT_ADDR_W-1:0]   out_wr_addr,
    output reg                     out_wr_row,

    // requant: how many channel values retire this cycle
    output reg                     rq_valid,
    output reg  [7:0]              rq_values,

    // observability, so a checker can rebuild the phase table independently
    output reg  [1:0]              cur_phase,
    output reg  [1:0]              cur_tap,
    output reg  [3:0]              cur_icg,
    output reg  [7:0]              cur_r,
    output reg  [7:0]              cur_c,
    output reg  [1:0]              cur_kh,
    output reg  [1:0]              cur_kw,
    output reg  [7:0]              cur_in_col0,
    output reg  [63:0]             out_cur_addr,

    // counters
    output reg  [31:0]             mac_cycles,
    output reg  [31:0]             in_reads,
    output reg  [31:0]             w_reads,
    output reg  [31:0]             out_writes,
    output reg  [31:0]             rq_total,
    output reg  [7:0]              rq_peak,
    output reg  [31:0]             border_slots,
    output reg  [31:0]             emitted,
    output reg  [7:0]              rq_pend_o,
    output reg  [1:0]              dbg_st,

    // ---- the engine's side of the contract ---------------------------------
    //
    // `cur_*` already trails the address outputs by one cycle, so it describes
    // the slot whose address was presented on the previous edge.  These two add
    // the two facts about that slot the MAC needs and cannot derive without
    // re-deriving the phase table, which the table exists to prevent.
    output reg                     cur_live,   // the slot does work
    output reg                     cur_last,   // its phase's last IC group
    // True when the NEXT slot ends a phase.  The engine needs one cycle of
    // warning: stalling on the phase-end slot itself is too late, because that
    // slot is the one carrying the final MAC contribution.
    output wire                    next_last,
    // **The store took the slot on the wires this cycle.**  Combinational, and
    // it has to be: what it says is decided by the store's answer in the cycle
    // the slot is presented, and a slot the store *refuses* in that cycle must
    // not be registered at all -- registering it pairs the slot's control with
    // the store's previous word.  `dbg_st == S_RUN` cannot serve the purpose
    // either: while held the state is still S_RUN and `cur_*` still describes the
    // slot on the wires, so a downstream register keyed on the state would
    // capture that slot twice and the MAC would accumulate it twice.
    output wire                    cur_issue,
    output reg                     cur_dy,      // scratch row this slot writes

    // Freezes the schedule.  The engine must not accept a requant request on a
    // different cycle from the one that completes a phase -- that is the
    // rollover -- so when the MAC cannot take it the schedule has to stop
    // rather than run ahead and lose the accumulator.  In the steady state this
    // is never asserted, and the engine counts the cycles it is, because
    // "never" is a claim and a counter is evidence.
    input wire write_phase_ready,
    output wire write_phase_reserve,
    output wire [8:0] write_phase_row,
    output wire [15:0] write_phase_addr,
    input  wire                    stall,

    // ---- the store's side of the same handshake -----------------------------
    //
    // **Not the same signal as `stall`.**  `stall` means "do not advance"; this
    // means "the slot already on the wires has not been taken, so present it
    // again".  A store made of RAM always takes a read, so this port did not
    // exist until the input store became a cache -- see
    // `control/d64_burst_prefetch.v`, whose whole reason to exist is that a
    // cache that cannot stall is a cache that returns the wrong word quietly.
    //
    // While this is high the schedule does not move, the enable, the address and
    // **the metadata that describes the slot** stay on the wires, and the slot is
    // replayed when the store finally takes it.  That is rule 1 of
    // `docs/F3_INTERFACES_2026-10-07.md` section 6.9: holding the address alone
    // is not enough, because the activation, the weight, the phase and the
    // last-MAC-slot relationship are what turn the word into the right operand
    // -- and a phase whose reads were half accepted is replayed whole.
    //
    // A build that folds this into `stall` and nothing else loses the refused
    // read and computes the slot on the store's previous word.  Measured, with
    // the tooth that fires, in `docs/TCONV_INPUT_STALL_2026-10-07.md`.
    //
    // **`z` is not a stall.**  A testbench that predates this port leaves it
    // floating, and a floating input read through a `case` is no match for either
    // arm of it -- the same shape `d64_burst_prefetch` relies on for an
    // unconnected `region_req`.  Nothing else reads `in_stall`; the sanitised copy
    // below (`store_hold`) is what the schedule uses, and it is a `case` and not
    // an `if` for exactly this reason.
    input  wire                    in_stall
);

    // ---- the phase table, from D3c4a --------------------------------------
    //
    // One descriptor per (phase, tap): {dy, dx, kh, kw}.  Written out rather than
    // computed, because the whole point of D3c4a was that the mapping from phase
    // to kernel index is not a formula anyone should re-derive in RTL.
    function [5:0] tap_desc(input [1:0] ph, input [1:0] t);
        begin
            case ({ph, t})
                4'd0:  tap_desc = {1'b0, 1'b0, 2'd1, 2'd1};   // EE  (1,1)
                4'd4:  tap_desc = {1'b0, 1'b0, 2'd1, 2'd2};   // EO  (1,2)
                4'd5:  tap_desc = {1'b0, 1'b1, 2'd1, 2'd0};   // EO  (1,0)
                4'd8:  tap_desc = {1'b0, 1'b0, 2'd2, 2'd1};   // OE  (2,1)
                4'd9:  tap_desc = {1'b1, 1'b0, 2'd0, 2'd1};   // OE  (0,1)
                4'd12: tap_desc = {1'b0, 1'b0, 2'd2, 2'd2};   // OO  (2,2)
                4'd13: tap_desc = {1'b0, 1'b1, 2'd2, 2'd0};   // OO  (2,0)
                4'd14: tap_desc = {1'b1, 1'b0, 2'd0, 2'd2};   // OO  (0,2)
                4'd15: tap_desc = {1'b1, 1'b1, 2'd0, 2'd0};   // OO  (0,0)
                default: tap_desc = 6'd0;
            endcase
        end
    endfunction

    function [2:0] tap_count(input [1:0] ph);
        begin
            case (ph)
                2'd0:    tap_count = 3'd1;    // EE
                2'd1:    tap_count = 3'd2;    // EO
                2'd2:    tap_count = 3'd2;    // OE
                default: tap_count = 3'd4;    // OO
            endcase
        end
    endfunction

    // Output row of the pair: the phases with dy = 1 write the odd output row, so
    // the descriptor's dy *is* the scratch row select.
    localparam integer OC_WORDS = (OC_LANES + WORD_CH - 1) / WORD_CH;
    localparam integer OUT_WORDS = PIX_PAR * OC_WORDS;
    localparam integer LAST_ROW  = H - 1;
    localparam integer LAST_COL  = W - 1;

    localparam S_IDLE = 2'd0, S_RUN = 2'd1, S_DRAIN = 2'd2, S_DONE = 2'd3;

    reg [1:0]  st;
    reg [1:0]  ph;
    reg [1:0]  tp;
    reg [3:0]  icg;
    reg [7:0]  r, c;

    // ---- the hold ------------------------------------------------------------
    //
    // The scheduler offers a slot on `in_rd_en`/`in_rd_addr` and the store either
    // takes it (`in_stall` low: its word is on the store's data output on the
    // next cycle) or refuses it (`in_stall` high: nothing is transferred and the
    // address is expected to stay).  The schedule cannot advance past a refusal,
    // and the **whole** slot has to be held and not just the address -- the
    // metadata that describes the slot is what makes its word the right operand,
    // and a re-presented address would otherwise take that metadata from the
    // slot that follows it.
    //
    // Read through a copy, so that an unconnected port is "no stall" and a build
    // that predates it is bit-for-bit the build it was.
    //
    // **A `case` and not an `if`.**  Measured in this simulator: a combinational
    // `if` on a `z` input leaves its target `x` (`if (z)` is not `if (0)`), and
    // an `x` here walks into `cur_issue` and from there into the engine's
    // pipeline.  A `case` with a `default` reads `z` as "no item matches" and
    // takes the default.  `pend` is in the sensitivity list so that the block
    // settles even when `in_stall` is a constant `z`: an `always @*` whose only
    // input never changes is never evaluated.
    reg store_hold;
    always @(in_stall or pend) begin
        case (in_stall)
            1'b1:    store_hold = 1'b1;
            default: store_hold = 1'b0;
        endcase
    end

    // A slot is on the wires and the store has not taken it yet.  Set where a
    // slot is presented and cleared on the cycle the store takes it, which is the
    // cycle the engine registers the slot -- the store's word follows the address
    // by one cycle, so the control that travels with the address has to be
    // registered on the same edge as the word.  Not cleared while a mac-side
    // `stall` pauses the schedule after a transfer: the slot is taken, and what
    // is left on the wires under that pause is dead.
    reg pend;

    assign cur_issue = pend & ~store_hold;

    wire [5:0] desc  = tap_desc(ph, tp);
    wire       dy    = desc[5];
    wire       dx    = desc[4];
    wire [1:0] kh    = desc[3:2];
    wire [1:0] kw    = desc[1:0];
    wire [2:0] n_tap = tap_count(ph);

    // The slot that follows the current (tp, icg), or none if this one is last.
    wire        adv_tp   = (icg == IC_GROUPS - 1);
    wire [1:0]  nxt_tp   = adv_tp ? tp + 2'd1 : tp;
    wire [3:0]  nxt_icg  = adv_tp ? 4'd0 : icg + 4'd1;
    wire        nxt_ex   = !((icg == IC_GROUPS - 1) && (tp == n_tap - 1));
    assign next_last = nxt_ex && (nxt_icg == IC_GROUPS - 1)
                              && (nxt_tp == n_tap - 1);

    // the input row and the first column this tap reads
    wire [8:0] src_row = {1'b0, r} + {8'd0, dy};
    wire [8:0] src_col = {1'b0, c} + {8'd0, dx};
    wire       row_ok  = (src_row <= LAST_ROW);
    // A slot whose first port is off the image does no work: no input read and no
    // weight read.  With PIX_PAR = 2 the second port can be off while the first is
    // on, and then the slot is still live for the first pixel.
    wire slot_live = row_ok && (src_col <= LAST_COL);
    // Only a phase's first slot needs a panel reservation. An issued read
    // still transfers and the A/B/PE pipeline drains while the next phase waits.
    wire first_phase_slot = (tp==0 && icg==0);
    wire write_hold = first_phase_slot && (write_phase_ready===1'b0);
    assign write_phase_row = {r,1'b0}+ph[1];
    assign write_phase_addr = (({8'd0,c}<<1)+ph[0])*OC_WORDS;
    assign write_phase_reserve = (st==S_RUN) && !store_hold && !stall && !write_hold && first_phase_slot;

    wire [IN_ADDR_W-1:0] row_w = src_row;
    wire [IN_ADDR_W-1:0] col_w = src_col;

    // ---- deferred output, exactly as the trunk's write-back is deferred -----
    //
    // A phase's accumulators are only complete when its last IC group lands, so
    // the words are queued and written during the next phase's slot.  Both layers
    // have at least PIX_PAR cycles of slot left over, so the queue drains without
    // costing a cycle anywhere except at the very end of the layer.
    reg [3:0]  out_pend;
    reg [2:0]  out_idx;
    reg [7:0]  out_r, out_c;
    reg        out_row;      // scratch row: the phase's dy, so 2r or 2r+1
    reg        out_odx;      // the phase's column parity, 0 for EE/OE, 1 for EO/OO

    // the requant queue: the MAC emits a whole phase's outputs in one cycle, and
    // REQ_LANES of them retire per cycle afterwards.
    reg [7:0]  rq_pend;
    reg [7:0]  rq_room;
    reg        rq_stall;
    wire [7:0] rq_issue = (rq_pend > REQ_LANES[7:0]) ? REQ_LANES[7:0] : rq_pend;
    wire [7:0] rq_next  = rq_pend - rq_issue;
    wire [7:0] rq_add   = PIX_PAR * OC_LANES[7:0];
    wire [7:0] rq_fill  = rq_next + rq_add;

    // Scratch address of the word being drained.  The output column is
    // 2 * (input column) + the phase's column parity, not the input column: the
    // transposed convolution doubles the grid, and writing at the input column
    // still covers the scratch exactly once if the parity is dropped -- it is a
    // bijection either way, which is why only a duplicate/hole count catches it.
    wire [15:0] out_pix  = {8'd0, out_c} + (out_idx / OC_WORDS);
    wire [15:0] out_col  = (out_pix << 1) + {15'd0, out_odx};
    wire [15:0] out_word = out_col * OC_WORDS + (out_idx % OC_WORDS);
    wire [15:0] out_addr = out_row * OUT_W * OC_WORDS + out_word;

    integer i;
    reg [31:0] rd_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0; cycles <= 32'd0; st <= S_IDLE;
            in_rd_en <= 0; in_rd_valid <= 0; in_rd_addr <= 0;
            w_rd_en <= 1'b0; w_rd_addr <= 0;
            out_wr_en <= 1'b0; out_wr_addr <= 0; out_wr_row <= 1'b0;
            rq_valid <= 1'b0; rq_values <= 8'd0;
            ph <= 2'd0; tp <= 2'd0; icg <= 4'd0; r <= 8'd0; c <= 8'd0;
            cur_phase <= 2'd0; cur_tap <= 2'd0; cur_icg <= 4'd0;
            cur_r <= 8'd0; cur_c <= 8'd0; cur_kh <= 2'd0; cur_kw <= 2'd0;
            cur_in_col0 <= 8'd0; out_cur_addr <= 64'd0;
            mac_cycles <= 32'd0; in_reads <= 32'd0; w_reads <= 32'd0;
            out_writes <= 32'd0; rq_total <= 32'd0; rq_peak <= 8'd0;
            border_slots <= 32'd0; emitted <= 32'd0;
            out_pend <= 4'd0; out_idx <= 3'd0;
            out_r <= 8'd0; out_c <= 8'd0; out_row <= 1'b0; out_odx <= 1'b0;
            rq_pend <= 8'd0; rq_room <= 8'd0; rq_stall <= 1'b0;
            rq_pend_o <= 8'd0; dbg_st <= S_IDLE;
            cur_live <= 1'b0; cur_last <= 1'b0; pend <= 1'b0;
            cur_dy <= 1'b0;
        end
        else begin
            out_wr_en <= 1'b0; rq_valid <= 1'b0; rq_values <= 8'd0;
            // One-per-slot pulses, cleared here and set in S_RUN.  Clearing them
            // only in S_RUN's else-branch left them holding the last slot's
            // values through S_DRAIN and S_DONE, so a downstream consumer saw
            // the final phase-end arrive again on every cycle of the drain.
            //
            // **A refused slot suspends the clearing of the slot itself.**  The
            // address, the port mask and the presentation's metadata are assigned
            // only where a slot is presented, so a hold keeps them by
            // construction; the two enables and the three per-slot bits are what
            // it would otherwise lose.  `pend` is not touched here: a presented
            // slot stays pending until the store takes it.
            if (!store_hold) begin
                in_rd_en <= 0; w_rd_en <= 1'b0;
                cur_live <= 1'b0; cur_last <= 1'b0; cur_dy <= 1'b0;
            end
            done <= 1'b0;
            if (busy) cycles <= cycles + 32'd1;
            rq_pend_o <= rq_pend;
            dbg_st <= st;

            // ---- the queues retire every cycle, MAC or not ------------------
            if (rq_pend != 8'd0) begin
                rq_valid  <= 1'b1;
                rq_values <= rq_issue;
                rq_pend   <= rq_pend - rq_issue;
                rq_total  <= rq_total + {24'd0, rq_issue};
            end
            if (out_pend != 4'd0) begin
                out_wr_en   <= 1'b1;
                out_wr_addr <= out_addr[OUT_ADDR_W-1:0];
                out_wr_row  <= out_row;
                out_cur_addr <= {48'd0, out_addr};
                out_writes  <= out_writes + 32'd1;
                out_pend    <= out_pend - 4'd1;
                out_idx     <= out_idx + 3'd1;
            end

            case (st)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1; cycles <= 32'd0;
                        ph <= 2'd0; tp <= 2'd0; icg <= 4'd0;
                        r <= 8'd0; c <= 8'd0;
                        mac_cycles <= 0; in_reads <= 0; w_reads <= 0;
                        out_writes <= 0; rq_total <= 0; rq_peak <= 0;
                        border_slots <= 0; emitted <= 0;
                        out_pend <= 0; out_idx <= 0; rq_pend <= 0; rq_stall <= 0;
                        pend <= 1'b0;
                        st <= S_RUN;
                    end
                end

                // One slot: one (phase, tap, IC group) triple.
                //
                // Three ways through it, and they are not the same thing: the
                // store refused the slot on the wires (hold it and present it
                // again, whole), the schedule is free (present the next one), or
                // the slot was taken and the requant refused it (pause).
                S_RUN: begin
                    if (store_hold) begin
                        // ---- the store refused the slot on the wires --------
                        // Nothing is assigned here, on purpose.  The state does
                        // not move, `pend` stays set, and the address, the
                        // enables and every metadata bit keep the values they
                        // were presented with, because the clear at the top of
                        // this block is suppressed for the hold.  The slot is not
                        // lost and the phase it belongs to is replayed whole,
                        // which is the point: half a phase's reads accepted is
                        // not half a phase.
                    end
                    else if (!stall && !write_hold) begin
                        cur_dy  <= dy;
                        cur_live <= slot_live;
                        cur_last <= (icg == IC_GROUPS - 1) && (tp == n_tap - 1);
                        cur_phase <= ph; cur_tap <= tp; cur_icg <= icg;
                        cur_r <= r; cur_c <= c;
                        cur_kh <= kh; cur_kw <= kw;
                        cur_in_col0 <= src_col[7:0];
                        mac_cycles <= mac_cycles + 32'd1;
                        // The slot is on the wires from the next edge until the
                        // store takes it.  Set here, cleared either where the
                        // store takes it or where a mac-side stall pauses the
                        // schedule with the wires already cleared.
                        pend <= 1'b1;

                        // Input reads.  Port p serves input pixel c + p, so with
                        // PIX_PAR = 2 the pair is two adjacent columns and both words
                        // come from the same row.  A tap off the bottom row or past
                        // the right edge is a spent slot, not a read.
                        rd_count = 32'd0;
                        for (i = 0; i < IN_PORTS; i = i + 1) begin
                            if (row_ok && ((src_col + i) <= LAST_COL)) begin
                                in_rd_en[i]    <= 1'b1;
                                in_rd_valid[i] <= 1'b1;
                                // Zero-extended explicitly.  `src_row` is nine bits and
                                // `src_col` eight, so a part-select at `IN_ADDR_W` reads
                                // past the end and yields x -- and an x address reads an
                                // x word, which the D3c4b testbench never saw because it
                                // counted reads and never checked where they went.
                                in_rd_addr[i*IN_ADDR_W +: IN_ADDR_W]
                                    <= (row_w * W) + col_w + i;
                                rd_count = rd_count + 32'd1;
                            end
                            else begin
                                in_rd_en[i]    <= 1'b0;
                                in_rd_valid[i] <= 1'b0;
                            end
                        end
                        in_reads <= in_reads + rd_count;
                        if (!slot_live)
                            border_slots <= border_slots + 32'd1;

                        // One weight tile per live MAC cycle, addressed from the phase
                        // table's kernel index -- not from the slot counter.
                        if (slot_live) begin
                            w_rd_en   <= 1'b1;
                            w_rd_addr <= W_BASE[W_ADDR_W-1:0]
                                         + (({4'd0, kh} * 3 + {4'd0, kw}) * IC_GROUPS
                                            + icg) * OC_GROUPS;
                            w_reads   <= w_reads + 32'd1;
                        end

                        // The requant queue gains a whole phase when the phase's last
                        // IC group lands.
                        if ((icg == IC_GROUPS - 1) && (tp == n_tap - 1)) begin
                            rq_pend  <= rq_pend + rq_add - rq_issue;
                            if (rq_peak < (rq_pend + rq_add - rq_issue))
                                rq_peak <= rq_pend + rq_add - rq_issue;
                            if (rq_pend + rq_add - rq_issue > 8'd64) rq_stall <= 1'b1;
                            // and one output word per produced pixel, queued
                            out_pend <= OUT_WORDS[3:0];
                            out_idx  <= 3'd0;
                            out_r    <= r;
                            out_c    <= c;
                            out_row  <= dy;
                            out_odx  <= ph[0];
                            emitted  <= emitted + PIX_PAR;
                        end

                        // ---- advance ------------------------------------------
                        if (icg == IC_GROUPS - 1) begin
                            icg <= 4'd0;
                            if (tp == n_tap - 1) begin
                                tp <= 2'd0;
                                if (ph == 2'd3) begin
                                    ph <= 2'd0;
                                    // next input step: PIX_PAR columns at a time
                                    if (c + PIX_PAR >= W) begin
                                        c <= 8'd0;
                                        if (r == LAST_ROW[7:0]) begin
                                            st <= S_DRAIN;
                                        end
                                        else
                                            r <= r + 8'd1;
                                    end
                                    else
                                        c <= c + PIX_PAR[7:0];
                                end
                                else
                                    ph <= ph + 2'd1;
                            end
                            else
                                tp <= tp + 2'd1;
                        end
                        else
                            icg <= icg + 4'd1;
                    end
                    else begin
                        // ---- the slot was taken and the requant refused it ---
                        // The slot is in the engine's pipeline already --
                        // `cur_issue` was high on the cycle the store took it --
                        // so this only pauses the schedule, and the wires are
                        // cleared so that the presentation cannot be taken twice.
                        pend <= 1'b0;
                    end
                end

                // Flush the last phase's words.  Cycles, not architecture: it is
                // OUT_WORDS cycles once per layer plus, now, however long the
                // store takes for the **last** read, which is a read like any
                // other and is taken before the drain starts.
                S_DRAIN: begin
                    if (pend && !store_hold) pend <= 1'b0;
                    if (!pend && !store_hold && (out_pend == 4'd0)) begin
                        st <= S_DONE; done <= 1'b1; busy <= 1'b0;
                    end
                end

                S_DONE: st <= S_IDLE;
            endcase
        end
    end

endmodule
