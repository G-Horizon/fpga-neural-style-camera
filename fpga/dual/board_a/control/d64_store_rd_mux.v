`timescale 1ns/1ps

// ============================================================================
// d64_store_rd_mux -- one store read port, N engine read ports
// ============================================================================
//
// What it is
// ----------
//
// A store with ONE read port presented to a consumer that asks for N of them.
// It spends N cycles on the N words where the consumer asked for one, holding
// the consumer with `e_stall` while it does.
//
// Why it exists, in one number
// ----------------------------
//
// `control/d64_burst_prefetch.v` is 8 RAM10 at `IN_PORTS = 1` and **48** at
// `IN_PORTS = 6` (docs/RAM10_PORTS_2026-10-07.md section 1), against a decoder
// half with 30 spare.  Above one port the array stops inferring a block RAM at
// all (`EFX-0657`) and becomes a logic array -- 84,029 LUT4 at two ports and
// 270,794 at six, on a device with 60,800.  So **one port is the only cache
// that fits**, and the port count the consumer needs is a parameter of this
// module instead.
//
// The consumer this was measured against
// --------------------------------------
//
// `control/d64_tail_engine.v` presents six ports (`IN_PORTS = 6`, one column a
// port, `4g-1 .. 4g+4`) and `control/d64_tconv_engine.v` presents two for
// ups.1 (`IN_PORTS = 2`, two adjacent columns).  Both ask for all N words of a
// read cycle in the NEXT cycle.  This module is N-agnostic; the number is a
// parameter and the arithmetic is the same at 2 as at 6.
//
// The contract it keeps is the engine's, unchanged
// ------------------------------------------------
//
//   * a read is TRANSFERRED in the cycle `e_stall` is low;
//   * the words for the addresses presented in that cycle are on `e_data` in
//     the following cycle, and are HELD there until the next transfer.
//
// That is `d64_burst_prefetch`'s own contract (`acc_d` selects, `rd_word_q`
// holds -- `d64_burst_prefetch.v:820-830`), and it is the contract
// `d64_tconv_engine` states at `:179-181` and `d64_tconv_scheduler` at
// `:232-243`.  The hold is not a convenience: the engines re-write their row
// register from the data port on cycles they are stalled, so a word that moved
// under a stall would write a row that is not the row the phase wants.
//
// Why `s_en` may not read `s_stall`
// ---------------------------------
//
// `d64_burst_prefetch`'s `in_stall` is a function of `in_rd_en`: it rises
// because a port presented a read the cache cannot serve.  An enable gated on
// the stall would therefore be a combinational loop.  So an address is
// presented whenever one is wanted, and `s_stall` only decides whether the
// transfer happened.
//
// Provenance
// ----------
//
// This module is `sim/_agent_ports/scratch/d64_tail_rd_mux.v` -- the serialiser
// that the measured one-port TAIL builds ran (`tools/run_tail_mux_cost.sh`;
// 196,616 / 491,528 / 557,832 cycles, 0 of 65,536 words wrong) -- moved out of
// scratch and given the name it will carry as RTL.  Two things differ and both
// are parameterisation, not logic: the index width is `$clog2(IN_PORTS+1)`
// instead of a literal 3 (identical at 6, one bit narrower at 2), and the
// `SAB_SWAP` generate has a two-port arm so the tooth exists at the decoder's
// port count as well as the tail's.  At `IN_PORTS = 6` and `SAB_SWAP = 0` every
// net is the scratch module's.
module d64_store_rd_mux #(
    parameter integer IN_PORTS = 6,
    parameter integer AW       = 16,
    parameter integer DW       = 64,
    // ---- the store's word and the engine's word ----------------------------
    // `DW` is the ENGINE's word: one port's word, and it is the width of
    // `e_data` divided by `IN_PORTS`.
    //
    // `SW` is the STORE's word -- one read answers one of these -- and `IC_PAR`
    // is how many channels ride in it.  **`SW = 0` means "the store's word is
    // the engine's word" and folds the narrowing below away entirely**, which is
    // the only build the TAIL has and the one this module was measured at.
    //
    // The narrowing exists because `control/d64_burst_prefetch.v` is built at
    // `IN_W = 10` when one instance serves the whole decoder
    // (`docs/RAM10_PORTS_2026-10-07.md` section 1: `IN_W = 10` is the shared
    // instance's width, the widest activation the decoder uses), so its port
    // word is `IC_PAR * IN_W` = 80 bits, while `d64_tconv_engine` at ups.1 is
    // built `IN_W = 8` and slices **8 bits a channel out of an 8-bit-packed
    // word** (`d64_tconv_engine.v:443`).  Handing it the 80-bit word would read
    // channel 1 from bits 8..15 of lane 0: a wrong activation with the right
    // address, which no address check can see.  `SW = 80, DW = 64, IC_PAR = 8`
    // takes bits [7:0] of each 10-bit lane, which is what the store's container
    // holds -- the value is U8 in a U16 container and zero-extended above it.
    parameter integer SW       = 0,     // 0 = the store word is `DW`
    parameter integer IC_PAR   = 0,     // 0 = one channel, i.e. no narrowing
    // **`e_stall` in a cycle the engine is asking for nothing.**
    //
    // 0 is `~finish`: the stall is high whenever a group is not completing,
    // including the cycles in which the engine presents no read at all.  That
    // is what `sim/_agent_ports/scratch/d64_tail_rd_mux.v` did, and it is what
    // the TAIL's measured builds ran with (196,616 / 491,528 / 557,832 cycles,
    // `tools/run_tail_mux_cost.sh`).  The tail can carry it because it reads
    // its `in_stall` through `wire s_stall = s_rd && in_stall`
    // (`control/d64_tail_engine.v:513`) -- a stall in a cycle with no read
    // presented has no effect on it at all.
    //
    // 1 is the rule `d64_burst_prefetch` keeps about its own port: `in_stall`
    // is a function of `in_rd_en`, so a store that is asked for nothing is not
    // stalled.  **`d64_tconv_engine` requires this reading**, because it does
    // not AND its `in_stall` with a read-valid: it registers the level
    // (`d64_tconv_engine.v:324-334`) and the scheduler will not present a slot
    // while that level is high (`d64_tconv_scheduler.v:254-270`, `:412-424`).
    // At 0 the decoder layer **deadlocks**: measured on ups.1 through this
    // module, 300,329 cycles with 0 groups served, the engine never presenting
    // its first slot.  That is the whole of the difference, and it is one line.
    //
    // The default is 0, so that at `IN_PORTS = 6`, `SAB_SWAP = 0` this file
    // reproduces the measured TAIL serialiser net for net.  A decoder build
    // sets it to 1.
    parameter integer NO_IDLE_STALL = 0,

    // Verification only, folded away at 0.  The last port is answered with
    // another port's word -- the one way a serialiser goes wrong, and a failure
    // on data rather than on a count.  `tools/run_tail_mux_cost.sh`'s TB_SAB.
    parameter integer SAB_SWAP = 0
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- the engine's side -------------------------------------------------
    input  wire [IN_PORTS-1:0]      e_en,
    input  wire [IN_PORTS*AW-1:0]   e_addr,
    output wire                     e_stall,
    output reg  [IN_PORTS*DW-1:0]   e_data,

    // ---- the store's side: ONE port ----------------------------------------
    output wire                     s_en,
    output wire [AW-1:0]            s_addr,
    // **Written out and not as `SWW`**: a port list cannot see a localparam
    // declared in the module body -- the trap `docs/F2_INTERFACES_2026-10-06.md`
    // section 0 names -- and this is the second place in this tree it bites.
    input  wire [((SW == 0) ? DW : SW)-1:0]  s_data,
    input  wire                     s_stall,

    // ---- evidence ----------------------------------------------------------
    output reg  [31:0]              transfers,     // reads transferred on the port
    output reg  [31:0]              stall_cycles,  // cycles the port refused
    output reg  [31:0]              groups,        // engine read cycles served
    output reg  [31:0]              mux_cycles     // cycles spent holding the engine
);
    localparam integer IW = $clog2(IN_PORTS+1);    // 0 .. IN_PORTS, inclusive
    localparam integer SWW = (SW == 0) ? DW : SW; // the store's word
    localparam integer NCH = (IC_PAR == 0) ? 1 : IC_PAR;
    localparam integer SRC_W = SWW / NCH;         // bits a channel holds below
    localparam integer DST_W = DW  / NCH;         // ... and above

    // One store word -> one engine word.  Bits [DST_W-1:0] of each of the
    // store's channels, in the same order, and nothing else: no shift, no
    // swap and no sign extension -- the store's container is zero-extended
    // above the value (`board/ddr3/d64_ups_u16_writer.v`'s own rule), so the
    // low bits ARE the value.
    function [DW-1:0] narrow;
        input [SWW-1:0] w;
        integer c;
        begin
            narrow = {DW{1'b0}};
            for (c = 0; c < NCH; c = c + 1)
                narrow[c*DST_W +: DST_W] = w[c*SRC_W +: DST_W];
        end
    endfunction

    reg [IW-1:0] idx;      // the next read to issue while a group is in flight
    reg [IW-1:0] got;      // words captured so far
    reg          issued;   // a transfer was made in the previous cycle
    reg          active;   // a group is in flight
    reg [DW-1:0] acc [0:IN_PORTS-1];

    wire busy = active && (idx < IN_PORTS);

    // **The idle state presents the first address in the same cycle the engine
    // asks for it**, and the collecting state walks the rest.  One cycle here
    // would be three cycles a phase.
    assign s_en   = active ? busy : |e_en;
    assign s_addr = active ? e_addr[idx*AW +: AW] : e_addr[0 +: AW];

    // The group is finished on the cycle the LAST word lands on `s_data`:
    // `got` counts the words already in `acc`, so the word arriving now is
    // word number `got`.  Finishing *here* rather than one cycle later is what
    // puts the engine's transfer in the same cycle as the last read's answer.
    wire finish = active && issued && (got == IN_PORTS-1);

    // **`e_stall` is combinational**, because it has to be low in the cycle the
    // group completes and not one cycle after it.  `NO_IDLE_STALL` is the one
    // reading of it that differs between the two engines this serves -- see the
    // parameter's own comment.
    wire idle = ~(|e_en | active);      // the engine is asking for nothing
    assign e_stall = (NO_IDLE_STALL && idle) ? 1'b0 : ~finish;

    // The N words of a group: every port below `got` is already in `acc`, and
    // port `got` is the one on `s_data` now.
    wire [IN_PORTS*DW-1:0] asm;
    genvar gk;
    generate
        for (gk = 0; gk < IN_PORTS; gk = gk + 1) begin : g_asm
            assign asm[gk*DW +: DW] = (gk == got) ? narrow(s_data) : acc[gk];
        end
    endgenerate

    // The tooth's reading: the group's words are handed to the wrong ports.
    // The six-port form is `sim/_agent_ports/scratch/d64_tail_rd_mux.v`'s,
    // verbatim -- it is the arm that measured 64,769 of 65,536 words wrong at
    // `tools/run_tail_mux_cost.sh` -- and the two-port form is the same idea at
    // the decoder's port count: each port answered with the other port's word.
    // **A `generate if` and not a conditional expression**: every branch of a
    // `?:` chain is elaborated, so the six-port arm's `asm[5*DW +: DW]` is
    // analysed in a two-port build as well and `vopt-2697` reports the
    // part-select as out of bounds -- three times, for a branch that is not
    // selected and cannot be.
    wire [IN_PORTS*DW-1:0] out_w;
    generate
        if (!SAB_SWAP)          assign out_w = asm;
        else if (IN_PORTS == 6) assign out_w =
            {asm[0*DW +: DW], asm[0*DW +: DW], asm[1*DW +: DW],
             asm[2*DW +: DW], asm[3*DW +: DW], asm[4*DW +: DW]};
        else if (IN_PORTS == 2) assign out_w =
            {asm[0*DW +: DW], asm[1*DW +: DW]};
        else                    assign out_w = asm;
    endgenerate

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx <= {IW{1'b0}}; got <= {IW{1'b0}};
            issued <= 1'b0; active <= 1'b0;
            e_data <= {IN_PORTS*DW{1'b0}};
            for (k = 0; k < IN_PORTS; k = k + 1) acc[k] <= {DW{1'b0}};
            transfers <= 32'd0; stall_cycles <= 32'd0; groups <= 32'd0;
            mux_cycles <= 32'd0;
        end else begin
            // `issued` is taken from the ADDRESS PRESENTED and not from
            // `busy`: the idle state issues its first read in the cycle the
            // engine asks, and a transfer made there has to be counted.
            issued <= s_en && !s_stall;
            if (busy && s_stall)  stall_cycles <= stall_cycles + 32'd1;
            if (busy && !s_stall) begin
                transfers <= transfers + 32'd1;
                idx       <= idx + 1'b1;
            end
            if (e_stall) mux_cycles <= mux_cycles + 32'd1;

            if (active) begin
                if (issued) begin
                    acc[got] <= narrow(s_data);
                    got      <= got + 1'b1;
                end
                if (finish) begin
                    e_data <= out_w;
                    active <= 1'b0;
                    idx    <= {IW{1'b0}};
                    got    <= {IW{1'b0}};
                    groups <= groups + 32'd1;
                end
            end else if (|e_en) begin
                // **The idle state presents address 0 in THIS cycle, and
                // whether it was taken decides where the next one comes from.**
                // Advancing the index unconditionally here is the bug this
                // comment is for: a store that refused the first read would
                // have that read counted and never made, and the group would
                // then finish one word short and deadlock -- which is exactly
                // what a one-port cache, whose first read is always a miss,
                // does.
                active <= 1'b1;
                idx    <= s_stall ? {IW{1'b0}} : {{(IW-1){1'b0}}, 1'b1};
                got    <= {IW{1'b0}};
            end
        end
    end
endmodule
