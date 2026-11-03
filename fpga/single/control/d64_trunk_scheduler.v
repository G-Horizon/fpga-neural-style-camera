`timescale 1ns/1ps

// D3c3b2 -- the memory-word-accurate schedule for one trunk block on one PE.
//
// D3c3b1 scheduled the *phases*.  It proved the overlap is real -- conv3's skip
// read and its result write do ride along with the PE, and the block lands on the
// overlap bound rather than the serial one -- but it drove addresses that could
// not have worked:
//
//   * a_wr_addr / a_rd_addr were `row * 64 + col` in 10 bits, which is 16 rows
//     wide.  From row 16 the address wraps, and the physical buffers are not 64
//     rows anyway: A is a ring of 11, B a ring of 8.
//   * conv2 held `a_rd_addr` at the output pixel for all 18 cycles instead of
//     walking 9 taps x 2 IC groups, so the 18-cycle cost was right and the
//     18-cycle *access sequence* did not exist.
//   * A and B have wr_bank / rd_bank ports and the scheduler had no such outputs.
//   * every conv1 MAC cycle asserted a_wr_en, but 16 output channels only exist
//     after the accumulation and the requant -- the write-back has to be pipelined
//     behind the pixel that produced it.
//
// This is the module those four things are true of.  Still no arithmetic: the MAC
// array, the requant and the add are a token pipeline, because the question is
// where every word goes on every cycle, not what it holds.
//
// The shape
// ---------
//
//   A  ring of A_ROWS (11) rows x 64 cols x 2 banks x 64 bit   conv1 -> conv2
//   B  ring of B_ROWS  (8) rows x 64 cols x 2 banks x 64 bit   conv2 -> conv3
//
//   conv1  4 PE cycles/pixel, 4 trunk reads   2 A words/pixel, pipelined
//   conv2  18 PE cycles/pixel                18 A reads, 2 B words/pixel
//   conv3  4 PE cycles/pixel, 4 trunk reads   4 trunk writes, 2 B reads
//
// Rings, not frame stores.  Rows are produced in strictly increasing order and
// consumed in strictly increasing order, so slot(row) = row mod ROWS and the slot
// pointers are wrap-around counters that never need a modulo.  Three monotone row
// counters (c1_row, c2_row, c3_row) are never reset at a band boundary, only at a
// block boundary, which is what makes the counters and the row numbers stay in
// step without arithmetic.
//
// The write-back is pipelined behind its pixel
// --------------------------------------------
//
// conv1's 16 output channels are not valid on the cycle the pixel starts; they are
// valid after C1_CYC cycles of accumulation plus the requant.  So the pixel's A
// words are issued during the *next* pixel, out of a pair of pending registers,
// and the phase runs two extra cycles at the end to drain the last one.
//
// conv3 is the one that matters, because there the trunk's read port and its write
// port are both wanted every cycle:
//
//   cycle 0   read X[N] bank 0    write Y[N-1] bank 0
//   cycle 1   read X[N] bank 1    write Y[N-1] bank 1
//   cycle 2   read X[N] bank 2    write Y[N-1] bank 2
//   cycle 3   read X[N] bank 3    write Y[N-1] bank 3
//
// One read and one write every cycle, which is exactly the RAM's capability and
// leaves nothing spare.  The cost is one 4-word holding register and C3_CYC extra
// cycles per band.  This is the claim D3c3b1 asserted without demonstrating.
//
// Reflection
// ----------
//
// D1 froze reflection padding.  conv2's window therefore reflects at the image
// edge rather than clamping or skipping: index -1 reads 1, index COLS reads
// COLS-2.  Which means the source row of a tap is not always row+dy, and the ring
// slot follows the source row, not the offset -- at row 0 the dy=-1 tap reads row
// 1, one slot *forward*.
//
// C3_IC
// -----
//
// conv3 is cin 16 -> cout 32, and 128 MACs per cycle is a 4x32 or an 8x16 split.
// Both cost 4 cycles per pixel.  They differ in the B interface: at 8x16 a 64-bit
// bank word is exactly one IC group and is consumed whole; at 4x32 a word holds
// two IC groups and the datapath needs a sub-word select (b_rd_sub).  The
// parameter exists so the testbench can run both and the choice is measured.
module d64_trunk_scheduler #(
    parameter ROWS          = 64,
    parameter COLS          = 64,
    parameter BAND          = 8,
    parameter A_ROWS        = 11,
    parameter B_ROWS        = 8,
    parameter TRUNK_BANK_W  = 2,
    parameter AB_BANK_W     = 1,
    parameter SWITCH_CYCLES = 64,
    parameter C1_CYC        = 4,     // cin 32 / IC 8
    parameter C2_CYC        = 18,    // 3x3 x (cin 16 / IC 8)
    parameter C3_CYC        = 4,     // cin 16 / IC 4 or IC 8
    parameter C3_IC         = 8      // 8 -> IC8 x OC16, 4 -> IC4 x OC32
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    // D3c5-b: the band boundary's wait.
    //
    // The X ring holds 16 rows of a 64-row tensor, so the prefetcher cannot have
    // the whole block resident and conv1 can, at a band boundary, arrive before
    // the row it wants does.  `hold` freezes the schedule there rather than
    // anywhere else in the phase, and the reason it may is that this is the one
    // place in the block where nothing is live, still owed or in flight: the
    // switch between bands is entered after the phase has drained, so holding it
    // cannot strand a write-back or a requant pass.
    //
    // `tb_d64_trunk_scheduler.v` drives it low and its cycle count is unchanged,
    // which is what keeps this additive rather than a re-schedule.
    input  wire                    hold,
    output reg                     busy,
    output reg                     done,
    output reg  [31:0]             cycles,

    // trunk store: one read port, one write port
    output reg                     trunk_rd_en,
    output reg  [11:0]             trunk_rd_addr,
    output reg  [TRUNK_BANK_W-1:0] trunk_rd_bank,
    output reg                     trunk_wr_en,
    output reg  [11:0]             trunk_wr_addr,
    output reg  [TRUNK_BANK_W-1:0] trunk_wr_bank,

    // A: conv1 writes, conv2 reads
    output reg                     a_wr_en,
    output reg  [9:0]              a_wr_addr,
    output reg  [AB_BANK_W-1:0]    a_wr_bank,
    output reg                     a_rd_en,
    output reg  [9:0]              a_rd_addr,
    output reg  [AB_BANK_W-1:0]    a_rd_bank,

    // B: conv2 writes, conv3 reads
    output reg                     b_wr_en,
    output reg  [9:0]              b_wr_addr,
    output reg  [AB_BANK_W-1:0]    b_wr_bank,
    output reg                     b_rd_en,
    output reg  [9:0]              b_rd_addr,
    output reg  [AB_BANK_W-1:0]    b_rd_bank,
    output reg                     b_rd_sub,   // only meaningful when C3_IC == 4

    output reg  [3:0]              mode,

    // D3c5-b: which band the schedule is on, and whether the switch in progress
    // leads into conv1.  Together they are the whole question the ring's owner
    // has to answer -- "are the next band's rows here yet" -- and neither is
    // derivable from the outside: `mode` alone cannot say which switch this is,
    // and the band counter is this module's own.
    output wire [3:0]              cur_band,
    output wire                    cur_bandsw,

    // ---- the operand interface (D3c5-a) ----------------------------------
    //
    // Additive.  The memory schedule above is unchanged and nothing here is
    // load-bearing for it -- `tb_d64_trunk_scheduler.v` does not know these
    // exist and still passes unchanged, which is the whole reason they are
    // separate outputs rather than a rewrite of the phase counters.
    //
    // These describe the slot whose address is being *issued this cycle*.  The
    // read is registered, so the data for it arrives two cycles later, and the
    // consumer pipelines them accordingly -- see `control/d64_conv_engine.v`,
    // where `a_*` is the stage that carries them alongside the address and `b_*`
    // the stage that meets the data.
    //
    //   cur_icg    which IC group this slot multiplies   conv1 0..3, conv2/3 0..1
    //   cur_tap    which of the nine taps                conv2 only, 0..8
    //   cur_ocg    which output-channel group            conv3 only, 0..1
    //   cur_live   this cycle issues a real MAC slot     (drain cycles do not)
    //   cur_pstart the bias for a new accumulator phase goes on the cycle *before*
    //              the phase's first slot, because the MAC's `pix_start` has
    //              priority over `slot_valid` and loading it on the same edge
    //              would silently drop the first slot's contribution
    //   cur_plast  this is the phase's last slot, so the requant is accepted here
    output wire [3:0]              cur_icg,
    output wire [3:0]              cur_tap,
    output wire                    cur_ocg,
    output wire                    cur_live,
    output wire                    cur_pstart,
    output wire                    cur_plast,
    output reg  [31:0]             trunk_reads,
    output reg  [31:0]             trunk_writes,
    output reg  [31:0]             switches,

    // Observability.  Every port reports the row, column and ring slot it is
    // touching, registered alongside its enable, so a checker can rebuild the
    // rings and the reflection independently instead of consulting the DUT's own
    // bookkeeping -- which is what D3c3b1's testbench did, and is why its liveness
    // check could only say "this row was read at least once".
    output reg  [6:0]  dbg_pix_row,
    output reg  [5:0]  dbg_pix_col,
    output reg  [5:0]  dbg_phase_cyc,
    output reg  [6:0]  dbg_a_wr_row,
    output reg  [5:0]  dbg_a_wr_col,
    output reg  [6:0]  dbg_a_rd_row,
    output reg  [5:0]  dbg_a_rd_col,
    output reg  [6:0]  dbg_b_wr_row,
    output reg  [5:0]  dbg_b_wr_col,
    output reg  [6:0]  dbg_b_rd_row,
    output reg  [5:0]  dbg_b_rd_col,
    output reg  [3:0]  dbg_a_wr_slot,
    output reg  [3:0]  dbg_a_rd_slot,
    output reg  [2:0]  dbg_b_wr_slot,
    output reg  [2:0]  dbg_b_rd_slot
);

    localparam M_IDLE = 4'd0, M_SW = 4'd1, M_C1 = 4'd2, M_C2 = 4'd3,
               M_C3 = 4'd4, M_DONE = 4'd5;

    localparam OFF_NONE = 2'd0, OFF_P1 = 2'd1, OFF_M1 = 2'd2;

    localparam integer BANDS = (ROWS + BAND - 1) / BAND;
    localparam [6:0] ROWS_Q = ROWS;
    localparam [7:0] BAND_Q = BAND;

    localparam integer A_WORDS  = 2;   // 16 conv1 output channels / 8 per word
    localparam integer B_WORDS  = 2;   // 16 conv2 output channels / 8 per word
    localparam integer C3_WORDS = 4;   // 32 conv3 output channels / 8 per word

    reg [3:0]  band_i;
    // Three monotone row counters, one per phase.  They are NOT reset at a band
    // boundary: conv1's rows run 0..ROWS-1 with no gaps across the whole block, and
    // so do conv2's and conv3's.  That is what lets slot(row) be a wrap-around
    // counter with no modulo, and it is also what removes the lookahead
    // recomputation the first version did (it charged band_lo of every band twice).
    reg [6:0]  c1_row, c2_row, c3_row;
    reg [5:0]  col;
    reg [5:0]  phase_cyc;
    reg [3:0]  dy_i, dx_i;          // dy+1, dx+1 for the 3x3 window
    reg [3:0]  next_mode;
    reg [31:0] sw_wait;
    reg        draining;

    // ring slot of the row each phase is on, and of the word being accessed
    reg [3:0]  a_wr_slot, a_row_slot;
    reg [2:0]  b_wr_slot, b_row_slot;

    // one pending output word per phase, pipelined behind the pixel that made it
    reg [6:0]  a_pend_row, b_pend_row, t_pend_row;
    reg [5:0]  a_pend_col, b_pend_col, t_pend_col;
    reg [3:0]  a_pend_slot, b_pend_slot;
    reg        a_pend_v, b_pend_v, t_pend_v;

    wire [6:0] band_lo = band_i * BAND_Q;
    wire [7:0] band_hi_raw = {1'b0, band_lo} + BAND_Q;
    // **The last band may be shorter than BAND, and the clamp below is what
    // makes that safe.**  At ROWS = 36 and BAND = 8 the bands are 8,8,8,8,4:
    // `band_hi_raw` is 40 and the image ends at 36, so this is the one place
    // in the block where a band boundary is not a multiple of BAND.  It is
    // exercised by `tb_d64_trunk_band_rect` and by nothing else -- at
    // ROWS = 64, BAND = 8 the eighth band's `band_hi_raw` is exactly 64 and
    // the clamp never fires.
    //
    // The tooth rounds the last band's length up to BAND (40 instead of 36),
    // which is what a scheduler that assumed every band was full would do,
    // and rounds it *up* rather than down so the failure is rows that do not
    // exist rather than rows left uncomputed.
`ifdef D64_TRUNK_BAND_TOOTH_ROUNDUP
    wire [6:0] band_hi = band_hi_raw[6:0];
`else
    wire [6:0] band_hi = (band_hi_raw > ROWS_Q) ? ROWS_Q : band_hi_raw[6:0];
`endif
    // conv1 must reach one row past its band for conv2's window, except at the
    // bottom of the image where the window reflects instead.
    wire [6:0] c1_end = (band_hi >= ROWS_Q) ? ROWS_Q : (band_hi + 7'd1);
    wire [6:0] c2_end = band_hi;

    assign cur_band   = band_i;
    assign cur_bandsw = (mode == M_SW) && (next_mode == M_C1);

    // --- the 3x3 window, reflected at the image edge ------------------------
    wire [6:0] src_row_m1 = (c2_row == 7'd0) ? 7'd1 : (c2_row - 7'd1);
    wire [6:0] src_row_p1 = (c2_row == ROWS_Q - 7'd1) ? (ROWS_Q - 7'd2) : (c2_row + 7'd1);
    wire [6:0] src_row    = (dy_i == 4'd0) ? src_row_m1 :
                            (dy_i == 4'd1) ? c2_row : src_row_p1;

    wire [5:0] src_col_m1 = (col == 6'd0) ? 6'd1 : (col - 6'd1);
    wire [5:0] src_col_p1 = (col == COLS - 1) ? (COLS - 2) : (col + 6'd1);
    wire [5:0] src_col    = (dx_i == 4'd0) ? src_col_m1 :
                            (dx_i == 4'd1) ? col : src_col_p1;

    // The ring slot follows the source row, not the offset.  At row 0 the dy=-1
    // tap reflects forward to row 1, so the slot step is +1, not -1; at the last
    // row the dy=+1 tap steps -1.  Getting this wrong is a silent one-row aliasing
    // bug, which is why the testbench checks slot == row mod A_ROWS on every
    // access rather than only on the writes.
    wire [1:0] off_row = (dy_i == 4'd0) ? ((c2_row == 7'd0)     ? OFF_P1 : OFF_M1) :
                         (dy_i == 4'd2) ? ((c2_row == ROWS_Q-1) ? OFF_M1 : OFF_P1) :
                         OFF_NONE;
    wire [3:0] a_rd_slot_c =
        (off_row == OFF_P1) ? ((a_row_slot == A_ROWS - 1) ? 4'd0 : (a_row_slot + 4'd1)) :
        (off_row == OFF_M1) ? ((a_row_slot == 4'd0) ? (A_ROWS - 1) : (a_row_slot - 4'd1)) :
        a_row_slot;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0; cycles <= 32'd0; mode <= M_IDLE;
            trunk_rd_en <= 1'b0; trunk_wr_en <= 1'b0;
            trunk_rd_addr <= 12'd0; trunk_wr_addr <= 12'd0;
            trunk_rd_bank <= {TRUNK_BANK_W{1'b0}};
            trunk_wr_bank <= {TRUNK_BANK_W{1'b0}};
            a_wr_en <= 1'b0; a_rd_en <= 1'b0; a_wr_addr <= 10'd0; a_rd_addr <= 10'd0;
            a_wr_bank <= {AB_BANK_W{1'b0}}; a_rd_bank <= {AB_BANK_W{1'b0}};
            b_wr_en <= 1'b0; b_rd_en <= 1'b0; b_wr_addr <= 10'd0; b_rd_addr <= 10'd0;
            b_wr_bank <= {AB_BANK_W{1'b0}}; b_rd_bank <= {AB_BANK_W{1'b0}};
            b_rd_sub <= 1'b0;
            band_i <= 4'd0; c1_row <= 7'd0; c2_row <= 7'd0; c3_row <= 7'd0;
            col <= 6'd0; phase_cyc <= 6'd0; dy_i <= 4'd0; dx_i <= 4'd0;
            next_mode <= M_C1; sw_wait <= 32'd0; draining <= 1'b0;
            a_wr_slot <= 4'd0; a_row_slot <= 4'd0;
            b_wr_slot <= 3'd0; b_row_slot <= 3'd0;
            a_pend_row <= 7'd0; a_pend_col <= 6'd0; a_pend_slot <= 4'd0; a_pend_v <= 1'b0;
            b_pend_row <= 7'd0; b_pend_col <= 6'd0; b_pend_slot <= 3'd0; b_pend_v <= 1'b0;
            t_pend_row <= 7'd0; t_pend_col <= 6'd0; t_pend_v <= 1'b0;
            trunk_reads <= 32'd0; trunk_writes <= 32'd0; switches <= 32'd0;
            dbg_pix_row <= 7'd0; dbg_pix_col <= 6'd0; dbg_phase_cyc <= 6'd0;
            dbg_a_wr_row <= 7'd0; dbg_a_wr_col <= 6'd0; dbg_a_wr_slot <= 4'd0;
            dbg_a_rd_row <= 7'd0; dbg_a_rd_col <= 6'd0; dbg_a_rd_slot <= 4'd0;
            dbg_b_wr_row <= 7'd0; dbg_b_wr_col <= 6'd0; dbg_b_wr_slot <= 3'd0;
            dbg_b_rd_row <= 7'd0; dbg_b_rd_col <= 6'd0; dbg_b_rd_slot <= 3'd0;
        end
        else begin
            // Every port is a one-cycle pulse unless a phase says otherwise.
            trunk_rd_en <= 1'b0; trunk_wr_en <= 1'b0;
            a_wr_en <= 1'b0; a_rd_en <= 1'b0;
            b_wr_en <= 1'b0; b_rd_en <= 1'b0; b_rd_sub <= 1'b0;
            // done is a one-cycle pulse, not a level.  The first version left it
            // high forever and never returned to M_IDLE, so a caller looping
            // "start; wait for done" ran exactly one block and reported five.
            done <= 1'b0;
            // Counted only while the schedule is advancing.  A held switch is
            // the DDR boundary's time, not the block's: the compute core is the
            // same 108,120 cycles whether the data was there or not, and folding
            // the wait into this number would make the one figure D3c3 froze
            // unmeasurable.  The wait has its own counter, in `d64_h0_block`.
            if (busy && !hold) cycles <= cycles + 32'd1;

            case (mode)
                M_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        band_i <= 4'd0;
                        c1_row <= 7'd0; c2_row <= 7'd0; c3_row <= 7'd0;
                        col <= 6'd0; phase_cyc <= 6'd0;
                        dy_i <= 4'd0; dx_i <= 4'd0;
                        a_wr_slot <= 4'd0; a_row_slot <= 4'd0;
                        b_wr_slot <= 3'd0; b_row_slot <= 3'd0;
                        a_pend_v <= 1'b0; b_pend_v <= 1'b0; t_pend_v <= 1'b0;
                        draining <= 1'b0;
                        cycles <= 32'd0;
                        trunk_reads <= 32'd0; trunk_writes <= 32'd0;
                        switches <= 32'd1;
                        next_mode <= M_C1; sw_wait <= SWITCH_CYCLES;
                        mode <= M_SW;
                    end
                end

                // Switch cost between phases and between bands.  The budget is
                // still D3b's placeholder; at 25 switches a block it is 1.5% of
                // the block, so nothing here rests on it.
                M_SW: begin
                    if (hold) begin
                        // D3c5-b.  `switches` is counted on the way INTO M_SW, so
                        // an extended switch is still one switch and the schedule
                        // this module was measured on is unchanged.
                        sw_wait <= sw_wait;
                    end
                    else if (sw_wait <= 32'd1) begin
                        // Counted on the way INTO M_SW, not out of it: both would
                        // double it.  The testbench caught 48 where 24 belong.
                        mode <= next_mode;
                        col <= 6'd0; phase_cyc <= 6'd0;
                        dy_i <= 4'd0; dx_i <= 4'd0;
                        draining <= 1'b0;
                        sw_wait <= SWITCH_CYCLES;
                    end
                    else
                        sw_wait <= sw_wait - 32'd1;
                end

                // conv1: 1x1, 32 -> 16.  One trunk bank word per PE cycle, one A
                // word per bank for the pixel that finished last.
                M_C1: begin
                    dbg_pix_row <= c1_row;
                    dbg_pix_col <= col;
                    dbg_phase_cyc <= phase_cyc;

                    if (a_pend_v && (phase_cyc < A_WORDS)) begin
                        a_wr_en   <= 1'b1;
                        a_wr_addr <= (a_pend_slot * COLS) + a_pend_col;
                        a_wr_bank <= phase_cyc[AB_BANK_W-1:0];
                        dbg_a_wr_row <= a_pend_row;
                        dbg_a_wr_col <= a_pend_col;
                        dbg_a_wr_slot <= a_pend_slot;
                        if (phase_cyc == A_WORDS - 1) a_pend_v <= 1'b0;
                    end

                    if (draining) begin
                        // Exit one cycle *after* the last word: the enable is
                        // registered, so leaving on the same edge would hold it
                        // into the switch phase.
                        if (phase_cyc == A_WORDS) begin
                            draining <= 1'b0;
                            next_mode <= M_C2;
                            mode <= M_SW;
                            switches <= switches + 32'd1;
                            col <= 6'd0; phase_cyc <= 6'd0;
                        end
                        else
                            phase_cyc <= phase_cyc + 6'd1;
                    end
                    else begin
                        trunk_rd_en   <= 1'b1;
                        trunk_rd_addr <= (c1_row * COLS) + col;
                        trunk_rd_bank <= phase_cyc[TRUNK_BANK_W-1:0];
                        trunk_reads   <= trunk_reads + 32'd1;

                        if (phase_cyc == C1_CYC - 1) begin
                            phase_cyc <= 6'd0;
                            a_pend_v    <= 1'b1;
                            a_pend_row  <= c1_row;
                            a_pend_col  <= col;
                            a_pend_slot <= a_wr_slot;
                            if (col == COLS - 1) begin
                                col <= 6'd0;
                                c1_row <= c1_row + 7'd1;
                                a_wr_slot <= (a_wr_slot == A_ROWS - 1) ? 4'd0
                                                                       : (a_wr_slot + 4'd1);
                                if (c1_row + 7'd1 >= c1_end) draining <= 1'b1;
                            end
                            else
                                col <= col + 6'd1;
                        end
                        else
                            phase_cyc <= phase_cyc + 6'd1;
                    end
                end

                // conv2: 3x3, 16 -> 16.  Nine taps, two IC groups each, one A bank
                // word per PE cycle -- which puts the A read port at 100% for the
                // whole phase.  The B write is the pixel that finished last.
                M_C2: begin
                    dbg_pix_row <= c2_row;
                    dbg_pix_col <= col;
                    dbg_phase_cyc <= phase_cyc;

                    if (b_pend_v && (phase_cyc < B_WORDS)) begin
                        b_wr_en   <= 1'b1;
                        b_wr_addr <= (b_pend_slot * COLS) + b_pend_col;
                        b_wr_bank <= phase_cyc[AB_BANK_W-1:0];
                        dbg_b_wr_row <= b_pend_row;
                        dbg_b_wr_col <= b_pend_col;
                        dbg_b_wr_slot <= b_pend_slot;
                        if (phase_cyc == B_WORDS - 1) b_pend_v <= 1'b0;
                    end

                    if (draining) begin
                        // Exit one cycle *after* the last word: the enable is
                        // registered, so leaving on the same edge would hold it
                        // into the switch phase.
                        if (phase_cyc == B_WORDS) begin
                            draining <= 1'b0;
                            next_mode <= M_C3;
                            mode <= M_SW;
                            switches <= switches + 32'd1;
                            col <= 6'd0; phase_cyc <= 6'd0;
                        end
                        else
                            phase_cyc <= phase_cyc + 6'd1;
                    end
                    else begin
                        a_rd_en       <= 1'b1;
                        a_rd_addr     <= (a_rd_slot_c * COLS) + src_col;
                        a_rd_bank     <= phase_cyc[AB_BANK_W-1:0];
                        dbg_a_rd_row  <= src_row;
                        dbg_a_rd_col  <= src_col;
                        dbg_a_rd_slot <= a_rd_slot_c;

                        // end of a tap: step dx, and dy when dx wraps
                        if (phase_cyc[0] && (phase_cyc != C2_CYC - 1)) begin
                            if (dx_i == 4'd2) begin
                                dx_i <= 4'd0;
                                dy_i <= (dy_i == 4'd2) ? 4'd0 : (dy_i + 4'd1);
                            end
                            else
                                dx_i <= dx_i + 4'd1;
                        end

                        if (phase_cyc == C2_CYC - 1) begin
                            phase_cyc <= 6'd0;
                            dy_i <= 4'd0; dx_i <= 4'd0;
                            b_pend_v    <= 1'b1;
                            b_pend_row  <= c2_row;
                            b_pend_col  <= col;
                            b_pend_slot <= b_wr_slot;
                            if (col == COLS - 1) begin
                                col <= 6'd0;
                                c2_row <= c2_row + 7'd1;
                                a_row_slot <= (a_row_slot == A_ROWS - 1) ? 4'd0
                                                                         : (a_row_slot + 4'd1);
                                b_wr_slot  <= (b_wr_slot == B_ROWS - 1) ? 3'd0
                                                                        : (b_wr_slot + 3'd1);
                                if (c2_row + 7'd1 >= c2_end) draining <= 1'b1;
                            end
                            else
                                col <= col + 6'd1;
                        end
                        else
                            phase_cyc <= phase_cyc + 6'd1;
                    end
                end

                // conv3: 1x1, 16 -> 32, plus the skip read and the write-back.
                // Every working cycle: one B read, one trunk read, one trunk write.
                M_C3: begin
                    dbg_pix_row <= c3_row;
                    dbg_pix_col <= col;
                    dbg_phase_cyc <= phase_cyc;

                    if (t_pend_v) begin
                        trunk_wr_en   <= 1'b1;
                        trunk_wr_addr <= (t_pend_row * COLS) + t_pend_col;
                        trunk_wr_bank <= phase_cyc[TRUNK_BANK_W-1:0];
                        trunk_writes  <= trunk_writes + 32'd1;
                        if (phase_cyc == C3_CYC - 1) t_pend_v <= 1'b0;
                    end

                    if (draining) begin
                        if (phase_cyc == C3_WORDS) begin
                            draining <= 1'b0;
                            t_pend_v <= 1'b0;
                            if (band_i + 4'd1 >= BANDS) begin
                                mode <= M_DONE; done <= 1'b1; busy <= 1'b0;
                            end
                            else begin
                                band_i <= band_i + 4'd1;
                                col <= 6'd0; phase_cyc <= 6'd0;
                                next_mode <= M_C1;
                                mode <= M_SW;
                                switches <= switches + 32'd1;
                            end
                        end
                        else
                            phase_cyc <= phase_cyc + 6'd1;
                    end
                    else begin
                        trunk_rd_en   <= 1'b1;
                        trunk_rd_addr <= (c3_row * COLS) + col;
                        trunk_rd_bank <= phase_cyc[TRUNK_BANK_W-1:0];
                        trunk_reads   <= trunk_reads + 32'd1;

                        b_rd_en       <= 1'b1;
                        b_rd_addr     <= (b_row_slot * COLS) + col;
                        dbg_b_rd_row  <= c3_row;
                        dbg_b_rd_col  <= col;
                        dbg_b_rd_slot <= b_row_slot;
                        if (C3_IC == 8)
                            b_rd_bank <= phase_cyc[0];
                        else begin
                            b_rd_bank <= phase_cyc[1];
                            b_rd_sub  <= phase_cyc[0];
                        end

                        if (phase_cyc == C3_CYC - 1) begin
                            phase_cyc <= 6'd0;
                            t_pend_v   <= 1'b1;
                            t_pend_row <= c3_row;
                            t_pend_col <= col;
                            if (col == COLS - 1) begin
                                col <= 6'd0;
                                c3_row <= c3_row + 7'd1;
                                b_row_slot <= (b_row_slot == B_ROWS - 1) ? 3'd0
                                                                         : (b_row_slot + 3'd1);
                                if (c3_row + 7'd1 >= band_hi) draining <= 1'b1;
                            end
                            else
                                col <= col + 6'd1;
                        end
                        else
                            phase_cyc <= phase_cyc + 6'd1;
                    end
                end

                M_DONE: mode <= M_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // the operand interface
    // ------------------------------------------------------------------
    //
    // Combinational from `mode`, `phase_cyc` and `draining`, all of which are
    // registers, so these are valid for the same cycle the address assignment in
    // the case statement above is evaluated -- which is the cycle they describe.

    wire op_c1 = (mode == M_C1) && !draining;
    wire op_c2 = (mode == M_C2) && !draining;
    wire op_c3 = (mode == M_C3) && !draining;

    assign cur_live = op_c1 | op_c2 | op_c3;

    // conv1's four cycles *are* the four IC groups.  conv2 and conv3 both walk
    // two IC groups, and their second axis differs: conv2's is the tap, conv3's
    // is the output-channel group.
    assign cur_icg = op_c1 ? phase_cyc[3:0]
                  : (op_c2 | op_c3) ? {3'd0, phase_cyc[0]}
                  : 4'd0;

    assign cur_tap = op_c2 ? {2'd0, phase_cyc[5:1]} : 4'd0;

    // conv3's phase is {icg, ocg} = {phase_cyc[0], phase_cyc[1]}, so the two OC
    // groups are cycles {0,1} and {2,3} and each has its own requant.
    assign cur_ocg = op_c3 & phase_cyc[1];

    assign cur_pstart = op_c1 ? (phase_cyc == 6'd0)
                      : op_c2 ? (phase_cyc == 6'd0)
                      : op_c3 ? ((phase_cyc == 6'd0) || (phase_cyc == 6'd2))
                      : 1'b0;

    // The last slot of the *accumulator phase*, which is one per pixel for conv1
    // and conv2 and two per pixel for conv3.
    assign cur_plast = op_c1 ? (phase_cyc == C1_CYC[5:0] - 6'd1)
                     : op_c2 ? (phase_cyc == C2_CYC[5:0] - 6'd1)
                     : op_c3 ? phase_cyc[0]
                     : 1'b0;

endmodule