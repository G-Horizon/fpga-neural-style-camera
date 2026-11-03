`timescale 1ns/1ps

// The X16 ring: the block input tensor as 16 rows instead of 64.
//
// D3c5-a closed with the whole tensor in the engine -- `d64_tc_ram #(.DW(80),
// .AW(14))`, 16,384 words, 160 KiB, which is not a memory this design has.  H0
// froze a different one: **16 rows of the 64**, four banks of 80 bits,
// 32 RAM10, which is what `synth/d64_h0_memory_root.v` measures and what
// `tools/d64_h0_cycle_model.py` sizes against the DDR's real service.
//
// Why 16 and why a ring at all
// ---------------------------
//
// A band is 8 rows.  conv1 walks the band plus one lookahead row, and the
// residual add in conv3 walks the band again -- the same rows, a whole conv2
// later.  So the live set at any moment is the band (8) plus the lookahead (1)
// plus whatever the prefetcher has pulled in for the next band (up to 7 before
// it needs a slot that is still live).  The cycle model sweeps 9..20 and 16 is
// where the starvation stops falling; `board/ddr3/ddr3_h0_replay.v` sweeps the
// same numbers on the board.  It is not a round number that happened to fit --
// it is the shallowest depth that stops conv1 waiting, and it is also exactly
// two RAM10 tiles a bank.
//
// The slot is `row mod 16`, and with the row counter monotone that is a slice,
// not a modulo.  Rows are produced in increasing order and released in
// increasing order, so the ring never needs to search for a slot.
//
// The live set, which is the whole safety argument
// -----------------------------------------------
//
// A slot is **live** from the last beat of its fill until the residual add has
// taken the last word out of it.  Two things can then go wrong and both are
// counted rather than hoped about:
//
//   ovf   a fill landed on a slot that was still live -- data destroyed
//   rbf   the schedule read a slot that did not hold the row it asked for --
//         data never written, and a picture made of whatever was there
//
// Both are structurally prevented by the caller: no fill is started for a row
// whose slot is live (`q_free`), and no switch into conv1 completes until the
// whole band answers `q_ready`.  The counters exist because "structurally
// prevented" is a claim, and the last three times this project made one it was
// wrong about a cycle.
//
// `slot_row` is compared, not just the valid bit.  A ring that only tracked
// validity would pass a stale row off as the one being asked for exactly once
// per wrap, which is a failure that looks like a picture.
module d64_x16_ring #(
    parameter ROWS     = 16,      // power of two; the ring's depth in rows
    parameter COLS     = 64,
    parameter BANKS    = 4,       // 32 channels / 8 per bank word
    parameter WORD_W   = 80,      // 8 channels x 10 bits, the trunk's contract
    parameter BANK_W   = 2,
    parameter ROW_W    = 6        // the image's row count: 64 rows -> 6 bits
) (
    input  wire              clk,
    input  wire              rst_n,

    // ---- the engine's read port -------------------------------------------
    // Address presented combinationally, data registered -- the same contract
    // as `d64_tc_ram`, so the engine's pipeline is untouched by the swap.
    input  wire [ROW_W+5:0]  rd_addr,      // row*COLS + col
    input  wire [BANK_W-1:0] rd_bank,
    input  wire              rd_used,      // the schedule is issuing a read
    output wire [WORD_W-1:0] rd_data,

    // ---- the fill port ----------------------------------------------------
    // One row at a time, one bank word a beat, in col-major order with the bank
    // fastest -- the DDR's own beat order, so nothing re-sequences.
    input  wire [ROW_W-1:0]  wr_row,
    input  wire [5:0]        wr_col,
    input  wire [BANK_W-1:0] wr_bank,
    input  wire [WORD_W-1:0] wr_word,
    input  wire              wr_en,
    input  wire              wr_last,      // this beat completes `wr_row`

    // ---- release ----------------------------------------------------------
    // Raised when the residual add has written the last word of a row, which is
    // the last moment that row is read.
    input  wire [ROW_W-1:0]  rel_row,
    input  wire              rel_en,

    // ---- residency --------------------------------------------------------
    // Two query ports, because the two questions are asked by different things
    // on the same cycle and neither can wait for the other: the band scan asks
    // "is the row I need about to be here", the prefetcher asks "may I start
    // filling the row I am on".  A shared port would make the fill wait for the
    // scan, which is the one dependency this ring exists not to have.
    input  wire [ROW_W-1:0]  q_row,
    output wire              q_ready,     // this slot holds exactly q_row, filled
    input  wire [ROW_W-1:0]  f_row,
    output wire              f_free,      // this slot holds nothing anyone needs

    output reg  [31:0]       ovf,
    output reg  [31:0]       rbf,
    // D3c5-c: whether any slot still holds a row nobody has released.  A block
    // boundary has to start from none.
    output wire              live
);

    localparam SLOT_W = $clog2(ROWS);

    // The row counter is monotone, so `row mod ROWS` is the low bits.  Stated as
    // a slice rather than a `%` because the whole point of the ring is that no
    // arithmetic sits on the address path.
    // The row is `rd_addr[11:6]` -- 64 rows of 64 columns -- so `row mod 16` is
    // `rd_addr[9:6]`, and a slice that ended at bit 5 would fold the column's top
    // bit into the slot number and alias rows 16 apart onto different slots.  It
    // is written `[SLOT_W+5:6]` and not `[SLOT_W-1:0] + 5` for exactly that
    // reason: the off-by-one here does not fail, it reads the wrong row.
    wire [SLOT_W-1:0] rd_slot = rd_addr[SLOT_W+5:6];
    wire [SLOT_W-1:0] wr_slot = wr_row[SLOT_W-1:0];
    wire [SLOT_W-1:0] re_slot = rel_row[SLOT_W-1:0];
    wire [SLOT_W-1:0] q_slot  = q_row[SLOT_W-1:0];

    reg [ROW_W-1:0] slot_row [0:ROWS-1];
    reg             slot_val [0:ROWS-1];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < ROWS; i = i + 1) begin
                slot_row[i] <= {ROW_W{1'b0}};
                slot_val[i] <= 1'b0;
            end
        end
        else begin
            // Release first and fill second, in that order, so a beat that
            // completes a row and a release of a *different* slot on the same
            // edge cannot be mistaken for each other.  They never address the
            // same slot -- a release is what makes the next fill legal -- so the
            // order here is documentation rather than arbitration.
            if (rel_en) slot_val[re_slot] <= 1'b0;
            if (wr_en && wr_last) begin
                slot_val[wr_slot] <= 1'b1;
                slot_row[wr_slot] <= wr_row;
            end
        end
    end

    // An unpacked array cannot be reduced in one expression, and it has to be a
    // fold rather than a flag maintained alongside `slot_val`, because a flag is
    // a second copy of the truth and this is the number a block boundary is
    // judged on.
    reg     live_r;
    integer li;
    always @* begin
        live_r = 1'b0;
        for (li = 0; li < ROWS; li = li + 1) live_r = live_r | slot_val[li];
    end

    assign live    = live_r;

    assign q_ready = slot_val[q_slot] && (slot_row[q_slot] == q_row);
    assign f_free  = !slot_val[f_row[SLOT_W-1:0]];

    // ---- the storage ------------------------------------------------------
    // The bank word is separate from the address here, unlike the flat store it
    // replaces, because this is exactly the shape the H0 root was measured in:
    // four `d64_trunk_row_buf` banks of 1024 x 80, 32 RAM10 in total.
    wire [SLOT_W+5:0] mem_rd = {rd_slot, rd_addr[5:0]};
    wire [SLOT_W+5:0] mem_wr = {wr_slot, wr_col};

    d64_trunk_row_buf #(.DEPTH(ROWS * COLS), .BANKS(BANKS), .WORD_W(WORD_W),
                        .ADDR_W(SLOT_W + 6), .BANK_W(BANK_W)) u_mem (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en), .wr_addr(mem_wr), .wr_bank(wr_bank), .wr_data(wr_word),
        // Read always enabled.  The engine's skip capture takes `rd_data` on the
        // cycle *after* the read was issued, and the schedule deasserts its read
        // enable on that cycle -- so an output-enable tied to the enable would
        // zero exactly the word the residual add is waiting for.  The flat store
        // it replaces had the same property, which is why the engine reads it
        // with `re` tied high.
        .rd_en(1'b1), .rd_addr(mem_rd), .rd_bank(rd_bank), .rd_data(rd_data)
    );

    // ---- the two violations ----------------------------------------------
    //
    // `ovf` is counted once per row, on its first beat, rather than on all 256:
    // a clobbered row is one mistake and reporting it 256 times hides how many
    // rows were lost.  `rbf` is counted per read, because there a cycle that
    // should not have happened is the whole finding.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ovf <= 32'd0;
            rbf <= 32'd0;
        end
        else begin
            if (wr_en && (wr_col == 6'd0) && (wr_bank == {BANK_W{1'b0}})
                && slot_val[wr_slot] && (slot_row[wr_slot] != wr_row))
                ovf <= ovf + 32'd1;
            if (rd_used && !(slot_val[rd_slot] && (slot_row[rd_slot] ==
                                                   rd_addr[ROW_W+5:6])))
                rbf <= rbf + 32'd1;
        end
    end

endmodule
