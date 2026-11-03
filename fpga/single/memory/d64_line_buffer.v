// One line buffer for the layers that run outside the trunk.
//
// D3c1 charges each active layer `(k - 1) rows x w_in columns x cin channels`,
// and with the trunk resident the worst of them is `downs.0`: 3x3 stride 2 on a
// 256-wide, 8-channel input, so two rows of 256 pixels of 8 channels of 8 bits.
// That is 32,768 bits, which divides to 4 RAM10, and this module exists so the
// division is replaced by a number.
//
// It is the same structure as the A and B rings -- a ring of rows addressed by
// pixel, one word per pixel, read with a row offset -- with two differences.
// There is one bank rather than two, because 8 channels is one 64-bit word and
// there is no IC group to select; and there is no band schedule, so the depth is
// the whole buffer rather than a ring the scheduler walks.
//
// The unconditional read with the enable applied to the output, and the
// `ram_style` attribute, are the inference pattern the trunk store needed: an
// enabled read makes the mapper build logic memory instead of a block.
module d64_line_buffer #(
    parameter ROWS   = 2,
    parameter COLS   = 256,
    parameter CH     = 8,
    parameter DATA_W = 8,
    parameter DEPTH  = ROWS * COLS,
    parameter WORD_W = CH * DATA_W,
    parameter ADDR_W = 9
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,
    input  wire [WORD_W-1:0] wr_data,

    input  wire              rd_en,
    input  wire [ADDR_W-1:0] rd_addr,
    output wire [WORD_W-1:0] rd_data
);

    (* ram_style = "block" *) reg [WORD_W-1:0] mem [0:DEPTH-1];
    reg [WORD_W-1:0] q;

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        q <= mem[rd_addr];
    end

    assign rd_data = rd_en ? q : {WORD_W{1'b0}};

endmodule
