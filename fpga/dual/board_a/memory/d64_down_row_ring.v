`timescale 1ns/1ps

// Five physical rows hold the active stride-2 triplet plus two source rows
// of lookahead.  Each row is 256 8-channel words for both downs.0 and .1.
module d64_down_row_ring (
    input  wire        clk,
    input  wire        wr_en,
    input  wire [2:0]  wr_slot,
    input  wire [7:0]  wr_word,
    input  wire [63:0] wr_data,
    input  wire        rd_en,
    input  wire [10:0] rd_addr,
    output reg  [63:0] rd_data
);
    reg [63:0] mem [0:1279];
    wire [10:0] wr_addr = {wr_slot, wr_word};

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        if (rd_en)
            rd_data <= mem[rd_addr];
    end
endmodule
