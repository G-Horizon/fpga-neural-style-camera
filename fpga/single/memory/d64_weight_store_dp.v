// The weight store, as a true dual-port array.
//
// d64_weight_store is one word per cycle, and that word has to be 1024 bits to
// feed 128 MACs at IC8 x OC16.  The mapper prices a 1024-bit word at
// ceil(1024/20) = 52 RAM10 columns -- a RAM10 is 512 words of 20 bits, and the
// width is what sets the block count, so a 1024-bit read costs 52 blocks whether
// it is 220 words deep or 130.  Measured, at every depth tried.  Fifty-two
// blocks does not fit M1.
//
// But two 512-bit reads are not the same thing as one 1024-bit read.  A RAM10 is
// single-port; a *DPRAM10* is the same 10,240 bits with two independent
// read-or-write ports, and one of those costs one block.  Two 512-bit reads out
// of 26 columns is 26 blocks, not 52 -- if the mapper infers the dual-port
// primitive instead of replicating a single-port array.
//
// That is the whole question this module exists to ask.  Each port is written
// with the canonical pattern -- an unconditional read plus a conditional write
// against the same address -- so the tool can see a read/write mux per port
// rather than two memories.
//
// The intended use: a tile is 8 IC x 8 OC = 64 weights = 512 bits, and one MAC
// cycle for the 128-MAC array reads tile 2k on port A (OC 0..7) and tile 2k+1 on
// port B (OC 8..15), both at once.  The loader uses the same two ports during a
// layer switch, when nothing is reading.
module d64_weight_store_dp #(
    parameter TILES  = 440,
    parameter WORD_W = 512,
    parameter ADDR_W = 9
) (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                we_a,
    input  wire [ADDR_W-1:0]   addr_a,
    input  wire [WORD_W-1:0]   wdata_a,
    output wire [WORD_W-1:0]   rdata_a,

    input  wire                we_b,
    input  wire [ADDR_W-1:0]   addr_b,
    input  wire [WORD_W-1:0]   wdata_b,
    output wire [WORD_W-1:0]   rdata_b
);

    (* ram_style = "block" *) reg [WORD_W-1:0] mem [0:TILES-1];
    reg [WORD_W-1:0] qa, qb;

    always @(posedge clk) begin
        if (we_a)
            mem[addr_a] <= wdata_a;
        qa <= mem[addr_a];
        if (we_b)
            mem[addr_b] <= wdata_b;
        qb <= mem[addr_b];
    end

    assign rdata_a = qa;
    assign rdata_b = qb;

endmodule
