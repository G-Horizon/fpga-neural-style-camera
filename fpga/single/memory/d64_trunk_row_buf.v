// Row buffer for the trunk block's intermediates: A (conv1 -> conv2) and B
// (conv2 -> conv3).
//
// A and B are the same shape of memory and the same module.  Both hold a band of
// rows of 16-channel 8-bit activations, read and written in 8-channel words so
// that one word is exactly one IC group at the factorisations D3b chose:
//
//     A   11 rows x 64 cols x 2 words x 64 bit  = 90,112 bit = 11 KiB
//     B    8 rows x 64 cols x 2 words x 64 bit  = 65,536 bit =  8 KiB
//
// One read port and one write port, which is what a band schedule needs: conv1
// writes A while nothing reads it, conv2 reads A and writes B, conv3 reads B
// while nothing writes it.  The phases do not overlap each other, so a single
// port of each kind is enough -- unlike the trunk store, whose read port is
// occupied in two different phases and therefore had to be sized for the worse
// of them.
//
// The depth is a parameter rather than band*cols so a caller can size it to the
// band it actually runs; the band is a schedule decision and this is the
// storage that follows from it.
module d64_trunk_row_buf #(
    parameter DEPTH     = 704,    // rows * cols
    parameter BANKS     = 2,      // 16 channels / 8 per word
    parameter WORD_W    = 64,     // 8 channels x 8 bits
    parameter ADDR_W    = 10,
    parameter BANK_W    = 1
) (
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  wr_en,
    input  wire [ADDR_W-1:0]     wr_addr,
    input  wire [BANK_W-1:0]     wr_bank,
    input  wire [WORD_W-1:0]     wr_data,

    input  wire                  rd_en,
    input  wire [ADDR_W-1:0]     rd_addr,
    input  wire [BANK_W-1:0]     rd_bank,
    output wire [WORD_W-1:0]     rd_data
);

    genvar b;
    wire [BANKS*WORD_W-1:0] bank_q;

    generate
        for (b = 0; b < BANKS; b = b + 1) begin : g_bank
            // The attribute is load-bearing here for the same reason it is in
            // the trunk store, and it was measured there: without it efx_map can
            // decide the array is not inferable and build it from LUT RAM.
            (* ram_style = "block" *) reg [WORD_W-1:0] mem [0:DEPTH-1];
            reg [WORD_W-1:0] q;
            reg              rd_hit_q;

            wire wr_hit = wr_en && (wr_bank == b);
            wire rd_hit = rd_en && (rd_bank == b);

            // Unconditional read, enable applied to the output -- the inference
            // pattern the trunk store needed too.
            //
            // **`rd_hit` is registered with the data, and that is not a detail.**
            // The bank is part of the address: the port presents `(rd_addr,
            // rd_bank)` on one cycle and the word for *that* pair has to appear on
            // the next.  Gating the output with the *current* `rd_bank` instead
            // returns the bank that happens to be selected a cycle later, which is
            // harmless while the two move together -- as they do in the resource
            // root this module was measured in, where both come off an LFSR -- and
            // is wrong the moment a caller walks the banks in order.  D3c5-b's X
            // ring does exactly that, and every value it read came back one bank
            // along: a picture, not a failure.  One flip-flop a bank.
            always @(posedge clk) begin
                if (wr_hit)
                    mem[wr_addr] <= wr_data;
                q <= mem[rd_addr];
                rd_hit_q <= rd_hit;
            end

            assign bank_q[b*WORD_W +: WORD_W] = rd_hit_q ? q : {WORD_W{1'b0}};
        end
    endgenerate

    // One term per bank; exactly one has rd_hit.
    integer k;
    reg [WORD_W-1:0] acc;
    always @* begin
        acc = {WORD_W{1'b0}};
        for (k = 0; k < BANKS; k = k + 1)
            acc = acc | bank_q[k*WORD_W +: WORD_W];
    end

    assign rd_data = acc;

endmodule
