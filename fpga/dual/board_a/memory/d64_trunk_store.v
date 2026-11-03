// The d64 trunk stream store: 64x64x32 at 10 bits, the tensor M1 depends on.
//
//     logical capacity   64 * 64 * 32 * 10 = 1,310,720 bits = 160 KiB
//     capacity floor     ceil(1,310,720 / 10,240) = 128 RAM10
//     mapped, D3c2a                                160 RAM10
//
// Why this leaf exists before any convolution engine
// --------------------------------------------------
//
// Every memory policy that matters -- M1 trunk resident, and M2/M3 which build on
// it -- assumes this tensor can actually live on chip.  128 RAM10 is a capacity
// division, A5 showed the physical count depends on how an array is organised,
// and the measurement said 160.  That is settled; what was not settled, and is
// what this revision is about, is whether the store can *feed* the PE.
//
// The interface is the PE's shape, not a generic memory's
// -------------------------------------------------------
//
// The first version emitted one channel per read port, OR-reduced from the
// banks.  That is 10 bits per cycle against a `PIX1 x IC8 x OC16` trunk conv1
// that needs eight activation channels per cycle: the capacity was there and the
// bandwidth was not, and the earlier document's claim that "read lanes per cycle
// is CHANNELS because every bank can be read at once" described an intention the
// module did not implement.
//
// So a read is a whole bank word:
//
//     read    pixel address + bank id  ->  CH_PER_BANK x DATA_W bits
//     write   pixel address + bank id  <-  CH_PER_BANK x DATA_W bits
//
// With BANKS=4 and CH_PER_BANK=8 that is 80 bits each way, which is exactly
// `IC_LANES=8` into conv1 and exactly the eight requantised outputs conv3
// produces per cycle.  The datapath shape and the memory shape are the same
// number, which is why `BANKS` is a parameter and 4 is the default rather than a
// free choice.
//
// The read port is single, and the skip does not need a second one
// ---------------------------------------------------------------
//
// A block is `x + conv3(conv2(conv1(x)))`, so the add needs `x` back after conv3
// has run.  The tempting answer is a second read port to fetch it again.  The
// cheaper one is to take `x` out of the store once, feed it to conv1 and drop a
// copy into a small delay FIFO on the way past -- the delay only has to cover a
// 1x1, a 3x3 line buffer and another 1x1, so it is a row-scale buffer and not
// another 160 KiB.  One read port is therefore the design, and `RD_PORTS` stays
// a parameter only so that a future need can be priced.
//
// One write port, fanned out to every replica
// -------------------------------------------
//
// A RAM10 has one read and one write port.  Replication buys read ports and no
// more write ports, which is the right shape: the trunk is written once per
// pixel per pass and read many times.
//
// Reads are registered, so a consumer sees its data one cycle after the address.
module d64_trunk_store #(
    parameter PIXELS    = 4096,   // 64 x 64
    parameter CHANNELS  = 32,
    parameter DATA_W    = 10,
    parameter BANKS     = 4,
    parameter RD_PORTS  = 1,
    // Derived, and parameters rather than localparams: ANSI port declarations
    // cannot see a localparam, which every root in this repo notes.
    parameter CH_PER_BANK = CHANNELS / BANKS,
    parameter WORD_W      = CH_PER_BANK * DATA_W,
    parameter PIX_ADDR_W  = 12,
    parameter BANK_SEL_W  = 2
) (
    input  wire                           clk,
    input  wire                           rst_n,

    // Write: every channel of one bank, at one pixel, in one cycle.
    //
    // Bank-wide rather than channel-wide, and that is measured: writing one
    // channel into a multi-channel word is a read-modify-write, and efx_map will
    // not turn that into a RAM.  With CH_PER_BANK=2 every bank fell back to
    // logic memory, 128 fragments.
    input  wire                           wr_en,
    input  wire [PIX_ADDR_W-1:0]          wr_addr,
    input  wire [BANK_SEL_W-1:0]          wr_bank,
    input  wire [WORD_W-1:0]              wr_data,

    // Read: every channel of one bank, at one pixel, per port.
    input  wire [RD_PORTS-1:0]            rd_en,
    input  wire [RD_PORTS*PIX_ADDR_W-1:0] rd_addr,
    input  wire [RD_PORTS*BANK_SEL_W-1:0] rd_bank,
    output wire [RD_PORTS*WORD_W-1:0]     rd_word
);

    genvar b, p;
    wire [RD_PORTS*BANKS*WORD_W-1:0] bank_word;

    generate
        for (b = 0; b < BANKS; b = b + 1) begin : g_bank
            for (p = 0; p < RD_PORTS; p = p + 1) begin : g_port

                // The attribute is load-bearing, not decoration.  Without it
                // efx_map does not infer block RAM for these arrays at all: it
                // reports "write ports > 2" for every bank and maps them into
                // logic memory, 5120 bits at a time.  The likely cause is that it
                // first tries to merge the banks -- they share an address space --
                // and then finds their write enables differ.  A single 4096x10
                // array in the same style infers correctly without the attribute,
                // which is what localised the fault to the banked structure.
                (* ram_style = "block" *) reg [WORD_W-1:0] mem [0:PIXELS-1];
                reg [WORD_W-1:0] q;

                wire [PIX_ADDR_W-1:0]  ra = rd_addr[p*PIX_ADDR_W +: PIX_ADDR_W];
                wire [BANK_SEL_W-1:0]  rb = rd_bank[p*BANK_SEL_W +: BANK_SEL_W];

                wire wr_hit = wr_en && (wr_bank == b);
                wire rd_hit = rd_en[p] && (rb == b);

                // The read is UNCONDITIONAL.  Gating it on rd_hit makes efx_map
                // count a second port and give up -- that was the first version's
                // failure, 256 fragments and no EFX_RAM10 line at all.  The
                // registered read with the enable applied to the output is what
                // the inference wants, and the bank select is the reduction
                // below anyway.
                //
                // The write is a full word.  No mask, no slice select: that is
                // what makes it a RAM write rather than a read-modify-write.
                always @(posedge clk) begin
                    if (wr_hit)
                        mem[wr_addr] <= wr_data;
                    q <= mem[ra];
                end

                // Each bank drives its OWN slot.  Driving rd_word directly from
                // every bank is a multiple-driver conflict -- the simulator
                // resolves it to x -- so the banks are collected and reduced.
                // Exactly one bank has rd_hit, so the reduction is the mux.
                assign bank_word[(p*BANKS + b)*WORD_W +: WORD_W] =
                    rd_hit ? q : {WORD_W{1'b0}};
            end
        end
    endgenerate

    // One reduction per read port, BANKS terms of WORD_W bits.  The whole word
    // comes out: there is no rd_chan and no per-channel select, because a
    // per-channel output was the bandwidth bug this revision exists to fix.
    genvar pp;
    generate
        for (pp = 0; pp < RD_PORTS; pp = pp + 1) begin : g_reduce
            integer k;
            reg [WORD_W-1:0] acc;
            always @* begin
                acc = {WORD_W{1'b0}};
                for (k = 0; k < BANKS; k = k + 1)
                    acc = acc | bank_word[(pp*BANKS + k)*WORD_W +: WORD_W];
            end
            assign rd_word[pp*WORD_W +: WORD_W] = acc;
        end
    endgenerate

endmodule
