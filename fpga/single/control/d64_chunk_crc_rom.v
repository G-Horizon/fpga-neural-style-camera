`timescale 1ns/1ps
// The expected per-chunk CRCs, one 32-bit word per (stage, chunk).
//
//   address  0.. 63   C1 -> the A ring's RAM writes,     chunk 0..63
//   address 64..127   C2 -> the B ring's RAM writes,     chunk 0..63
//   address 128..255  C3 -> the requant branch write,    chunk 0..127
//
// **The address is the tag.**  The expected value and the chunk it belongs to
// travel together as one word, so a mismatch cannot be reported against the
// wrong chunk by a separate tag register disagreeing with the address that was
// actually presented.
//
// **An empty `INIT_FILE` is a defined state, not an accident.**  The table is
// cleared first and only then read, so a build that forgets the parameter gets
// 256 zero words: every chunk mismatches and the probe says "everything is
// wrong" rather than quietly agreeing with itself.  A `$readmemh` that does not
// open is silent, and silence that reads as a pass is the failure this stage has
// paid for more than once.
//
// **No `generate`, no attributes.**  Written in the plainest form this file can
// take, because ModelSim 10.4 segfaults during elaboration on the version that
// had both -- and a simulator that dies before time zero is a worse oracle than
// a slightly less elegant module.
//
// Synchronous read, one cycle of latency: the address is registered, so the data
// for chunk N arrives while chunk N+1's events may already be folding.
module d64_chunk_crc_rom #(
    parameter INIT_FILE = ""
)(
    input  wire        clk,
    input  wire [7:0]  addr,
    output reg  [31:0] dout
);
    reg [31:0] mem [0:255];
    integer i;

    initial begin
        for (i = 0; i < 256; i = i + 1) mem[i] = 32'd0;
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) dout <= mem[addr];
endmodule
