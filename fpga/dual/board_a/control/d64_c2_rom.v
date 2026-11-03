`timescale 1ns/1ps
// A plain synchronous reference ROM, parameterised in width and depth.
//
// **An empty `INIT_FILE` is a defined all-zero table.**  The contents are cleared
// first and read second, so a build that forgets the parameter gets zeros: every
// comparison then fails and the probe says "everything is wrong" rather than
// quietly agreeing with itself.  A `$readmemh` that cannot open its file is
// silent, and silence that reads as a pass is the failure this project has paid
// for more than once -- the chunk CRC ROM carries the same guard for the same
// reason.
//
// No `generate`, no attributes, and no nested declarations: ModelSim 10.4
// segfaults during elaboration on the version of that other ROM that had them,
// and a simulator that dies before time zero is a worse oracle than a slightly
// plainer module.
module d64_c2_rom #(
    parameter DW = 64,
    parameter AW = 8,
    parameter INIT_FILE = ""
)(
    input  wire           clk,
    input  wire [AW-1:0]  addr,
    output reg  [DW-1:0]  dout
);
    reg [DW-1:0] mem [0:(1<<AW)-1];
    integer i;

    initial begin
        for (i = 0; i < (1 << AW); i = i + 1) mem[i] = {DW{1'b0}};
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) dout <= mem[addr];
endmodule
