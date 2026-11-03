// A registered-read block RAM: one write port, one read port, enable on the
// output.
//
// There is nothing clever here and that is the point.  Three arrays in the D3c4c-c
// integration root are this shape -- the input staging window, the two-row output
// scratch, and (through `d64_weight_store`, which is the same pattern at a
// different width) the weight image -- and the resource report can only name a
// resource it can see as an instance.  A bare `reg [..] mem [..]` inside a
// generate block is attributed to the enclosing module, so "the scratch is four
// RAM10 and the optimizer did not delete it" would not be checkable.
//
// Two details are load-bearing and both are copied from `d64_weight_store`,
// where they were measured rather than assumed:
//
//   * the read is **unconditional** and `re` is applied to the output.  A read
//     enable on the address makes the mapper build a logic memory instead of a
//     block.
//   * `ram_style = "block"` is an instruction, not a hint.
module d64_tc_ram #(
    parameter DW = 80,
    parameter AW = 9
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] wa,
    input  wire [DW-1:0] wd,
    input  wire          re,
    input  wire [AW-1:0] ra,
    output wire [DW-1:0] rd
);

    (* ram_style = "block" *) reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [DW-1:0] q;

    always @(posedge clk) begin
        if (we) mem[wa] <= wd;
        q <= mem[ra];
    end

    assign rd = re ? q : {DW{1'b0}};

endmodule
