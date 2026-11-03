`timescale 1ns/1ps
// DDR-only release stage. Existing global reset still owns asynchronous
// assertion; deassertion waits one further25MHz edge. Other domains keep their
// existing reset paths. This does not waive any controller SR timing arc.
module d64_ddr_clk25_release (
    input wire clk_25m,
    input wire global_rst_n,
    output reg ddr_release_n
);
    initial ddr_release_n=1'b0;
    always @(posedge clk_25m or negedge global_rst_n)
        if(!global_rst_n) ddr_release_n<=1'b0;
        else ddr_release_n<=1'b1;
endmodule
