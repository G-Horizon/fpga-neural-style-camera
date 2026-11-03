`timescale 1ns/1ps
// ABI2 physical write port: 494 x 512 bits at the existing PE clock.
// ABI1-equivalent logical read port: 247 x 1024 bits, reconstructed from two
// 200MHz reads. Required related-clock edges are PE+2.5ns and PE+7.5ns.
// The address decoder reverses the documented per-layer OC-half packing.
module d64_weight_store_abi2_mp2 #(
    parameter INIT_FILE="",
    parameter integer EN_SABOTAGE=0,
    parameter integer UNGATED_READ=0
) (
    input wire clk,clk_fast,rst_n,
    input wire wr_en,rd_en,
    input wire [8:0] wr_addr,
    input wire [7:0] rd_addr,
    input wire [511:0] wr_data,
    output wire rd_valid,
    output wire [1023:0] rd_data
);
    (* ram_style="block" *) reg [511:0] mem [0:493];
    reg [7:0] ra_q;
    reg re_q;
    reg [511:0] ram_q,low_q;
    // PE reset release is synchronous to its positive edge. The first fast
    // positive edge is +2.5ns and reads low; the next +7.5ns edge reads high.
    // A flip-flop phase avoids routing a PLL clock into non-clock RAM/FF pins.
    reg half;
    always @(posedge clk_fast or negedge rst_n)begin
        if(!rst_n)half<=1'b0;
        else half<=~half;
    end
    wire [8:0] physical_ra={ra_q,half};
    generate if(INIT_FILE!="")begin:g_init
        initial $readmemh(INIT_FILE,mem);
    end endgenerate
    always @(posedge clk)begin
        if(wr_en)mem[wr_addr]<=wr_data;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin ra_q<=0;re_q<=0;end
        else begin ra_q<=rd_addr;re_q<=rd_en;end
    end
    always @(posedge clk_fast)begin
        ram_q<=mem[physical_ra];
        if(half)low_q<=ram_q;
    end
    // ABI3 guarantees these two physical halves already have native layout.
    // Only rd_valid responses are consumed by the existing CNN clients.
    assign rd_valid=re_q;
    assign rd_data=(UNGATED_READ || (EN_SABOTAGE ? rd_en : re_q)) ? {ram_q,low_q} : 1024'd0;

endmodule
