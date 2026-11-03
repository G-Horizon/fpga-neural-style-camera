// One payload bank: independent write/read clocks, synchronous read, no RAM reset.
// The descriptor handshake, not simultaneous access semantics, owns each bank.
module eth_packet_ram #(
    parameter ADDR_WIDTH=10,
    parameter DEPTH=1024
)(
    input wr_clk, input wr_en,
    input [ADDR_WIDTH-1:0] wr_addr, input [7:0] wr_data,
    input rd_clk, input [ADDR_WIDTH-1:0] rd_addr,
    output reg [7:0] rd_data
);
    (* ram_style="block" *) reg [7:0] payload [0:DEPTH-1];
    always @(posedge wr_clk) if(wr_en) payload[wr_addr] <= wr_data;
    always @(posedge rd_clk) rd_data <= payload[rd_addr];
endmodule

// Shared reset assertion is mandatory. Each domain releases locally after two
// clock edges. A stopped clock therefore cannot publish a partial reset epoch.
module eth_reset_release(input clk, input rst_n, output local_rst_n);
    (* async_reg="true" *) reg release_s1, release_s2;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin release_s1<=0; release_s2<=0; end
        else begin release_s1<=1; release_s2<=release_s1; end
    assign local_rst_n=release_s2;
endmodule
