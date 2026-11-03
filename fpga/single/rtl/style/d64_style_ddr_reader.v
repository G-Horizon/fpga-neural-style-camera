`timescale 1ns/1ps
// Ordered, backpressurable profile stream; bursts never cross a 4 KiB page.
module d64_style_ddr_reader #(
    parameter integer MAX_BURST_BEATS=256,
    parameter integer ABORT_EN=0,
    parameter integer RESERVE_AR_READY=0
) (
    input wire clk,input wire rst_n,input wire request,
    input wire [31:0] base_addr,input wire [15:0] beats,
    output wire stream_valid,input wire stream_ready,output wire [127:0] stream_data,
    output reg stream_error,output wire idle,
    output wire arvalid,input wire arready,output wire [31:0] araddr,
    output wire [7:0] arlen,output wire [2:0] arsize,output wire [1:0] arburst,
    input wire rvalid,output wire rready,input wire [127:0] rdata,input wire rlast,input wire [1:0] rresp,
    input wire abort
);
    localparam IDLE=0,AR=1,R=2,DRAIN=3;
    reg [1:0] state;
    reg [31:0] address;
    reg [15:0] left;
    reg [8:0] count;
    reg abort_pending;
    reg ar_exposed;
    wire abort_active=(ABORT_EN!=0)&&abort;
    // The optional one-beat profile mode reserves consumer READY before
    // exposing a new address. Once exposed, AXI VALID is held until accepted.
    wire ar_allowed=(RESERVE_AR_READY==0)||stream_ready||abort_active||ar_exposed;
    wire [8:0] page_beats=9'd256-{1'b0,address[11:4]};
    wire [8:0] limited=(page_beats>MAX_BURST_BEATS)?MAX_BURST_BEATS:page_beats;
    wire [8:0] chunk=left<limited?left[8:0]:limited;
    assign araddr=address;assign arlen=chunk-1;assign arsize=4;assign arburst=1;assign arvalid=(state==AR)&&ar_allowed;
    assign idle=state==IDLE;
    assign stream_valid=(state==R)&&!abort_active&&rvalid&&(rresp==0)&&(rlast==(count==1));
    assign stream_data=rdata;
    assign rready=(state==DRAIN)||((state==R)&&(abort_active||stream_ready||rresp!=0||rlast!=(count==1)));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;address<=0;left<=0;count<=0;stream_error<=0;abort_pending<=0;ar_exposed<=0;end
        else begin
            stream_error<=0;
            if(state==IDLE)begin abort_pending<=0;ar_exposed<=0;end
            if(request&&state==IDLE&&!abort_active)begin
                stream_error<=0;address<=base_addr;left<=beats;
                if(base_addr[3:0]!=0||beats==0)stream_error<=1;else state<=AR;
            end
            // ARVALID cannot be withdrawn once presented. Latch cancellation
            // while waiting for ARREADY, retain the descriptor, then drain the
            // accepted burst even if abort has already returned low.
            if(state==AR)begin
                if(abort_active)abort_pending<=1;
                if(arvalid&&arready)begin
                    count<=chunk;state<=(abort_active||abort_pending)?DRAIN:R;ar_exposed<=0;
                end else if(arvalid)ar_exposed<=1;
            end
            if(state==R&&abort_active)begin
                // The consumer can have stopped ready after header/CRC/timeout
                // failure. Drain independently of it, ending only on real LAST.
                state<=(rvalid&&rlast)?IDLE:DRAIN;
            end else if(state==R&&rvalid&&rready)begin
                if(rresp!=0||rlast!=(count==1))begin
                    stream_error<=1;state<=rlast?IDLE:DRAIN;
                end else begin
                    count<=count-1;left<=left-1;address<=address+16;
                    if(count==1)state<=left==1?IDLE:AR;
                end
            end
            if(state==DRAIN&&rvalid&&rlast)state<=IDLE;
        end
    end
endmodule
