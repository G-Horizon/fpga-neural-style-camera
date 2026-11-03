`timescale 1ns/1ps
// Laboratory FSC4/FSC5/FSC6 protocol. Full status/epoch and fault words remain
// 32 bits; only optional telemetry fields are omitted from the packet.
module d64_compact_status_uart #(
    parameter integer CLK_HZ=100000000,BAUD=115200,WORDS=4,
    parameter integer PERIOD_CYCLES=25000000
)(
    input wire clk,rst_n,
    input wire [351:0] words,
    input wire [31:0] camera_frame_id,
    output wire tx
);
    wire [191:0] compact_words={camera_frame_id,words[63:32],
        words[191:160],words[127:96],words[95:64],words[31:0]};
    localparam [31:0] MAGIC=WORDS==4 ? 32'h34435346 :
        WORDS==5 ? 32'h35435346 : 32'h36435346;
    d64_status_uart #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.WORDS(WORDS),
        .PERIOD_CYCLES(PERIOD_CYCLES),.PACKET_MAGIC(MAGIC))u_status(
        .clk(clk),.rst_n(rst_n),.words(compact_words[WORDS*32-1:0]),.tx(tx));
    // synthesis translate_off
    initial if(WORDS<4||WORDS>6)$fatal(1,"COMPACT_UART_WORDS_MUST_BE_4_5_6");
    // synthesis translate_on
endmodule
