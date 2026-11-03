`timescale 1ns/1ps
// FS01 + sequence(u32 LE) + WORDS*u32 LE + IEEE CRC32(sequence+words).
module d64_status_uart #(
    parameter integer CLK_HZ=100000000, BAUD=115200, WORDS=11,
    parameter integer PERIOD_CYCLES=25000000,
    parameter [31:0] PACKET_MAGIC=32'h31305346
)(input wire clk,input wire rst_n,input wire [WORDS*32-1:0] words,output wire tx);
    localparam IDLE=0,SEND=1,WAIT_LOW=2,WAIT_HIGH=3;
    localparam integer CRC_START=8+WORDS*4;
    localparam integer TIMER_W=$clog2(PERIOD_CYCLES+1);
    reg[1:0]state;reg[TIMER_W-1:0]timer;reg[31:0]seq,crc;
    reg[WORDS*32-1:0]snapshot;
    reg[7:0]index,tx_data;reg tx_valid;wire tx_ready;
    reg[7:0]next_byte;
    function[31:0]update_crc;input[31:0]old;input[7:0]b;reg[31:0]c;integer i;
        begin c=old^{24'd0,b};for(i=0;i<8;i=i+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;update_crc=c;end
    endfunction
    always @*begin
        if(index==0)next_byte=PACKET_MAGIC[7:0];
        else if(index==1)next_byte=PACKET_MAGIC[15:8];
        else if(index==2)next_byte=PACKET_MAGIC[23:16];
        else if(index==3)next_byte=PACKET_MAGIC[31:24];
        else if(index<8)begin
            case(index[1:0])
                0:next_byte=seq[7:0];1:next_byte=seq[15:8];
                2:next_byte=seq[23:16];3:next_byte=seq[31:24];
            endcase
        end
        else if(index<CRC_START)next_byte=snapshot[7:0];
        else begin
            case(index[1:0])
                0:next_byte=~crc[7:0];1:next_byte=~crc[15:8];
                2:next_byte=~crc[23:16];3:next_byte=~crc[31:24];
            endcase
        end
    end
    uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD))u_tx(.clk(clk),.rst_n(rst_n),.data(tx_data),.valid(tx_valid),.ready(tx_ready),.tx(tx));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;timer<=0;seq<=0;crc<=32'hffffffff;snapshot<=0;index<=0;tx_data<=0;tx_valid<=0;end
        else begin
            tx_valid<=0;
            case(state)
                IDLE:if(timer==PERIOD_CYCLES-1)begin
                    timer<=0;snapshot<=words;index<=0;crc<=32'hffffffff;state<=SEND;
                end else timer<=timer+1;
                SEND:if(tx_ready)begin
                    tx_data<=next_byte;tx_valid<=1;state<=WAIT_LOW;
                    if(index>=4&&index<CRC_START)crc<=update_crc(crc,next_byte);
                    if(index>=8&&index<CRC_START)snapshot<=snapshot>>8;
                end
                WAIT_LOW:if(!tx_ready)state<=WAIT_HIGH;
                WAIT_HIGH:if(tx_ready)begin
                    if(index==CRC_START+3)begin state<=IDLE;seq<=seq+1;end
                    else begin index<=index+1;state<=SEND;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
