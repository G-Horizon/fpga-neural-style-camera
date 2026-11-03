// One-RAM candidate: RX data holds during PE TX payload writes; parser must not depend on it in those cycles.
`timescale 1ns/1ps
// 512-byte mode-0 full-duplex mailbox. S3 is master. RX and TX payload use
// separate clocked byte memories. Header registers preload first MISO bit
// without a SCLK edge. READY means committed response, never capture progress.
// Commands are immutable until PE response_commit acknowledges the request.
// The parser owns protocol/CRC/DDRx validation; only byte 5 NOP is interpreted.
module d64_spi_mailbox_shared_ram (
    input wire clk_pe,rst_n,
    input wire spi_sclk,spi_cs_n,spi_mosi,output wire spi_miso,
    output reg ready,
    output reg rx_done,input wire [8:0] rx_rd_addr,output wire [7:0] rx_rd_data,
    input wire tx_we,input wire [8:0] tx_addr,input wire [7:0] tx_data,
    input wire response_commit,
    output reg short_packet_fault,output reg busy_packet_fault,
    output reg tx_overlap_fault
);
    // One explicit EFX_DPRAM10 holds RX[0:511] and TXpayload[512:991].
    reg [7:0] tx_header[0:31];
    reg [11:0] rx_bit_count,tx_bit_count;
    reg [7:0] rx_shift,rx_opcode;
    reg rx_full;
    reg req_toggle,ack_toggle,complete_toggle,busy_toggle;
    (* async_reg="true" *) reg [1:0] ack_sync,req_sync,complete_sync,busy_sync,cs_sync;
    reg req_seen,busy_seen,cs_previous,transaction_open;
    reg start_complete,commit_pending;
    reg [2:0] end_wait;
    wire [7:0] tx_q;reg [7:0] tx_payload_hold;
    integer i;
    wire [7:0] incoming_byte={rx_shift[6:0],spi_mosi};
    wire request_free=req_toggle==ack_sync[1];
    wire [8:0] tx_byte=tx_bit_count[11:3];
    // At the last bit of a byte, fetch the next payload byte before the next
    // falling edge. During header clocks continually preload payload byte 0.
    // Prefetch next byte on its predecessor's FIRST bit; the final bit
    // is reserved for RX writes to this same SPI-domain RAM port.
    wire [8:0] tx_payload_addr=tx_byte<32 ? 9'd0 :
        tx_byte==511 ? 9'd479 : (tx_byte-9'd31);
    wire [7:0] tx_byte_value=tx_bit_count<256 ? tx_header[tx_bit_count[7:3]] : tx_payload_hold;
    assign spi_miso=spi_cs_n ? 1'b0 : tx_byte_value[3'd7-tx_bit_count[2:0]];

    // CS is only a reset/control input, not an internal event clock.
    always @(posedge spi_sclk or posedge spi_cs_n or negedge rst_n)begin
        if(!rst_n)begin rx_bit_count<=0;rx_shift<=0;rx_opcode<=0;rx_full<=0;end
        else if(spi_cs_n)begin rx_bit_count<=0;rx_shift<=0;rx_opcode<=0;rx_full<=0;end
        else if(!rx_full)begin
            rx_shift<=incoming_byte;
            if(rx_bit_count[2:0]==7)begin
                if(rx_bit_count[11:3]==5)rx_opcode<=incoming_byte;
                if(rx_bit_count==4095)rx_full<=1;
            end
            if(rx_bit_count!=4095)rx_bit_count<=rx_bit_count+1'b1;
        end
    end
    always @(posedge spi_sclk or negedge rst_n)begin
        if(!rst_n)begin ack_sync<=0;req_toggle<=0;complete_toggle<=0;busy_toggle<=0;end
        else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(!spi_cs_n&&!rx_full&&rx_bit_count==4095)begin
                complete_toggle<=~complete_toggle;
                if(rx_opcode!=0)begin
                    if(request_free)req_toggle<=~req_toggle;
                    else busy_toggle<=~busy_toggle;
                end
            end
        end
    end
    // Byte writes finish only on the eighth mode-0 sampling edge. No memory
    // transaction is undone by CS release; partial data is never published.
    wire spi_write=!spi_cs_n&&!rx_full&&rx_bit_count[2:0]==7&&request_free;
    wire spi_read=!spi_cs_n&&tx_bit_count[2:0]==0;
    wire pe_write=tx_we&&spi_cs_n&&tx_addr>=32;
    wire [9:0] ram_addr_a=spi_write ? {1'b0,rx_bit_count[11:3]} : {1'b1,tx_payload_addr};
    wire [9:0] ram_addr_b=pe_write ? 10'd480+{1'b0,tx_addr} : {1'b0,rx_rd_addr};
    EFX_DPRAM10 #(.READ_WIDTH_A(8),.WRITE_WIDTH_A(8),.READ_WIDTH_B(8),.WRITE_WIDTH_B(8),
        .OUTPUT_REG_A(1'b0),.OUTPUT_REG_B(1'b0),
        .WRITE_MODE_A("NO_CHANGE"),.WRITE_MODE_B("NO_CHANGE")) u_shared_ram(
        .CLKA(spi_sclk),.WEA(spi_write),.CLKEA(spi_write||spi_read),.RSTA(1'b0),
        .ADDRENA(1'b1),.WDATAA(incoming_byte),.ADDRA(ram_addr_a),.RDATAA(tx_q),
        .CLKB(clk_pe),.WEB(pe_write),.CLKEB(1'b1),.RSTB(1'b0),
        .ADDRENB(1'b1),.WDATAB(tx_data),.ADDRB(ram_addr_b),.RDATAB(rx_rd_data));
    always @(negedge spi_sclk or posedge spi_cs_n or negedge rst_n)begin
        if(!rst_n)begin tx_bit_count<=0;tx_payload_hold<=0;end
        else if(spi_cs_n)begin tx_bit_count<=0;tx_payload_hold<=0;end
        else begin
            if(tx_bit_count==255||(tx_bit_count>=256&&tx_bit_count[2:0]==7&&tx_bit_count!=4095))
                tx_payload_hold<=tx_q;
            if(tx_bit_count!=4095)tx_bit_count<=tx_bit_count+1'b1;
        end
    end

    // PE can service a command and assert READY with SCLK stopped. The ACK
    // synchronizer only controls the next RX ownership decision, not READY.
    always @(posedge clk_pe or negedge rst_n)begin
        if(!rst_n)begin
            req_sync<=0;complete_sync<=0;busy_sync<=0;cs_sync<=3;
            req_seen<=0;busy_seen<=0;ack_toggle<=0;ready<=0;rx_done<=0;
            cs_previous<=1;transaction_open<=0;start_complete<=0;end_wait<=0;
            commit_pending<=0;short_packet_fault<=0;busy_packet_fault<=0;tx_overlap_fault<=0;
            for(i=0;i<32;i=i+1)tx_header[i]<=0;
        end else begin
            req_sync<={req_sync[0],req_toggle};complete_sync<={complete_sync[0],complete_toggle};
            busy_sync<={busy_sync[0],busy_toggle};cs_sync<={cs_sync[0],spi_cs_n};
            cs_previous<=cs_sync[1];rx_done<=0;
            if(req_sync[1]!=req_seen&&cs_sync[1]&&spi_cs_n)begin
                req_seen<=req_sync[1];rx_done<=1;ready<=0;
            end
            if(busy_sync[1]!=busy_seen)begin busy_seen<=busy_sync[1];busy_packet_fault<=1;end
            if(!cs_sync[1])ready<=0;
            if(cs_previous&&!cs_sync[1])begin
                transaction_open<=1;start_complete<=complete_sync[1];end_wait<=0;
            end
            if(!cs_previous&&cs_sync[1]&&transaction_open)end_wait<=4;
            else if(end_wait!=0)begin
                end_wait<=end_wait-1'b1;
                if(end_wait==1)begin
                    if(complete_sync[1]==start_complete)short_packet_fault<=1;
                    transaction_open<=0;
                end
            end
            if(response_commit)commit_pending<=1;
            if((response_commit||commit_pending)&&spi_cs_n&&cs_sync==3)begin
                ready<=1;commit_pending<=0;ack_toggle<=req_seen;
            end
            if(tx_we&&!spi_cs_n)tx_overlap_fault<=1;
            if(tx_we&&spi_cs_n&&tx_addr<32)tx_header[tx_addr[4:0]]<=tx_data;
        end
    end

endmodule
