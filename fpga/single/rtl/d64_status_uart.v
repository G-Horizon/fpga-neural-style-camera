`timescale 1ns/1ps
module d64_status_uart #(
 parameter integer CLK_HZ=100000000,BAUD=115200,WORDS=11,PERIOD_CYCLES=25000000,
 parameter[31:0]PACKET_MAGIC=32'h31305346,
 parameter integer DSP_REGISTER_ENABLE=0
)(input wire clk,rst_n,input wire[WORDS*32-1:0]words,output wire tx);
 generate if(DSP_REGISTER_ENABLE!=0)begin:g_dsp
  d64_status_uart_dsp_core #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.WORDS(WORDS),.PERIOD_CYCLES(PERIOD_CYCLES),.PACKET_MAGIC(PACKET_MAGIC))u(.clk(clk),.rst_n(rst_n),.words(words),.tx(tx));
 end else begin:g_ff
  d64_status_uart_ff_core #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.WORDS(WORDS),.PERIOD_CYCLES(PERIOD_CYCLES),.PACKET_MAGIC(PACKET_MAGIC))u(.clk(clk),.rst_n(rst_n),.words(words),.tx(tx));
 end endgenerate
endmodule
`timescale 1ns/1ps
// FS01 + sequence(u32 LE) + WORDS*u32 LE + IEEE CRC32(sequence+words).
module d64_status_uart_dsp_core #(
    parameter integer CLK_HZ=100000000, BAUD=115200, WORDS=11,
    parameter integer PERIOD_CYCLES=25000000,
    parameter [31:0] PACKET_MAGIC=32'h31305346
)(input wire clk,input wire rst_n,input wire [WORDS*32-1:0] words,output wire tx);
    localparam IDLE=0,SEND=1,WAIT_LOW=2,WAIT_HIGH=3;
    localparam integer CRC_START=8+WORDS*4;
    localparam integer TIMER_W=$clog2(PERIOD_CYCLES+1);
    reg[1:0]state;reg[TIMER_W-1:0]timer;wire[31:0]seq,crc;
    localparam integer PACK_CRC=(WORDS%3==2);
    wire[WORDS*32-1:0]snapshot;
    reg[7:0]index,tx_data;reg tx_valid;wire tx_ready;
    reg[7:0]next_byte;
    reg crc_active;reg[2:0]crc_bit_index;

    wire[31:0]crc_next=(crc>>1)^((crc[0]^tx_data[crc_bit_index])?32'hedb88320:32'd0);
    wire snapshot_load=state==IDLE&&timer==PERIOD_CYCLES-1;
    wire snapshot_shift=state==SEND&&tx_ready&&index>=8&&index<CRC_START;
    wire[WORDS*32-1:0]snapshot_next=snapshot_load?words:(snapshot>>8);
    wire snapshot_ce=snapshot_load||snapshot_shift;
    generate if(!PACK_CRC)begin:g_crc_ff
        reg[31:0]crc_ff;
        assign crc=crc_ff;
        always @(posedge clk or negedge rst_n)
         if(!rst_n)crc_ff<=32'hffffffff;
         else if(snapshot_load)crc_ff<=32'hffffffff;
         else if(crc_active)crc_ff<=crc_next;
    end endgenerate
    genvar part;
    generate for(part=0;part<(WORDS*32+47)/48;part=part+1)begin:g_snapshot_dsp
        localparam integer W=(WORDS*32-part*48>=48)?48:WORDS*32-part*48;
        wire[47:0]dsp_o;
        wire[47:0]dsp_i;
        wire last_crc=PACK_CRC&&(part==(WORDS*32+47)/48-1);
        wire group_ce=snapshot_ce||(last_crc&&crc_active);
        if(PACK_CRC&&(part==(WORDS*32+47)/48-1))begin:g_crc_packed
            assign dsp_i={snapshot_load?32'd0:(crc_active?~crc_next:~crc),snapshot_ce?snapshot_next[part*48+:W]:snapshot[part*48+:W]};
            assign crc=~dsp_o[47:16];
        end else begin:g_plain
            assign dsp_i={{(48-W){1'b0}},snapshot_next[part*48+:W]};
        end
        wire[47:0]cascade;
        EFX_DSP48 #(.SIGNED(0),.A_REG(1),.B_REG(1),.C_REG(1),.M_SEL("P"),.N_SEL("CONST0"),.CASCOUT_SEL("ABC"))u_pack(
         .A({3'd0,dsp_i[47:32]}),.B({2'd0,dsp_i[31:16]}),.C({2'd0,dsp_i[15:0]}),.CASCIN(48'd0),
         .OP(2'd0),.SHIFT_ENA(1'b0),.CLK(clk),.CE(group_ce),.RST(!rst_n),.O(),.CASCOUT(cascade),.OVFL());
        EFX_DSP48 #(.SIGNED(0),.M_SEL("P"),.N_SEL("CASCIN"))u_out(
         .A(19'd0),.B(18'd0),.C(18'd0),.CASCIN(cascade),.OP(2'd0),.SHIFT_ENA(1'b0),
         .CLK(1'b0),.CE(1'b0),.RST(1'b0),.O(dsp_o),.CASCOUT(),.OVFL());
        assign snapshot[part*48+:W]=dsp_o[W-1:0];
    end endgenerate
    wire[47:0]seq_dsp;
    wire seq_ce=state==WAIT_HIGH&&tx_ready&&index==CRC_START+3;
    EFX_DSP48 #(.SIGNED(0),.W_REG(1),.M_SEL("C"),.N_SEL("W"))u_seq_dsp(
     .A(19'd0),.B(18'd0),.C(18'd1),.CASCIN(48'd0),.OP(2'd0),.SHIFT_ENA(1'b0),
     .CLK(clk),.CE(seq_ce),.RST(!rst_n),.O(seq_dsp),.CASCOUT(),.OVFL());
    assign seq=seq_dsp[31:0];
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
        if(!rst_n)begin state<=IDLE;timer<=0;index<=0;tx_data<=0;tx_valid<=0;crc_active<=0;crc_bit_index<=0;end
        else begin
            tx_valid<=0;
            // Compute one CRC bit per idle APP clock while the UART sends this byte.
            // The next SEND occurs after the full 8N1 byte, retaining wire timing.
            if(crc_active)begin
                if(crc_bit_index==7)crc_active<=0;
                else crc_bit_index<=crc_bit_index+1'b1;
            end
            case(state)
                IDLE:if(timer==PERIOD_CYCLES-1)begin
                    timer<=0;index<=0;state<=SEND;
                end else timer<=timer+1;
                SEND:if(tx_ready)begin
                    tx_data<=next_byte;tx_valid<=1;state<=WAIT_LOW;
                    if(index>=4&&index<CRC_START)begin crc_active<=1;crc_bit_index<=0;end
                    
                end
                WAIT_LOW:if(!tx_ready)state<=WAIT_HIGH;
                WAIT_HIGH:if(tx_ready)begin
                    if(index==CRC_START+3)begin state<=IDLE;end
                    else begin index<=index+1;state<=SEND;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule

`timescale 1ns/1ps
// FS01 + sequence(u32 LE) + WORDS*u32 LE + IEEE CRC32(sequence+words).
module d64_status_uart_ff_core #(
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
