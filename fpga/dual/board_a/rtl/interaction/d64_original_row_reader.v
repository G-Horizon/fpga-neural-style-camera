`timescale 1ns/1ps
// Synchronous128x64 response reservoir. Receive full bursts without
// backpressure, then unpack registered RAM reads. The final8bit RAM is
// explicit to prevent the1024-bit small-memory threshold from mapping it
// into512 FF; this reservoir maps to7 RAM10 at Titanium20-bit width.
// Four caches: bank={rowParity, plane}. Full-burst RREADY workaround retained.
// row_base is absolute and held; root owns frame/slot leases.
// Only a complete good row is made valid. Invalid/FID mismatch fails closed.
module d64_original_row_reader #(
    parameter integer W=960,H=540,STORED_H=H,CACHE_PIXELS=2
) (
    input wire clk,input wire rst_n,
    input wire row_req,input wire [1:0] row_bank,input wire [31:0] row_base,
    input wire [10:0] row_y,input wire [31:0] row_fid,
    output wire row_ready,output reg row_done,output reg row_failed,
    output reg axi_fault,output reg protocol_fault,
    output wire [31:0] araddr,output wire [7:0] arlen,
    output wire [2:0] arsize,output wire [1:0] arburst,
    output wire arvalid,input wire arready,
    input wire [127:0] rdata,input wire rvalid,output wire rready,
    input wire rlast,input wire [1:0] rresp,
    input wire pix_clk,input wire pix_rst_n,input wire pix_valid,
    input wire [1:0] pix_bank,input wire [10:0] pix_x,pix_y,
    input wire [31:0] pix_fid,
    output wire pixel_valid,output wire [15:0] pixel_rgb565,
    output wire [31:0] pixel_fid,output wire [10:0] pixel_y
);
    localparam [1:0] IDLE=0,AR=1,R=2,DRAIN=3;
    localparam integer CACHE_BITS=CACHE_PIXELS*16,WORDS_PER_BEAT=8/CACHE_PIXELS;
    localparam integer CACHE_ADDR_W=9;
    (* ram_style="block" *) reg [CACHE_BITS-1:0] bank0 [0:W/CACHE_PIXELS-1];
    (* ram_style="block" *) reg [CACHE_BITS-1:0] bank1 [0:W/CACHE_PIXELS-1];
    (* ram_style="block" *) reg [CACHE_BITS-1:0] bank2 [0:W/CACHE_PIXELS-1];
    (* ram_style="block" *) reg [CACHE_BITS-1:0] bank3 [0:W/CACHE_PIXELS-1];
    reg [1:0] state;
    reg [1:0] bank;
    reg [31:0] address,fid;
    reg [10:0] y;
    reg [7:0] remaining,chunk,received;
    reg [CACHE_ADDR_W-1:0] pair_address;
    reg bad,burst_done;
    reg [7:0] consumed;
    reg [1:0] unpack_part;
    reg [3:0] meta_toggle,meta_valid;
    reg [31:0] meta_fid[0:3];
    reg [10:0] meta_y[0:3];integer ai,pi;
    wire [8:0] to_boundary=(13'd4096-{1'b0,address[11:0]})>>4;
    wire [8:0] next_chunk=remaining<64 ? remaining : 9'd64;
    wire [7:0] safe_chunk=to_boundary<next_chunk ? to_boundary[7:0] : next_chunk[7:0];
    wire [10:0] clamped_y=row_y>=STORED_H ? STORED_H-1 : row_y;
    wire [31:0] extended_y={21'd0,clamped_y};
    // RGB565 row bytes: 960*2 = 2048-128; no DSP for the default address.
    wire [31:0] row_offset=W==960 ? (extended_y<<11)-(extended_y<<7) : extended_y*(W*2);
    wire fire=rvalid&&rready;
    // Complete AXI bursts are received without backpressure, then unpacked.
    // The128-bit synchronous reservoir is written and read in separate phases.
    // First read waits one clock after LAST. On subword3 prefetch the next beat,
    // retaining four cache-word clocks per beat after that initial latency.
    wire receiving=state==R&&!burst_done;
    wire response_write=receiving&&fire&&received<chunk;
    wire response_read=state==R&&burst_done&&!bad&&received==chunk&&consumed<received;
    reg response_valid;
    (* ram_style="block" *) reg [119:0] response_ram[0:63];
    reg [119:0] response_word_lo;wire [127:0] response_word;
    assign response_word[119:0]=response_word_lo;
    wire unpack_valid=response_read&&response_valid;
    wire [5:0] response_read_address=(unpack_valid&&unpack_part==3)?consumed+1'b1:consumed;
    always @(posedge clk)begin
        if(response_write)response_ram[received[5:0]]<=rdata[119:0];
        else if(response_read)response_word_lo<=response_ram[response_read_address];
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)response_valid<=0;
        else response_valid<=response_read;
    end
    EFX_RAM10 #(.WRITE_WIDTH(8),.READ_WIDTH(8),.OUTPUT_REG(1'b0),.WRITE_MODE("READ_FIRST")) u_response_tail(
        .WCLK(clk),.WCLKE(1'b1),.WADDREN(1'b1),.WE({2{response_write}}),
        .WDATA(rdata[127:120]),.WADDR({4'd0,received[5:0]}),
        .RCLK(clk),.RE(response_read),.RST(1'b0),.RADDREN(1'b1),
        .RADDR({4'd0,response_read_address}),.RDATA(response_word[127:120]));
    wire [31:0] incoming_pair=unpack_part==0 ? response_word[31:0] :
                              unpack_part==1 ? response_word[63:32] :
                              unpack_part==2 ? response_word[95:64] : response_word[127:96];
    assign row_ready=state==IDLE;
    assign araddr=address;assign arlen=chunk-1'b1;
    assign arsize=4;assign arburst=1;assign arvalid=state==AR&&chunk!=0;
    assign rready=state==DRAIN||receiving;
    always @(posedge clk)begin
        if(unpack_valid)begin
            case(bank)
                0:bank0[pair_address]<=incoming_pair;1:bank1[pair_address]<=incoming_pair;
                2:bank2[pair_address]<=incoming_pair;3:bank3[pair_address]<=incoming_pair;
            endcase
        end
    end
    // synthesis translate_off
    initial if(W%8!=0||W<8||W>1024||H<2||H>2048||STORED_H<1||STORED_H>H||
               (CACHE_PIXELS!=2))
        $fatal(1,"ORIGINAL_READER_GEOMETRY");
    // synthesis translate_on
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;bank<=0;address<=0;fid<=0;y<=0;
            remaining<=0;chunk<=0;received<=0;pair_address<=0;
            bad<=0;burst_done<=0;consumed<=0;unpack_part<=0;
            row_done<=0;row_failed<=0;axi_fault<=0;protocol_fault<=0;
            meta_toggle<=0;meta_valid<=0;
            for(ai=0;ai<4;ai=ai+1)begin meta_fid[ai]<=0;meta_y[ai]<=0;end
        end else begin
            row_done<=0;row_failed<=0;
            case(state)
                IDLE:if(row_req)begin
                    bank<=row_bank;fid<=row_fid;y<=row_y;
                    address<=row_base+row_offset;
                    remaining<=W/8;pair_address<=0;bad<=row_y>=H;
                    burst_done<=0;received<=0;consumed<=0;unpack_part<=0;
                    meta_valid[row_bank]<=0;
                    meta_toggle[row_bank]<=~meta_toggle[row_bank];
                    // One cycle prepares address; chunk computed in AR.
                    chunk<=0;state<=AR;
                end
                AR:begin
                    if(chunk==0)chunk<=safe_chunk;
                    else if(arready)begin received<=0;consumed<=0;unpack_part<=0;burst_done<=0;state<=R;end
                end
                R:begin
                    if(unpack_valid)begin
                        pair_address<=pair_address+1'b1;
                        if(unpack_part==3)begin consumed<=consumed+1'b1;unpack_part<=0;end
                        else unpack_part<=unpack_part+1'b1;
                    end
                    if(fire)begin
                        received<=received+1'b1;
                        if(rresp!=0)begin bad<=1;axi_fault<=1;end
                        if(rlast!=(received==chunk-1'b1))begin
                            bad<=1;protocol_fault<=1;
                            if(!rlast&&received==chunk-1'b1)state<=DRAIN;
                        end
                        if(rlast)burst_done<=1;
                    end
                    if(burst_done&&(bad||received!=chunk||consumed==received))begin
                        if(bad||received!=chunk)begin
                            row_failed<=1;meta_valid[bank]<=0;
                            meta_toggle[bank]<=~meta_toggle[bank];state<=IDLE;
                        end else if(remaining==chunk)begin
                            row_done<=1;meta_valid[bank]<=1;
                            meta_fid[bank]<=fid;meta_y[bank]<=y;
                            meta_toggle[bank]<=~meta_toggle[bank];state<=IDLE;
                        end else begin
                            remaining<=remaining-chunk;address<=address+({24'd0,chunk}<<4);
                            chunk<=0;received<=0;consumed<=0;unpack_part<=0;burst_done<=0;state<=AR;
                        end
                    end
                end
                DRAIN:if(fire&&rlast)begin
                    row_failed<=1;state<=IDLE;
                    meta_toggle[bank]<=~meta_toggle[bank];
                end
            endcase
        end
    end
    reg [CACHE_BITS-1:0] read0,read1,read2,read3;
    wire [CACHE_ADDR_W-1:0] pixel_word_address=pix_x[9:1];
    always @(posedge pix_clk)begin
        read0<=bank0[pixel_word_address];read1<=bank1[pixel_word_address];
        read2<=bank2[pixel_word_address];read3<=bank3[pixel_word_address];
    end
    (* async_reg="true" *) reg [3:0] sync1,sync2;
    reg [3:0] seen,pvalid;
    reg [31:0] pfid[0:3];
    reg [10:0] py[0:3];
    reg [1:0] select_bank;reg request_valid;
    reg select_half;
    reg [31:0] request_fid;
    reg [10:0] request_y;
    wire [CACHE_BITS-1:0] selected=select_bank==0?read0:select_bank==1?read1:select_bank==2?read2:read3;
    assign pixel_valid=request_valid;
    assign pixel_rgb565=selected[select_half*16+:16];
    assign pixel_fid=request_fid;assign pixel_y=request_y;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            sync1<=0;sync2<=0;seen<=0;pvalid<=0;
            for(pi=0;pi<4;pi=pi+1)begin pfid[pi]<=0;py[pi]<=0;end
            select_bank<=0;select_half<=0;request_valid<=0;request_fid<=0;request_y<=0;
        end else begin
            sync1<=meta_toggle;sync2<=sync1;
            for(pi=0;pi<4;pi=pi+1)if(sync2[pi]!=seen[pi])begin
                seen[pi]<=sync2[pi];pvalid[pi]<=meta_valid[pi];pfid[pi]<=meta_fid[pi];py[pi]<=meta_y[pi];
            end
            request_valid<=pix_valid&&pix_x<W&&pix_y<H&&
                pvalid[pix_bank]&&pfid[pix_bank]==pix_fid&&py[pix_bank]==pix_y;
            request_fid<=pix_fid;request_y<=pix_y;
            select_bank<=pix_bank;select_half<=pix_x[0];
        end
    end
endmodule
