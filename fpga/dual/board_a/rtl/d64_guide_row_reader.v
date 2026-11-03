`timescale 1ns/1ps
// Synchronous128x64 response reservoir, with explicit final8bit physical RAM.
// Only burst storage changed: same receive/drain/cache/FID/edge policy.
// Root owns bank leases: never fill a bank currently scanned by pixel consumer.
// Only a complete good row is made valid. Invalid/FID mismatch fails closed.
module d64_guide_row_reader #(
    parameter integer W=1920,H=1080,STORED_H=H-1,CACHE_PIXELS=4,
    parameter [31:0] BASE=32'h02000000,SLOT_BYTES=32'h00200000
) (
    input wire clk,input wire rst_n,
    input wire row_req,input wire row_bank,input wire [2:0] row_slot,
    input wire [10:0] row_y,input wire [31:0] row_fid,
    output wire row_ready,output reg row_done,output reg row_failed,
    output reg axi_fault,output reg protocol_fault,
    output wire [31:0] araddr,output wire [7:0] arlen,
    output wire [2:0] arsize,output wire [1:0] arburst,
    output wire arvalid,input wire arready,
    input wire [127:0] rdata,input wire rvalid,output wire rready,
    input wire rlast,input wire [1:0] rresp,
    input wire pix_clk,input wire pix_rst_n,input wire pix_valid,
    input wire pix_bank,input wire [10:0] pix_x,pix_y,
    input wire [31:0] pix_fid,
    output wire guide_valid,output wire [7:0] guide_g,
    output wire [31:0] guide_fid,output wire [10:0] guide_y
);
    localparam [1:0] IDLE=0,AR=1,R=2,DRAIN=3;
    localparam integer CACHE_BITS=CACHE_PIXELS*8,WORDS_PER_BEAT=16/CACHE_PIXELS;
    localparam integer CACHE_ADDR_W=CACHE_PIXELS==4?9:10;
    (* ram_style="block" *) reg [CACHE_BITS-1:0] bank0 [0:W/CACHE_PIXELS-1];
    (* ram_style="block" *) reg [CACHE_BITS-1:0] bank1 [0:W/CACHE_PIXELS-1];
    reg [1:0] state;
    reg bank;
    reg [31:0] address,fid;
    reg [10:0] y;
    reg [7:0] remaining,chunk,received;
    reg [CACHE_ADDR_W-1:0] pair_address;
    reg bad,burst_done;
    reg [7:0] consumed;
    reg [1:0] unpack_part;
    reg [1:0] meta_toggle,meta_valid;
    reg [31:0] meta_fid0,meta_fid1;
    reg [10:0] meta_y0,meta_y1;
    wire [8:0] to_boundary=(13'd4096-{1'b0,address[11:0]})>>4;
    wire [8:0] next_chunk=remaining<64 ? remaining : 9'd64;
    wire [7:0] safe_chunk=to_boundary<next_chunk ? to_boundary[7:0] : next_chunk[7:0];
    wire [10:0] clamped_y=row_y>=STORED_H ? STORED_H-1 : row_y;
    wire [31:0] extended_y={21'd0,clamped_y};
    //1920 =2048-128. Default path does not consume a DSP for row address.
    wire [31:0] row_offset=W==1920 ? (extended_y<<11)-(extended_y<<7) : extended_y*W;
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
            if(bank)bank1[pair_address]<=incoming_pair;
            else bank0[pair_address]<=incoming_pair;
        end
    end
    // synthesis translate_off
    initial if(W%16!=0||W<16||W>2048||H<2||H>2048||STORED_H<1||STORED_H>H||
               (CACHE_PIXELS!=4))
        $fatal(1,"GUIDE_READER_GEOMETRY");
    // synthesis translate_on
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;bank<=0;address<=0;fid<=0;y<=0;
            remaining<=0;chunk<=0;received<=0;pair_address<=0;
            bad<=0;burst_done<=0;consumed<=0;unpack_part<=0;
            row_done<=0;row_failed<=0;axi_fault<=0;protocol_fault<=0;
            meta_toggle<=0;meta_valid<=0;meta_fid0<=0;meta_fid1<=0;meta_y0<=0;meta_y1<=0;
        end else begin
            row_done<=0;row_failed<=0;
            case(state)
                IDLE:if(row_req)begin
                    bank<=row_bank;fid<=row_fid;y<=row_y;
                    address<=BASE+row_slot*SLOT_BYTES+row_offset;
                    remaining<=W/16;pair_address<=0;bad<=row_y>=H;
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
                            if(bank)begin meta_fid1<=fid;meta_y1<=y;end
                            else begin meta_fid0<=fid;meta_y0<=y;end
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
    reg [CACHE_BITS-1:0] read0,read1;
    wire [CACHE_ADDR_W-1:0] pixel_word_address=CACHE_PIXELS==4?pix_x[10:2]:pix_x[10:1];
    always @(posedge pix_clk)begin
        read0<=bank0[pixel_word_address];read1<=bank1[pixel_word_address];
    end
    (* async_reg="true" *) reg [1:0] sync1,sync2;
    reg [1:0] seen,pvalid;
    reg [31:0] pfid0,pfid1;
    reg [10:0] py0,py1;
    reg select_bank,request_valid;
    reg [1:0] select_byte;
    reg [31:0] request_fid;
    reg [10:0] request_y;
    wire [CACHE_BITS-1:0] selected=select_bank ? read1 : read0;
    assign guide_valid=request_valid;
    assign guide_g=selected[select_byte*8+:8];
    assign guide_fid=request_fid;assign guide_y=request_y;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            sync1<=0;sync2<=0;seen<=0;pvalid<=0;
            pfid0<=0;pfid1<=0;py0<=0;py1<=0;
            select_bank<=0;select_byte<=0;request_valid<=0;request_fid<=0;request_y<=0;
        end else begin
            sync1<=meta_toggle;sync2<=sync1;
            if(sync2[0]!=seen[0])begin
                seen[0]<=sync2[0];pvalid[0]<=meta_valid[0];pfid0<=meta_fid0;py0<=meta_y0;
            end
            if(sync2[1]!=seen[1])begin
                seen[1]<=sync2[1];pvalid[1]<=meta_valid[1];pfid1<=meta_fid1;py1<=meta_y1;
            end
            request_valid<=pix_valid&&pix_x<W&&pix_y<H&&
                (pix_bank?pvalid[1]&&pfid1==pix_fid&&py1==pix_y:
                          pvalid[0]&&pfid0==pix_fid&&py0==pix_y);
            request_fid<=pix_fid;request_y<=pix_y;
            select_bank<=pix_bank;select_byte<=CACHE_PIXELS==4?pix_x[1:0]:{1'b0,pix_x[0]};
        end
    end
endmodule
