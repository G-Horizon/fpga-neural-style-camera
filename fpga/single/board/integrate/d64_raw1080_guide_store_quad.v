`timescale 1ns/1ps
// Full-frame scalar G8 capture, four bytes packed per32bit FIFO word.
// Same2048B reservoir uses512x32, two RAM10, not128wide seven-RAM FIFO.
// Read four quads in four clocks and assemble AXI128 with complete-burst credit.
module d64_raw1080_guide_store_quad #(
    parameter integer W=1920,H=1080,
    parameter [31:0] BASE=32'h02000000,SLOT_BYTES=32'h00200000,
    parameter integer FIFO_AW=9,BURST_BEATS=16,SOURCE_COMPLETE=1
) (
    input wire clk,input wire rst_n,
    input wire in_valid,input wire [7:0] in_g,
    input wire [10:0] in_x,input wire [10:0] in_y,
    input wire [31:0] in_fid,input wire frame_bad,
    input wire frame_slot_valid,input wire [2:0] frame_slot,
    output wire capture_started,
    output reg capturing,output reg [2:0] active_slot,
    output reg [31:0] active_fid,
    output reg committed,output reg failed,
    output reg [2:0] committed_slot,output reg [31:0] committed_fid,
    output reg overflow_fault,output reg protocol_fault,output reg axi_fault,
    output wire [31:0] awaddr,output wire [7:0] awlen,
    output wire [2:0] awsize,output wire [1:0] awburst,
    output wire awvalid,input wire awready,
    output wire [127:0] wdata,output wire [15:0] wstrb,
    output wire wlast,output wire wvalid,input wire wready,
    input wire bvalid,output wire bready,input wire [1:0] bresp
);
    localparam integer DEPTH=1<<FIFO_AW,FRAME_BEATS=W*H/16;
    localparam [2:0] IDLE=0,AW=1,RSTART=2,FILL=3,WDATA=4,BRESP=5;
    reg [2:0] state;
    (* ram_style="block" *) reg [31:0] fifo [0:DEPTH-1];
    reg [31:0] read_q;
    reg [23:0] pack;
    reg [1:0] pack_lane,fill_index;
    reg [127:0] beat;
    reg [FIFO_AW-1:0] wp,rp;
    reg [FIFO_AW:0] count;
    reg [10:0] expected_x,expected_y;
    reg [31:0] offset;
    reg [21:0] accepted_pixels;
    reg [17:0] written_beats;
    reg [7:0] burst_count,beat_index;
    reg ended,bad;
    wire first=in_valid&&in_x==0&&in_y==0;
    wire start=first&&!capturing&&state==IDLE&&count==0&&frame_slot_valid;
    assign capture_started=start;
    wire sample=in_valid&&((capturing&&!ended)||start);
    wire row_end=in_x==W-1;
    wire final_sample=row_end&&in_y==H-1;
    wire read_quad=state==RSTART||(state==FILL&&fill_index!=3);
    wire quad_ready=sample&&pack_lane==3;
    wire can_push=count<DEPTH||read_quad;
    wire push=quad_ready&&can_push;
    wire fire_w=wvalid&&wready;
    assign awaddr=BASE+(active_slot*SLOT_BYTES)+offset;
    assign awlen=burst_count-1'b1;assign awsize=4;assign awburst=1;
    assign awvalid=state==AW;
    assign wdata=beat;assign wstrb=16'hffff;
    assign wlast=beat_index==burst_count-1'b1;
    assign wvalid=state==WDATA;assign bready=state==BRESP;
    always @(posedge clk)begin
        if(push)fifo[wp]<={in_g,pack};
        if(read_quad)read_q<=fifo[rp];
    end
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1)begin:g_assemble
        always @(posedge clk)if(state==FILL&&fill_index==lane)beat[lane*32+:32]<=read_q;
    end endgenerate
    // synthesis translate_off
    initial begin
        if(W%16!=0||W<16||H<1||W>2048||H>2048||FIFO_AW<3||
           BURST_BEATS<1||BURST_BEATS>128||BURST_BEATS*4>DEPTH||SOURCE_COMPLETE!=1)
            $fatal(1,"GUIDE_QUAD_STORE_INVALID_GEOMETRY_OR_FIFO");
        if(W*H>SLOT_BYTES)$fatal(1,"GUIDE_QUAD_STORE_SLOT_TOO_SMALL");
    end
    // synthesis translate_on
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;wp<=0;rp<=0;count<=0;fill_index<=0;pack<=0;pack_lane<=0;
            capturing<=0;active_slot<=0;active_fid<=0;
            committed<=0;failed<=0;committed_slot<=0;committed_fid<=0;
            expected_x<=0;expected_y<=0;offset<=0;
            accepted_pixels<=0;written_beats<=0;burst_count<=0;beat_index<=0;
            ended<=0;bad<=0;overflow_fault<=0;protocol_fault<=0;axi_fault<=0;
        end else begin
            committed<=0;failed<=0;
            count<=count+(push?1'b1:1'b0)-(read_quad?1'b1:1'b0);
            if(push)wp<=wp+1'b1;
            if(read_quad)rp<=rp+1'b1;
            if(frame_bad&&capturing)bad<=1;
            if(first&&!capturing&&state==IDLE&&count==0&&!frame_slot_valid)begin
                failed<=1;committed_slot<=frame_slot;committed_fid<=in_fid;
            end
            if(first&&capturing&&!ended)begin bad<=1;protocol_fault<=1;ended<=1;end
            if(quad_ready&&!can_push)begin bad<=1;overflow_fault<=1;end
            if(start)begin
                capturing<=1;active_slot<=frame_slot;active_fid<=in_fid;
                expected_x<=1;expected_y<=0;accepted_pixels<=1;
                written_beats<=0;offset<=0;ended<=0;bad<=frame_bad;
                pack<={in_g,16'd0};pack_lane<=1;
            end else if(sample)begin
                accepted_pixels<=accepted_pixels+1'b1;
                if(in_x!=expected_x||in_y!=expected_y||in_fid!=active_fid)begin
                    bad<=1;protocol_fault<=1;
                end
                pack<={in_g,pack[23:8]};pack_lane<=pack_lane+2'd1;
                if(row_end)begin expected_x<=0;expected_y<=expected_y+1'b1;end
                else expected_x<=expected_x+1'b1;
                if(final_sample)ended<=1;
            end
            case(state)
                IDLE:begin
                    if(capturing&&(count>=BURST_BEATS*4||ended&&count>=4))begin
                        burst_count<=count>=BURST_BEATS*4 ? BURST_BEATS : count>>2;
                        beat_index<=0;state<=AW;
                    end else if(capturing&&ended&&count<4)begin
                        capturing<=0;ended<=0;count<=0;rp<=wp;pack_lane<=0;pack<=0;
                        committed_slot<=active_slot;committed_fid<=active_fid;
                        if(!bad&&!frame_bad&&count==0&&pack_lane==0&&accepted_pixels==W*H&&
                           written_beats==FRAME_BEATS)committed<=1;
                        else failed<=1;
                    end
                end
                AW:if(awready)state<=RSTART;
                RSTART:begin fill_index<=0;state<=FILL;end
                FILL:begin
                    if(fill_index==3)state<=WDATA;
                    else fill_index<=fill_index+2'd1;
                end
                WDATA:if(fire_w)begin
                    written_beats<=written_beats+1'b1;
                    if(wlast)state<=BRESP;
                    else begin beat_index<=beat_index+1'b1;state<=RSTART;end
                end
                BRESP:if(bvalid)begin
                    if(bresp!=0)begin bad<=1;axi_fault<=1;end
                    offset<=offset+({24'd0,burst_count}<<4);state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
