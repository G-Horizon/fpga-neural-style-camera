`timescale 1ns/1ps
// Full native RGB888, four pixels/128-bit DDR word. Triple buffer ownership:
// a capture never writes the current scan buffer or the latest committed frame.
// A frame becomes visible only after its final successful BRESP.
module camera_frame_writer #(
    parameter integer W=1920,H=1080,FIFO_AW=11,
    parameter [31:0] SLOT_BYTES=32'h00800000
)(
    input wire camera_clk,camera_rst_n,rgb_valid,
    input wire [95:0] rgb4,input wire rgb_sof,rgb_eof,
    input wire [31:0] rgb_fid,input wire source_bad,
    input wire clk,rst_n,display_valid,input wire [1:0] display_slot,
    output reg latest_valid,output reg [1:0] latest_slot,
    output reg [31:0] latest_fid,output reg fault_sticky,
    output reg [31:0] frames_committed,
    output wire [31:0] awaddr,output wire [7:0] awlen,
    output wire [2:0] awsize,output wire [1:0] awburst,
    output wire awvalid,input wire awready,
    output wire [127:0] wdata,output wire [15:0] wstrb,
    output wire wlast,wvalid,input wire wready,
    input wire bvalid,output wire bready,input wire [1:0] bresp
);
    localparam integer FRAME_BEATS=W*H/4;
    wire [127:0] packed_rgb;
    genvar p;
    generate for(p=0;p<4;p=p+1)begin:g_pack
        assign packed_rgb[p*32+:32]={8'd0,rgb4[p*24+:24]};
    end endgenerate
    wire full,empty,rv;wire [161:0] rd;
    reg camera_fault;
    always @(posedge camera_clk or negedge camera_rst_n)
        if(!camera_rst_n)camera_fault<=0;
        else if(source_bad||(rgb_valid&&full))camera_fault<=1;
    (* async_reg="true" *) reg [1:0] bad_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)bad_sync<=0;else bad_sync<={bad_sync[0],camera_fault};
    localparam [2:0] SEEK=0,FILL=1,AW=2,LOAD=3,DATA=4,RESP=5;
    reg [2:0] state;
    reg [6:0] issued,received;
    reg seek_pending;
    wire re=state==SEEK ? (!empty&&!seek_pending) :
                         (state==FILL&&!empty&&issued<64);
    d64_async_fifo #(.DW(162),.AW(FIFO_AW)) u_rgb_queue(
        .wr_clk(camera_clk),.wr_rst_n(camera_rst_n),.wr_en(rgb_valid&&!full),
        .wr_data({rgb_fid,rgb_eof,rgb_sof,packed_rgb}),.wr_full(full),
        .wr_almost_full(),.wr_level(),.rd_clk(clk),.rd_rst_n(rst_n),
        .rd_en(re),.rd_data(rd),.rd_valid(rv),.rd_empty(empty),.rd_level());
    (* ram_style="block" *) reg [127:0] burst_ram[0:63];
    reg [127:0] data_q;
    reg [5:0] index;
    reg [1:0] capture_slot;
    reg [31:0] capture_fid,offset;
    reg [20:0] words;
    reg active,bad,ended;
    reg [1:0] free_slot;reg free_valid;integer s;
    always @* begin
        free_slot=0;free_valid=0;
        for(s=0;s<3;s=s+1)
            if((!display_valid||display_slot!=s)&&(!latest_valid||latest_slot!=s))begin
                free_valid=1;free_slot=s;
            end
    end
    wire fire=wvalid&&wready;
    wire [5:0] ram_address=state==LOAD ? 0 : (fire ? index+1'b1 : index);
    always @(posedge clk)begin
        if(rv&&((state==SEEK&&rd[128]&&free_valid)||(state==FILL)))
            burst_ram[state==SEEK ? 0 : received[5:0]]<=rd[127:0];
        if(state==LOAD||state==DATA)data_q<=burst_ram[ram_address];
    end
    assign awaddr=capture_slot*SLOT_BYTES+offset;
    assign awlen=63;assign awsize=4;assign awburst=1;assign awvalid=state==AW;
    assign wdata=data_q;assign wstrb=16'hffff;
    assign wlast=index==63;assign wvalid=state==DATA;assign bready=state==RESP;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=SEEK;issued<=0;received<=0;seek_pending<=0;index<=0;
            capture_slot<=0;capture_fid<=0;offset<=0;words<=0;active<=0;bad<=0;ended<=0;
            latest_valid<=0;latest_slot<=0;latest_fid<=0;fault_sticky<=0;frames_committed<=0;
        end else begin
            if(bad_sync[1])begin fault_sticky<=1;if(active)bad<=1;end
            if(state==SEEK)begin
                if(re)seek_pending<=1;
                if(rv)begin
                    seek_pending<=0;
                    if(rd[128]&&free_valid)begin
                        capture_slot<=free_slot;capture_fid<=rd[161:130];
                        words<=1;offset<=0;active<=1;bad<=bad_sync[1]||rd[129];ended<=0;
                        issued<=1;received<=1;state<=FILL;
                    end
                end
            end
            if(state==FILL)begin
                if(re)issued<=issued+1'b1;
                if(rv)begin
                    received<=received+1'b1;words<=words+1'b1;
                    if(rd[128]||rd[161:130]!=capture_fid||words>=FRAME_BEATS)begin bad<=1;fault_sticky<=1;end
                    if(rd[129])begin
                        ended<=1;
                        if(words!=FRAME_BEATS-1||received!=63)begin bad<=1;fault_sticky<=1;end
                    end
                    if(received==63)state<=AW;
                end
            end
            if(state==AW&&awready)begin index<=0;state<=LOAD;end
            if(state==LOAD)state<=DATA;
            if(state==DATA&&fire)begin
                if(index==63)state<=RESP;else index<=index+1'b1;
            end
            if(state==RESP&&bvalid)begin
                if(bresp!=0)begin bad<=1;fault_sticky<=1;end
                if(ended||words>=FRAME_BEATS)begin
                    if(ended&&words==FRAME_BEATS&&!bad&&!bad_sync[1]&&bresp==0)begin
                        latest_slot<=capture_slot;latest_fid<=capture_fid;latest_valid<=1;
                        frames_committed<=frames_committed+1'b1;
                    end
                    active<=0;state<=SEEK;
                end else begin
                    offset<=offset+1024;issued<=0;received<=0;state<=FILL;
                end
            end
        end
    end
    // synthesis translate_off
    initial if(W%4||FRAME_BEATS%64||W*H*4>SLOT_BYTES||FRAME_BEATS>2097151)
        $fatal(1,"CAMERA_WRITER_GEOMETRY");
    // synthesis translate_on
endmodule
