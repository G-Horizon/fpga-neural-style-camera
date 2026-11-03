`timescale 1ns/1ps
// Two core-clock burst banks. Filling the next bank overlaps one AXI write.
// A single capture context remains reserved through the final BRESP.
module camera_frame_writer_pingpong128 #(
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
    localparam integer FRAME_BURSTS=FRAME_BEATS/128;
    wire [127:0] packed_rgb;
    genvar p;
    generate for(p=0;p<4;p=p+1)begin:g_pack
        assign packed_rgb[p*32+:32]={8'd0,rgb4[p*24+:24]};
    end endgenerate
    wire full,empty,rv;
    wire [161:0] rd;
    reg camera_fault;
    always @(posedge camera_clk or negedge camera_rst_n)
        if(!camera_rst_n)camera_fault<=0;
        else if(source_bad||(rgb_valid&&full))camera_fault<=1;
    (* async_reg="true" *) reg [1:0] bad_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)bad_sync<=0;else bad_sync<={bad_sync[0],camera_fault};

    localparam [2:0] F_SEEK=0,F_RUN=1,F_WAIT_BANK=2,F_WAIT_FRAME=3,F_HALT=4;
    localparam [2:0] A_IDLE=0,A_AW=1,A_LOAD=2,A_DATA=3,A_RESP=4;
    reg [2:0] fill_state,axi_state;
    reg seek_pending,fill_bank,send_bank;
    reg [7:0] issued,received;
    reg [1:0] bank_full,bank_end;
    reg [31:0] bank_addr[0:1];
    reg [20:0] bank_words[0:1];
    reg [31:0] bank_fid[0:1];
    reg [31:0] capture_fid,next_offset;
    reg [1:0] capture_slot;
    reg [20:0] words,retired_bursts;
    reg active,ended;
    reg [6:0] index;
    reg [127:0] data_q;
    (* ram_style="block" *) reg [127:0] burst_ram[0:255];
    reg [1:0] free_slot;
    reg free_valid;
    integer s;
    always @* begin
        free_slot=0;free_valid=0;
        for(s=0;s<3;s=s+1)
            if((!display_valid||display_slot!=s)&&(!latest_valid||latest_slot!=s))begin
                free_valid=1;free_slot=s;
            end
    end
    wire re=!fault_sticky&&!bad_sync[1]&&!empty&&
            ((fill_state==F_SEEK&&!seek_pending)||
             (fill_state==F_RUN&&issued<128));
    d64_async_fifo #(.DW(162),.AW(FIFO_AW)) u_rgb_queue(
        .wr_clk(camera_clk),.wr_rst_n(camera_rst_n),.wr_en(rgb_valid&&!full),
        .wr_data({rgb_fid,rgb_eof,rgb_sof,packed_rgb}),.wr_full(full),
        .wr_almost_full(),.wr_level(),.rd_clk(clk),.rd_rst_n(rst_n),
        .rd_en(re),.rd_data(rd),.rd_valid(rv),.rd_empty(empty),.rd_level());

    wire seed=rv&&fill_state==F_SEEK&&rd[128]&&free_valid&&!fault_sticky&&!bad_sync[1];
    wire fill_write=rv&&fill_state==F_RUN;
    wire fire=wvalid&&wready;
    wire [6:0] ram_address=axi_state==A_LOAD ? 7'd0 :
                                   (fire ? index+1'b1 : index);
    // One physical simple-dual-port RAM, two independently owned logical banks.
    // Keep the registered read directly on the RAM, without a mux across memories.
    wire burst_write=seed||fill_write;
    wire [7:0] burst_write_address={fill_bank,(seed ? 7'd0 : received[6:0])};
    wire [7:0] burst_read_address={send_bank,ram_address};
    always @(posedge clk)begin
        if(burst_write)burst_ram[burst_write_address]<=rd[127:0];
    end
    always @(posedge clk)begin
        if(axi_state==A_LOAD||axi_state==A_DATA)
            data_q<=burst_ram[burst_read_address];
    end
    assign awaddr=bank_addr[send_bank];
    assign awlen=127;assign awsize=4;assign awburst=1;
    assign awvalid=axi_state==A_AW;
    assign wdata=data_q;assign wstrb=16'hffff;
    assign wlast=index==127;assign wvalid=axi_state==A_DATA;
    assign bready=axi_state==A_RESP;

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            fill_state<=F_SEEK;axi_state<=A_IDLE;
            seek_pending<=0;fill_bank<=0;send_bank<=0;
            issued<=0;received<=0;bank_full<=0;bank_end<=0;
            bank_addr[0]<=0;bank_addr[1]<=0;
            bank_words[0]<=0;bank_words[1]<=0;bank_fid[0]<=0;bank_fid[1]<=0;
            capture_slot<=0;capture_fid<=0;next_offset<=0;
            words<=0;retired_bursts<=0;active<=0;ended<=0;index<=0;
            latest_valid<=0;latest_slot<=0;latest_fid<=0;
            fault_sticky<=0;frames_committed<=0;
        end else begin
            if(bad_sync[1])begin fault_sticky<=1;fill_state<=F_HALT;end
            if(fill_state==F_SEEK)begin
                if(re)seek_pending<=1;
                if(rv)seek_pending<=0;
                if(seed)begin
                    capture_slot<=free_slot;capture_fid<=rd[161:130];
                    words<=1;retired_bursts<=0;active<=1;ended<=0;
                    next_offset<=0;issued<=1;received<=1;
                    fill_state<=F_RUN;
                    // No supported frame fits in one word. Reject malformed SOF/EOF.
                    if(rd[129])begin fault_sticky<=1;fill_state<=F_HALT;end
                end
            end
            if(fill_state==F_WAIT_BANK&&!bank_full[fill_bank]&&!fault_sticky&&!bad_sync[1])begin
                issued<=0;received<=0;fill_state<=F_RUN;
            end
            if(fill_state==F_RUN)begin
                if(re)issued<=issued+1'b1;
                if(rv)begin
                    received<=received+1'b1;words<=words+1'b1;
                    if(rd[128]||rd[161:130]!=capture_fid||words>=FRAME_BEATS||
                       (rd[129]!=(words==FRAME_BEATS-1)))begin
                        fault_sticky<=1;fill_state<=F_HALT;
                    end else if(received==127)begin
                        bank_addr[fill_bank]<=capture_slot*SLOT_BYTES+next_offset;
                        bank_words[fill_bank]<=words+1'b1;
                        bank_fid[fill_bank]<=capture_fid;
                        bank_end[fill_bank]<=rd[129];
                        bank_full[fill_bank]<=1;
                        next_offset<=next_offset+2048;
                        fill_bank<=~fill_bank;
                        issued<=0;received<=0;
                        if(rd[129])begin ended<=1;fill_state<=F_WAIT_FRAME;end
                        else fill_state<=F_WAIT_BANK;
                    end
                end
            end
            // An already presented AXI transaction always finishes, even after a fault.
            if(axi_state==A_IDLE&&bank_full[send_bank])axi_state<=A_AW;
            if(axi_state==A_AW&&awready)begin index<=0;axi_state<=A_LOAD;end
            if(axi_state==A_LOAD)axi_state<=A_DATA;
            if(axi_state==A_DATA&&fire)begin
                if(index==127)axi_state<=A_RESP;else index<=index+1'b1;
            end
            if(axi_state==A_RESP&&bvalid)begin
                bank_full[send_bank]<=0;send_bank<=~send_bank;
                retired_bursts<=retired_bursts+1'b1;
                axi_state<=A_IDLE;
                if(bresp!=0)begin fault_sticky<=1;fill_state<=F_HALT;end
                if(bank_end[send_bank])begin
                    if(ended&&words==FRAME_BEATS&&bank_words[send_bank]==FRAME_BEATS&&
                       retired_bursts==FRAME_BURSTS-1&&bank_fid[send_bank]==capture_fid&&
                       !fault_sticky&&!bad_sync[1]&&bresp==0&&fill_state==F_WAIT_FRAME)begin
                        latest_slot<=capture_slot;latest_fid<=capture_fid;latest_valid<=1;
                        frames_committed<=frames_committed+1'b1;
                        active<=0;ended<=0;fill_state<=F_SEEK;
                    end else begin
                        active<=0;fault_sticky<=1;fill_state<=F_HALT;
                    end
                end
            end
            // Later state transitions above must never cancel a synchronized source fault.
            if(bad_sync[1])fill_state<=F_HALT;
        end
    end
    // synthesis translate_off
    initial if(W<=0||H<=0||W%4||FRAME_BEATS%128||W*H*4>SLOT_BYTES||FRAME_BEATS>2097151||FIFO_AW<2)
        $fatal(1,"CAMERA_WRITER_GEOMETRY");
    // synthesis translate_on
endmodule
