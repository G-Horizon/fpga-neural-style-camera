`timescale 1ns/1ps
// Native 1:1 RGB888 scanout. Whole 128-bit beats fill two dual-clock line RAMs.
// Row N+1 is requested at the first active pixel of N. Frame0 is prefetched
// during vertical blanking. Slot changes happen only when the prior read is idle.
module camera_native_display #(
    parameter integer W=1920,H=1080,H_FP=88,H_SYNC=44,H_BP=148,V_FP=4,V_SYNC=5,V_BP=36,
    parameter [31:0] SLOT_BYTES=32'h00800000
)(
    input wire clk,rst_n,pix_clk,pix_rst_n,
    input wire latest_valid,input wire [1:0] latest_slot,
    input wire [31:0] latest_fid,output reg [31:0] frames_presented,
    output reg display_valid,output reg [1:0] display_slot,
    output reg fault_sticky,output reg underrun_sticky,
    output wire [31:0] araddr,output wire [7:0] arlen,
    output wire [2:0] arsize,output wire [1:0] arburst,
    output wire arvalid,input wire arready,input wire [127:0] rdata,
    input wire rvalid,output wire rready,input wire rlast,input wire [1:0] rresp,
    output wire [23:0] pixel_rgb,output wire pixel_de,pixel_hs,pixel_vs,
    output wire [11:0] pixel_x,pixel_y
);
    localparam integer ROW_WORDS=W/4,ADDR_W=$clog2(ROW_WORDS);
    wire hs,vs,de,frame;wire [11:0] x,y,ax,ay;
    d64_video_timing #(.H_ACT(W),.H_FP(H_FP),.H_SYNC(H_SYNC),.H_BP(H_BP),
        .V_ACT(H),.V_FP(V_FP),.V_SYNC(V_SYNC),.V_BP(V_BP)) u_raster(
        .clk(pix_clk),.rst_n(pix_rst_n),.hs(hs),.vs(vs),.de(de),.frame(frame),
        .x(x),.y(y),.ax(ax),.ay(ay));
    reg [31:0] epoch;
    reg [31:0] scan_fid,row_fid;
    always @(posedge pix_clk or negedge pix_rst_n)
        if(!pix_rst_n)epoch<=0;else if(frame)epoch<=epoch+1'b1;
    wire request_zero=x==0&&y==1;
    wire request_next=de&&ax==0&&ay<H-1;
    wire request=request_zero||request_next;
    wire [10:0] request_y=request_zero ? 11'd0 : ay[10:0]+1'b1;
    wire cmd_full,cmd_empty,cmd_valid;wire [42:0] cmd_data;
    reg request_overflow;
    always @(posedge pix_clk or negedge pix_rst_n)
        if(!pix_rst_n)request_overflow<=0;else if(request&&cmd_full)request_overflow<=1;
    localparam [2:0] IDLE=0,AR=1,R=2,DRAIN=3,COMMIT=4;
    reg [2:0] state;reg pending;
    wire cmd_read=state==IDLE&&!pending&&!cmd_empty;
    d64_async_fifo #(.DW(43),.AW(4)) u_requests(
        .wr_clk(pix_clk),.wr_rst_n(pix_rst_n),.wr_en(request&&!cmd_full),
        .wr_data({epoch,request_y}),.wr_full(cmd_full),.wr_almost_full(),.wr_level(),
        .rd_clk(clk),.rd_rst_n(rst_n),.rd_en(cmd_read),.rd_data(cmd_data),
        .rd_valid(cmd_valid),.rd_empty(cmd_empty),.rd_level());
    (* ram_style="block" *) reg [127:0] line0[0:ROW_WORDS-1];
    (* ram_style="block" *) reg [127:0] line1[0:ROW_WORDS-1];
    reg [127:0] pix_q0,pix_q1;
    reg [ADDR_W-1:0] word_index;
    reg [31:0] address,row_epoch,scan_epoch;
    reg [10:0] row_y;
    reg [8:0] remaining;
    reg [7:0] chunk,received;
    reg bad;
    wire [8:0] boundary=(13'd4096-{1'b0,address[11:0]})>>4;
    wire [8:0] max_chunk=remaining<64?remaining:9'd64;
    wire [7:0] safe_chunk=boundary<max_chunk ? boundary[7:0] : max_chunk[7:0];
    wire fire=rvalid&&rready;
    wire [1:0] selected_slot=latest_valid?latest_slot:display_slot;
    reg [1:0] meta_toggle,meta_valid;
    reg [31:0] meta_epoch[0:1],meta_fid[0:1];reg [10:0] meta_row[0:1];
    always @(posedge clk)begin
        if(state==R&&fire&&received<chunk)begin
            if(row_y[0])line1[word_index]<=rdata;else line0[word_index]<=rdata;
        end
    end
    always @(posedge pix_clk)begin
        pix_q0<=line0[ax[11:2]];pix_q1<=line1[ax[11:2]];
    end
    assign araddr=address;assign arlen=chunk-1'b1;
    assign arsize=4;assign arburst=1;assign arvalid=state==AR&&chunk!=0;
    assign rready=state==R||state==DRAIN;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;pending<=0;address<=0;row_epoch<=0;scan_epoch<=0;row_y<=0;
            remaining<=0;chunk<=0;received<=0;word_index<=0;bad<=0;
            display_valid<=0;display_slot<=0;fault_sticky<=0;scan_fid<=0;row_fid<=0;
            meta_toggle<=0;meta_valid<=0;meta_epoch[0]<=0;meta_epoch[1]<=0;
            meta_fid[0]<=0;meta_fid[1]<=0;
            meta_row[0]<=0;meta_row[1]<=0;
        end else begin
            if(cmd_read)pending<=1;
            case(state)
                IDLE:if(cmd_valid)begin
                    pending<=0;row_epoch<=cmd_data[42:11];row_y<=cmd_data[10:0];
                    remaining<=ROW_WORDS;word_index<=0;chunk<=0;received<=0;
                    if(cmd_data[10:0]==0)begin
                        scan_epoch<=cmd_data[42:11];
                        if(latest_valid)scan_fid<=latest_fid;
                        row_fid<=latest_valid?latest_fid:scan_fid;
                        display_valid<=latest_valid||display_valid;display_slot<=selected_slot;
                        address<=selected_slot*SLOT_BYTES;bad<=!(latest_valid||display_valid);
                        state<=(latest_valid||display_valid)?AR:COMMIT;
                    end else begin
                        row_fid<=scan_fid;
                        address<=display_slot*SLOT_BYTES+cmd_data[10:0]*(W*4);
                        bad<=!display_valid||cmd_data[42:11]!=scan_epoch||cmd_data[10:0]>=H;
                        state<=display_valid&&cmd_data[42:11]==scan_epoch&&cmd_data[10:0]<H?AR:COMMIT;
                    end
                end
                AR:begin
                    if(chunk==0)chunk<=safe_chunk;
                    else if(arready)begin received<=0;state<=R;end
                end
                R:if(fire)begin
                    if(rresp!=0)begin bad<=1;fault_sticky<=1;end
                    if(received<chunk)word_index<=word_index+1'b1;
                    received<=received+1'b1;
                    if(rlast)begin
                        if(received!=chunk-1'b1)begin bad<=1;fault_sticky<=1;state<=COMMIT;end
                        else if(remaining==chunk)state<=COMMIT;
                        else begin
                            remaining<=remaining-chunk;address<=address+({24'd0,chunk}<<4);
                            chunk<=0;state<=AR;
                        end
                    end else if(received==chunk-1'b1)begin
                        bad<=1;fault_sticky<=1;state<=DRAIN;
                    end
                end
                DRAIN:if(fire&&rlast)state<=COMMIT;
                COMMIT:begin
                    meta_epoch[row_y[0]]<=row_epoch;meta_row[row_y[0]]<=row_y;
                    meta_fid[row_y[0]]<=row_fid;
                    meta_valid[row_y[0]]<=!bad;
                    meta_toggle[row_y[0]]<=~meta_toggle[row_y[0]];state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
    (* async_reg="true" *) reg [1:0] toggle_s1,toggle_s2;
    reg [1:0] seen,valid_pix;
    reg [31:0] epoch_pix[0:1],fid_pix[0:1];reg [10:0] row_pix[0:1];integer b;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            toggle_s1<=0;toggle_s2<=0;seen<=0;valid_pix<=0;
            epoch_pix[0]<=0;epoch_pix[1]<=0;row_pix[0]<=0;row_pix[1]<=0;
            fid_pix[0]<=0;fid_pix[1]<=0;
        end else begin
            toggle_s1<=meta_toggle;toggle_s2<=toggle_s1;
            for(b=0;b<2;b=b+1)if(toggle_s2[b]!=seen[b])begin
                seen[b]<=toggle_s2[b];valid_pix[b]<=meta_valid[b];
                epoch_pix[b]<=meta_epoch[b];row_pix[b]<=meta_row[b];
                fid_pix[b]<=meta_fid[b];
            end
        end
    end
    reg de_d,hs_d,vs_d,bank_d,good_d;
    reg [1:0] lane_d;reg [11:0] x_d,y_d;
    reg [23:0] rgb_q;reg de_q,hs_q,vs_q;reg [11:0] x_q,y_q;
    reg scan_armed,good_q,present_frame_good,presented_once;
    reg [31:0] fid_d,fid_q,present_frame_fid,last_presented_fid;
    wire good=valid_pix[ay[0]]&&row_pix[ay[0]]==ay[10:0]&&epoch_pix[ay[0]]==epoch;
    wire [127:0] q=bank_d?pix_q1:pix_q0;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            de_d<=0;hs_d<=0;vs_d<=0;bank_d<=0;good_d<=0;lane_d<=0;x_d<=0;y_d<=0;
            rgb_q<=0;de_q<=0;hs_q<=0;vs_q<=0;x_q<=0;y_q<=0;underrun_sticky<=0;scan_armed<=0;
            good_q<=0;fid_d<=0;fid_q<=0;present_frame_good<=0;presented_once<=0;
            present_frame_fid<=0;last_presented_fid<=0;frames_presented<=0;
        end else begin
            de_d<=de;hs_d<=hs;vs_d<=vs;bank_d<=ay[0];lane_d<=ax[1:0];good_d<=good;
            fid_d<=fid_pix[ay[0]];fid_q<=fid_d;good_q<=good_d;
            x_d<=ax;y_d<=ay;de_q<=de_d;hs_q<=hs_d;vs_q<=vs_d;x_q<=x_d;y_q<=y_d;
            rgb_q<=de_d&&good_d ? (q>>(lane_d*32)) : 24'd0;
            if(de&&good&&ax==0&&ay==0)scan_armed<=1;
            if(scan_armed&&de&&!good)underrun_sticky<=1;
            if(request_overflow)underrun_sticky<=1;
            // Count a new camera frame only after every displayed active
            // pixel was valid and belonged to the same committed frame ID.
            // This counter belongs to pix_clk, not the DDR/core clock.
            if(de_q)begin
                if(x_q==0&&y_q==0)begin
                    present_frame_good<=good_q;present_frame_fid<=fid_q;
                end else if(!good_q||fid_q!=present_frame_fid)present_frame_good<=0;
                if(x_q==W-1&&y_q==H-1&&present_frame_good&&good_q&&
                   fid_q==present_frame_fid&&(!presented_once||fid_q!=last_presented_fid))begin
                    frames_presented<=frames_presented+1'b1;
                    last_presented_fid<=fid_q;presented_once<=1;
                end
            end
        end
    end
    assign pixel_rgb=rgb_q;assign pixel_de=de_q;assign pixel_hs=hs_q;assign pixel_vs=vs_q;
    assign pixel_x=x_q;assign pixel_y=y_q;
    // synthesis translate_off
    initial if(W%4||W<8||W>2048||H<2||H>2048||W*H*4>SLOT_BYTES)
        $fatal(1,"CAMERA_NATIVE_DISPLAY_GEOMETRY");
    // synthesis translate_on
endmodule
