`timescale 1ns/1ps
// Native 1:1 RGB888 scanout with four padded dual-clock line banks.
// Blank rows y=1,2,3 request camera rows 0,1,2. Active row N requests N+3.
// Row zero alone selects the frame slot/FID. All metadata qualifications remain strict.
module camera_native_display_prefetch4 #(
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
    output wire [11:0] pixel_x,pixel_y,
    output wire [511:0] pixel_diag_live,
    output wire [1023:0] core_diag_live
);
    localparam integer ROW_WORDS=W/4,ADDR_W=$clog2(ROW_WORDS),BANK_WORDS=1<<ADDR_W;
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
    wire request_one=x==0&&y==2;
    wire request_two=x==0&&y==3;
    wire request_next=de&&ax==0&&ay<H-3;
    wire request=request_zero||request_one||request_two||request_next;
    wire [10:0] request_y=request_zero ? 11'd0 : request_one ? 11'd1 :
                                   request_two ? 11'd2 : ay[10:0]+11'd3;
    wire cmd_full,cmd_empty,cmd_valid;wire [42:0] cmd_data;
    wire [4:0] diag_cmd_rd_level;
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
        .rd_valid(cmd_valid),.rd_empty(cmd_empty),.rd_level(diag_cmd_rd_level));
    // One standard SDP array, bank bits above the padded per-row word address.
    // W1920 => 4*512 words x 128 bits; write clk=100MHz, read pix_clk=148.75MHz.
    (* ram_style="block" *) reg [127:0] line_ram[0:4*BANK_WORDS-1];
    reg [127:0] pix_q;

    reg [ADDR_W-1:0] word_index;
    reg [31:0] address,row_epoch,scan_epoch;
    reg [10:0] row_y;
    reg [8:0] remaining;
    reg [7:0] chunk,received;
    reg bad;
    wire [ADDR_W+1:0] line_write_address={row_y[1:0],word_index};
    wire [ADDR_W+1:0] line_read_address={ay[1:0],ax[ADDR_W+1:2]};
    wire [8:0] boundary=(13'd4096-{1'b0,address[11:0]})>>4;
    wire [8:0] max_chunk=remaining<128?remaining:9'd128;
    wire [7:0] safe_chunk=boundary<max_chunk ? boundary[7:0] : max_chunk[7:0];
    wire fire=rvalid&&rready;
    wire [1:0] selected_slot=latest_valid?latest_slot:display_slot;
    reg [3:0] meta_toggle,meta_valid;
    reg [31:0] meta_epoch[0:3],meta_fid[0:3];reg [10:0] meta_row[0:3];integer mb;
    always @(posedge clk)begin
        if(state==R&&fire&&received<chunk)begin
            line_ram[line_write_address]<=rdata;
        end
    end
    always @(posedge pix_clk)begin
        pix_q<=line_ram[line_read_address];
    end
    assign araddr=address;assign arlen=chunk-1'b1;
    assign arsize=4;assign arburst=1;assign arvalid=state==AR&&chunk!=0;
    assign rready=state==R||state==DRAIN;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;pending<=0;address<=0;row_epoch<=0;scan_epoch<=0;row_y<=0;
            remaining<=0;chunk<=0;received<=0;word_index<=0;bad<=0;
            display_valid<=0;display_slot<=0;fault_sticky<=0;scan_fid<=0;row_fid<=0;
            meta_toggle<=0;meta_valid<=0;
            for(mb=0;mb<4;mb=mb+1)begin
                meta_epoch[mb]<=0;meta_fid[mb]<=0;meta_row[mb]<=0;
            end
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
                    meta_epoch[row_y[1:0]]<=row_epoch;meta_row[row_y[1:0]]<=row_y;
                    meta_fid[row_y[1:0]]<=row_fid;
                    meta_valid[row_y[1:0]]<=!bad;
                    meta_toggle[row_y[1:0]]<=~meta_toggle[row_y[1:0]];state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
    (* async_reg="true" *) reg [3:0] toggle_s1,toggle_s2;
    reg [3:0] seen,valid_pix;
    reg [31:0] epoch_pix[0:3],fid_pix[0:3];reg [10:0] row_pix[0:3];integer b;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            toggle_s1<=0;toggle_s2<=0;seen<=0;valid_pix<=0;
            for(b=0;b<4;b=b+1)begin
                epoch_pix[b]<=0;row_pix[b]<=0;fid_pix[b]<=0;
            end
        end else begin
            toggle_s1<=meta_toggle;toggle_s2<=toggle_s1;
            for(b=0;b<4;b=b+1)if(toggle_s2[b]!=seen[b])begin
                seen[b]<=toggle_s2[b];valid_pix[b]<=meta_valid[b];
                epoch_pix[b]<=meta_epoch[b];row_pix[b]<=meta_row[b];
                fid_pix[b]<=meta_fid[b];
            end
        end
    end
    reg de_d,hs_d,vs_d,good_d;reg [1:0] bank_d;
    reg [1:0] lane_d;reg [11:0] x_d,y_d;
    reg [23:0] rgb_q;reg de_q,hs_q,vs_q;reg [11:0] x_q,y_q;
    reg scan_armed,good_q,present_frame_good,presented_once;
    reg [31:0] fid_d,fid_q,present_frame_fid,last_presented_fid;
    wire good=valid_pix[ay[1:0]]&&row_pix[ay[1:0]]==ay[10:0]&&epoch_pix[ay[1:0]]==epoch;
    wire [127:0] q=pix_q;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            de_d<=0;hs_d<=0;vs_d<=0;bank_d<=0;good_d<=0;lane_d<=0;x_d<=0;y_d<=0;
            rgb_q<=0;de_q<=0;hs_q<=0;vs_q<=0;x_q<=0;y_q<=0;underrun_sticky<=0;scan_armed<=0;
            good_q<=0;fid_d<=0;fid_q<=0;present_frame_good<=0;presented_once<=0;
            present_frame_fid<=0;last_presented_fid<=0;frames_presented<=0;
        end else begin
            de_d<=de;hs_d<=hs;vs_d<=vs;bank_d<=ay[1:0];lane_d<=ax[1:0];good_d<=good;
            fid_d<=fid_pix[ay[1:0]];fid_q<=fid_d;good_q<=good_d;
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

    // BEGIN PASSIVE DISPLAY OBSERVATION: never feeds functional controls.
    reg [31:0] dp[0:15];
    reg [31:0] dc[0:31];
    genvar diag_word;
    generate for(diag_word=0;diag_word<16;diag_word=diag_word+1)begin:g_diag_pixel
        assign pixel_diag_live[diag_word*32+:32]=dp[diag_word];
    end endgenerate
    generate for(diag_word=0;diag_word<32;diag_word=diag_word+1)begin:g_diag_core
        assign core_diag_live[diag_word*32+:32]=dc[diag_word];
    end endgenerate
    integer diag_pi,diag_ci;
    wire diag_pixel_miss=scan_armed&&de&&!good;
    wire diag_underrun_trigger=diag_pixel_miss||request_overflow;
    wire [31:0] diag_pixel_flags={19'd0,cmd_full,
        (toggle_s2[ay[1:0]]!=seen[ay[1:0]]),ay[1:0],good,(request&&cmd_full),request_overflow,
        (epoch_pix[ay[1:0]]!=epoch),(row_pix[ay[1:0]]!=ay[10:0]),!valid_pix[ay[1:0]],de,scan_armed,1'b1};
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)begin
            for(diag_pi=0;diag_pi<16;diag_pi=diag_pi+1)dp[diag_pi]<=0;
        end else begin
            dp[0]<=dp[0]+1'b1;
            if(de)dp[1]<=dp[1]+1'b1;
            if(diag_pixel_miss)dp[2]<=dp[2]+1'b1;
            if(request&&cmd_full)dp[3]<=dp[3]+1'b1;
            if(dp[4]==0&&diag_underrun_trigger)begin
                dp[4]<=diag_pixel_flags;dp[5]<=dp[0];dp[6]<={8'd0,ay,ax};dp[7]<=epoch;
                dp[8]<={14'd0,request_overflow,cmd_full,present_frame_good,scan_armed,
                        valid_pix[ay[1:0]],ay[1:0],row_pix[ay[1:0]]};
                dp[9]<=epoch_pix[ay[1:0]];dp[10]<=fid_pix[ay[1:0]];
                dp[11]<={4'd0,request_y,request_next,request_two,request_one,request_zero,
                         request,toggle_s2,seen,valid_pix};
                dp[12]<=present_frame_fid;
            end
            if(de&&ax==0)begin dp[13]<=dp[13]+1'b1;if(good)dp[14]<=dp[14]+1'b1;end
            if(de_q&&x_q==W-1&&y_q==H-1&&
               !(present_frame_good&&good_q&&fid_q==present_frame_fid))dp[15]<=dp[15]+1'b1;
        end
    end

    // Whole-row service pressure threshold, not a substitute for pixel good.
    // Includes a row in progress; excludes request FIFO residence and CDC time.
    localparam integer DIAG_ROW_LIMIT=((64'd100000000*(W+H_FP+H_SYNC+H_BP))+
                                      64'd148750000-1)/64'd148750000;
    reg diag_row_active,diag_row_late;
    reg [31:0] diag_row_start,diag_row_ar_wait,diag_row_r_wait;
    wire [31:0] diag_elapsed=dc[0]-diag_row_start;
    wire diag_ar_wait=state==AR&&chunk!=0&&!arready;
    wire diag_r_wait=(state==R||state==DRAIN)&&!rvalid;
    wire diag_read_error=state==R&&fire&&
        (rresp!=0||(rlast&&received!=chunk-1'b1)||(!rlast&&received==chunk-1'b1));
    wire diag_reject=state==IDLE&&cmd_valid&&cmd_data[10:0]!=0&&display_valid&&
        (cmd_data[42:11]!=scan_epoch||cmd_data[10:0]>=H);
    wire [31:0] diag_context={1'b0,chunk,remaining,state,row_y};
    wire [8:0] diag_word_index={{(9-ADDR_W){1'b0}},word_index};
    wire [31:0] diag_late_flags={16'd0,rresp,(row_y>=H),(row_epoch!=scan_epoch),
        rlast,rready,rvalid,(diag_cmd_rd_level==16),cmd_empty,
        (state==DRAIN),(state==R),(state==AR),bad,display_valid,(row_y==0),1'b1};
    wire [31:0] diag_error_flags={22'd0,rresp,rlast,display_valid,(row_y==0),bad,
        (!rlast&&received==chunk-1'b1),(rlast&&received!=chunk-1'b1),(rresp!=0),1'b1};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            for(diag_ci=0;diag_ci<32;diag_ci=diag_ci+1)dc[diag_ci]<=0;
            diag_row_active<=0;diag_row_late<=0;diag_row_start<=0;
            diag_row_ar_wait<=0;diag_row_r_wait<=0;
        end else begin
            dc[0]<=dc[0]+1'b1;
            if(state==IDLE&&cmd_valid)begin
                dc[1]<=dc[1]+1'b1;diag_row_active<=1;diag_row_late<=0;
                diag_row_start<=dc[0];diag_row_ar_wait<=0;diag_row_r_wait<=0;
            end
            if(state==COMMIT)begin
                dc[2]<=dc[2]+1'b1;
                if(bad)dc[3]<=dc[3]+1'b1;
                diag_row_active<=0;
            end
            if(arvalid&&arready)dc[4]<=dc[4]+1'b1;
            if(fire)dc[5]<=dc[5]+1'b1;
            if(diag_ar_wait)begin dc[6]<=dc[6]+1'b1;diag_row_ar_wait<=diag_row_ar_wait+1'b1;end
            if(diag_r_wait)begin dc[7]<=dc[7]+1'b1;diag_row_r_wait<=diag_row_r_wait+1'b1;end
            if(diag_cmd_rd_level>dc[11])dc[11]<=diag_cmd_rd_level;
            if(diag_row_active)begin
                if(diag_elapsed>dc[8])dc[8]<=diag_elapsed;
                if(diag_row_ar_wait>dc[9])dc[9]<=diag_row_ar_wait;
                if(diag_row_r_wait>dc[10])dc[10]<=diag_row_r_wait;
                if(!diag_row_late&&diag_elapsed>=DIAG_ROW_LIMIT)begin
                    diag_row_late<=1;dc[12]<=dc[12]+1'b1;
                    if(dc[13]==0)begin
                        dc[13]<=diag_late_flags;dc[14]<=dc[0];dc[15]<=row_epoch;
                        dc[16]<=diag_context;dc[17]<=diag_elapsed;
                        dc[18]<=diag_row_ar_wait;dc[19]<=diag_row_r_wait;
                        dc[20]<={9'd0,received,diag_word_index,diag_cmd_rd_level,pending};
                    end
                end
            end
            if(diag_read_error&&dc[21]==0)begin
                dc[21]<=diag_error_flags;dc[22]<=dc[0];dc[23]<=row_epoch;
                dc[24]<=diag_context;dc[25]<={4'd0,rresp,rlast,received,chunk,remaining};
            end
            if(diag_reject)begin
                dc[26]<=dc[26]+1'b1;
                if(dc[27]==0)begin
                    dc[27]<={25'd0,(diag_cmd_rd_level==16),cmd_empty,(cmd_data[10:0]==0),
                             display_valid,(cmd_data[10:0]>=H),(cmd_data[42:11]!=scan_epoch),1'b1};
                    dc[28]<=dc[0];dc[29]<=cmd_data[42:11];
                    dc[30]<={15'd0,latest_slot,display_slot,latest_valid,display_valid,cmd_data[10:0]};
                    dc[31]<=scan_epoch;
                end
            end
        end
    end
    // END PASSIVE DISPLAY OBSERVATION.

    // synthesis translate_off
    initial if(W%4||W<8||W>2048||H<4||H>2048||V_SYNC+V_BP<4||W*H*4>SLOT_BYTES)
        $fatal(1,"CAMERA_NATIVE_DISPLAY_GEOMETRY");
    // synthesis translate_on
endmodule
