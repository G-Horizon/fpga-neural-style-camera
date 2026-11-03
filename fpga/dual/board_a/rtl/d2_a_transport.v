`timescale 1ns/1ps
module d2_a_transport(
 input wire clk,rst_n,pix_clk,pix_rst_n,frame_start,de,
 input wire valid,output wire ready,input wire [1:0] slot,
 input wire [31:0] fid,epoch,input wire [1:0] style,layout,input wire fps,input wire [7:0] effect,
 output reg released,output reg released_ok,output reg [1:0] released_slot,output reg fault,
 output reg [31:0] transmitted,output wire [23:0] rgb,
 output wire [95:0] diag_core,output wire [127:0] diag_pixel,output wire diag_pixel_valid,
 output reg [239:0] cmd,input wire [137:0] rsp
);
 localparam WORDS=203616;
 wire desc_full,desc_empty,desc_rvalid;wire [78:0] desc_data;
 reg desc_write;reg [78:0] desc_hold;reg desc_re;
 d64_async_fifo #(.DW(79),.AW(2)) u_desc(.wr_clk(clk),.wr_rst_n(rst_n),.wr_en(desc_write),.wr_data(desc_hold),.wr_full(desc_full),.wr_almost_full(),.wr_level(),
 .rd_clk(pix_clk),.rd_rst_n(pix_rst_n),.rd_en(desc_re),.rd_data(desc_data),.rd_valid(desc_rvalid),.rd_empty(desc_empty),.rd_level());
 wire fifo_full,fifo_empty,fifo_valid;wire [128:0] fifo_data;wire [9:0] fifo_level;reg fifo_read;
 wire fifo_write=rsp[6]&&cmd[239];
 wire response_bad;
 d64_async_fifo #(.DW(129),.AW(9)) u_words(.wr_clk(clk),.wr_rst_n(rst_n),.wr_en(fifo_write),.wr_data({response_bad,rsp[134:7]}),.wr_full(fifo_full),.wr_almost_full(),.wr_level(fifo_level),
 .rd_clk(pix_clk),.rd_rst_n(pix_rst_n),.rd_en(fifo_read),.rd_data(fifo_data),.rd_valid(fifo_valid),.rd_empty(fifo_empty),.rd_level());
 wire ack_full,ack_empty,ack_rvalid;wire [2:0] ack_data;reg ack_write;reg [2:0] ack_hold;
 d64_async_fifo #(.DW(3),.AW(2)) u_done(.wr_clk(pix_clk),.wr_rst_n(pix_rst_n),.wr_en(ack_write),.wr_data(ack_hold),.wr_full(ack_full),.wr_almost_full(),.wr_level(),
 .rd_clk(clk),.rd_rst_n(rst_n),.rd_en(!ack_empty),.rd_data(ack_data),.rd_valid(ack_rvalid),.rd_empty(ack_empty),.rd_level());
 localparam IDLE=0,AR=1,R=2,WAIT_DONE=3;
 reg [1:0] state,slot_q;reg [17:0] word_index;reg [7:0] chunk,received;
 assign response_bad=rsp[137:136]!=0||rsp[135]!=(received==chunk-1);
 wire [31:0] segment_offset=word_index<129600?(word_index<<4):word_index<194400?(32'h200000+((word_index-129600)<<4)):(32'h300000+((word_index-194400)<<4));
 wire [31:0] segment_remaining=word_index<129600?129600-word_index:word_index<194400?194400-word_index:203616-word_index;
 wire [31:0] address=32'h01000000+{8'd0,slot_q,22'd0}+segment_offset;
 wire [8:0] boundary=(4096-{1'b0,address[11:0]})>>4;
 wire [8:0] limit=segment_remaining<128?segment_remaining:128;
 wire [8:0] next_chunk=boundary<limit?boundary:limit;
 assign ready=state==IDLE&&!desc_full&&!fault;
 always @*begin
 cmd=0;cmd[192]=1;cmd[193]=state==AR&&chunk!=0&&fifo_level<=384;cmd[225:194]=address;cmd[233:226]=chunk-1;cmd[236:234]=4;cmd[238:237]=1;cmd[239]=state==R&&!fifo_full;
 end
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin state<=IDLE;slot_q<=0;word_index<=0;chunk<=0;received<=0;desc_write<=0;desc_hold<=0;released<=0;released_ok<=0;released_slot<=0;fault<=0;transmitted<=0;end
 else begin
 desc_write<=0;released<=0;
 if(valid&&ready)begin slot_q<=slot;word_index<=0;chunk<=128;received<=0;desc_hold<={slot,fid,epoch,style,layout,fps,effect};desc_write<=1;state<=AR;end
 case(state)
 AR:if(cmd[193]&&rsp[5])begin state<=R;received<=0;end
 R:if(fifo_write)begin
 if(rsp[137:136]!=0||rsp[135]!=(received==chunk-1))fault<=1;
 if(rsp[135])begin
 word_index<=word_index+chunk;
 if(word_index+chunk==WORDS)state<=IDLE;
 else begin state<=AR;chunk<=0;end
 end else received<=received+1;
 end
 default:begin end
 endcase
 if(state==AR&&chunk==0)chunk<=next_chunk;
 if(ack_rvalid)begin released<=1;released_ok<=ack_data[0];released_slot<=ack_data[2:1];if(ack_data[0])transmitted<=transmitted+1;else fault<=1;end
 end end
 reg have_desc,desc_pending,launched,draining,clear;
 reg active_valid,pixel_bad,bad_ack_pending;reg [1:0] active_slot,completed_slot,bad_ack_slot;
 reg [78:0] pixel_desc;reg [17:0] popped;reg read_pending;
 wire [23:0] payload_data;wire [5:0] available;wire in_ready,byte_fault;
 wire [1:0] payload_need,payload_take;wire tx_ready,frame_done,frame_ok,underflow;
 wire [31:0] tx_fid,diag_pixels,diag_consumed;wire diag_packet_bad;
 d2_word128_to_bytes24 u_bytes(.clk(pix_clk),.rst_n(pix_rst_n),.clear(clear),
 .in_data(fifo_data[127:0]),.in_valid(fifo_valid&&!clear&&!draining),.in_ready(in_ready),
 .out_data(payload_data),.available(available),.take(payload_take),.fault_sticky(byte_fault));
 d2_hdmi_payload_tx u_packet(.clk(pix_clk),.rst_n(pix_rst_n),.frame_start(frame_start),.de(de),
 .desc_valid(have_desc&&!launched&&available>=16),.desc_ready(tx_ready),.source_fid(pixel_desc[76:45]),.style_epoch(pixel_desc[44:13]),
 .style(pixel_desc[12:11]),.layout(pixel_desc[10:9]),.fps_enable(pixel_desc[8]),.effect(pixel_desc[7:0]),
 .payload_data(payload_data),.payload_valid(available>=payload_need&&!pixel_bad&&!(fifo_valid&&fifo_data[128])),.payload_need(payload_need),.payload_take(payload_take),
 .rgb(rgb),.frame_done(frame_done),.frame_ok(frame_ok),.frame_done_fid(tx_fid),.underflow_sticky(underflow),.diag_pixels(diag_pixels),.diag_consumed(diag_consumed),.diag_bad(diag_packet_bad));
 always @(posedge pix_clk or negedge pix_rst_n)begin
 if(!pix_rst_n)begin have_desc<=0;desc_pending<=0;desc_re<=0;pixel_desc<=0;launched<=0;draining<=0;active_valid<=0;pixel_bad<=0;bad_ack_pending<=0;bad_ack_slot<=0;active_slot<=0;completed_slot<=0;clear<=0;popped<=0;read_pending<=0;fifo_read<=0;ack_write<=0;ack_hold<=0;end
 else begin
 desc_re<=0;fifo_read<=0;ack_write<=0;clear<=0;
 if(!have_desc&&!desc_pending&&!desc_empty)begin desc_re<=1;desc_pending<=1;end
 if(desc_rvalid)begin pixel_desc<=desc_data;have_desc<=1;desc_pending<=0;launched<=0;draining<=0;pixel_bad<=0;popped<=0;clear<=1;end
 if(have_desc&&!clear&&!read_pending&&popped<WORDS&&!fifo_empty&&(draining||in_ready))begin fifo_read<=1;read_pending<=1;end
 if(fifo_valid)begin read_pending<=0;popped<=popped+1;if(fifo_data[128])pixel_bad<=1;end
 if(tx_ready&&have_desc&&!launched&&available>=16)launched<=1;
 if(frame_start)begin
 completed_slot<=active_slot;active_valid<=have_desc&&!launched&&available>=16;
 if(have_desc&&!launched&&available>=16)active_slot<=pixel_desc[78:77];
 end
 if(frame_done&&!ack_full)begin
 if(frame_ok)begin ack_hold<={completed_slot,1'b1};ack_write<=1;end
 else begin bad_ack_pending<=1;bad_ack_slot<=completed_slot;end
 if(!frame_ok&&have_desc&&launched)begin draining<=1;clear<=1;end
 end
 if(bad_ack_pending&&popped==WORDS&&!read_pending&&!ack_full)begin ack_hold<={bad_ack_slot,1'b0};ack_write<=1;bad_ack_pending<=0;end
 if(have_desc&&launched&&popped==WORDS&&!read_pending&&(draining||available==0))begin
 have_desc<=0;launched<=0;draining<=0;clear<=1;
 end
 end end
 reg [31:0] diag_tx_causes;reg [31:0] diag_tx_status,diag_pixel_flags;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)diag_tx_causes<=0;
 else begin
 if(state==R&&fifo_write&&rsp[137:136]!=0)diag_tx_causes[0]<=1;
 if(state==R&&fifo_write&&rsp[135]!=(received==chunk-1))diag_tx_causes[1]<=1;
 if(ack_rvalid&&!ack_data[0])diag_tx_causes[2]<=1;
 end
 always @*begin
 diag_tx_status=0;diag_tx_status[1:0]=state;diag_tx_status[3:2]=slot_q;
 diag_tx_status[11:4]=chunk;diag_tx_status[19:12]=received;
 diag_tx_status[20]=desc_full;diag_tx_status[21]=fifo_full;diag_tx_status[31:22]=fifo_level;
 diag_pixel_flags=0;diag_pixel_flags[0]=underflow;diag_pixel_flags[1]=byte_fault;
 diag_pixel_flags[2]=pixel_bad;diag_pixel_flags[3]=fifo_valid&&fifo_data[128];
 diag_pixel_flags[4]=frame_done;diag_pixel_flags[5]=frame_ok;diag_pixel_flags[6]=diag_packet_bad;
 diag_pixel_flags[7]=have_desc;diag_pixel_flags[8]=launched;diag_pixel_flags[9]=draining;
 diag_pixel_flags[15:10]=available;diag_pixel_flags[16]=read_pending;diag_pixel_flags[17]=fifo_empty;
 diag_pixel_flags[19:18]=active_slot;diag_pixel_flags[21:20]=completed_slot;diag_pixel_flags[22]=ack_full;
 end
 assign diag_core={{14'd0,word_index},diag_tx_status,diag_tx_causes};
 d2_diag_snapshot #(.W(128))u_diag_pix(.src_clk(pix_clk),.src_rst_n(pix_rst_n),
 .trigger(underflow||byte_fault||pixel_bad||(fifo_valid&&fifo_data[128])||(frame_done&&!frame_ok)),
 .source_data({diag_consumed,diag_pixels,14'd0,popped,diag_pixel_flags}),
 .dst_clk(clk),.dst_rst_n(rst_n),.data(diag_pixel),.valid(diag_pixel_valid));
endmodule
