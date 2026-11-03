`timescale 1ns/1ps
module d2_a_capture #(
 parameter integer FIFO_AW=9,
 parameter integer GUIDE_FIFO_AW=10,
 parameter integer PREVIEW_CAPTURE_ENABLE=1,
 parameter integer SLOT_AW=2
)(
 input wire cam_clk,cam_rst_n,clk,rst_n,
 input wire pipeline_core_rst_n,pipeline_cam_rst_n,recovering,input wire [(1<<SLOT_AW)-1:0] preserve_slots,
 output reg hard_fault,output wire quiescent,
 output wire [191:0] diag_service,output wire diag_service_saturated,
 input wire valid,input wire [95:0] rgb4,input wire [10:0] x0,y,
 input wire [31:0] fid,input wire sof,eof,source_bad,
 input wire diag_packed_bad,diag_isp_bad,input wire [7:0] diag_bridge_code,
 output wire [191:0] diag_core,output wire [95:0] diag_cam,output wire diag_cam_valid,
 input wire release_valid,input wire [SLOT_AW-1:0] release_slot,
 output reg committed,output reg [SLOT_AW-1:0] committed_slot,output reg [31:0] committed_fid,
 output reg fault,output reg [31:0] captured,dropped,
 output reg [239:0] cmd,input wire [137:0] rsp
);
 localparam integer SLOTS=(1<<SLOT_AW),FIFO_DW=162+SLOT_AW,SOF_BIT=160+SLOT_AW,EOF_BIT=161+SLOT_AW;
 localparam [SLOTS-1:0] SLOT_ONE={{(SLOTS-1){1'b0}},1'b1};
 wire rel_full,rel_empty,rel_valid;wire [SLOT_AW-1:0] rel_data;
 d64_async_fifo #(.DW(SLOT_AW),.AW(SLOT_AW+1)) u_release(.wr_clk(clk),.wr_rst_n(rst_n),.wr_en(release_valid),.wr_data(release_slot),.wr_full(rel_full),.wr_almost_full(),.wr_level(),
 .rd_clk(cam_clk),.rd_rst_n(cam_rst_n),.rd_en(!rel_empty),.rd_data(rel_data),.rd_valid(rel_valid),.rd_empty(rel_empty),.rd_level());
 reg [SLOTS-1:0] busy;reg taking;reg [SLOT_AW-1:0] slot;
 reg soft_reset_seen;reg [SLOTS-1:0] reset_releases;
 (* async_reg="true" *)reg [1:0] admit_sync;
 always @(posedge cam_clk or negedge cam_rst_n)
 if(!cam_rst_n)admit_sync<=0;else admit_sync<={admit_sync[0],!recovering};
 wire any_free=!(&busy);reg [SLOT_AW-1:0] next_slot;
 integer free_index;reg free_found;
 always @*begin
 next_slot=SLOTS-1;free_found=0;
 for(free_index=0;free_index<SLOTS;free_index=free_index+1)
 if(!free_found&&!busy[free_index])begin next_slot=free_index;free_found=1;end
 end
 wire use_frame=sof?any_free:taking;wire [SLOT_AW-1:0] use_slot=sof?next_slot:slot;
 reg cam_fault;reg [31:0] drop_cam,drop_gray;
 reg [127:0] gp,pp,cp;reg [1:0] gn,cn;reg [2:0] pn;
 reg [7:0] cx,cy;reg [10:0] tx,ty;
 reg [FIFO_DW-1:0] wd[0:2];reg [2:0] we;
 wire [2:0] full,empty,rv;wire [FIFO_DW-1:0] rd[0:2];localparam MAX_FIFO_AW=(GUIDE_FIFO_AW>FIFO_AW)?GUIDE_FIFO_AW:FIFO_AW;
 wire [MAX_FIFO_AW:0] level[0:2];reg [2:0] re;
 localparam [2:0] REQUIRED_PLANES=PREVIEW_CAPTURE_ENABLE ? 3'b111 : 3'b101;
 genvar z;generate for(z=0;z<3;z=z+1)begin:g_fifo
 if(z!=1||PREVIEW_CAPTURE_ENABLE)begin:g_enabled
 localparam THIS_AW=(z==0)?GUIDE_FIFO_AW:FIFO_AW;
 wire [THIS_AW:0] fifo_level;
 assign level[z]=fifo_level;
 d64_async_fifo #(.DW(FIFO_DW),.AW(THIS_AW)) u_fifo(.wr_clk(cam_clk),.wr_rst_n(pipeline_cam_rst_n),.wr_en(we[z]),.wr_data(wd[z]),.wr_full(full[z]),.wr_almost_full(),.wr_level(),
 .rd_clk(clk),.rd_rst_n(pipeline_core_rst_n),.rd_en(re[z]),.rd_data(rd[z]),.rd_valid(rv[z]),.rd_empty(empty[z]),.rd_level(fifo_level));
 end else begin:g_disabled
 assign full[z]=1'b0;assign empty[z]=1'b1;assign rv[z]=1'b0;
 assign rd[z]={FIFO_DW{1'b0}};assign level[z]=0;
 end
 end endgenerate
 integer j;reg [127:0] tmp;reg [15:0] p0,p2;reg [23:0] cr;
 always @(posedge cam_clk or negedge cam_rst_n)begin
 if(!cam_rst_n)begin soft_reset_seen<=0;reset_releases<=0;busy<=0;taking<=0;slot<=0;cam_fault<=0;drop_cam<=0;drop_gray<=0;gp<=0;pp<=0;cp<=0;gn<=0;pn<=0;cn<=0;cx<=0;cy<=0;tx<=3;ty<=3;we<=0;wd[0]<=0;wd[1]<=0;wd[2]<=0;end
 else begin
 we<=0;
 if(rel_valid)busy[rel_data]<=0;
 if(!pipeline_cam_rst_n)begin
 if(!soft_reset_seen)reset_releases<=rel_valid?(SLOT_ONE<<rel_data):{SLOTS{1'b0}};
 else if(rel_valid)reset_releases[rel_data]<=1;
 soft_reset_seen<=1;taking<=0;slot<=0;cam_fault<=0;gp<=0;pp<=0;cp<=0;gn<=0;pn<=0;cn<=0;cx<=0;cy<=0;tx<=3;ty<=3;
 // Release FIFO is deliberately kept alive across capture recovery.
 if(rel_valid)busy[rel_data]<=0;
 end else begin
 if(soft_reset_seen)begin
 busy<=preserve_slots&~reset_releases;
 if(rel_valid)busy[rel_data]<=0;
 end
 soft_reset_seen<=0;
 if(source_bad||(we&full)!=0)cam_fault<=1;
 if(valid&&!cam_fault&&!source_bad&&admit_sync[1])begin
 if(sof)begin
 taking<=any_free;slot<=next_slot;gn<=0;pn<=0;cn<=0;cx<=0;cy<=0;tx<=3;ty<=3;
 if(any_free)busy[next_slot]<=1;else begin drop_cam<=drop_cam+1;drop_gray<=((drop_cam+1)>>1)^(drop_cam+1);end
 end
 if(use_frame)begin
 tmp=gp;for(j=0;j<4;j=j+1)tmp[(sof?0:gn)*32+j*8+:8]=rgb4[j*24+8+:8];gp<=tmp;
 if((sof?0:gn)==3)begin we[0]<=1;wd[0]<={eof,(x0==12&&y==0),use_slot,fid,tmp};gn<=0;end else gn<=(sof?0:gn)+1;
 if(PREVIEW_CAPTURE_ENABLE&&!y[0])begin
 p0={rgb4[23:19],rgb4[15:10],rgb4[7:3]};p2={rgb4[71:67],rgb4[63:58],rgb4[55:51]};tmp=pp;
 tmp[(sof?0:pn)*16+:16]=p0;tmp[((sof?0:pn)+1)*16+:16]=p2;pp<=tmp;
 if((sof?0:pn)==6)begin we[1]<=1;wd[1]<={(x0==1916&&y==1078),(x0==12&&y==0),use_slot,fid,tmp};pn<=0;end else pn<=(sof?0:pn)+2;
 end
 if(y==ty&&x0<=tx&&x0+3>=tx)begin
 cr=rgb4[(tx-x0)*24+:24];tmp=cp;tmp[cn*32+:32]={8'd0,cr};cp<=tmp;
 if(cn==3)begin we[2]<=1;wd[2]<={(cx==255&&cy==143),(cx==3&&cy==0),use_slot,fid,tmp};cn<=0;end else cn<=cn+1;
 if(cx==255)begin cx<=0;tx<=3;cy<=cy+1;ty<=ty+(cy[0]?8:7);end
 else begin cx<=cx+1;tx<=tx+(cx[0]?8:7);end
 end
 if(eof)taking<=0;
 end
 end
 end
 end end
 (* async_reg="true" *) reg [1:0] bad_sync;
 always @(posedge clk or negedge rst_n)if(!rst_n)bad_sync<=0;else bad_sync<={bad_sync[0],cam_fault};
 (* async_reg="true" *)reg [31:0] drop_s1,drop_s2;
 integer db;reg [31:0] drop_binary;
 always @*begin drop_binary[31]=drop_s2[31];for(db=30;db>=0;db=db-1)drop_binary[db]=drop_binary[db+1]^drop_s2[db];end
 always @(posedge clk or negedge rst_n)if(!rst_n)begin drop_s1<=0;drop_s2<=0;end else begin drop_s1<=drop_gray;drop_s2<=drop_s1;end
 // Fill and AXI service are independent. The two banks form a two-entry
 // FIFO: a bank is never refilled until its B response has been consumed.
 localparam IDLE=0,FILL=1,AW=2,LOAD=3,DATA=4,B=5;
 reg [2:0] state;reg [1:0] plane,round,burst_plane;
 reg filling,fill_bank,tx_bank,active_bank;
 reg [1:0] bank_ready,bank_taken,bank_eof;
 reg [1:0] bank_plane[0:1];reg [SLOT_AW-1:0] bank_slot[0:1];
 reg [31:0] bank_index[0:1],bank_fid[0:1];
 reg [5:0] issued,received,sent;reg [4:0] raddr;
 (* ram_style="block" *) reg [127:0] ram[0:63];reg [127:0] q;
 reg [31:0] index[0:2],frame_fid[0:2];reg [SLOT_AW-1:0] frame_slot[0:2];
 reg [2:0] completed[0:SLOTS-1];reg burst_eof;reg [SLOT_AW-1:0] burst_slot;reg [31:0] burst_fid,burst_index;
 wire [31:0] total=plane==0?129600:plane==1?64800:9216;
 wire [31:0] offset=burst_plane==0?0:burst_plane==1?32'h200000:32'h340000;
 wire [31:0] base=32'h01000000+({{(32-SLOT_AW){1'b0}},burst_slot}<<22);
 assign quiescent=state==IDLE&&!cmd[0]&&!cmd[46]&&!cmd[192];
 wire [4:0] next_read=(state==DATA&&rsp[1]&&sent!=31)?sent[4:0]+1'b1:raddr;
 always @(posedge clk)begin
 if(pipeline_core_rst_n&&rv[plane]&&filling)ram[{fill_bank,received[4:0]}]<=rd[plane][127:0];
 q<=ram[{active_bank,next_read}];
 end
 always @*begin
 re=0;if(pipeline_core_rst_n&&!fault&&!bad_sync[1]&&filling&&issued<32)re[plane]=!empty[plane];
 cmd=0;
 cmd[32:1]=base+offset+(burst_index<<4);cmd[40:33]=31;cmd[43:41]=4;cmd[45:44]=1;
 cmd[0]=state==AW;cmd[46]=state==DATA;cmd[174:47]=q;cmd[190:175]=16'hffff;cmd[191]=sent==31;cmd[192]=state==B;cmd[239]=1;
 end
 integer k;reg [1:0] pick;reg found;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 state<=IDLE;plane<=0;round<=0;burst_plane<=0;filling<=0;fill_bank<=0;tx_bank<=0;active_bank<=0;bank_ready<=0;bank_taken<=0;bank_eof<=0;
 issued<=0;received<=0;sent<=0;raddr<=0;fault<=0;hard_fault<=0;committed<=0;committed_slot<=0;committed_fid<=0;captured<=0;dropped<=0;burst_eof<=0;burst_slot<=0;burst_fid<=0;burst_index<=0;
 for(k=0;k<2;k=k+1)begin bank_plane[k]<=0;bank_slot[k]<=0;bank_index[k]<=0;bank_fid[k]<=0;end
 for(k=0;k<3;k=k+1)begin index[k]<=0;frame_fid[k]<=0;frame_slot[k]<=0;end
 for(k=0;k<SLOTS;k=k+1)completed[k]<=0;
 end else begin
 committed<=0;dropped<=drop_binary;
 if(rel_full)hard_fault<=1;
 if(!pipeline_core_rst_n)begin
 state<=IDLE;plane<=0;round<=0;burst_plane<=0;filling<=0;fill_bank<=0;tx_bank<=0;active_bank<=0;bank_ready<=0;bank_taken<=0;bank_eof<=0;
 issued<=0;received<=0;sent<=0;raddr<=0;fault<=0;burst_eof<=0;burst_slot<=0;burst_fid<=0;burst_index<=0;
 for(k=0;k<2;k=k+1)begin bank_plane[k]<=0;bank_slot[k]<=0;bank_index[k]<=0;bank_fid[k]<=0;end
 for(k=0;k<3;k=k+1)begin index[k]<=0;frame_fid[k]<=0;frame_slot[k]<=0;end
 for(k=0;k<SLOTS;k=k+1)completed[k]<=0;
 end else begin
 if(bad_sync[1])fault<=1;
 if(!filling&&!bank_taken[fill_bank]&&!fault&&!bad_sync[1]&&!recovering&&!hard_fault)begin
 found=0;pick=0;
 for(k=0;k<3;k=k+1)if(!found&&(PREVIEW_CAPTURE_ENABLE||(round+k)%3!=1)&&level[(round+k)%3]>=32)begin pick=(round+k)%3;found=1;end
 if(found)begin
 plane<=pick;issued<=0;received<=0;filling<=1;bank_taken[fill_bank]<=1;bank_eof[fill_bank]<=0;
 bank_plane[fill_bank]<=pick;bank_index[fill_bank]<=index[pick];
 end
 end
 if(filling)begin
 if(fault||bad_sync[1]||recovering||hard_fault)filling<=0;
 else begin
 if(re[plane])issued<=issued+1'b1;
 if(rv[plane])begin
 if(received==0)begin bank_slot[fill_bank]<=rd[plane][159+SLOT_AW:160];bank_fid[fill_bank]<=rd[plane][159:128];end
 if(index[plane]==0&&received==0)begin
 if(!rd[plane][SOF_BIT])fault<=1;
 frame_fid[plane]<=rd[plane][159:128];frame_slot[plane]<=rd[plane][159+SLOT_AW:160];
 end else if(rd[plane][SOF_BIT]||frame_fid[plane]!=rd[plane][159:128]||frame_slot[plane]!=rd[plane][159+SLOT_AW:160])fault<=1;
 if(rd[plane][EOF_BIT]!=(index[plane]+received==total-1))fault<=1;
 if(rd[plane][EOF_BIT])bank_eof[fill_bank]<=1;
 received<=received+1'b1;
 if(received==31)begin
 filling<=0;bank_ready[fill_bank]<=1;fill_bank<=!fill_bank;round<=plane==2?0:plane+1'b1;
 if(index[plane]+32==total)index[plane]<=0;else index[plane]<=index[plane]+32;
 end
 end
 end
 end
 case(state)
 IDLE:if(bank_ready[tx_bank]&&!fault&&!bad_sync[1]&&!recovering&&!hard_fault)begin
 active_bank<=tx_bank;burst_plane<=bank_plane[tx_bank];burst_slot<=bank_slot[tx_bank];burst_fid<=bank_fid[tx_bank];burst_index<=bank_index[tx_bank];burst_eof<=bank_eof[tx_bank];
 sent<=0;raddr<=0;state<=AW;
 end
 AW:if(rsp[0])begin raddr<=0;state<=LOAD;end
 LOAD:state<=DATA;
 DATA:if(rsp[1])begin if(sent==31)state<=B;else begin sent<=sent+1'b1;raddr<=sent[4:0]+1'b1;end end
 B:if(rsp[2])begin
 if(rsp[4:3]!=0)begin fault<=1;hard_fault<=1;end
 if(burst_eof)begin
 completed[burst_slot][burst_plane]<=1;
 if((completed[burst_slot]|(3'b001<<burst_plane))==REQUIRED_PLANES)begin
 completed[burst_slot]<=0;
 if(!fault&&!bad_sync[1]&&!recovering&&!hard_fault&&rsp[4:3]==0)begin committed<=1;committed_slot<=burst_slot;committed_fid<=burst_fid;captured<=captured+1'b1;end
 end
 end
 bank_ready[active_bank]<=0;bank_taken[active_bank]<=0;tx_bank<=!tx_bank;state<=IDLE;
 end
 default:begin fault<=1;hard_fault<=1;state<=IDLE;end
 endcase
 end
 end
 end
 // Passive service statistics use the continuous core clock and survive
 // capture-only recovery. Wait lengths saturate at 24'hffffff (167.77215ms
 // at100MHz); the saturation flag means at least that duration, not exact.
 reg [MAX_FIFO_AW:0] guide_highwater;
 reg [23:0] aw_wait,w_wait,b_wait,gap_wait;
 reg [23:0] aw_wait_max,w_wait_max,b_wait_max,gap_wait_max;
 reg [31:0] responded_bursts;
 wire aw_stall=state==AW&&!rsp[0];
 wire w_stall=state==DATA&&!rsp[1];
 wire b_stall=state==B&&!rsp[2];
 wire g8_no_pop=level[0]>=32&&!re[0];
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 guide_highwater<=0;aw_wait<=0;w_wait<=0;b_wait<=0;gap_wait<=0;
 aw_wait_max<=0;w_wait_max<=0;b_wait_max<=0;gap_wait_max<=0;responded_bursts<=0;
 end else begin
 if(level[0]>guide_highwater)guide_highwater<=level[0];
 if(aw_stall)begin
 if(aw_wait!=24'hffffff)aw_wait<=aw_wait+1'b1;
 if(aw_wait!=24'hffffff&&aw_wait+1'b1>aw_wait_max)aw_wait_max<=aw_wait+1'b1;
 end else aw_wait<=0;
 if(w_stall)begin
 if(w_wait!=24'hffffff)w_wait<=w_wait+1'b1;
 if(w_wait!=24'hffffff&&w_wait+1'b1>w_wait_max)w_wait_max<=w_wait+1'b1;
 end else w_wait<=0;
 if(b_stall)begin
 if(b_wait!=24'hffffff)b_wait<=b_wait+1'b1;
 if(b_wait!=24'hffffff&&b_wait+1'b1>b_wait_max)b_wait_max<=b_wait+1'b1;
 end else b_wait<=0;
 if(g8_no_pop)begin
 if(gap_wait!=24'hffffff)gap_wait<=gap_wait+1'b1;
 if(gap_wait!=24'hffffff&&gap_wait+1'b1>gap_wait_max)gap_wait_max<=gap_wait+1'b1;
 end else gap_wait<=0;
 if(state==B&&rsp[2])responded_bursts<=responded_bursts+1'b1;
 end
 assign diag_service={8'd0,gap_wait_max,responded_bursts,8'd0,b_wait_max,8'd0,w_wait_max,8'd0,aw_wait_max,{{(31-MAX_FIFO_AW){1'b0}},guide_highwater}};
 assign diag_service_saturated=(aw_wait_max==24'hffffff)||(w_wait_max==24'hffffff)||(b_wait_max==24'hffffff)||(gap_wait_max==24'hffffff);
 reg [31:0] diag_causes,diag_first_index;reg diag_first_seen;
 reg [31:0] diag_events,diag_status,diag_cam_flags;
 always @*begin
 diag_events=0;
 diag_events[0]=bad_sync[1];diag_events[1]=rel_full;
 if(filling&&rv[plane])begin
 diag_events[2]=index[plane]==0&&received==0&&!rd[plane][SOF_BIT];
 diag_events[3]=!(index[plane]==0&&received==0)&&rd[plane][SOF_BIT];
 diag_events[4]=!(index[plane]==0&&received==0)&&frame_fid[plane]!=rd[plane][159:128];
 diag_events[5]=!(index[plane]==0&&received==0)&&frame_slot[plane]!=rd[plane][159+SLOT_AW:160];
 diag_events[6]=rd[plane][EOF_BIT]!=(index[plane]+received==total-1);
 end
 diag_events[7]=state==B&&rsp[2]&&rsp[4:3]!=0;
 diag_events[8]=state>5;
 diag_status=0;diag_status[2:0]=state;diag_status[4:3]=plane;diag_status[6:5]=round;
 diag_status[12:7]=issued;diag_status[18:13]=received;diag_status[24:19]=sent;
 diag_status[31]=filling;diag_status[30:29]=bank_ready;diag_status[25]=rel_full;diag_status[26]=bad_sync[1];diag_status[27]=fault;diag_status[28]=diag_first_seen;
 diag_cam_flags=0;diag_cam_flags[0]=source_bad;diag_cam_flags[3:1]=we&full;
 diag_cam_flags[6:4]=full;diag_cam_flags[9:7]=we;diag_cam_flags[10]=cam_fault;
 diag_cam_flags[11]=diag_packed_bad;diag_cam_flags[12]=diag_isp_bad;diag_cam_flags[23:16]=diag_bridge_code;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin diag_causes<=0;diag_first_index<=0;diag_first_seen<=0;end
 else begin
 diag_causes<=diag_causes|diag_events;
 if((|diag_events)&&!diag_first_seen)begin diag_first_seen<=1;diag_first_index<=index[plane];end
 end
 d2_diag_snapshot #(.W(96))u_diag_cam(.src_clk(cam_clk),.src_rst_n(cam_rst_n),
 .trigger(source_bad||((we&full)!=0)),.source_data({fid,10'd0,y,x0,diag_cam_flags}),
 .dst_clk(clk),.dst_rst_n(rst_n),.data(diag_cam),.valid(diag_cam_valid));
 assign diag_core={diag_first_index,index[2],index[1],index[0],diag_status,diag_causes};
 // synthesis translate_off
 initial if(PREVIEW_CAPTURE_ENABLE!=0&&PREVIEW_CAPTURE_ENABLE!=1)
   $fatal(1,"PREVIEW_CAPTURE_ENABLE must be0or1");
 initial if(SLOT_AW<2||SLOT_AW>3)$fatal(1,"SLOT_AW must be2or3");
 // synthesis translate_on
endmodule
