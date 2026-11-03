// Observation only. Inputs are the frozen bridge's actual pre-edge registers.
module ppi_bridge_reject_observer #(
 parameter integer IDLE_GUARD=4,MAX_PREFIX=2,MAX_HEAD_WAIT=8
)(
 input wire clk,rst_n,
 input wire [2:0] state,
 input wire [15:0] quiet,head_age,prefix_count,body_offset,
 input wire [31:0] ppi_data,
 input wire [3:0] ppi_valid,ppi_sync,ppi_stop,ppi_sot,sync_rise,seen_sync,
 input wire boundary_good,activity_seen,ever_fs,frame_open,
 input wire [7:0] packet_error,
 input wire [31:0] local_fid,
 input wire [15:0] sensor_number,previous_sensor_number,
 input wire [10:0] rows_in_frame,
 input wire [9:0] packet_words,
 input wire [3:0] reservoir_bytes,
 input wire packet_has_bank,pending_last_valid,
 input wire [15:0] crc,received_crc,
 output reg [31:0] sample_seq,
 output wire [511:0] first_context,
 output reg [511:0] current_context
);
 localparam [2:0] WAIT_IDLE=0,WAIT_HEAD=1,BODY=2,TAIL=3;
 reg txn_open,candidate_seen,first_valid;
 reg [31:0] txn_id,txn_start,candidate_header;
 reg [15:0] accumulated_mask;
 reg [511:0] first_hold;
 wire quiet_close=quiet>=IDLE_GUARD-1&&ppi_valid==0&&ppi_sync==0&&ppi_stop==15;
 wire head_close=state==WAIT_HEAD&&activity_seen&&quiet_close;
 wire packet_close=state!=WAIT_IDLE&&state!=WAIT_HEAD&&quiet_close;
 wire arm_head=state==WAIT_IDLE&&quiet_close;
 wire first_activity=state==WAIT_HEAD&&!txn_open&&(|ppi_valid|| |sync_rise);
 wire candidate_now=state==WAIT_HEAD&&ppi_valid==15&&!candidate_seen;
 wire header_valid=candidate_seen||candidate_now;
 wire [31:0] header=candidate_now?ppi_data:(candidate_seen?candidate_header:32'd0);
 wire [31:0] effective_txn=first_activity?txn_id+1'b1:txn_id;
 wire [31:0] effective_start=first_activity?sample_seq:txn_start;
 reg [3:0] nonb8;
 reg [15:0] mask_now;
 integer lane;
 always @*begin
  nonb8=0;
  for(lane=0;lane<4;lane=lane+1)
   if(ppi_valid[lane]&&ppi_data[lane*8+:8]!=8'hb8)nonb8[lane]=1;
  mask_now=0;
  if(state==WAIT_HEAD&&ppi_valid==15)begin
   mask_now[0]=!(|(seen_sync|sync_rise));
   mask_now[1]=!boundary_good;
   mask_now[2]=head_age>=MAX_HEAD_WAIT;
  end
  if(state==WAIT_HEAD&&|ppi_valid&&ppi_valid!=15)begin
   mask_now[3]=prefix_count>=MAX_PREFIX;
   mask_now[4]=|nonb8;
  end
  mask_now[5]=head_close;
  mask_now[6]=state==BODY&&|sync_rise&&packet_error==0;
  mask_now[7]=state==TAIL&&|sync_rise&&packet_error==0;
  current_context=0;
  current_context[0*32+:32]={3'd0,seen_sync,state,activity_seen,boundary_good,frame_open,ever_fs,header_valid,(|mask_now),mask_now};
  current_context[1*32+:32]=sample_seq;
  current_context[2*32+:32]=effective_txn;
  current_context[3*32+:32]=effective_start;
  current_context[4*32+:32]=header;
  current_context[5*32+:32]=ppi_data;
  current_context[6*32+:32]={head_age,quiet};
  current_context[7*32+:32]={body_offset,prefix_count};
  current_context[8*32+:32]=local_fid;
  current_context[9*32+:32]={previous_sensor_number,sensor_number};
  current_context[10*32+:32]={5'd0,pending_last_valid,packet_has_bank,reservoir_bytes,packet_words,rows_in_frame};
  current_context[11*32+:32]={packet_error,ppi_sync,nonb8,ppi_sot,ppi_stop,ppi_valid,sync_rise};
  current_context[12*32+:32]={header_valid?header[23:8]:16'd0,header_valid?header[7:6]:2'd0,header_valid?header[5:0]:6'h3f,packet_error};
  current_context[13*32+:32]={received_crc,crc};
  current_context[14*32+:32]={16'd0,accumulated_mask|mask_now};
 end
 assign first_context=first_valid?first_hold:((|mask_now)?current_context:512'd0);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   sample_seq<=0;txn_open<=0;candidate_seen<=0;first_valid<=0;
   txn_id<=0;txn_start<=0;candidate_header<=0;accumulated_mask<=0;first_hold<=0;
  end else begin
   sample_seq<=sample_seq+1'b1;
   if(first_activity)begin txn_open<=1;txn_id<=txn_id+1'b1;txn_start<=sample_seq;end
   if(candidate_now)begin candidate_seen<=1;candidate_header<=ppi_data;end
   if(|mask_now)begin
    accumulated_mask<=accumulated_mask|mask_now;
    if(!first_valid)begin first_valid<=1;first_hold<=current_context;end
   end
   // The functional parser closes/discards this transaction on the same edge.
   // Its count_error task snapshots first_context BEFORE these NBA clears.
   if(arm_head||head_close||packet_close)begin
    txn_open<=0;candidate_seen<=0;candidate_header<=0;first_valid<=0;first_hold<=0;accumulated_mask<=0;
   end
  end
 end
endmodule
