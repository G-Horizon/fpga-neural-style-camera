`timescale 1ns/1ps
// Cross-window last-event anchored rates, rounded to 0.01/s. Inputs belong to APP.
// Three statistics share a shift/add multiplier, restoring divider and BCD converter.
module d64_stage_rate_status #(
 parameter integer CLK_HZ=100000000,WINDOW_CYCLES=200000000,
 parameter integer DEFAULT_ON=1,REMOTE_DEBUG_ENABLE=0
)(
 input wire clk_count,rst_count_n,
 input wire[31:0] camera_complete_count,style_new_count,
 input wire scan_event,toggle_event,remote_set_valid,remote_set_value,
 input wire clk_pix,rst_pix_n,frame_start,
 output wire enabled_count_state,
 output reg[19:0] camera_bcd_pix,style_bcd_pix,scan_bcd_pix,
 output reg[2:0] valid_pix,output reg enabled_pix,
 output reg[19:0] last_camera_bcd_count,last_style_bcd_count,last_scan_bcd_count,
 output reg[16:0] camera_rate_x100,style_rate_x100,scan_rate_x100,
 output reg[2:0] valid_count,output reg snapshot_done
);
 localparam integer CW=(WINDOW_CYCLES>1)?$clog2(WINDOW_CYCLES):1;
 localparam[63:0] RATE_SCALE=64'd100*CLK_HZ;
 localparam IDLE=0,LOAD=1,MUL=2,ROUND=3,DIV=4,BCD_INIT=5,BCD=6,NEXT=7,COMMIT=8;
 reg[CW-1:0] window_count;
 reg[31:0] previous_camera,previous_style;
 wire[31:0] delta[0:2];
 assign delta[0]=camera_complete_count-previous_camera;
 assign delta[1]=style_new_count-previous_style;
 assign delta[2]=scan_event?32'd1:32'd0;
 reg[63:0] absolute_tick;
 reg[2:0] seen,bad_window,anchor_valid;
 reg[63:0] first_tick[0:2],last_tick[0:2];reg[31:0]intervals[0:2];
 wire[63:0] end_tick[0:2],elapsed[0:2];
 genvar g;generate for(g=0;g<3;g=g+1)begin:g_endpoint
  assign end_tick[g]=delta[g]!=0?absolute_tick:last_tick[g];
  assign elapsed[g]=end_tick[g]-first_tick[g];
 end endgenerate
 reg[31:0] sample_intervals[0:2],sample_ticks[0:2];reg[2:0] sample_valid;
 reg[3:0] state;reg[1:0] metric;
 reg[31:0] multiplier,divisor;
 reg[63:0] mul_sum,mul_term,quotient;
 reg[32:0] remainder;
 reg[6:0] bit_count;
 wire[63:0] sum_next=mul_sum+(multiplier[0]?mul_term:64'd0);
 wire[32:0] rem_shift={remainder[31:0],quotient[63]};
 wire div_ge=rem_shift>={1'b0,divisor};
 wire[63:0] quotient_next={quotient[62:0],div_ge};
 reg[16:0] binary_value,binary_shift;
 reg[19:0] bcd_work,bcd_adjust;
 wire[19:0] bcd_next={bcd_adjust[18:0],binary_shift[16]};
 reg[19:0] result_bcd[0:2];reg[16:0] result_raw[0:2];
 reg enabled_count;
 assign enabled_count_state=enabled_count;
 reg[63:0] held_payload;
 reg request_toggle,ack_toggle;
 (* async_reg="true" *)reg[1:0] request_sync,ack_sync;
 wire[63:0] live_payload={enabled_count,valid_count,last_camera_bcd_count,last_style_bcd_count,last_scan_bcd_count};
 integer i,b;
 always @*begin
  bcd_adjust=bcd_work;
  for(b=0;b<5;b=b+1)if(bcd_work[b*4+:4]>=5)bcd_adjust[b*4+:4]=bcd_work[b*4+:4]+4'd3;
 end
 always @(posedge clk_count or negedge rst_count_n)begin
  if(!rst_count_n)begin
   window_count<=0;absolute_tick<=0;previous_camera<=0;previous_style<=0;seen<=0;bad_window<=0;anchor_valid<=0;sample_valid<=0;
   state<=IDLE;metric<=0;multiplier<=0;divisor<=1;mul_sum<=0;mul_term<=0;quotient<=0;remainder<=0;
   bit_count<=0;binary_value<=0;binary_shift<=0;bcd_work<=0;
   last_camera_bcd_count<=0;last_style_bcd_count<=0;last_scan_bcd_count<=0;
   camera_rate_x100<=0;style_rate_x100<=0;scan_rate_x100<=0;valid_count<=0;snapshot_done<=0;
   enabled_count<=(DEFAULT_ON!=0);held_payload<={(DEFAULT_ON!=0),63'd0};request_toggle<=0;ack_sync<=0;
   for(i=0;i<3;i=i+1)begin first_tick[i]<=0;last_tick[i]<=0;intervals[i]<=0;sample_intervals[i]<=0;sample_ticks[i]<=0;result_bcd[i]<=0;result_raw[i]<=0;end
  end else begin
   previous_camera<=camera_complete_count;previous_style<=style_new_count;absolute_tick<=absolute_tick+1'b1;
   ack_sync<={ack_sync[0],ack_toggle};snapshot_done<=0;
   if(REMOTE_DEBUG_ENABLE!=0&&remote_set_valid)enabled_count<=remote_set_value;
   else if(toggle_event)enabled_count<=!enabled_count;
   if(ack_sync[1]==request_toggle&&held_payload!=live_payload)begin held_payload<=live_payload;request_toggle<=!request_toggle;end
   if(window_count==WINDOW_CYCLES-1)begin
    window_count<=0;seen<=0;bad_window<=0;
    for(i=0;i<3;i=i+1)begin
     // A counter rollback/reset is not interpreted as billions of frames.
     sample_valid[i]<=!bad_window[i]&&delta[i]<=65535&&(intervals[i]+delta[i])<=1000000000&&
       (!(seen[i]||delta[i]!=0)||((anchor_valid[i]||seen[i])&&elapsed[i]!=0&&elapsed[i][63:32]==0));
     sample_intervals[i]<=(anchor_valid[i]||seen[i])?(intervals[i]+delta[i]):32'd0;
     sample_ticks[i]<=(seen[i]||delta[i]!=0)?elapsed[i][31:0]:32'd1;
     intervals[i]<=0;
     // Carry the endpoint across gates: a pause is included when frames resume.
     // Empty windows show zero and retain this anchor, rather than hiding gaps.
     if(seen[i]||delta[i]!=0)begin first_tick[i]<=end_tick[i];last_tick[i]<=end_tick[i];anchor_valid[i]<=1;end
    end
    // Production 2s window greatly exceeds the <360-cycle arithmetic pass.
    if(state==IDLE)begin metric<=0;state<=LOAD;end
   end else begin
    window_count<=window_count+1'b1;
    for(i=0;i<3;i=i+1)if(delta[i]!=0)begin
     if(delta[i]>65535)bad_window[i]<=1;
     seen[i]<=1;
     if(!anchor_valid[i]&&!seen[i])begin first_tick[i]<=absolute_tick;intervals[i]<=0;end
     else intervals[i]<=intervals[i]+delta[i];
     last_tick[i]<=absolute_tick;
    end
   end
   case(state)
    IDLE:begin end
    LOAD:begin
     if(!sample_valid[metric]||sample_intervals[metric]==0)begin
      result_bcd[metric]<=0;result_raw[metric]<=0;state<=NEXT;
     end else begin
      multiplier<=sample_intervals[metric];divisor<=sample_ticks[metric];mul_sum<=0;mul_term<=RATE_SCALE;bit_count<=0;state<=MUL;
     end
    end
    MUL:begin
     mul_sum<=sum_next;multiplier<=multiplier>>1;mul_term<=mul_term<<1;
     if(bit_count==31)state<=ROUND;else bit_count<=bit_count+1'b1;
    end
    ROUND:begin quotient<=mul_sum+{33'd0,divisor[31:1]};remainder<=0;bit_count<=0;state<=DIV;end
    DIV:begin
     quotient<=quotient_next;remainder<=div_ge?(rem_shift-{1'b0,divisor}):rem_shift;
     if(bit_count==63)begin binary_value<=quotient_next>99999?17'd99999:quotient_next[16:0];state<=BCD_INIT;end
     else bit_count<=bit_count+1'b1;
    end
    BCD_INIT:begin binary_shift<=binary_value;bcd_work<=0;bit_count<=0;state<=BCD;end
    BCD:begin
     bcd_work<=bcd_next;binary_shift<=binary_shift<<1;
     if(bit_count==16)begin result_bcd[metric]<=bcd_next;result_raw[metric]<=binary_value;state<=NEXT;end
     else bit_count<=bit_count+1'b1;
    end
    NEXT:if(metric==2)state<=COMMIT;else begin metric<=metric+1'b1;state<=LOAD;end
    COMMIT:begin
     last_camera_bcd_count<=result_bcd[0];last_style_bcd_count<=result_bcd[1];last_scan_bcd_count<=result_bcd[2];
     camera_rate_x100<=result_raw[0];style_rate_x100<=result_raw[1];scan_rate_x100<=result_raw[2];
     valid_count<=sample_valid;snapshot_done<=1;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
 // Source holds the whole tuple until the frame-boundary consumer acknowledges.
 always @(posedge clk_pix or negedge rst_pix_n)begin
  if(!rst_pix_n)begin request_sync<=0;ack_toggle<=0;camera_bcd_pix<=0;style_bcd_pix<=0;scan_bcd_pix<=0;valid_pix<=0;enabled_pix<=(DEFAULT_ON!=0);end
  else begin
   request_sync<={request_sync[0],request_toggle};
   if(frame_start&&request_sync[1]!=ack_toggle)begin
    {enabled_pix,valid_pix,camera_bcd_pix,style_bcd_pix,scan_bcd_pix}<=held_payload;ack_toggle<=request_sync[1];
   end
  end
 end
 // synthesis translate_off
 initial if(CLK_HZ<1||WINDOW_CYCLES<1024||WINDOW_CYCLES>32'h7fffffff)$fatal(1,"STAGE_RATE_PARAMETERS");
 // synthesis translate_on
endmodule
