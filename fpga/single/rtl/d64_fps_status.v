`timescale 1ns/1ps
module d64_fps_status #(
 parameter integer CLK_HZ=100000000,DEFAULT_ON=1,REMOTE_DEBUG_ENABLE=0,
 parameter integer TICK_HZ=12500,TICK_SHIFT=13
)(
 input wire clk_count,rst_count_n,camera_event,publish_event,scan_event,toggle_event,
 input wire clk_pix,rst_pix_n,frame_start,
 input wire remote_set_valid,remote_set_value,output wire enabled_count_state,
 output wire[7:0]fps_bcd_pix,output reg enabled_pix,output reg[7:0]last_bcd_count,
 output reg[15:0]camera_bcd_pix,style_bcd_pix,scan_bcd_pix,output reg[2:0]valid_pix
);
 // TICK_HZ is retained for source compatibility; the exact tick is CLK_HZ/8192.
 // TICK_SHIFT13 is production. Smaller values accelerate verification only,
 // retaining the production numerator and logical timestamp unit.
 localparam integer DW=TICK_SHIFT;
 localparam[63:0]NUMERATOR_WIDE=64'd3200*CLK_HZ/8192;
 localparam[25:0]NUMERATOR=NUMERATOR_WIDE[25:0];
 localparam IDLE=0,DIV=1,BCD_INIT=2,BCD=3,COMMIT=4;
 reg[DW-1:0]tick_div;reg[15:0]tick_count;
 wire tick_step=&tick_div;
 wire quiet_epoch=tick_step&&(&tick_count[13:0]);
 wire[2:0]events={scan_event,publish_event,camera_event};
 reg[2:0]seen_epoch,zero_pending;
 reg anchored,sample_pending;reg[1:0]sample_metric;
 reg[15:0]anchor;reg[5:0]interval_count;
 wire[15:0]span=tick_count-anchor;
 reg enabled_count;assign enabled_count_state=enabled_count;
 assign fps_bcd_pix=style_bcd_pix[15:8];
 reg[2:0]state;reg[1:0]metric;
 reg[13:0]quotient;reg[16:0]remainder;reg[15:0]divisor;reg overflow;
 reg[4:0]bit_count;reg[15:0]bcd_work,bcd_adjust;
 wire[16:0]shifted={remainder[15:0],NUMERATOR[bit_count]};
 wire ge=shifted>={1'b0,divisor};wire[13:0]qnext={quotient[12:0],ge};
 wire[16:0]rnext=ge?(shifted-{1'b0,divisor}):shifted;
 wire round_up=({remainder,1'b0}>={2'b0,divisor});
 wire[15:0]bnext={bcd_adjust[14:0],quotient[13]};
 // Serial per-line mailbox: enabled + metric selector +16BCD. Selector3
 // updates only the enable switch. PIX retains all three lines independently.
 reg[18:0]held_payload;reg request_toggle,ack_toggle;
 (* async_reg="true" *)reg[1:0]request_sync,ack_sync;
 wire mailbox_free=ack_sync[1]==request_toggle;
 wire control_dirty=enabled_count!=held_payload[18];
 integer i,b;
 always @*begin
  bcd_adjust=bcd_work;
  for(b=0;b<4;b=b+1)if(bcd_work[b*4+:4]>=5)bcd_adjust[b*4+:4]=bcd_work[b*4+:4]+4'd3;
 end
 always @(posedge clk_count or negedge rst_count_n)begin
  if(!rst_count_n)begin
   tick_div<=0;tick_count<=0;anchored<=0;sample_pending<=0;sample_metric<=0;seen_epoch<=0;zero_pending<=0;anchor<=0;interval_count<=0;
   state<=IDLE;metric<=0;quotient<=0;remainder<=0;divisor<=0;overflow<=0;bit_count<=0;bcd_work<=0;
   enabled_count<=(DEFAULT_ON!=0);held_payload<={(DEFAULT_ON!=0),2'd3,16'd0};request_toggle<=0;ack_sync<=0;last_bcd_count<=0;
  end else begin
   tick_div<=tick_div+1'b1;if(tick_step)tick_count<=tick_count+1'b1;
   ack_sync<={ack_sync[0],ack_toggle};
   if(REMOTE_DEBUG_ENABLE!=0&&remote_set_valid)enabled_count<=remote_set_value;
   else if(toggle_event)enabled_count<=!enabled_count;
   if(mailbox_free&&control_dirty)begin held_payload<={enabled_count,2'd3,16'd0};request_toggle<=!request_toggle;end
   // One sampler polls stages. It counts32 uninterrupted real event
   // intervals of the selected stage, including pauses inside that window.
   if(events[sample_metric]&&!sample_pending)begin
    if(!anchored||span[15])begin anchored<=1;anchor<=tick_count;interval_count<=0;end
    else if(interval_count==31)begin divisor<=span;sample_pending<=1;anchored<=0;interval_count<=0;end
    else interval_count<=interval_count+1'b1;
   end
   // Silence monitoring remains independent for all three stages, including
   // inactive stages. An inactive selected stage cannot block polling forever.
   for(i=0;i<3;i=i+1)begin
    if(events[i])seen_epoch[i]<=1;
    if(quiet_epoch)begin
     seen_epoch[i]<=events[i];
     if(!seen_epoch[i]&&!events[i])begin
      zero_pending[i]<=1;
      if(sample_metric==i)begin anchored<=0;interval_count<=0;sample_pending<=0;sample_metric<=sample_metric==2?0:sample_metric+1'b1;end
     end
    end
   end
   case(state)
    IDLE:begin
     if(|zero_pending)begin
      if(zero_pending[0])metric<=0;else if(zero_pending[1])metric<=1;else metric<=2;
      bcd_work<=0;state<=COMMIT;
     end else if(sample_pending)begin
      metric<=sample_metric;quotient<=0;sample_pending<=0;
      sample_metric<=sample_metric==2?0:sample_metric+1'b1;anchored<=0;interval_count<=0;
      remainder<=0;overflow<=0;bit_count<=25;state<=DIV;
     end
    end
    DIV:begin
     quotient<=qnext;remainder<=rnext;overflow<=overflow|quotient[13];
     if(bit_count==0)state<=BCD_INIT;else bit_count<=bit_count-1'b1;
    end
    BCD_INIT:begin quotient<=(divisor==0||overflow||quotient>=9999)?14'd9999:(quotient+round_up);bcd_work<=0;bit_count<=0;state<=BCD;end
    BCD:begin
     bcd_work<=bnext;quotient<=quotient<<1;
     if(bit_count==13)state<=COMMIT;else bit_count<=bit_count+1'b1;
    end
    COMMIT:if(mailbox_free&&!control_dirty)begin
     held_payload<={enabled_count,metric,zero_pending[metric]?16'd0:bcd_work};request_toggle<=!request_toggle;
     if(metric==1)last_bcd_count<=zero_pending[1]?8'd0:bcd_work[15:8];
     zero_pending[metric]<=quiet_epoch&&!seen_epoch[metric]&&!events[metric];state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
 always @(posedge clk_pix or negedge rst_pix_n)begin
  if(!rst_pix_n)begin request_sync<=0;ack_toggle<=0;camera_bcd_pix<=0;style_bcd_pix<=0;scan_bcd_pix<=0;valid_pix<=0;enabled_pix<=(DEFAULT_ON!=0);end
  else begin
   request_sync<={request_sync[0],request_toggle};
   if(frame_start&&request_sync[1]!=ack_toggle)begin
    enabled_pix<=held_payload[18];ack_toggle<=request_sync[1];
    case(held_payload[17:16])
     0:begin camera_bcd_pix<=held_payload[15:0];valid_pix[0]<=1;end
     1:begin style_bcd_pix<=held_payload[15:0];valid_pix[1]<=1;end
     2:begin scan_bcd_pix<=held_payload[15:0];valid_pix[2]<=1;end
    endcase
   end
  end
 end
 // synthesis translate_off
 initial if(CLK_HZ<1||TICK_SHIFT<1||TICK_SHIFT>13||(64'd3200*CLK_HZ)%8192||NUMERATOR_WIDE>26'h3ffffff)$fatal(1,"COMPACT_RATE_PARAMETERS");
 always @(posedge clk_count)if(rst_count_n&&events[sample_metric]&&!sample_pending&&anchored&&!span[15]&&interval_count==31&&(state==DIV||state==BCD_INIT))$fatal(1,"SPAN_OVERWROTE_ACTIVE_DIVISOR");
 // synthesis translate_on
endmodule
