module ppi_err_observer_ring128 #(
 parameter integer ADDR_BITS=11,PRE=1024,POST=1023,ERR_LANE=3
)(
 input wire byte_clk,byte_rst_n,
 input wire [127:0] sample,
 input wire cfg_complete_async,
 input wire fatal_event_valid,fatal_event_toggle,
 input wire [1023:0] fatal_event_context,
 input wire read_clk,read_rst_n,read_en,
 input wire [ADDR_BITS-1:0] read_addr,
 output reg [127:0] read_data,
 output reg read_valid,
 output reg fatal_seen_read,trigger_seen_read,raw_done_read,
 output reg [1023:0] fatal_context_read,
 output reg [255:0] ring_info_read,
 output wire armed_read,history_ready_read,cfg_ready_read
);
 localparam integer DEPTH=1<<ADDR_BITS;
 (* ram_style="block" *)reg [127:0] samples[0:DEPTH-1];
 reg [ADDR_BITS-1:0] write_ptr;
 reg [ADDR_BITS:0] pre_available,post_remaining;
 reg [31:0] sample_seq;
 reg triggered,raw_done;
 reg [255:0] ring_info_hold;
 (* async_reg="true" *)reg cfg_meta_byte,cfg_sync_byte;
 reg [3:0] previous_err;
 wire [3:0] err_level=sample[51:48];
 wire [3:0] err_rise=err_level&~previous_err;
 wire history_ready=(pre_available>=PRE);
 wire armed=cfg_sync_byte&&history_ready&&!triggered;
 wire accept_err=armed&&err_rise[ERR_LANE];
 wire [ADDR_BITS:0] actual_pre=pre_available;
 wire [ADDR_BITS-1:0] start_ptr=write_ptr-actual_pre[ADDR_BITS-1:0];
 wire [31:0] total_count=actual_pre+1+POST;
 always @(posedge byte_clk or negedge byte_rst_n)begin
  if(!byte_rst_n)begin cfg_meta_byte<=0;cfg_sync_byte<=0;previous_err<=0;end
  else begin
   cfg_meta_byte<=cfg_complete_async;cfg_sync_byte<=cfg_meta_byte;
   previous_err<=err_level;
  end
 end
 always @(posedge byte_clk)if(byte_rst_n&&!raw_done)samples[write_ptr]<=sample;
 always @(posedge byte_clk or negedge byte_rst_n)begin
  if(!byte_rst_n)begin
   write_ptr<=0;pre_available<=0;post_remaining<=0;sample_seq<=0;
   triggered<=0;raw_done<=0;ring_info_hold<=0;
  end else begin
   sample_seq<=sample_seq+1'b1;
   if(!raw_done)begin
    write_ptr<=write_ptr+1'b1;
    if(!triggered)begin
     if(accept_err)begin
      // Cause and ring detection are the same real byte edge E. E is at
      // logical PRE, with exactly PRE earlier samples and POST later samples.
      triggered<=1;post_remaining<=POST;
      if(POST==0)raw_done<=1;
      ring_info_hold<=0;
      ring_info_hold[0*32+:32]<=32'h45545231; // ETR1: Err observation, not fatal.
      ring_info_hold[1*32+:32]<=sample_seq;
      ring_info_hold[2*32+:32]<=sample_seq;
      ring_info_hold[3*32+:32]<=sample_seq-actual_pre;
      ring_info_hold[4*32+:32]<={{(32-2*ADDR_BITS){1'b0}},write_ptr,start_ptr};
      ring_info_hold[5*32+:32]<={{(32-2*(ADDR_BITS+1)){1'b0}},actual_pre,POST[ADDR_BITS:0]};
      ring_info_hold[6*32+:32]<=total_count;
      ring_info_hold[7*32+:32]<={8'd0,err_level,err_rise,8'h01,1'b0,ERR_LANE[1:0],1'b1,cfg_sync_byte,(actual_pre==PRE),1'b1,1'b1};
     end else if(pre_available<PRE)pre_available<=pre_available+1'b1;
    end else if(post_remaining!=0)begin
     post_remaining<=post_remaining-1'b1;
     if(post_remaining==1)raw_done<=1;
    end
   end
  end
 end
 (* async_reg="true" *)reg fatal_meta,fatal_sync,fatal_valid_meta,fatal_valid_sync;
 (* async_reg="true" *)reg trigger_meta,trigger_sync,done_meta,done_sync;
 (* async_reg="true" *)reg armed_meta,armed_sync,history_meta,history_sync,cfg_meta_read,cfg_sync_read;
 reg fatal_consumed;
 reg [ADDR_BITS-1:0] read_start;
 reg [ADDR_BITS:0] read_count;
 assign armed_read=armed_sync;
 assign history_ready_read=history_sync;
 assign cfg_ready_read=cfg_sync_read;
 always @(posedge read_clk or negedge read_rst_n)begin
  if(!read_rst_n)begin
   fatal_meta<=0;fatal_sync<=0;fatal_valid_meta<=0;fatal_valid_sync<=0;
   trigger_meta<=0;trigger_sync<=0;done_meta<=0;done_sync<=0;fatal_consumed<=0;
   armed_meta<=0;armed_sync<=0;history_meta<=0;history_sync<=0;cfg_meta_read<=0;cfg_sync_read<=0;
   fatal_seen_read<=0;trigger_seen_read<=0;raw_done_read<=0;
   fatal_context_read<=0;ring_info_read<=0;read_start<=0;read_count<=0;
  end else begin
   fatal_meta<=fatal_event_toggle;fatal_sync<=fatal_meta;
   fatal_valid_meta<=fatal_event_valid;fatal_valid_sync<=fatal_valid_meta;
   trigger_meta<=triggered;trigger_sync<=trigger_meta;done_meta<=raw_done;done_sync<=done_meta;
   armed_meta<=armed;armed_sync<=armed_meta;
   history_meta<=history_ready;history_sync<=history_meta;
   cfg_meta_read<=cfg_sync_byte;cfg_sync_read<=cfg_meta_read;
   if(fatal_valid_sync&&!fatal_seen_read&&fatal_sync!=fatal_consumed)begin
    fatal_consumed<=fatal_sync;fatal_seen_read<=1;fatal_context_read<=fatal_event_context;
   end
   if(trigger_sync&&!trigger_seen_read)begin
    trigger_seen_read<=1;ring_info_read<=ring_info_hold;
    read_start<=ring_info_hold[4*32+:ADDR_BITS];
    read_count<=ring_info_hold[6*32+:ADDR_BITS+1];
   end
   if(done_sync)raw_done_read<=1;
  end
 end
 wire memory_read=read_en&&raw_done_read&&trigger_seen_read&&{1'b0,read_addr}<read_count;
 wire [ADDR_BITS-1:0] memory_read_address=read_start+read_addr;
 always @(posedge read_clk)begin
  if(memory_read)read_data<=samples[memory_read_address];else read_data<=0;
 end
 always @(posedge read_clk or negedge read_rst_n)
  if(!read_rst_n)read_valid<=0;else read_valid<=memory_read;
 // synthesis translate_off
 initial if(ADDR_BITS<3||ADDR_BITS>11||PRE<1||POST<0||PRE+1+POST!=DEPTH||ERR_LANE<0||ERR_LANE>3)
  $fatal(1,"invalid Err observer geometry/lane");
 // synthesis translate_on
endmodule
