`timescale 1ns/1ps
module d2_a_peripherals #(
 parameter integer CLK_HZ=100000000,BAUD=1000000,PCM_AW=10,
 parameter TEMPLATE_FILE="../../audio/data/voice_combined.mem"
)(
 input wire clk,rst_n,uart_rx,output wire uart_tx,
 input wire [1:0] requested_style,active_style,layout,input wire fps,effect,model_valid,busy,
 input wire [31:0] epoch,capture_fid,computed_fid,transmitted_fid,faults,
 input wire cap_valid,cap_ready,cap_sof,cap_eof,input wire [383:0] cap_rgb,
 input wire [7:0] cap_x0,cap_y,input wire [31:0] cap_fid,input wire source_bad,
 output reg control_valid,output reg [7:0] control_action,control_value,
 output wire voice_valid,voice_next,output wire [1:0] voice_style,
 output wire gesture_trigger,output wire voice_fault,
 output reg [31:0] packet_errors,output wire [31:0] session_id
);
 function [15:0] crc_byte(input [15:0] old,input [7:0] b);
 integer n;reg [15:0] c;begin c=old^{b,8'd0};for(n=0;n<8;n=n+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end endfunction
 wire rx_valid,framing;wire [7:0] rx_data;wire tx_ready;reg tx_valid;reg [7:0] tx_data;
 d2_uart_phy #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_uart(.clk(clk),.rst_n(rst_n),.rx(uart_rx),.rx_valid(rx_valid),.rx_data(rx_data),.framing_error(framing),.tx_valid(tx_valid),.tx_data(tx_data),.tx_ready(tx_ready),.tx(uart_tx));
 reg [7:0] packet[0:127];reg [7:0] ptype,version;reg [31:0] pseq;reg [15:0] plen,crc,received_crc;
 reg [3:0] parser;reg [7:0] index;reg packet_pending;reg [31:0] timeout;
 reg abort_rx;
 localparam integer TIMEOUT=CLK_HZ/100;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin parser<=0;index<=0;ptype<=0;version<=0;pseq<=0;plen<=0;crc<=16'hffff;received_crc<=0;packet_pending<=0;timeout<=0;packet_errors<=0;abort_rx<=0;end
 else begin
 packet_pending<=0;abort_rx<=0;
 if(parser!=0&&!rx_valid&&!framing)begin
 if(timeout==TIMEOUT-1)begin parser<=0;packet_errors<=packet_errors+1;abort_rx<=1;timeout<=0;end
 else timeout<=timeout+1;
 end
 if(framing)begin parser<=0;packet_errors<=packet_errors+1;abort_rx<=1;end
 else if(rx_valid)begin
 timeout<=0;
 case(parser)
 0:if(rx_data==8'ha5)parser<=1;
 1:if(rx_data==8'h5a)begin parser<=2;crc<=16'hffff;end else parser<=rx_data==8'ha5 ? 1 : 0;
 2:begin version<=rx_data;crc<=crc_byte(crc,rx_data);parser<=3;end
 3:begin ptype<=rx_data;crc<=crc_byte(crc,rx_data);index<=0;parser<=4;end
 4:begin pseq[index*8+:8]<=rx_data;crc<=crc_byte(crc,rx_data);if(index==3)begin parser<=5;index<=0;end else index<=index+1;end
 5:begin plen[7:0]<=rx_data;crc<=crc_byte(crc,rx_data);parser<=6;end
 6:begin plen[15:8]<=rx_data;crc<=crc_byte(crc,rx_data);index<=0;
 if(rx_data!=0||plen[7:0]>128||version!=1)begin parser<=0;packet_errors<=packet_errors+1;abort_rx<=1;end
 else parser<=plen[7:0]==0?8:7;end
 7:begin packet[index]<=rx_data;crc<=crc_byte(crc,rx_data);if(index==plen-1)parser<=8;else index<=index+1;end
 8:begin received_crc[7:0]<=rx_data;parser<=9;end
 9:begin parser<=0;received_crc[15:8]<=rx_data;if({rx_data,received_crc[7:0]}==crc)packet_pending<=1;else begin packet_errors<=packet_errors+1;abort_rx<=1;end end
 default:parser<=0;
 endcase
 end
 end end
 wire [31:0] packet_session={packet[3],packet[2],packet[1],packet[0]};
 wire [31:0] packet_first_sample={packet[7],packet[6],packet[5],packet[4]};
 reg [31:0] session,next_audio;reg audio_contiguous;
 assign session_id=session;
 reg command_seen,pcm_seen;reg [31:0] command_seq,pcm_seq;reg [7:0] last_action,last_value,last_result;
 reg [15:0] pcm_crc;reg [31:0] pcm_first;reg [15:0] pcm_length;
 reg signed [15:0] pcm_ram[0:(1<<PCM_AW)-1];reg [PCM_AW-1:0] pcm_wptr,pcm_rptr;
 reg [PCM_AW:0] pcm_level;wire [PCM_AW:0] pcm_free=(1<<PCM_AW)-pcm_level;
 reg emitting;reg [6:0] emit_count,emit_index;reg abort_command;
 wire pcm_ready;reg pcm_pending;wire pcm_pop=pcm_level!=0&&!pcm_pending&&!abort_rx&&!abort_command;
 wire pcm_push=emitting&&!abort_rx&&!abort_command;
 reg signed [15:0] pcm_q;
 wire pcm_abort=abort_rx||abort_command;
 // The voice input accepts a stable head word only when its ready is high.
 always @(posedge clk)if(pcm_pop)pcm_q<=pcm_ram[pcm_rptr];
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)pcm_pending<=0;else if(pcm_abort)pcm_pending<=0;
 else begin if(pcm_pop)pcm_pending<=1;else if(pcm_pending&&pcm_ready)pcm_pending<=0;end end
 d64_voice_pipeline #(.TEMPLATE_FILE(TEMPLATE_FILE)) u_voice(.clk(clk),.rst_n(rst_n),.abort_event(pcm_abort),.pcm_valid(pcm_pending&&!pcm_abort),.pcm(pcm_q),.pcm_ready(pcm_ready),.voice_valid(voice_valid),.voice_style_id(voice_style),.voice_next(voice_next),.command_class(),.rejected(),.protocol_fault(voice_fault),.reject_reason(),.best_score(),.second_score());
 d64_wave_detector u_gesture(.clk(clk),.rst_n(rst_n),.cap_valid(cap_valid),.cap_ready(cap_ready),.cap_rgb(cap_rgb),.cap_x0(cap_x0),.cap_y(cap_y),.cap_fid(cap_fid),.cap_sof(cap_sof),.cap_eof(cap_eof),.source_bad(source_bad),.effect_trigger(gesture_trigger),.primed(),.frame_good());
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin pcm_wptr<=0;pcm_rptr<=0;pcm_level<=0;end
 else if(pcm_abort)begin pcm_wptr<=0;pcm_rptr<=0;pcm_level<=0;end
 else begin
 if(pcm_push)begin pcm_ram[pcm_wptr]<={packet[9+emit_index*2],packet[8+emit_index*2]};pcm_wptr<=pcm_wptr+1;end
 if(pcm_pop)pcm_rptr<=pcm_rptr+1;
 case({pcm_push,pcm_pop})2'b10:pcm_level<=pcm_level+1;2'b01:pcm_level<=pcm_level-1;default:begin end endcase
 end end
 reg [2:0] response_delay;reg response_push;reg [359:0] response_word;reg [359:0] responses[0:3];reg [1:0] response_wptr,response_rptr;reg [2:0] response_count;
 reg sending;reg [5:0] tx_index;reg [15:0] tx_crc;reg [359:0] response_hold;
 wire response_pop=!sending&&response_count!=0;
 function [319:0] make_status(input [7:0] result);
 reg [319:0] s;begin s=0;s[31:0]=session;s[39:32]=result;s[47:40]={6'd0,requested_style};s[55:48]=model_valid?{6'd0,active_style}:8'hff;s[63:56]={6'd0,layout};
 s[71:64]={7'd0,fps};s[79:72]={7'd0,effect};s[87:80]={7'd0,model_valid};s[95:88]={7'd0,busy};s[127:96]=epoch;s[159:128]=capture_fid;s[191:160]=computed_fid;s[223:192]=transmitted_fid;s[255:224]=next_audio;s[287:256]=faults;s[303:288]=pcm_free;s[319:304]=16'h0007;make_status=s;end endfunction
 task respond(input [7:0] result);begin response_push<=1;response_word<={pseq,ptype|8'h80,make_status(result)};end endtask
 wire legal_control=(packet[4]==1&&packet[5]<3)||(packet[4]==2&&packet[5]==0)||(packet[4]==3&&packet[5]<3)||(packet[4]==4&&packet[5]==0)||(packet[4]==5&&packet[5]<2)||(packet[4]==6&&packet[5]==1);
 wire [15:0] sample_count=(plen-8)>>1;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin session<=0;next_audio<=0;audio_contiguous<=0;command_seen<=0;command_seq<=0;last_action<=0;last_value<=0;last_result<=0;pcm_seen<=0;pcm_seq<=0;pcm_crc<=0;pcm_first<=0;pcm_length<=0;emitting<=0;emit_count<=0;emit_index<=0;abort_command<=0;response_delay<=0;response_push<=0;response_word<=0;control_valid<=0;control_action<=0;control_value<=0;end
 else begin
 response_push<=0;control_valid<=0;abort_command<=0;
 if(response_delay!=0)begin response_delay<=response_delay-1;if(response_delay==1)respond(0);end
 if(abort_rx)begin audio_contiguous<=0;emitting<=0;end
 if(emitting&&!abort_rx)begin
 if(emit_index==emit_count-1)begin emitting<=0;respond(0);end else emit_index<=emit_index+1;
 end
 if(packet_pending)begin
 if(response_count>=4||emitting)begin abort_command<=1;audio_contiguous<=0;end
 else if(response_count==3)respond(3);
 else if(ptype==0)begin
 if(plen!=4||packet_session==0)respond(1);
 else begin
 if(session!=packet_session)begin session<=packet_session;command_seen<=0;pcm_seen<=0;audio_contiguous<=0;next_audio<=0;abort_command<=1;end
 // HELLO response must carry the new session in its actual payload.
 response_push<=1;response_word<={pseq,8'h80,make_status(0)};response_word[31:0]<=packet_session;
 end end
 else if(session==0||packet_session!=session)respond(2);
 else case(ptype)
 1:begin
 if(plen!=6||!legal_control||pseq==0)respond(1);
 else if(command_seen&&pseq==command_seq)begin if(packet[4]==last_action&&packet[5]==last_value)respond(last_result);else respond(1);end
 else if(command_seen&&$signed(pseq-command_seq)<=0)respond(1);
 else begin control_valid<=1;control_action<=packet[4];control_value<=packet[5];command_seen<=1;command_seq<=pseq;last_action<=packet[4];last_value<=packet[5];last_result<=0;response_delay<=4;end
 end
 2:if(plen==4)respond(0);else respond(1);
 3:begin
 if(plen<10||plen>128||plen[0])begin abort_command<=1;audio_contiguous<=0;respond(1);end
 else if(pcm_seen&&pseq==pcm_seq)begin
 if(received_crc==pcm_crc&&packet_first_sample==pcm_first&&plen==pcm_length)respond(0);
 else begin abort_command<=1;audio_contiguous<=0;respond(5);end
 end
 else if((pcm_seen&&$signed(pseq-pcm_seq)<=0)||(audio_contiguous&&packet_first_sample!=next_audio)||pcm_free<sample_count)begin abort_command<=1;audio_contiguous<=0;respond(5);end
 else begin emitting<=1;emit_index<=0;emit_count<=sample_count;next_audio<=packet_first_sample+sample_count;audio_contiguous<=1;pcm_seen<=1;pcm_seq<=pseq;pcm_crc<=received_crc;pcm_first<=packet_first_sample;pcm_length<=plen;end
 end
 4:if(plen==8)begin abort_command<=1;audio_contiguous<=0;next_audio<=next_audio+packet_first_sample;respond(0);end else respond(1);
 default:respond(1);
 endcase
 end
 end end
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 response_wptr<=0;response_rptr<=0;response_count<=0;
 // Four shallow response entries deliberately use registers, preserving scarce line/weight RAMs.
 for(integer ri=0;ri<4;ri=ri+1)responses[ri]<=0;
 end
 else begin
 if(response_push)begin responses[response_wptr]<=response_word;response_wptr<=response_wptr+1;end
 if(response_pop)response_rptr<=response_rptr+1;
 case({response_push,response_pop})2'b10:response_count<=response_count+1;2'b01:response_count<=response_count-1;default:begin end endcase
 end end
 reg [7:0] current_byte;
 always @*begin
 case(tx_index)
 0:current_byte=8'ha5;1:current_byte=8'h5a;2:current_byte=1;3:current_byte=response_hold[327:320];
 4,5,6,7:current_byte=response_hold[328+(tx_index-4)*8+:8];
 8:current_byte=40;9:current_byte=0;50:current_byte=tx_crc[7:0];51:current_byte=tx_crc[15:8];
 default:current_byte=response_hold[(tx_index-10)*8+:8];
 endcase
 tx_valid=sending;tx_data=current_byte;
 end
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin sending<=0;tx_index<=0;tx_crc<=16'hffff;response_hold<=0;end
 else begin
 if(response_pop)begin response_hold<=responses[response_rptr];sending<=1;tx_index<=0;tx_crc<=16'hffff;end
 if(sending&&tx_ready)begin
 if(tx_index>=2&&tx_index<50)tx_crc<=crc_byte(tx_crc,current_byte);
 if(tx_index==51)sending<=0;else tx_index<=tx_index+1;
 end end end
endmodule
