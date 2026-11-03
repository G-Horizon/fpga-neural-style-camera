`timescale 1ns/1ps
`include "audio/data/voice_template_constants.vh"
// Small personal template prototype. Templates are generated from real local
// audio; unqualified for other speakers/room noise. Flat byte memories make
// RAM costs explicit. No runtime divide or multiply in temporal normalization.
module d64_voice_matcher #(
    parameter TEMPLATE_FILE="audio/data/voice_combined.mem",
    parameter integer VAD_THRESHOLD=80,HANG_FRAMES=6,MAX_FRAMES=64,
    parameter integer MAX_SCORE=`VOICE_MAX_SCORE,MIN_MARGIN=`VOICE_MIN_MARGIN,
    parameter integer COOLDOWN_FRAMES=31,
    parameter integer LEN0=`VOICE_LEN_0,LEN1=`VOICE_LEN_1,
    parameter integer LEN2=`VOICE_LEN_2,LEN3=`VOICE_LEN_3
)(
    input wire clk,rst_n,abort_event,
    input wire frame_valid,input wire [63:0] features,input wire [15:0] amplitude,
    output reg voice_valid,output reg [1:0] voice_style_id,output reg voice_next,
    output reg [1:0] command_class,output reg rejected,
    output reg [1:0] reject_reason,output reg protocol_fault,
    output reg [15:0] best_score,second_score,
    output wire busy
);
    localparam IDLE=0,WRITE=1,INIT=2,READ=3,DIFF=4,RANK=5,DECIDE=6,READ_TEMPLATE=7,SAVE_TEMPLATE=8;
    reg [3:0] state;
    reg [7:0] phrase_count,last_voiced_len,phrase_len,cooldown;
    reg active,too_long;
    reg [3:0] quiet_count,time_bin;
    reg [2:0] band;
    reg [1:0] class_idx,best_class;
    reg [12:0] running_score;
    reg [15:0] class_score;
    reg [12:0] phase;
    reg [9:0] write_addr;
    reg [63:0] held_features;
    // Bottom 512 bytes: <=64 phrase frames. Top 512: four 16x8 templates.
    // One shared initialized byte RAM, one write and one read port.
    // Read phrase/template serially; two simultaneous read ports duplicate
    // the memory under this toolchain and cost two physical RAM10s.
    reg [7:0] store[0:1023];
    reg [7:0] feature_byte,template_byte;
    reg [7:0] ram_q;
    initial $readmemh(TEMPLATE_FILE,store);
    wire voiced=amplitude>VAD_THRESHOLD;
    wire [7:0] difference=feature_byte>=template_byte ? feature_byte-template_byte : template_byte-feature_byte;
    wire [12:0] score_plus=running_score+difference;
    wire [9:0] feature_address={1'b0,phase[10:5],band};
    wire [9:0] template_address={1'b1,class_idx,time_bin,band};
    assign busy=state!=IDLE;
    function [7:0] train_length;
        input [1:0] c;
        begin case(c)0:train_length=LEN0;1:train_length=LEN1;2:train_length=LEN2;default:train_length=LEN3;endcase end
    endfunction
    wire duration_good=phrase_len>=(train_length(class_idx)>>1)&&phrase_len<=train_length(class_idx)*2;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;phrase_count<=0;last_voiced_len<=0;phrase_len<=0;cooldown<=0;
            active<=0;too_long<=0;quiet_count<=0;time_bin<=0;band<=0;class_idx<=0;best_class<=0;
            running_score<=0;class_score<=0;phase<=0;write_addr<=0;held_features<=0;
            feature_byte<=0;template_byte<=0;voice_valid<=0;voice_style_id<=0;voice_next<=0;
            command_class<=0;rejected<=0;reject_reason<=0;protocol_fault<=0;
            best_score<=16'hffff;second_score<=16'hffff;
        end else if(abort_event)begin
            state<=IDLE;active<=0;phrase_count<=0;last_voiced_len<=0;too_long<=0;quiet_count<=0;
            voice_valid<=0;voice_next<=0;rejected<=0;
        end else begin
            voice_valid<=0;voice_next<=0;rejected<=0;
            if(state==READ||state==READ_TEMPLATE)
                ram_q<=store[state==READ ? feature_address : template_address];
            if(frame_valid)begin
                if(cooldown!=0)cooldown<=cooldown-1'b1;
                if(state!=IDLE)protocol_fault<=1;
            end
            case(state)
                IDLE:if(frame_valid)begin
                    if(active)begin
                        if(phrase_count<MAX_FRAMES)begin
                            write_addr<={phrase_count[6:0],3'd0};held_features<=features;band<=0;state<=WRITE;
                            phrase_count<=phrase_count+1'b1;
                        end else if(voiced)too_long<=1;
                        if(voiced)begin last_voiced_len<=phrase_count+1'b1;quiet_count<=0;end
                        else if(quiet_count==HANG_FRAMES-1)begin
                            active<=0;quiet_count<=0;phrase_len<=last_voiced_len;
                            if(too_long||last_voiced_len<10||last_voiced_len>MAX_FRAMES)begin
                                state<=IDLE;rejected<=1;reject_reason<=1;
                            end else state<=INIT;
                        end else quiet_count<=quiet_count+1'b1;
                    end else if(voiced)begin
                        active<=1;too_long<=0;quiet_count<=0;phrase_count<=1;last_voiced_len<=1;
                        write_addr<=0;held_features<=features;band<=0;state<=WRITE;
                    end
                end
                WRITE:begin
                    store[write_addr+band]<=held_features[band*8+:8];
                    if(band==7)state<=IDLE;else band<=band+1'b1;
                end
                INIT:begin
                    class_idx<=0;time_bin<=0;band<=0;phase<=phrase_len;running_score<=0;
                    best_score<=16'hffff;second_score<=16'hffff;best_class<=0;state<=READ;
                end
                READ:state<=READ_TEMPLATE;
                READ_TEMPLATE:begin feature_byte<=ram_q;state<=SAVE_TEMPLATE;end
                SAVE_TEMPLATE:begin template_byte<=ram_q;state<=DIFF;end
                DIFF:begin
                    if(band==7)begin
                        band<=0;
                        if(time_bin==15)begin class_score<=duration_good?score_plus:16'hffff;state<=RANK;end
                        else begin time_bin<=time_bin+1'b1;phase<=phase+{phrase_len,1'b0};running_score<=score_plus;state<=READ;end
                    end else begin band<=band+1'b1;running_score<=score_plus;state<=READ;end
                end
                RANK:begin
                    if(class_score<best_score)begin second_score<=best_score;best_score<=class_score;best_class<=class_idx;end
                    else if(class_score<second_score)second_score<=class_score;
                    if(class_idx==3)state<=DECIDE;
                    else begin class_idx<=class_idx+1'b1;time_bin<=0;band<=0;phase<=phrase_len;running_score<=0;state<=READ;end
                end
                DECIDE:begin
                    command_class<=best_class;
                    if(best_score<=MAX_SCORE&&{1'b0,second_score}>={1'b0,best_score}+MIN_MARGIN)begin
                        if(cooldown==0)begin
                            voice_valid<=1;voice_next<=best_class==3;
                            voice_style_id<=best_class+1'b1;cooldown<=COOLDOWN_FRAMES;
                        end else begin rejected<=1;reject_reason<=3;end
                    end else begin rejected<=1;reject_reason<=2;end
                    state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
