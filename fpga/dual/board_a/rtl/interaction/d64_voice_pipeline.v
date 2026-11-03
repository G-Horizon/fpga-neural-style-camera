`timescale 1ns/1ps
// Feed ONLY contiguous 16 kHz signed PCM16 mono. Abort on transport sequence
// gaps/disconnect; never join fragments across a missing audio packet.
module d64_voice_pipeline #(
    parameter TEMPLATE_FILE="audio/data/voice_combined.mem"
)(
    input wire clk,rst_n,abort_event,pcm_valid,input wire signed [15:0] pcm,
    output wire pcm_ready,voice_valid,output wire [1:0] voice_style_id,
    output wire voice_next,output wire [1:0] command_class,
    output wire rejected,protocol_fault,output wire [1:0] reject_reason,
    output wire [15:0] best_score,second_score
);
    wire frame_valid;wire [63:0] features;wire [15:0] amplitude;wire unused_busy;
    d64_voice_features u_features(.clk(clk),.rst_n(rst_n),.abort_event(abort_event),
        .pcm_valid(pcm_valid),.pcm(pcm),.pcm_ready(pcm_ready),
        .frame_valid(frame_valid),.frame_features(features),.frame_amplitude(amplitude));
    d64_voice_matcher #(.TEMPLATE_FILE(TEMPLATE_FILE)) u_matcher(
        .clk(clk),.rst_n(rst_n),.abort_event(abort_event),.frame_valid(frame_valid),
        .features(features),.amplitude(amplitude),.voice_valid(voice_valid),
        .voice_style_id(voice_style_id),.voice_next(voice_next),.command_class(command_class),
        .rejected(rejected),.reject_reason(reject_reason),.protocol_fault(protocol_fault),
        .best_score(best_score),.second_score(second_score),.busy(unused_busy));
endmodule
