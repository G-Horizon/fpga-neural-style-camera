`timescale 1ns/1ps
// Active-low keys: KEY0 next style, KEY1 FPS, KEY2 comparison, KEY3 effect.
// Events pulse once after a debounced press; holding never repeats.
// target_style advances independently of loader completion.
module d64_ui_keys #(
    parameter integer DEBOUNCE_CYCLES=1000000
)(
    input wire clk,rst_n,input wire [3:0] keys_n,
    input wire [1:0] active_style,input wire model_valid,
    // External selectors share the desired state and win over simultaneous KEY0.
    input wire external_style_valid,external_next,input wire [1:0] external_style,
    output reg request_valid,output reg [1:0] request_style,
    output reg fps_toggle,compare_toggle,effect_trigger,
    output reg [1:0] target_style,output reg target_pending
);
    localparam integer DBW=(DEBOUNCE_CYCLES>1)?$clog2(DEBOUNCE_CYCLES):1;
    (* async_reg="true" *) reg [3:0] sync0,sync1;
    reg [3:0] stable;
    reg [DBW-1:0] debounce_count;
    reg [3:0] candidate;
    wire [3:0] pressed;
    wire vector_ready=sync1==candidate&&candidate!=stable&&debounce_count==DEBOUNCE_CYCLES-1;
    assign pressed=vector_ready?(stable&~candidate):4'd0;
    wire external_allowed=external_style_valid && (external_next || external_style!=0);
    wire [1:0] cycle_base=(!target_pending && model_valid && active_style!=0)?
        active_style:target_style;
    function [1:0] next_style;
        input [1:0] current;
        begin case(current)
            2'd1:next_style=2'd2;
            2'd2:next_style=2'd3;
            default:next_style=2'd1;
        endcase end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            sync0<=4'hf;sync1<=4'hf;stable<=4'hf;candidate<=4'hf;
            request_valid<=0;request_style<=1;target_style<=1;target_pending<=0;
            fps_toggle<=0;compare_toggle<=0;effect_trigger<=0;
            debounce_count<=0;
        end else begin
            sync0<=keys_n;sync1<=sync0;
            request_valid<=0;fps_toggle<=0;compare_toggle<=0;effect_trigger<=0;
            // One quiet interval for the entire synchronized key vector.
            if(sync1!=candidate)begin candidate<=sync1;debounce_count<=0;end
            else if(candidate==stable)debounce_count<=0;
            else if(debounce_count==DEBOUNCE_CYCLES-1)begin stable<=candidate;debounce_count<=0;end
            else debounce_count<=debounce_count+1'b1;
            // Retain the desired target through intermediate load completions.
            if(target_pending && model_valid && active_style==target_style)
                target_pending<=0;
            if(!target_pending && model_valid && active_style!=0)
                target_style<=active_style;
            if(external_allowed)begin
                target_style<=external_next?next_style(cycle_base):external_style;
                target_pending<=1;
                request_style<=external_next?next_style(cycle_base):external_style;
                request_valid<=1;
            end else if(pressed[0])begin
                target_style<=next_style(cycle_base);target_pending<=1;
                request_style<=next_style(cycle_base);request_valid<=1;
            end
            if(pressed[1])fps_toggle<=1;
            if(pressed[2])compare_toggle<=1;
            if(pressed[3])effect_trigger<=1;
        end
    end
    // synthesis translate_off
    initial if(DEBOUNCE_CYCLES<1)$fatal(1,"DEBOUNCE_CYCLES must be positive");
    // synthesis translate_on
endmodule
