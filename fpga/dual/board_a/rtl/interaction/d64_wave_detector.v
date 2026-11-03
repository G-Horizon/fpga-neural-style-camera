`timescale 1ns/1ps
// Fixed-ROI, near-field bright-foreground left/right/left motion prototype.
// This is not a general hand/gesture classifier. It only observes accepted
// CNN capture packets and never drives ready or changes CNN backpressure.
module d64_wave_detector #(
    parameter integer PIXEL_DIFF=24,
    parameter integer ROI_MIN=6,
    parameter integer GLOBAL_MAX=48,
    parameter integer SEQUENCE_FRAMES=12,
    parameter integer COOLDOWN_FRAMES=60,
    parameter integer BRIGHT_FOREGROUND=1
) (
    input wire clk,input wire rst_n,
    input wire cap_valid,input wire cap_ready,
    input wire [383:0] cap_rgb,
    input wire [7:0] cap_x0,cap_y,
    input wire [31:0] cap_fid,
    input wire cap_sof,cap_eof,input wire source_bad,
    output reg effect_trigger,
    output reg primed,output reg frame_good
);
    localparam [2:0] NONE=0,LEFT=1,RIGHT=2,LEFT_RIGHT=3,RIGHT_LEFT=4;
    reg active,bad,compare_frame,have_fid;
    reg [31:0] frame_fid,last_fid;
    reg [7:0] expected_x,expected_y,samples,global_motion;
    reg [4:0] left_enter,right_enter;
    reg [2:0] motion_state;
    reg [3:0] age;
    reg [5:0] cooldown;
    wire fire=cap_valid&&cap_ready;
    wire coords_ok=cap_x0==expected_x&&cap_y==expected_y&&cap_y<144;
    wire end_coords=cap_x0==240&&cap_y==143;
    wire [31:0] fid_delta=cap_fid-last_fid;
    wire forward_fid=!have_fid||(fid_delta!=0&&!fid_delta[31]);
    wire start_ok=cap_x0==0&&cap_y==0&&!cap_eof&&forward_fid;
    wire compare_start=primed&&!active&&have_fid&&fid_delta==32'd1;
    wire packet_ok=active&&!bad&&coords_ok&&cap_fid==frame_fid;
    wire sample_ce=fire&&!cap_sof&&!source_bad&&packet_ok&&cap_y[3:0]==8;
    wire [7:0] gray=cap_rgb[207:200]; // lane 8, G byte: x0+8, y=16*r+8
    wire [7:0] q7 [0:17];
    wire [7:0] q [0:17];
    wire [7:0] previous=q[17];
    genvar stage,bitno;
    generate for(stage=0;stage<18;stage=stage+1)begin:g_stage
        for(bitno=0;bitno<8;bitno=bitno+1)begin:g_bit
            wire encoded;
            if(stage==0)assign encoded=~gray[bitno];
            else assign encoded=q7[stage-1][bitno];
            // Q7 is the noninverting physical cascade only; fixed A=0 Q
            // decodes the oldest stored inverted byte at the final stage.
            // No reset fanout/FF replacement of this 144-byte history.
            EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
              .D(encoded),.A(3'd0),.CLK(clk),.CE(sample_ce),.Q(q[stage][bitno]),.Q7(q7[stage][bitno]));
        end
    end endgenerate
    wire [8:0] delta=gray>=previous?{1'b0,gray}-{1'b0,previous}:{1'b0,previous}-{1'b0,gray};
    wire moving=delta>=PIXEL_DIFF;
    // Arrival polarity distinguishes entering a ROI from the old ROI clearing
    // when a hand moves between regions. Dark foreground can be configured.
    wire entering=moving&&(BRIGHT_FOREGROUND?(gray>previous):(gray<previous));
    wire roi_y=cap_y[7:4]>=2&&cap_y[7:4]<=6;
    wire roi_l=roi_y&&cap_x0[7:4]>=4&&cap_x0[7:4]<=7;
    wire roi_r=roi_y&&cap_x0[7:4]>=8&&cap_x0[7:4]<=11;
    wire tag_l=left_enter>=ROI_MIN&&right_enter<ROI_MIN;
    wire tag_r=right_enter>=ROI_MIN&&left_enter<ROI_MIN;
    wire ambiguous=left_enter>=ROI_MIN&&right_enter>=ROI_MIN;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            active<=0;bad<=0;compare_frame<=0;have_fid<=0;
            frame_fid<=0;last_fid<=0;expected_x<=0;expected_y<=0;
            samples<=0;global_motion<=0;left_enter<=0;right_enter<=0;
            motion_state<=NONE;age<=0;cooldown<=0;effect_trigger<=0;primed<=0;frame_good<=0;
        end else begin
            effect_trigger<=0;frame_good<=0;
            if(source_bad)begin
                active<=0;bad<=1;compare_frame<=0;primed<=0;motion_state<=NONE;age<=0;
            end else if(fire)begin
                if(cap_sof)begin
                    active<=1;bad<=!start_ok;frame_fid<=cap_fid;
                    expected_x<=16;expected_y<=0;
                    samples<=0;global_motion<=0;left_enter<=0;right_enter<=0;
                    compare_frame<=start_ok&&compare_start;
                    if(!start_ok||!compare_start)begin primed<=0;motion_state<=NONE;age<=0;end
                end else if(!active)begin
                    primed<=0;compare_frame<=0;motion_state<=NONE;age<=0;
                end else begin
                    if(!packet_ok||(cap_eof&&!end_coords)||(!cap_eof&&end_coords))begin
                        bad<=1;primed<=0;compare_frame<=0;motion_state<=NONE;age<=0;
                    end
                    if(expected_x==240)begin expected_x<=0;expected_y<=expected_y+1'b1;end
                    else expected_x<=expected_x+8'd16;
                    if(sample_ce)begin
                        samples<=samples+1'b1;
                        if(moving)global_motion<=global_motion+1'b1;
                        if(entering&&roi_l)left_enter<=left_enter+1'b1;
                        if(entering&&roi_r)right_enter<=right_enter+1'b1;
                    end
                    if(cap_eof)begin
                        active<=0;
                        if(packet_ok&&end_coords&&samples==144)begin
                            frame_good<=1;primed<=1;have_fid<=1;last_fid<=frame_fid;
                            if(cooldown!=0)begin cooldown<=cooldown-1'b1;motion_state<=NONE;age<=0;end
                            else if(!compare_frame||global_motion>=GLOBAL_MAX||ambiguous)begin motion_state<=NONE;age<=0;end
                            else if(motion_state!=NONE&&age>=SEQUENCE_FRAMES-1)begin motion_state<=NONE;age<=0;end
                            else begin
                                if(motion_state!=NONE)age<=age+1'b1;
                                if(tag_l)begin
                                    case(motion_state)
                                        NONE:begin motion_state<=LEFT;age<=0;end
                                        RIGHT:motion_state<=RIGHT_LEFT;
                                        LEFT_RIGHT:begin effect_trigger<=1;cooldown<=COOLDOWN_FRAMES;motion_state<=NONE;age<=0;end
                                        default:begin end
                                    endcase
                                end else if(tag_r)begin
                                    case(motion_state)
                                        NONE:begin motion_state<=RIGHT;age<=0;end
                                        LEFT:motion_state<=LEFT_RIGHT;
                                        RIGHT_LEFT:begin effect_trigger<=1;cooldown<=COOLDOWN_FRAMES;motion_state<=NONE;age<=0;end
                                        default:begin end
                                    endcase
                                end
                            end
                        end else begin primed<=0;compare_frame<=0;motion_state<=NONE;age<=0;end
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(ROI_MIN<1||ROI_MIN>20||GLOBAL_MAX<=40||GLOBAL_MAX>144||SEQUENCE_FRAMES<3||SEQUENCE_FRAMES>16||COOLDOWN_FRAMES<1||COOLDOWN_FRAMES>63)
            $fatal(1,"invalid fixed-ROI detector thresholds");
    end
    // synthesis translate_on
endmodule

