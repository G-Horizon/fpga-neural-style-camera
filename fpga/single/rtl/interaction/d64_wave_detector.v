`timescale 1ns/1ps
// Fixed-position, near-field open-palm -> closed-fist shape approximation.
// First complete good frame after reset/loss must show the empty scene.
// No hand CNN, finger segmentation, ready generation, or capture backpressure.
module d64_wave_detector #(
    parameter integer PIXEL_DIFF=24,
    parameter integer ROI_MIN=6,
    parameter integer GLOBAL_MAX=48,
    parameter integer SEQUENCE_FRAMES=24,
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
    localparam [1:0] IDLE=0,OPEN_ONE=1,ARMED=2,FIST_ONE=3;
    reg active,bad,compare_frame,have_fid;
    reg [31:0] frame_fid,last_fid;
    reg [7:0] expected_x,expected_y,samples,global_motion;
    reg [5:0] roi_count;
    reg [3:0] core_count;
    reg wing_l,wing_r,top_extent,bottom_extent;
    reg [1:0] motion_state;
    reg [4:0] age;
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
    wire [7:0] gray=cap_rgb[207:200]; // x=16*c+8, y=16*r+8
    wire [7:0] q7 [0:17];
    wire [7:0] q [0:17];
    wire [7:0] background=q[17];
    // Loading a complete empty frame writes 144 samples. Afterwards every
    // sample shifts the oldest byte back into the ring: reference freezes
    // without a 144-byte FF mux or another BRAM. A bad/partial frame unprimes
    // the ring, so the next complete good frame overwrites stale samples.
    wire [7:0] reference_in=compare_frame?background:gray;
    genvar stage,bitno;
    generate for(stage=0;stage<18;stage=stage+1)begin:g_stage
        for(bitno=0;bitno<8;bitno=bitno+1)begin:g_bit
            wire encoded;
            if(stage==0)assign encoded=~reference_in[bitno];
            else assign encoded=q7[stage-1][bitno];
            EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
              .D(encoded),.A(3'd0),.CLK(clk),.CE(sample_ce),.Q(q[stage][bitno]),.Q7(q7[stage][bitno]));
        end
    end endgenerate
    wire [8:0] delta=gray>=background?{1'b0,gray}-{1'b0,background}:{1'b0,background}-{1'b0,gray};
    wire moving=delta>=PIXEL_DIFF;
    wire foreground=moving&&(BRIGHT_FOREGROUND?(gray>background):(gray<background));
    wire roi_y=cap_y[7:4]>=2&&cap_y[7:4]<=6;
    wire roi=roi_y&&cap_x0[7:4]>=4&&cap_x0[7:4]<=11;
    wire core=cap_y[7:4]>=3&&cap_y[7:4]<=5&&cap_x0[7:4]>=6&&cap_x0[7:4]<=9;
    // Open = occupied core plus left/right/top/bottom expansion. Fist = an
    // occupied core with smaller silhouette and no extremal side/top cells.
    // Frozen background also recognizes a still hand, not only frame motion.
    wire tag_open=roi_count>=ROI_MIN*3+2&&core_count>=ROI_MIN+2&&
        wing_l&&wing_r&&top_extent&&bottom_extent;
    wire tag_fist=roi_count>=ROI_MIN+2&&roi_count<=ROI_MIN*3&&
        core_count>=ROI_MIN+2&&!wing_l&&!wing_r&&!top_extent;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            active<=0;bad<=0;compare_frame<=0;have_fid<=0;
            frame_fid<=0;last_fid<=0;expected_x<=0;expected_y<=0;
            samples<=0;global_motion<=0;roi_count<=0;core_count<=0;
            wing_l<=0;wing_r<=0;top_extent<=0;bottom_extent<=0;
            motion_state<=IDLE;age<=0;cooldown<=0;effect_trigger<=0;primed<=0;frame_good<=0;
        end else begin
            effect_trigger<=0;frame_good<=0;
            if(source_bad)begin
                active<=0;bad<=1;compare_frame<=0;primed<=0;motion_state<=IDLE;age<=0;
            end else if(fire)begin
                if(cap_sof)begin
                    active<=1;bad<=!start_ok;frame_fid<=cap_fid;
                    expected_x<=16;expected_y<=0;
                    samples<=0;global_motion<=0;roi_count<=0;core_count<=0;
                    wing_l<=0;wing_r<=0;top_extent<=0;bottom_extent<=0;
                    compare_frame<=start_ok&&compare_start;
                    if(!start_ok||!compare_start)begin primed<=0;motion_state<=IDLE;age<=0;end
                end else if(!active)begin
                    primed<=0;compare_frame<=0;motion_state<=IDLE;age<=0;
                end else begin
                    if(!packet_ok||(cap_eof&&!end_coords)||(!cap_eof&&end_coords))begin
                        bad<=1;primed<=0;compare_frame<=0;motion_state<=IDLE;age<=0;
                    end
                    if(expected_x==240)begin expected_x<=0;expected_y<=expected_y+1'b1;end
                    else expected_x<=expected_x+8'd16;
                    if(sample_ce)begin
                        samples<=samples+1'b1;
                        if(moving)global_motion<=global_motion+1'b1;
                        if(foreground&&roi)begin
                            roi_count<=roi_count+1'b1;
                            if(core)core_count<=core_count+1'b1;
                            if(cap_x0[7:4]<=5)wing_l<=1;
                            if(cap_x0[7:4]>=10)wing_r<=1;
                            if(cap_y[7:4]==2)top_extent<=1;
                            if(cap_y[7:4]==6)bottom_extent<=1;
                        end
                    end
                    if(cap_eof)begin
                        active<=0;
                        if(packet_ok&&end_coords&&samples==144)begin
                            frame_good<=1;primed<=1;have_fid<=1;last_fid<=frame_fid;
                            if(cooldown!=0)begin cooldown<=cooldown-1'b1;motion_state<=IDLE;age<=0;end
                            else if(!compare_frame||global_motion>=GLOBAL_MAX)begin motion_state<=IDLE;age<=0;end
                            else if(motion_state!=IDLE&&age>=SEQUENCE_FRAMES-1)begin motion_state<=IDLE;age<=0;end
                            else begin
                                if(motion_state!=IDLE)age<=age+1'b1;
                                case(motion_state)
                                    IDLE:if(tag_open)begin motion_state<=OPEN_ONE;age<=0;end
                                    OPEN_ONE:begin
                                        if(tag_open)motion_state<=ARMED;
                                        else begin motion_state<=IDLE;age<=0;end
                                    end
                                    ARMED:if(tag_fist)motion_state<=FIST_ONE;
                                    FIST_ONE:begin
                                        if(tag_fist)begin effect_trigger<=1;cooldown<=COOLDOWN_FRAMES;motion_state<=IDLE;age<=0;end
                                        else motion_state<=ARMED;
                                    end
                                endcase
                            end
                        end else begin primed<=0;compare_frame<=0;motion_state<=IDLE;age<=0;end
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(ROI_MIN<1||ROI_MIN>10||GLOBAL_MAX<=40||GLOBAL_MAX>144||SEQUENCE_FRAMES<4||SEQUENCE_FRAMES>32||COOLDOWN_FRAMES<1||COOLDOWN_FRAMES>63)
            $fatal(1,"invalid fixed-ROI palm/fist thresholds");
    end
    // synthesis translate_on
endmodule
