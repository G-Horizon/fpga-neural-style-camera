`timescale 1ns/1ps
`default_nettype none
module camera_bridge_input_stats #(
    parameter integer CLK_HZ=80000000, WIDTH=1920, HEIGHT=1080
)(
    input wire clk,rst_n,pix_valid,raw_sof,raw_sol,raw_eol,raw_eof,source_bad,
    input wire display_clk,display_rst_n,display_frame_start,
    output reg [7:0] fps_bcd,
    output reg geometry_valid,update_pulse,
    output reg [31:0] complete_frames,bad_frames,
    output reg [7:0] fps_bcd_pix,
    output reg geometry_valid_pix
);
    localparam integer LINE_BEATS=WIDTH/4, FRAME_BEATS=WIDTH*HEIGHT/4;
    localparam integer CW=(CLK_HZ>1)?$clog2(CLK_HZ):1;
    localparam integer MAX_FRAME_CYCLES=(CLK_HZ>=4)?CLK_HZ/2:2;
    reg [CW-1:0] window_cycles,frame_age;
    reg active,frame_bad;
    reg [15:0] x,y;
    reg [31:0] frame_groups;
    reg [7:0] window_bcd;
    wire [15:0] this_x=raw_sof?16'd0:x;
    wire [15:0] this_y=raw_sof?16'd0:y;
    wire [31:0] groups_before=raw_sof?32'd0:frame_groups;
    wire active_this=active||raw_sof;
    wire expected_eol=this_x==LINE_BEATS-1;
    wire expected_eof=expected_eol&&this_y==HEIGHT-1;
    wire marker_error=!active_this || (raw_sof&&active) ||
        raw_sol!=(this_x==0) || raw_eol!=expected_eol || raw_eof!=expected_eof;
    wire good_close=pix_valid&&raw_eof&&!source_bad&&!marker_error&&
        !(raw_sof?1'b0:frame_bad)&&groups_before==FRAME_BEATS-1&&
        (raw_sof||frame_age<MAX_FRAME_CYCLES-1);
    wire [7:0] incremented=(window_bcd==8'h99)?8'h99:
        window_bcd[3:0]==9?{window_bcd[7:4]+4'd1,4'd0}:
        {window_bcd[7:4],window_bcd[3:0]+4'd1};
    wire [7:0] closing_bcd=good_close?incremented:window_bcd;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            window_cycles<=0;frame_age<=0;active<=0;frame_bad<=0;x<=0;y<=0;frame_groups<=0;
            window_bcd<=0;fps_bcd<=0;geometry_valid<=0;update_pulse<=0;complete_frames<=0;bad_frames<=0;
        end else begin
            update_pulse<=0;
            if(active)begin
                if(frame_age<MAX_FRAME_CYCLES-1)frame_age<=frame_age+1'b1;
                if(source_bad)frame_bad<=1;
                if(frame_age==MAX_FRAME_CYCLES-1&&!(pix_valid&&raw_eof))begin
                    active<=0;frame_bad<=0;bad_frames<=bad_frames+1'b1;
                end
            end else frame_age<=0;
            if(pix_valid)begin
                if(raw_sof)begin
                    if(active)bad_frames<=bad_frames+1'b1;
                    active<=1;frame_age<=0;frame_bad<=source_bad||marker_error;
                end else if(active)frame_bad<=frame_bad||source_bad||marker_error;
                if(active_this)begin
                    frame_groups<=groups_before+1'b1;
                    if(expected_eol)begin x<=0;y<=this_y+1'b1;end
                    else begin x<=this_x+1'b1;y<=this_y;end
                end
                if(raw_eof)begin
                    active<=0;frame_bad<=0;
                    if(good_close)complete_frames<=complete_frames+1'b1;
                    else bad_frames<=bad_frames+1'b1;
                end
            end
            if(window_cycles==CLK_HZ-1)begin
                window_cycles<=0;window_bcd<=0;fps_bcd<=closing_bcd;
                geometry_valid<=closing_bcd!=0;update_pulse<=1;
            end else begin
                window_cycles<=window_cycles+1'b1;
                if(good_close)window_bcd<=incremented;
            end
        end
    end
    // Bundled data stays unchanged until the HDMI domain acknowledges it.
    // Only the request/ack bits pass through synchronizers. The payload has
    // an explicit max-delay bound in the project constraints.
    reg [8:0] held_payload;
    reg request_toggle,ack_toggle;
    (* async_reg="true" *) reg [1:0] ack_sync,request_sync;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin held_payload<=0;request_toggle<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(ack_sync[1]==request_toggle&&(held_payload!={geometry_valid,fps_bcd}||update_pulse))begin
                held_payload<={geometry_valid,fps_bcd};request_toggle<=!request_toggle;
            end
        end
    end
    always @(posedge display_clk or negedge display_rst_n)begin
        if(!display_rst_n)begin request_sync<=0;ack_toggle<=0;fps_bcd_pix<=0;geometry_valid_pix<=0;end
        else begin
            request_sync<={request_sync[0],request_toggle};
            if(display_frame_start&&request_sync[1]!=ack_toggle)begin
                {geometry_valid_pix,fps_bcd_pix}<=held_payload;ack_toggle<=request_sync[1];
            end
        end
    end
endmodule
`default_nettype wire
