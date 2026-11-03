`timescale 1ns/1ps
`default_nettype none
module camera_input_stats #(
    parameter integer CLK_HZ=50000000,
    parameter integer WIDTH=1920, HEIGHT=1080
)(
    input wire clk,rst_n,pix_valid,vsync,
    input wire [3:0] pixels_per_clock,
    input wire mode_verified,
    input wire high_bits_nonzero,
    input wire display_clk,display_rst_n,display_frame_start,
    output reg [7:0] fps_bcd,
    output reg geometry_valid,
    output reg update_pulse,
    output reg [31:0] complete_frames,bad_frames,
    output reg [7:0] fps_bcd_pix,
    output reg geometry_valid_pix
);
    localparam integer FRAME_BEATS=WIDTH*HEIGHT/4;
    localparam integer CW=(CLK_HZ>1)?$clog2(CLK_HZ):1;
    localparam integer MAX_FRAME_CYCLES=(CLK_HZ>=4)?CLK_HZ/2:2;
    reg [CW-1:0] window_cycles;
    reg vs_q,seen_start,frame_bad;
    reg [CW-1:0] frame_age;
    reg [31:0] frame_beats;
    reg [7:0] window_bcd;
    wire fs=vsync&&!vs_q;
    wire input_bad=pix_valid&&(pixels_per_clock!=4||high_bits_nonzero);
    wire good_close=fs&&seen_start&&!frame_bad&&mode_verified&&frame_age<MAX_FRAME_CYCLES-1&&frame_beats==FRAME_BEATS;
    wire [7:0] incremented=(window_bcd==8'h99)?8'h99:
        window_bcd[3:0]==9?{window_bcd[7:4]+4'd1,4'd0}:
        {window_bcd[7:4],window_bcd[3:0]+4'd1};
    wire [7:0] closing_bcd=good_close?incremented:window_bcd;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            window_cycles<=0;window_bcd<=0;fps_bcd<=0;geometry_valid<=0;update_pulse<=0;
            vs_q<=0;seen_start<=0;frame_bad<=0;frame_beats<=0;frame_age<=0;
            complete_frames<=0;bad_frames<=0;
        end else begin
            vs_q<=vsync;update_pulse<=0;
            if(fs)begin
                if(seen_start)begin
                    if(good_close)complete_frames<=complete_frames+1'b1;
                    else if(bad_frames!=32'hffffffff)bad_frames<=bad_frames+1'b1;
                end
                seen_start<=1;frame_age<=0;
                frame_beats<=pix_valid?32'd1:32'd0;
                frame_bad<=input_bad;
            end else if(seen_start)begin
                if(frame_age==MAX_FRAME_CYCLES-1)begin
                    seen_start<=0;frame_beats<=0;frame_bad<=0;
                    if(bad_frames!=32'hffffffff)bad_frames<=bad_frames+1'b1;
                end else begin
                    frame_age<=frame_age+1'b1;
                    if(pix_valid)begin
                    if(frame_beats!=32'hffffffff)frame_beats<=frame_beats+1'b1;
                    else frame_bad<=1;
                    if(input_bad)frame_bad<=1;
                    end
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
