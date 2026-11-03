`timescale 1ns/1ps
module d64_fps_status #(
    parameter integer CLK_HZ=100000000,
    parameter integer DEFAULT_ON=1,REMOTE_DEBUG_ENABLE=0
)(
    input wire clk_count,rst_count_n,publish_event,toggle_event,
    input wire clk_pix,rst_pix_n,frame_start,
    input wire remote_set_valid,remote_set_value,output wire enabled_count_state,
    output reg [7:0] fps_bcd_pix,output reg enabled_pix,
    output wire [7:0] last_bcd_count
);
    localparam integer CW=(CLK_HZ>1)?$clog2(CLK_HZ):1;
    reg [CW-1:0] window_count;
    reg [7:0] count_bcd,last_bcd;
    assign last_bcd_count=last_bcd;
    reg enabled_count;
    assign enabled_count_state=enabled_count;
    reg [8:0] held_payload;
    reg request_toggle,ack_toggle;
    (* async_reg="true" *) reg [1:0] request_sync,ack_sync;
    wire [7:0] incremented_bcd=(count_bcd==8'h99)?8'h99:
        (count_bcd[3:0]==4'd9)?{count_bcd[7:4]+4'd1,4'd0}:
        {count_bcd[7:4],count_bcd[3:0]+4'd1};
    always @(posedge clk_count or negedge rst_count_n) begin
        if(!rst_count_n) begin
            window_count<=0;count_bcd<=0;last_bcd<=0;
            enabled_count<=(DEFAULT_ON!=0);held_payload<={(DEFAULT_ON!=0),8'd0};
            request_toggle<=0;ack_sync<=0;
        end else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(REMOTE_DEBUG_ENABLE!=0&&remote_set_valid)enabled_count<=remote_set_value;
            else if(toggle_event)enabled_count<=!enabled_count;
            if(window_count==CLK_HZ-1) begin
                window_count<=0;last_bcd<=publish_event?incremented_bcd:count_bcd;count_bcd<=0;
            end else begin
                window_count<=window_count+1'b1;
                if(publish_event)count_bcd<=incremented_bcd;
            end
            if(ack_sync[1]==request_toggle && held_payload!={enabled_count,last_bcd}) begin
                held_payload<={enabled_count,last_bcd};request_toggle<=!request_toggle;
            end
        end
    end
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if(!rst_pix_n) begin
            request_sync<=0;ack_toggle<=0;fps_bcd_pix<=0;enabled_pix<=(DEFAULT_ON!=0);
        end else begin
            request_sync<={request_sync[0],request_toggle};
            if(frame_start && request_sync[1]!=ack_toggle) begin
                {enabled_pix,fps_bcd_pix}<=held_payload;ack_toggle<=request_sync[1];
            end
        end
    end
endmodule
