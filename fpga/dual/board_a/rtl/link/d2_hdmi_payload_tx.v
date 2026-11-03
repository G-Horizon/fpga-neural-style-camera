module d2_hdmi_payload_tx #(
    parameter integer GUIDE_W=1920, GUIDE_H=1080,
    parameter integer PREVIEW_W=960, PREVIEW_H=540,
    parameter integer FEATURE_W=64, FEATURE_H=36, FEATURE_C=32,
    parameter integer HEADER_BYTES=4096,
    parameter integer CARRIER_W=1920, CARRIER_H=1080
) (
    input wire clk, rst_n, frame_start, de,
    input wire desc_valid,
    output wire desc_ready,
    input wire [31:0] source_fid, style_epoch,
    input wire [1:0] style, layout,
    input wire fps_enable,
    input wire [7:0] effect,
    input wire [23:0] payload_data,
    input wire payload_valid,
    output reg [1:0] payload_need, payload_take,
    output reg [23:0] rgb,
    output reg frame_done, frame_ok,
    output reg [31:0] frame_done_fid,
    output wire [31:0] diag_pixels,diag_consumed,output wire diag_bad,
    output reg underflow_sticky
);
    `include "d2_crc32.vh"
    localparam integer GUIDE_BYTES=GUIDE_W*GUIDE_H;
    localparam integer PREVIEW_BYTES=PREVIEW_W*PREVIEW_H*2;
    localparam integer FEATURE_BYTES=FEATURE_W*FEATURE_H*FEATURE_C*2;
    localparam integer PAYLOAD_BYTES=GUIDE_BYTES+PREVIEW_BYTES+FEATURE_BYTES;
    localparam integer PAYLOAD_END=HEADER_BYTES+PAYLOAD_BYTES;
    localparam integer CARRIER_PIXELS=CARRIER_W*CARRIER_H;
    reg have_frame, active, bad;
    reg [31:0] fid_q, epoch_q;
    reg [1:0] style_q, layout_q;
    reg fps_q;
    reg [7:0] effect_q;
    reg [31:0] byte_index, pixels, payload_consumed;
    reg [31:0] crc_payload, crc_header, header_crc;
    reg [6:0] header_build;
    wire header_ready=header_build==60;
    assign desc_ready=frame_start && rst_n;
    assign diag_pixels=pixels;assign diag_consumed=payload_consumed;assign diag_bad=bad;
    reg [479:0] header_body;
    always @* begin
        header_body=0;
        header_body[0*8+:64]=64'h3156324430364954; // TI60D2V1
        header_body[8*8+:16]=1;
        header_body[10*8+:16]=HEADER_BYTES;
        header_body[12*8+:32]={31'd0,active};
        header_body[16*8+:32]=fid_q;
        header_body[20*8+:32]=epoch_q;
        header_body[24*8+:8]={6'd0,style_q};
        header_body[25*8+:8]={6'd0,layout_q};
        header_body[26*8+:8]={7'd0,fps_q};
        header_body[27*8+:8]=effect_q;
        header_body[28*8+:32]=GUIDE_BYTES;
        header_body[32*8+:32]=PREVIEW_BYTES;
        header_body[36*8+:32]=FEATURE_BYTES;
        header_body[40*8+:16]=GUIDE_W;
        header_body[42*8+:16]=GUIDE_H;
        header_body[44*8+:16]=FEATURE_W;
        header_body[46*8+:16]=FEATURE_H;
        header_body[48*8+:16]=FEATURE_C;
        header_body[50*8+:16]=1;
        header_body[52*8+:32]=32'h00000503;
        header_body[56*8+:32]=PAYLOAD_BYTES;
    end
    function [7:0] footer_byte;
        input integer index;
        reg [63:0] magic;
        reg [31:0] check;
        begin
            magic=64'h213156444e453244; // D2ENDV1!
            check=(~crc_payload) ^ (bad ? 32'd1 : 32'd0);
            if(index<8) footer_byte=magic[index*8+:8];
            else if(index<12) footer_byte=check[(index-8)*8+:8];
            else footer_byte=fid_q[(index-12)*8+:8];
        end
    endfunction
    integer j, g, p;
    reg [7:0] b;
    reg [31:0] crc_payload_next;
    always @* begin
        rgb=0;payload_need=0;payload_take=0;p=0;
        crc_payload_next=crc_payload;b=0;g=0;
        for(j=0;j<3;j=j+1) begin
            g=byte_index+j;
            b=0;
            if(de && have_frame) begin
                if(g<60) b=header_body[g*8+:8];
                else if(g<64) b=header_crc[(g-60)*8+:8];
                else if(g>=HEADER_BYTES && g<PAYLOAD_END && active) begin
                    b=payload_valid ? payload_data[p*8+:8] : 8'd0;
                    p=p+1;
                    payload_need=payload_need+1'b1;
                    crc_payload_next=d2_crc32_byte(crc_payload_next,b);
                end
                else if(g>=PAYLOAD_END && g<PAYLOAD_END+16 && active) b=footer_byte(g-PAYLOAD_END);
            end
            rgb[23-j*8-:8]=b;
        end
        if(payload_valid) payload_take=payload_need;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            have_frame<=0;active<=0;bad<=0;fid_q<=0;epoch_q<=0;style_q<=0;layout_q<=0;fps_q<=0;effect_q<=0;
            byte_index<=0;pixels<=0;payload_consumed<=0;crc_payload<=32'hffffffff;
            crc_header<=32'hffffffff;header_crc<=0;header_build<=60;
            frame_done<=0;frame_ok<=0;frame_done_fid<=0;underflow_sticky<=0;
        end else begin
            frame_done<=0;frame_ok<=0;
            if(frame_start) begin
                frame_done<=have_frame && active;
                frame_ok<=have_frame && active && !bad && pixels==CARRIER_PIXELS && payload_consumed==PAYLOAD_BYTES;
                frame_done_fid<=fid_q;
                have_frame<=1;active<=desc_valid;bad<=0;
                fid_q<=desc_valid?source_fid:0;epoch_q<=desc_valid?style_epoch:0;
                style_q<=desc_valid?style:0;layout_q<=desc_valid?layout:0;fps_q<=desc_valid&&fps_enable;effect_q<=desc_valid?effect:0;
                byte_index<=0;pixels<=0;payload_consumed<=0;crc_payload<=32'hffffffff;
                crc_header<=32'hffffffff;header_crc<=0;header_build<=0;
                if(de) bad<=1;
            end else begin
                if(header_build<60) begin
                    crc_header<=d2_crc32_byte(crc_header,header_body[header_build*8+:8]);
                    if(header_build==59) header_crc<=~d2_crc32_byte(crc_header,header_body[header_build*8+:8]);
                    header_build<=header_build+1'b1;
                end
                if(de && have_frame) begin
                    byte_index<=byte_index+3;pixels<=pixels+1;
                    crc_payload<=crc_payload_next;
                    payload_consumed<=payload_consumed+payload_take;
                    if(!header_ready || pixels>=CARRIER_PIXELS) bad<=1;
                    if(payload_need!=0 && !payload_valid) begin bad<=1;underflow_sticky<=1;end
                end
            end
        end
    end
endmodule
