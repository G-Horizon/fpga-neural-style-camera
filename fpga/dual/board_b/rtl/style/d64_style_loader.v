`timescale 1ns/1ps
// One active model: call only after all CNN requests/responses have retired.
// AXI adapter sends 2,179 ordered 128-bit beats, starting at profile_addr.
// CRC32/IEEE is byte serial (little endian), keeping its area bounded. The last
// beat's high64 padding is checked zero and excluded from the payload CRC.
// A failed load may already have changed weights: invalidate stays asserted
// until a whole validated profile has completed. Display buffers are separate.
module d64_style_loader #(
    parameter [31:0] ASSET_BASE=32'h00700000,
    parameter [31:0] ASSET_STRIDE=32'h00010000,
    parameter [3:0] ALLOWED_STYLE_MASK=4'b1110,
    parameter integer TIMEOUT_CYCLES=1000000
) (
    input wire clk,input wire rst_n,
    input wire start,input wire [1:0] style,
    output reg request,output reg [31:0] profile_addr,
    output wire [15:0] profile_beats,
    input wire stream_valid,output wire stream_ready,
    input wire [127:0] stream_data,input wire stream_error,
    output reg wt_we,output reg [8:0] wt_addr,output wire [511:0] wt_data,
    output reg cfg_we,output reg [8:0] cfg_index,output reg [63:0] cfg_data,
    output reg [9:0] profile_trunk_zero,
    output reg busy,output reg done,output reg failed,
    output reg invalidate,output reg [3:0] error_code
);
    localparam HEADER=0,PAYLOAD=1,CRC=2,FINISH=3,IDLE=4;
    localparam integer WEIGHT_BEATS=1976,PAYLOAD_BEATS=2178;
    reg [2:0] state;
    reg [1:0] style_q;
    reg [11:0] beat;
    reg [3:0] byte_index;
    reg [127:0] beat_q;
    // The first seven chunks remain here. On the eighth chunk, beat_q already
    // holds the final 128 bits for the registered, one-clock wt_we interval.
    reg [383:0] tile_q;
    reg [31:0] crc_q,expected_crc,timer;
    reg [4:0] bytes_in_beat;
    reg cfg_second;
    wire [31:0] manifest_crc;
    wire [9:0] manifest_zero;
    d64_style_profile_manifest u_manifest(style_q,manifest_crc,manifest_zero);
    assign profile_beats=16'd2179;
    assign stream_ready=busy&&(state==HEADER||state==PAYLOAD);
    // The consumer captures on wt_we before CRC shifts beat_q on that edge.
    // wt_data has no hold requirement while wt_we is low.
    assign wt_data={beat_q,tile_q};
    function [31:0] crc_byte;
        input [31:0] old_crc;input [7:0] data;
        reg [31:0] c;integer n;
        begin
            c=old_crc^{24'd0,data};
            for(n=0;n<8;n=n+1)c=c[0]?(c>>1)^32'hedb88320:(c>>1);
            crc_byte=c;
        end
    endfunction
    wire [31:0] crc_next=crc_byte(crc_q,beat_q[7:0]);
    task fail;
        input [3:0] code;
        begin busy<=0;failed<=1;error_code<=code;state<=IDLE;end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            state<=IDLE;busy<=0;done<=0;failed<=0;request<=0;invalidate<=0;
            wt_we<=0;cfg_we<=0;wt_addr<=0;cfg_index<=0;cfg_data<=0;
            profile_addr<=0;profile_trunk_zero<=0;error_code<=0;
            beat<=0;byte_index<=0;bytes_in_beat<=0;beat_q<=0;tile_q<=0;
            crc_q<=32'hffffffff;expected_crc<=0;style_q<=0;timer<=0;cfg_second<=0;
        end else begin
            request<=0;done<=0;failed<=0;wt_we<=0;cfg_we<=0;
            if(start&&!busy)begin
                state<=HEADER;busy<=1;style_q<=style;beat<=0;timer<=0;
                crc_q<=32'hffffffff;invalidate<=0;error_code<=0;
                if(!ALLOWED_STYLE_MASK[style])begin
                    // Reject a removed profile before any DDR request/write.
                    fail(4'd7);
                end else begin
                    request<=1;profile_addr<=ASSET_BASE+style*ASSET_STRIDE;
                end
            end else if(busy)begin
                timer<=timer+1;
                if(stream_error)fail(4'd1);
                else if(timer>=TIMEOUT_CYCLES-1)fail(4'd2);
                else case(state)
                    HEADER:if(stream_valid)begin
                        // Reserved header bits must be zero, not silently ignored.
                        if(stream_data[31:0]!=32'h31545344 || stream_data[39:32]!=8'd3 ||
                           stream_data[47:40]!={6'd0,style_q} || stream_data[63:48]!=16'd494 ||
                           stream_data[79:64]!=16'd403 || stream_data[95:90]!=0 ||
                           stream_data[89:80]!=manifest_zero || stream_data[127:96]!=manifest_crc)fail(4'd3);
                        else begin
                            expected_crc<=stream_data[127:96];
                            profile_trunk_zero<=stream_data[89:80];state<=PAYLOAD;
                        end
                    end
                    PAYLOAD:if(stream_valid)begin
                        if(beat==PAYLOAD_BEATS-1 && stream_data[127:64]!=0)fail(4'd4);
                        else begin
                            invalidate<=1;beat_q<=stream_data;byte_index<=0;
                            bytes_in_beat<=(beat==PAYLOAD_BEATS-1)?5'd8:5'd16;
                            cfg_second<=0;state<=CRC;
                            if(beat<WEIGHT_BEATS)begin
                                if(beat[1:0]!=3)tile_q<={stream_data,tile_q[383:128]};
                                if(beat[1:0]==3)begin wt_we<=1;wt_addr<=beat[10:2];end
                            end else begin
                                cfg_index<=(beat-WEIGHT_BEATS)*2;
                                cfg_data<=stream_data[63:0];cfg_we<=1;
                                cfg_second<=(beat!=PAYLOAD_BEATS-1);
                            end
                        end
                    end
                    CRC:begin
                        if(cfg_second)begin
                            cfg_index<=cfg_index+1;cfg_data<=beat_q[127:64];
                            cfg_we<=1;cfg_second<=0;
                        end
                        crc_q<=crc_next;beat_q<=beat_q>>8;
                        if({1'b0,byte_index}+1==bytes_in_beat)begin
                            if(beat==PAYLOAD_BEATS-1)state<=FINISH;
                            else begin beat<=beat+1;state<=PAYLOAD;end
                        end else byte_index<=byte_index+1;
                    end
                    FINISH:begin
                        if((crc_q^32'hffffffff)!=expected_crc)fail(4'd5);
                        else begin busy<=0;done<=1;invalidate<=0;state<=IDLE;end
                    end
                    default:fail(4'd6);
                endcase
            end
        end
    end
    // synthesis translate_off
    initial if(ALLOWED_STYLE_MASK[0])$fatal(1,"style loader excludes removed ID0");
    // synthesis translate_on
endmodule
