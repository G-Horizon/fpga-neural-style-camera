`timescale 1ns/1ps
// PCM16 mono at 16 kHz. Eight sequential Goertzel bins, one shared 16x16
// multiplier. PCM producer must honor ready; at 100 MHz the actual 16 kHz
// sample spacing is much longer than the serial arithmetic service time.
module d64_voice_features (
    input wire clk, rst_n, abort_event,
    input wire pcm_valid, input wire signed [15:0] pcm,
    output wire pcm_ready,
    output reg frame_valid, output reg [63:0] frame_features,
    output reg [15:0] frame_amplitude
);
    localparam IDLE=0, SMUL=1, SUPD=2, P1MUL=3, P1SAVE=4,
        P2MUL=5, P2SAVE=6, XMUL=7, XSAVE=8, CMUL=9, FINAL=10, PUBLISH=11;
    reg [3:0] state;
    reg [7:0] sample_count;
    reg [2:0] band;
    reg signed [15:0] s1[0:7], s2[0:7], sample_scaled, cross_q14;
    reg signed [31:0] product;
    reg signed [33:0] energy_sum;
    reg [23:0] amplitude_sum;
    reg [7:0] logs[0:7], max_log;
    reg signed [15:0] mul_a, mul_b;
    integer i;
    function signed [15:0] coefficient;
        input [2:0] b;
        begin
            case(b)
                0:coefficient=16'sd32413; 1:coefficient=16'sd31786;
                2:coefficient=16'sd30274; 3:coefficient=16'sd27246;
                4:coefficient=16'sd20788; 5:coefficient=16'sd9512;
                6:coefficient=-16'sd6393; default:coefficient=-16'sd23170;
            endcase
        end
    endfunction
    function [7:0] bit_length;
        input [31:0] v;
        integer n;
        begin bit_length=0; for(n=0;n<32;n=n+1) if(v[n])bit_length=n+1; end
    endfunction
    wire signed [31:0] filtered = $signed(sample_scaled)+($signed(product)>>>14)-$signed(s2[band]);
    wire signed [15:0] clipped = filtered>16383 ? 16'sd16383 :
        filtered < -16383 ? -16'sd16383 : filtered[15:0];
    wire signed [33:0] power = energy_sum-$signed(product);
    wire [7:0] this_log = power>0 ? bit_length(power[31:0]) : 8'd0;
    wire [15:0] absolute_pcm = pcm[15] ? (~pcm+16'd1) : pcm;
    assign pcm_ready=state==IDLE;
    always @* begin
        mul_a=s1[band]; mul_b=coefficient(band);
        case(state)
            P1MUL:mul_b=s1[band];
            P2MUL:begin mul_a=s2[band];mul_b=s2[band];end
            XMUL:mul_b=s2[band];
            CMUL:begin mul_a=cross_q14;mul_b=coefficient(band);end
            default:begin end
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;sample_count<=0;band<=0;sample_scaled<=0;
            product<=0;energy_sum<=0;cross_q14<=0;amplitude_sum<=0;
            frame_valid<=0;frame_features<=0;frame_amplitude<=0;max_log<=0;
            for(i=0;i<8;i=i+1)begin s1[i]<=0;s2[i]<=0;logs[i]<=0;end
        end else if(abort_event) begin
            state<=IDLE;sample_count<=0;band<=0;amplitude_sum<=0;
            frame_valid<=0;max_log<=0;
            for(i=0;i<8;i=i+1)begin s1[i]<=0;s2[i]<=0;logs[i]<=0;end
        end else begin
            frame_valid<=0;
            if(state==SMUL||state==P1MUL||state==P2MUL||state==XMUL||state==CMUL)
                product<=mul_a*mul_b;
            case(state)
                IDLE:if(pcm_valid)begin
                    sample_scaled<=$signed(pcm)>>>6;band<=0;state<=SMUL;
                    amplitude_sum<=amplitude_sum+absolute_pcm;
                    if(sample_count==255)frame_amplitude<=(amplitude_sum+absolute_pcm)>>8;
                end
                SMUL:state<=SUPD;
                SUPD:begin
                    s1[band]<=clipped;s2[band]<=s1[band];
                    if(band==7)begin
                        if(sample_count==255)begin band<=0;max_log<=0;state<=P1MUL;end
                        else begin sample_count<=sample_count+1'b1;state<=IDLE;end
                    end else begin band<=band+1'b1;state<=SMUL;end
                end
                P1MUL:state<=P1SAVE;
                P1SAVE:begin energy_sum<=$signed(product);state<=P2MUL;end
                P2MUL:state<=P2SAVE;
                P2SAVE:begin energy_sum<=energy_sum+$signed(product);state<=XMUL;end
                XMUL:state<=XSAVE;
                XSAVE:begin cross_q14<=$signed(product)>>>14;state<=CMUL;end
                CMUL:state<=FINAL;
                FINAL:begin
                    logs[band]<=this_log;if(this_log>max_log)max_log<=this_log;
                    s1[band]<=0;s2[band]<=0;
                    if(band==7)state<=PUBLISH;
                    else begin band<=band+1'b1;state<=P1MUL;end
                end
                PUBLISH:begin
                    for(i=0;i<8;i=i+1)frame_features[i*8+:8]<=max_log-logs[i];
                    frame_valid<=1;sample_count<=0;amplitude_sum<=0;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
