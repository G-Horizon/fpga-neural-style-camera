`timescale 1ns/1ps
`default_nettype none
module camera_mode_header_check #(
    parameter integer WIDTH=1920
)(
    input wire byte_clk,byte_rst_n,
    input wire [5:0] datatype,
    input wire [15:0] word_count,
    input wire [1:0] vc,
    input wire pix_clk,pix_rst_n,
    output wire verified
);
    // The receiver's ordinary packet sideband is in recovered-byte-clock
    // domain, unlike its video outputs. Compare it only in that source domain.
    // A stable packet tuple qualifies the configured RAW10 line width; this
    // is mode confirmation, not a counter of individual packet edges.
    reg [23:0] previous_header;
    reg correct_seen,wrong_seen;
    wire [23:0] header={vc,datatype,word_count};
    always @(posedge byte_clk or negedge byte_rst_n)begin
        if(!byte_rst_n)begin previous_header<=0;correct_seen<=0;wrong_seen<=0;end
        else begin
            previous_header<=header;
            if(header==previous_header&&datatype==6'h2b&&vc==0&&word_count!=0)begin
                if(word_count==WIDTH*10/8)correct_seen<=1;
                else wrong_seen<=1;
            end
        end
    end
    wire mode_ok=correct_seen&&!wrong_seen;
    (* async_reg="true" *) reg [1:0] mode_sync;
    always @(posedge pix_clk or negedge pix_rst_n)begin
        if(!pix_rst_n)mode_sync<=0;
        else mode_sync<={mode_sync[0],mode_ok};
    end
    assign verified=mode_sync[1];
endmodule
`default_nettype wire
