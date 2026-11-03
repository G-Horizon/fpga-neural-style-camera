`timescale 1ns/1ps
// Scalar guide covers columns 0..W-2, rows 0..H-2. Repeat the final
// column on one idle cycle; the reader owns the bottom-row repetition.
// Arbitrary idle cycles within a line are valid. Coordinate/FID violations
// or a new sample during the reserved duplicate cycle require reset.
module d64_scalar_guide_edge_extend #(
    parameter integer W=1920,H=1080
)(
    input wire clk,rst_n,
    input wire in_valid,
    input wire [7:0] in_gray,
    input wire [15:0] in_x,in_y,
    input wire [31:0] in_fid,
    output wire out_valid,
    output wire [7:0] out_gray,
    output wire [10:0] out_x,out_y,
    output wire [31:0] out_fid,
    output wire out_sof,out_eol,out_eof,
    output wire fault
);
    reg bad,pending,frame_active;
    reg [10:0] expected_x,expected_y;
    reg [7:0] duplicate_gray;
    reg [31:0] active_fid;
    wire position_ok=in_x=={5'd0,expected_x}&&in_y=={5'd0,expected_y};
    wire fid_ok=!frame_active||in_fid==active_fid;
    wire protocol_error=in_valid&&(pending||!position_ok||!fid_ok);
    assign fault=bad||protocol_error;
    assign out_valid=rst_n&&!fault&&(pending||in_valid);
    assign out_gray=pending ? duplicate_gray : in_gray;
    assign out_x=pending ? W-1 : in_x[10:0];
    assign out_y=pending ? expected_y : in_y[10:0];
    assign out_fid=pending ? active_fid : in_fid;
    assign out_sof=out_valid&&out_x==0&&out_y==0;
    assign out_eol=out_valid&&out_x==W-1;
    assign out_eof=out_eol&&out_y==H-2;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            bad<=0;pending<=0;frame_active<=0;expected_x<=0;expected_y<=0;
            duplicate_gray<=0;active_fid<=0;
        end else begin
            if(protocol_error)bad<=1;
            if(!fault)begin
                if(pending)begin
                    pending<=0;
                    if(expected_y==H-2)begin frame_active<=0;expected_y<=0;end
                    else expected_y<=expected_y+1'b1;
                end else if(in_valid)begin
                    if(!frame_active)begin active_fid<=in_fid;frame_active<=1;end
                    if(in_x==W-2)begin
                        duplicate_gray<=in_gray;
                        pending<=1;expected_x<=0;
                    end else expected_x<=expected_x+1'b1;
                end
            end
        end
    end
    // synthesis translate_off
    initial if(W<2||W>2048||H<3||H>2048)$fatal(1,"SCALAR_GUIDE_EDGE_GEOMETRY");
    // synthesis translate_on
endmodule
