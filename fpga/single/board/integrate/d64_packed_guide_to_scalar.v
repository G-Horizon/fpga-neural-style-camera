`timescale 1ns/1ps
// FID headers and four-pixel payload share one FIFO: no unsynchronized bus.
// Two-word prefetch sustains one byte per core cycle after the synchronous RAM.
module d64_packed_guide_to_scalar #(
    parameter integer W=1920,H=1080,AW=9
) (
    input wire clk_pix,rst_pix_n,
    input wire frame_header,input wire [31:0] header_fid,
    input wire guide_valid,input wire [31:0] guide4,
    input wire guide_sof,guide_eol,guide_eof,
    input wire source_fault,
    input wire clk,rst_n,
    output wire out_valid,output wire [7:0] out_gray,
    output wire [10:0] out_x,out_y,output wire [31:0] out_fid,
    output wire fault
);
    wire full,empty,read_valid;
    wire [35:0] read_data;
    wire write_enable=frame_header||guide_valid;
    wire [35:0] write_data=frame_header ? {4'b1000,header_fid} :
        {1'b0,guide_sof,guide_eol,guide_eof,guide4};
    reg source_bad;
    always @(posedge clk_pix or negedge rst_pix_n)
        if(!rst_pix_n)source_bad<=0;
        else if(source_fault||(write_enable&&full)||(frame_header&&guide_valid))source_bad<=1;
    reg [35:0] word0,word1;
    reg [1:0] count,byte_index;
    reg pending,bad,have_fid;
    reg [31:0] fid;
    reg [10:0] x,y;
    wire pop=count!=0&&(word0[35]||byte_index==3);
    wire read_enable=!empty&&!pending&&(count<2||pop);
    d64_async_fifo #(.DW(36),.AW(AW)) u_fifo(
        .wr_clk(clk_pix),.wr_rst_n(rst_pix_n),.wr_en(write_enable&&!full),.wr_data(write_data),
        .wr_full(full),.wr_almost_full(),.wr_level(),
        .rd_clk(clk),.rd_rst_n(rst_n),.rd_en(read_enable),.rd_data(read_data),
        .rd_valid(read_valid),.rd_empty(empty),.rd_level());
    (* async_reg="true" *) reg [1:0] bad_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)bad_sync<=0;
        else bad_sync<={bad_sync[0],source_bad};
    assign fault=bad||bad_sync[1];
    // A detected protocol/source failure requires coordinated reset. Do not
    // relabel remaining payload as a new frame after a premature header.
    assign out_valid=count!=0&&!word0[35]&&have_fid&&!fault;
    assign out_gray=word0[byte_index*8+:8];
    assign out_x=x;assign out_y=y;assign out_fid=fid;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            word0<=0;word1<=0;count<=0;byte_index<=0;pending<=0;
            bad<=0;have_fid<=0;fid<=0;x<=0;y<=0;
        end else begin
            if(read_enable)pending<=1;
            if(read_valid)pending<=0;
            case({read_valid,pop})
                2'b10:begin
                    if(count==0)word0<=read_data;
                    else if(count==1)word1<=read_data;
                    else bad<=1;
                    if(count<2)count<=count+1'b1;
                end
                2'b01:begin word0<=word1;count<=count-1'b1;end
                2'b11:begin
                    if(count==1)word0<=read_data;
                    else begin word0<=word1;word1<=read_data;end
                end
                default:begin end
            endcase
            if(count!=0)begin
                if(word0[35])begin
                    if(have_fid)bad<=1;
                    fid<=word0[31:0];have_fid<=1;byte_index<=0;x<=0;y<=0;
                end else begin
                    if(!have_fid)bad<=1;
                    if(byte_index==0&&
                       (word0[34]!=(x==0&&y==0)||word0[33]!=(x==W-4)||
                        word0[32]!=((x==W-4)&&(y==H-1))))bad<=1;
                    byte_index<=byte_index+1'b1;
                    if(x==W-1)begin x<=0;y<=y+1'b1;end
                    else x<=x+1'b1;
                    if(x==W-1&&y==H-1)have_fid<=0;
                end
            end
        end
    end
endmodule
