`timescale 1ns/1ps
// Center-aligned 15/2 enlargement. Three source-width row banks, no output row.
// Coefficients are round(256 * fractional_source_coordinate); the vertical
// blend retains all Q8 bits and only the final horizontal result is rounded.
// The raster must provide modulo-4096 active coordinates during blanking too.
module d64_bilinear_scaler #(
    parameter integer SRC_W=256, SRC_H=144,
    parameter integer WIN_W=1920, WIN_H=1080,
    parameter integer X0=0, Y0=0,
    parameter [23:0] BORDER=24'h000000
) (
    input wire clk, rst_n,
    input wire hs_i,vs_i,de_i,frame_i,source_valid_i,
    input wire [11:0] ax_i,ay_i,
    output reg load_req,
    output reg [7:0] load_row,
    input wire row_ready,row_good,load_we,
    input wire [7:0] load_wi,
    input wire [23:0] load_wd,
    output reg hs_o,vs_o,de_o,
    output reg [23:0] rgb_o,
    output reg pixel_good_o,
    output reg [11:0] ax_o,ay_o,
    output reg starved,
    output wire row_swap_attempt,row_swap_miss
);
    localparam integer PIPELINE_LATENCY=3;
    localparam [11:0] PRE_X=(X0-12)&4095;
    localparam [11:0] LOCK_X=(X0-1)&4095;
    wire in_y=(ay_i>=Y0)&&(ay_i<Y0+WIN_H);
    wire in_win=de_i&&in_y&&(ax_i>=X0)&&(ax_i<X0+WIN_W);
    wire line_start=in_win&&(ax_i==X0);
    wire preline=in_y&&(ax_i==PRE_X);
    wire lockline=in_y&&(ax_i==LOCK_X);
    reg signed [9:0] source_y,source_x;
    reg [4:0] frac_y,frac_x;
    wire [5:0] frac_y_step={1'b0,frac_y}+6'd4;
    wire [5:0] frac_x_step={1'b0,frac_x}+6'd4;
    wire x_carry=frac_x_step>=6'd30;
    wire y_carry=frac_y_step>=6'd30;
    wire [7:0] wanted_lo=(source_y<0)?8'd0:
        (source_y>=SRC_H-1)?SRC_H-1:source_y[7:0];
    wire [7:0] wanted_hi=(source_y<0)?8'd0:
        (source_y>=SRC_H-1)?SRC_H-1:source_y[7:0]+8'd1;
    function [8:0] coefficient;
        input [4:0] phase;
        begin
            case(phase)
                1:coefficient=9'd9; 3:coefficient=9'd26;
                5:coefficient=9'd43; 7:coefficient=9'd60;
                9:coefficient=9'd77; 11:coefficient=9'd94;
                13:coefficient=9'd111; 15:coefficient=9'd128;
                17:coefficient=9'd145; 19:coefficient=9'd162;
                21:coefficient=9'd179; 23:coefficient=9'd196;
                25:coefficient=9'd213; 27:coefficient=9'd230;
                29:coefficient=9'd247;
                default:coefficient=9'd0;
            endcase
        end
    endfunction
    wire [8:0] x_weight=((source_x<0)||(source_x>=SRC_W-1))?
        9'd0:coefficient(frac_x);
    reg [8:0] y_weight;
    reg [7:0] line_row_lo;
    reg [2:0] bank_valid;
    reg [7:0] bank_row0,bank_row1,bank_row2;
    wire lo0=bank_valid[0]&&(bank_row0==wanted_lo);
    wire lo1=bank_valid[1]&&(bank_row1==wanted_lo);
    wire lo2=bank_valid[2]&&(bank_row2==wanted_lo);
    wire hi0=bank_valid[0]&&(bank_row0==wanted_hi);
    wire hi1=bank_valid[1]&&(bank_row1==wanted_hi);
    wire hi2=bank_valid[2]&&(bank_row2==wanted_hi);
    wire pair_good=(lo0||lo1||lo2)&&(hi0||hi1||hi2);
    reg [1:0] lo_bank,hi_bank;
    reg line_good,preline_good;
    reg left_valid,right_valid,next_valid;
    assign row_swap_attempt=line_start;
    assign row_swap_miss=line_start&&!line_good;

    // Sequential unique rows. A bank is reused only after the current output
    // line has stopped reading its old row. Completion requires all 256 writes.
    reg fetch_busy,fetch_seen_low,fetch_discard,write_bad,frame_live;
    reg [8:0] write_count;
    reg [7:0] next_row,fetch_row;
    reg [1:0] next_bank,fetch_bank;
    wire [8:0] prefetch_limit={1'b0,line_row_lo}+9'd2;
    wire completion=fetch_busy&&fetch_seen_low&&row_ready;
    wire all_written=(write_count==9'd256)||
        (load_we&&(write_count==9'd255)&&(load_wi==8'd255));
    wire write0=load_we&&fetch_busy&&!fetch_discard&&(fetch_bank==0);
    wire write1=load_we&&fetch_busy&&!fetch_discard&&(fetch_bank==1);
    wire write2=load_we&&fetch_busy&&!fetch_discard&&(fetch_bank==2);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            load_req<=0;load_row<=0;bank_valid<=0;
            bank_row0<=0;bank_row1<=0;bank_row2<=0;
            fetch_busy<=0;fetch_seen_low<=0;fetch_discard<=0;frame_live<=0;
            write_bad<=0;write_count<=0;next_row<=0;fetch_row<=0;
            next_bank<=0;fetch_bank<=0;
        end else begin
            load_req<=0;
            if(fetch_busy&&!row_ready)fetch_seen_low<=1;
            if(load_we&&fetch_busy&&!fetch_discard) begin
                if((write_count>=SRC_W)||(load_wi!=write_count[7:0]))write_bad<=1;
                if(write_count<9'd511)write_count<=write_count+9'd1;
            end
            if(completion) begin
                fetch_busy<=0;
                if(!fetch_discard)begin
                    case(fetch_bank)
                        0:begin bank_row0<=fetch_row;bank_valid[0]<=row_good&&all_written&&!write_bad;end
                        1:begin bank_row1<=fetch_row;bank_valid[1]<=row_good&&all_written&&!write_bad;end
                        2:begin bank_row2<=fetch_row;bank_valid[2]<=row_good&&all_written&&!write_bad;end
                    endcase
                end
            end else if(!fetch_busy&&frame_live&&source_valid_i&&(next_row<SRC_H)&&
                ({1'b0,next_row}<=prefetch_limit)&&row_ready)begin
                load_req<=1;load_row<=next_row;fetch_row<=next_row;
                fetch_bank<=next_bank;fetch_busy<=1;fetch_seen_low<=0;
                fetch_discard<=0;write_bad<=0;write_count<=0;
                bank_valid[next_bank]<=0;
                next_row<=next_row+8'd1;
                next_bank<=(next_bank==2)?2'd0:next_bank+2'd1;
            end
            if(frame_i)begin
                load_req<=0;bank_valid<=0;next_row<=0;next_bank<=0;frame_live<=1;
                // An old request is drained, never relabelled as the new FID.
                if(fetch_busy)fetch_discard<=1;
            end
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            source_y<=-10'sd1;frac_y<=5'd17;
            source_x<=-10'sd1;frac_x<=5'd17;
            line_row_lo<=0;y_weight<=0;lo_bank<=0;hi_bank<=0;
            line_good<=0;preline_good<=0;starved<=0;
        end else if(frame_i)begin
            source_y<=-10'sd1;frac_y<=5'd17;
            source_x<=-10'sd1;frac_x<=5'd17;
            line_row_lo<=0;line_good<=0;preline_good<=0;
        end else begin
            if(preline)begin
                line_row_lo<=wanted_lo;preline_good<=pair_good;
                lo_bank<=lo0?0:lo1?1:2;
                hi_bank<=hi0?0:hi1?1:2;
                y_weight<=((source_y<0)||(source_y>=SRC_H-1))?0:coefficient(frac_y);
            end
            if(lockline)line_good<=preline_good&&left_valid&&right_valid&&next_valid;
            if(line_start)begin
                frac_y<=y_carry?frac_y_step-6'd30:frac_y_step;
                if(y_carry)source_y<=source_y+10'sd1;
                if(!line_good)starved<=1;
            end
            if(!in_win)begin source_x<=-10'sd1;frac_x<=5'd17;end
            else begin
                frac_x<=x_carry?frac_x_step-6'd30:frac_x_step;
                if(x_carry)source_x<=source_x+10'sd1;
            end
        end
    end

    // One address per bank each clock. Horizontal endpoints are recycled over
    // seven/eight output pixels, leaving enough time to calculate the next one.
    reg [1:0] prime_step;
    wire rotate=in_win&&x_carry&&(source_x>=0)&&(source_x<SRC_W-1);
    wire issue=preline||(prime_step!=0)||rotate;
    wire [7:0] runtime_col=(source_x>=SRC_W-3)?SRC_W-1:source_x[7:0]+8'd3;
    wire [7:0] ram_col=preline?8'd0:(prime_step==1)?8'd1:
        (prime_step==2)?8'd2:runtime_col;
    wire [1:0] issue_role=preline?2'd0:(prime_step==1)?2'd1:2'd2;
    wire [23:0] q0,q1,q2;
    d64_tc_ram #(.DW(24),.AW(8)) u_row0(.clk(clk),.we(write0),.wa(load_wi),.wd(load_wd),.re(1'b1),.ra(ram_col),.rd(q0));
    d64_tc_ram #(.DW(24),.AW(8)) u_row1(.clk(clk),.we(write1),.wa(load_wi),.wd(load_wd),.re(1'b1),.ra(ram_col),.rd(q1));
    d64_tc_ram #(.DW(24),.AW(8)) u_row2(.clk(clk),.we(write2),.wa(load_wi),.wd(load_wd),.re(1'b1),.ra(ram_col),.rd(q2));
    wire [23:0] top_q=(lo_bank==0)?q0:(lo_bank==1)?q1:q2;
    wire [23:0] bot_q=(hi_bank==0)?q0:(hi_bank==1)?q1:q2;
    reg issue_d1,issue_d2,issue_d3;
    reg [1:0] role_d1,role_d2,role_d3;
    reg signed [18:0] v_product[0:2];
    reg [15:0] v_base[0:2];
    reg [15:0] v_sum[0:2];
    reg [15:0] left_q8[0:2],right_q8[0:2],next_q8[0:2];
    integer c;
    wire signed [9:0] yw_signed=$signed({1'b0,y_weight});
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            prime_step<=0;issue_d1<=0;issue_d2<=0;issue_d3<=0;
            role_d1<=0;role_d2<=0;role_d3<=0;
            left_valid<=0;right_valid<=0;next_valid<=0;
            for(c=0;c<3;c=c+1)begin
                v_product[c]<=0;v_base[c]<=0;v_sum[c]<=0;
                left_q8[c]<=0;right_q8[c]<=0;next_q8[c]<=0;
            end
        end else begin
            if(preline)begin prime_step<=1;left_valid<=0;right_valid<=0;next_valid<=0;end
            else if(prime_step==1)prime_step<=2;
            else if(prime_step==2)prime_step<=0;
            issue_d1<=issue;issue_d2<=issue_d1;issue_d3<=issue_d2;
            role_d1<=issue_role;role_d2<=role_d1;role_d3<=role_d2;
            for(c=0;c<3;c=c+1)begin
                v_product[c]<=($signed({1'b0,bot_q[c*8+:8]})-
                    $signed({1'b0,top_q[c*8+:8]}))*yw_signed;
                v_base[c]<={top_q[c*8+:8],8'd0};
                v_sum[c]<=$signed({3'd0,v_base[c]})+v_product[c];
                if(issue_d3)begin
                    case(role_d3)
                        0:left_q8[c]<=v_sum[c];
                        1:right_q8[c]<=v_sum[c];
                        2:next_q8[c]<=v_sum[c];
                    endcase
                end
                if(rotate)begin left_q8[c]<=right_q8[c];right_q8[c]<=next_q8[c];end
            end
            if(issue_d3)begin
                case(role_d3)
                    0:left_valid<=1;1:right_valid<=1;2:next_valid<=1;
                endcase
            end
            if(rotate)begin
                left_valid<=right_valid;right_valid<=next_valid;next_valid<=0;
            end
            if(frame_i)begin
                prime_step<=0;issue_d1<=0;issue_d2<=0;issue_d3<=0;
                left_valid<=0;right_valid<=0;next_valid<=0;
            end
        end
    end
    reg signed [26:0] h_product[0:2];
    reg [23:0] h_base[0:2];
    reg signed [27:0] h_sum[0:2];
    reg [2:0] hs_pipe,vs_pipe,de_pipe,win_pipe,good_pipe;
    reg [11:0] ax_pipe0,ay_pipe0,ax_pipe1,ay_pipe1;
    wire signed [9:0] xw_signed=$signed({1'b0,x_weight});
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            hs_pipe<=0;vs_pipe<=0;de_pipe<=0;win_pipe<=0;good_pipe<=0;
            ax_pipe0<=0;ay_pipe0<=0;ax_pipe1<=0;ay_pipe1<=0;
            ax_o<=0;ay_o<=0;hs_o<=0;vs_o<=0;de_o<=0;rgb_o<=0;pixel_good_o<=0;
            for(c=0;c<3;c=c+1)begin h_product[c]<=0;h_base[c]<=0;h_sum[c]<=0;end
        end else begin
            hs_pipe<={hs_pipe[1:0],hs_i};vs_pipe<={vs_pipe[1:0],vs_i};
            de_pipe<={de_pipe[1:0],de_i};win_pipe<={win_pipe[1:0],in_win};
            good_pipe<={good_pipe[1:0],in_win&&line_good&&left_valid&&right_valid};
            ax_pipe0<=ax_i;ay_pipe0<=ay_i;ax_pipe1<=ax_pipe0;ay_pipe1<=ay_pipe0;
            ax_o<=ax_pipe1;ay_o<=ay_pipe1;
            hs_o<=hs_pipe[1];vs_o<=vs_pipe[1];de_o<=de_pipe[1];
            pixel_good_o<=good_pipe[1];
            for(c=0;c<3;c=c+1)begin
                h_product[c]<=($signed({1'b0,right_q8[c]})-
                    $signed({1'b0,left_q8[c]}))*xw_signed;
                h_base[c]<={left_q8[c],8'd0};
                h_sum[c]<=$signed({4'd0,h_base[c]})+
                    $signed({h_product[c][26],h_product[c]})+28'sd32768;
                rgb_o[c*8+:8]<=(win_pipe[1]&&good_pipe[1])?h_sum[c][23:16]:BORDER[c*8+:8];
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(SRC_W!=256||SRC_H!=144||WIN_W!=1920||WIN_H!=1080)
            $fatal(1,"d64_bilinear_scaler requires exact 256x144 -> 1920x1080 geometry");
    end
    // synthesis translate_on
endmodule
