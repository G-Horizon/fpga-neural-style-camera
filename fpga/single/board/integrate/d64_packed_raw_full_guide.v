`timescale 1ns/1ps
// Original-resolution green-channel guide. Top/bottom and left/right borders
// replicate the nearest RAW tap. Not RGB luminance, and not a sparse guide.
// The parent owns the three RAW line RAMs. Final-row flush borrows their read
// port during frame blanking; an overlapping new source frame is a fault.
module d64_packed_raw_full_guide #(
    parameter integer W_SRC=1920,H_SRC=1080,
    parameter [1:0] CFA=0,
    parameter [9:0] BLACK=64,
    parameter [15:0] GAIN_Q8=256,WB_G_Q8=256,
    parameter integer GAMMA_EN=1
)(
    input wire clk,rst_n,raw_in_valid,valid_p,eol_p,eof_p,
    input wire [15:0] x_p,y_p,
    input wire [1:0] head_p,
    input wire [31:0] fid_p,
    input wire [39:0] word0,word1,word2,prev0,prev1,prev2,mem_rd0,mem_rd1,mem_rd2,
    output wire flush_req,
    output reg [8:0] flush_addr,
    output wire flush_consume,
    output reg guide4_valid,
    output reg [31:0] guide4,
    output reg [10:0] guide_x0,guide_y,
    output reg [31:0] guide_frame_id,
    output reg guide_sof,guide_sol,guide_eol,guide_eof,guide_fault_sticky
);
    localparam integer WORDS=W_SRC/4;
    reg [39:0] older0,older1,older2;
    reg flush_active,flush_p;
    reg [8:0] flush_p_addr;
    reg [1:0] final_head;
    reg [31:0] final_fid;
    reg right_pending,right_final;
    reg [1:0] right_head;
    reg [15:0] right_y;
    reg [31:0] right_fid;
    assign flush_req=flush_active;
    wire flush_accept=flush_active&&!raw_in_valid;
    assign flush_consume=flush_p&&!valid_p;

    // Select a previous word of four centers. For a right-border flush,
    // current_word is not needed: center3's right tap is replicated.
    reg [39:0] mid_prev,top_prev,bot_prev,mid_cur,mid_old;
    reg [10:0] emit_x,emit_y;
    reg [31:0] emit_fid;
    reg emit,last_word;
    reg [1:0] hb,mb,tb;
    always @* begin
        emit=0;last_word=0;emit_x=0;emit_y=0;emit_fid=0;
        mid_prev=0;top_prev=0;bot_prev=0;mid_cur=0;mid_old=0;
        hb=head_p;mb=0;tb=0;
        if(right_pending)begin
            emit=1;last_word=1;emit_x=W_SRC-4;emit_y=right_y;emit_fid=right_fid;
            hb=right_head;
            mb=right_final ? hb : (hb==0 ? 2 : hb-1'b1);
            tb=right_final ? (hb==0 ? 2 : hb-1'b1) : (hb==2 ? 0 : hb+1'b1);
            mid_prev=mb==0 ? prev0 : mb==1 ? prev1 : prev2;
            top_prev=right_y==0 ? mid_prev : (tb==0 ? prev0 : tb==1 ? prev1 : prev2);
            bot_prev=right_final ? mid_prev : (hb==0 ? prev0 : hb==1 ? prev1 : prev2);
            mid_old=mb==0 ? older0 : mb==1 ? older1 : older2;
        end else if(valid_p && y_p>=1 && x_p>=4)begin
            emit=1;emit_x=x_p-4;emit_y=y_p-1'b1;emit_fid=fid_p;
            mb=hb==0 ? 2 : hb-1'b1;tb=hb==2 ? 0 : hb+1'b1;
            mid_prev=mb==0 ? prev0 : mb==1 ? prev1 : prev2;
            top_prev=y_p==1 ? mid_prev : (tb==0 ? prev0 : tb==1 ? prev1 : prev2);
            bot_prev=hb==0 ? prev0 : hb==1 ? prev1 : prev2;
            mid_cur=mb==0 ? word0 : mb==1 ? word1 : word2;
            mid_old=mb==0 ? older0 : mb==1 ? older1 : older2;
        end else if(flush_p && flush_p_addr>=1)begin
            emit=1;emit_x=(flush_p_addr-1'b1)*4;emit_y=H_SRC-1;emit_fid=final_fid;
            mb=final_head;tb=mb==0 ? 2 : mb-1'b1;
            mid_prev=mb==0 ? prev0 : mb==1 ? prev1 : prev2;
            top_prev=tb==0 ? prev0 : tb==1 ? prev1 : prev2;
            bot_prev=mid_prev;
            mid_cur=mb==0 ? mem_rd0 : mb==1 ? mem_rd1 : mem_rd2;
            mid_old=mb==0 ? older0 : mb==1 ? older1 : older2;
        end
    end
    function [7:0] photo;
        input [9:0] v;
        reg [25:0] t;
        begin
            t=v>BLACK ? (v-BLACK)*GAIN_Q8 : 0;
            t=t>>8;photo=t>1023 ? 8'hff : t[9:2];
        end
    endfunction
    function is_green;
        input xp,yp;
        begin is_green=(CFA==0||CFA==3) ? xp!=yp : xp==yp;end
    endfunction
    `include "d64_packed_raw_colour_functions.vh"
    wire [31:0] next_guide;
    genvar k;
    generate for(k=0;k<4;k=k+1)begin:g_green
        wire [7:0] c=photo(mid_prev[k*10+:10]);
        wire [7:0] u=photo(top_prev[k*10+:10]);
        wire [7:0] d=photo(bot_prev[k*10+:10]);
        wire [7:0] l,r;
        if(k==0)begin:g_left
            assign l=emit_x==0 ? c : photo(mid_old[39:30]);
        end else begin:g_left_inside
            assign l=photo(mid_prev[(k-1)*10+:10]);
        end
        if(k==3)begin:g_right
            assign r=last_word ? c : photo(mid_cur[9:0]);
        end else begin:g_right_inside
            assign r=photo(mid_prev[(k+1)*10+:10]);
        end
        wire [9:0] sum={2'b0,u}+{2'b0,d}+{2'b0,l}+{2'b0,r};
        wire [7:0] g=is_green((emit_x+k)%2,emit_y[0]) ? c : sum[9:2];
        wire [7:0] corrected=isp_wb_q8(g,WB_G_Q8);
        assign next_guide[k*8+:8]=GAMMA_EN ? isp_gamma8(corrected) : corrected;
    end endgenerate
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            older0<=0;older1<=0;older2<=0;flush_active<=0;flush_addr<=0;
            flush_p<=0;flush_p_addr<=0;final_head<=0;final_fid<=0;
            right_pending<=0;right_final<=0;right_head<=0;right_y<=0;right_fid<=0;
            guide4_valid<=0;guide4<=0;guide_x0<=0;guide_y<=0;guide_frame_id<=0;
            guide_sof<=0;guide_sol<=0;guide_eol<=0;guide_eof<=0;guide_fault_sticky<=0;
        end else begin
            guide4_valid<=emit;
            guide_sof<=emit && emit_x==0 && emit_y==0;
            guide_sol<=emit && emit_x==0;
            guide_eol<=emit && emit_x==W_SRC-4;
            guide_eof<=emit && emit_x==W_SRC-4 && emit_y==H_SRC-1;
            if(emit)begin
                guide4<=next_guide;guide_x0<=emit_x;guide_y<=emit_y;guide_frame_id<=emit_fid;
            end
            right_pending<=0;
            if(valid_p || flush_consume)begin older0<=prev0;older1<=prev1;older2<=prev2;end
            flush_p<=flush_accept;
            if(flush_accept)begin
                flush_p_addr<=flush_addr;
                if(flush_addr==WORDS-1)flush_active<=0;
                else flush_addr<=flush_addr+1'b1;
            end
            if(valid_p && eol_p && y_p>=1)begin
                right_pending<=1;right_final<=0;right_head<=head_p;
                right_y<=y_p-1'b1;right_fid<=fid_p;
            end
            if(valid_p && eof_p)begin
                flush_active<=1;flush_addr<=0;final_head<=head_p;final_fid<=fid_p;
            end
            if(flush_p && flush_p_addr==WORDS-1)begin
                right_pending<=1;right_final<=1;right_head<=final_head;
                right_y<=H_SRC-1;right_fid<=final_fid;
            end
            if(raw_in_valid && (flush_active||flush_p))begin
                guide_fault_sticky<=1;flush_active<=0;flush_p<=0;
            end
        end
    end
endmodule
