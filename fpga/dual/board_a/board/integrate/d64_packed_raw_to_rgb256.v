`timescale 1ns/1ps
// Four adjacent RAW10 samples per clk_pix. The 3x3 Bayer neighborhood is
// reconstructed on the ORIGINAL raster before emitting a target RGB pixel.
// Source line RAM addresses follow the incoming write stream. Backpressure
// cannot extend a source-bank lifetime; RGB pixels are buffered across CDC.
module d64_packed_raw_to_rgb256 #(
    parameter integer W_SRC=1920, H_SRC=1080, W_OUT=256, H_OUT=144,
    parameter [1:0] CFA=0,
    parameter [9:0] BLACK=64,
    parameter [15:0] GAIN_Q8=256,
    parameter [15:0] WB_R_Q8=519, WB_G_Q8=256, WB_B_Q8=427,
    parameter integer GAMMA_EN=1, GUIDE_GAMMA_EN=GAMMA_EN, FIFO_AW=9, GUIDE_ENABLE=0,
    parameter integer FAULT_STICKY=0
)(
    input wire clk_pix, rst_pix_n,
    input wire raw_valid,
    input wire [39:0] raw4,
    input wire raw_sof, raw_sol, raw_eol, raw_eof,
    input wire [31:0] raw_frame_id,
    input wire clk, rst_n,
    output reg d64_valid,
    input wire d64_ready,
    output reg [31:0] d64_frame_id,
    output reg d64_sof,
    output reg [7:0] d64_x0, d64_y,
    output reg [383:0] d64_rgb888,
    output reg d64_eol, d64_eof,
    output reg [31:0] src_beats, out_px, dropped_beats, marker_errors,
    output reg [31:0] beats_out, frames_out,
    output reg overflow_sticky,
    output wire guide4_valid,
    output wire [31:0] guide4,
    output wire [10:0] guide_x0,guide_y,
    output wire [31:0] guide_frame_id,
    output wire guide_sof,guide_sol,guide_eol,guide_eof,
    output wire guide_fault_sticky
);
    localparam integer WORDS=W_SRC/4;
    (* ram_style="block" *) reg [39:0] row0[0:WORDS-1];
    (* ram_style="block" *) reg [39:0] row1[0:WORDS-1];
    (* ram_style="block" *) reg [39:0] row2[0:WORDS-1];
    reg [39:0] rd0,rd1,rd2;
    reg [15:0] bx,sy;
    reg [1:0] head;
    wire [15:0] wa=raw_sof ? 0 : bx;
    wire [15:0] wy=raw_sof ? 0 : sy;
    wire [1:0] wb=raw_sof ? 0 : head;
    wire guide_flush_req,guide_flush_consume;
    wire [8:0] guide_flush_addr;
    wire [15:0] read_addr=(GUIDE_ENABLE==0) ? wa :
        (raw_valid ? wa : {7'b0,guide_flush_addr});
    always @(posedge clk_pix) begin
        if(raw_valid) begin
            if(wb==0) row0[wa]<=raw4;
            if(wb==1) row1[wa]<=raw4;
            if(wb==2) row2[wa]<=raw4;
        end
        if(raw_valid || guide_flush_req)begin
            rd0<=row0[read_addr]; rd1<=row1[read_addr]; rd2<=row2[read_addr];
        end
    end
    reg pv,psof,psol,peol,peof;
    reg [15:0] px,py;
    reg [1:0] ph;
    reg [39:0] pd;
    reg [31:0] pfid;
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if(!rst_pix_n) begin
            bx<=0;sy<=0;head<=0;pv<=0;psof<=0;psol<=0;peol<=0;peof<=0;
            px<=0;py<=0;ph<=0;pd<=0;pfid<=0;src_beats<=0;marker_errors<=0;
        end else begin
            pv<=raw_valid;
            if(raw_valid) begin
                src_beats<=src_beats+1'b1;
                psof<=raw_sof;psol<=raw_sol;peol<=raw_eol;peof<=raw_eof;
                px<={wa[13:0],2'b00};py<=wy;ph<=wb;pd<=raw4;pfid<=raw_frame_id;
                if(raw_sol!=(wa==0) || raw_eol!=(wa==WORDS-1) ||
                    raw_eof!=((wa==WORDS-1)&&(wy==H_SRC-1)))
                    marker_errors<=FAULT_STICKY ? 32'd1 : marker_errors+1'b1;
                if(raw_eol) begin
                    bx<=0;
                    sy<=raw_eof ? 0 : wy+1'b1;
                    head<=raw_eof ? 0 : ((wb==2) ? 0 : wb+1'b1);
                end else begin bx<=wa+1'b1;sy<=wy;head<=wb;end
            end
        end
    end
    wire [39:0] q0=ph==0 ? pd : rd0;
    wire [39:0] q1=ph==1 ? pd : rd1;
    wire [39:0] q2=ph==2 ? pd : rd2;
    reg [39:0] prev0,prev1,prev2;
    generate if(GUIDE_ENABLE!=0)begin:g_full_guide
        d64_packed_raw_full_guide #(.W_SRC(W_SRC),.H_SRC(H_SRC),.CFA(CFA),
            .BLACK(BLACK),.GAIN_Q8(GAIN_Q8),.WB_G_Q8(WB_G_Q8),.GAMMA_EN(GUIDE_GAMMA_EN)) u_guide(
            .clk(clk_pix),.rst_n(rst_pix_n),.raw_in_valid(raw_valid),
            .valid_p(pv),.eol_p(peol),.eof_p(peof),.x_p(px),.y_p(py),.head_p(ph),.fid_p(pfid),
            .word0(q0),.word1(q1),.word2(q2),.prev0(prev0),.prev1(prev1),.prev2(prev2),
            .mem_rd0(rd0),.mem_rd1(rd1),.mem_rd2(rd2),
            .flush_req(guide_flush_req),.flush_addr(guide_flush_addr),.flush_consume(guide_flush_consume),
            .guide4_valid(guide4_valid),.guide4(guide4),.guide_x0(guide_x0),.guide_y(guide_y),
            .guide_frame_id(guide_frame_id),.guide_sof(guide_sof),.guide_sol(guide_sol),
            .guide_eol(guide_eol),.guide_eof(guide_eof),.guide_fault_sticky(guide_fault_sticky));
    end else begin:g_no_guide
        assign guide_flush_req=0;assign guide_flush_addr=0;assign guide_flush_consume=0;
        assign guide4_valid=0;assign guide4=0;assign guide_x0=0;assign guide_y=0;assign guide_frame_id=0;
        assign guide_sof=0;assign guide_sol=0;assign guide_eol=0;assign guide_eof=0;assign guide_fault_sticky=0;
    end endgenerate
    wire [1:0] midbank=ph==0 ? 2 : ph-1'b1;
    wire [1:0] topbank=ph==2 ? 0 : ph+1'b1;
    wire [39:0] mq=midbank==0 ? q0 : midbank==1 ? q1 : q2;
    wire [39:0] tq=topbank==0 ? q0 : topbank==1 ? q1 : q2;
    wire [39:0] mp=midbank==0 ? prev0 : midbank==1 ? prev1 : prev2;
    wire [39:0] tp=topbank==0 ? prev0 : topbank==1 ? prev1 : prev2;
    wire [39:0] bp=ph==0 ? prev0 : ph==1 ? prev1 : prev2;
    // For each target center c, consume the beat containing c+1. A sample
    // offset -1 is in the preceding word; offsets 0..3 are in this word.
    function [9:0] tap;
        input [39:0] old_word,new_word;
        input [2:0] offset;
        begin
            case(offset)
                3'd7:tap=old_word[39:30];
                3'd6:tap=old_word[29:20];
                3'd0:tap=new_word[9:0];
                3'd1:tap=new_word[19:10];
                3'd2:tap=new_word[29:20];
                default:tap=new_word[39:30];
            endcase
        end
    endfunction
    reg [15:0] tx,ty,ox,oy;
    wire [2:0] ci=tx[2:0]-px[2:0];
    wire sample_fire=pv && py==ty+1'b1 && tx+1'b1>=px && tx+1'b1<px+4;
    wire [9:0] s_c=tap(mp,mq,ci);
    wire [9:0] s_l=tap(mp,mq,ci-1'b1),s_r=tap(mp,mq,ci+1'b1);
    wire [9:0] s_u=tap(tp,tq,ci),s_d=tap(bp,pd,ci);
    wire [9:0] s_ul=tap(tp,tq,ci-1'b1),s_ur=tap(tp,tq,ci+1'b1);
    wire [9:0] s_dl=tap(bp,pd,ci-1'b1),s_dr=tap(bp,pd,ci+1'b1);
    function [7:0] atap;
        input [9:0] v;
        reg [25:0] t;
        begin
            t=(v>BLACK) ? ((v-BLACK)*GAIN_Q8) : 26'd0;
            t=t>>8; atap=(t>1023) ? 8'hff : t[9:2];
        end
    endfunction
    function [1:0] cfa_col;
        input [1:0] pat;
        input xp,yp;
        begin
            case(pat)
            0:cfa_col=xp ? (yp ? 2 : 1) : (yp ? 1 : 0);
            1:cfa_col=xp ? (yp ? 1 : 0) : (yp ? 2 : 1);
            2:cfa_col=xp ? (yp ? 1 : 2) : (yp ? 0 : 1);
            default:cfa_col=xp ? (yp ? 0 : 1) : (yp ? 1 : 2);
            endcase
        end
    endfunction
    wire [7:0] tc=atap(s_c),tu=atap(s_u),td=atap(s_d),tl=atap(s_l),tr=atap(s_r);
    wire [7:0] tul=atap(s_ul),tur=atap(s_ur),tdl=atap(s_dl),tdr=atap(s_dr);
    wire [9:0] gs={2'b0,tu}+{2'b0,td}+{2'b0,tl}+{2'b0,tr};
    wire [9:0] ds={2'b0,tul}+{2'b0,tur}+{2'b0,tdl}+{2'b0,tdr};
    wire [8:0] hs={1'b0,tl}+{1'b0,tr},vs={1'b0,tu}+{1'b0,td};
    wire [1:0] cc=cfa_col(CFA,tx[0],ty[0]),cu=cfa_col(CFA,tx[0],~ty[0]);
    reg [7:0] dm_r,dm_g,dm_b;
    always @* begin
        case(cc)
        0:begin dm_r=tc;dm_g=gs[9:2];dm_b=ds[9:2];end
        2:begin dm_b=tc;dm_g=gs[9:2];dm_r=ds[9:2];end
        default:begin dm_g=tc;
            if(cu==0)begin dm_r=vs[8:1];dm_b=hs[8:1];end
            else begin dm_b=vs[8:1];dm_r=hs[8:1];end
        end
        endcase
    end
    // Colour functions are inserted verbatim from the tested source ISP.
    `include "d64_packed_raw_colour_functions.vh"
    wire [7:0] wr=isp_wb_q8(dm_r,WB_R_Q8),wg=isp_wb_q8(dm_g,WB_G_Q8),wbl=isp_wb_q8(dm_b,WB_B_Q8);
    wire [23:0] rgb=GAMMA_EN ? {isp_gamma8(wr),isp_gamma8(wg),isp_gamma8(wbl)} : {wr,wg,wbl};
    wire fifo_full,fifo_empty,fifo_valid;
    wire [FIFO_AW:0] wr_level;
    wire room=(wr_level<=(1<<FIFO_AW)-16);
    reg packet_drop;
    wire drop_this=ox[3:0]==0 ? !room : packet_drop;
    wire push=sample_fire && !drop_this;
    wire [70:0] wd={pfid,oy[7:0],ox[7:4],
        (oy==H_OUT-1 && ox==W_OUT-1),(ox==W_OUT-1),(oy==0&&ox==0),rgb};
    wire [70:0] fifo_rd;
    reg pending;
    wire re=!fifo_empty && !pending && !d64_valid;
    d64_async_fifo #(.DW(71),.AW(FIFO_AW)) u_pixels(
        .wr_clk(clk_pix),.wr_rst_n(rst_pix_n),.wr_en(push),.wr_data(wd),
        .wr_full(fifo_full),.wr_almost_full(),.wr_level(wr_level),
        .rd_clk(clk),.rd_rst_n(rst_n),.rd_en(re),.rd_data(fifo_rd),
        .rd_valid(fifo_valid),.rd_empty(fifo_empty),.rd_level());
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if(!rst_pix_n) begin
            tx<=3;ty<=3;ox<=0;oy<=0;prev0<=0;prev1<=0;prev2<=0;
            out_px<=0;dropped_beats<=0;packet_drop<=0;overflow_sticky<=0;
        end else begin
            if(pv) begin
                prev0<=q0;prev1<=q1;prev2<=q2;
                if(psof) begin tx<=3;ty<=3;ox<=0;oy<=0;end
                else begin
                    if(psol) begin tx<=3;ox<=0;end
                    if(sample_fire) begin
                        out_px<=out_px+1'b1;
                        tx<=tx+(ox[0] ? 7 : 8);ox<=ox+1'b1;
                        if(ox[3:0]==0) begin
                            packet_drop<=!room;
                            if(!room) begin
                                dropped_beats<=FAULT_STICKY ? 32'd1 : dropped_beats+1'b1;
                                overflow_sticky<=1;
                            end
                        end
                        if(push && fifo_full) overflow_sticky<=1;
                    end
                    if(peol && py==ty+1'b1) begin
                        ty<=ty+(oy[0] ? 7 : 8);oy<=oy+1'b1;
                    end
                end
            end else if(guide_flush_consume)begin
                prev0<=rd0;prev1<=rd1;prev2<=rd2;
            end
        end
    end
    reg [3:0] lane;
    reg assembly_sof;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pending<=0;d64_valid<=0;d64_frame_id<=0;d64_sof<=0;
            d64_x0<=0;d64_y<=0;d64_rgb888<=0;d64_eol<=0;d64_eof<=0;
            lane<=0;assembly_sof<=0;beats_out<=0;frames_out<=0;
        end else begin
            if(re)pending<=1;
            if(fifo_valid)pending<=0;
            if(d64_valid&&d64_ready) begin
                d64_valid<=0;beats_out<=beats_out+1'b1;
                if(d64_eof)frames_out<=frames_out+1'b1;
            end
            if(fifo_valid) begin
                // The output bank itself assembles the next beat while valid=0.
                // Reads stop once the final lane asserts valid, so this bank is
                // held unchanged throughout downstream backpressure.
                d64_rgb888[lane*24+:24]<=fifo_rd[23:0];
                if(lane==0)assembly_sof<=fifo_rd[24];
                if(lane==15) begin
                    d64_frame_id<=fifo_rd[70:39];d64_y<=fifo_rd[38:31];
                    d64_x0<={fifo_rd[30:27],4'b0};d64_eof<=fifo_rd[26];
                    d64_eol<=fifo_rd[25];d64_sof<=assembly_sof;
                    d64_valid<=1;lane<=0;
                end else lane<=lane+1'b1;
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(W_SRC<4 || H_SRC<3 || W_OUT<16 || H_OUT<2 || W_SRC*2!=W_OUT*15 || H_SRC*2!=H_OUT*15 || W_SRC%4!=0 || W_OUT%16!=0 || W_OUT>256 || H_OUT>256)
            $fatal(1,"PACKED_ISP_REQUIRES_FIXED_15_OVER_2_GEOMETRY");
        if(FIFO_AW<5 || WB_R_Q8>1024 || WB_G_Q8>1024 || WB_B_Q8>1024)
            $fatal(1,"PACKED_ISP_PARAMETER_DOMAIN");
        if((GAMMA_EN!=0&&GAMMA_EN!=1)||(GUIDE_GAMMA_EN!=0&&GUIDE_GAMMA_EN!=1)||(GUIDE_ENABLE!=0&&GUIDE_ENABLE!=1))
            $fatal(1,"PACKED_ISP_ENABLE_DOMAIN");
    end
    // synthesis translate_on
endmodule
