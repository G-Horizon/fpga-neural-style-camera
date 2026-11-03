`timescale 1ns/1ps
// Full-resolution camera ISP: four adjacent RAW10 samples -> four RGB888.
// The first and last row/column are emitted, with Bayer-preserving mirrored
// extension (-1 -> 1, W -> W-2). No spatial resize, CNN or detail fusion.
// RAW input cannot stall. Three RAW line banks retain a 3x3 neighbourhood.
// Horizontal lookahead costs one word, vertical lookahead one source row.
// The last row is flushed during vertical blanking; allow W_SRC/4+4 clocks.
// Each RGB lane is {R,G,B}; lane 0 occupies rgb4[23:0]. All ports use clk.
module camera_raw4_rgb888 #(
    parameter integer W_SRC=1920,H_SRC=1080,
    parameter [1:0] CFA=0,
    parameter [9:0] BLACK=64,
    parameter [15:0] GAIN_Q8=256,
    parameter [15:0] WB_R_Q8=519,WB_G_Q8=256,WB_B_Q8=427,
    parameter integer GAMMA_EN=1
)(
    input wire clk,rst_n,
    input wire raw_valid,
    input wire [39:0] raw4,
    input wire raw_sof,raw_sol,raw_eol,raw_eof,
    input wire [31:0] raw_frame_id,
    output reg rgb4_valid,
    output reg [95:0] rgb4,
    output reg [10:0] rgb_x0,rgb_y,
    output reg [31:0] rgb_frame_id,
    output reg rgb_sof,rgb_sol,rgb_eol,rgb_eof,
    output reg fault_sticky
);
    localparam integer WORDS=W_SRC/4;
    (* ram_style="block" *) reg [39:0] row0[0:WORDS-1];
    (* ram_style="block" *) reg [39:0] row1[0:WORDS-1];
    (* ram_style="block" *) reg [39:0] row2[0:WORDS-1];
    reg [39:0] rd0,rd1,rd2;
    reg [10:0] bx,sy;
    reg [1:0] head;
    reg input_frame;
    reg [31:0] input_fid;
    wire [10:0] wa=raw_sof ? 0 : bx;
    wire [10:0] wy=raw_sof ? 0 : sy;
    wire [1:0] wb=raw_sof ? 0 : head;
    reg flush_active,flush_p;
    reg [10:0] flush_addr,flush_p_addr;
    reg [1:0] final_head;
    reg [31:0] final_fid;
    wire flush_accept=flush_active&&!raw_valid;
    wire [10:0] read_addr=raw_valid ? wa : flush_addr;
    always @(posedge clk)begin
        if(raw_valid)begin
            if(wb==0)row0[wa]<=raw4;
            if(wb==1)row1[wa]<=raw4;
            if(wb==2)row2[wa]<=raw4;
        end
        if(raw_valid||flush_accept)begin
            rd0<=row0[read_addr];rd1<=row1[read_addr];rd2<=row2[read_addr];
        end
    end
    reg pv,peol,peof;
    reg [10:0] px,py;
    reg [1:0] ph;
    reg [39:0] pd;
    reg [31:0] pfid;
    wire [39:0] q0=ph==0 ? pd : rd0;
    wire [39:0] q1=ph==1 ? pd : rd1;
    wire [39:0] q2=ph==2 ? pd : rd2;
    reg [39:0] prev0,prev1,prev2,older0,older1,older2;
    reg right_pending,right_final;
    reg [1:0] right_head;
    reg [10:0] right_y;
    reg [31:0] right_fid;
    // The centre word is the preceding horizontal word. cur supplies its
    // rightmost neighbour, older supplies its leftmost neighbour.
    reg [39:0] mp,tp,bp,mc,tc,bc,mo,to,bo;
    reg [1:0] hb,mb,tb;
    reg emit,last_word;
    reg [10:0] ex,ey;
    reg [31:0] efid;
    always @*begin
        emit=0;last_word=0;ex=0;ey=0;efid=0;
        mp=0;tp=0;bp=0;mc=0;tc=0;bc=0;mo=0;to=0;bo=0;
        hb=ph;mb=0;tb=0;
        if(right_pending)begin
            emit=1;last_word=1;ex=W_SRC-4;ey=right_y;efid=right_fid;
            hb=right_head;
            mb=right_final ? hb : (hb==0 ? 2 : hb-1'b1);
            tb=right_final ? (hb==0 ? 2 : hb-1'b1) : (hb==2 ? 0 : hb+1'b1);
            mp=mb==0 ? prev0 : mb==1 ? prev1 : prev2;
            mo=mb==0 ? older0 : mb==1 ? older1 : older2;
            tp=tb==0 ? prev0 : tb==1 ? prev1 : prev2;
            to=tb==0 ? older0 : tb==1 ? older1 : older2;
            bp=right_final ? tp : (hb==0 ? prev0 : hb==1 ? prev1 : prev2);
            bo=right_final ? to : (hb==0 ? older0 : hb==1 ? older1 : older2);
            if(ey==0)begin tp=bp;to=bo;end
        end else if(pv&&py>=1&&px>=4)begin
            emit=1;ex=px-4;ey=py-1'b1;efid=pfid;
            mb=hb==0 ? 2 : hb-1'b1;tb=hb==2 ? 0 : hb+1'b1;
            mp=mb==0 ? prev0 : mb==1 ? prev1 : prev2;
            mc=mb==0 ? q0 : mb==1 ? q1 : q2;
            mo=mb==0 ? older0 : mb==1 ? older1 : older2;
            tp=tb==0 ? prev0 : tb==1 ? prev1 : prev2;
            tc=tb==0 ? q0 : tb==1 ? q1 : q2;
            to=tb==0 ? older0 : tb==1 ? older1 : older2;
            bp=hb==0 ? prev0 : hb==1 ? prev1 : prev2;
            bc=hb==0 ? q0 : hb==1 ? q1 : q2;
            bo=hb==0 ? older0 : hb==1 ? older1 : older2;
            if(ey==0)begin tp=bp;tc=bc;to=bo;end
        end else if(flush_p&&flush_p_addr>=1)begin
            emit=1;ex=(flush_p_addr-1'b1)*4;ey=H_SRC-1;efid=final_fid;
            mb=final_head;tb=mb==0 ? 2 : mb-1'b1;
            mp=mb==0 ? prev0 : mb==1 ? prev1 : prev2;
            mc=mb==0 ? rd0 : mb==1 ? rd1 : rd2;
            mo=mb==0 ? older0 : mb==1 ? older1 : older2;
            tp=tb==0 ? prev0 : tb==1 ? prev1 : prev2;
            tc=tb==0 ? rd0 : tb==1 ? rd1 : rd2;
            to=tb==0 ? older0 : tb==1 ? older1 : older2;
            bp=tp;bc=tc;bo=to;
        end
    end
    function [9:0] left_tap;
        input [39:0] old_word,word;
        input integer lane;
        input first_word;
        begin
            if(lane==0)left_tap=first_word ? word[19:10] : old_word[39:30];
            else left_tap=word[(lane-1)*10+:10];
        end
    endfunction
    function [9:0] right_tap;
        input [39:0] word,next_word;
        input integer lane;
        input last;
        begin
            if(lane==3)right_tap=last ? word[29:20] : next_word[9:0];
            else right_tap=word[(lane+1)*10+:10];
        end
    endfunction
    function [7:0] photo;
        input [9:0] v;
        reg [25:0] t;
        begin
            t=v>BLACK ? (v-BLACK)*GAIN_Q8 : 0;
            t=t>>8;photo=t>1023 ? 8'hff : t[9:2];
        end
    endfunction
    function [1:0] cfa_col;
        input xp,yp;
        begin
            case(CFA)
                0:cfa_col=xp ? (yp ? 2 : 1) : (yp ? 1 : 0);
                1:cfa_col=xp ? (yp ? 1 : 0) : (yp ? 2 : 1);
                2:cfa_col=xp ? (yp ? 1 : 2) : (yp ? 0 : 1);
                default:cfa_col=xp ? (yp ? 0 : 1) : (yp ? 1 : 2);
            endcase
        end
    endfunction
    function [7:0] isp_wb_q8;
        input [7:0] pixel;
        input [15:0] gain_q8;
        reg [23:0] wide,sum,rounded;
        integer wb_bit;
        begin
            wide={16'd0,pixel};sum=24'd0;
            for(wb_bit=0;wb_bit<16;wb_bit=wb_bit+1)
                if(gain_q8[wb_bit])sum=sum+(wide<<wb_bit);
            rounded=(sum+24'd128)>>8;
            isp_wb_q8=(rounded>24'd255)?8'd255:rounded[7:0];
        end
    endfunction

    // Exact integer8 gamma2.2 table: round(255*(x/255)^(1/2.2)).
    // No files, memory init or run-time pow; full case has a defined default.
    function [7:0] isp_gamma8;
        input [7:0] pixel;
        begin
            case(pixel)
                8'd0:isp_gamma8=8'd0;
                8'd1:isp_gamma8=8'd21;
                8'd2:isp_gamma8=8'd28;
                8'd3:isp_gamma8=8'd34;
                8'd4:isp_gamma8=8'd39;
                8'd5:isp_gamma8=8'd43;
                8'd6:isp_gamma8=8'd46;
                8'd7:isp_gamma8=8'd50;
                8'd8:isp_gamma8=8'd53;
                8'd9:isp_gamma8=8'd56;
                8'd10:isp_gamma8=8'd59;
                8'd11:isp_gamma8=8'd61;
                8'd12:isp_gamma8=8'd64;
                8'd13:isp_gamma8=8'd66;
                8'd14:isp_gamma8=8'd68;
                8'd15:isp_gamma8=8'd70;
                8'd16:isp_gamma8=8'd72;
                8'd17:isp_gamma8=8'd74;
                8'd18:isp_gamma8=8'd76;
                8'd19:isp_gamma8=8'd78;
                8'd20:isp_gamma8=8'd80;
                8'd21:isp_gamma8=8'd82;
                8'd22:isp_gamma8=8'd84;
                8'd23:isp_gamma8=8'd85;
                8'd24:isp_gamma8=8'd87;
                8'd25:isp_gamma8=8'd89;
                8'd26:isp_gamma8=8'd90;
                8'd27:isp_gamma8=8'd92;
                8'd28:isp_gamma8=8'd93;
                8'd29:isp_gamma8=8'd95;
                8'd30:isp_gamma8=8'd96;
                8'd31:isp_gamma8=8'd98;
                8'd32:isp_gamma8=8'd99;
                8'd33:isp_gamma8=8'd101;
                8'd34:isp_gamma8=8'd102;
                8'd35:isp_gamma8=8'd103;
                8'd36:isp_gamma8=8'd105;
                8'd37:isp_gamma8=8'd106;
                8'd38:isp_gamma8=8'd107;
                8'd39:isp_gamma8=8'd109;
                8'd40:isp_gamma8=8'd110;
                8'd41:isp_gamma8=8'd111;
                8'd42:isp_gamma8=8'd112;
                8'd43:isp_gamma8=8'd114;
                8'd44:isp_gamma8=8'd115;
                8'd45:isp_gamma8=8'd116;
                8'd46:isp_gamma8=8'd117;
                8'd47:isp_gamma8=8'd118;
                8'd48:isp_gamma8=8'd119;
                8'd49:isp_gamma8=8'd120;
                8'd50:isp_gamma8=8'd122;
                8'd51:isp_gamma8=8'd123;
                8'd52:isp_gamma8=8'd124;
                8'd53:isp_gamma8=8'd125;
                8'd54:isp_gamma8=8'd126;
                8'd55:isp_gamma8=8'd127;
                8'd56:isp_gamma8=8'd128;
                8'd57:isp_gamma8=8'd129;
                8'd58:isp_gamma8=8'd130;
                8'd59:isp_gamma8=8'd131;
                8'd60:isp_gamma8=8'd132;
                8'd61:isp_gamma8=8'd133;
                8'd62:isp_gamma8=8'd134;
                8'd63:isp_gamma8=8'd135;
                8'd64:isp_gamma8=8'd136;
                8'd65:isp_gamma8=8'd137;
                8'd66:isp_gamma8=8'd138;
                8'd67:isp_gamma8=8'd139;
                8'd68:isp_gamma8=8'd140;
                8'd69:isp_gamma8=8'd141;
                8'd70:isp_gamma8=8'd142;
                8'd71:isp_gamma8=8'd143;
                8'd72:isp_gamma8=8'd144;
                8'd73:isp_gamma8=8'd144;
                8'd74:isp_gamma8=8'd145;
                8'd75:isp_gamma8=8'd146;
                8'd76:isp_gamma8=8'd147;
                8'd77:isp_gamma8=8'd148;
                8'd78:isp_gamma8=8'd149;
                8'd79:isp_gamma8=8'd150;
                8'd80:isp_gamma8=8'd151;
                8'd81:isp_gamma8=8'd151;
                8'd82:isp_gamma8=8'd152;
                8'd83:isp_gamma8=8'd153;
                8'd84:isp_gamma8=8'd154;
                8'd85:isp_gamma8=8'd155;
                8'd86:isp_gamma8=8'd156;
                8'd87:isp_gamma8=8'd156;
                8'd88:isp_gamma8=8'd157;
                8'd89:isp_gamma8=8'd158;
                8'd90:isp_gamma8=8'd159;
                8'd91:isp_gamma8=8'd160;
                8'd92:isp_gamma8=8'd160;
                8'd93:isp_gamma8=8'd161;
                8'd94:isp_gamma8=8'd162;
                8'd95:isp_gamma8=8'd163;
                8'd96:isp_gamma8=8'd164;
                8'd97:isp_gamma8=8'd164;
                8'd98:isp_gamma8=8'd165;
                8'd99:isp_gamma8=8'd166;
                8'd100:isp_gamma8=8'd167;
                8'd101:isp_gamma8=8'd167;
                8'd102:isp_gamma8=8'd168;
                8'd103:isp_gamma8=8'd169;
                8'd104:isp_gamma8=8'd170;
                8'd105:isp_gamma8=8'd170;
                8'd106:isp_gamma8=8'd171;
                8'd107:isp_gamma8=8'd172;
                8'd108:isp_gamma8=8'd173;
                8'd109:isp_gamma8=8'd173;
                8'd110:isp_gamma8=8'd174;
                8'd111:isp_gamma8=8'd175;
                8'd112:isp_gamma8=8'd175;
                8'd113:isp_gamma8=8'd176;
                8'd114:isp_gamma8=8'd177;
                8'd115:isp_gamma8=8'd178;
                8'd116:isp_gamma8=8'd178;
                8'd117:isp_gamma8=8'd179;
                8'd118:isp_gamma8=8'd180;
                8'd119:isp_gamma8=8'd180;
                8'd120:isp_gamma8=8'd181;
                8'd121:isp_gamma8=8'd182;
                8'd122:isp_gamma8=8'd182;
                8'd123:isp_gamma8=8'd183;
                8'd124:isp_gamma8=8'd184;
                8'd125:isp_gamma8=8'd184;
                8'd126:isp_gamma8=8'd185;
                8'd127:isp_gamma8=8'd186;
                8'd128:isp_gamma8=8'd186;
                8'd129:isp_gamma8=8'd187;
                8'd130:isp_gamma8=8'd188;
                8'd131:isp_gamma8=8'd188;
                8'd132:isp_gamma8=8'd189;
                8'd133:isp_gamma8=8'd190;
                8'd134:isp_gamma8=8'd190;
                8'd135:isp_gamma8=8'd191;
                8'd136:isp_gamma8=8'd192;
                8'd137:isp_gamma8=8'd192;
                8'd138:isp_gamma8=8'd193;
                8'd139:isp_gamma8=8'd194;
                8'd140:isp_gamma8=8'd194;
                8'd141:isp_gamma8=8'd195;
                8'd142:isp_gamma8=8'd195;
                8'd143:isp_gamma8=8'd196;
                8'd144:isp_gamma8=8'd197;
                8'd145:isp_gamma8=8'd197;
                8'd146:isp_gamma8=8'd198;
                8'd147:isp_gamma8=8'd199;
                8'd148:isp_gamma8=8'd199;
                8'd149:isp_gamma8=8'd200;
                8'd150:isp_gamma8=8'd200;
                8'd151:isp_gamma8=8'd201;
                8'd152:isp_gamma8=8'd202;
                8'd153:isp_gamma8=8'd202;
                8'd154:isp_gamma8=8'd203;
                8'd155:isp_gamma8=8'd203;
                8'd156:isp_gamma8=8'd204;
                8'd157:isp_gamma8=8'd205;
                8'd158:isp_gamma8=8'd205;
                8'd159:isp_gamma8=8'd206;
                8'd160:isp_gamma8=8'd206;
                8'd161:isp_gamma8=8'd207;
                8'd162:isp_gamma8=8'd207;
                8'd163:isp_gamma8=8'd208;
                8'd164:isp_gamma8=8'd209;
                8'd165:isp_gamma8=8'd209;
                8'd166:isp_gamma8=8'd210;
                8'd167:isp_gamma8=8'd210;
                8'd168:isp_gamma8=8'd211;
                8'd169:isp_gamma8=8'd212;
                8'd170:isp_gamma8=8'd212;
                8'd171:isp_gamma8=8'd213;
                8'd172:isp_gamma8=8'd213;
                8'd173:isp_gamma8=8'd214;
                8'd174:isp_gamma8=8'd214;
                8'd175:isp_gamma8=8'd215;
                8'd176:isp_gamma8=8'd215;
                8'd177:isp_gamma8=8'd216;
                8'd178:isp_gamma8=8'd217;
                8'd179:isp_gamma8=8'd217;
                8'd180:isp_gamma8=8'd218;
                8'd181:isp_gamma8=8'd218;
                8'd182:isp_gamma8=8'd219;
                8'd183:isp_gamma8=8'd219;
                8'd184:isp_gamma8=8'd220;
                8'd185:isp_gamma8=8'd220;
                8'd186:isp_gamma8=8'd221;
                8'd187:isp_gamma8=8'd221;
                8'd188:isp_gamma8=8'd222;
                8'd189:isp_gamma8=8'd223;
                8'd190:isp_gamma8=8'd223;
                8'd191:isp_gamma8=8'd224;
                8'd192:isp_gamma8=8'd224;
                8'd193:isp_gamma8=8'd225;
                8'd194:isp_gamma8=8'd225;
                8'd195:isp_gamma8=8'd226;
                8'd196:isp_gamma8=8'd226;
                8'd197:isp_gamma8=8'd227;
                8'd198:isp_gamma8=8'd227;
                8'd199:isp_gamma8=8'd228;
                8'd200:isp_gamma8=8'd228;
                8'd201:isp_gamma8=8'd229;
                8'd202:isp_gamma8=8'd229;
                8'd203:isp_gamma8=8'd230;
                8'd204:isp_gamma8=8'd230;
                8'd205:isp_gamma8=8'd231;
                8'd206:isp_gamma8=8'd231;
                8'd207:isp_gamma8=8'd232;
                8'd208:isp_gamma8=8'd232;
                8'd209:isp_gamma8=8'd233;
                8'd210:isp_gamma8=8'd233;
                8'd211:isp_gamma8=8'd234;
                8'd212:isp_gamma8=8'd234;
                8'd213:isp_gamma8=8'd235;
                8'd214:isp_gamma8=8'd235;
                8'd215:isp_gamma8=8'd236;
                8'd216:isp_gamma8=8'd236;
                8'd217:isp_gamma8=8'd237;
                8'd218:isp_gamma8=8'd237;
                8'd219:isp_gamma8=8'd238;
                8'd220:isp_gamma8=8'd238;
                8'd221:isp_gamma8=8'd239;
                8'd222:isp_gamma8=8'd239;
                8'd223:isp_gamma8=8'd240;
                8'd224:isp_gamma8=8'd240;
                8'd225:isp_gamma8=8'd241;
                8'd226:isp_gamma8=8'd241;
                8'd227:isp_gamma8=8'd242;
                8'd228:isp_gamma8=8'd242;
                8'd229:isp_gamma8=8'd243;
                8'd230:isp_gamma8=8'd243;
                8'd231:isp_gamma8=8'd244;
                8'd232:isp_gamma8=8'd244;
                8'd233:isp_gamma8=8'd245;
                8'd234:isp_gamma8=8'd245;
                8'd235:isp_gamma8=8'd246;
                8'd236:isp_gamma8=8'd246;
                8'd237:isp_gamma8=8'd247;
                8'd238:isp_gamma8=8'd247;
                8'd239:isp_gamma8=8'd248;
                8'd240:isp_gamma8=8'd248;
                8'd241:isp_gamma8=8'd249;
                8'd242:isp_gamma8=8'd249;
                8'd243:isp_gamma8=8'd249;
                8'd244:isp_gamma8=8'd250;
                8'd245:isp_gamma8=8'd250;
                8'd246:isp_gamma8=8'd251;
                8'd247:isp_gamma8=8'd251;
                8'd248:isp_gamma8=8'd252;
                8'd249:isp_gamma8=8'd252;
                8'd250:isp_gamma8=8'd253;
                8'd251:isp_gamma8=8'd253;
                8'd252:isp_gamma8=8'd254;
                8'd253:isp_gamma8=8'd254;
                8'd254:isp_gamma8=8'd255;
                8'd255:isp_gamma8=8'd255;
                default:isp_gamma8=8'd0;
            endcase
        end
    endfunction

    wire [95:0] next_rgb;
    genvar k;
    generate for(k=0;k<4;k=k+1)begin:g_rgb
        wire [7:0] c=photo(mp[k*10+:10]);
        wire [7:0] u=photo(tp[k*10+:10]),d=photo(bp[k*10+:10]);
        wire [7:0] l=photo(left_tap(mo,mp,k,ex==0));
        wire [7:0] r=photo(right_tap(mp,mc,k,last_word));
        wire [7:0] ul=photo(left_tap(to,tp,k,ex==0));
        wire [7:0] ur=photo(right_tap(tp,tc,k,last_word));
        wire [7:0] dl=photo(left_tap(bo,bp,k,ex==0));
        wire [7:0] dr=photo(right_tap(bp,bc,k,last_word));
        wire [9:0] gs={2'b0,u}+{2'b0,d}+{2'b0,l}+{2'b0,r};
        wire [9:0] ds={2'b0,ul}+{2'b0,ur}+{2'b0,dl}+{2'b0,dr};
        wire [8:0] hs={1'b0,l}+{1'b0,r},vs={1'b0,u}+{1'b0,d};
        wire [1:0] cc=cfa_col(k%2,ey[0]),cu=cfa_col(k%2,~ey[0]);
        reg [7:0] rr,gg,bb;
        always @*begin
            case(cc)
                0:begin rr=c;gg=gs[9:2];bb=ds[9:2];end
                2:begin bb=c;gg=gs[9:2];rr=ds[9:2];end
                default:begin gg=c;
                    if(cu==0)begin rr=vs[8:1];bb=hs[8:1];end
                    else begin bb=vs[8:1];rr=hs[8:1];end
                end
            endcase
        end
        wire [7:0] wr=isp_wb_q8(rr,WB_R_Q8),wg=isp_wb_q8(gg,WB_G_Q8),wbv=isp_wb_q8(bb,WB_B_Q8);
        assign next_rgb[k*24+:24]=GAMMA_EN ? {isp_gamma8(wr),isp_gamma8(wg),isp_gamma8(wbv)} : {wr,wg,wbv};
    end endgenerate
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            bx<=0;sy<=0;head<=0;pv<=0;peol<=0;peof<=0;input_frame<=0;input_fid<=0;
            px<=0;py<=0;ph<=0;pd<=0;pfid<=0;
            prev0<=0;prev1<=0;prev2<=0;older0<=0;older1<=0;older2<=0;
            flush_active<=0;flush_p<=0;flush_addr<=0;flush_p_addr<=0;
            final_head<=0;final_fid<=0;right_pending<=0;right_final<=0;
            right_head<=0;right_y<=0;right_fid<=0;
            rgb4_valid<=0;rgb4<=0;rgb_x0<=0;rgb_y<=0;rgb_frame_id<=0;
            rgb_sof<=0;rgb_sol<=0;rgb_eol<=0;rgb_eof<=0;fault_sticky<=0;
        end else begin
            pv<=raw_valid;
            if(raw_valid)begin
                px<=wa*4;py<=wy;ph<=wb;pd<=raw4;pfid<=raw_frame_id;
                peol<=raw_eol;peof<=raw_eof;
                if(raw_sof)begin
                    if(input_frame)fault_sticky<=1;
                    input_frame<=1;input_fid<=raw_frame_id;
                end else if(!input_frame||raw_frame_id!=input_fid)fault_sticky<=1;
                if(raw_eof)input_frame<=0;
                if(raw_sol!=(wa==0)||raw_eol!=(wa==WORDS-1)||
                   raw_eof!=((wa==WORDS-1)&&(wy==H_SRC-1))||
                   raw_sof!=((wa==0)&&(wy==0)))fault_sticky<=1;
                if(raw_eol)begin
                    bx<=0;sy<=raw_eof ? 0 : wy+1'b1;
                    head<=raw_eof ? 0 : (wb==2 ? 0 : wb+1'b1);
                end else begin bx<=wa+1'b1;sy<=wy;head<=wb;end
            end
            if(pv||flush_p)begin
                older0<=prev0;older1<=prev1;older2<=prev2;
                prev0<=pv ? q0 : rd0;prev1<=pv ? q1 : rd1;prev2<=pv ? q2 : rd2;
            end
            right_pending<=0;
            if(pv&&peol&&py>=1)begin
                right_pending<=1;right_final<=0;right_head<=ph;
                right_y<=py-1'b1;right_fid<=pfid;
            end
            if(pv&&peof)begin
                flush_active<=1;flush_addr<=0;final_head<=ph;final_fid<=pfid;
            end
            flush_p<=flush_accept;
            if(flush_accept)begin
                flush_p_addr<=flush_addr;
                if(flush_addr==WORDS-1)flush_active<=0;
                else flush_addr<=flush_addr+1'b1;
            end
            if(flush_p&&flush_p_addr==WORDS-1)begin
                right_pending<=1;right_final<=1;right_head<=final_head;
                right_y<=H_SRC-1;right_fid<=final_fid;
            end
            // Include the cycle before flush_active rises: peof is already
            // in the RAW RAM pipeline and that blanking interval is owed.
            if(raw_valid&&(flush_active||flush_p||(pv&&peof)))begin
                fault_sticky<=1;flush_active<=0;flush_p<=0;
            end
            rgb4_valid<=emit;
            rgb_sof<=emit&&ex==0&&ey==0;rgb_sol<=emit&&ex==0;
            rgb_eol<=emit&&ex==W_SRC-4;rgb_eof<=emit&&ex==W_SRC-4&&ey==H_SRC-1;
            if(emit)begin rgb4<=next_rgb;rgb_x0<=ex;rgb_y<=ey;rgb_frame_id<=efid;end
        end
    end
    // synthesis translate_off
    initial begin
        if(W_SRC<8||W_SRC>2048||W_SRC%4!=0||H_SRC<2||H_SRC>2048)
            $fatal(1,"CAMERA_ISP_INVALID_GEOMETRY");
        if(GAMMA_EN!=0&&GAMMA_EN!=1)$fatal(1,"CAMERA_ISP_INVALID_GAMMA");
    end
    // synthesis translate_on
endmodule
