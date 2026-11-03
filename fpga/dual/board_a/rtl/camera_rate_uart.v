`timescale 1ns/1ps
`default_nettype none
module camera_rate_uart #(parameter integer CLK_HZ=50000000,BAUD=115200)(
 input wire clk,rst_n,input wire [7:0] fps_bcd,input wire geometry_valid,
 input wire [31:0] complete_frames,bad_frames,input wire [191:0] capture_debug,input wire source_fault,
 input wire [127:0] csi_status,
 input wire core_clk,core_rst_n,input wire [31:0] commits,presents,input wire write_bad,read_bad,
 input wire display_clk,display_rst_n,scan_start,underrun,
 input wire byte_clk,byte_rst_n,input wire [3:0] ppi_sot,ppi_sync, input wire [608:0] diag_live,
 input wire cfg_clk,cfg_rst_n,config_done,config_error,input wire [191:0] registers,input wire [63:0] i2c_status,
 input wire trace_ready,input wire [127:0] trace_data,output wire [10:0] trace_addr,
 input wire trace_event_seen,trace_trigger_seen,input wire [1023:0] trace_event_context,input wire [255:0] trace_ring_info,
 input wire [31:0] official_good,official_bad,bridge_eof_count,bridge_word_count,bridge_flags,
 input wire official_adapter_bad,input wire [359:0] bridge_byte_live,
 input wire [511:0] display_pixel_live,input wire [1023:0] display_core_live,
 input wire [511:0] pingpong_camera_live,input wire [1023:0] pingpong_core_live,
 output reg txd
);
localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
reg [127:0] sot_count,sync_count,snap_sot,snap_sync;
reg [31:0] timer,scan_count,byte_count;
reg [3:0] request;reg pending;reg [19:0] timeout_count;
reg [7:0] snap_fps;reg snap_geometry,snap_source;
reg [31:0] snap_good,snap_bad;reg [191:0] snap_debug;
wire [2209:0] held_core;wire [576:0] held_scan;wire [1256:0] held_byte;wire [257:0] held_cfg;
wire ack_core,ack_scan,ack_byte,ack_cfg;
always @(posedge display_clk or negedge display_rst_n)
 if(!display_rst_n)scan_count<=0;else if(scan_start)scan_count<=scan_count+1'b1;
always @(posedge byte_clk or negedge byte_rst_n)
 if(!byte_rst_n)byte_count<=0;else byte_count<=byte_count+1'b1;
camera_rate_snapshot #(.W(2210)) u_core(.clk(core_clk),.rst_n(core_rst_n),.request(request[0]),.live({pingpong_core_live,display_core_live,csi_status,commits,write_bad,read_bad}),.held(held_core),.ack(ack_core));
camera_rate_snapshot #(.W(577)) u_scan(.clk(display_clk),.rst_n(display_rst_n),.request(request[1]),.live({display_pixel_live,scan_count,presents,underrun}),.held(held_scan),.ack(ack_scan));
camera_rate_snapshot #(.W(1257)) u_byte(.clk(byte_clk),.rst_n(byte_rst_n),.request(request[2]),.live({bridge_byte_live,diag_live,sync_count,sot_count,byte_count}),.held(held_byte),.ack(ack_byte));
camera_rate_snapshot #(.W(258)) u_cfg(.clk(cfg_clk),.rst_n(cfg_rst_n),.request(request[3]),.live({config_done,config_error,registers,i2c_status}),.held(held_cfg),.ack(ack_cfg));
(* async_reg="true" *) reg [3:0] ack1,ack2;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin ack1<=0;ack2<=0;end else begin ack1<={ack_cfg,ack_byte,ack_scan,ack_core};ack2<=ack1;end
wire [3:0] ack_now=~(ack2^request);
reg [127:0] snap_csi;
reg [3:0] sot_previous,sync_previous;
integer lane;
always @(posedge byte_clk or negedge byte_rst_n)
 if(!byte_rst_n)begin sot_count<=0;sync_count<=0;sot_previous<=0;sync_previous<=0;end
 else begin
  sot_previous<=ppi_sot;sync_previous<=ppi_sync;
  for(lane=0;lane<4;lane=lane+1)begin
   if(ppi_sot[lane]&&!sot_previous[lane])sot_count[lane*32+:32]<=sot_count[lane*32+:32]+1'b1;
   if(ppi_sync[lane]&&!sync_previous[lane])sync_count[lane*32+:32]<=sync_count[lane*32+:32]+1'b1;
  end
 end
reg [31:0] snap_commit,snap_present,snap_scan,snap_byte; reg [615:0] snap_cd;
reg [511:0] snap_pc;reg [1023:0] snap_pk;
reg [359:0] snap_bp;reg [511:0] snap_dp;reg [1023:0] snap_dc;
reg [31:0] snap_official_good,snap_official_bad,snap_bridge_eof,snap_bridge_words,snap_bridge_flags;reg snap_official_adapter_bad;
reg [191:0] snap_regs;reg [63:0] snap_i2c;reg [7:0] snap_flags;reg [3:0] snap_ack;
reg busy;reg [16:0] index;reg [3:0] bit_index;reg [15:0] baud_count;reg [9:0] serial_bits;
function [7:0] hx;input [3:0] n;begin hx=n<10?8'h30+n:8'h61+n-10;end endfunction
localparam integer TRACE_START=292,LENGTH=67347;
reg [31:0] seconds,snap_seconds;reg snap_trace_ready,snap_ev,snap_tg;reg [1023:0] snap_fc;reg [255:0] snap_ri;
wire [16:0] next_index=index+1'b1;
assign trace_addr=(next_index-TRACE_START)>>5;
function [7:0] character;input [16:0] at;begin case(at)
0:character=8'h43;
1:character=8'h36;
2:character=8'h30;
3:character=8'h20;
4:character=8'h66;
5:character=8'h70;
6:character=8'h73;
7:character=8'h3d;
8:character=hx(snap_fps>>4);
9:character=hx(snap_fps>>0);
10:character=8'h20;
11:character=8'h67;
12:character=8'h6f;
13:character=8'h6f;
14:character=8'h64;
15:character=8'h3d;
16:character=hx(snap_good>>28);
17:character=hx(snap_good>>24);
18:character=hx(snap_good>>20);
19:character=hx(snap_good>>16);
20:character=hx(snap_good>>12);
21:character=hx(snap_good>>8);
22:character=hx(snap_good>>4);
23:character=hx(snap_good>>0);
24:character=8'h20;
25:character=8'h62;
26:character=8'h61;
27:character=8'h64;
28:character=8'h3d;
29:character=hx(snap_bad>>28);
30:character=hx(snap_bad>>24);
31:character=hx(snap_bad>>20);
32:character=hx(snap_bad>>16);
33:character=hx(snap_bad>>12);
34:character=hx(snap_bad>>8);
35:character=hx(snap_bad>>4);
36:character=hx(snap_bad>>0);
37:character=8'h20;
38:character=8'h63;
39:character=8'h6f;
40:character=8'h6d;
41:character=8'h3d;
42:character=hx(snap_commit>>28);
43:character=hx(snap_commit>>24);
44:character=hx(snap_commit>>20);
45:character=hx(snap_commit>>16);
46:character=hx(snap_commit>>12);
47:character=hx(snap_commit>>8);
48:character=hx(snap_commit>>4);
49:character=hx(snap_commit>>0);
50:character=8'h20;
51:character=8'h70;
52:character=8'h72;
53:character=8'h65;
54:character=8'h73;
55:character=8'h3d;
56:character=hx(snap_present>>28);
57:character=hx(snap_present>>24);
58:character=hx(snap_present>>20);
59:character=hx(snap_present>>16);
60:character=hx(snap_present>>12);
61:character=hx(snap_present>>8);
62:character=hx(snap_present>>4);
63:character=hx(snap_present>>0);
64:character=8'h20;
65:character=8'h73;
66:character=8'h63;
67:character=8'h61;
68:character=8'h6e;
69:character=8'h3d;
70:character=hx(snap_scan>>28);
71:character=hx(snap_scan>>24);
72:character=hx(snap_scan>>20);
73:character=hx(snap_scan>>16);
74:character=hx(snap_scan>>12);
75:character=hx(snap_scan>>8);
76:character=hx(snap_scan>>4);
77:character=hx(snap_scan>>0);
78:character=8'h20;
79:character=8'h62;
80:character=8'h79;
81:character=8'h74;
82:character=8'h65;
83:character=8'h3d;
84:character=hx(snap_byte>>28);
85:character=hx(snap_byte>>24);
86:character=hx(snap_byte>>20);
87:character=hx(snap_byte>>16);
88:character=hx(snap_byte>>12);
89:character=hx(snap_byte>>8);
90:character=hx(snap_byte>>4);
91:character=hx(snap_byte>>0);
92:character=8'h20;
93:character=8'h72;
94:character=8'h62;
95:character=8'h3d;
96:character=hx(snap_regs>>188);
97:character=hx(snap_regs>>184);
98:character=hx(snap_regs>>180);
99:character=hx(snap_regs>>176);
100:character=hx(snap_regs>>172);
101:character=hx(snap_regs>>168);
102:character=hx(snap_regs>>164);
103:character=hx(snap_regs>>160);
104:character=hx(snap_regs>>156);
105:character=hx(snap_regs>>152);
106:character=hx(snap_regs>>148);
107:character=hx(snap_regs>>144);
108:character=hx(snap_regs>>140);
109:character=hx(snap_regs>>136);
110:character=hx(snap_regs>>132);
111:character=hx(snap_regs>>128);
112:character=hx(snap_regs>>124);
113:character=hx(snap_regs>>120);
114:character=hx(snap_regs>>116);
115:character=hx(snap_regs>>112);
116:character=hx(snap_regs>>108);
117:character=hx(snap_regs>>104);
118:character=hx(snap_regs>>100);
119:character=hx(snap_regs>>96);
120:character=hx(snap_regs>>92);
121:character=hx(snap_regs>>88);
122:character=hx(snap_regs>>84);
123:character=hx(snap_regs>>80);
124:character=hx(snap_regs>>76);
125:character=hx(snap_regs>>72);
126:character=hx(snap_regs>>68);
127:character=hx(snap_regs>>64);
128:character=hx(snap_regs>>60);
129:character=hx(snap_regs>>56);
130:character=hx(snap_regs>>52);
131:character=hx(snap_regs>>48);
132:character=hx(snap_regs>>44);
133:character=hx(snap_regs>>40);
134:character=hx(snap_regs>>36);
135:character=hx(snap_regs>>32);
136:character=hx(snap_regs>>28);
137:character=hx(snap_regs>>24);
138:character=hx(snap_regs>>20);
139:character=hx(snap_regs>>16);
140:character=hx(snap_regs>>12);
141:character=hx(snap_regs>>8);
142:character=hx(snap_regs>>4);
143:character=hx(snap_regs>>0);
144:character=8'h20;
145:character=8'h69;
146:character=8'h32;
147:character=8'h63;
148:character=8'h3d;
149:character=hx(snap_i2c>>60);
150:character=hx(snap_i2c>>56);
151:character=hx(snap_i2c>>52);
152:character=hx(snap_i2c>>48);
153:character=hx(snap_i2c>>44);
154:character=hx(snap_i2c>>40);
155:character=hx(snap_i2c>>36);
156:character=hx(snap_i2c>>32);
157:character=hx(snap_i2c>>28);
158:character=hx(snap_i2c>>24);
159:character=hx(snap_i2c>>20);
160:character=hx(snap_i2c>>16);
161:character=hx(snap_i2c>>12);
162:character=hx(snap_i2c>>8);
163:character=hx(snap_i2c>>4);
164:character=hx(snap_i2c>>0);
165:character=8'h20;
166:character=8'h66;
167:character=8'h6c;
168:character=8'h61;
169:character=8'h67;
170:character=8'h73;
171:character=8'h3d;
172:character=hx(snap_flags>>4);
173:character=hx(snap_flags>>0);
174:character=8'h20;
175:character=8'h61;
176:character=8'h63;
177:character=8'h6b;
178:character=8'h3d;
179:character=hx(snap_ack>>0);
180:character=8'h20;
181:character=8'h64;
182:character=8'h62;
183:character=8'h67;
184:character=8'h3d;
185:character=hx(snap_debug>>188);
186:character=hx(snap_debug>>184);
187:character=hx(snap_debug>>180);
188:character=hx(snap_debug>>176);
189:character=hx(snap_debug>>172);
190:character=hx(snap_debug>>168);
191:character=hx(snap_debug>>164);
192:character=hx(snap_debug>>160);
193:character=hx(snap_debug>>156);
194:character=hx(snap_debug>>152);
195:character=hx(snap_debug>>148);
196:character=hx(snap_debug>>144);
197:character=hx(snap_debug>>140);
198:character=hx(snap_debug>>136);
199:character=hx(snap_debug>>132);
200:character=hx(snap_debug>>128);
201:character=hx(snap_debug>>124);
202:character=hx(snap_debug>>120);
203:character=hx(snap_debug>>116);
204:character=hx(snap_debug>>112);
205:character=hx(snap_debug>>108);
206:character=hx(snap_debug>>104);
207:character=hx(snap_debug>>100);
208:character=hx(snap_debug>>96);
209:character=hx(snap_debug>>92);
210:character=hx(snap_debug>>88);
211:character=hx(snap_debug>>84);
212:character=hx(snap_debug>>80);
213:character=hx(snap_debug>>76);
214:character=hx(snap_debug>>72);
215:character=hx(snap_debug>>68);
216:character=hx(snap_debug>>64);
217:character=hx(snap_debug>>60);
218:character=hx(snap_debug>>56);
219:character=hx(snap_debug>>52);
220:character=hx(snap_debug>>48);
221:character=hx(snap_debug>>44);
222:character=hx(snap_debug>>40);
223:character=hx(snap_debug>>36);
224:character=hx(snap_debug>>32);
225:character=hx(snap_debug>>28);
226:character=hx(snap_debug>>24);
227:character=hx(snap_debug>>20);
228:character=hx(snap_debug>>16);
229:character=hx(snap_debug>>12);
230:character=hx(snap_debug>>8);
231:character=hx(snap_debug>>4);
232:character=hx(snap_debug>>0);
233:character=8'h20;
234:character=8'h63;
235:character=8'h73;
236:character=8'h69;
237:character=8'h3d;
238:character=hx(snap_csi>>124);
239:character=hx(snap_csi>>120);
240:character=hx(snap_csi>>116);
241:character=hx(snap_csi>>112);
242:character=hx(snap_csi>>108);
243:character=hx(snap_csi>>104);
244:character=hx(snap_csi>>100);
245:character=hx(snap_csi>>96);
246:character=hx(snap_csi>>92);
247:character=hx(snap_csi>>88);
248:character=hx(snap_csi>>84);
249:character=hx(snap_csi>>80);
250:character=hx(snap_csi>>76);
251:character=hx(snap_csi>>72);
252:character=hx(snap_csi>>68);
253:character=hx(snap_csi>>64);
254:character=hx(snap_csi>>60);
255:character=hx(snap_csi>>56);
256:character=hx(snap_csi>>52);
257:character=hx(snap_csi>>48);
258:character=hx(snap_csi>>44);
259:character=hx(snap_csi>>40);
260:character=hx(snap_csi>>36);
261:character=hx(snap_csi>>32);
262:character=hx(snap_csi>>28);
263:character=hx(snap_csi>>24);
264:character=hx(snap_csi>>20);
265:character=hx(snap_csi>>16);
266:character=hx(snap_csi>>12);
267:character=hx(snap_csi>>8);
268:character=hx(snap_csi>>4);
269:character=hx(snap_csi>>0);
270:character=" ";271:character="t";272:character="r";273:character="=";
274:character=snap_trace_ready?"1":"0";275:character=" ";276:character="t";277:character="m";278:character="=";
279:character=hx(snap_seconds>>28);280:character=hx(snap_seconds>>24);281:character=hx(snap_seconds>>20);282:character=hx(snap_seconds>>16);
283:character=hx(snap_seconds>>12);284:character=hx(snap_seconds>>8);285:character=hx(snap_seconds>>4);286:character=hx(snap_seconds);
287:character=" ";288:character="r";289:character="a";290:character="w";291:character="=";
65828:character=" ";65829:character="s";65830:character="o";65831:character="t";65832:character="=";
65865:character=" ";65866:character="s";65867:character="y";65868:character="n";65869:character="=";
65902:character=" ";65903:character="c";65904:character="d";65905:character="=";66060:character=" ";
66061:character="s";
66062:character="m";
66063:character="=";
66064:character="1";
66065:character=" ";
66066:character="o";
66067:character="g";
66068:character="=";
66077:character=" ";
66078:character="o";
66079:character="b";
66080:character="=";
66089:character=" ";
66090:character="o";
66091:character="a";
66092:character="=";
66093:character=snap_official_adapter_bad?"1":"0";
66094:character=" ";
66095:character="b";
66096:character="e";
66097:character="=";
66106:character=" ";
66107:character="b";
66108:character="w";
66109:character="=";
66118:character=" ";
66119:character="b";
66120:character="f";
66121:character="=";
66130:character=" ";66131:character="b";66132:character="p";66133:character="=";66224:character=" ";66225:character="d";66226:character="p";66227:character="=";
66356:character=" ";66357:character="d";66358:character="c";66359:character="=";66616:character=" ";66617:character="p";66618:character="c";66619:character="=";
66748:character=" ";66749:character="p";66750:character="k";66751:character="=";67008:character=" ";67009:character="e";67010:character="v";67011:character="=";67012:character=snap_ev?"1":"0";
67013:character=" ";67014:character="t";67015:character="g";67016:character="=";67017:character=snap_tg?"1":"0";
67018:character=" ";67019:character="f";67020:character="c";67021:character="=";
67278:character=" ";67279:character="r";67280:character="i";67281:character="=";67346:character=8'h0a;
default:begin
 if(at>=67022&&at<67278)character=hx(snap_fc>>((67277-at)*4));
 else if(at>=67282&&at<67346)character=hx(snap_ri>>((67345-at)*4));
 else  if(at>=66620&&at<66748)character=hx(snap_pc>>((66747-at)*4));
 else if(at>=66752&&at<67008)character=hx(snap_pk>>((67007-at)*4));
 else if(at>=66228&&at<66356)character=hx(snap_dp>>((66355-at)*4));
 else if(at>=66360&&at<66616)character=hx(snap_dc>>((66615-at)*4));
 else if(at>=66134&&at<66224)character=hx(snap_bp>>((66223-at)*4));
 else if(at>=66069&&at<66077)character=hx(snap_official_good>>((66076-at)*4));
 else if(at>=66081&&at<66089)character=hx(snap_official_bad>>((66088-at)*4));
 else if(at>=66098&&at<66106)character=hx(snap_bridge_eof>>((66105-at)*4));
 else if(at>=66110&&at<66118)character=hx(snap_bridge_words>>((66117-at)*4));
 else if(at>=66122&&at<66130)character=hx(snap_bridge_flags>>((66129-at)*4));
 else if(at>=65906&&at<66060)character=hx(snap_cd>>((66059-at)*4));
 else if(at>=65833&&at<65865)character=hx(snap_sot>>((65864-at)*4));
 else if(at>=65870&&at<65902)character=hx(snap_sync>>((65901-at)*4));
 else if(at>=TRACE_START&&at<65828)character=snap_trace_ready?hx(trace_data>>((31-((at-TRACE_START)%32))*4)):8'h30;
 else character=8'h20;
end endcase end endfunction
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 seconds<=0;snap_seconds<=0;snap_trace_ready<=0;snap_ev<=0;snap_tg<=0;snap_fc<=0;snap_ri<=0;timer<=0;request<=0;pending<=0;timeout_count<=0;txd<=1;busy<=0;index<=0;bit_index<=0;baud_count<=0;serial_bits<=10'h3ff;
 snap_pc<=0;snap_pk<=0;snap_dp<=0;snap_dc<=0;snap_bp<=0;snap_official_good<=0;snap_official_bad<=0;snap_bridge_eof<=0;snap_bridge_words<=0;snap_bridge_flags<=0;snap_official_adapter_bad<=0;snap_cd<=0;snap_sot<=0;snap_sync<=0;snap_csi<=0;snap_debug<=0;snap_fps<=0;snap_geometry<=0;snap_source<=0;snap_good<=0;snap_bad<=0;snap_commit<=0;snap_present<=0;snap_scan<=0;snap_byte<=0;snap_regs<=0;snap_i2c<=0;snap_flags<=0;snap_ack<=0;
 end else begin
 if(timer==CLK_HZ-1)begin
 timer<=0;seconds<=seconds+1'b1;
 if(!pending&&!busy)begin
// A timed-out domain retains its outstanding request. Toggling it again
// would make its old ACK equal the request and falsely report fresh data.
request<=request^ack_now;pending<=1;timeout_count<=0;snap_seconds<=seconds+1'b1;snap_trace_ready<=trace_ready;snap_ev<=trace_event_seen;snap_tg<=trace_trigger_seen;snap_fc<=trace_event_context;snap_ri<=trace_ring_info;
 snap_pc<=pingpong_camera_live;
 snap_official_good<=official_good;snap_official_bad<=official_bad;snap_official_adapter_bad<=official_adapter_bad;
 snap_bridge_eof<=bridge_eof_count;snap_bridge_words<=bridge_word_count;snap_bridge_flags<=bridge_flags;
 snap_debug<=capture_debug;snap_fps<=fps_bcd;snap_geometry<=geometry_valid;snap_source<=source_fault;snap_good<=complete_frames;snap_bad<=bad_frames;
 end
 end else timer<=timer+1'b1;
 if(pending)begin
 timeout_count<=timeout_count+1'b1;
 if((&ack_now)||timeout_count==CLK_HZ/100-1)begin
 pending<=0;snap_ack<=ack_now;
snap_pk<=ack_now[0]?held_core[2209:1186]:1024'd0;
snap_dc<=ack_now[0]?held_core[1185:162]:1024'd0;
snap_dp<=ack_now[1]?held_scan[576:65]:512'd0;
snap_csi<=ack_now[0]?held_core[161:34]:128'd0;
snap_commit<=ack_now[0]?held_core[33:2]:32'd0;snap_present<=ack_now[1]?held_scan[32:1]:32'd0;
snap_scan<=ack_now[1]?held_scan[64:33]:32'd0;snap_byte<=ack_now[2]?held_byte[31:0]:32'd0;
snap_sot<=ack_now[2]?held_byte[159:32]:128'd0;snap_sync<=ack_now[2]?held_byte[287:160]:128'd0;
snap_cd<=ack_now[2]?{7'd0,held_byte[896:288]}:616'd0;
snap_bp<=ack_now[2]?held_byte[1256:897]:360'd0;
snap_regs<=ack_now[3]?held_cfg[255:64]:192'd0;snap_i2c<=ack_now[3]?held_cfg[63:0]:64'd0;
snap_flags<={1'b0,snap_source,(held_cfg[257:256]&{2{ack_now[3]}}),(held_core[1:0]&{2{ack_now[0]}}),(held_scan[0]&ack_now[1]),snap_geometry};
 busy<=1;index<=0;bit_index<=0;baud_count<=DIV-1;serial_bits<={1'b1,8'h43,1'b0};txd<=0;
 end
 end
 if(busy)begin
 if(baud_count==0)begin
 baud_count<=DIV-1;
 if(bit_index==9)begin
 if(index==LENGTH-1)begin busy<=0;txd<=1;end
 else begin index<=index+1'b1;serial_bits<={1'b1,character(index+1'b1),1'b0};bit_index<=0;txd<=0;end
 end else begin serial_bits<={1'b1,serial_bits[9:1]};txd<=serial_bits[1];bit_index<=bit_index+1'b1;end
 end else baud_count<=baud_count-1'b1;
 end
 end
end
endmodule
module camera_rate_snapshot #(parameter integer W=32)(input wire clk,rst_n,request,input wire [W-1:0] live,output reg [W-1:0] held,output reg ack);
(* async_reg="true" *) reg req1,req2;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin req1<=0;req2<=0;held<=0;ack<=0;end
 else begin req1<=request;req2<=req1;if(req2!=ack)begin held<=live;ack<=req2;end end
end
endmodule
`default_nettype wire

