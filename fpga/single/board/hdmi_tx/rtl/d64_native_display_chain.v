`timescale 1ns/1ps
// Continuous HDMI transport. SCALE1 centers the native network image; positive
// integer SCALE repeats pixels to enlarge it without changing CNN geometry/FOV.
// publish_* belongs to clk_axi. Payload remains held until acknowledgement
// from the next pixel-frame boundary. active_base/fid belong to clk_axi.
module d64_native_display_chain #(
    parameter integer FAULT_STICKY = 0,
    parameter integer INTERACTION_ENABLE = 0,
    parameter integer WIPE_ENABLE = 0, REMOTE_DEBUG_ENABLE=0,
    parameter integer IMG_W=256, IMG_H=144,
    parameter integer SCALE=1,
    parameter integer BILINEAR_ENABLE=0, DST_W=0, DST_H=0,
    parameter integer GUIDE_ENABLE=0,GUIDE_STORED_H=1080,
    parameter integer FPS_OSD_ENABLE=0, FPS_DEFAULT_ON=1, FPS_CLK_HZ=100000000,
    parameter integer H_ACT=1280, H_FP=1760, H_SYNC=40, H_BP=220,
    parameter integer V_ACT=720, V_FP=5, V_SYNC=5, V_BP=20,
    parameter integer TIMEOUT=1024,
    parameter [31:0] INITIAL_BASE=32'h005c0000
) (
    input wire clk_pix, rst_pix_n, clk_axi, rst_axi_n,
    input wire publish_valid,input wire camera_complete_event,
    input wire fps_toggle,remote_fps_set_valid,remote_fps_set_value,
    output wire fps_requested_pe,output wire [5:0] remote_actual_state_pe,
    output wire publish_ready,
    input wire [31:0] publish_base, publish_fid,
    input wire publish_guide_valid,
    input wire publish_original_valid,
    input wire [31:0] publish_guide_fid,
    input wire [2:0] publish_guide_slot,input wire [1:0] guide_strength,
    output reg [31:0] active_base, active_fid,
    output reg active_valid,
    output wire frame_start_pix,
    output reg frame_start_axi,
    output wire comparison_publication_window_axi,
    output reg first_scan_axi,
    output reg first_scan_pix,
    output reg [31:0] scan_fid_axi,
    output wire vblank_pix,
    output wire [9:0] tmds_data0, tmds_data1, tmds_data2, tmds_clk,
    output wire axi_arvalid,
    input wire axi_arready,
    output wire [31:0] axi_araddr,
    output wire [7:0] axi_arlen,
    output wire [2:0] axi_arsize,
    output wire [1:0] axi_arburst,
    input wire axi_rvalid,
    output wire axi_rready,
    input wire [127:0] axi_rdata,
    input wire axi_rlast,
    input wire [1:0] axi_rresp,
    output wire [31:0] rows_loaded, underrun, axi_errors, bad_rows,
    output wire starved,
    output wire [31:0] guide_faults,
    output wire guide_reader_axi_fault,guide_reader_protocol_fault,
    output wire guide_reader_outstanding,output wire [2:0] guide_reader_slot,
    output wire [31:0] guide_araddr,output wire [7:0] guide_arlen,
    output wire [2:0] guide_arsize,output wire [1:0] guide_arburst,
    output wire guide_arvalid,input wire guide_arready,
    input wire [127:0] guide_rdata,input wire guide_rvalid,output wire guide_rready,
    input wire guide_rlast,input wire [1:0] guide_rresp,
    input wire compare_enabled_pe,effect_enabled_pe,
    input wire [1:0] display_mode_pe,
    input wire comparison_publish_valid,output wire comparison_publish_ready,
    input wire [31:0] comparison_original_base,comparison_half_base,comparison_fid,
    input wire [15:0] comparison_epoch,input wire [2:0] comparison_slot,
    output wire [4:0] comparison_protect_mask_pe,
    output wire [31:0] comparison_araddr,output wire [7:0] comparison_arlen,
    output wire [2:0] comparison_arsize,output wire [1:0] comparison_arburst,
    output wire comparison_arvalid,input wire comparison_arready,
    input wire [127:0] comparison_rdata,input wire comparison_rvalid,
    output wire comparison_rready,input wire comparison_rlast,input wire [1:0] comparison_rresp,
    output wire [31:0] comparison_faults,output wire [7:0] processing_fps,
    output wire full_render_de,full_render_good,output wire [23:0] full_render_rgb,
    output wire [11:0] full_render_ax,full_render_ay,output wire [31:0] full_render_fid,
    output wire [15:0] full_render_epoch,output wire [2:0] full_render_guide_slot,
    output wire full_render_slot_valid
);
    localparam integer WIN_W=(DST_W>0)?DST_W:IMG_W*SCALE;
    localparam integer WIN_H=(DST_H>0)?DST_H:IMG_H*SCALE;
    localparam integer IMAGE_X0=(H_ACT-WIN_W)/2,IMAGE_Y0=(V_ACT-WIN_H)/2;
    reg req_tog, ack_tog;
    reg [31:0] held_base, held_fid;
    reg held_guide_valid,pix_guide_valid;
    reg held_original_valid,pix_original_valid;
    reg [31:0] held_guide_fid,pix_guide_fid;
    reg [2:0] held_guide_slot,pix_guide_slot;
    (* async_reg="true" *) reg [1:0] ack_s, req_s;
    reg ack_seen;
    reg [31:0] pix_base, pix_fid;
    reg pix_valid, frame_tog;
    (* async_reg="true" *) reg [1:0] frame_s;
    reg frame_seen;
    reg scan_tog;
    reg [31:0] scan_fid_pix;
    (* async_reg="true" *) reg [1:0] scan_s;
    reg scan_seen;
    assign publish_ready = ack_s[1] == req_tog;
    always @(posedge clk_axi or negedge rst_axi_n) begin
        if (!rst_axi_n) begin
            req_tog<=0; held_base<=INITIAL_BASE; held_fid<=0;
            ack_s<=0; ack_seen<=0; active_base<=INITIAL_BASE;
            active_fid<=0; active_valid<=0;
            held_guide_valid<=0;held_guide_slot<=0;held_guide_fid<=0;
            held_original_valid<=0;
            frame_s<=0; frame_seen<=0; frame_start_axi<=0;
            scan_s<=0; scan_seen<=0; first_scan_axi<=0; scan_fid_axi<=0;
        end else begin
            ack_s <= {ack_s[0],ack_tog};
            scan_s <= {scan_s[0],scan_tog};
            first_scan_axi <= scan_s[1]!=scan_seen;
            if(scan_s[1]!=scan_seen) begin
                scan_seen<=scan_s[1]; scan_fid_axi<=scan_fid_pix;
            end
            frame_s <= {frame_s[0],frame_tog};
            frame_start_axi <= frame_s[1] != frame_seen;
            if(frame_s[1] != frame_seen) frame_seen <= frame_s[1];
            if(publish_valid && publish_ready) begin
                held_base<=publish_base; held_fid<=publish_fid;
                held_guide_valid<=publish_guide_valid;held_guide_slot<=publish_guide_slot;
                held_guide_fid<=publish_guide_fid;
                held_original_valid<=WIPE_ENABLE!=0&&publish_original_valid&&publish_guide_valid;
                req_tog<=!req_tog;
            end
            if(ack_s[1] != ack_seen) begin
                ack_seen<=ack_s[1]; active_base<=held_base;
                active_fid<=held_fid; active_valid<=1;
            end
        end
    end
    wire hs,vs,de,frame;
    wire [11:0] x,y,ax,ay;
    d64_video_timing #(.H_ACT(H_ACT),.H_FP(H_FP),.H_SYNC(H_SYNC),.H_BP(H_BP),
        .V_ACT(V_ACT),.V_FP(V_FP),.V_SYNC(V_SYNC),.V_BP(V_BP)) u_timing(
        .clk(clk_pix),.rst_n(rst_pix_n),.hs(hs),.vs(vs),.de(de),
        .frame(frame),.x(x),.y(y),.ax(ax),.ay(ay));
    assign frame_start_pix=frame;
    assign vblank_pix=(y<V_SYNC+V_BP)||(y>=V_SYNC+V_BP+V_ACT);
    // Candidate: publish complete comparison pairs before the PIX boundary,
    // during the final vertical front porch. Pending protection is still
    // synchronized back to PIX well before the following active-frame SOF.
    wire comparison_preboundary_pix=(y>=V_SYNC+V_BP+V_ACT);
    (* async_reg="true" *) reg [1:0] comparison_window_sync;
    always @(posedge clk_axi or negedge rst_axi_n)
        if(!rst_axi_n)comparison_window_sync<=0;
        else comparison_window_sync<={comparison_window_sync[0],comparison_preboundary_pix};
    assign comparison_publication_window_axi=comparison_window_sync[1];
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if(!rst_pix_n) begin
            req_s<=0; ack_tog<=0; pix_base<=INITIAL_BASE;
            pix_fid<=0; pix_valid<=0; frame_tog<=0;
            pix_guide_valid<=0;pix_guide_slot<=0;pix_guide_fid<=0;
            pix_original_valid<=0;
        end else begin
            req_s<={req_s[0],req_tog};
            if(frame) begin
                frame_tog<=!frame_tog;
                if(req_s[1] != ack_tog) begin
                    pix_base<=held_base; pix_fid<=held_fid;
                    pix_valid<=1; ack_tog<=req_s[1];
                    pix_guide_valid<=held_guide_valid;pix_guide_slot<=held_guide_slot;
                    pix_guide_fid<=held_guide_fid;
                    pix_original_valid<=held_original_valid;
                end
            end
        end
    end
    wire load_req,row_ready,row_good,load_we;
    wire [7:0] load_row,load_wi;
    wire [23:0] load_wd;
    wire sw_we;
    wire [7:0] sw_wi;
    wire [23:0] sw_wd;
    d64_video_row_ddr #(
        .FAULT_STICKY(FAULT_STICKY),.SRC_W(IMG_W),.TIMEOUT(TIMEOUT),.SRC_BASE(INITIAL_BASE)) u_reader(
        .clk_pe(clk_pix),.rst_pe_n(rst_pix_n),.load_req(load_req && pix_valid && load_row<IMG_H),
        .load_row(load_row),.row_ready(row_ready),.row_good(row_good),.load_we(load_we),
        .load_wi(load_wi),.load_wd(load_wd),.frame_id(pix_fid[1:0]),
        .src_base(pix_base),.frame_tick(frame),.clk_axi(clk_axi),.arst_axi_n(rst_axi_n),
        .axi_arvalid(axi_arvalid),.axi_arready(axi_arready),.axi_araddr(axi_araddr),
        .axi_arlen(axi_arlen),.axi_arsize(axi_arsize),.axi_arburst(axi_arburst),
        .axi_rvalid(axi_rvalid),.axi_rready(axi_rready),.axi_rdata(axi_rdata),
        .axi_rlast(axi_rlast),.axi_rresp(axi_rresp),.rows_loaded(rows_loaded),
        .stale_beats(),.underrun(underrun),.axi_errors(axi_errors),
        .bad_rows(bad_rows),.first_bad_row(),.page_refusals());
    d64_rgbx_display_adapter u_rgb_order(
        .load_we_i(load_we),.load_wi_i(load_wi),.load_wd_i(load_wd),
        .load_we_o(sw_we),.load_wi_o(sw_wi),.load_wd_o(sw_wd));
    wire shs,svs,sde;
    wire [23:0] rgb;
    wire starved_raw,row_swap_attempt,row_swap_miss,pixel_good;
    wire [11:0] rgb_ax,rgb_ay;
    reg [11:0] scan_ax1,scan_ay1,scan_ax2,scan_ay2;
    generate if(BILINEAR_ENABLE!=0) begin:g_bilinear
    d64_bilinear_scaler #(.SRC_W(IMG_W),.SRC_H(IMG_H),.WIN_W(WIN_W),.WIN_H(WIN_H),
        .X0(IMAGE_X0),.Y0(IMAGE_Y0)) u_bilinear(
        .clk(clk_pix),.rst_n(rst_pix_n),.source_valid_i(pix_valid),
        .hs_i(hs),.vs_i(vs),.de_i(de),.frame_i(frame),.ax_i(ax),.ay_i(ay),
        .load_req(load_req),.load_row(load_row),.row_ready(row_ready),.row_good(row_good),
        .load_we(sw_we),.load_wi(sw_wi),.load_wd(sw_wd),
        .hs_o(shs),.vs_o(svs),.de_o(sde),.rgb_o(rgb),.pixel_good_o(pixel_good),
        .ax_o(rgb_ax),.ay_o(rgb_ay),.starved(starved_raw),
        .row_swap_attempt(row_swap_attempt),.row_swap_miss(row_swap_miss));
    end else begin:g_nearest
    d64_video_scaler #(.SRC_W(IMG_W),.SRC_H(IMG_H),.WIN_W(WIN_W),.WIN_H(WIN_H),
        .X0(IMAGE_X0),.Y0(IMAGE_Y0),
        .SOURCE_ROW_REPEAT((SCALE>1)?SCALE:0),.CHECK_ROW_GOOD(1)) u_native(
        .clk(clk_pix),.rst_n(rst_pix_n),.hs_i(hs),.vs_i(vs),.de_i(de),
        .frame_i(frame),.ax_i(ax),.ay_i(ay),.load_req(load_req),
        .load_row(load_row),.row_ready(row_ready),.row_good(row_good),.load_we(sw_we),
        .load_wi(sw_wi),.load_wd(sw_wd),.hs_o(shs),.vs_o(svs),.de_o(sde),
        .rgb_o(rgb),.pixel_good_o(pixel_good),.starved(starved_raw),
        .row_swap_attempt(row_swap_attempt),.row_swap_miss(row_swap_miss));
    assign rgb_ax=scan_ax2;
    assign rgb_ay=scan_ay2;
    end endgenerate
    reg visible_starved;
    always @(posedge clk_pix or negedge rst_pix_n)
        if(!rst_pix_n) visible_starved<=0;
        else if(pix_valid && row_swap_miss)
            visible_starved<=1;
    assign starved=visible_starved;
    wire render_hs,render_vs,render_de,render_good;
    wire [31:0] render_fid;
    wire [23:0] render_rgb;
    wire [11:0] render_ax,render_ay;
    generate if(GUIDE_ENABLE!=0)begin:g_guide_filter
        wire [1:0] strength_gray=guide_strength^(guide_strength>>1);
        (* async_reg="true" *) reg [1:0] strength_sync1,strength_sync2;
        reg [1:0] frame_strength;
        always @(posedge clk_pix or negedge rst_pix_n)
            if(!rst_pix_n)begin strength_sync1<=0;strength_sync2<=0;frame_strength<=0;end
            else begin
                strength_sync1<=strength_gray;strength_sync2<=strength_sync1;
                if(frame)frame_strength<={strength_sync2[1],strength_sync2[1]^strength_sync2[0]};
            end
        d64_guide_display_filter #(.W(WIN_W),.H(WIN_H),.STORED_H(GUIDE_STORED_H),.CACHE_PIXELS(4)) u_filter(
            .clk_axi(clk_axi),.rst_axi_n(rst_axi_n),.clk_pix(clk_pix),.rst_pix_n(rst_pix_n),
            .frame_i(frame),.hs_i(shs),.vs_i(svs),.de_i(sde),.style_good_i(pixel_good&&pix_valid),
            .style_rgb_i(rgb),.ax_i(rgb_ax),.ay_i(rgb_ay),.fid_i(pix_guide_fid),
            .guide_slot_i(pix_guide_slot),.guide_slot_valid_i(pix_guide_valid),.strength_i(frame_strength),
            .hs_o(render_hs),.vs_o(render_vs),.de_o(render_de),.good_o(render_good),.rgb_o(render_rgb),
            .ax_o(render_ax),.ay_o(render_ay),.fid_o(render_fid),.mismatch_faultcounter(guide_faults),
            .reader_axi_fault(guide_reader_axi_fault),.reader_protocol_fault(guide_reader_protocol_fault),
            .outstanding_request_axi(guide_reader_outstanding),.outstanding_request_slot_axi(guide_reader_slot),
            .axi_araddr(guide_araddr),.axi_arlen(guide_arlen),.axi_arsize(guide_arsize),.axi_arburst(guide_arburst),
            .axi_arvalid(guide_arvalid),.axi_arready(guide_arready),.axi_rdata(guide_rdata),
            .axi_rvalid(guide_rvalid),.axi_rready(guide_rready),.axi_rlast(guide_rlast),.axi_rresp(guide_rresp));
    end else begin:g_no_guide
        assign render_hs=shs;assign render_vs=svs;assign render_de=sde;assign render_good=pixel_good;
        assign render_rgb=rgb;assign render_ax=rgb_ax;assign render_ay=rgb_ay;
        assign render_fid=pix_guide_fid;
        assign guide_faults=0;assign guide_reader_axi_fault=0;assign guide_reader_protocol_fault=0;
        assign guide_reader_outstanding=0;assign guide_reader_slot=0;
        assign guide_araddr=0;assign guide_arlen=0;assign guide_arsize=4;assign guide_arburst=1;
        assign guide_arvalid=0;assign guide_rready=1;
    end endgenerate
    // Both modes carry the same raster latency: bilinear3/nearest2, plus
    // guide3 when enabled. The comparison reader/compositor is2 registered
    // stages, so only its output receives the difference in delay.
    localparam integer FULL_STAGES=(BILINEAR_ENABLE!=0?3:2)+(GUIDE_ENABLE!=0?3:0);
    localparam integer COMPARISON_DELAY=FULL_STAGES-2;
    (* async_reg="true" *) reg [1:0] effect_sync;
    reg effect_frame;
    reg [15:0] epoch_pipe[0:FULL_STAGES-1];
    reg [2:0] slot_pipe[0:FULL_STAGES-1];
    reg valid_pipe[0:FULL_STAGES-1];
    integer meta_index;
    always @(posedge clk_pix or negedge rst_pix_n)begin
        if(!rst_pix_n)begin
            effect_sync<=0;effect_frame<=0;
            for(meta_index=0;meta_index<FULL_STAGES;meta_index=meta_index+1)begin
                epoch_pipe[meta_index]<=0;slot_pipe[meta_index]<=0;valid_pipe[meta_index]<=0;
            end
        end else begin
            effect_sync<={effect_sync[0],INTERACTION_ENABLE!=0&&effect_enabled_pe};
            if(frame)effect_frame<=effect_sync[1];
            epoch_pipe[0]<=pix_fid[31:16];slot_pipe[0]<=pix_guide_slot;
            valid_pipe[0]<=pix_guide_valid&&pix_valid;
            for(meta_index=1;meta_index<FULL_STAGES;meta_index=meta_index+1)begin
                epoch_pipe[meta_index]<=epoch_pipe[meta_index-1];
                slot_pipe[meta_index]<=slot_pipe[meta_index-1];valid_pipe[meta_index]<=valid_pipe[meta_index-1];
            end
        end
    end
    wire [23:0] full_source=pix_valid?(effect_frame&&render_de&&render_good?~render_rgb:render_rgb):24'd0;
    assign full_render_de=render_de;assign full_render_good=render_good&&pix_valid;
    // Stored comparison pixels are the immutable half-fusion result. Live
    // presentation effects are applied below, so a retained pair can respond
    // to KEY3/expiry without rewriting its protected half plane.
    assign full_render_rgb=pix_valid?render_rgb:24'd0;
    assign full_render_ax=render_ax;assign full_render_ay=render_ay;
    assign full_render_fid=render_fid;assign full_render_epoch=epoch_pipe[FULL_STAGES-1];
    assign full_render_guide_slot=slot_pipe[FULL_STAGES-1];
    assign full_render_slot_valid=valid_pipe[FULL_STAGES-1];
    wire display_hs,display_vs,display_de,display_good;
    wire [23:0] source_rgb;wire [11:0] display_ax,display_ay;
    wire compare_selected;wire [11:0] visible_first_y;wire [31:0] visible_scan_fid;
    wire [1:0] remote_accepted_mode_pix,remote_actual_mode_pix;
    generate if(INTERACTION_ENABLE!=0)begin:g_interaction
        reg cmp_request,cmp_ack,cmp_ack_seen;
        reg [31:0] held_original,held_half,held_cfid;
        reg [15:0] held_cepoch;reg [2:0] held_cslot;
        (* async_reg="true" *) reg [1:0] cmp_ack_sync,cmp_req_sync,cmp_enable_sync,cmp_mode_back;
        reg cmp_mode,cmp_valid,cmp_current_valid_pe;
        reg wipe_mode;
        // Three display states cannot form a cyclic two-bit Gray code.
        // Hold the mode until a pixel-frame boundary acknowledges it. Rapid
        // presses leave the latest requested state queued in the PE domain.
        reg [1:0] mode_held,mode_selected;
        reg mode_request,mode_ack;
        (* async_reg="true" *) reg [1:0] mode_ack_sync,mode_req_sync;
        reg [31:0] cmp_original,cmp_half,cmp_fid;
        reg [15:0] cmp_epoch;reg [2:0] cmp_slot,cmp_current_slot_pe;
        wire cmp_reader_outstanding;wire [2:0] cmp_reader_slot;
        wire [31:0] cmp_deadlines;wire cmp_axi_fault,cmp_protocol_fault;
        (* async_reg="true" *) reg [1:0] deadline_sync;
        assign remote_accepted_mode_pix=WIPE_ENABLE!=0?mode_selected:{1'b0,cmp_mode};
        assign remote_actual_mode_pix=wipe_mode?2'd2:(cmp_mode?2'd1:2'd0);
        assign comparison_publish_ready=cmp_ack_sync[1]==cmp_request;
        wire protect_transition=compare_enabled_pe||cmp_mode_back[1]||
            (WIPE_ENABLE!=0&&mode_held==1);
        wire protect_pending=(cmp_request!=cmp_ack_sync[1])||(cmp_ack_sync[1]!=cmp_ack_seen);
        assign comparison_protect_mask_pe=
            // A pending pair remains immutable even if a newer display-mode
            // request is still crossing to PIX. PIX may accept that pair on
            // the intervening boundary before seeing the new mode.
            (protect_pending?(5'b1<<held_cslot):5'd0)|
            (protect_transition&&cmp_current_valid_pe?(5'b1<<cmp_current_slot_pe):5'd0)|
            (cmp_reader_outstanding?(5'b1<<cmp_reader_slot):5'd0);
        assign comparison_faults={29'd0,cmp_axi_fault,cmp_protocol_fault,deadline_sync[1]};
        always @(posedge clk_axi or negedge rst_axi_n)begin
            if(!rst_axi_n)begin
                cmp_request<=0;cmp_ack_sync<=0;cmp_ack_seen<=0;cmp_mode_back<=0;deadline_sync<=0;
                held_original<=0;held_half<=0;held_cfid<=0;held_cepoch<=0;held_cslot<=0;
                cmp_current_valid_pe<=0;cmp_current_slot_pe<=0;
                mode_held<=0;mode_request<=0;mode_ack_sync<=0;
            end else begin
                mode_ack_sync<={mode_ack_sync[0],mode_ack};
                if(WIPE_ENABLE!=0&&mode_ack_sync[1]==mode_request&&mode_held!=display_mode_pe)begin
                    mode_held<=display_mode_pe<3?display_mode_pe:2'd0;
                    mode_request<=~mode_request;
                end
                cmp_ack_sync<={cmp_ack_sync[0],cmp_ack};cmp_mode_back<={cmp_mode_back[0],cmp_mode};
                deadline_sync<={deadline_sync[0],cmp_deadlines!=0};
                if(!cmp_mode_back[1])cmp_current_valid_pe<=0;
                if(comparison_publish_valid&&comparison_publish_ready)begin
                    held_original<=comparison_original_base;held_half<=comparison_half_base;
                    held_cfid<=comparison_fid;held_cepoch<=comparison_epoch;held_cslot<=comparison_slot;
                    cmp_request<=~cmp_request;
                end
                if(cmp_ack_sync[1]!=cmp_ack_seen)begin
                    cmp_ack_seen<=cmp_ack_sync[1];cmp_current_slot_pe<=held_cslot;
                    cmp_current_valid_pe<=cmp_valid;
                end
            end
        end
        // Select one complete source for the next raster at the PIX boundary.
        // A style publication can be applied on this same edge, so compare the
        // incoming descriptor's epoch when pending, otherwise the current one.
        // Keep the user's comparison preference; an obsolete pair falls back to
        // actual full-render pixels until a complete pair for that epoch arrives.
        wire full_pending_pix=(req_s[1]!=ack_tog);
        wire [15:0] next_full_epoch=full_pending_pix?held_fid[31:16]:pix_fid[31:16];
        wire next_full_valid=full_pending_pix||pix_valid;
        wire next_original_valid=full_pending_pix?held_original_valid:pix_original_valid;
        wire mode_pending_pix=mode_req_sync[1]!=mode_ack;
        wire [1:0] next_display_mode=WIPE_ENABLE!=0?
            (mode_pending_pix?mode_held:mode_selected):(cmp_enable_sync[1]?2'd1:2'd0);
        wire pair_pending_pix=(cmp_req_sync[1]!=cmp_ack);
        wire [15:0] next_pair_epoch=pair_pending_pix?held_cepoch:cmp_epoch;
        wire next_pair_valid=pair_pending_pix||cmp_valid;
        wire next_compare_mode=next_display_mode==1&&next_full_valid&&
            next_pair_valid&&(next_pair_epoch==next_full_epoch);
        wire next_wipe_mode=WIPE_ENABLE!=0&&next_display_mode==2&&next_full_valid&&next_original_valid;
        always @(posedge clk_pix or negedge rst_pix_n)begin
            if(!rst_pix_n)begin
                cmp_req_sync<=0;cmp_ack<=0;cmp_enable_sync<=0;cmp_mode<=0;cmp_valid<=0;
                cmp_original<=0;cmp_half<=0;cmp_fid<=0;cmp_epoch<=0;cmp_slot<=0;
                wipe_mode<=0;mode_selected<=0;mode_req_sync<=0;mode_ack<=0;
            end else begin
                mode_req_sync<={mode_req_sync[0],mode_request};
                cmp_req_sync<={cmp_req_sync[0],cmp_request};
                cmp_enable_sync<={cmp_enable_sync[0],compare_enabled_pe};
                if(frame)begin
                    cmp_mode<=next_compare_mode;
                    wipe_mode<=next_wipe_mode;
                    if(mode_pending_pix)begin mode_selected<=mode_held;mode_ack<=mode_req_sync[1];end
                    if(next_display_mode!=1)cmp_valid<=0;
                    if(cmp_req_sync[1]!=cmp_ack)begin
                        cmp_original<=held_original;cmp_half<=held_half;cmp_fid<=held_cfid;
                        cmp_epoch<=held_cepoch;cmp_slot<=held_cslot;
                        cmp_valid<=next_display_mode==1;cmp_ack<=cmp_req_sync[1];
                    end
                end
            end
        end
        wire chs,cvs,cde,cgood;wire [23:0] crgb;wire [11:0] cax,cay;
        wire [31:0] wipe_original_base=32'h03000000+{9'd0,pix_guide_slot,20'd0};
        d64_comparison_display #(.WIPE_ENABLE(WIPE_ENABLE)) u_compare(.clk_axi(clk_axi),.rst_axi_n(rst_axi_n),
            .clk_pix(clk_pix),.rst_pix_n(rst_pix_n),.frame_i(frame),.enabled_i(cmp_mode||wipe_mode),
            .wipe_i(wipe_mode),.source_valid_i(wipe_mode?(pix_original_valid&&pix_valid):cmp_valid),
            .original_base_i(wipe_mode?wipe_original_base:cmp_original),.style_base_i(cmp_half),
            .fid_i(wipe_mode?pix_guide_fid:cmp_fid),.epoch_i(wipe_mode?pix_fid[31:16]:cmp_epoch),
            .frame_slot_i(wipe_mode?pix_guide_slot:cmp_slot),
            .hs_i(hs),.vs_i(vs),.de_i(de),.ax_i(ax),.ay_i(ay),
            .hs_o(chs),.vs_o(cvs),.de_o(cde),.good_o(cgood),.rgb_o(crgb),.ax_o(cax),.ay_o(cay),
            .deadline_faults(cmp_deadlines),.reader_axi_fault(cmp_axi_fault),.reader_protocol_fault(cmp_protocol_fault),
            .outstanding_request_axi(cmp_reader_outstanding),.outstanding_slot_axi(cmp_reader_slot),
            .araddr(comparison_araddr),.arlen(comparison_arlen),.arsize(comparison_arsize),.arburst(comparison_arburst),
            .arvalid(comparison_arvalid),.arready(comparison_arready),.rdata(comparison_rdata),
            .rvalid(comparison_rvalid),.rready(comparison_rready),.rlast(comparison_rlast),.rresp(comparison_rresp));
        wire ahs,avs,ade,agood;wire [23:0] argb;wire [11:0] aax,aay;
        if(COMPARISON_DELAY>0)begin:g_align
            reg [COMPARISON_DELAY-1:0] hp,vp,dp,gp;
            reg [23:0] rp[0:COMPARISON_DELAY-1];
            reg [11:0] xp[0:COMPARISON_DELAY-1],yp[0:COMPARISON_DELAY-1];integer ai;
            always @(posedge clk_pix or negedge rst_pix_n)begin
                if(!rst_pix_n)begin
                    hp<=0;vp<=0;dp<=0;gp<=0;
                    for(ai=0;ai<COMPARISON_DELAY;ai=ai+1)begin rp[ai]<=0;xp[ai]<=0;yp[ai]<=0;end
                end else begin
                    hp<=(hp<<1)|chs;vp<=(vp<<1)|cvs;
                    dp<=(dp<<1)|cde;gp<=(gp<<1)|cgood;
                    rp[0]<=crgb;xp[0]<=cax;yp[0]<=cay;
                    for(ai=1;ai<COMPARISON_DELAY;ai=ai+1)begin rp[ai]<=rp[ai-1];xp[ai]<=xp[ai-1];yp[ai]<=yp[ai-1];end
                end
            end
            assign ahs=hp[COMPARISON_DELAY-1];assign avs=vp[COMPARISON_DELAY-1];
            assign ade=dp[COMPARISON_DELAY-1];assign agood=gp[COMPARISON_DELAY-1];
            assign argb=rp[COMPARISON_DELAY-1];assign aax=xp[COMPARISON_DELAY-1];assign aay=yp[COMPARISON_DELAY-1];
        end else begin:g_direct
            assign ahs=chs;assign avs=cvs;assign ade=cde;assign agood=cgood;
            assign argb=crgb;assign aax=cax;assign aay=cay;
        end
        wire wipe_left=wipe_mode&&render_ax<960;
        assign compare_selected=cmp_mode||wipe_mode;
        assign visible_first_y=cmp_mode?12'd270:IMAGE_Y0;
        assign display_hs=cmp_mode?ahs:render_hs;assign display_vs=cmp_mode?avs:render_vs;
        assign display_de=cmp_mode?ade:render_de;
        assign display_good=cmp_mode?agood:(wipe_left?agood:render_good);
        assign display_ax=cmp_mode?aax:render_ax;assign display_ay=cmp_mode?aay:render_ay;
        wire comparison_effect_pixel=effect_frame&&ade&&agood&&
            aax>=960&&aax<1920&&aay>=270&&aay<810;
        assign source_rgb=cmp_mode?(comparison_effect_pixel?~argb:argb):(wipe_left?argb:full_source);
        assign visible_scan_fid=cmp_mode?{cmp_epoch,cmp_fid[15:0]}:pix_fid;
    end else begin:g_legacy_transport
        assign remote_accepted_mode_pix=0;assign remote_actual_mode_pix=0;
        assign comparison_publish_ready=1;assign comparison_protect_mask_pe=0;assign comparison_faults=0;
        assign comparison_araddr=0;assign comparison_arlen=0;assign comparison_arsize=4;assign comparison_arburst=1;
        assign comparison_arvalid=0;assign comparison_rready=1;assign compare_selected=0;
        assign visible_first_y=IMAGE_Y0;
        assign display_hs=render_hs;assign display_vs=render_vs;assign display_de=render_de;assign display_good=render_good;
        assign display_ax=render_ax;assign display_ay=render_ay;assign source_rgb=full_source;
        assign visible_scan_fid=pix_fid;
    end endgenerate
    wire scan_first=pix_valid&&display_de&&display_good&&
        display_ax==(compare_selected?960:IMAGE_X0)&&display_ay==visible_first_y;
    reg [1:0] encoder_first_d;
    reg [31:0] pending_scan_fid;
    always @(posedge clk_pix or negedge rst_pix_n) begin
        if(!rst_pix_n) begin
            scan_ax1<=0;scan_ay1<=0;scan_ax2<=0;scan_ay2<=0;
            scan_tog<=0;scan_fid_pix<=0;encoder_first_d<=0;
            pending_scan_fid<=0;first_scan_pix<=0;
        end else begin
            scan_ax1<=ax;scan_ay1<=ay;scan_ax2<=scan_ax1;scan_ay2<=scan_ay1;
            encoder_first_d<={encoder_first_d[0],
                scan_first};
            first_scan_pix<=encoder_first_d[1];
            if(encoder_first_d[1]) begin
                scan_tog<=!scan_tog;scan_fid_pix<=pending_scan_fid;
            end

            if(scan_first) begin
                pending_scan_fid<=visible_scan_fid;
            end
        end
    end

    wire [23:0] encoder_rgb;
    wire [7:0] fps_bcd,fps_bcd_count;
    wire[15:0]camera_fps_bcd,style_fps_bcd,scan_fps_bcd;wire[2:0]fps_valid;
    wire style_publication_event=(ack_s[1]!=ack_seen)&&(!active_valid||held_fid!=active_fid);
    wire enabled;
    wire [7:0] fps_tens={4'd0,fps_bcd_count[7:4]};
    assign processing_fps=(fps_tens<<3)+(fps_tens<<1)+{4'd0,fps_bcd_count[3:0]};
        d64_fps_status #(.CLK_HZ(FPS_CLK_HZ),.DEFAULT_ON(FPS_DEFAULT_ON),.REMOTE_DEBUG_ENABLE(REMOTE_DEBUG_ENABLE)) u_status(
            .clk_count(clk_axi),.rst_count_n(rst_axi_n),
            .camera_event(camera_complete_event),.publish_event(style_publication_event),.scan_event(frame_start_axi),.toggle_event(fps_toggle),
            .remote_set_valid(remote_fps_set_valid),.remote_set_value(remote_fps_set_value),.enabled_count_state(fps_requested_pe),
            .clk_pix(clk_pix),.rst_pix_n(rst_pix_n),.frame_start(frame),
            .fps_bcd_pix(fps_bcd),.enabled_pix(enabled),.last_bcd_count(fps_bcd_count),
            .camera_bcd_pix(camera_fps_bcd),.style_bcd_pix(style_fps_bcd),.scan_bcd_pix(scan_fps_bcd),.valid_pix(fps_valid));
    // PIX state changes on raster boundaries. Hold the entire tuple until the
    // PE domain acknowledges it; individual bus bits are never synchronized.
    generate if(REMOTE_DEBUG_ENABLE!=0)begin:g_remote_feedback
        wire [5:0] live={remote_accepted_mode_pix,remote_actual_mode_pix,enabled,effect_frame};
        reg [5:0] held,state_pe;reg request,ack;
        (* async_reg="true" *) reg [1:0] ack_sync,request_sync;
        always @(posedge clk_pix or negedge rst_pix_n)begin
            if(!rst_pix_n)begin held<={4'd0,(FPS_DEFAULT_ON!=0),1'b0};request<=0;ack_sync<=0;end
            else begin
                ack_sync<={ack_sync[0],ack};
                if(ack_sync[1]==request&&held!=live)begin held<=live;request<=~request;end
            end
        end
        always @(posedge clk_axi or negedge rst_axi_n)begin
            if(!rst_axi_n)begin state_pe<={4'd0,(FPS_DEFAULT_ON!=0),1'b0};ack<=0;request_sync<=0;end
            else begin
                request_sync<={request_sync[0],request};
                if(request_sync[1]!=ack)begin state_pe<=held;ack<=request_sync[1];end
            end
        end
        assign remote_actual_state_pe=state_pe;
    end else begin:g_no_remote_feedback assign remote_actual_state_pe=0;end endgenerate
    generate if(FPS_OSD_ENABLE!=0) begin:g_fps
        d64_fps_overlay u_overlay(.enabled(enabled),.valid(fps_valid),.camera_bcd(camera_fps_bcd),.style_bcd(style_fps_bcd),.scan_bcd(scan_fps_bcd),.de_i(display_de),
            .ax_i(display_ax),.ay_i(display_ay),.rgb_i(source_rgb),.rgb_o(encoder_rgb),
            .overlay_active(),.overlay_ink());
    end else begin:g_no_fps
        assign encoder_rgb=source_rgb;
    end endgenerate
    dvi_encoder u_tmds(.pixelclk(clk_pix),.rstin(!rst_pix_n),
        .red_din(encoder_rgb[23:16]),
        .green_din(encoder_rgb[15:8]),
        .blue_din(encoder_rgb[7:0]),
        .hsync(display_hs),.vsync(display_vs),.de(display_de),.tmds_data0(tmds_data0),
        .tmds_data1(tmds_data1),.tmds_data2(tmds_data2),.tmds_clk(tmds_clk));
endmodule

