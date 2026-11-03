`timescale 1ns/1ps
// Two RGB565 DDR planes, four caches indexed by rowParity and plane.
// Rows0/1 of both planes are prefetched in vblank. Once the current
// plane is consumed, its cache can receive row(y+2), a full row early.
// Busy requests remain pending; actual pixel consumption owns deadline fault.
// Root owns all frame/slot leases and provides matching fullFID+style epoch.
module d64_comparison_display #(
    parameter integer W=960,H=540,Y0=270,WIPE_ENABLE=0
)(
    input wire clk_axi,rst_axi_n,clk_pix,rst_pix_n,
    input wire frame_i,enabled_i,source_valid_i,wipe_i,
    input wire [31:0] original_base_i,style_base_i,fid_i,
    input wire [15:0] epoch_i,input wire [2:0] frame_slot_i,
    input wire hs_i,vs_i,de_i,input wire [11:0] ax_i,ay_i,
    output reg hs_o,vs_o,de_o,good_o,output reg [23:0] rgb_o,
    output reg [11:0] ax_o,ay_o,
    output reg [31:0] deadline_faults,
    output wire reader_axi_fault,reader_protocol_fault,
    output wire outstanding_request_axi,output wire [2:0] outstanding_slot_axi,
    output wire [31:0] araddr,output wire [7:0] arlen,
    output wire [2:0] arsize,output wire [1:0] arburst,
    output wire arvalid,input wire arready,
    input wire [127:0] rdata,input wire rvalid,output wire rready,
    input wire rlast,input wire [1:0] rresp
);
    reg frame_pending,active,first_pending,active_wipe;
    reg original_pending,style_pending,initial_loading;reg [1:0] initial_index;reg[9:0]original_pending_y,style_pending_y;
    reg [31:0] active_original,active_style,active_fid;
    reg [15:0] active_epoch;
    reg [2:0] active_slot;reg [3:0] prepared;
    wire pixel_valid;wire [15:0] pixel_rgb565;
    reg hs_d,vs_d,de_d,window_d,active_d;reg [11:0] ax_d,ay_d;
    reg req_toggle,ack_toggle,ack_good,ack_seen;
    reg [1:0] held_bank;reg [31:0] held_base,held_fid;
    reg [15:0] held_epoch;reg [2:0] held_slot;reg [10:0] held_y;
    (* async_reg="true" *) reg [1:0] ack_sync,req_sync;
    wire request_idle=ack_sync[1]==req_toggle;
    wire image_window=active_wipe ? de_i&&ay_i<2*H&&ax_i<W :
        de_i&&ay_i>=Y0&&ay_i<Y0+H&&ax_i<2*W;
    wire [10:0] image_y=active_wipe ? ay_i[11:1] : ay_i-Y0;
    wire plane_i=!active_wipe&&ax_i>=W;
    wire [1:0] bank_i={image_y[0],plane_i};
    wire [10:0] image_x=active_wipe ? ax_i[11:1] : plane_i?ax_i-W:ax_i;
    wire style_next=!active_wipe&&image_window&&ax_i==2*W-1&&image_y<H-2;
    // A stored original row is used by TWO output rows in wipe mode. The
    // previous row-parity bank can be overwritten only after the odd row's
    // last left pixel has issued its registered RAM read. ax==W is outside
    // the left window, so this release must not be gated by image_window.
    wire original_next=active_wipe ?
        de_i&&ax_i==W&&ay_i<2*H&&ay_i[0]&&image_y<H-2 :
        image_window&&ax_i==W&&image_y<H-2;
    always @(posedge clk_pix or negedge rst_pix_n)begin
        if(!rst_pix_n)begin
            frame_pending<=0;active<=0;first_pending<=0;prepared<=0;active_wipe<=0;
            original_pending<=0;style_pending<=0;initial_loading<=0;initial_index<=0;original_pending_y<=0;style_pending_y<=0;
            active_original<=0;active_style<=0;active_fid<=0;active_epoch<=0;active_slot<=0;
            req_toggle<=0;ack_sync<=0;ack_seen<=0;
            held_base<=0;held_fid<=0;held_epoch<=0;held_slot<=0;held_bank<=0;held_y<=0;
            deadline_faults<=0;
        end else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(ack_sync[1]!=ack_seen)begin
                ack_seen<=ack_sync[1];
                if(held_fid==active_fid&&held_epoch==active_epoch&&held_slot==active_slot)
                    prepared[held_bank]<=ack_good;
            end
            if(frame_i)frame_pending<=1;
            if(frame_pending)begin
                frame_pending<=0;active<=enabled_i&&source_valid_i;
                active_wipe<=WIPE_ENABLE!=0&&wipe_i;
                active_original<=original_base_i;active_style<=style_base_i;
                active_fid<=fid_i;active_epoch<=epoch_i;active_slot<=frame_slot_i;
                first_pending<=0;prepared<=0;
                original_pending<=0;style_pending<=0;
                initial_loading<=enabled_i&&source_valid_i;initial_index<=0;
                original_pending_y<=0;style_pending_y<=0;
            end else if(!frame_i&&active)begin
                // Issue only from a previously latched pending request. A
                // style row becomes eligible on the first Hblank clock after
                // the previous right picture's last registered RAM read.
                if(request_idle&&initial_loading)begin
                    held_base<=initial_index[0]?active_style:active_original;
                    held_fid<=active_fid;held_epoch<=active_epoch;held_slot<=active_slot;
                    held_bank<=initial_index;held_y<={10'd0,initial_index[1]};
                    req_toggle<=~req_toggle;prepared[initial_index]<=0;
                    if(active_wipe)begin
                        // Original plane only: row0/bank0 then row1/bank2.
                        if(initial_index==2)initial_loading<=0;else initial_index<=2;
                    end else begin
                        if(initial_index==3)initial_loading<=0;else initial_index<=initial_index+1'b1;
                    end
                end else if(request_idle&&(original_pending||style_pending))begin
                    held_base<=original_pending?active_original:active_style;
                    held_fid<=active_fid;held_epoch<=active_epoch;held_slot<=active_slot;
                    held_bank<=original_pending?{original_pending_y[0],1'b0}:{style_pending_y[0],1'b1};
                    held_y<=original_pending?{1'b0,original_pending_y}:{1'b0,style_pending_y};
                    req_toggle<=~req_toggle;
                    if(original_pending)prepared[{original_pending_y[0],1'b0}]<=0;
                    else prepared[{style_pending_y[0],1'b1}]<=0;
                    if(original_pending)original_pending<=0;else style_pending<=0;
                end
                if(original_next)begin if(original_pending)deadline_faults<=1;original_pending<=1;original_pending_y<=image_y+2'd2;end
                if(style_next)begin if(style_pending)deadline_faults<=1;style_pending<=1;style_pending_y<=image_y+2'd2;end
                if(window_d&&active_d&&!pixel_valid)deadline_faults<=1;
            end
        end
    end
    reg request_started,row_req;
    wire row_ready,row_done,row_failed;
    always @(posedge clk_axi or negedge rst_axi_n)begin
        if(!rst_axi_n)begin req_sync<=0;ack_toggle<=0;ack_good<=0;request_started<=0;row_req<=0;end
        else begin
            req_sync<={req_sync[0],req_toggle};row_req<=0;
            if(!request_started&&req_sync[1]!=ack_toggle&&row_ready)begin
                row_req<=1;request_started<=1;
            end
            if(request_started&&(row_done||row_failed))begin
                request_started<=0;ack_good<=row_done;ack_toggle<=req_sync[1];
            end
        end
    end
    assign outstanding_request_axi=request_started||req_sync[1]!=ack_toggle;
    assign outstanding_slot_axi=held_slot;
    d64_original_row_reader #(.W(W),.H(H)) u_rows(
        .clk(clk_axi),.rst_n(rst_axi_n),.row_req(row_req),.row_bank(held_bank),
        .row_base(held_base),.row_y(held_y),.row_fid(held_fid),
        .row_ready(row_ready),.row_done(row_done),.row_failed(row_failed),
        .axi_fault(reader_axi_fault),.protocol_fault(reader_protocol_fault),
        .araddr(araddr),.arlen(arlen),.arsize(arsize),.arburst(arburst),
        .arvalid(arvalid),.arready(arready),.rdata(rdata),.rvalid(rvalid),.rready(rready),.rlast(rlast),.rresp(rresp),
        .pix_clk(clk_pix),.pix_rst_n(rst_pix_n),
        .pix_valid(active&&image_window&&prepared[bank_i]),
        .pix_bank(bank_i),.pix_x(image_x),.pix_y(image_y),.pix_fid(active_fid),
        .pixel_valid(pixel_valid),.pixel_rgb565(pixel_rgb565),.pixel_fid(),.pixel_y());
    // One RAM-read stage plus this output stage; all raster metadata follows it.
    always @(posedge clk_pix or negedge rst_pix_n)begin
        if(!rst_pix_n)begin
            hs_d<=0;vs_d<=0;de_d<=0;window_d<=0;active_d<=0;ax_d<=0;ay_d<=0;
            hs_o<=0;vs_o<=0;de_o<=0;good_o<=0;rgb_o<=0;ax_o<=0;ay_o<=0;
        end else begin
            hs_d<=hs_i;vs_d<=vs_i;de_d<=de_i;window_d<=image_window;active_d<=active;
            ax_d<=ax_i;ay_d<=ay_i;
            hs_o<=hs_d;vs_o<=vs_d;de_o<=de_d;ax_o<=ax_d;ay_o<=ay_d;
            good_o<=!window_d||(active_d&&pixel_valid);
            rgb_o<=window_d&&active_d&&pixel_valid?
                {pixel_rgb565[15:11],pixel_rgb565[15:13],
                 pixel_rgb565[10:5],pixel_rgb565[10:9],pixel_rgb565[4:0],pixel_rgb565[4:2]}:24'd0;
        end
    end
    // synthesis translate_off
    initial if(W!=960||H!=540||Y0!=270)$fatal(1,"comparison requires two960x540 planes on1080p");
    // synthesis translate_on
endmodule
