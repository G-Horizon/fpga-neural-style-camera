`timescale 1ns/1ps
// Original source-resolution guide to held-frame style display stream.
// Root must hold guide_slot/fid until the pixel-frame publication, and lease
// outstanding_request_slot while an old request finishes across a new frame.
// Pixel stream has3 registered stages:1 RAM/style alignment,1 HP center,
// then1 fusion output. Input sample edge n emerges afteredge n+2.
module d64_guide_display_filter #(
    parameter integer W=1920,H=1080,FAULT_STICKY=1,STORED_H=H-1,CACHE_PIXELS=2,
    parameter [31:0] GUIDE_BASE=32'h02000000,GUIDE_SLOT_BYTES=32'h00200000
) (
    input wire clk_axi,input wire rst_axi_n,input wire clk_pix,input wire rst_pix_n,
    input wire frame_i,hs_i,vs_i,de_i,style_good_i,
    input wire [23:0] style_rgb_i,input wire [11:0] ax_i,ay_i,
    input wire [31:0] fid_i,input wire [2:0] guide_slot_i,
    input wire guide_slot_valid_i,input wire [1:0] strength_i,
    output wire hs_o,vs_o,de_o,good_o,output wire [23:0] rgb_o,
    output wire [11:0] ax_o,ay_o,output wire [31:0] fid_o,
    output wire [31:0] mismatch_faultcounter,
    output wire reader_axi_fault,reader_protocol_fault,
    output wire outstanding_request_axi,output wire [2:0] outstanding_request_slot_axi,
    output wire [7:0] outstanding_request_mask_axi,
    output wire [31:0] axi_araddr,output wire [7:0] axi_arlen,
    output wire [2:0] axi_arsize,output wire [1:0] axi_arburst,
    output wire axi_arvalid,input wire axi_arready,
    input wire [127:0] axi_rdata,input wire axi_rvalid,output wire axi_rready,
    input wire axi_rlast,input wire [1:0] axi_rresp
);
    reg frame_pending,active_valid;
    reg [31:0] active_fid;
    reg [2:0] active_slot;
    reg [10:0] next_y,allowed_y;
    reg req_toggle,ack_toggle;
    reg [31:0] held_fid;
    reg [2:0] held_slot;
    (* syn_preserve=1 *) reg [7:0] held_slot_mask;
    reg [10:0] held_y;
    reg held_bank;
    (* async_reg="true" *) reg [1:0] ack_sync,req_sync;
    wire request_idle=ack_sync[1]==req_toggle;
    // frame_i also updates root's held pixel descriptor with nonblocking writes.
    // Wait one pixel edge so we sample the NEW descriptor, not previous frame.
    always @(posedge clk_pix or negedge rst_pix_n)begin
        if(!rst_pix_n)begin
            frame_pending<=0;active_valid<=0;active_fid<=0;active_slot<=0;
            next_y<=H;allowed_y<=0;req_toggle<=0;held_fid<=0;held_slot<=0;held_slot_mask<=8'h01;
            held_y<=0;held_bank<=0;ack_sync<=0;
        end else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(frame_i)frame_pending<=1;
            if(frame_pending)begin
                frame_pending<=0;active_valid<=guide_slot_valid_i;
                active_fid<=fid_i;active_slot<=guide_slot_i;
                next_y<=0;allowed_y<=1;
            end else if(!frame_i)begin
                if(de_i&&ax_i==W-1&&ay_i<H-2)allowed_y<=ay_i+2;
                if(active_valid&&request_idle&&next_y<H&&next_y<=allowed_y)begin
                    held_fid<=active_fid;held_slot<=active_slot;
                    held_slot_mask<=8'b1<<active_slot;
                    held_y<=next_y;held_bank<=next_y[0];
                    req_toggle<=~req_toggle;next_y<=next_y+1'b1;
                end
            end
        end
    end
    reg request_started,row_req;
    wire row_ready,row_done,row_failed;
    always @(posedge clk_axi or negedge rst_axi_n)begin
        if(!rst_axi_n)begin req_sync<=0;ack_toggle<=0;request_started<=0;row_req<=0;end
        else begin
            req_sync<={req_sync[0],req_toggle};row_req<=0;
            if(!request_started&&req_sync[1]!=ack_toggle&&row_ready)begin
                row_req<=1;request_started<=1;
            end
            if(request_started&&(row_done||row_failed))begin
                request_started<=0;ack_toggle<=req_sync[1];
            end
        end
    end
    assign outstanding_request_axi=request_started||req_sync[1]!=ack_toggle;
    assign outstanding_request_slot_axi=held_slot;
    // Held bundle capture, not a per-bit two-flop synchronizer. The source
    // changes with req_toggle and remains held through the existing ACK.
    // Sample every APP edge, before the original second-stage request gate.
    (* syn_preserve=1 *) reg [7:0] held_slot_mask_sample_axi;
    always @(posedge clk_axi or negedge rst_axi_n)
        if(!rst_axi_n)held_slot_mask_sample_axi<=8'h01;
        else held_slot_mask_sample_axi<=held_slot_mask;
    assign outstanding_request_mask_axi=outstanding_request_axi?held_slot_mask_sample_axi:8'd0;
    wire guide_valid;
    wire [7:0] guide_gray;
    wire [31:0] guide_fid;
    wire [10:0] guide_y;
    d64_guide_row_reader #(.W(W),.H(H),.STORED_H(STORED_H),.CACHE_PIXELS(CACHE_PIXELS),.BASE(GUIDE_BASE),.SLOT_BYTES(GUIDE_SLOT_BYTES)) u_reader(
        .clk(clk_axi),.rst_n(rst_axi_n),.row_req(row_req),.row_bank(held_bank),
        .row_slot(held_slot),.row_y(held_y),.row_fid(held_fid),
        .row_ready(row_ready),.row_done(row_done),.row_failed(row_failed),
        .axi_fault(reader_axi_fault),.protocol_fault(reader_protocol_fault),
        .araddr(axi_araddr),.arlen(axi_arlen),.arsize(axi_arsize),.arburst(axi_arburst),
        .arvalid(axi_arvalid),.arready(axi_arready),
        .rdata(axi_rdata),.rvalid(axi_rvalid),.rready(axi_rready),
        .rlast(axi_rlast),.rresp(axi_rresp),
        .pix_clk(clk_pix),.pix_rst_n(rst_pix_n),.pix_valid(de_i&&style_good_i&&guide_slot_valid_i),
        .pix_bank(ay_i[0]),.pix_x(ax_i[10:0]),.pix_y(ay_i[10:0]),.pix_fid(fid_i),
        .guide_valid(guide_valid),.guide_g(guide_gray),.guide_fid(guide_fid),.guide_y(guide_y));
    reg hs_delay,vs_delay,de_delay,good_delay;
    reg [23:0] rgb_delay;
    reg [10:0] x_delay,y_delay;
    reg [31:0] fid_delay;
    reg [1:0] strength_delay;
    always @(posedge clk_pix or negedge rst_pix_n)begin
        if(!rst_pix_n)begin
            hs_delay<=0;vs_delay<=0;de_delay<=0;good_delay<=0;rgb_delay<=0;
            x_delay<=0;y_delay<=0;fid_delay<=0;strength_delay<=0;
        end else begin
            hs_delay<=hs_i;vs_delay<=vs_i;de_delay<=de_i;good_delay<=style_good_i;
            rgb_delay<=style_rgb_i;x_delay<=ax_i[10:0];y_delay<=ay_i[10:0];
            fid_delay<=fid_i;strength_delay<=strength_i;
        end
    end
    wire [10:0] x_out,y_out;wire detail_applied,pair_mismatch;
    d64_guide_horizontal_fusion #(.W(W)) u_horizontal(
        .clk(clk_pix),.rst_n(rst_pix_n),.in_de(de_delay),
        .in_hsync(hs_delay),.in_vsync(vs_delay),.in_good(good_delay),
        .in_rgb(rgb_delay),.in_x(x_delay),.in_y(y_delay),.in_fid(fid_delay),
        .guide_valid(guide_valid&&guide_y==y_delay),.guide_gray(guide_gray),
        .guide_fid(guide_fid),.strength(strength_delay),
        .out_de(de_o),.out_hsync(hs_o),.out_vsync(vs_o),.out_good(good_o),
        .out_rgb(rgb_o),.out_x(x_out),.out_y(y_out),.out_fid(fid_o),
        .detail_applied(detail_applied),.pair_mismatch(pair_mismatch));
    assign ax_o={1'b0,x_out};assign ay_o={1'b0,y_out};
    // This public counter belongs to clk_pix. Root should snapshot it or, with
    // FAULT_STICKY1 default, synchronize the stable sticky bit into AXI domain.
    generate if(FAULT_STICKY!=0)begin:g_sticky
        reg fault;
        always @(posedge clk_pix or negedge rst_pix_n)
            if(!rst_pix_n)fault<=0;
            else if(pair_mismatch&&de_o&&good_o)fault<=1;
        assign mismatch_faultcounter={31'd0,fault};
    end else begin:g_counter
        reg [31:0] count;
        always @(posedge clk_pix or negedge rst_pix_n)
            if(!rst_pix_n)count<=0;
            else if(pair_mismatch&&de_o&&good_o&&count!=32'hffffffff)count<=count+1'b1;
        assign mismatch_faultcounter=count;
    end endgenerate
endmodule
