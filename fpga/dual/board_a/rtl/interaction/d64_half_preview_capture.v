`timescale 1ns/1ps
// Capture even points from the1080p half-fusion render BEFORE the live
// presentation effect, comparison, or FPS overlay, without HDMI
// backpressure. Only RGB565 crosses the dual-clock FIFO; a stable descriptor
// and stable end status cross with toggle handshakes. Both domains must reset
// together. The last PE pixel waits for full raster completion/status, so an
// invalid final odd row can never arrive after a successful DDR publication.
module d64_half_preview_capture #(
    parameter integer SRC_W=1920,SRC_H=1080,FIFO_AW=9,WRITER_FIFO_AW=9,BURST_BEATS=16,
    parameter integer ADMISSION_ENABLE=0,
    parameter [31:0] BASE=32'h03500000,SLOT_BYTES=32'h00100000
)(
    input wire clk_pix,rst_pix_n,
    input wire render_de,render_good,input wire [23:0] render_rgb,
    input wire [11:0] render_ax,render_ay,input wire [31:0] render_fid,
    input wire [15:0] render_epoch,input wire [2:0] guide_slot,input wire slot_valid,
    // Sampled only at SOF. Later admission changes never poison a held ticket.
    input wire admit_frame,
    output wire busy_pix,output wire [4:0] ticket_mask_pix,
    output reg [31:0] skipped_frames_pix,output reg overflow_fault_pix,
    input wire clk_pe,rst_pe_n,
    output wire busy_pe,output wire [4:0] ticket_mask_pe,
    output reg committed,failed,output reg [2:0] committed_slot,
    output reg [31:0] committed_fid,output reg [15:0] committed_epoch,
    output wire writer_axi_fault,writer_protocol_fault,writer_overflow_fault,
    output wire [31:0] awaddr,output wire [7:0] awlen,
    output wire [2:0] awsize,output wire [1:0] awburst,
    output wire awvalid,input wire awready,
    output wire [127:0] wdata,output wire [15:0] wstrb,
    output wire wlast,wvalid,input wire wready,
    input wire bvalid,output wire bready,input wire [1:0] bresp
);
    localparam integer W=SRC_W/2,H=SRC_H/2,PIXELS=W*H;
    reg ticket_busy,capturing,poison_pix,start_toggle,end_toggle;
    reg [31:0] held_fid;reg [15:0] held_epoch;reg [2:0] held_slot;
    reg [19:0] selected_count,pushed_count,end_pushed_count;
    reg held_end_bad;
    reg [11:0] expected_x,expected_y;
    reg ack_toggle;
    (* async_reg="true" *) reg [1:0] ack_sync,start_sync,end_sync;
    wire first=render_de&&render_ax==0&&render_ay==0;
    wire selected=render_de&&render_ax<SRC_W&&render_ay<SRC_H&&!render_ax[0]&&!render_ay[0];
    wire start_now=first&&!ticket_busy&&slot_valid&&(ADMISSION_ENABLE==0||admit_frame);
    wire sample=selected&&(start_now||(capturing&&!first&&selected_count<PIXELS));
    wire fifo_full,fifo_empty,fifo_rd_valid;
    wire [15:0] fifo_rd_data;
    wire fifo_write=sample&&!fifo_full;
    wire [15:0] input565={render_rgb[23:19],render_rgb[15:10],render_rgb[7:3]};
    assign busy_pix=ticket_busy;
    assign ticket_mask_pix=ticket_busy?(5'b00001<<held_slot):5'd0;
    always @(posedge clk_pix or negedge rst_pix_n)begin
        if(!rst_pix_n)begin
            ticket_busy<=0;capturing<=0;poison_pix<=0;start_toggle<=0;end_toggle<=0;
            held_fid<=0;held_epoch<=0;held_slot<=0;ack_sync<=0;
            selected_count<=0;pushed_count<=0;end_pushed_count<=0;held_end_bad<=0;
            expected_x<=0;expected_y<=0;skipped_frames_pix<=0;overflow_fault_pix<=0;
        end else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(ticket_busy&&!capturing&&ack_sync[1]==start_toggle)ticket_busy<=0;
            if(first&&!start_now&&!capturing)skipped_frames_pix<=skipped_frames_pix+1'b1;
            if(start_now)begin
                ticket_busy<=1;capturing<=1;start_toggle<=~start_toggle;
                held_fid<=render_fid;held_epoch<=render_epoch;held_slot<=guide_slot;
                poison_pix<=!render_good||fifo_full;
                selected_count<=1;pushed_count<=fifo_full?0:1;
                expected_x<=2;expected_y<=0;
                if(fifo_full)overflow_fault_pix<=1;
            end else if(capturing)begin
                if(render_de&&(!render_good||!slot_valid||render_fid!=held_fid||
                    render_epoch!=held_epoch||guide_slot!=held_slot))poison_pix<=1;
                if(sample)begin
                    selected_count<=selected_count+1'b1;
                    if(fifo_write)pushed_count<=pushed_count+1'b1;
                    else begin poison_pix<=1;overflow_fault_pix<=1;end
                    if(render_ax!=expected_x||render_ay!=expected_y)poison_pix<=1;
                    if(expected_x==SRC_W-2)begin expected_x<=0;expected_y<=expected_y+2;end
                    else expected_x<=expected_x+2;
                end
                // Unexpected next SOF closes an incomplete old frame; it is
                // skipped rather than relabelled as the previous descriptor.
                if(first||(render_de&&render_ax==SRC_W-1&&render_ay==SRC_H-1))begin
                    capturing<=0;end_toggle<=~end_toggle;end_pushed_count<=pushed_count;
                    held_end_bad<=poison_pix||first||selected_count!=PIXELS||!render_good||
                        !slot_valid||render_fid!=held_fid||render_epoch!=held_epoch||guide_slot!=held_slot;
                    if(first)skipped_frames_pix<=skipped_frames_pix+1'b1;
                end
            end
        end
    end
    reg pe_active,end_seen,end_seen_toggle,pe_poison,read_pending;
    reg [31:0] fid_q;reg [15:0] epoch_q;reg [2:0] slot_q;
    reg [19:0] sample_index,read_count,pushed_q;
    reg [10:0] writer_x,writer_y;
    wire writer_ready,writer_committed,writer_failed;
    wire final_allowed=sample_index<PIXELS-1||end_seen;
    wire fifo_read=pe_active&&!read_pending&&!fifo_empty&&final_allowed&&writer_ready;
    // If overflow removed pixels, drain the FIFO and pad a poisoned frame.
    // Completing its fixed geometry lets every AW/W/B retire before ticket ACK.
    wire pad=pe_active&&end_seen&&pe_poison&&!read_pending&&!fifo_rd_valid&&fifo_empty&&
        read_count==pushed_q&&sample_index<PIXELS&&writer_ready;
    wire writer_input=pe_active&&(fifo_rd_valid||pad);
    assign busy_pe=pe_active;
    assign ticket_mask_pe=pe_active?(5'b00001<<slot_q):5'd0;
    d64_async_fifo #(.DW(16),.AW(FIFO_AW)) u_pixels(
        .wr_clk(clk_pix),.wr_rst_n(rst_pix_n),.wr_en(fifo_write),.wr_data(input565),
        .wr_full(fifo_full),.wr_almost_full(),.wr_level(),
        .rd_clk(clk_pe),.rd_rst_n(rst_pe_n),.rd_en(fifo_read),.rd_data(fifo_rd_data),
        .rd_valid(fifo_rd_valid),.rd_empty(fifo_empty),.rd_level());
    d64_original_store_quad #(.W(W),.H(H),.BASE(BASE),.SLOT_BYTES(SLOT_BYTES),
        .FIFO_AW(WRITER_FIFO_AW),.BURST_BEATS(BURST_BEATS)) u_writer(
        .clk(clk_pe),.rst_n(rst_pe_n),.in_valid(writer_input),.in_ready(writer_ready),
        .in_rgb565(pad?16'd0:fifo_rd_data),.in_x(writer_x),.in_y(writer_y),.in_fid(fid_q),
        .frame_bad(pe_poison),.frame_slot_valid(pe_active),.frame_slot(slot_q),
        .capture_started(),.capturing(),.active_slot(),.active_fid(),
        .committed(writer_committed),.failed(writer_failed),.committed_slot(),.committed_fid(),
        .overflow_fault(writer_overflow_fault),.protocol_fault(writer_protocol_fault),.axi_fault(writer_axi_fault),
        .awaddr(awaddr),.awlen(awlen),.awsize(awsize),.awburst(awburst),.awvalid(awvalid),.awready(awready),
        .wdata(wdata),.wstrb(wstrb),.wlast(wlast),.wvalid(wvalid),.wready(wready),
        .bvalid(bvalid),.bready(bready),.bresp(bresp));
    always @(posedge clk_pe or negedge rst_pe_n)begin
        if(!rst_pe_n)begin
            start_sync<=0;end_sync<=0;ack_toggle<=0;end_seen_toggle<=0;
            pe_active<=0;end_seen<=0;pe_poison<=0;read_pending<=0;
            fid_q<=0;epoch_q<=0;slot_q<=0;sample_index<=0;read_count<=0;pushed_q<=0;
            writer_x<=0;writer_y<=0;committed<=0;failed<=0;
            committed_slot<=0;committed_fid<=0;committed_epoch<=0;
        end else begin
            start_sync<={start_sync[0],start_toggle};end_sync<={end_sync[0],end_toggle};
            committed<=0;failed<=0;
            if(!pe_active&&start_sync[1]!=ack_toggle)begin
                pe_active<=1;fid_q<=held_fid;epoch_q<=held_epoch;slot_q<=held_slot;
                sample_index<=0;read_count<=0;writer_x<=0;writer_y<=0;
                end_seen<=0;pe_poison<=0;read_pending<=0;
            end
            if(pe_active&&end_sync[1]!=end_seen_toggle)begin
                end_seen_toggle<=end_sync[1];end_seen<=1;
                pushed_q<=end_pushed_count;pe_poison<=held_end_bad;
            end
            if(fifo_read)read_pending<=1;
            if(fifo_rd_valid)begin read_pending<=0;read_count<=read_count+1'b1;end
            if(writer_input)begin
                sample_index<=sample_index+1'b1;
                if(writer_x==W-1)begin writer_x<=0;writer_y<=writer_y+1'b1;end
                else writer_x<=writer_x+1'b1;
            end
            if(pe_active&&(writer_committed||writer_failed))begin
                committed<=writer_committed&&!pe_poison;failed<=writer_failed||pe_poison;
                committed_slot<=slot_q;committed_fid<=fid_q;committed_epoch<=epoch_q;
                pe_active<=0;ack_toggle<=start_sync[1];
            end
        end
    end
    // synthesis translate_off
    initial if(SRC_W%16!=0||SRC_H%2!=0||SRC_W<16||SRC_W>2048||SRC_H<4||SRC_H>2048||PIXELS>=1048576)
        $fatal(1,"HALF_PREVIEW_GEOMETRY");
    // synthesis translate_on
endmodule
