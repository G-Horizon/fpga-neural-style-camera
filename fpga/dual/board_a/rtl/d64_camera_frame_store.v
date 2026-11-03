`timescale 1ns/1ps
// Camera-only input service. Three DDR slots protect the frame being read while
// capture replaces the latest complete frame. Only successful B retirement
// publishes a frame. RGB888 is transported in RGBX without changing RGB bits.
module d64_camera_frame_store #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter integer ADMISSION_ENABLE = 0,
    parameter integer W=256, H=144,
    parameter [31:0] BASE=32'h00000000,
    parameter [31:0] SLOT_BYTES=32'h00040000
) (
    input wire clk, input wire rst_n,
    input wire frame_bad,
    // Sample once at accepted RGB SOF; no RAW/ROI stream is truncated.
    input wire admit_frame,
    input wire in_valid, output wire in_ready,
    input wire [383:0] in_rgb, input wire [31:0] in_frame_id,
    input wire in_sof, input wire in_eol, input wire in_eof,
    input wire [7:0] in_x, input wire [7:0] in_y,
    output wire frame_available, input wire read_start,
    output wire ready_frame_valid,output wire [31:0] ready_frame_id,
    output reg read_done, output reg read_failed,
    output reg out_valid, input wire out_ready,
    output reg [383:0] out_rgb, output reg [31:0] out_frame_id,
    output reg out_sof, output reg out_eol, output reg out_eof,
    output reg [7:0] out_x, output reg [7:0] out_y,
    output wire [31:0] awaddr, output wire [7:0] awlen,
    output wire [2:0] awsize, output wire [1:0] awburst,
    output wire awvalid, input wire awready,
    output wire [127:0] wdata, output wire [15:0] wstrb,
    output wire wlast, output wire wvalid, input wire wready,
    input wire bvalid, output wire bready, input wire [1:0] bresp,
    output wire [31:0] araddr, output wire [7:0] arlen,
    output wire [2:0] arsize, output wire [1:0] arburst,
    output wire arvalid, input wire arready,
    input wire [127:0] rdata, input wire rvalid, output wire rready,
    input wire rlast, input wire [1:0] rresp,
    output reg [31:0] captured_frames, output reg [31:0] dropped_frames,
    output reg [31:0] axi_errors, output reg [31:0] protocol_errors
);
    localparam [1:0] CW_IDLE=0,CW_AW=1,CW_W=2,CW_B=3;
    localparam [1:0] CR_IDLE=0,CR_AR=1,CR_R=2,CR_DRAIN=3;
    reg [1:0] ws, rs, cap_slot, ready_slot, read_slot;
    reg ready_valid, reading, capturing, cap_bad;
    reg dropping;
    reg [31:0] cap_id, ready_id;
    reg [15:0] cap_words;
    reg [383:0] hold_rgb;
    reg [7:0] hold_x, hold_y;
    reg hold_eof;
    reg [1:0] quarter;
    reg [7:0] next_x, next_y;
    reg [7:0] row, rb;
    reg [7:0] read_x;
    reg [1:0] rq;
    wire [1:0] free_slot = (!reading || read_slot!=0) && (!ready_valid || ready_slot!=0) ? 0 :
                           (!reading || read_slot!=1) && (!ready_valid || ready_slot!=1) ? 1 : 2;
    assign in_ready = (ws==CW_IDLE);
    assign frame_available = ready_valid && !reading;
    assign ready_frame_valid=ready_valid;
    assign ready_frame_id=ready_id;
    assign awaddr = BASE + cap_slot*SLOT_BYTES + hold_y*(W*4) + hold_x*4 + quarter*16;
    assign awlen=0; assign awsize=4; assign awburst=1;
    assign awvalid=(ws==CW_AW);
    genvar k;
    generate for(k=0;k<4;k=k+1) begin:g_rgbx
        assign wdata[k*32+:32]={8'd0,hold_rgb[(quarter*4+k)*24+:24]};
    end endgenerate
    assign wstrb=16'hffff; assign wlast=1; assign wvalid=(ws==CW_W);
    assign bready=(ws==CW_B);
    assign araddr=BASE+read_slot*SLOT_BYTES+row*(W*4)+read_x*4;
    // The four beats retire before the local block can be held by the CNN.
    assign arlen=3; assign arsize=4; assign arburst=1;
    assign arvalid=(rs==CR_AR);
    assign rready=(rs==CR_DRAIN) || ((rs==CR_R) && !out_valid);
    wire input_fire=in_valid&&in_ready;
    wire reject_start=input_fire&&in_sof&&ADMISSION_ENABLE!=0&&!admit_frame;
    wire output_fire=out_valid&&out_ready;
    wire bad_marker = in_x!=next_x || in_y!=next_y || in_frame_id!=cap_id ||
                      in_eol!=(in_x==W-16) || in_eof!=((in_x==W-16)&&(in_y==H-1));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ws<=CW_IDLE;rs<=CR_IDLE;cap_slot<=0;ready_slot<=0;read_slot<=0;
            ready_valid<=0;reading<=0;capturing<=0;cap_bad<=0;
            dropping<=0;
            cap_id<=0;ready_id<=0;cap_words<=0;hold_rgb<=0;hold_x<=0;hold_y<=0;hold_eof<=0;
            quarter<=0;next_x<=0;next_y<=0;row<=0;rb<=0;rq<=0;read_x<=0;
            out_valid<=0;out_rgb<=0;out_frame_id<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_x<=0;out_y<=0;
            read_done<=0;read_failed<=0;captured_frames<=0;dropped_frames<=0;axi_errors<=0;protocol_errors<=0;
        end else begin
            read_done<=0; read_failed<=0;
            if(frame_bad&&capturing)cap_bad<=1;
            if(reject_start)begin
                // Drain a complete rejected ROI frame without issuing writes.
                // An interrupted older capture and this rejected SOF each
                // contribute one drop; normal admission is not a protocol fault.
                dropped_frames<=dropped_frames+(capturing?32'd2:32'd1);
                dropping<=!in_eof;capturing<=0;
            end else if(input_fire&&dropping&&!in_sof)begin
                if(in_eof)dropping<=0;
            end else if(input_fire) begin
                if(in_sof) begin
                    dropping<=0;
                    if(capturing) dropped_frames<=dropped_frames+1;
                    capturing<=1;cap_slot<=free_slot;cap_id<=in_frame_id;
                    cap_bad<=(frame_bad || in_x!=0 || in_y!=0 || in_eol || in_eof);
                    cap_words<=1;next_x<=16;next_y<=0;
                end else if(capturing) begin
                    cap_words<=cap_words+1;
                    if(bad_marker) begin cap_bad<=1;protocol_errors<=protocol_errors+1;end
                    if(next_x==W-16) begin next_x<=0;next_y<=next_y+1;end
                    else next_x<=next_x+16;
                end
                if(in_sof || capturing) begin
                    hold_rgb<=in_rgb;hold_x<=in_x;hold_y<=in_y;hold_eof<=in_eof;
                    quarter<=0;ws<=CW_AW;
                end
            end
            if(ws==CW_AW && awready) ws<=CW_W;
            if(ws==CW_W && wready) ws<=CW_B;
            if(ws==CW_B && bvalid) begin
                if(bresp!=0) begin cap_bad<=1;axi_errors <= FAULT_STICKY ? 32'd1 : axi_errors + 32'd1;end
                if(quarter!=3) begin quarter<=quarter+1;ws<=CW_AW;end
                else begin
                    ws<=CW_IDLE;
                    if(hold_eof) begin
                        capturing<=0;
                        if(!cap_bad && !frame_bad && bresp==0 && cap_words==W*H/16) begin
                            ready_valid<=1;ready_slot<=cap_slot;ready_id<=cap_id;
                            captured_frames<=captured_frames+1;
                        end else dropped_frames<=dropped_frames+1;
                    end
                end
            end
            if(read_start && frame_available) begin
                read_slot<=ready_slot;out_frame_id<=ready_id;reading<=1;
                if(!(ws==CW_B && bvalid && quarter==3 && hold_eof &&
                     !cap_bad && !frame_bad && bresp==0 && cap_words==W*H/16))ready_valid<=0;
                row<=0;rb<=0;rq<=0;read_x<=0;rs<=CR_AR;
            end
            if(rs==CR_AR && arready) begin rs<=CR_R;rb<=0;rq<=0;end
            if(output_fire) begin
                out_valid<=0;
                if(out_eol) begin
                    if(out_eof) begin reading<=0;rs<=CR_IDLE;read_done<=1;end
                    else begin row<=row+1;read_x<=0;rs<=CR_AR;end
                end else begin read_x<=read_x+16;rs<=CR_AR;end
            end
            if(rs==CR_R && rvalid && rready) begin
                if(rresp!=0 || rlast!=(rb==3)) begin
                    axi_errors <= FAULT_STICKY ? 32'd1 : axi_errors + 32'd1;out_valid<=0;
                    if(rlast) begin reading<=0;rs<=CR_IDLE;read_failed<=1;end
                    else rs<=CR_DRAIN;
                end else begin
                    // The register becomes visible only after all four beats.
                    out_rgb[rq*96+:96]<={rdata[119:96],rdata[87:64],rdata[55:32],rdata[23:0]};
                    rb<=rb+1;rq<=rq+1;
                    if(rq==3) begin
                        out_valid<=1;out_x<=read_x;out_y<=row;
                        out_sof<=(row==0 && read_x==0);out_eol<=(read_x==W-16);
                        out_eof<=(row==H-1 && read_x==W-16);
                    end
                end
            end
            if(rs==CR_DRAIN && rvalid && rlast) begin reading<=0;rs<=CR_IDLE;read_failed<=1;end
        end
    end
endmodule
