`timescale 1ns/1ps
// Two independent DDR snapshot slots. Network ownership never pins a CNN slot.
// Copy only complete publications, skip occupied slots, and invalidate a copy
// if the CNN announces reuse of its source. Every issued AXI transaction drains.
module d64_preview_snapshot #(
    parameter integer PIXELS=256*144, CAPTURE_DIV=3,
    parameter [31:0] BASE=32'h03a00000, STRIDE=32'h00040000
)(
    input wire clk,rst_n,publish_event,
    input wire [31:0] publish_base,publish_fid,
    input wire [15:0] publish_epoch,input wire [1:0] publish_style,
    input wire [7:0] publish_effect,
    input wire overwrite_event,input wire [31:0] overwrite_base,
    input wire lease_active,input wire lease_slot,
    output reg latest_valid,output reg latest_slot,
    output reg [31:0] latest_fid,output reg [15:0] latest_epoch,
    output reg [1:0] latest_style,output reg [7:0] latest_effect,
    output wire [31:0] latest_base,output wire busy,
    output reg dropped,output reg fault,
    output wire arvalid,input wire arready,output wire [31:0] araddr,
    output wire [7:0] arlen,output wire [2:0] arsize,output wire [1:0] arburst,
    input wire rvalid,output wire rready,input wire [127:0] rdata,
    input wire rlast,input wire [1:0] rresp,
    output wire awvalid,input wire awready,output wire [31:0] awaddr,
    output wire [7:0] awlen,output wire [2:0] awsize,output wire [1:0] awburst,
    output wire wvalid,input wire wready,output wire [127:0] wdata,
    output wire [15:0] wstrb,output wire wlast,
    input wire bvalid,output wire bready,input wire [1:0] bresp
);
    localparam IDLE=0,AR=1,R=2,AW=3,W=4,B=5,DRAIN=6;
    localparam integer BEATS=PIXELS/4;
    reg [2:0] state;
    reg [15:0] beat;
    reg [31:0] source_base,copy_fid;
    reg [15:0] copy_epoch;
    reg [1:0] copy_style;
    reg [7:0] copy_effect;
    reg slot,invalid;
    reg [127:0] held;
    reg [7:0] cadence;
    wire source_invalid=overwrite_event && overwrite_base==source_base;
    wire chosen_slot=lease_active ? !lease_slot : !latest_slot;
    assign latest_base=BASE+(latest_slot?STRIDE:0);
    assign busy=state!=IDLE;
    assign arvalid=state==AR;assign araddr=source_base+{beat,4'b0};
    assign arlen=0;assign arsize=4;assign arburst=1;
    assign rready=state==R||state==DRAIN;
    assign awvalid=state==AW;assign awaddr=BASE+(slot?STRIDE:0)+{beat,4'b0};
    assign awlen=0;assign awsize=4;assign awburst=1;
    assign wvalid=state==W;assign wdata=held;assign wstrb=16'hffff;assign wlast=1;
    assign bready=state==B;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;beat<=0;source_base<=0;slot<=0;invalid<=0;held<=0;
            latest_valid<=0;latest_slot<=0;latest_fid<=0;latest_epoch<=0;
            latest_style<=1;latest_effect<=0;copy_fid<=0;copy_epoch<=0;
            copy_style<=1;copy_effect<=0;cadence<=0;dropped<=0;fault<=0;
        end else begin
            if(busy && source_invalid)invalid<=1;
            if(publish_event)begin
                if(cadence==CAPTURE_DIV-1)cadence<=0;else cadence<=cadence+1'b1;
                if(cadence==0)begin
                    if(state==IDLE)begin
                        source_base<=publish_base;copy_fid<=publish_fid;
                        copy_epoch<=publish_epoch;copy_style<=publish_style;
                        copy_effect<=publish_effect;slot<=chosen_slot;
                        if(latest_valid && latest_slot==chosen_slot)latest_valid<=0;
                        beat<=0;invalid<=overwrite_event&&overwrite_base==publish_base;
                        state<=AR;
                    end else dropped<=1;
                end
            end
            case(state)
                AR:if(arready)state<=R;
                R:if(rvalid)begin
                    if(rresp!=0 || !rlast)begin
                        invalid<=1;fault<=1;
                        if(rlast)begin state<=IDLE;dropped<=1;end
                        else state<=DRAIN;
                    end else if(invalid||source_invalid)begin state<=IDLE;dropped<=1;end
                    else begin held<=rdata;state<=AW;end
                end
                AW:if(awready)state<=W;
                W:if(wready)state<=B;
                B:if(bvalid)begin
                    if(bresp!=0)begin invalid<=1;fault<=1;end
                    if(invalid||source_invalid||bresp!=0)begin state<=IDLE;dropped<=1;end
                    else if(beat==BEATS-1)begin
                        latest_valid<=1;latest_slot<=slot;latest_fid<=copy_fid;
                        latest_epoch<=copy_epoch;latest_style<=copy_style;
                        latest_effect<=copy_effect;state<=IDLE;
                    end else begin beat<=beat+1'b1;state<=AR;end
                end
                DRAIN:if(rvalid&&rlast)begin state<=IDLE;dropped<=1;end
                default:begin end
            endcase
        end
    end
endmodule
