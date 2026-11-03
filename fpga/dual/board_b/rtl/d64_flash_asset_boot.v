`timescale 1ns/1ps
// Read-only SPI mode-0 boot DMA. Configuration flash is never erased or written.
// Each profile is stored at a 64 KiB boundary, including the zero-padded final
// AXI beat. SPI pauses low while a DDR write retires; runtime style switches
// read DDR, so the conservative SPI clock affects boot time only.
module d64_flash_asset_boot #(
    parameter [23:0] FLASH_BASE=24'h700000,
    parameter [31:0] DDR_BASE=32'h00700000,
    parameter [1:0] FIRST_PROFILE=1, LAST_PROFILE=3,
    parameter integer PROFILE_BEATS=2179,
    parameter integer HALF_PERIOD=8,
    parameter integer TIMEOUT_CYCLES=100000000
) (
    input wire clk,input wire rst_n,input wire start,
    output reg spi_clk,output reg spi_cs_n,output wire spi_mosi,input wire spi_miso,
    output wire[19:0] observe_boot,
    output reg [3:0] asset_ready,output reg busy,output reg done,output reg failed,
    output wire [31:0] awaddr,output wire [7:0] awlen,output wire [2:0] awsize,output wire [1:0] awburst,
    output wire awvalid,input wire awready,
    output wire [127:0] wdata,output wire [15:0] wstrb,output wire wlast,output wire wvalid,input wire wready,
    input wire bvalid,output wire bready,input wire [1:0] bresp
);
    localparam IDLE=0,CMD=1,DATA=2,AW=3,W=4,B=5,GAP=6;
    reg [2:0] state;
    reg [31:0] command,timer;
    reg [7:0] div;
    reg [7:0] bits;
    reg [1:0] profile;
    reg [15:0] word_index;
    reg [127:0] word_q;
    reg [7:0] byte_q;
    reg timeout_pending;
    assign observe_boot={word_index[13:0],profile,state,timeout_pending};
    assign spi_mosi=command[31];
    assign awaddr=DDR_BASE+{14'd0,profile,16'd0}+{12'd0,word_index,4'd0};
    assign awlen=0;assign awsize=4;assign awburst=1;assign awvalid=state==AW;
    assign wdata=word_q;assign wstrb=16'hffff;assign wlast=1;assign wvalid=state==W;assign bready=state==B;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;spi_clk<=0;spi_cs_n<=1;asset_ready<=0;busy<=0;done<=0;failed<=0;
            command<=0;timer<=0;div<=0;bits<=0;profile<=FIRST_PROFILE;word_index<=0;word_q<=0;byte_q<=0;
            timeout_pending<=0;
        end else begin
            done<=0;
            if(start&&!busy)begin
                state<=CMD;spi_cs_n<=0;spi_clk<=0;busy<=1;failed<=0;asset_ready<=0;
                command<={8'h03,FLASH_BASE+{6'd0,FIRST_PROFILE,16'd0}};
                div<=0;bits<=0;profile<=FIRST_PROFILE;word_index<=0;timer<=0;
                timeout_pending<=0;
            end else if(busy)begin
                timer<=timer+1;
                if(timer>=TIMEOUT_CYCLES-1&&!timeout_pending)begin
                    failed<=1;timeout_pending<=1;spi_clk<=0;spi_cs_n<=1;
                    // Once AW transferred, finish this W and consume B. A
                    // timeout is a fault, not cancellation of an AXI transfer.
                    case(state)
                        AW:if(awready)state<=W;else begin state<=IDLE;busy<=0;end
                        W:if(wready)state<=B;
                        B:if(bvalid)begin state<=IDLE;busy<=0;end
                        default:begin state<=IDLE;busy<=0;end
                    endcase
                end
                else case(state)
                    CMD,DATA:begin
                        if(div==HALF_PERIOD-1)begin
                            div<=0;
                            if(!spi_clk)begin
                                spi_clk<=1;
                                if(state==DATA)begin
                                    byte_q<={byte_q[6:0],spi_miso};
                                    if(bits[2:0]==7)word_q[bits[6:3]*8+:8]<={byte_q[6:0],spi_miso};
                                end
                            end else begin
                                spi_clk<=0;
                                if(state==CMD)begin
                                    command<=command<<1;
                                    if(bits==31)begin state<=DATA;bits<=0;end else bits<=bits+1;
                                end else if(bits==127)begin state<=AW;bits<=0;end
                                else bits<=bits+1;
                            end
                        end else div<=div+1;
                    end
                    AW:if(awready)state<=W;
                    W:if(wready)state<=B;
                    B:if(bvalid)begin
                        if(bresp!=0||timeout_pending)begin state<=IDLE;busy<=0;failed<=1;spi_cs_n<=1;end
                        else if(word_index==PROFILE_BEATS-1)begin
                            asset_ready[profile]<=1;spi_cs_n<=1;div<=0;
                            if(profile==LAST_PROFILE)begin state<=IDLE;busy<=0;done<=1;end
                            else begin profile<=profile+1;state<=GAP;end
                        end else begin word_index<=word_index+1;state<=DATA;div<=0;end
                    end
                    GAP:if(div==31)begin
                        word_index<=0;bits<=0;div<=0;spi_cs_n<=0;
                        command<={8'h03,FLASH_BASE+{6'd0,profile,16'd0}};state<=CMD;
                    end else div<=div+1;
                    default:begin state<=IDLE;busy<=0;failed<=1;spi_cs_n<=1;end
                endcase
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(FIRST_PROFILE<1||FIRST_PROFILE>LAST_PROFILE||LAST_PROFILE>3)
            $fatal(1,"boot supports retained style IDs 1 through 3");
        if(HALF_PERIOD<1||HALF_PERIOD>256||PROFILE_BEATS<1||PROFILE_BEATS>65536)
            $fatal(1,"invalid flash timing/profile length");
    end
    // synthesis translate_on
endmodule
