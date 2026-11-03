`timescale 1ns/1ps
module camera_capture_recovery #(
    parameter integer QUIESCENT_CYCLES=4,
    parameter integer RESET_CYCLES=128,
    parameter integer RELEASE_CYCLES=32
)(
    input wire clk,rst_n,
    input wire fault,quiescent,
    output reg pipeline_release_n,
    output reg [31:0] recovery_count,
    output reg [31:0] fault_count,
    output wire recovering
);
    localparam [1:0] RUN=0,DRAIN=1,RESET=2,RELEASE=3;
    localparam integer MAX_WAIT=(RESET_CYCLES>RELEASE_CYCLES) ? RESET_CYCLES : RELEASE_CYCLES;
    localparam integer WAIT_BITS=(MAX_WAIT<2) ? 1 : $clog2(MAX_WAIT);
    localparam integer QUIET_BITS=(QUIESCENT_CYCLES<2) ? 1 : $clog2(QUIESCENT_CYCLES);
    reg [1:0] state;
    reg [QUIET_BITS-1:0] quiet_count;
    reg [WAIT_BITS-1:0] wait_count;
    assign recovering=state!=RUN;

    // fault and quiescent originate in this core clock domain. The top must
    // asynchronously assert and synchronously release each affected clock's
    // local reset from pipeline_release_n. No CSI, DDR, or display reset is
    // generated here. Quiescent must include all outstanding write responses.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=RUN;pipeline_release_n<=0;
            quiet_count<=0;wait_count<=0;
            recovery_count<=0;fault_count<=0;
        end else begin
            case(state)
            RUN:begin
                pipeline_release_n<=1;quiet_count<=0;wait_count<=0;
                if(fault)begin state<=DRAIN;fault_count<=fault_count+1'b1;end
            end
            DRAIN:begin
                pipeline_release_n<=1;
                if(!quiescent)quiet_count<=0;
                else if(quiet_count==QUIESCENT_CYCLES-1)begin
                    state<=RESET;pipeline_release_n<=0;
                    quiet_count<=0;wait_count<=0;
                end else quiet_count<=quiet_count+1'b1;
            end
            RESET:begin
                pipeline_release_n<=0;
                if(wait_count==RESET_CYCLES-1)begin
                    state<=RELEASE;pipeline_release_n<=1;wait_count<=0;
                    recovery_count<=recovery_count+1'b1;
                end else wait_count<=wait_count+1'b1;
            end
            RELEASE:begin
                pipeline_release_n<=1;
                if(wait_count==RELEASE_CYCLES-1)begin
                    state<=RUN;wait_count<=0;
                end else wait_count<=wait_count+1'b1;
            end
            default:begin
                // An illegal controller state does not justify interrupting
                // an AXI transaction. Re-enter DRAIN before any local reset.
                state<=DRAIN;pipeline_release_n<=1;quiet_count<=0;wait_count<=0;
            end
            endcase
        end
    end
    // synthesis translate_off
    initial if(QUIESCENT_CYCLES<1||RESET_CYCLES<1||RELEASE_CYCLES<1)
        $fatal(1,"CAPTURE_RECOVERY_PARAMETERS");
    // synthesis translate_on
endmodule
