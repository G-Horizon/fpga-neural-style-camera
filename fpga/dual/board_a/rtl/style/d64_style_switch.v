`timescale 1ns/1ps
// Requests are sampled in the PE/control clock domain. Synchronize/debounce GPIO
// before this module. Keep the last accepted request; do not queue four old jobs.
// safe_to_load means current job retired AND every PE/FIFO/AXI response drained.
// Display remains on its old committed frontbuffer independently of this FSM.
module d64_style_switch #(
    parameter [1:0] BOOT_STYLE=1,
    parameter [3:0] ALLOWED_STYLE_MASK=4'b1110,
    parameter integer BOOT_VALID=1,
    parameter integer SWITCH_BUDGET_CYCLES=10000000,
    parameter integer BOOT_SOURCE_ARM_ENABLE=0
) (
    input wire clk,input wire rst_n,
    input wire request_valid,input wire [1:0] request_style,
    input wire request_is_boot,bootstrap_source_ready,
    input wire [3:0] asset_ready,
    input wire safe_to_load,
    output reg load_start,output reg [1:0] load_style,
    input wire load_done,input wire load_failed,input wire load_invalidate,
    output wire block_new_jobs,output reg model_valid,
    output reg [1:0] active_style,output reg [15:0] active_epoch,
    output reg profile_publish,
    // Assert when first scan of a complete new-epoch display buffer starts.
    input wire display_commit,input wire [15:0] display_epoch,
    output reg switch_done,output reg switch_fault,output reg deadline_missed,
    output reg [31:0] switch_cycles,
    output wire boot_wait_source,output reg boot_source_arm,boot_active,
    output reg [31:0] boot_total_cycles
);
    localparam READY=0,QUIESCE=1,LOAD=2,WAIT_DISPLAY=3,FAULT=4;
    reg [2:0] state;
    reg pending;reg [1:0] pending_style;
    reg boot_pending,boot_source_seen;
    assign boot_wait_source=boot_pending&&!boot_source_seen;
    reg displayed_valid;reg [15:0] displayed_epoch;
    assign block_new_jobs=(state==QUIESCE)||(state==LOAD)||(state==FAULT)||!model_valid;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=READY;pending<=0;pending_style<=BOOT_STYLE;load_style<=BOOT_STYLE;
            displayed_valid<=0;displayed_epoch<=0;
            load_start<=0;active_style<=BOOT_STYLE;active_epoch<=0;model_valid<=BOOT_VALID;
            profile_publish<=0;switch_done<=0;switch_fault<=0;deadline_missed<=0;switch_cycles<=0;
            boot_pending<=0;boot_source_seen<=0;boot_source_arm<=0;boot_active<=0;boot_total_cycles<=0;
        end else begin
            load_start<=0;profile_publish<=0;switch_done<=0;switch_fault<=0;boot_source_arm<=0;
            // Automatic cold boot has no renderable input before the first
            // complete RGB+guide pair. Its render-visible stopwatch arms once.
            // Every human request retains request-to-visible timing, including
            // an override while boot is waiting or still loading.
            if(state!=READY&&state!=FAULT)begin
                if(!boot_pending||boot_source_seen)begin
                    switch_cycles<=switch_cycles+1;
                    if(switch_cycles>=SWITCH_BUDGET_CYCLES-1)deadline_missed<=1;
                end
                // Loading cannot hide behind an absent camera: preserve the
                // original budget from automatic request through model load.
                if(boot_pending&&!model_valid&&boot_total_cycles>=SWITCH_BUDGET_CYCLES-1)
                    deadline_missed<=1;
            end
            if(boot_active)begin
                if(boot_total_cycles!=32'hffffffff)boot_total_cycles<=boot_total_cycles+1;
                if(!boot_source_seen&&bootstrap_source_ready)begin
                    boot_source_seen<=1;boot_source_arm<=1;
                    if(boot_pending)switch_cycles<=0;
                end
                // Record total cold-start wall time independently of any
                // operator request that superseded the automatic boot style.
                if(display_commit&&model_valid&&display_epoch==active_epoch&&(!boot_pending||boot_source_seen))
                    boot_active<=0;
            end
            if(load_invalidate)model_valid<=0;
            if(display_commit)begin displayed_valid<=1;displayed_epoch<=display_epoch;end
            if(request_valid)begin
                boot_pending<=(BOOT_SOURCE_ARM_ENABLE!=0)&&request_is_boot&&
                    !model_valid&&(active_epoch==0);
                if((BOOT_SOURCE_ARM_ENABLE!=0)&&request_is_boot&&!model_valid&&(active_epoch==0))begin
                    boot_active<=1;boot_total_cycles<=0;
                    boot_source_seen<=bootstrap_source_ready;
                    if(bootstrap_source_ready)boot_source_arm<=1;
                end
                pending<=1;pending_style<=request_style;
                switch_cycles<=0;deadline_missed<=0;
                if(state==READY||state==FAULT||state==WAIT_DISPLAY)begin
                    state<=QUIESCE;switch_cycles<=0;deadline_missed<=0;
                end
            end
            case(state)
                READY:begin end
                QUIESCE:if(safe_to_load&&pending&&!request_valid)begin
                    if(!ALLOWED_STYLE_MASK[pending_style]||!asset_ready[pending_style])begin
                        switch_fault<=1;state<=model_valid?READY:FAULT;pending<=0;
                    end else if(pending_style==active_style&&model_valid)begin
                        // A duplicate request during LOAD still has to wait for
                        // the first new-epoch scan; active weights are not a
                        // displayed frame. An already displayed same style may
                        // finish immediately without loading it again.
                        pending<=0;
                        if((displayed_valid&&displayed_epoch==active_epoch)||
                           (display_commit&&display_epoch==active_epoch))begin
                            state<=READY;switch_done<=1;
                        end else state<=WAIT_DISPLAY;
                    end else begin
                        load_style<=pending_style;load_start<=1;pending<=0;state<=LOAD;
                    end
                end
                LOAD:begin
                    if(load_failed)begin switch_fault<=1;state<=FAULT;end
                    else if(load_done)begin
                        model_valid<=1;active_style<=load_style;active_epoch<=active_epoch+1;
                        profile_publish<=1;
                        if(pending||request_valid)state<=QUIESCE;
                        else state<=WAIT_DISPLAY;
                    end
                end
                WAIT_DISPLAY:if(!request_valid&&display_commit&&display_epoch==active_epoch&&(!boot_pending||boot_source_seen))begin
                    switch_done<=1;state<=READY;boot_pending<=0;
                end
                FAULT:begin end
                default:begin state<=FAULT;switch_fault<=1;model_valid<=0;end
            endcase
        end
    end
    // synthesis translate_off
    initial if(ALLOWED_STYLE_MASK[0]||!ALLOWED_STYLE_MASK[BOOT_STYLE])
        $fatal(1,"style switch requires retained boot style and excludes ID0");
    // synthesis translate_on
endmodule
