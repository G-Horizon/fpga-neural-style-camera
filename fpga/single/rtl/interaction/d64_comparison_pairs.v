`timescale 1ns/1ps
// Completion records are invalidated on slot allocation/retirement. The root
// must honor all display/reader/capture masks before recycling a slot. FID is
// full32; the half plane also records the style epoch. Invalidation wins over
// same-edge commits, so a late transaction cannot resurrect a reused slot.
module d64_comparison_pairs #(
    parameter integer HALF_INVALIDATION_ENABLE=0,
    parameter [31:0] ORIGINAL_BASE=32'h03000000,HALF_BASE=32'h03500000,
    parameter [31:0] SLOT_BYTES=32'h00100000
)(
    input wire clk,rst_n,input wire [4:0] invalidate_slots,
    input wire [4:0] invalidate_half_slots,
    input wire guide_commit,input wire [2:0] guide_slot,input wire [31:0] guide_fid,
    input wire original_commit,input wire [2:0] original_slot,input wire [31:0] original_fid,
    input wire half_commit,input wire [2:0] half_slot,input wire [31:0] half_fid,input wire [15:0] half_epoch,
    input wire [2:0] query_slot,input wire [31:0] query_fid,input wire [15:0] query_epoch,
    output wire query_valid,output wire [31:0] query_original_base,query_half_base,
    input wire [2:0] original_query_slot,input wire [31:0] original_query_fid,
    output wire original_query_valid,
    output wire [4:0] complete_slots
);
    reg [4:0] guide_valid,original_valid,half_valid;
    reg [31:0] guides[0:4],originals[0:4],halves[0:4];
    reg [15:0] epochs[0:4];
    integer i;
    wire [4:0] original_guide_ready;
    generate genvar k;for(k=0;k<5;k=k+1)begin:g_complete
        assign original_guide_ready[k]=guide_valid[k]&&original_valid[k]&&
            guides[k]==originals[k];
        assign complete_slots[k]=original_guide_ready[k]&&half_valid[k]&&
            guides[k]==halves[k];
    end endgenerate
    assign query_valid=(query_slot<5)&&complete_slots[query_slot]&&guides[query_slot]==query_fid&&epochs[query_slot]==query_epoch;
    // The full-size wipe needs only an immutable original matching the full
    // render FID. A missing/rewritten half-size comparison plane is unrelated.
    // Allocation/failure invalidation suppresses a query immediately, before
    // the completion bits are cleared at the following PE edge.
    assign original_query_valid=(original_query_slot<5)&&!invalidate_slots[original_query_slot]&&
        original_guide_ready[original_query_slot]&&
        guides[original_query_slot]==original_query_fid;
    assign query_original_base=ORIGINAL_BASE+query_slot*SLOT_BYTES;
    assign query_half_base=HALF_BASE+query_slot*SLOT_BYTES;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            guide_valid<=0;original_valid<=0;half_valid<=0;
            for(i=0;i<5;i=i+1)begin guides[i]<=0;originals[i]<=0;halves[i]<=0;epochs[i]<=0;end
        end else begin
            if(guide_commit&&(guide_slot<5)&&!invalidate_slots[guide_slot])begin guide_valid[guide_slot]<=1;guides[guide_slot]<=guide_fid;end
            if(original_commit&&(original_slot<5)&&!invalidate_slots[original_slot])begin original_valid[original_slot]<=1;originals[original_slot]<=original_fid;end
            if(half_commit&&(half_slot<5)&&!invalidate_slots[half_slot]&&
                !(HALF_INVALIDATION_ENABLE!=0&&invalidate_half_slots[half_slot]))begin
                half_valid[half_slot]<=1;halves[half_slot]<=half_fid;epochs[half_slot]<=half_epoch;
            end
            for(i=0;i<5;i=i+1)begin
                if(invalidate_slots[i])begin guide_valid[i]<=0;original_valid[i]<=0;half_valid[i]<=0;end
                else if(HALF_INVALIDATION_ENABLE!=0&&invalidate_half_slots[i])half_valid[i]<=0;
            end
        end
    end
endmodule
