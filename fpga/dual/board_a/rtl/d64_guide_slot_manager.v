`timescale 1ns/1ps
// Five raw-guide DDR slots. All ownership comparisons use full 32-bit FIDs.
// capture_slot_valid is an availability level; the writer locks it on the
// source's first pixel and reports capture_started with that slot and FID.
module d64_guide_slot_manager #(parameter integer EXTERNAL_PROTECTION=0,SOURCE_QUERY_ENABLE=0)(
    input wire clk,rst_n,
    input wire source_first,
    input wire [31:0] source_fid,
    output wire capture_slot_valid,
    output reg [2:0] capture_slot,
    input wire capture_started,
    input wire [2:0] capture_started_slot,
    input wire [31:0] capture_started_fid,
    input wire guide_committed,
    input wire [2:0] guide_committed_slot,
    input wire [31:0] guide_committed_fid,
    input wire guide_failed,
    input wire [2:0] guide_failed_slot,
    input wire [31:0] guide_failed_fid,
    input wire camera_ready_valid,
    input wire [31:0] camera_ready_fid,
    input wire [31:0] source_query_fid,
    output wire source_query_allocated,
    output wire camera_ready_guide_ready,
    input wire camera_claim,
    input wire [31:0] camera_claim_fid,
    input wire job_release,
    input wire publish_commit,
    input wire publish_output_slot,
    input wire retire_output_slot,
    input wire retire_output_index,
    input wire [1:0] result_protect_mask,
    input wire [4:0] external_protected_slots,
    input wire display_valid,
    input wire display_output_slot,
    input wire reader_outstanding,
    input wire [2:0] reader_guide_slot,
    output reg job_guide_slot_valid,
    output reg [2:0] job_guide_slot,
    output wire [31:0] job_guide_fid,
    output wire job_guide_ready,
    output reg [1:0] published_guide_valid,
    output reg [2:0] published_guide_slot0,published_guide_slot1,
    output wire [31:0] published_guide_fid0,published_guide_fid1,
    output wire [4:0] protected_slots,
    output wire [4:0] committed_slots,
    output reg [31:0] drop_no_slot,
    output reg [31:0] protocol_fault
);
    reg [4:0] allocated,capturing,committed;
    reg job_active;
    reg [31:0] fid0,fid1,fid2,fid3,fid4;
    function [31:0] owner_fid;
        input [2:0] slot;
        begin case(slot)
            0:owner_fid=fid0;1:owner_fid=fid1;
            2:owner_fid=fid2;3:owner_fid=fid3;4:owner_fid=fid4;default:owner_fid=0;
        endcase end
    endfunction
    function [4:0] slot_mask;
        input [2:0] slot;
        begin case(slot)
            0:slot_mask=5'b00001;1:slot_mask=5'b00010;
            2:slot_mask=5'b00100;3:slot_mask=5'b01000;4:slot_mask=5'b10000;default:slot_mask=0;
        endcase end
    endfunction
    wire [4:0] ready_mask={allocated[4]&&(fid4==camera_ready_fid),
        allocated[3]&&(fid3==camera_ready_fid),
        allocated[2]&&(fid2==camera_ready_fid),
        allocated[1]&&(fid1==camera_ready_fid),
        allocated[0]&&(fid0==camera_ready_fid)}&{5{camera_ready_valid}};
    // Query the FIFO's full FID, not merely the latest RAW header. This also
    // handles a delayed RGB SOF after a newer RAW source has arrived.
    assign source_query_allocated=SOURCE_QUERY_ENABLE!=0&&
        ((allocated[0]&&fid0==source_query_fid)||(allocated[1]&&fid1==source_query_fid)||
         (allocated[2]&&fid2==source_query_fid)||(allocated[3]&&fid3==source_query_fid)||(allocated[4]&&fid4==source_query_fid));
    assign camera_ready_guide_ready=|(ready_mask&committed);
    wire [4:0] claim_matches={allocated[4]&&(fid4==camera_claim_fid),
        allocated[3]&&(fid3==camera_claim_fid),
        allocated[2]&&(fid2==camera_claim_fid),
        allocated[1]&&(fid1==camera_claim_fid),
        allocated[0]&&(fid0==camera_claim_fid)};
    wire [4:0] commit_matches={fid4==guide_committed_fid,fid3==guide_committed_fid,fid2==guide_committed_fid,
        fid1==guide_committed_fid,fid0==guide_committed_fid};
    wire [4:0] failed_matches={fid4==guide_failed_fid,fid3==guide_failed_fid,fid2==guide_failed_fid,
        fid1==guide_failed_fid,fid0==guide_failed_fid};
    wire retire_allowed=!result_protect_mask[retire_output_index]&&
        (!display_valid||(display_output_slot!=retire_output_index));
    wire publish_allowed=!result_protect_mask[publish_output_slot]&&
        (!display_valid||(display_output_slot!=publish_output_slot));
    wire claim_allowed=!job_active||job_release||(publish_commit&&publish_allowed);
    wire [4:0] incoming_claim_mask=claim_matches&{5{camera_claim&&claim_allowed}};
    wire protect_result0=published_guide_valid[0]&&
        (result_protect_mask[0]||(display_valid&&!display_output_slot));
    wire protect_result1=published_guide_valid[1]&&
        (result_protect_mask[1]||(display_valid&&display_output_slot));
    assign protected_slots=(EXTERNAL_PROTECTION!=0?external_protected_slots:5'd0)|capturing|ready_mask|incoming_claim_mask|
        (job_guide_slot_valid?slot_mask(job_guide_slot):5'd0)|
        (protect_result0?slot_mask(published_guide_slot0):5'd0)|
        (protect_result1?slot_mask(published_guide_slot1):5'd0)|
        (reader_outstanding?slot_mask(reader_guide_slot):5'd0);
    wire [4:0] free_slots=~protected_slots;
    assign capture_slot_valid=rst_n&&(|free_slots);
    always @*begin
        if(free_slots[0])capture_slot=0;
        else if(free_slots[1])capture_slot=1;
        else if(free_slots[2])capture_slot=2;
        else if(free_slots[3])capture_slot=3;
        else capture_slot=4;
    end
    assign committed_slots=committed;
    // The owner is immutable while job_guide_slot_valid protects this slot.
    // Therefore its full FID remains exactly the FID matched at claim time;
    // another 32-bit copy and equality network would duplicate that invariant.
    assign job_guide_fid=job_guide_slot_valid?owner_fid(job_guide_slot):32'd0;
    assign job_guide_ready=job_guide_slot_valid&&allocated[job_guide_slot]&&
        committed[job_guide_slot];
    assign published_guide_fid0=published_guide_valid[0]?owner_fid(published_guide_slot0):32'd0;
    assign published_guide_fid1=published_guide_valid[1]?owner_fid(published_guide_slot1):32'd0;
    wire start_allowed=(capture_started_slot<5)&&free_slots[capture_started_slot]&&
        (!source_first||(capture_started_fid==source_fid));
    wire commit_allowed=(guide_committed_slot<5)&&allocated[guide_committed_slot]&&capturing[guide_committed_slot]&&
        commit_matches[guide_committed_slot];
    wire failed_allowed=(guide_failed_slot<5)&&allocated[guide_failed_slot]&&capturing[guide_failed_slot]&&
        failed_matches[guide_failed_slot];
    wire same_edge_start_claim=capture_started&&start_allowed&&
        (capture_started_fid==camera_claim_fid);
    wire unique_claim=(claim_matches==5'b00001)||(claim_matches==5'b00010)||
        (claim_matches==5'b00100)||(claim_matches==5'b01000)||(claim_matches==5'b10000);
    wire duplicate_claim=(|claim_matches)&&!unique_claim;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            allocated<=0;capturing<=0;committed<=0;job_active<=0;
            fid0<=0;fid1<=0;fid2<=0;fid3<=0;fid4<=0;
            job_guide_slot_valid<=0;job_guide_slot<=0;
            published_guide_valid<=0;published_guide_slot0<=0;published_guide_slot1<=0;
            drop_no_slot<=0;protocol_fault<=0;
        end else begin
            if(source_first&&!capture_slot_valid)drop_no_slot<=32'd1;
            if(capture_started)begin
                if(start_allowed)begin
                    allocated[capture_started_slot]<=1;
                    capturing[capture_started_slot]<=1;
                    committed[capture_started_slot]<=0;
                    case(capture_started_slot)
                        0:fid0<=capture_started_fid;1:fid1<=capture_started_fid;
                        2:fid2<=capture_started_fid;3:fid3<=capture_started_fid;4:fid4<=capture_started_fid;
                    endcase
                    // Old inactive descriptors never point to the new owner.
                    if(published_guide_valid[0]&&(published_guide_slot0==capture_started_slot))published_guide_valid[0]<=0;
                    if(published_guide_valid[1]&&(published_guide_slot1==capture_started_slot))published_guide_valid[1]<=0;
                end else protocol_fault<=32'd1;
            end
            if(guide_committed)begin
                if(commit_allowed)begin
                    committed[guide_committed_slot]<=1;capturing[guide_committed_slot]<=0;
                end else protocol_fault<=32'd1;
            end
            if(guide_failed)begin
                if(failed_allowed)begin
                    committed[guide_failed_slot]<=0;capturing[guide_failed_slot]<=0;
                end else protocol_fault<=32'd1;
            end
            if(guide_committed&&guide_failed&&
                guide_committed_slot==guide_failed_slot)protocol_fault<=32'd1;
            if(retire_output_slot)begin
                if(retire_allowed)published_guide_valid[retire_output_index]<=0;
                else protocol_fault<=32'd1;
            end
            if(publish_commit)begin
                if(publish_allowed)begin
                published_guide_valid[publish_output_slot]<=job_guide_ready;
                if(!publish_output_slot)published_guide_slot0<=job_guide_slot;
                else published_guide_slot1<=job_guide_slot;
                job_guide_slot_valid<=0;job_active<=0;
                end else protocol_fault<=32'd1;
            end
            if(job_release)begin job_guide_slot_valid<=0;job_active<=0;end
            if(camera_claim)begin
                if(!claim_allowed)protocol_fault<=32'd1;
                else begin
                job_active<=1;
                if(duplicate_claim)begin job_guide_slot_valid<=0;protocol_fault<=32'd1;end
                else if(unique_claim)begin
                    job_guide_slot_valid<=1;
                    case(claim_matches)
                        5'b00001:job_guide_slot<=0;5'b00010:job_guide_slot<=1;
                        5'b00100:job_guide_slot<=2;5'b01000:job_guide_slot<=3;5'b10000:job_guide_slot<=4;
                    endcase
                end else if(same_edge_start_claim)begin
                    job_guide_slot_valid<=1;job_guide_slot<=capture_started_slot;
                end else job_guide_slot_valid<=0;
                end
            end
        end
    end
endmodule
