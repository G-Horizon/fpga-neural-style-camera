`timescale 1ns/1ps
// Eight native guides stay leased from camera commit through remote compute
// and display retirement. All ports below belong to the application clock.
module d2e_a_slots(
 input wire clk,rst_n,session_valid,input wire[31:0]local_session,remote_session,
 input wire capture_commit,input wire[2:0]capture_slot,input wire[31:0]capture_fid,
 output reg release_valid,output reg[2:0]release_slot,output wire[7:0]owned_mask,
 input wire[1:0]ui_style,input wire ui_fps,input wire[7:0]ui_effect,input wire[31:0]ui_epoch,
 input wire compute_ready,compute_done,compute_fault,output reg compute_start,
 output reg[2:0]compute_slot,output wire[1:0]compute_profile,output wire[31:0]compute_fid,
 output wire tx_job_valid,input wire tx_job_ready,output wire[31:0]tx_base,tx_fid,tx_epoch,
 output wire[1:0]tx_style,tx_layout,output wire tx_fps,output wire[7:0]tx_effect,
 input wire tx_done,tx_done_ok,input wire[31:0]tx_done_fid,
 input wire allocate_valid,output wire allocate_ready,allocate_ok,output wire[31:0]allocate_base,
 input wire[31:0]allocate_fid,allocate_epoch,input wire[1:0]allocate_style,allocate_layout,
 input wire allocate_fps,input wire[7:0]allocate_effect,
 input wire commit_valid,commit_cancel,output wire commit_ready,
 input wire[31:0]commit_base,commit_fid,commit_epoch,input wire[1:0]commit_style,commit_layout,
 input wire commit_fps,input wire[7:0]commit_effect,
 input wire abort_valid,input wire[31:0]abort_base,abort_fid,
 output reg publish_valid,input wire publish_ready,output wire[31:0]publish_base,publish_fid,publish_epoch,
 output wire[2:0]publish_slot,output wire publish_fps,
 input wire active_valid,input wire[31:0]active_fid,active_epoch,active_base,
 input wire[7:0]reader_protect_mask,
 output reg fault,output reg[31:0]computed_frames,returned_frames,published_frames,retired_frames,discarded_frames
);
 localparam FREE=0,CAPTURED=1,COMPUTING=2,COMPUTED=3,WAITING=4,RESULT=5,PENDING=6,DISPLAY=7,RETIRING=8;
 reg[3:0]state[0:7];reg[31:0]fid[0:7],epoch[0:7];
 reg[1:0]style[0:7];reg fps[0:7];reg[7:0]effect[0:7];
 reg[7:0]stale,rx_lease,feature_acked;reg[3:0]retire_guard[0:7];
 reg compute_busy,tx_busy,publish_busy,tx_offer_valid;reg[2:0]tx_slot,pub_slot;
 reg previous_session_valid;reg[31:0]previous_local,previous_remote;
 wire session_change=previous_session_valid&&(!session_valid||local_session!=previous_local||remote_session!=previous_remote);
 wire session_current=session_valid&&!session_change;
 // Unaccepted jobs are cancellable at a session boundary. Mask the offer in
 // that same cycle so a ready receiver cannot accept the old-session job.
 // Within one session valid and all metadata remain held until acceptance.
 assign tx_job_valid=tx_offer_valid&&session_current;
 function[31:0]result_address;input[2:0]s;begin result_address=32'h03000000+{11'd0,s,18'd0};end endfunction
 function[31:0]tensor_address;input[2:0]s;begin tensor_address=32'h01300000+{7'd0,s,22'd0};end endfunction
 function valid_result_address;input[31:0]b;begin valid_result_address=(b&32'hffe3ffff)==32'h03000000;end endfunction
 assign compute_profile=compute_busy?style[compute_slot]+2'd1:ui_style;
 assign compute_fid=fid[compute_slot];
 assign tx_base=tensor_address(tx_slot);assign tx_fid=fid[tx_slot];assign tx_epoch=epoch[tx_slot];
 assign tx_style=style[tx_slot];assign tx_layout=0;assign tx_fps=fps[tx_slot];assign tx_effect=effect[tx_slot];
 assign publish_slot=pub_slot;assign publish_base=result_address(pub_slot);assign publish_fid=fid[pub_slot];
 assign publish_epoch=epoch[pub_slot];assign publish_fps=fps[pub_slot];
 genvar g;generate for(g=0;g<8;g=g+1)begin:o assign owned_mask[g]=state[g]!=FREE;end endgenerate
 integer i,j;reg free_found,alloc_found,capture_duplicate;
 reg[2:0]free_pick,alloc_pick;
 wire cap_found,tx_found,pub_found;wire[2:0]cap_pick,tx_pick,pub_pick;
 wire[7:0]cap_eligible,tx_eligible,pub_eligible;wire[599:0]pick_metadata;
 wire[74:0]pick_word[0:7];reg[2:0]pick_index;wire[74:0]pick_current_meta=pick_word[pick_index];
 generate for(g=0;g<8;g=g+1)begin:pick_inputs
  assign cap_eligible[g]=state[g]==CAPTURED&&!stale[g];
  assign tx_eligible[g]=state[g]==COMPUTED&&!stale[g];
  assign pub_eligible[g]=state[g]==RESULT&&feature_acked[g]&&!stale[g];
  assign pick_word[g]={effect[g],fps[g],style[g],epoch[g],fid[g]};
  assign pick_metadata[g*75+:75]=pick_word[g];
 end endgenerate
 wire pick_restart=capture_commit||compute_done||tx_done||commit_valid||session_change;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)pick_index<=0;
  else if(!session_current||pick_restart)pick_index<=0;
  else pick_index<=pick_index+1'b1;
 wire cap_claim=!compute_busy&&compute_ready&&cap_found&&session_current&&!fault;
 wire tx_claim=!tx_busy&&!tx_offer_valid&&tx_found&&session_current&&!fault;
 wire pub_claim=!publish_busy&&!publish_valid&&pub_found&&session_current&&!fault;
 d2e_a_pick_scan u_cap_pick(clk,rst_n,session_current,local_session,remote_session,pick_restart,cap_eligible,pick_metadata,pick_index,pick_current_meta,cap_claim,cap_found,cap_pick);
 d2e_a_pick_scan u_tx_pick(clk,rst_n,session_current,local_session,remote_session,pick_restart,tx_eligible,pick_metadata,pick_index,pick_current_meta,tx_claim,tx_found,tx_pick);
 d2e_a_pick_scan u_pub_pick(clk,rst_n,session_current,local_session,remote_session,pick_restart,pub_eligible,pick_metadata,pick_index,pick_current_meta,pub_claim,pub_found,pub_pick);
 always @*begin
  free_found=0;alloc_found=0;capture_duplicate=0;
  free_pick=0;alloc_pick=0;
  for(i=0;i<8;i=i+1)begin
   if(state[i]!=FREE&&fid[i]==capture_fid)capture_duplicate=1;
   if(!free_found&&state[i]!=FREE&&stale[i]&&!rx_lease[i]&&
      !(compute_busy&&compute_slot==i)&&!(tx_busy&&tx_slot==i)&&!(tx_offer_valid&&tx_slot==i)&&
      state[i]!=PENDING&&state[i]!=DISPLAY&&!reader_protect_mask[i]&&
      !(capture_commit&&capture_slot==i))begin free_found=1;free_pick=i;end
   if(!alloc_found&&state[i]==WAITING&&!stale[i]&&!rx_lease[i]&&fid[i]==allocate_fid&&epoch[i]==allocate_epoch&&
      style[i]==allocate_style&&allocate_layout==0&&fps[i]==allocate_fps&&effect[i]==allocate_effect)begin alloc_found=1;alloc_pick=i;end
  end
 end
 assign allocate_ready=1;
 assign allocate_ok=session_current&&!fault&&alloc_found;
 assign allocate_base=result_address(alloc_pick);
 assign commit_ready=1;
 wire[2:0]commit_slot=commit_base[20:18],abort_slot=abort_base[20:18];
 wire commit_matches=valid_result_address(commit_base)&&rx_lease[commit_slot]&&state[commit_slot]==WAITING&&
  fid[commit_slot]==commit_fid&&epoch[commit_slot]==commit_epoch&&style[commit_slot]==commit_style&&
  commit_layout==0&&fps[commit_slot]==commit_fps&&effect[commit_slot]==commit_effect;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   release_valid<=0;release_slot<=0;compute_start<=0;compute_busy<=0;compute_slot<=0;
   tx_offer_valid<=0;tx_busy<=0;tx_slot<=0;publish_valid<=0;publish_busy<=0;pub_slot<=0;
   stale<=0;rx_lease<=0;feature_acked<=0;previous_session_valid<=0;previous_local<=0;previous_remote<=0;
   fault<=0;computed_frames<=0;returned_frames<=0;published_frames<=0;retired_frames<=0;discarded_frames<=0;
   for(j=0;j<8;j=j+1)begin state[j]<=FREE;fid[j]<=0;epoch[j]<=0;style[j]<=0;fps[j]<=1;effect[j]<=0;retire_guard[j]<=0;end
  end else begin
   release_valid<=0;compute_start<=0;
   previous_session_valid<=session_valid;previous_local<=local_session;previous_remote<=remote_session;
   if(capture_commit)begin
    if(state[capture_slot]!=FREE)fault<=1;
    // A recovery may restart source FIDs while an older guide is still live.
    // Collect only the new duplicate lease; never let it enter compute/RX.
    else begin state[capture_slot]<=CAPTURED;fid[capture_slot]<=capture_fid;stale[capture_slot]<=!session_current||capture_duplicate;feature_acked[capture_slot]<=0;rx_lease[capture_slot]<=0;end
   end
   if(cap_claim)begin
    compute_slot<=cap_pick;compute_busy<=1;compute_start<=1;state[cap_pick]<=COMPUTING;
    epoch[cap_pick]<=ui_epoch;style[cap_pick]<=ui_style-2'd1;fps[cap_pick]<=ui_fps;effect[cap_pick]<=ui_effect;
   end
   if(compute_done)begin
    if(!compute_busy||state[compute_slot]!=COMPUTING)fault<=1;
    else begin compute_busy<=0;state[compute_slot]<=COMPUTED;computed_frames<=computed_frames+1'b1;end
   end
   if(compute_fault)fault<=1;
   if(tx_claim)begin tx_slot<=tx_pick;tx_offer_valid<=1;end
   if(tx_job_valid&&tx_job_ready)begin tx_offer_valid<=0;tx_busy<=1;state[tx_slot]<=WAITING;end
   if(tx_done)begin
    if(!tx_busy||tx_done_fid!=fid[tx_slot])fault<=1;
    else begin tx_busy<=0;if(tx_done_ok)feature_acked[tx_slot]<=1;else stale[tx_slot]<=1;end
   end
   if(allocate_valid&&allocate_ok)rx_lease[alloc_pick]<=1;
   if(commit_valid)begin
    if(!commit_matches)fault<=1;
    else begin
     if(commit_cancel)begin
      // RX emits its abort pulse on the following edge. Keep that lease until
      // the pulse is consumed, so collection cannot recycle the guide first.
      stale[commit_slot]<=1;
     end else begin
      rx_lease[commit_slot]<=0;
      if(!session_current||stale[commit_slot])stale[commit_slot]<=1;
      else begin state[commit_slot]<=RESULT;returned_frames<=returned_frames+1'b1;end
     end
    end
   end
   if(abort_valid)begin
    if(!valid_result_address(abort_base)||fid[abort_slot]!=abort_fid||state[abort_slot]!=WAITING)fault<=1;
    else begin rx_lease[abort_slot]<=0;stale[abort_slot]<=1;end
   end
   if(pub_claim)begin
    pub_slot<=pub_pick;publish_valid<=1;publish_busy<=1;state[pub_pick]<=PENDING;
   end
   if(publish_valid&&publish_ready)publish_valid<=0;
   // A held publication survives link loss: its complete result and guide are
   // immutable until the pixel boundary acknowledges that same descriptor.
   if(publish_busy&&!publish_valid&&active_valid&&active_fid==fid[pub_slot]&&active_epoch==epoch[pub_slot]&&active_base==result_address(pub_slot))begin
    publish_busy<=0;state[pub_slot]<=DISPLAY;published_frames<=published_frames+1'b1;
    for(j=0;j<8;j=j+1)if(state[j]==DISPLAY)begin state[j]<=RETIRING;retire_guard[j]<=8;end
   end
   for(j=0;j<8;j=j+1)if(state[j]==RETIRING)begin
    if(retire_guard[j]!=0)retire_guard[j]<=retire_guard[j]-1'b1;
    else if(!reader_protect_mask[j])stale[j]<=1;
   end
   if(free_found)begin
    state[free_pick]<=FREE;stale[free_pick]<=0;feature_acked[free_pick]<=0;
    release_valid<=1;release_slot<=free_pick;retired_frames<=retired_frames+1'b1;
    if(state[free_pick]!=RETIRING)discarded_frames<=discarded_frames+1'b1;
   end
   if(session_change)begin
    tx_offer_valid<=0;
    for(j=0;j<8;j=j+1)if(state[j]!=FREE&&state[j]!=PENDING&&state[j]!=DISPLAY&&state[j]!=RETIRING)stale[j]<=1;
   end
  end
 end
endmodule

// A shared free-running index gives all three classes one75-bit source read
// port. Each private sweep starts only at index0 and visits all eight slots.
// A winning full identity is captured during the sweep and held in best_meta;
// no second indexed winner-metadata mux or duplicate ticket register is needed.
module d2e_a_pick_scan(
 input wire clk,rst_n,session_current,input wire[31:0]local_session,remote_session,
 input wire restart,input wire[7:0]eligible,input wire[599:0]metadata,
 input wire[2:0]index,input wire[74:0]current_meta,
 input wire take,output wire valid,output reg[2:0]pick
);
 reg[2:0]best_slot;reg best_valid,held,scanning;
 reg[31:0]best_fid,ticket_local,ticket_remote;reg[74:0]best_meta;
 wire[74:0]meta_word[0:7];
 genvar k;generate for(k=0;k<8;k=k+1)begin:constant_metadata_view
  assign meta_word[k]=metadata[k*75+:75];
 end endgenerate
 wire[31:0]current_fid=current_meta[31:0];
 wire better=eligible[index]&&(!best_valid||$signed(current_fid-best_fid)<0);
 wire winner_valid=best_valid||eligible[index];
 wire[2:0]winner=better?index:best_slot;
 wire[74:0]live_meta=meta_word[pick];
 assign valid=held&&!restart&&session_current&&ticket_local==local_session&&ticket_remote==remote_session&&eligible[pick]&&live_meta==best_meta;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin best_slot<=0;best_valid<=0;best_fid<=0;held<=0;scanning<=0;pick<=0;best_meta<=0;ticket_local<=0;ticket_remote<=0;end
  else if(!session_current||restart)begin best_valid<=0;held<=0;scanning<=0;end
  else if(held)begin
   if(!valid||take)begin best_valid<=0;held<=0;scanning<=0;end
  end else if(!scanning&&index!=0)best_valid<=0;
  else begin
   scanning<=1;
   if(better)begin best_valid<=1;best_slot<=index;best_fid<=current_fid;best_meta<=current_meta;end
   if(index==7)begin
    best_valid<=0;scanning<=0;held<=winner_valid;pick<=winner;
    if(winner_valid)begin ticket_local<=local_session;ticket_remote<=remote_session;end
   end
  end
 end
endmodule
