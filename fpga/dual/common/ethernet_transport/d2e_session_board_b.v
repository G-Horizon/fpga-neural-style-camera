`timescale 1ns/1ps
// A initiates a three-message session confirmation. Either role can request
// re-establishment. PHY negotiation is autonomous; this FSM never configures it.
module d2e_session #(
 parameter integer ROLE=0,QUIET_CYCLES=100000,RETRY_CYCLES=1000000,
 parameter integer PING_CYCLES=10000000,DEAD_CYCLES=50000000
)(
 input wire clk,rst_n,link_qualified,force_restart,
 input wire control_valid,output wire control_ready,input wire[479:0]control_header,
 output reg packet_valid,input wire packet_ready,output reg[479:0]packet_header,
 output reg session_valid,output reg[31:0]local_session,remote_session,
 output reg[31:0]established_sessions,restarts,rejected_controls
);
 `include "d2e_protocol.vh"
 localparam DOWN=0,QUIET=1,A_HELLO=2,A_CONFIRM=3,B_WAIT=4,RUN=5;
 reg[2:0]state;
 reg[31:0]timer,age,ping_timer;
 wire slot_free=!packet_valid||packet_ready;
 assign control_ready=slot_free;
 wire control_ok=control_header[63:0]==`D2E_MAGIC&&control_header[71:64]==1&&
  control_header[87:80]==0&&control_header[95:88]<=4&&control_header[335:320]==0&&
  control_header[479:160]==0;
 wire is_hello=control_header[79:72]==`D2E_HELLO&&control_header[95:88]==0;
 wire is_ack=control_header[79:72]==`D2E_HELLO_ACK;
 wire pair_matches=control_header[127:96]==remote_session&&control_header[159:128]==local_session;
 function[31:0]next_generation(input[31:0]old);begin next_generation=old==32'hffffffff ? 1 : old+1'b1;end endfunction
 function[479:0]make_control(input[7:0]kind,flags,input[31:0]local_id,peer_id);
 reg[479:0]h;
 begin
  h=0;h[63:0]=`D2E_MAGIC;h[71:64]=1;h[79:72]=kind;h[95:88]=flags;
  h[127:96]=local_id;h[159:128]=peer_id;make_control=h;
 end endfunction
 task send_control(input[7:0]kind,flags,input[31:0]local_id,peer_id);
 begin packet_header<=make_control(kind,flags,local_id,peer_id);packet_valid<=1;timer<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=DOWN;timer<=0;age<=0;ping_timer<=0;packet_valid<=0;packet_header<=0;
   session_valid<=0;local_session<=0;remote_session<=0;established_sessions<=0;restarts<=0;rejected_controls<=0;
  end else begin
   if(packet_valid&&packet_ready)packet_valid<=0;
   if(!link_qualified||force_restart)begin
    if(session_valid)restarts<=restarts+1'b1;
    state<=DOWN;timer<=0;age<=0;ping_timer<=0;session_valid<=0;
   end else begin
    case(state)
     DOWN:begin state<=QUIET;timer<=0;session_valid<=0;remote_session<=0;end
     QUIET:begin
      if(timer==QUIET_CYCLES-1&&slot_free)begin
       timer<=RETRY_CYCLES-1;
       if(ROLE==0)begin local_session<=next_generation(local_session);state<=A_HELLO;end
       else state<=B_WAIT;
      end else if(timer<QUIET_CYCLES-1)timer<=timer+1'b1;
     end
     A_HELLO:begin
      if(timer==RETRY_CYCLES-1&&slot_free)send_control(`D2E_HELLO,0,local_session,0);
      else if(timer<RETRY_CYCLES-1)timer<=timer+1'b1;
     end
     A_CONFIRM:begin
      if(timer==RETRY_CYCLES-1&&slot_free)send_control(`D2E_HELLO_ACK,1,local_session,remote_session);
      else if(timer<RETRY_CYCLES-1)timer<=timer+1'b1;
     end
     B_WAIT:begin
      if(timer==RETRY_CYCLES-1&&slot_free)begin
       if(remote_session==0)send_control(`D2E_HELLO,0,0,0);
       else send_control(`D2E_HELLO_ACK,0,local_session,remote_session);
      end else if(timer<RETRY_CYCLES-1)timer<=timer+1'b1;
     end
     RUN:begin
      age<=age+1'b1;
      if(age==DEAD_CYCLES-1)begin state<=DOWN;session_valid<=0;restarts<=restarts+1'b1;end
      else if(ping_timer==PING_CYCLES-1&&slot_free)begin
       send_control(`D2E_HELLO_ACK,3,local_session,remote_session);ping_timer<=0;
      end else if(ping_timer<PING_CYCLES-1)ping_timer<=ping_timer+1'b1;
     end
    endcase
    if(control_valid&&control_ready)begin
     if(!control_ok||(control_header[79:72]!=`D2E_HELLO&&control_header[79:72]!=`D2E_HELLO_ACK))rejected_controls<=rejected_controls+1'b1;
     else if(ROLE==0)begin
      if(is_hello)begin
       // A B-reboot request cannot inherit an old A generation, even when
       // B restarted with the same reset counter as a previous connection.
       if(state==RUN)begin local_session<=next_generation(local_session);session_valid<=0;restarts<=restarts+1'b1;end
       if(state!=DOWN&&state!=QUIET)begin state<=A_HELLO;remote_session<=0;timer<=RETRY_CYCLES-1;end
      end else if(is_ack&&control_header[159:128]==local_session&&control_header[127:96]!=0)begin
       if(control_header[95:88]==0&&(state==A_HELLO||state==A_CONFIRM||state==RUN))begin
        if(state==RUN&&remote_session!=control_header[127:96])begin session_valid<=0;restarts<=restarts+1'b1;end
        remote_session<=control_header[127:96];state<=A_CONFIRM;
        send_control(`D2E_HELLO_ACK,1,local_session,control_header[127:96]);
       end else if(control_header[95:88]==2&&state==A_CONFIRM&&pair_matches)begin
        session_valid<=1;state<=RUN;age<=0;ping_timer<=0;established_sessions<=established_sessions+1'b1;
       end else if(state==RUN&&pair_matches&&control_header[95:88]==3)begin
        age<=0;send_control(`D2E_HELLO_ACK,4,local_session,remote_session);
       end else if(state==RUN&&pair_matches&&control_header[95:88]==4)age<=0;
       else rejected_controls<=rejected_controls+1'b1;
      end else rejected_controls<=rejected_controls+1'b1;
     end else begin
      if(is_hello&&control_header[127:96]!=0&&control_header[159:128]==0&&state!=DOWN&&state!=QUIET)begin
       if(state!=B_WAIT||remote_session!=control_header[127:96])begin
        local_session<=next_generation(local_session);
        send_control(`D2E_HELLO_ACK,0,next_generation(local_session),control_header[127:96]);
       end else send_control(`D2E_HELLO_ACK,0,local_session,control_header[127:96]);
       if(session_valid)restarts<=restarts+1'b1;
       remote_session<=control_header[127:96];session_valid<=0;state<=B_WAIT;
      end else if(is_ack&&pair_matches&&control_header[95:88]==1&&(state==B_WAIT||state==RUN))begin
       if(!session_valid)established_sessions<=established_sessions+1'b1;
       session_valid<=1;state<=RUN;age<=0;ping_timer<=0;
       send_control(`D2E_HELLO_ACK,2,local_session,remote_session);
      end else if(is_ack&&state==RUN&&pair_matches&&control_header[95:88]==3)begin
       age<=0;send_control(`D2E_HELLO_ACK,4,local_session,remote_session);
      end else if(is_ack&&state==RUN&&pair_matches&&control_header[95:88]==4)age<=0;
      else rejected_controls<=rejected_controls+1'b1;
     end
    end
   end
  end
 end
 // synthesis translate_off
 initial if((ROLE!=0&&ROLE!=1)||QUIET_CYCLES<2||RETRY_CYCLES<2||DEAD_CYCLES<=PING_CYCLES)$fatal(1,"D2E_SESSION_PARAMETERS");
 // synthesis translate_on
endmodule
