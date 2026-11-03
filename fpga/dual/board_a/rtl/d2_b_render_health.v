`timescale 1ns/1ps
// Counts a result only after its last qualified visible pixel. Raster scans
// and repeated presentations of the same full FID/epoch have separate counts.
module d2_b_render_health #(
 parameter integer W=1920,H=1080
)(
 input wire clk,rst_n,de,good,health_fault,input wire[11:0]ax,ay,
 input wire[31:0]fid,epoch,
 output reg[31:0]complete_frames,distinct_frames,repeated_frames,rejected_frames,
 output reg qualified_pulse,output reg[31:0]last_complete_fid,last_complete_epoch
);
 reg open,bad,seen;reg[11:0]expected_x,expected_y;reg[31:0]count,frame_fid,frame_epoch;
 wire first=de&&ax==0&&ay==0;
 wire last=de&&ax==W-1&&ay==H-1;
 wire current_bad=!good||health_fault||fid!=frame_fid||epoch!=frame_epoch||ax!=expected_x||ay!=expected_y;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   open<=0;bad<=0;seen<=0;expected_x<=0;expected_y<=0;count<=0;frame_fid<=0;frame_epoch<=0;
   complete_frames<=0;distinct_frames<=0;repeated_frames<=0;rejected_frames<=0;qualified_pulse<=0;
   last_complete_fid<=0;last_complete_epoch<=0;
  end else begin
   qualified_pulse<=0;
   if(first)begin
    if(open)rejected_frames<=rejected_frames+1'b1;
    open<=1;bad<=!good||health_fault;count<=1;frame_fid<=fid;frame_epoch<=epoch;
    expected_x<=1;expected_y<=0;
   end else if(de&&open)begin
    if(current_bad)bad<=1;
    count<=count+1'b1;
    if(expected_x==W-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end
    else expected_x<=expected_x+1'b1;
    if(last)begin
     open<=0;
     if(!bad&&!current_bad&&count==W*H-1)begin
      complete_frames<=complete_frames+1'b1;qualified_pulse<=1;
      if(!seen||fid!=last_complete_fid||epoch!=last_complete_epoch)distinct_frames<=distinct_frames+1'b1;
      else repeated_frames<=repeated_frames+1'b1;
      last_complete_fid<=fid;last_complete_epoch<=epoch;seen<=1;
     end else rejected_frames<=rejected_frames+1'b1;
    end
   end
   if(open&&health_fault)bad<=1;
  end
 end
 // synthesis translate_off
 initial if(W<2||H<1||W>4096||H>4096)$fatal(1,"D2_RENDER_GEOMETRY");
 // synthesis translate_on
endmodule
