`timescale 1ns/1ps
`default_nettype none
module camera_raw_trace(
 input wire byte_clk,byte_rst_n,armed,input wire [63:0] sample,
 input wire [3:0] trigger_rd,trigger_empty,trigger_hs,
 input wire read_clk,read_rst_n,input wire [9:0] read_addr,
 output reg [63:0] read_data,output reg ready
);
 reg [63:0] memory [0:1023];
 reg [9:0] write_addr,start_hold,start_read,post_left,arm_age;
 reg [4:0] history;
 reg triggered,done,rd_previous;
 reg [3:0] empty_previous,hs_previous;
 (* async_reg="true" *) reg [1:0] arm_sync,done_sync;
 reg [7:0] hs_length;reg hs_any_previous;
 wire hs_any=|trigger_hs;
 wire short_end=!hs_any&&hs_any_previous&&hs_length>=2&&hs_length<=32;
 wire [7:0] trigger_flags={7'd0,short_end};
 wire trigger_now=!triggered&&arm_sync[1]&&history==16&&short_end;
 always @(posedge byte_clk or negedge byte_rst_n)begin
  if(!byte_rst_n)begin hs_length<=0;hs_any_previous<=0;end
  else begin
   hs_any_previous<=hs_any;
   if(!hs_any)hs_length<=0;
   else if(hs_length!=255)hs_length<=hs_length+1'b1;
  end
 end
 always @(posedge byte_clk)
  if(byte_rst_n&&!done)memory[write_addr]<={sample[63:60],trigger_now?trigger_flags[3:0]:4'd0,sample[55:0]};
 always @(posedge byte_clk or negedge byte_rst_n)begin
  if(!byte_rst_n)begin
   arm_sync<=0;write_addr<=0;start_hold<=0;post_left<=0;history<=0;arm_age<=0;
   triggered<=0;done<=0;rd_previous<=0;empty_previous<=4'hf;hs_previous<=0;
  end else begin
   arm_sync<={arm_sync[0],armed};rd_previous<=|trigger_rd;
   empty_previous<=trigger_empty;hs_previous<=trigger_hs;
   if(arm_sync[1]&&!triggered&&arm_age!=1023)arm_age<=arm_age+1'b1;
   if(!done)begin
    write_addr<=write_addr+1'b1;
    if(history<16)history<=history+1'b1;
    if(trigger_now)begin
     triggered<=1;start_hold<=write_addr-10'd16;post_left<=10'd1007;
    end else if(triggered)begin
     if(post_left==1)begin done<=1;post_left<=0;end
     else post_left<=post_left-1'b1;
    end
   end
  end
 end
 // Frozen dual-clock RAM. Data is never overwritten once done is asserted.
 // The held start pointer settles long before its synchronized done token.
 always @(posedge read_clk or negedge read_rst_n)begin
  if(!read_rst_n)begin done_sync<=0;start_read<=0;ready<=0;end
  else begin
   done_sync<={done_sync[0],done};
   if(done_sync[1]&&!ready)begin start_read<=start_hold;ready<=1;end
  end
 end
 wire [9:0] physical_read_addr=start_read+read_addr;
 always @(posedge read_clk)read_data<=memory[physical_read_addr];
endmodule
`default_nettype wire
