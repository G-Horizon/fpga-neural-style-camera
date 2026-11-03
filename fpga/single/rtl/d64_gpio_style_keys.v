`timescale 1ns/1ps
// Board keys are active low. Synchronize every key, then require a stable
// transition for 2 ms. One request per press; holding a key does not reload.
module d64_gpio_style_keys #(parameter integer DEBOUNCE_CYCLES=200000)(
    input wire clk,input wire rst_n,input wire [3:0] keys_n,
    output reg request_valid,output reg [1:0] request_style
);
reg[3:0] sync0,sync1,stable;
reg[31:0] count[0:3];integer i;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin sync0<=15;sync1<=15;stable<=15;request_valid<=0;request_style<=0;for(i=0;i<4;i=i+1)count[i]<=0;end
 else begin
  sync0<=keys_n;sync1<=sync0;request_valid<=0;
  for(i=0;i<4;i=i+1)begin
   if(sync1[i]==stable[i])count[i]<=0;
   else if(count[i]==DEBOUNCE_CYCLES-1)begin
    stable[i]<=sync1[i];count[i]<=0;
    if(!sync1[i])begin request_valid<=1;request_style<=i;end
   end else count[i]<=count[i]+1;
  end
 end
end
endmodule
