`timescale 1ns/1ps
// A one-shot diagnostic snapshot. Held payload never changes again before reset.
// The independent request has two synchronization flops and two extra settling
// cycles. The held launch -> data_q capture is explicitly constrained to 8 ns.
module d2_diag_snapshot #(parameter W=96)(
 input wire src_clk,src_rst_n,trigger,input wire [W-1:0] source_data,
 input wire dst_clk,dst_rst_n,output wire [W-1:0] data,output wire valid);
 reg [W-1:0] held;reg request;
 (* async_reg="true" *)reg [1:0] request_sync;
 reg [1:0] settle;reg [W-1:0] data_q;reg valid_q;
 always @(posedge src_clk or negedge src_rst_n)
 if(!src_rst_n)begin held<=0;request<=0;end
 else if(trigger&&!request)begin held<=source_data;request<=1;end
 always @(posedge dst_clk or negedge dst_rst_n)
 if(!dst_rst_n)begin request_sync<=0;settle<=0;data_q<=0;valid_q<=0;end
 else begin
 request_sync<={request_sync[0],request};
 if(request_sync[1]&&!valid_q)begin
 if(settle==2)begin data_q<=held;valid_q<=1;end else settle<=settle+1'b1;
 end end
 assign data=data_q;assign valid=valid_q;
endmodule
