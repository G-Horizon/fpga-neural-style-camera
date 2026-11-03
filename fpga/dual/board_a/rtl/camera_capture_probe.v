`timescale 1ns/1ps
module camera_capture_probe(input wire clk,rst_n,pix_valid,vsync,input wire [3:0] ppc,input wire high_bits,mode_verified,output wire [191:0] debug);
reg vsq,armed,high_seen;reg [3:0] last_ppc;reg [25:0] packing_bad;
reg [31:0] frames,words,frame_words,last_words,min_words,max_words;
wire fs=vsync&&!vsq;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin vsq<=0;armed<=0;frames<=0;words<=0;frame_words<=0;last_words<=0;min_words<=32'hffffffff;max_words<=0;last_ppc<=0;packing_bad<=0;high_seen<=0;end
 else begin
 vsq<=vsync;
 if(pix_valid)begin words<=words+1'b1;last_ppc<=ppc;if(high_bits)high_seen<=1;if((ppc!=4||high_bits)&&!(&packing_bad))packing_bad<=packing_bad+1'b1;end
 if(fs)begin
 frames<=frames+1'b1;armed<=1;frame_words<=pix_valid?1:0;
 if(armed)begin last_words<=frame_words;if(frame_words<min_words)min_words<=frame_words;if(frame_words>max_words)max_words<=frame_words;end
 end else if(pix_valid&&armed)frame_words<=frame_words+1'b1;
 end
end
assign debug={frames,words,last_words,min_words,max_words,mode_verified,high_seen,last_ppc,packing_bad};
endmodule
