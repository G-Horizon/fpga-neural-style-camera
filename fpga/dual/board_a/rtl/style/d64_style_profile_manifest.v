`timescale 1ns/1ps
// Generated from PROFILES.json; a header cannot relabel another payload.
module d64_style_profile_manifest(input wire [1:0] style,output reg [31:0] crc,output reg [9:0] trunk_zero);
always @* begin case(style)
2'd1:begin crc=32'h20f0cd7e;trunk_zero=10'd497;end
2'd2:begin crc=32'h4478795a;trunk_zero=10'd512;end
2'd3:begin crc=32'hc4069d86;trunk_zero=10'd402;end
default:begin crc=0;trunk_zero=0;end
endcase end
endmodule
