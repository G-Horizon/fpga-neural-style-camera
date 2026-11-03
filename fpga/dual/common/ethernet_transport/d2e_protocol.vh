`ifndef D2E_PROTOCOL_VH
`define D2E_PROTOCOL_VH
// Little-endian byte fields. EtherType is 0x88B5; MAC/FCS is a separate layer.
`define D2E_MAGIC 64'h3148544530366954
`define D2E_VERSION 8'd1
`define D2E_HEADER_BYTES 64
`define D2E_MAX_DATA 960
`define D2E_FEATURE_BYTES 147456
`define D2E_RESULT_BYTES 147456
`define D2E_HELLO 8'd1
`define D2E_HELLO_ACK 8'd2
`define D2E_BEGIN 8'd3
`define D2E_BEGIN_ACK 8'd4
`define D2E_DATA 8'd5
`define D2E_DATA_ACK 8'd6
`define D2E_END 8'd7
`define D2E_END_ACK 8'd8
`define D2E_ABORT 8'd9
`define D2E_NAK 8'd10
`define D2E_STREAM_FEATURE 8'd1
`define D2E_STREAM_RGBX 8'd2
// Header: magic0..7/version8/type9/stream10/flags11;
// local_session12/peer_session16/FID20/style_epoch24/message_sequence28;
// total_bytes32/offset36/data_length40(u16)/style42/layout43/FPS44/effect45;
// status46/reserved47/frame_crc48/lease_token52/reserved56(u32)/header_crc60.
`endif
function [31:0] d2e_crc_byte;
 input [31:0] previous;
 input [7:0] value;
 integer bit_n;reg [31:0] c;
 begin
  c=previous^{24'd0,value};
  for(bit_n=0;bit_n<8;bit_n=bit_n+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;
  d2e_crc_byte=c;
 end
endfunction
