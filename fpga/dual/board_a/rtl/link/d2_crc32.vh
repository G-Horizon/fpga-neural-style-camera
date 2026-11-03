function [31:0] d2_crc32_byte;
    input [31:0] crc;
    input [7:0] data;
    reg [31:0] q;
    integer k;
    begin
        q=crc ^ {24'd0,data};
        for(k=0;k<8;k=k+1) q=(q>>1) ^ (q[0] ? 32'hedb88320 : 32'd0);
        d2_crc32_byte=q;
    end
endfunction
