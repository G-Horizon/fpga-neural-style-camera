module d2_word128_to_bytes24 (
    input wire clk, rst_n, clear,
    input wire [127:0] in_data,
    input wire in_valid,
    output wire in_ready,
    output wire [23:0] out_data,
    output wire [5:0] available,
    input wire [1:0] take,
    output reg fault_sticky
);
    reg [255:0] buffer;
    reg [5:0] count;
    wire take_ok=({4'd0,take}<=count);
    wire [5:0] used=take_ok ? {4'd0,take} : 6'd0;
    wire [5:0] remaining=count-used;
    assign in_ready=remaining<=16;
    assign out_data=buffer[23:0];
    assign available=count;
    reg [255:0] next_buffer;
    reg [5:0] next_count;
    always @* begin
        next_buffer=buffer >> (used*8);
        next_count=remaining;
        if(in_valid && in_ready) begin
            next_buffer=next_buffer | ({128'd0,in_data} << (remaining*8));
            next_count=remaining+16;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin buffer<=0;count<=0;fault_sticky<=0;end
        else if(clear) begin buffer<=0;count<=0;end
        else begin
            buffer<=next_buffer;count<=next_count;
            if(!take_ok) fault_sticky<=1;
        end
    end
endmodule
