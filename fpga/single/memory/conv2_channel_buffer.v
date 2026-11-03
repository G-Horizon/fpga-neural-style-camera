// Single-output-channel INT32 buffer for the two-pass Conv2 InstanceNorm
// flow.  Conv2 writes one sample per output pixel during Pass1; after the
// channel boundary handshake, InstanceNorm reads the same samples in order
// during Pass2.  The first implementation deliberately uses one shared
// memory instead of ping-pong buffers so the ownership boundary is explicit.

module conv2_channel_buffer #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 16384,
    parameter ADDR_WIDTH = 14
)(
    input wire                         clk,
    input wire                         rst_n,

    input wire                         write_en,
    input wire [ADDR_WIDTH-1:0]        write_addr,
    input wire signed [DATA_WIDTH-1:0] write_data,

    input wire                         read_en,
    input wire [ADDR_WIDTH-1:0]        read_addr,
    output reg                         read_valid,
    output reg signed [DATA_WIDTH-1:0] read_data
);

    reg signed [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // The memory contents are intentionally not reset.  Every address is
    // written during Pass1 before it can be read during Pass2.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_valid <= 1'b0;
            read_data  <= {DATA_WIDTH{1'b0}};
        end
        else begin
            read_valid <= read_en;

            if (write_en)
                mem[write_addr] <= write_data;

            if (read_en)
                read_data <= mem[read_addr];
        end
    end

endmodule
