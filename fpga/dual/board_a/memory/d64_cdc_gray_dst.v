// The destination half of a Gray-coded counter crossing.  Destination domain only.
//
// The other half is `d64_cdc_gray_src.v`; read its header first for why the class is
// the coding.  This half is the synchroniser and the decode.
//
// ---------------------------------------------------------------------------
// What the destination may and may not see
// ---------------------------------------------------------------------------
//
// * It MAY see a stale value.  The pointer lags by STAGES clocks and that is what a
//   synchronised pointer is for.
// * It MAY see a value that was skipped, if the source moved on more than once before
//   the destination looked.  The Gray code passes through every intermediate value, so
//   a skipped value is a value that was never sampled, not a value that was never
//   written.
// * It may NEVER see a value the source did not hold.  That is the invariant, and it
//   is what the destination half cannot establish on its own -- it holds only if the
//   crossed bus is Gray, which is the other half's job.  The gate for it is
//   `tb_cdc_gray.v`, whose tooth compiles the same destination against a binary bus.
//
// The decode is the standard recursive XOR, the same function `d64_async_fifo` uses.
// It is a function of the *settled* word (`g_s2`), never of the first synchroniser
// stage: decoding a metastable word is how a one-bit change turns into a many-bit
// change at the very back of the crossing.
module d64_cdc_gray_dst #(
    parameter W      = 6,
    parameter STAGES = 2
) (
    input  wire         dst_clk,
    input  wire         dst_rst_n,
    input  wire [W-1:0] x_gray,      // the crossing, driven by the source half
    output wire [W-1:0] dst_gray,    // the settled Gray word
    output wire [W-1:0] dst_bin      // and its decode: the destination's view
);
    (* async_reg = "true" *) reg [W-1:0] g_s1;
    (* async_reg = "true" *) reg [W-1:0] g_s2;

    always @(posedge dst_clk or negedge dst_rst_n)
        if (!dst_rst_n) begin
            g_s1 <= {W{1'b0}};
            g_s2 <= {W{1'b0}};
        end
        else begin
            g_s1 <= x_gray;
            g_s2 <= g_s1;
        end

    function [W-1:0] gray_to_binary;
        input [W-1:0] gray;
        integer gi;
        begin
            gray_to_binary[W-1] = gray[W-1];
            for (gi = W-2; gi >= 0; gi = gi - 1)
                gray_to_binary[gi] = gray_to_binary[gi+1] ^ gray[gi];
        end
    endfunction

    assign dst_gray = g_s2;
    assign dst_bin  = gray_to_binary(g_s2);

endmodule
