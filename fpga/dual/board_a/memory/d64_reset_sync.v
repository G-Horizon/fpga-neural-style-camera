// Asynchronous assertion, synchronous release, for one clock domain.
//
// This exists because reset synchronisation belongs to the *domain* and not to any one
// module inside it.  The FIFO used to carry its own copy, which meant its reset was
// released two clocks later than everything around it -- a latency the module hid and the
// caller could not see.  A testbench that counted writes from the moment it released the
// external reset counted writes the FIFO dropped while its private reset was still
// asserted, and every word after that was one out; the honest fix is not a longer wait
// but removing the hidden state.
//
// So every user of this domain takes the same reset: the producer, the FIFO's write port,
// the consumer, the FIFO's read port.  There is one release per domain and nothing in the
// middle of a domain wakes up late.
//
// The flops carry `async_reg` because they are a reset synchroniser and the physical tools
// need to know that, exactly as the pointer synchronisers do.
module d64_reset_sync #(
    parameter STAGES = 2
) (
    input  wire clk,
    input  wire arst_n,      // asynchronous, asserted low
    output wire rst_n        // synchronous release, asserted low
);

    (* async_reg = "true" *) reg [STAGES-1:0] sync;

    always @(posedge clk or negedge arst_n)
        if (!arst_n) sync <= {STAGES{1'b0}};
        else         sync <= {sync[STAGES-2:0], 1'b1};

    assign rst_n = sync[STAGES-1];

endmodule
