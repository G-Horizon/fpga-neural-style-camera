// Snapshot mailbox for observer counters only. No functional state is changed.
// Source held bus stays immutable until another request is accepted. Requests
// to a stopped domain remain pending; another live domain can still refresh.
module eth_diag_status_mailbox #(parameter WIDTH=32)(
    input source_clk,source_rst_n,input [WIDTH-1:0] source_words,
    input app_clk,app_rst_n,input request,
    output reg [WIDTH-1:0] app_words,output reg fresh,output busy
);
    reg request_toggle,ack_toggle,pending;
    (* async_reg="true" *)reg request_s1,request_s2,ack_s1,ack_s2;
    reg [WIDTH-1:0] source_held;
    assign busy=pending;
    always @(posedge source_clk or negedge source_rst_n)begin
        if(!source_rst_n)begin request_s1<=0;request_s2<=0;ack_toggle<=0;source_held<=0;end
        else begin
            request_s1<=request_toggle;request_s2<=request_s1;
            if(request_s2!=ack_toggle)begin source_held<=source_words;ack_toggle<=request_s2;end
        end
    end
    always @(posedge app_clk or negedge app_rst_n)begin
        if(!app_rst_n)begin request_toggle<=0;ack_s1<=0;ack_s2<=0;pending<=0;app_words<=0;fresh<=0;end
        else begin
            ack_s1<=ack_toggle;ack_s2<=ack_s1;
            if(request)begin
                fresh<=0;
                if(!pending)begin request_toggle<=~request_toggle;pending<=1;end
            end
            if(pending && ack_s2==request_toggle)begin app_words<=source_held;pending<=0;fresh<=1;end
        end
    end
endmodule

module eth_diag_status_cdc(
    input app_clk,app_rst_n,request,
    input tx_clk,tx_rst_n,input [31:0] tx_sent_frames,
    input rx_clk,rx_rst_n,input [223:0] rx_counters,
    output [255:0] app_mac_words,output [1:0] fresh_mask,busy_mask
);
    eth_diag_status_mailbox #(32) u_tx_status(tx_clk,tx_rst_n,tx_sent_frames,
        app_clk,app_rst_n,request,app_mac_words[31:0],fresh_mask[0],busy_mask[0]);
    eth_diag_status_mailbox #(224) u_rx_status(rx_clk,rx_rst_n,rx_counters,
        app_clk,app_rst_n,request,app_mac_words[255:32],fresh_mask[1],busy_mask[1]);
endmodule
