`timescale 1ns/1ps
module tb_eth_diag_status;
    reg ac=0,tc=0,rc=0,rst_n=0,rrun=1,request=0;
    always #5 ac=~ac;always #4 tc=~tc;always #4.013 if(rrun)rc=~rc;
    reg [31:0] tx_count=0,rx_count=0;
    wire [223:0] rx_words={rx_count+6,rx_count+5,rx_count+4,rx_count+3,rx_count+2,rx_count+1,rx_count};
    wire [255:0] words;wire [1:0] fresh,busy;
    eth_diag_status_cdc dut(ac,rst_n,request,tc,rst_n,tx_count,rc,rst_n,rx_words,words,fresh,busy);
    always @(posedge tc)if(!rst_n)tx_count<=0;else tx_count<=tx_count+1;
    always @(posedge rc)if(!rst_n)rx_count<=0;else rx_count<=rx_count+1;
    task capture;
        begin @(negedge ac);request=1;@(negedge ac);request=0;repeat(40)@(posedge ac);end
    endtask
    integer j;reg [31:0] previous_tx;
    task coherent;
        begin for(j=1;j<7;j=j+1)if(words[32+j*32+:32]!==words[63:32]+j)$fatal(1,"RX coherent source counter group");end
    endtask
    initial begin
        #31 rst_n=1;repeat(10)@(posedge ac);capture();
        if(fresh!=3 || busy)$fatal(1,"live snapshot");coherent();previous_tx=words[31:0];
        @(negedge rc);rrun=0;capture();
        if(fresh!=1 || busy!=2 || words[31:0]<=previous_tx)$fatal(1,"stopped RX blocked live TX report");
        previous_tx=words[31:0];capture();
        if(fresh!=1 || busy!=2 || words[31:0]<=previous_tx)$fatal(1,"repeated report toggled pending stopped request");
        rrun=1;repeat(40)@(posedge ac);if(fresh!=3||busy)$fatal(1,"resumed RX request lost");coherent();
        @(negedge rc);rrun=0;capture();#1 rst_n=0;#1;
        if(words||fresh||busy)$fatal(1,"stopped mailbox reset");
        #21 rst_n=1;rrun=1;repeat(20)@(posedge ac);
        if(words||fresh||busy)$fatal(1,"old mailbox epoch republished");
        capture();if(fresh!=3)$fatal(1,"new mailbox epoch");coherent();
        $display("STATUS_CDC_STOPPED_RESET_PASS");$finish;
    end
    initial begin #20000;$fatal(1,"status timeout");end
endmodule
