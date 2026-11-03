// Store an entire application payload before emitting a standard Ethernet frame.
// All app-facing signals/counters use app_clk; GMII/sent_frames use tx_clk.
module eth_packet_tx #(
    parameter MAX_APP_BYTES=1024, parameter ADDR_WIDTH=(MAX_APP_BYTES<=1024?10:11),
    parameter [47:0] LOCAL_MAC=48'h020000000001,
    parameter [47:0] PEER_MAC=48'h020000000002,
    parameter [15:0] ETHERTYPE=16'h88b5
)(
    input app_clk, app_rst_n, tx_clk, tx_rst_n,
    input [7:0] s_tx_data, input s_tx_valid, s_tx_last,
    output s_tx_ready,
    input tx_enable,
    output reg [7:0] gmii_txd, output reg gmii_tx_en,
    output gmii_tx_er,
    output reg [31:0] queued_frames, oversize_frames, sent_frames
);
    reg [1:0] tx_request, tx_ack;
    (* async_reg="true" *) reg [1:0] tx_ack_s1, tx_ack_s2;
    (* async_reg="true" *) reg [1:0] tx_request_s1, tx_request_s2;
    reg [10:0] tx_length_held0, tx_length_held1;
    reg write_bank, dropping;
    reg [10:0] write_index;
    wire accept=s_tx_valid && s_tx_ready;
    wire bank_free=(tx_request[write_bank]==tx_ack_s2[write_bank]);
    assign s_tx_ready=app_rst_n && (dropping || bank_free);
    wire wr0=accept && !dropping && !write_bank;
    wire wr1=accept && !dropping && write_bank;
    reg read_bank, next_bank;
    reg [10:0] payload_length, payload_index;
    wire [7:0] q0,q1;
    localparam IDLE=0, PREAMBLE=1, HEADER=2, PAYLOAD=3, FCS=4, GAP=5;
    reg [2:0] state;
    reg [4:0] index;
    wire [ADDR_WIDTH-1:0] read_address=(state==PAYLOAD && payload_index+1<payload_length) ? payload_index+1'b1 : {ADDR_WIDTH{1'b0}};
    eth_packet_ram #(ADDR_WIDTH,MAX_APP_BYTES) u_bank0(
        .wr_clk(app_clk),.wr_en(wr0),.wr_addr(write_index[ADDR_WIDTH-1:0]),.wr_data(s_tx_data),
        .rd_clk(tx_clk),.rd_addr(read_address),.rd_data(q0));
    eth_packet_ram #(ADDR_WIDTH,MAX_APP_BYTES) u_bank1(
        .wr_clk(app_clk),.wr_en(wr1),.wr_addr(write_index[ADDR_WIDTH-1:0]),.wr_data(s_tx_data),
        .rd_clk(tx_clk),.rd_addr(read_address),.rd_data(q1));
    always @(posedge app_clk or negedge app_rst_n) begin
        if(!app_rst_n) begin
            tx_ack_s1<=0;tx_ack_s2<=0;tx_request<=0;
            tx_length_held0<=0;tx_length_held1<=0;write_bank<=0;
            write_index<=0;dropping<=0;queued_frames<=0;oversize_frames<=0;
        end else begin
            tx_ack_s1<=tx_ack;tx_ack_s2<=tx_ack_s1;
            if(accept) begin
                if(dropping) begin
                    if(s_tx_last) begin dropping<=0;write_index<=0;oversize_frames<=oversize_frames+1'b1;end
                end else if(s_tx_last) begin
                    if(write_bank) tx_length_held1<=write_index+1'b1;
                    else tx_length_held0<=write_index+1'b1;
                    tx_request[write_bank]<=~tx_request[write_bank];
                    write_bank<=~write_bank;write_index<=0;queued_frames<=queued_frames+1'b1;
                end else if(write_index==MAX_APP_BYTES-1) begin
                    dropping<=1;write_index<=0;
                end else write_index<=write_index+1'b1;
            end
        end
    end
    function [31:0] crc_byte;
        input [31:0] crc; input [7:0] data;
        reg [31:0] c;integer n;
        begin c=crc^data;for(n=0;n<8;n=n+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;crc_byte=c;end
    endfunction
    function [7:0] header_byte;
        input [4:0] p;
        begin
            if(p<6) header_byte=PEER_MAC>>((5-p)*8);
            else if(p<12) header_byte=LOCAL_MAC>>((11-p)*8);
            else if(p==12) header_byte=ETHERTYPE[15:8];
            else header_byte=ETHERTYPE[7:0];
        end
    endfunction
    reg [31:0] crc;
    wire [7:0] payload_byte=(payload_index<payload_length) ? (read_bank?q1:q0) : 8'h00;
    assign gmii_tx_er=1'b0;
    always @(posedge tx_clk or negedge tx_rst_n) begin
        if(!tx_rst_n) begin
            tx_request_s1<=0;tx_request_s2<=0;tx_ack<=0;state<=IDLE;
            next_bank<=0;read_bank<=0;payload_length<=0;payload_index<=0;
            index<=0;crc<=32'hffffffff;gmii_txd<=0;gmii_tx_en<=0;sent_frames<=0;
        end else begin
            tx_request_s1<=tx_request;tx_request_s2<=tx_request_s1;
            gmii_txd<=0;gmii_tx_en<=0;
            case(state)
                IDLE: if(tx_enable && tx_request_s2[next_bank]!=tx_ack[next_bank]) begin
                    read_bank<=next_bank;
                    payload_length<=next_bank?tx_length_held1:tx_length_held0;
                    index<=0;crc<=32'hffffffff;state<=PREAMBLE;
                end
                PREAMBLE: begin
                    gmii_tx_en<=1;gmii_txd<=(index==7)?8'hd5:8'h55;
                    if(index==7) begin index<=0;state<=HEADER;end else index<=index+1'b1;
                end
                HEADER: begin
                    gmii_tx_en<=1;gmii_txd<=header_byte(index);crc<=crc_byte(crc,header_byte(index));
                    if(index==13) begin payload_index<=0;state<=PAYLOAD;end else index<=index+1'b1;
                end
                PAYLOAD: begin
                    gmii_tx_en<=1;gmii_txd<=payload_byte;crc<=crc_byte(crc,payload_byte);
                    if(payload_index+1>=payload_length && payload_index>=45) begin state<=FCS;index<=0;end
                    else payload_index<=payload_index+1'b1;
                end
                FCS: begin
                    gmii_tx_en<=1;gmii_txd<=~(crc>>(index*8));
                    if(index==3) begin
                        tx_ack[read_bank]<=tx_request_s2[read_bank];next_bank<=~read_bank;
                        sent_frames<=sent_frames+1'b1;index<=0;state<=GAP;
                    end else index<=index+1'b1;
                end
                GAP: if(index==11) begin index<=0;state<=IDLE;end else index<=index+1'b1;
                default: state<=IDLE;
            endcase
        end
    end
endmodule
