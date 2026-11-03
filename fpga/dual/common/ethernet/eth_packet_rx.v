// Validate a complete frame in RXC domain before publishing an immutable bank.
// FCS remains in a four-byte tail window and is never delivered as payload.
module eth_packet_rx #(
    parameter MAX_APP_BYTES=1024, parameter ADDR_WIDTH=(MAX_APP_BYTES<=1024?10:11),
    parameter [47:0] LOCAL_MAC=48'h020000000001,
    parameter [15:0] ETHERTYPE=16'h88b5,
    parameter ACCEPT_BROADCAST=0
)(
    input app_clk, app_rst_n, rx_clk, rx_rst_n,
    input [7:0] gmii_rxd, input gmii_rx_dv, gmii_rx_er, rx_enable,
    output reg [7:0] m_rx_data, output m_rx_valid, input m_rx_ready,
    output m_rx_last, output reg [10:0] m_rx_length,
    output reg [47:0] m_rx_src_mac,
    output reg [31:0] good_frames, bad_fcs, bad_header, bad_length,
    output reg [31:0] phy_errors, no_buffer, bad_preamble,
    output reg [31:0] delivered_frames
);
    function [31:0] crc_byte;
        input [31:0] crc;input [7:0] data;reg [31:0] c;integer n;
        begin c=crc^data;for(n=0;n<8;n=n+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_byte=c;end
    endfunction
    reg [1:0] rx_request,rx_ack;
    (* async_reg="true" *) reg [1:0] rx_ack_s1,rx_ack_s2;
    (* async_reg="true" *) reg [1:0] rx_request_s1,rx_request_s2;
    reg [10:0] rx_length_held0,rx_length_held1;
    reg [47:0] rx_source_held0,rx_source_held1;
    reg write_bank,next_write_bank,missing_bank,frame_error;
    reg [1:0] state;
    localparam SEARCH=0,PRE=1,FRAME=2,IGNORE=3;
    reg [3:0] pre_count;
    reg [15:0] byte_count;
    reg [31:0] crc,tail;
    reg [47:0] destination,source;
    reg [15:0] protocol;
    wire length_ok=(byte_count>=64 && byte_count<=MAX_APP_BYTES+18);
    wire header_ok=(destination==LOCAL_MAC || (ACCEPT_BROADCAST && destination==48'hffffffffffff)) && protocol==ETHERTYPE;
    wire crc_ok=(crc==32'hdebb20e3);
    wire ram_write=(state==FRAME && gmii_rx_dv && !missing_bank && byte_count>=18 && byte_count<MAX_APP_BYTES+18);
    wire [ADDR_WIDTH-1:0] write_address=byte_count-18;
    reg read_bank,next_read_bank;
    reg [10:0] read_index;
    localparam APP_IDLE=0,APP_LOAD=1,APP_SEND=2;
    reg [1:0] app_state;
    wire [ADDR_WIDTH-1:0] read_address=(app_state==APP_SEND && m_rx_ready && !m_rx_last)?read_index+1'b1:read_index;
    wire [7:0] q0,q1;
    eth_packet_ram #(ADDR_WIDTH,MAX_APP_BYTES) u_bank0(
        .wr_clk(rx_clk),.wr_en(ram_write&&!write_bank),.wr_addr(write_address),.wr_data(tail[7:0]),
        .rd_clk(app_clk),.rd_addr(read_address),.rd_data(q0));
    eth_packet_ram #(ADDR_WIDTH,MAX_APP_BYTES) u_bank1(
        .wr_clk(rx_clk),.wr_en(ram_write&&write_bank),.wr_addr(write_address),.wr_data(tail[7:0]),
        .rd_clk(app_clk),.rd_addr(read_address),.rd_data(q1));
    always @(posedge rx_clk or negedge rx_rst_n) begin
        if(!rx_rst_n) begin
            rx_ack_s1<=0;rx_ack_s2<=0;rx_request<=0;
            rx_length_held0<=0;rx_length_held1<=0;rx_source_held0<=0;rx_source_held1<=0;
            state<=SEARCH;pre_count<=0;byte_count<=0;crc<=32'hffffffff;tail<=0;
            destination<=0;source<=0;protocol<=0;write_bank<=0;next_write_bank<=0;
            missing_bank<=0;frame_error<=0;good_frames<=0;bad_fcs<=0;bad_header<=0;
            bad_length<=0;phy_errors<=0;no_buffer<=0;bad_preamble<=0;
        end else begin
            rx_ack_s1<=rx_ack;rx_ack_s2<=rx_ack_s1;
            case(state)
                SEARCH: if(gmii_rx_dv) begin
                    if(rx_enable && gmii_rxd==8'h55 && !gmii_rx_er) begin state<=PRE;pre_count<=1;end
                    else begin state<=IGNORE;bad_preamble<=bad_preamble+1'b1;end
                end
                PRE: begin
                    if(!gmii_rx_dv) begin state<=SEARCH;bad_preamble<=bad_preamble+1'b1;end
                    else if(!rx_enable || gmii_rx_er) begin state<=IGNORE;bad_preamble<=bad_preamble+1'b1;end
                    else if(pre_count==7 && gmii_rxd==8'hd5) begin
                        state<=FRAME;byte_count<=0;crc<=32'hffffffff;tail<=0;
                        destination<=0;source<=0;protocol<=0;frame_error<=0;
                        write_bank<=next_write_bank;
                        missing_bank<=(rx_request[next_write_bank]!=rx_ack_s2[next_write_bank]);
                    end else if(pre_count<7 && gmii_rxd==8'h55) pre_count<=pre_count+1'b1;
                    else begin state<=IGNORE;bad_preamble<=bad_preamble+1'b1;end
                end
                FRAME: if(gmii_rx_dv) begin
                    if(byte_count!=16'hffff) byte_count<=byte_count+1'b1;
                    crc<=crc_byte(crc,gmii_rxd);tail<={gmii_rxd,tail[31:8]};
                    if(byte_count<6) destination<={destination[39:0],gmii_rxd};
                    else if(byte_count<12) source<={source[39:0],gmii_rxd};
                    else if(byte_count<14) protocol<={protocol[7:0],gmii_rxd};
                    if(gmii_rx_er || !rx_enable) frame_error<=1;
                end else begin
                    state<=SEARCH;
                    if(!crc_ok) bad_fcs<=bad_fcs+1'b1;
                    if(!header_ok) bad_header<=bad_header+1'b1;
                    if(!length_ok) bad_length<=bad_length+1'b1;
                    if(frame_error || !rx_enable) phy_errors<=phy_errors+1'b1;
                    if(missing_bank) no_buffer<=no_buffer+1'b1;
                    if(crc_ok && header_ok && length_ok && !frame_error && rx_enable && !missing_bank) begin
                        if(write_bank) begin rx_length_held1<=byte_count-18;rx_source_held1<=source;end
                        else begin rx_length_held0<=byte_count-18;rx_source_held0<=source;end
                        rx_request[write_bank]<=~rx_request[write_bank];next_write_bank<=~write_bank;
                        good_frames<=good_frames+1'b1;
                    end
                end
                IGNORE: if(!gmii_rx_dv) state<=SEARCH;
            endcase
        end
    end
    assign m_rx_valid=app_rst_n && app_state==APP_SEND;
    assign m_rx_last=(read_index+1==m_rx_length);
    always @(posedge app_clk or negedge app_rst_n) begin
        if(!app_rst_n) begin
            rx_request_s1<=0;rx_request_s2<=0;rx_ack<=0;app_state<=APP_IDLE;
            read_bank<=0;next_read_bank<=0;read_index<=0;
            m_rx_data<=0;m_rx_length<=0;m_rx_src_mac<=0;delivered_frames<=0;
        end else begin
            rx_request_s1<=rx_request;rx_request_s2<=rx_request_s1;
            case(app_state)
                APP_IDLE: if(rx_request_s2[next_read_bank]!=rx_ack[next_read_bank]) begin
                    read_bank<=next_read_bank;read_index<=0;
                    m_rx_length<=next_read_bank?rx_length_held1:rx_length_held0;
                    m_rx_src_mac<=next_read_bank?rx_source_held1:rx_source_held0;
                    app_state<=APP_LOAD;
                end
                APP_LOAD: begin m_rx_data<=read_bank?q1:q0;app_state<=APP_SEND;end
                APP_SEND: if(m_rx_ready) begin
                    if(m_rx_last) begin
                        rx_ack[read_bank]<=rx_request_s2[read_bank];next_read_bank<=~read_bank;
                        read_index<=0;app_state<=APP_IDLE;delivered_frames<=delivered_frames+1'b1;
                    end else begin read_index<=read_index+1'b1;app_state<=APP_LOAD;end
                end
                default: app_state<=APP_IDLE;
            endcase
        end
    end
endmodule
