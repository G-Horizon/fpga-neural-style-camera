module validated_ppi_raw4_bridge #(
    parameter integer W=1920,H=1080,BANKS=3,FIFO_BITS=4,
    parameter integer IDLE_GUARD=4,MAX_PREFIX=2,MAX_HEAD_WAIT=8,
    parameter integer FLUSH_IDLE=W/4+4
)(
    input wire byte_clk,byte_rst_n,
    input wire [31:0] ppi_data,
    input wire [3:0] ppi_valid,ppi_sync,ppi_stop,ppi_sot,
    input wire pixel_clk,pixel_rst_n,
    output reg raw_valid,
    output reg [39:0] raw4,
    output reg raw_sof,raw_sol,raw_eol,raw_eof,
    output reg [31:0] raw_frame_id,
    output reg source_bad_sticky,
    output reg [7:0] fatal_code,fatal_code_byte,
    output reg [31:0] emitted_rows,emitted_frames,
    output reg [31:0] accepted_rows,accepted_frames,startup_discard_packets,
    output reg [31:0] buffer_overflows,ecc_errors,crc_errors,geometry_errors,
    output reg [31:0] boundary_errors,short_packets,missing_sync_transactions,sequence_discontinuities,
    output reg [127:0] sot_events,sync_events
);
    localparam integer WORDS=W/4,WC=W*5/4,DEPTH=1<<FIFO_BITS;
    localparam integer BI=$clog2(BANKS);
    localparam [2:0] WAIT_IDLE=0,WAIT_HEAD=1,BODY=2,TAIL=3,REJECT=4;
    localparam [1:0] P_IDLE=0,P_READ=1,P_DRAIN=2,P_FLUSH=3;
    (* ram_style="block" *) reg [39:0] line_mem[0:BANKS*512-1];
    reg [60:0] bank_desc_hold[0:BANKS-1];
    reg [BANKS-1:0] bank_req_toggle,bank_ack_toggle,bank_busy,bank_published;
    (* async_reg="true" *) reg [BANKS-1:0] ack_meta,ack_sync,req_meta,req_sync;
    reg [BI-1:0] write_bank,packet_bank,pending_last_bank,read_bank;
    reg packet_has_bank,pending_last_valid;
    reg [7:0] lane_fifo[0:3][0:DEPTH-1];
    reg [FIFO_BITS-1:0] lane_wr[0:3],lane_rd[0:3];
    reg [FIFO_BITS:0] lane_used[0:3];
    reg [2:0] state;
    reg [15:0] quiet,head_age,prefix_count,body_offset,crc,received_crc,packet_number;
    reg [5:0] packet_dt;
    reg [7:0] packet_error;
    reg [3:0] seen_sync,old_sync,old_sot;
    reg activity_seen,boundary_good,ever_fs,frame_open;
    reg [15:0] sensor_number,previous_sensor_number;
    reg [31:0] local_fid;
    reg [10:0] rows_in_frame;
    reg [63:0] reservoir;
    reg [3:0] reservoir_bytes;
    reg [9:0] packet_words;
    integer lane,j,need,consume;
    reg can_consume;
    reg [3:0] sync_rise;
    reg [15:0] next_crc,next_received;
    reg [63:0] next_reservoir;
    integer next_reservoir_bytes;
    reg [7:0] data_byte;
    reg [39:0] unpacked;
    reg prefix_bad;

    function automatic [BI-1:0] following_bank(input [BI-1:0] bank);
        following_bank=(bank==BANKS-1) ? 0 : bank+1'b1;
    endfunction
    function automatic [7:0] ecc24(input [23:0] h);
        ecc24={2'b0,^(h&24'heffc00),^(h&24'hdf03f0),^(h&24'hb8e38e),
                  ^(h&24'h749a6d),^(h&24'hf2555b),^(h&24'hf12cb7)};
    endfunction
    function automatic [15:0] crc_byte(input [15:0] c,input [7:0] d);
        reg [15:0] v;integer k;
        begin v=c^{8'b0,d};for(k=0;k<8;k=k+1)v=v[0]?(v>>1)^16'h8408:v>>1;crc_byte=v;end
    endfunction
    task automatic count_error(input [7:0] code);
        begin
            if(fatal_code_byte==0)fatal_code_byte<=code;
            case(code)
                8'h10:boundary_errors<=boundary_errors+1;
                8'h11:ecc_errors<=ecc_errors+1;
                8'h12:geometry_errors<=geometry_errors+1;
                8'h13:crc_errors<=crc_errors+1;
                8'h14:short_packets<=short_packets+1;
                8'h15:buffer_overflows<=buffer_overflows+1;
                default:geometry_errors<=geometry_errors+1;
            endcase
        end
    endtask
    always @* begin
        sync_rise=ppi_sync&~old_sync;
        need=(WC+2-body_offset>=4)?4:WC+2-body_offset;
        can_consume=state==BODY;
        for(integer n=0;n<4;n=n+1)if(n<need && lane_used[n]==0)can_consume=0;
    end

    always @(posedge byte_clk or negedge byte_rst_n) begin
        if(!byte_rst_n) begin
            ack_meta<=0;ack_sync<=0;bank_req_toggle<=0;bank_busy<=0;bank_published<=0;
            write_bank<=0;packet_bank<=0;pending_last_bank<=0;packet_has_bank<=0;pending_last_valid<=0;
            state<=WAIT_IDLE;quiet<=0;head_age<=0;prefix_count<=0;body_offset<=0;
            crc<=16'hffff;received_crc<=0;packet_number<=0;packet_dt<=0;packet_error<=0;
            seen_sync<=0;old_sync<=0;old_sot<=0;activity_seen<=0;boundary_good<=1;
            ever_fs<=0;frame_open<=0;sensor_number<=0;previous_sensor_number<=0;local_fid<=0;rows_in_frame<=0;
            reservoir<=0;reservoir_bytes<=0;packet_words<=0;
            fatal_code_byte<=0;accepted_rows<=0;accepted_frames<=0;startup_discard_packets<=0;
            buffer_overflows<=0;ecc_errors<=0;crc_errors<=0;geometry_errors<=0;boundary_errors<=0;
            short_packets<=0;missing_sync_transactions<=0;sequence_discontinuities<=0;sot_events<=0;sync_events<=0;
            for(lane=0;lane<4;lane=lane+1)begin lane_wr[lane]<=0;lane_rd[lane]<=0;lane_used[lane]<=0;end
            for(lane=0;lane<BANKS;lane=lane+1)bank_desc_hold[lane]<=0;
        end else begin
            ack_meta<=bank_ack_toggle;ack_sync<=ack_meta;
            old_sync<=ppi_sync;old_sot<=ppi_sot;
            for(lane=0;lane<4;lane=lane+1)begin
                if(ppi_sot[lane]&&!old_sot[lane])sot_events[lane*32+:32]<=sot_events[lane*32+:32]+1;
                if(sync_rise[lane])sync_events[lane*32+:32]<=sync_events[lane*32+:32]+1;
            end
            for(lane=0;lane<BANKS;lane=lane+1)
                if(bank_busy[lane]&&bank_published[lane]&&ack_sync[lane]==bank_req_toggle[lane])begin
                    bank_busy[lane]<=0;bank_published[lane]<=0;
                end
            if(ppi_valid==0&&ppi_sync==0&&ppi_stop==15)begin
                if(quiet<16'hffff)quiet<=quiet+1;
            end else quiet<=0;
            if(fatal_code_byte==0) begin
                case(state)
                WAIT_IDLE:begin
                    if(|ppi_valid || |sync_rise)activity_seen<=1;
                    if(quiet>=IDLE_GUARD-1&&ppi_valid==0&&ppi_sync==0&&ppi_stop==15)begin
                        state<=WAIT_HEAD;seen_sync<=0;prefix_count<=0;head_age<=0;boundary_good<=1;activity_seen<=0;
                        if(activity_seen)startup_discard_packets<=startup_discard_packets+1;
                    end
                end
                WAIT_HEAD:begin
                    seen_sync<=seen_sync|sync_rise;
                    if(|seen_sync&&head_age<16'hffff)head_age<=head_age+1;
                    if(|ppi_valid|| |sync_rise)activity_seen<=1;
                    if(ppi_valid==15)begin
                        packet_dt<=ppi_data[5:0];packet_number<=ppi_data[23:8];packet_error<=0;
                        packet_has_bank<=0;state<=TAIL;
                        for(lane=0;lane<4;lane=lane+1)begin lane_wr[lane]<=0;lane_rd[lane]<=0;lane_used[lane]<=0;end
                        if(!( |(seen_sync|sync_rise)) || !boundary_good || head_age>=MAX_HEAD_WAIT)begin
                            packet_error<=8'h10;state<=REJECT;
                        end else if(ppi_data[31:24]!=ecc24(ppi_data[23:0]))begin
                            packet_error<=8'h11;state<=REJECT;
                        end else if(ppi_data[7:6]!=0 || !(ppi_data[5:0]==0||ppi_data[5:0]==1||
                            (ppi_data[5:0]==6'h2b&&ppi_data[23:8]==WC)))begin
                            packet_error<=8'h12;state<=REJECT;
                        end else if(ppi_data[5:0]==6'h2b)begin
                            state<=BODY;body_offset<=0;crc<=16'hffff;received_crc<=0;
                            reservoir<=0;reservoir_bytes<=0;packet_words<=0;
                            if(ever_fs)begin
                                if(!frame_open||rows_in_frame>=H)begin packet_error<=8'h12;state<=REJECT;end
                                else if(bank_busy[write_bank])begin packet_error<=8'h15;state<=REJECT;end
                                else begin
                                    packet_has_bank<=1;packet_bank<=write_bank;bank_busy[write_bank]<=1;bank_published[write_bank]<=0;
                                    bank_desc_hold[write_bank]<={1'b0,(rows_in_frame==H-1),rows_in_frame,sensor_number,local_fid};
                                    write_bank<=following_bank(write_bank);
                                end
                            end
                        end
                    end else if(|ppi_valid)begin
                        prefix_count<=prefix_count+1;prefix_bad=prefix_count>=MAX_PREFIX;
                        for(lane=0;lane<4;lane=lane+1)
                            if(ppi_valid[lane]&&ppi_data[lane*8+:8]!=8'hb8)prefix_bad=1;
                        if(prefix_bad)boundary_good<=0;
                    end
                end
                BODY:begin
                    seen_sync<=seen_sync|sync_rise;
                    if(|sync_rise && packet_error==0)packet_error<=8'h10;
                    for(lane=0;lane<4;lane=lane+1)begin
                        consume=can_consume&&lane<need;
                        if(ppi_valid[lane])begin
                            if(lane_used[lane]<DEPTH||consume)begin
                                lane_fifo[lane][lane_wr[lane]]<=ppi_data[lane*8+:8];lane_wr[lane]<=lane_wr[lane]+1;
                            end else if(packet_error==0)packet_error<=8'h15;
                        end
                        if(consume)lane_rd[lane]<=lane_rd[lane]+1;
                        if((lane_used[lane]<DEPTH||consume)&&ppi_valid[lane])lane_used[lane]<=lane_used[lane]+1-consume;
                        else if(consume)lane_used[lane]<=lane_used[lane]-1;
                    end
                    if(can_consume)begin
                        next_crc=crc;next_received=received_crc;
                        next_reservoir=reservoir;next_reservoir_bytes=reservoir_bytes;
                        for(j=0;j<4;j=j+1)if(j<need)begin
                            data_byte=lane_fifo[j][lane_rd[j]];
                            if(body_offset+j<WC)begin
                                next_crc=crc_byte(next_crc,data_byte);
                                next_reservoir[next_reservoir_bytes*8+:8]=data_byte;next_reservoir_bytes=next_reservoir_bytes+1;
                            end else if(body_offset+j==WC)next_received[7:0]=data_byte;
                            else next_received[15:8]=data_byte;
                        end
                        if(next_reservoir_bytes>=5)begin
                            unpacked={next_reservoir[31:24],next_reservoir[39:38],
                                      next_reservoir[23:16],next_reservoir[37:36],
                                      next_reservoir[15:8],next_reservoir[35:34],
                                      next_reservoir[7:0],next_reservoir[33:32]};
                            if(packet_has_bank && packet_words<WORDS)line_mem[packet_bank*512+packet_words]<=unpacked;
                            packet_words<=packet_words+1;
                            next_reservoir=next_reservoir>>40;next_reservoir_bytes=next_reservoir_bytes-5;
                        end
                        reservoir<=next_reservoir;reservoir_bytes<=next_reservoir_bytes;
                        crc<=next_crc;received_crc<=next_received;body_offset<=body_offset+need;
                        if(body_offset+need==WC+2)begin
                            state<=TAIL;
                            if(next_crc!=next_received && packet_error==0)packet_error<=8'h13;
                        end
                    end
                end
                TAIL:begin seen_sync<=seen_sync|sync_rise;if(|sync_rise && packet_error==0)packet_error<=8'h10;end
                REJECT:seen_sync<=seen_sync|sync_rise;
                default:state<=WAIT_IDLE;
                endcase

                if(state==WAIT_HEAD&&activity_seen&&quiet>=IDLE_GUARD-1&&ppi_valid==0&&ppi_sync==0&&ppi_stop==15)begin
                    if(ever_fs)count_error(8'h10);else startup_discard_packets<=startup_discard_packets+1;
                    if(seen_sync!=15)missing_sync_transactions<=missing_sync_transactions+1;
                    seen_sync<=0;prefix_count<=0;head_age<=0;boundary_good<=1;activity_seen<=0;
                end
                if(state!=WAIT_IDLE&&state!=WAIT_HEAD&&quiet>=IDLE_GUARD-1&&ppi_valid==0&&ppi_sync==0&&ppi_stop==15)begin
                    if(seen_sync!=15)missing_sync_transactions<=missing_sync_transactions+1;
                    if(state==BODY)begin
                        if(ever_fs)count_error(packet_error!=0 ? packet_error : 8'h14);
                        else startup_discard_packets<=startup_discard_packets+1;
                    end else if(packet_error!=0||state==REJECT)begin
                        if(ever_fs)count_error(packet_error);else startup_discard_packets<=startup_discard_packets+1;
                    end else if(packet_dt==0)begin
                        if(frame_open)count_error(8'h12);
                        else begin
                            if(ever_fs && packet_number!=previous_sensor_number+16'd1)
                                sequence_discontinuities<=sequence_discontinuities+1;
                            previous_sensor_number<=packet_number;sensor_number<=packet_number;
                            if(ever_fs)local_fid<=local_fid+1;
                            ever_fs<=1;frame_open<=1;rows_in_frame<=0;pending_last_valid<=0;
                        end
                    end else if(!ever_fs)startup_discard_packets<=startup_discard_packets+1;
                    else if(packet_dt==1)begin
                        if(!frame_open||packet_number!=sensor_number||rows_in_frame!=H||!pending_last_valid)
                            count_error(8'h12);
                        else begin
                            bank_desc_hold[pending_last_bank][60]<=1;
                            bank_req_toggle[pending_last_bank]<=~bank_req_toggle[pending_last_bank];bank_published[pending_last_bank]<=1;
                            frame_open<=0;pending_last_valid<=0;accepted_frames<=accepted_frames+1;
                        end
                    end else if(packet_dt==6'h2b)begin
                        if(!packet_has_bank||!frame_open||packet_words!=WORDS||reservoir_bytes!=0)count_error(8'h16);
                        else begin
                            accepted_rows<=accepted_rows+1;rows_in_frame<=rows_in_frame+1;
                            if(rows_in_frame==H-1)begin pending_last_valid<=1;pending_last_bank<=packet_bank;end
                            else begin bank_req_toggle[packet_bank]<=~bank_req_toggle[packet_bank];bank_published[packet_bank]<=1;end
                        end
                    end
                    state<=WAIT_HEAD;seen_sync<=0;prefix_count<=0;head_age<=0;boundary_good<=1;activity_seen<=0;
                    for(lane=0;lane<4;lane=lane+1)begin lane_wr[lane]<=0;lane_rd[lane]<=0;lane_used[lane]<=0;end
                end
            end
        end
    end

    reg [1:0] pixel_state;
    reg [60:0] read_desc;
    reg [8:0] issue_addr;
    reg [39:0] ram_read_q;
    reg pipe_valid,pipe_sof,pipe_sol,pipe_eol,pipe_eof;
    reg [31:0] pipe_fid;
    reg [15:0] flush_remaining;
    reg output_frame_open;
    reg [31:0] output_fid;
    reg [10:0] expected_row;
    (* async_reg="true" *) reg byte_bad_meta,byte_bad_sync;
    wire read_enable=pixel_state==P_READ&&!source_bad_sticky&&!byte_bad_sync;
    always @(posedge pixel_clk)if(read_enable)ram_read_q<=line_mem[read_bank*512+issue_addr];
    always @(posedge pixel_clk or negedge pixel_rst_n)begin
        if(!pixel_rst_n)begin
            req_meta<=0;req_sync<=0;bank_ack_toggle<=0;read_bank<=0;read_desc<=0;
            pixel_state<=P_IDLE;issue_addr<=0;pipe_valid<=0;pipe_sof<=0;pipe_sol<=0;pipe_eol<=0;pipe_eof<=0;pipe_fid<=0;
            raw_valid<=0;raw4<=0;raw_sof<=0;raw_sol<=0;raw_eol<=0;raw_eof<=0;raw_frame_id<=0;
            source_bad_sticky<=0;fatal_code<=0;emitted_rows<=0;emitted_frames<=0;
            flush_remaining<=0;output_frame_open<=0;output_fid<=0;expected_row<=0;byte_bad_meta<=0;byte_bad_sync<=0;
        end else begin
            req_meta<=bank_req_toggle;req_sync<=req_meta;byte_bad_meta<=|fatal_code_byte;byte_bad_sync<=byte_bad_meta;
            pipe_valid<=read_enable;
            if(read_enable)begin
                pipe_sof<=read_desc[58:48]==0&&issue_addr==0;pipe_sol<=issue_addr==0;
                pipe_eol<=issue_addr==WORDS-1;pipe_eof<=read_desc[59]&&issue_addr==WORDS-1;pipe_fid<=read_desc[31:0];
            end
            raw_valid<=pipe_valid&&!source_bad_sticky&&!byte_bad_sync;
            raw_sof<=pipe_valid&&pipe_sof&&!source_bad_sticky&&!byte_bad_sync;
            raw_sol<=pipe_valid&&pipe_sol&&!source_bad_sticky&&!byte_bad_sync;
            raw_eol<=pipe_valid&&pipe_eol&&!source_bad_sticky&&!byte_bad_sync;
            raw_eof<=pipe_valid&&pipe_eof&&!source_bad_sticky&&!byte_bad_sync;
            if(pipe_valid)begin raw4<=ram_read_q;raw_frame_id<=pipe_fid;end
            if(raw_valid&&raw_eol&&!source_bad_sticky)begin
                emitted_rows<=emitted_rows+1;
                if(raw_eof)emitted_frames<=emitted_frames+1;
            end
            if(byte_bad_sync)begin source_bad_sticky<=1;if(fatal_code==0)fatal_code<=fatal_code_byte;end
            if(!source_bad_sticky&&!byte_bad_sync)begin
                case(pixel_state)
                P_IDLE:if(req_sync[read_bank]!=bank_ack_toggle[read_bank])begin
                    read_desc<=bank_desc_hold[read_bank];issue_addr<=0;
                    if(bank_desc_hold[read_bank][58:48]!=expected_row ||
                        (bank_desc_hold[read_bank][59]!=(expected_row==H-1)) ||
                        (expected_row==H-1&&!bank_desc_hold[read_bank][60]) ||
                        (expected_row==0&&output_frame_open) ||
                        (expected_row!=0&&(!output_frame_open||bank_desc_hold[read_bank][31:0]!=output_fid)))begin
                        source_bad_sticky<=1;fatal_code<=8'h80;
                    end else begin
                        if(expected_row==0)begin output_frame_open<=1;output_fid<=bank_desc_hold[read_bank][31:0];end
                        pixel_state<=P_READ;
                    end
                end
                P_READ:begin
                    if(issue_addr==WORDS-1)pixel_state<=P_DRAIN;
                    else issue_addr<=issue_addr+1;
                end
                P_DRAIN:if(raw_valid&&raw_eol)begin
                    // This edge is the ISP's actual consumption edge, after
                    // the prior edge's output-register NBA. Only now release.
                    bank_ack_toggle[read_bank]<=req_sync[read_bank];read_bank<=following_bank(read_bank);
                    if(raw_eof)begin expected_row<=0;output_frame_open<=0;flush_remaining<=FLUSH_IDLE;pixel_state<=P_FLUSH;end
                    else begin expected_row<=expected_row+1;pixel_state<=P_IDLE;end
                end
                P_FLUSH:begin
                    if(flush_remaining==1)begin flush_remaining<=0;pixel_state<=P_IDLE;end
                    else flush_remaining<=flush_remaining-1;
                end
                default:pixel_state<=P_IDLE;
                endcase
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(W<8||W>2048||W%4||H<2||H>2047||BANKS<2||BANKS>4||FIFO_BITS<2||FLUSH_IDLE<WORDS+4)
            $fatal(1,"VALIDATED_PPI_BRIDGE_GEOMETRY");
    end
    // synthesis translate_on
endmodule
