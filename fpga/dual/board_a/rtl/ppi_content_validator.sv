// Independent PPI CONTENT diagnostic. Never drives or repairs Sync/FS/pixels.
// Contract and qualification limits are documented in README.md.
module ppi_content_validator #(
    parameter integer EXPECT_LINES = 1080,
    parameter integer EXPECT_WC = 2400,
    parameter integer IDLE_GUARD = 4,
    parameter integer MAX_PREFIX = 2,
    parameter integer MAX_HEAD_WAIT = 8,
    parameter integer REQUIRE_INCREMENT = 0,
    parameter integer FIFO_BITS = 4
)(
    input wire clk, reset_n,
    input wire [31:0] rx_data,
    input wire [3:0] rx_valid, rx_sync, rx_stop, rx_sot,
    input wire observation_end,
    output reg [31:0] content_packets, content_raw_packets, content_fs, content_fe,
    output reg [31:0] content_frames,
    output reg [31:0] ecc_errors, crc_errors, geometry_errors,
    output reg [31:0] boundary_errors, short_packets, fifo_overflows,
    output reg [31:0] incomplete_observations, prefix_valid_bytes, trailing_valid_bytes,
    output reg [31:0] missing_sync_transactions, extra_sync_transactions,
    output reg [31:0] unframed_packets,
    output reg [31:0] sequence_discontinuities,
    output reg [127:0] sot_events, sync_events,
    output reg frame_open,
    output reg [15:0] active_frame_number,
    output reg [15:0] active_line_count
);
    localparam integer DEPTH = 1 << FIFO_BITS;
    localparam [2:0] WAIT_IDLE=0, WAIT_HEAD=1, BODY=2, TAIL=3, REJECT=4;
    reg [2:0] state;
    reg [7:0] fifo [0:3][0:DEPTH-1];
    reg [FIFO_BITS-1:0] wr [0:3], rd [0:3];
    reg [FIFO_BITS:0] used [0:3];
    reg [3:0] old_sot, old_sync, seen_sync;
    reg [15:0] quiet, prefix_count, head_age, body_offset, crc, received_crc;
    reg packet_good, boundary_good, frame_good;
    reg [5:0] packet_dt;
    reg [15:0] packet_number, previous_fs;
    reg [31:0] packet_valid_total;
    reg have_previous_fs, activity_seen;
    integer lane, j, need, consume, valid_bytes, prefix_bytes, tail_bytes;
    reg can_consume;
    reg [7:0] b;
    reg [15:0] next_crc, next_received;
    reg [3:0] rising_sync;
    reg new_boundary_bad;

    function automatic [7:0] ecc24(input [23:0] h);
        begin
            ecc24={2'b00, ^(h & 24'heffc00), ^(h & 24'hdf03f0),
                ^(h & 24'hb8e38e), ^(h & 24'h749a6d),
                ^(h & 24'hf2555b), ^(h & 24'hf12cb7)};
        end
    endfunction
    function automatic [15:0] crc_byte(input [15:0] c, input [7:0] d);
        reg [15:0] v; integer k;
        begin
            v=c ^ {8'b0,d};
            for(k=0;k<8;k=k+1) v=v[0] ? (v>>1)^16'h8408 : v>>1;
            crc_byte=v;
        end
    endfunction

    always @* begin
        valid_bytes=0;
        for(integer n=0;n<4;n=n+1) valid_bytes=valid_bytes+rx_valid[n];
        rising_sync=rx_sync & ~old_sync;
        need=(EXPECT_WC+2-body_offset >= 4) ? 4 : EXPECT_WC+2-body_offset;
        can_consume=(state==BODY);
        for(integer n=0;n<4;n=n+1)
            if(n<need && used[n]==0) can_consume=0;
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=WAIT_IDLE; quiet<=0; prefix_count<=0; head_age<=0; seen_sync<=0;
            old_sot<=0; old_sync<=0; body_offset<=0; crc<=16'hffff; received_crc<=0;
            packet_good<=0; boundary_good<=0; frame_good<=0; packet_dt<=0; packet_number<=0;
            frame_open<=0; active_frame_number<=0; active_line_count<=0;
            previous_fs<=0; have_previous_fs<=0; activity_seen<=0;
            packet_valid_total<=0;
            content_packets<=0;content_raw_packets<=0;content_fs<=0;content_fe<=0;content_frames<=0;
            ecc_errors<=0;crc_errors<=0;geometry_errors<=0;boundary_errors<=0;short_packets<=0;
            fifo_overflows<=0;incomplete_observations<=0;prefix_valid_bytes<=0;trailing_valid_bytes<=0;
            missing_sync_transactions<=0;extra_sync_transactions<=0;sot_events<=0;sync_events<=0;
            unframed_packets<=0;
            sequence_discontinuities<=0;
            for(lane=0;lane<4;lane=lane+1) begin wr[lane]<=0;rd[lane]<=0;used[lane]<=0;end
        end else begin
            old_sot<=rx_sot; old_sync<=rx_sync;
            for(lane=0;lane<4;lane=lane+1) begin
                if(rx_sot[lane]&&!old_sot[lane]) sot_events[32*lane+:32]<=sot_events[32*lane+:32]+1;
                if(rising_sync[lane]) sync_events[32*lane+:32]<=sync_events[32*lane+:32]+1;
            end
            if(rx_valid==0 && rx_sync==0 && rx_stop==15) begin
                if(quiet<16'hffff) quiet<=quiet+1;
            end else quiet<=0;

            case(state)
            WAIT_IDLE: begin
                if(|rx_valid || |rising_sync) activity_seen<=1;
                if(quiet>=IDLE_GUARD-1 && rx_valid==0 && rx_sync==0 && rx_stop==15) begin
                    state<=WAIT_HEAD;seen_sync<=rising_sync;prefix_count<=0;head_age<=0;boundary_good<=1;activity_seen<=0;
                    if(activity_seen) incomplete_observations<=incomplete_observations+1;
                end
            end
            WAIT_HEAD: begin
                seen_sync<=seen_sync | rising_sync;
                if(|seen_sync && head_age<16'hffff) head_age<=head_age+1;
                if(|rx_valid || |rising_sync) activity_seen<=1;
                if(rx_valid==15) begin
                    packet_dt<=rx_data[5:0];packet_number<=rx_data[23:8];
                    packet_valid_total<=4;
                    packet_good<=1; state<=TAIL;
                    for(lane=0;lane<4;lane=lane+1) begin wr[lane]<=0;rd[lane]<=0;used[lane]<=0;end
                    if(!( |(seen_sync|rising_sync)) || !boundary_good || head_age>=MAX_HEAD_WAIT) begin
                        boundary_errors<=boundary_errors+1;packet_good<=0;state<=REJECT;
                    end else if(rx_data[31:24]!=ecc24(rx_data[23:0])) begin
                        ecc_errors<=ecc_errors+1;packet_good<=0;state<=REJECT;
                    end else if(rx_data[7:6]!=0 ||
                        !(rx_data[5:0]==0 || rx_data[5:0]==1 ||
                          (rx_data[5:0]==6'h2b && rx_data[23:8]==EXPECT_WC))) begin
                        geometry_errors<=geometry_errors+1;packet_good<=0;state<=REJECT;
                    end else if(rx_data[5:0]==6'h2b) begin
                        state<=BODY;body_offset<=0;crc<=16'hffff;received_crc<=0;
                    end
                end else if(|rx_valid) begin
                    prefix_valid_bytes<=prefix_valid_bytes+valid_bytes;
                    prefix_count<=prefix_count+1;
                    new_boundary_bad=(prefix_count>=MAX_PREFIX);
                    for(lane=0;lane<4;lane=lane+1)
                        if(rx_valid[lane] && rx_data[8*lane+:8]!=8'hb8) new_boundary_bad=1;
                    if(new_boundary_bad) boundary_good<=0;
                end
            end
            BODY: begin
                seen_sync<=seen_sync|rising_sync;
                packet_valid_total<=packet_valid_total+valid_bytes;
                if(|rising_sync) begin
                    extra_sync_transactions<=extra_sync_transactions+1;boundary_good<=0;
                    if(boundary_good) boundary_errors<=boundary_errors+1;
                end
                for(lane=0;lane<4;lane=lane+1) begin
                    consume=can_consume && lane<need;
                    if(rx_valid[lane]) begin
                        if(used[lane]<DEPTH || consume) begin
                            fifo[lane][wr[lane]]<=rx_data[8*lane+:8];wr[lane]<=wr[lane]+1;
                        end else begin
                            fifo_overflows<=fifo_overflows+1;packet_good<=0;boundary_good<=0;
                        end
                    end
                    if(consume) rd[lane]<=rd[lane]+1;
                    if((used[lane]<DEPTH || consume) && rx_valid[lane])
                        used[lane]<=used[lane]+1-consume;
                    else if(consume) used[lane]<=used[lane]-1;
                end
                if(can_consume) begin
                    next_crc=crc;next_received=received_crc;
                    for(j=0;j<4;j=j+1) if(j<need) begin
                        b=fifo[j][rd[j]];
                        if(body_offset+j<EXPECT_WC) next_crc=crc_byte(next_crc,b);
                        else if(body_offset+j==EXPECT_WC) next_received[7:0]=b;
                        else next_received[15:8]=b;
                    end
                    crc<=next_crc;received_crc<=next_received;body_offset<=body_offset+need;
                    if(body_offset+need==EXPECT_WC+2) begin
                        state<=TAIL;
                        if(next_crc!=next_received) begin crc_errors<=crc_errors+1;packet_good<=0;end
                    end
                end
            end
            TAIL: begin
                seen_sync<=seen_sync|rising_sync;
                packet_valid_total<=packet_valid_total+valid_bytes;
                if(|rising_sync) begin
                    extra_sync_transactions<=extra_sync_transactions+1;boundary_good<=0;
                    if(boundary_good) boundary_errors<=boundary_errors+1;
                end
            end
            REJECT: begin
                seen_sync<=seen_sync|rising_sync;
            end
            default: state<=WAIT_IDLE;
            endcase

            if(state==WAIT_HEAD && activity_seen && quiet>=IDLE_GUARD-1 &&
                rx_valid==0 && rx_sync==0 && rx_stop==15) begin
                boundary_errors<=boundary_errors+1;frame_good<=0;
                if(seen_sync!=15) missing_sync_transactions<=missing_sync_transactions+1;
                seen_sync<=0;prefix_count<=0;head_age<=0;boundary_good<=1;activity_seen<=0;
            end

            // A real idle interval closes a packet only after PPI data has drained.
            // StopState alone never ends payload. The queue depth bounds valid skew.
            if(state!=WAIT_IDLE && state!=WAIT_HEAD && quiet>=IDLE_GUARD-1 &&
                rx_valid==0 && rx_sync==0 && rx_stop==15) begin
                if(seen_sync!=15) missing_sync_transactions<=missing_sync_transactions+1;
                if(state==BODY) begin short_packets<=short_packets+1;frame_good<=0;end
                else if(state==REJECT || !packet_good || !boundary_good) frame_good<=0;
                else begin
                    content_packets<=content_packets+1;
                    if(packet_dt==6'h2b)
                        trailing_valid_bytes<=trailing_valid_bytes+packet_valid_total-(EXPECT_WC+6);
                    else trailing_valid_bytes<=trailing_valid_bytes+packet_valid_total-4;
                    if(packet_dt==0) begin
                        content_fs<=content_fs+1;
                        frame_good<=1;
                        if(frame_open) geometry_errors<=geometry_errors+1;
                        if(have_previous_fs && packet_number!=previous_fs+16'd1) begin
                            sequence_discontinuities<=sequence_discontinuities+1;
                            if(REQUIRE_INCREMENT) frame_good<=0;
                        end
                        frame_open<=1;active_frame_number<=packet_number;active_line_count<=0;
                        previous_fs<=packet_number;have_previous_fs<=1;
                    end else if(packet_dt==1) begin
                        content_fe<=content_fe+1;
                        if(frame_open && frame_good && active_line_count==EXPECT_LINES &&
                           packet_number==active_frame_number) content_frames<=content_frames+1;
                        else if(!have_previous_fs) unframed_packets<=unframed_packets+1;
                        else geometry_errors<=geometry_errors+1;
                        frame_open<=0;frame_good<=0;
                    end else begin
                        content_raw_packets<=content_raw_packets+1;
                        if(!have_previous_fs) unframed_packets<=unframed_packets+1;
                        else if(!frame_open || active_line_count>=EXPECT_LINES) begin
                            geometry_errors<=geometry_errors+1;frame_good<=0;
                        end else active_line_count<=active_line_count+1;
                    end
                end
                state<=WAIT_HEAD;seen_sync<=0;prefix_count<=0;head_age<=0;boundary_good<=1;activity_seen<=0;
                for(lane=0;lane<4;lane=lane+1) begin wr[lane]<=0;rd[lane]<=0;used[lane]<=0;end
            end
            if(observation_end) begin
                if((state!=WAIT_IDLE && state!=WAIT_HEAD) || activity_seen || frame_open)
                    incomplete_observations<=incomplete_observations+1;
                state<=WAIT_IDLE;quiet<=0;frame_open<=0;frame_good<=0;activity_seen<=0;
                for(lane=0;lane<4;lane=lane+1) begin wr[lane]<=0;rd[lane]<=0;used[lane]<=0;end
            end
        end
    end
endmodule
