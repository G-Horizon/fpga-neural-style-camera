`timescale 1ns/1ps
// FPS2 request service. RX/TX are the mailbox's synchronous byte windows.
// One leased independent DDR snapshot; ordered RGB888 reads never pin CNN.
module d64_preview_service #(
    parameter integer FRAME_BYTES=256*144*3,
    parameter integer AUDIO_ENABLE=1,REMOTE_DEBUG_ENABLE=0
)(
    input wire clk,rst_n,frame_tick,rx_done,
    output reg [8:0] rx_addr,input wire [7:0] rx_data,
    output wire tx_we,output wire [8:0] tx_addr,output wire [7:0] tx_data,
    output wire response_commit,
    input wire latest_valid,latest_slot,input wire [31:0] latest_base,latest_fid,
    input wire [15:0] latest_epoch,input wire [1:0] latest_style,
    input wire [7:0] latest_effect,processing_fps,
    input wire [15:0] remote_status,remote_active_epoch,input wire [1:0] remote_active_style,
    output reg remote_control_valid,output reg [2:0] remote_control_action,output reg [1:0] remote_control_value,
    output reg lease_active,lease_slot,
    output wire pcm_valid,output reg signed [15:0] pcm,input wire pcm_ready,
    output reg pcm_abort,style_valid,style_next,output reg [1:0] style_id,
    output wire arvalid,input wire arready,output wire [31:0] araddr,
    output wire [7:0] arlen,output wire [2:0] arsize,output wire [1:0] arburst,
    input wire rvalid,output wire rready,input wire [127:0] rdata,
    input wire rlast,input wire [1:0] rresp,
    output reg protocol_fault,output reg ddr_fault
);
    localparam IDLE=0,H_WAIT=1,H_READ=2,DISPATCH=3,
        C_WAIT=4,C_READ=5,C_CHECK=6,P_WAIT=7,P_READ=8,P_READY=9,
        AR=10,R=11,EMIT=12,DRAIN=13,HEADER=14,COMMIT=15;
    reg [3:0] state;
    reg [5:0] index;
    reg [31:0] seq,req_fid,req_offset,req_crc;
    reg magic_ok,version_ok;
    reg req_crc_nonzero;
    wire request_crc_bad=(AUDIO_ENABLE!=0)?(req_crc!=0):req_crc_nonzero;
    reg [7:0] opcode;
    reg [2:0] flags;reg [1:0] req_style;reg [8:0] req_length;
    reg flags_high_nonzero,style_high_nonzero,length_high_nonzero;
    localparam integer OFFSET_W=(FRAME_BYTES<=131071)?17:32;
    localparam integer RAW_OFFSET_W=(FRAME_BYTES<=131071)?18:32;
    reg [31:0] lease_base,lease_fid;
    reg [OFFSET_W-1:0] lease_offset;
    reg [RAW_OFFSET_W-1:0] lease_raw_offset;
    reg [15:0] lease_epoch;
    reg [1:0] lease_style;
    reg [7:0] lease_effect;
    reg [5:0] lease_age;
    reg [31:0] next_audio,crc;
    reg [8:0] payload_index,reply_index;
    reg [8:0] reply_length;
    reg [7:0] reply_status,reply_flags,reply_style,reply_effect,pcm_low;
    reg [31:0] reply_fid,reply_crc;
    reg [OFFSET_W-1:0] reply_offset;
    wire [31:0] reply_offset_full={{(32-OFFSET_W){1'b0}},reply_offset};
    wire [8:0] reply_pixels=reply_length/9'd3;
    wire [OFFSET_W-1:0] remaining_bytes=FRAME_BYTES-lease_offset;
    reg [15:0] reply_epoch;
    reg [127:0] held;
    reg [3:0] beat_byte;
    reg [31:0] raw_address;
    reg first_read;
    reg [7:0] rgb_byte;
    always @* begin
        case(beat_byte)
            0:rgb_byte=held[7:0];1:rgb_byte=held[15:8];2:rgb_byte=held[23:16];
            3:rgb_byte=held[39:32];4:rgb_byte=held[47:40];5:rgb_byte=held[55:48];
            6:rgb_byte=held[71:64];7:rgb_byte=held[79:72];8:rgb_byte=held[87:80];
            9:rgb_byte=held[103:96];10:rgb_byte=held[111:104];11:rgb_byte=held[119:112];
            default:rgb_byte=8'hxx;
        endcase
    end
    function [31:0] crc_byte;
        input [31:0] value;input [7:0] b;reg [31:0] v;integer bitno;
        begin v=value^b;for(bitno=0;bitno<8;bitno=bitno+1)
            v=v[0]?(v>>1)^32'hedb88320:(v>>1);crc_byte=v;end
    endfunction
    function [7:0] header_byte;
        input [5:0] n;
        begin case(n)
            0:header_byte=8'h46;1:header_byte=8'h50;2:header_byte=8'h53;3:header_byte=8'h32;
            4:header_byte=1;5:header_byte=opcode;6:header_byte=reply_flags;7:header_byte=reply_status;
            8,9,10,11:header_byte=seq[n[1:0]*8+:8];
            12,13,14,15:header_byte=reply_fid[n[1:0]*8+:8];
            16,17,18,19:header_byte=reply_offset_full[n[1:0]*8+:8];
            20:header_byte=reply_length[7:0];21:header_byte={7'd0,reply_length[8]};
            22:header_byte=reply_style;23:header_byte=processing_fps;
            24:header_byte=reply_epoch[7:0];25:header_byte=reply_epoch[15:8];
            26:header_byte=reply_effect;27:header_byte=(REMOTE_DEBUG_ENABLE!=0)?8'hd1:0;
            28,29,30,31:header_byte=reply_crc[n[1:0]*8+:8];
            default:header_byte=0;
        endcase end
    endfunction
    assign tx_we=state==HEADER||state==EMIT;
    assign tx_addr=state==HEADER?{3'd0,index}:9'd32+reply_index;
    assign tx_data=state==HEADER?header_byte(index):rgb_byte;
    assign response_commit=state==COMMIT;
    assign pcm_valid=(AUDIO_ENABLE!=0)&&state==P_READY;
    assign arvalid=state==AR;assign araddr=raw_address;
    assign arlen=0;assign arsize=4;assign arburst=1;
    assign rready=state==R||state==DRAIN;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;rx_addr<=0;index<=0;magic_ok<=0;seq<=0;req_fid<=0;req_offset<=0;
            req_crc<=0;req_crc_nonzero<=0;version_ok<=0;opcode<=0;flags<=0;req_style<=0;req_length<=0;
            flags_high_nonzero<=0;style_high_nonzero<=0;length_high_nonzero<=0;
            lease_active<=0;lease_slot<=0;lease_base<=0;lease_fid<=0;lease_epoch<=0;
            lease_style<=1;lease_effect<=0;lease_offset<=0;lease_raw_offset<=0;lease_age<=0;
            next_audio<=0;crc<=32'hffffffff;payload_index<=0;reply_index<=0;
            reply_length<=0;reply_status<=0;reply_flags<=0;reply_style<=1;reply_effect<=0;
            reply_fid<=0;reply_offset<=0;reply_crc<=0;reply_epoch<=0;pcm<=0;pcm_low<=0;
            pcm_abort<=0;style_valid<=0;style_next<=0;style_id<=1;held<=0;beat_byte<=0;
            raw_address<=0;first_read<=0;protocol_fault<=0;ddr_fault<=0;
            remote_control_valid<=0;remote_control_action<=0;remote_control_value<=0;
        end else begin
            pcm_abort<=0;style_valid<=0;style_next<=0;remote_control_valid<=0;
            if(frame_tick && lease_active && state==IDLE)begin
                if(lease_age==63)lease_active<=0;else lease_age<=lease_age+1'b1;
            end
            case(state)
                IDLE:if(rx_done)begin
                    rx_addr<=0;index<=0;magic_ok<=1;seq<=0;req_fid<=0;req_offset<=0;req_crc<=0;req_crc_nonzero<=0;
                    req_length<=0;length_high_nonzero<=0;state<=H_WAIT;
                end
                H_WAIT:state<=H_READ;
                H_READ:begin
                    case(index)
                        0:magic_ok<=magic_ok&&(rx_data==8'h46);
                        1:magic_ok<=magic_ok&&(rx_data==8'h50);
                        2:magic_ok<=magic_ok&&(rx_data==8'h53);
                        3:magic_ok<=magic_ok&&(rx_data==8'h32);
                        4:version_ok<=(rx_data==1);5:opcode<=rx_data;
                        6:begin flags<=rx_data[2:0];flags_high_nonzero<=|rx_data[7:3];end
                        8,9,10,11:seq[index[1:0]*8+:8]<=rx_data;
                        12,13,14,15:req_fid[index[1:0]*8+:8]<=rx_data;
                        16,17,18,19:req_offset[index[1:0]*8+:8]<=rx_data;
                        20:req_length[7:0]<=rx_data;
                        21:begin req_length[8]<=rx_data[0];length_high_nonzero<=|rx_data[7:1];end
                        22:begin req_style<=rx_data[1:0];style_high_nonzero<=|rx_data[7:2];end
                        28,29,30,31:begin
                            if(AUDIO_ENABLE!=0)req_crc[index[1:0]*8+:8]<=rx_data;
                            else req_crc_nonzero<=req_crc_nonzero|(rx_data!=0);
                        end
                        default:begin end
                    endcase
                    if(index==31)state<=DISPATCH;
                    else begin index<=index+1'b1;rx_addr<=rx_addr+1'b1;state<=H_WAIT;end
                end
                DISPATCH:begin
                    reply_status<=0;reply_length<=0;reply_crc<=0;index<=0;
                    reply_flags<=latest_valid?1:0;
                    reply_fid<=lease_active?lease_fid:latest_fid;
                    reply_style<=lease_active?lease_style:latest_style;
                    reply_epoch<=lease_active?lease_epoch:latest_epoch;
                    reply_effect<=lease_active?lease_effect:latest_effect;
                    reply_offset<=FRAME_BYTES;state<=HEADER;
                    if(!magic_ok||!version_ok||length_high_nonzero||req_length>480)begin
                        reply_status<=3;protocol_fault<=1;
                    end else if(opcode!=5 && request_crc_bad)begin
                        reply_status<=5;protocol_fault<=1;
                    end else case(opcode)
                        1:begin end
                        2:begin
                            if(lease_active)reply_status<=2;
                            else if(!latest_valid)reply_status<=1;
                            else begin
                                lease_active<=1;lease_slot<=latest_slot;lease_base<=latest_base;
                                lease_fid<=latest_fid;lease_epoch<=latest_epoch;lease_style<=latest_style;
                                lease_effect<=latest_effect;lease_offset<=0;lease_raw_offset<=0;lease_age<=0;
                            end
                        end
                        3:begin
                            if(!lease_active||req_fid!=lease_fid)reply_status<=4;
                            else if(req_offset!=lease_offset||req_length==0||req_length[8:0]%9'd3!=9'd0||
                                    req_offset>FRAME_BYTES||req_length>remaining_bytes)reply_status<=3;
                            else begin
                                lease_age<=0;reply_flags<=1;reply_offset<=req_offset;
                                reply_length<=req_length;reply_index<=0;crc<=32'hffffffff;
                                raw_address<=lease_base+(lease_raw_offset & 32'hfffffff0);first_read<=1;state<=AR;
                            end
                        end
                        4:begin
                            if(!lease_active||req_fid!=lease_fid)reply_status<=4;
                            else lease_active<=0;
                        end
                        5:begin
                            if(AUDIO_ENABLE==0||req_length==0||req_length[0])reply_status<=3;
                            else begin
                                crc<=32'hffffffff;payload_index<=0;rx_addr<=32;state<=C_WAIT;
                            end
                        end
                        6:begin
                            if(flags[0]||(!style_high_nonzero&&(req_style==1||req_style==2||req_style==3)))begin
                                style_valid<=1;style_next<=flags[0];style_id<=req_style[1:0];
                            end else reply_status<=3;
                        end
                        7:begin
                            if(REMOTE_DEBUG_ENABLE==0)begin reply_status<=3;protocol_fault<=1;end
                            else begin
                                // A control ACK is the pre-action live snapshot, never lease metadata.
                                // The action pulse cannot modify lease state or issue AXI requests.
                                reply_fid<=latest_fid;reply_epoch<=remote_active_epoch;
                                reply_style<={6'd0,remote_active_style};reply_effect<={7'd0,remote_status[11]};
                                reply_offset<=remote_status;
                                if(flags_high_nonzero||style_high_nonzero||req_length!=0||req_fid!=0||req_offset!=0||
                                   flags>4||
                                   ((flags==0||flags==3||flags==4)&&req_style!=0)||
                                   (flags==1&&req_style>1)||(flags==2&&req_style>2))reply_status<=3;
                                else if(flags!=0)begin
                                    remote_control_valid<=1;remote_control_action<=flags[2:0];
                                    remote_control_value<=req_style[1:0];
                                end
                            end
                        end
                        default:begin reply_status<=3;protocol_fault<=1;end
                    endcase
                end
                C_WAIT:if(AUDIO_ENABLE!=0)state<=C_READ;
                    else begin reply_status<=3;reply_length<=0;reply_crc<=0;index<=0;state<=HEADER;end
                C_READ:if(AUDIO_ENABLE!=0)begin
                    crc<=crc_byte(crc,rx_data);
                    if(payload_index==req_length-1)state<=C_CHECK;
                    else begin payload_index<=payload_index+1'b1;rx_addr<=rx_addr+1'b1;state<=C_WAIT;end
                end else begin reply_status<=3;reply_length<=0;reply_crc<=0;index<=0;state<=HEADER;end
                C_CHECK:if(AUDIO_ENABLE!=0)begin
                    if((~crc)!=req_crc)begin reply_status<=5;pcm_abort<=1;index<=0;state<=HEADER;end
                    else begin
                        if(req_offset!=next_audio)pcm_abort<=1;
                        payload_index<=0;rx_addr<=32;state<=P_WAIT;
                    end
                end else begin reply_status<=3;reply_length<=0;reply_crc<=0;index<=0;state<=HEADER;end
                P_WAIT:if(AUDIO_ENABLE!=0)state<=P_READ;
                    else begin reply_status<=3;reply_length<=0;reply_crc<=0;index<=0;state<=HEADER;end
                P_READ:if(AUDIO_ENABLE!=0)begin
                    if(!payload_index[0])begin
                        pcm_low<=rx_data;payload_index<=payload_index+1'b1;
                        rx_addr<=rx_addr+1'b1;state<=P_WAIT;
                    end else begin pcm<={rx_data,pcm_low};state<=P_READY;end
                end else begin reply_status<=3;reply_length<=0;reply_crc<=0;index<=0;state<=HEADER;end
                P_READY:if(AUDIO_ENABLE!=0)begin
                    if(pcm_ready)begin
                        if(payload_index==req_length-1)begin
                            next_audio<=req_offset+(req_length>>1);index<=0;state<=HEADER;
                        end else begin payload_index<=payload_index+1'b1;rx_addr<=rx_addr+1'b1;state<=P_WAIT;end
                    end
                end else begin reply_status<=3;reply_length<=0;reply_crc<=0;index<=0;state<=HEADER;end
                AR:if(arready)state<=R;
                R:if(rvalid)begin
                    if(rresp!=0||!rlast)begin
                        reply_status<=6;reply_length<=0;reply_crc<=0;ddr_fault<=1;index<=0;
                        if(rlast)state<=HEADER;else state<=DRAIN;
                    end else begin held<=rdata;beat_byte<=first_read?{2'b00,lease_raw_offset[3:2]}*3:0;first_read<=0;state<=EMIT;end
                end
                EMIT:begin
                    crc<=crc_byte(crc,rgb_byte);
                    reply_index<=reply_index+1'b1;
                    if(reply_index==reply_length-1)begin
                        reply_crc<=~crc_byte(crc,rgb_byte);index<=0;state<=HEADER;
                        lease_offset<=lease_offset+reply_length;
                        lease_raw_offset<=lease_raw_offset+{reply_pixels,2'b00};
                    end else if(beat_byte==11)begin raw_address<=raw_address+16;state<=AR;end
                    else beat_byte<=beat_byte+1'b1;
                end
                DRAIN:if(rvalid&&rlast)state<=HEADER;
                HEADER:begin if(index==31)state<=COMMIT;else index<=index+1'b1;end
                COMMIT:state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
