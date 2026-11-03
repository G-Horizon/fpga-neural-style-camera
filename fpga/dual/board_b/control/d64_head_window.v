`timescale 1ns/1ps

// Native P4 HEAD window: 192 x 144 bits in one physical SDP array.
// Three rows, each split into 64 four-pixel words. A word holds six interleaved
// RGB samples: left halo, four centers and right halo, with identical reflection.
// A source beat closes the previous 16-pixel chunk; its four physical words are
// written serially while d64_ready is low. The last beat also closes the final
// chunk. Only its final sub-write marks the row ready, so no reader sees a
// partially constructed row. Input comes from DDR and may be backpressured.
// rd_data retains the legacy color-major 432-bit bus to keep the seam stable;
// only the low six bytes of each 144-bit color region are meaningful. rd_sub
// selects the physical P4 word, and the controller's sample index is pix+kx.
module d64_head_window #(
    // **The shape, and this module was missing from the 2026-10-07 sweep.**
    // Twelve sites were enumerated in `docs/RECT_BOUNDARIES_2026-10-07.md`
    // across `d64_head_ctrl` and `d64_tail_engine`; the two height boundaries
    // below are the same kind of literal in a third module, and they are the
    // ones the *acceptance* counters -- `accepted_rows` and `accepted_beats`,
    // which the board gate compares against `H` and `16*H` -- are measured
    // against.  A window that still expected EOF at row 255 reads a 144-row
    // frame as a truncated 256-row one: the frame's last beat carries an EOF
    // the window was not expecting, so `marker_violations` counts it -- one,
    // measured, which is also the whole of the tooth's failure at 36 rows -- and
    // a source that kept streaming would find `expected_y` climbing past a frame
    // that has already ended.
    //
    // The bound is `H - 1` and not an 8-bit localparam for the reason
    // `d64_head_ctrl` states at its own copy: `expected_y` is unsigned here, so
    // the integer form is exactly the same comparison, and it is the form that
    // stays right when H moves.  `W` is carried for `reflect_x`, whose `255`
    // is the same literal one axis over and is bit-identical to `W - 1` at
    // W = 256.
    parameter integer H = 256,
    parameter integer W = 256
) (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         d64_valid,
    output wire         d64_ready,
    input  wire [31:0]  d64_frame_id,
    input  wire         d64_sof,
    input  wire [7:0]   d64_x0,
    input  wire [7:0]   d64_y,
    input  wire [383:0] d64_rgb888,
    input  wire         d64_eol,
    input  wire         d64_eof,

    input  wire         row_permit,
    output wire [7:0]   next_input_y,
    output wire [1:0]   next_input_slot,
    output reg  [2:0]   row_ready,
    output wire [23:0]  row_tags,
    output reg  [31:0]  frame_id,
    output reg          frame_id_valid,
    output reg  [31:0]  accepted_beats,
    output reg  [31:0]  accepted_rows,
    output reg  [31:0]  marker_violations,
    output reg  [31:0]  read_before_ready,

    input  wire         rd_en,
    input  wire [7:0]   rd_row,
    input  wire [3:0]   rd_group,
    input  wire [1:0]   rd_sub,
    output wire [431:0] rd_data,   // {B, G, R}, 18 bytes each
    output reg          rd_valid
);


    // Three complete row slots, 64 P4 windows each. RAM read remains one cycle.
    reg [7:0] row_tag[0:2];
    assign row_tags={row_tag[2],row_tag[1],row_tag[0]};
    reg [1:0] load_slot;
    reg [7:0] expected_y;
    reg [3:0] beat_index;
    reg wr_busy;
    reg [383:0] current_beat;
    reg [119:0] previous_last5;
    reg [2:0] write_phase,last_phase;
    reg [3:0] write_group;
    reg [1:0] write_slot;
    reg [7:0] write_row;
    reg write_closes_row;
    assign next_input_y=expected_y;
    assign next_input_slot=load_slot;
    assign d64_ready=rst_n && !wr_busy && ((beat_index!=0)||row_permit);
    wire accept=d64_valid && d64_ready;
    wire [7:0] beat_x0={beat_index,4'd0};
    wire marker_bad=(d64_y!=expected_y)||(d64_x0!=beat_x0)||
        (d64_sof!=((expected_y==0)&&(beat_index==0)))||
        (d64_eol!=(beat_index==15))||
        (d64_eof!=((expected_y==H-1)&&(beat_index==15)))||
        (frame_id_valid&&(d64_frame_id!=frame_id));
    // Phase 0 of a non-first beat closes the previous group's last window.
    // The other phases write this group's first three windows. The frame-row
    // final beat additionally writes its last window with reflected pixel254.
    wire closes_previous=(write_group!=0)&&(write_phase==0);
    wire [1:0] current_sub=(write_group==0)?write_phase[1:0]:
                                                       write_phase[1:0]-2'd1;
    wire [3:0] preceding_group=write_group-4'd1;
    wire [7:0] wr_addr=closes_previous?{write_slot,preceding_group,2'd3}:
                                      {write_slot,write_group,current_sub};
    wire [23:0] left_halo=(write_group==0)?current_beat[24+:24]:
                                             previous_last5[96+:24];
    reg [143:0] write_data;
    always @* begin
        if(closes_previous) write_data={current_beat[0+:24],previous_last5};
        else case(current_sub)
            0:write_data={current_beat[0+:120],left_halo};
            1:write_data=current_beat[72+:144];
            2:write_data=current_beat[168+:144];
            default:write_data={current_beat[336+:24],current_beat[264+:120]};
        endcase
    end
    (* ram_style="block" *) reg [143:0] mem[0:191];
    reg [143:0] q;
    reg found_row;
    reg [1:0] found_slot;
    wire [7:0] rd_addr={found_slot,rd_group,rd_sub};
    always @* begin
        found_row=0;found_slot=0;
        if(row_ready[0]&&row_tag[0]==rd_row)begin found_row=1;found_slot=0;end
        else if(row_ready[1]&&row_tag[1]==rd_row)begin found_row=1;found_slot=1;end
        else if(row_ready[2]&&row_tag[2]==rd_row)begin found_row=1;found_slot=2;end
    end
    genvar i;
    generate for(i=0;i<6;i=i+1)begin:g_unpack
        assign rd_data[i*8+:8]=q[i*24+16+:8];
        assign rd_data[144+i*8+:8]=q[i*24+8+:8];
        assign rd_data[288+i*8+:8]=q[i*24+:8];
    end endgenerate
    assign rd_data[48+:96]=0;
    assign rd_data[192+:96]=0;
    assign rd_data[336+:96]=0;
    always @(posedge clk) begin
        if(wr_busy)mem[wr_addr]<=write_data;
        q<=mem[rd_addr];
    end
    integer r;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            load_slot<=0;expected_y<=0;beat_index<=0;wr_busy<=0;
            current_beat<=0;previous_last5<=0;
            write_phase<=0;last_phase<=0;write_group<=0;write_slot<=0;
            write_row<=0;write_closes_row<=0;
            row_ready<=0;for(r=0;r<3;r=r+1)row_tag[r]<=0;
            frame_id<=0;frame_id_valid<=0;accepted_beats<=0;
            accepted_rows<=0;marker_violations<=0;read_before_ready<=0;rd_valid<=0;
        end else begin
            rd_valid<=rd_en;
            if(rd_en&&!found_row)read_before_ready<=read_before_ready+1;
            if(wr_busy)begin
                if(write_phase==last_phase)begin
                    wr_busy<=0;
                    previous_last5<=current_beat[264+:120];
                    if(write_closes_row)begin
                        row_tag[write_slot]<=write_row;
                        row_ready[write_slot]<=1;
                        accepted_rows<=accepted_rows+1;
                        load_slot<=(load_slot==2)?0:load_slot+1'b1;
                    end
                end else write_phase<=write_phase+1'b1;
            end
            if(accept)begin
                current_beat<=d64_rgb888;
                write_group<=beat_index;write_slot<=load_slot;write_row<=expected_y;
                write_closes_row<=beat_index==15;
                write_phase<=0;last_phase<=(beat_index==0)?2:((beat_index==15)?4:3);
                wr_busy<=1;
                accepted_beats<=accepted_beats+1;
                if(marker_bad)marker_violations<=marker_violations+1;
                if(!frame_id_valid)begin frame_id<=d64_frame_id;frame_id_valid<=1;end
                if(beat_index==0)row_ready[load_slot]<=0;
                if(beat_index==15)begin
                    beat_index<=0;
                    if(expected_y!=H-1)expected_y<=expected_y+1'b1;
                end else beat_index<=beat_index+1'b1;
            end
        end
    end
endmodule
