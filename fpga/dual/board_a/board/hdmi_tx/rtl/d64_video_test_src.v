`timescale 1ns/1ps

// The demo's source: a 256x256 test image, made one row at a time on request.
//
// H1 is "our own pattern instead of the vendor's", and this is it.  It answers to
// the same port the network's output store will (`load_req`/`load_row` in,
// `load_we`/`load_wi`/`load_wd` back), so the picture on the monitor when the
// network is wired in changes without the display path changing at all.
//
// The image is chosen to be diagnostic rather than pretty, because a test pattern
// whose only virtue is that it is not the vendor's has not earned its place:
//
//   R = column    a horizontal ramp, so a dropped or doubled column is visible
//   G = row       a vertical ramp, so a dropped or doubled row is visible
//   B = c ^ r     a fine diagonal checker, so a *positional* error is visible --
                 //  ramps alone survive a shift, and this does not
//   plus a 2-pixel white border, which puts the exact bounds of the scaled window
//   on the screen.  Nearest neighbour at 720/256 is not an integer ratio, so the
//   border is also where uneven pixel doubling shows up first.
//
// A real picture would need its 1.57 Mbit to come from somewhere -- see the header
// of d64_video_scaler for why it cannot be on chip -- and that somewhere is the
// network.  This needs no memory at all: it is 65536 combinations, computed.
module d64_video_test_src #(
    parameter integer SRC_W     = 256,
    parameter integer SRC_H     = 256,
    parameter integer BORDER_PX = 2
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        load_req,
    input  wire [7:0]  load_row,
    output reg         row_ready,     // level: the requested row is complete
    output reg         load_we,
    output reg  [7:0]  load_wi,
    output reg  [23:0] load_wd
);

    // One pixel a cycle: 256 cycles against a 720-cycle window line, so the row is
    // always finished an order of magnitude before the swap that consumes it.
    reg       busy;
    reg [7:0] col;
    reg [7:0] row;

    function [23:0] pixel;
        input [7:0] r;
        input [7:0] c;
        begin
            if ((r < BORDER_PX) || (r >= SRC_H - BORDER_PX) ||
                (c < BORDER_PX) || (c >= SRC_W - BORDER_PX))
                pixel = 24'hFFFFFF;
            else
                pixel = {c, r, c ^ r};
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            col       <= 8'd0;
            row       <= 8'd0;
            row_ready <= 1'b0;
            load_we   <= 1'b0;
            load_wi   <= 8'd0;
            load_wd   <= 24'h0;
        end
        else if (load_req && !busy) begin
            busy      <= 1'b1;
            row       <= load_row;
            col       <= 8'd0;
            row_ready <= 1'b0;                  // drops while the row is in flight
            load_we   <= 1'b1;
            load_wi   <= 8'd0;
            load_wd   <= pixel(load_row, 8'd0);
        end
        else if (busy) begin
            if (col == SRC_W - 1) begin
                busy      <= 1'b0;
                row_ready <= 1'b1;              // rises only when it is whole
                load_we   <= 1'b0;
            end
            else begin
                col     <= col + 8'd1;
                load_wi <= col + 8'd1;
                load_wd <= pixel(row, col + 8'd1);
            end
        end
    end

endmodule
