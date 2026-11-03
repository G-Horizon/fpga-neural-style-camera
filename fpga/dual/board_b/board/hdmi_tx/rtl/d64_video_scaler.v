`timescale 1ns/1ps

// The 256x256 source, put on the screen.
//
// The network produces 256x256x3.  This turns that into a raster by nearest
// neighbour, and the interesting decisions are both about *where the source
// lives*, because the obvious answer does not fit.
//
// A 256x256 RGB888 frame buffer is 1.57 Mbit, and a RAM10 is 512 words of 20 bits,
// so 65536 words is 128 blocks of depth before width is even counted:
//
//   one pixel per 24-bit word     ceil(24/20) x 128 = 256 RAM10  -- the whole device
//   packed five pixels per word     6 columns x   26 = 156 RAM10
//   RGB565, one pixel per word      1 column  x  128 = 128 RAM10
//
// The ledger this has to fit in is H0's, not M1's -- the DDR3 pivot moved the trunk
// off chip and `docs/D3C1_DDR3_MEMORY_POLICY.md` is the measured budget:
//
//   DDR3 controller 22 + weights 52 + A/B rings 22 + X ring 32
//   + write FIFO 26 + read CDC 7 + line buffer 4          = 165 of 256, 91 spare
//   less these two rows, 4                                = 169,            87 spare
//
// The cheapest on-chip frame buffer of any packing is 128, and 169 + 128 > 256.
// **There is no frame buffer to be had**, not by a little, and a design that used
// one here would be measuring something nobody can build.
//
// So the source is *streamed*, and that is affordable because of a property of the
// scaler: nearest neighbour is monotonic.  Source rows are consumed in order and
// source columns are consumed in order, so the display only ever needs the current
// row and the one after it.  Two rows of 256 x 24 bits is **4 RAM10**, against 128
// at the very least for a frame buffer, and it is the same shape the rest of this
// project already uses everywhere else.
//
// The row loader is a port, not a generator.  `load_req`/`load_row` out and
// `load_we`/`load_wi`/`load_wd` in is the whole interface, so the demo's pattern
// generator and the real network's output store attach to the same pins -- which
// is the point of building it this way now rather than wiring a ROM and rewriting
// it later.
//
// How the two rows work
// ---------------------
//
// Ping-pong, one swap per *window* line.  During window line n the display reads
// the bank holding s(n) while the loader fills the other bank with s(n+1); at the
// start of line n+1 the two swap.  The loader therefore has a whole window line
// -- 720 pixel clocks -- to write 256 pixels, and it starts at the beginning of
// the line before it is needed.
//
// One request per window line, which is one more than the source needs
// ---------------------------------------------------------------
//
// `line_0 && row_ready` swaps and re-requests on *every* window line, including the
// two or three in a row where `stepy` is 0 and the display wants the row it already
// has.  `d64_video_test_src` does not mind -- it can regenerate a row as often as it
// is asked -- but a one-pass 120 MHz tail cannot: it produces each source row once
// and has no way to send it again.
//
// H3 should therefore be:
//
//   stepy == 0   keep the bank, request nothing
//   stepy == 1   swap to the row already prefetched, then request the next unique one
//
// which is what "the display only needs the current row and the next" actually
// means.  It is **not done here** on purpose: with a synthetic source it would be
// optimising an interface against a producer that does not exist yet, and the
// change belongs with the tail it has to match.
//
// `row_ready` is a **level**, not a pulse.  A pulse would race with the swap: the
// scaler samples readiness on the same edge it issues the next request, so a
// one-cycle `load_done` arriving on that edge is either consumed twice or lost
// depending on which side of the edge it lands.  A level that drops when a load
// starts and rises when it finishes has no such edge.
//
// Both banks are read at `sx` every cycle and the bank select is applied to the
// registered outputs.  That is what makes the swap atomic: at the swap edge both
// banks have already captured the same address, so only the select changes and the
// first pixel of the new line is the right one rather than the old bank's.
//
// The scaler
// ----------
//
// `sx` and `sy` are Bresenham, not division: `sx(i) = floor(i * SRC_W / WIN_W)`
// from an accumulator that adds `SRC_W` per output pixel and subtracts `WIN_W`
// when it overflows.  A divide per pixel would be absurd at 74.25 MHz and this is
// exact.  The window is a parameter, so an integer scale is the same code.
module d64_video_scaler #(
    parameter integer SRC_W  = 256,
    parameter integer SRC_H  = 256,
    parameter integer WIN_W  = 720,      // the source is shown 720x720, centred:
    parameter integer WIN_H  = 720,      // the full height, square pixels
    parameter integer X0     = 280,      // (1280 - 720) / 2
    parameter integer Y0     = 0,
    parameter [23:0]  BORDER = 24'h000000,
    //0 preserves the historical per-window-line loader behavior. Positive
    // integer repetition holds each source row and fetches it only once.
    parameter integer SOURCE_ROW_REPEAT = 0,
    parameter integer CHECK_ROW_GOOD = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // The raster, from d64_video_timing.  x/y and de/hs/vs describe the same
    // position, which is the contract that module keeps.
    input  wire        hs_i,
    input  wire        vs_i,
    input  wire        de_i,
    input  wire        frame_i,
    // Active-picture coordinates, not the raw raster: X0/Y0 are offsets into the
    // picture, which is what "the window is centred" means.
    input  wire [11:0] ax_i,
    input  wire [11:0] ay_i,

    // The source row loader: ask for a row, and receive one.
    output reg         load_req,
    output reg  [7:0]  load_row,
    input  wire        row_ready,
    input  wire        row_good,
    input  wire        load_we,
    input  wire [7:0]  load_wi,
    input  wire [23:0] load_wd,

    // The raster, out.  All four move together and are aligned with `rgb_o`.
    output reg         hs_o,
    output reg         vs_o,
    output reg         de_o,
    output reg  [23:0] rgb_o,
    output reg         pixel_good_o,

    // Sticky: a window line started before its source row had arrived.  The line
    // is repeated rather than corrupted, and the gate is that this never fires.
    output reg         starved,
    output wire        row_swap_attempt,
    output wire        row_swap_miss
);

    localparam integer AWX = $clog2(WIN_W + SRC_W);
    localparam integer AWY = $clog2(WIN_H + SRC_H);

    // ------------------------------------------------------------------
    // where the raster is, in source coordinates
    // ------------------------------------------------------------------

    wire [11:0] wx = ax_i - X0[11:0];

    wire in_col  = de_i && (ax_i >= X0[11:0]) && (ax_i < X0[11:0] + WIN_W[11:0]);
    wire in_row  = de_i && (ay_i >= Y0[11:0]) && (ay_i < Y0[11:0] + WIN_H[11:0]);
    wire in_win  = in_col && in_row;
    // **`in_row` is load-bearing, and it was not here.**  `line_0` used to be
    // `in_col && (wx == 0)` -- window *columns* only -- so it fired once on
    // every active line that carries window columns, including the `Y0` active
    // lines above the window and the lines below it.  Each firing steps the
    // vertical Bresenham accumulator, swaps the row banks and raises
    // `load_req`, so a window whose top is below the top of the active picture
    // is displaced **downward by exactly `Y0` source rows**: its first line
    // carries source row `Y0` where source row 0 belongs, and every line after
    // it is `Y0` rows ahead of where it should be.  Reproduced in
    // `sim/scaler_y0/`, gate `tools/run_scaler_y0.sh` (task #102): a 256x256
    // window at `Y0 = 8` and again
    // at `Y0 = 20` is shifted by exactly that many source rows, and at `Y0 = 0`
    // -- the board's preview geometry, `WIN_H = V_ACT` -- `in_row` is true for
    // every active line and this line changes nothing.
    wire line_0  = in_win && (wx == 12'd0);
    // The consumer exposes its actual swap decision edge. Wrappers must not
    // reconstruct this event from independent raster coordinates.
    localparam integer REPEAT_W=(SOURCE_ROW_REPEAT>1)?$clog2(SOURCE_ROW_REPEAT):1;
    reg [REPEAT_W-1:0] repeat_phase;
    wire source_group_start=(SOURCE_ROW_REPEAT==0)||(repeat_phase==0);
    wire accepted_row_good=(CHECK_ROW_GOOD==0)||row_good;
    assign row_swap_attempt=line_0 && source_group_start;
    assign row_swap_miss=row_swap_attempt && (!row_ready||!accepted_row_good);

    // ------------------------------------------------------------------
    // horizontal: which source column, once per output pixel
    // ------------------------------------------------------------------
    //
    // Reset whenever the raster is not in the window, so the first window pixel of
    // a line always reads column 0.  `sx` is `SRC_W` wide because the bound is
    // floor((WIN_W-1)*SRC_W/WIN_W) <= SRC_W-1.

    reg [AWX-1:0] accx;
    reg [7:0]     sx;

    wire [AWX:0] accx_n  = accx + SRC_W;
    wire         stepx   = (accx_n >= WIN_W);
    wire [AWX:0] accx_nx = stepx ? (accx_n - WIN_W) : accx_n;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accx <= {AWX{1'b0}};
            sx   <= 8'd0;
        end
        else if (!in_col) begin
            accx <= {AWX{1'b0}};
            sx   <= 8'd0;
        end
        else begin
            accx <= accx_nx[AWX-1:0];
            if (stepx) sx <= sx + 8'd1;
        end
    end

    // ------------------------------------------------------------------
    // vertical: which source row, once per window line
    // ------------------------------------------------------------------
    //
    // `sy_next` is the row the *next* window line will show, and it is the one
    // being loaded now.  Stepping it and issuing the request happen together at
    // the top of the line, because the step is what decides which row that is.

    reg [AWY-1:0] accy;
    reg [7:0]     sy_next;
    reg           bank;              // the bank the display is reading
    reg           bank_good;

    wire [AWY:0] accy_n  = accy + SRC_H;
    wire         stepy   = (accy_n >= WIN_H);
    wire [AWY:0] accy_nx = stepy ? (accy_n - WIN_H) : accy_n;
    wire [7:0]   sy_step = sy_next + {7'd0, stepy};

    generate if(SOURCE_ROW_REPEAT==0)begin:g_legacy_rows
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accy    <= {AWY{1'b0}};
            sy_next <= 8'd0;
            bank    <= 1'b0;
            bank_good<=0;repeat_phase<=0;
        end
        else if (frame_i) begin
            // A frame starts at s(0) = 0 and the first swap takes the display into
            // bank 1, so the prime writes row 0 into bank 1 -- which is exactly
            // `~bank` and needs no special case.
            accy    <= {AWY{1'b0}};
            sy_next <= 8'd0;
            bank    <= 1'b0;
            bank_good<=0;repeat_phase<=0;
        end
        else if (line_0 && row_ready) begin
            accy    <= accy_nx[AWY-1:0];
            sy_next <= sy_step;
            bank    <= ~bank;
            bank_good<=accepted_row_good;
        end
        else if(line_0)bank_good<=0;
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n)        starved <= 1'b0;
        else if (row_swap_miss) starved <= 1'b1;

    // ------------------------------------------------------------------
    // the row requests
    // ------------------------------------------------------------------
    //
    // One at the top of every window line, plus one at the top of the frame to
    // prime bank 1.  Both target `~bank`, which is the bank the display is not
    // reading.

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            load_req <= 1'b0;
            load_row <= 8'd0;
        end
        else begin
            load_req <= frame_i | (line_0 & row_ready);
            // `frame_i` primes s(0) = 0.  A line request asks for the row the
            // *next* line will show, which is one Bresenham step on from the row
            // being loaded now.
            load_row <= frame_i ? 8'd0 : sy_step;
        end
    end
    end else begin:g_unique_integer_rows
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                accy<=0;sy_next<=0;bank<=0;bank_good<=0;repeat_phase<=0;starved<=0;
            end else if(frame_i)begin
                accy<=0;sy_next<=0;bank<=0;bank_good<=0;repeat_phase<=0;
            end else if(line_0)begin
                if(repeat_phase==SOURCE_ROW_REPEAT-1)repeat_phase<=0;
                else repeat_phase<=repeat_phase+1'b1;
                if(source_group_start)begin
                    if(row_ready)begin
                        bank<=~bank;bank_good<=accepted_row_good;
                        sy_next<=sy_next+8'd1;
                    end else bank_good<=0;
                    if(!row_ready||!accepted_row_good)starved<=1;
                end
            end
        end
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin load_req<=0;load_row<=0;end
            else begin
                load_req<=frame_i||(row_swap_attempt&&row_ready&&(sy_next<SRC_H-1));
                load_row<=frame_i?8'd0:sy_next+8'd1;
            end
        end
    end endgenerate

    // ------------------------------------------------------------------
    // the two row banks
    // ------------------------------------------------------------------

    // The loader writes `~bank` -- the bank the display is NOT reading.  Getting
    // this backwards is not a wrong picture, it is a *nearly* right one: the
    // loader rewrites the line being scanned a few pixels behind the scan, so the
    // display shows the previous row's contents all frame, and a frame whose rows
    // happen to repeat looks perfectly fine.
    wire we0 = load_we &  bank;
    wire we1 = load_we & ~bank;

    wire [23:0] q0, q1;

    d64_tc_ram #(.DW(24), .AW(8)) u_row0 (
        .clk(clk), .we(we0), .wa(load_wi), .wd(load_wd),
        .re(1'b1), .ra(sx), .rd(q0)
    );

    d64_tc_ram #(.DW(24), .AW(8)) u_row1 (
        .clk(clk), .we(we1), .wa(load_wi), .wd(load_wd),
        .re(1'b1), .ra(sx), .rd(q1)
    );

    wire [23:0] src_data = bank ? q1 : q0;

    // ------------------------------------------------------------------
    // the output pipeline
    // ------------------------------------------------------------------
    //
    // Two stages, and both are needed.  The first pairs `in_col` with the data
    // that arrives a cycle after the address it gated; without it the enable and
    // the pixel are one apart and every line shifts by one.  The second is the
    // registered output the encoder expects, and it takes `hs`/`vs` with it so
    // that sync and data keep the same relationship they had at the input.

    reg de_d, win_d, hs_d, vs_d,good_d;
    // RAMdata at a swap edge uses the newly selected bank. Its good verdict
    // must use the same edge and then traverse the same two pixel stages.
    wire selected_good=row_swap_attempt ? (row_ready&&accepted_row_good) : bank_good;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d <= 1'b0; win_d <= 1'b0; hs_d <= 1'b0; vs_d <= 1'b0;
            good_d<=0;
        end
        else begin
            // **`de` is the whole active region, not the window.**  DVI uses it to
            // choose between video and control symbols, so it has to be high for
            // all 1280x720 -- the letterbox is black *video*, not blanking.  `de`
            // as the window drew 720x720 of picture and then sent control symbols
            // across the rest of every line, which is a raster no monitor will
            // lock to.
            de_d  <= de_i;
            win_d <= in_win;
            hs_d  <= hs_i;
            vs_d  <= vs_i;
            good_d<=in_win&&selected_good;
        end
    end

    // `win_d` is one cycle behind the address and `src_data` is one cycle behind
    // it, so these are the same pixel.
    wire [23:0] pix = win_d ? src_data : BORDER;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_o <= 1'b0; hs_o <= 1'b0; vs_o <= 1'b0; rgb_o <= 24'h0;
            pixel_good_o<=0;
        end
        else begin
            de_o  <= de_d;
            hs_o  <= hs_d;
            vs_o  <= vs_d;
            rgb_o <= pix;
            pixel_good_o<=good_d;
        end
    end

endmodule
