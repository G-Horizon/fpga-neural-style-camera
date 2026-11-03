`timescale 1ns/1ps

// Our own video raster, replacing the vendor test-pattern module's timing role.
//
// The vendor's `color_bar_rgb` is a pattern generator that happens to contain a
// timing generator, and the two cannot be separated from the outside.  H1 needs
// the raster without the pattern, so this produces the raster and nothing else:
// `hs`, `vs`, `de` and the position, for a CEA-861 format 4 raster.
//
//   1280x720 active, 40/220/110 horizontal, 5/20/5 vertical
//   frame 1650 x 750  ->  60.1010 Hz at a **74.375 MHz** pixel clock
//
// **The pixel clock is 74.375 MHz and this comment said 74.25 for months.**
// `hdmi_tx.sdc:11` is the authority -- "74.375 MHz, not 1000" -- and it sets the
// period to 13.445 ns, which is 74.377 MHz; the PLL runs from a 25 MHz reference
// and cannot reach 74.25.  So the raster is 0.168% faster than the CEA-861
// format 4 timing it is patterned on, and the real refresh is
// 74.375e6 / (1650 x 750) = **60.1010 Hz**, not 60.0000.
//
// That is not a nitpick.  The monitor has been accepting a 0.168%-fast signal on
// this raster for weeks, and that is the only measured evidence this project has
// about what the panel tolerates -- it is what makes a 0.168%-fast 30 Hz raster
// (see `docs/HDMI_30HZ_CANDIDATES_2026-10-07.md`, candidate C1) a reasonable
// recommendation rather than a gamble.
//
// The porch order is the vendor's -- sync, then back porch, then active, then
// front porch -- so that the active window lands where H0 already put it on the
// monitor.  Every other rotation of the same cycle would be equally valid CEA and
// would move the picture.
//
// **Everything here is registered and mutually consistent.**  `x`/`y` say where
// the raster is and `hs`/`vs`/`de` describe that same position, so a consumer can
// form an address from `x`/`y` and know that `de` refers to it.  That is worth
// stating because the vendor's module is *not* like this: its outputs are one
// cycle behind the state that produced them, which shifts its picture one pixel
// right and bleeds one de cycle into the front porch.  Harmless on a monitor, and
// confusing in a testbench, and there was no reason to reproduce it.
//
// Sync polarity is positive on both, which is what CEA-861 format 4 specifies and
// what H0 already used.
module d64_video_timing #(
    parameter integer H_ACT  = 1280,
    parameter integer H_FP   = 110,
    parameter integer H_SYNC = 40,
    parameter integer H_BP   = 220,
    parameter integer V_ACT  = 720,
    parameter integer V_FP   = 5,
    parameter integer V_SYNC = 5,
    parameter integer V_BP   = 20,
    parameter integer XW     = 12,
    parameter integer YW     = 12
) (
    input  wire          clk,
    input  wire          rst_n,

    output reg           hs,
    output reg           vs,
    output reg           de,
    output reg           frame,      // one cycle at the frame's first pixel
    output reg  [XW-1:0] x,
    output reg  [YW-1:0] y,
    // Position *within the active picture*.  `x`/`y` count the whole frame, porches
    // included, so a window placed at "column 280" is 280 columns after the sync
    // and not 280 into the picture.  That is a distinction with a bug in it: the
    // scaler's Y0 of 0 against the raw raster put its 720-row window at screen rows
    // 0..719, of which only 25..719 are active, so it drew 695 lines and the TB
    // -- which placed the window correctly -- disagreed on 53% of the frame.
    output reg  [XW-1:0] ax,
    output reg  [YW-1:0] ay
);

    localparam integer H_TOTAL = H_SYNC + H_BP + H_ACT + H_FP;
    localparam integer V_TOTAL = V_SYNC + V_BP + V_ACT + V_FP;
    localparam integer H_A0    = H_SYNC + H_BP;
    localparam integer V_A0    = V_SYNC + V_BP;

    // The counter is separate from the outputs on purpose.  Registering `hs` from
    // the counter while also advancing the counter would put the two a cycle
    // apart, which is the vendor's mistake; the outputs are registered from the
    // same counter value they accompany.
    reg [XW-1:0] xc;
    reg [YW-1:0] yc;

    wire last_x = (xc == H_TOTAL - 1);
    wire last_y = (yc == V_TOTAL - 1);

    wire hs_c = (xc < H_SYNC);
    wire vs_c = (yc < V_SYNC);
    wire de_c = (xc >= H_A0) && (xc < H_A0 + H_ACT) &&
                (yc >= V_A0) && (yc < V_A0 + V_ACT);
    wire fr_c = (xc == {XW{1'b0}}) && (yc == {YW{1'b0}});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xc <= {XW{1'b0}};
            yc <= {YW{1'b0}};
            x  <= {XW{1'b0}};
            y  <= {YW{1'b0}};
            ax <= {XW{1'b0}};
            ay <= {YW{1'b0}};
            hs <= 1'b0; vs <= 1'b0; de <= 1'b0; frame <= 1'b0;
        end
        else begin
            xc <= last_x ? {XW{1'b0}} : xc + 1'b1;
            if (last_x) yc <= last_y ? {YW{1'b0}} : yc + 1'b1;

            x  <= xc;
            y  <= yc;
            ax <= xc - H_A0[XW-1:0];
            ay <= yc - V_A0[YW-1:0];
            hs <= hs_c;
            vs <= vs_c;
            de <= de_c;
            frame <= fr_c;
        end
    end

endmodule
