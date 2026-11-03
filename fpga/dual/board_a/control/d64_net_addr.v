`timescale 1ns/1ps

// ===========================================================================
// F2 section 2.3 -- where the decoder's intermediates live in DDR.
//
// A pure combinational address/stride map.  No clock, no state, no memory, no
// register: four functions, one per tensor, and one parameter list that says
// out loud what each region is.
//
// ---------------------------------------------------------------------------
// The layout, and why it is HWC
// ---------------------------------------------------------------------------
//
// The tconv engine does not address its input store by channel.  It addresses
// it by *pixel* and then multiplies -- the assign is on :311 and the
// multiply on :312:
//
//   control/d64_tconv_engine.v:311-312
//     in_rd_addr[gp*IN_WORD_W +: IN_WORD_W] =
//         (s_in_rd_addr[gp*IN_ADDR_W +: IN_ADDR_W] * IC_GROUPS) + icg_w;
//
// so a pixel's IC_GROUPS words are contiguous and the pixel index (h*W + w) is
// the slower term.  The trunk writes res4 the same way -- `C3_WORDS = 4` words
// of eight channels per pixel, control/d64_trunk_scheduler.v:213.  Both are
// channels-last with the channel group innermost, and this map is that same
// ordering expressed in bytes:
//
//     addr(n,c,h,w) = BASE + n*S_N + h*S_H + w*S_W + c*S_C,     S_C smallest
//
// One consequence is what the prefetch policy below rests on: a *row* -- one
// image row, every channel -- is one contiguous run of W*C*ELEM_B bytes.
//
// ---------------------------------------------------------------------------
// The map.  Byte addresses; bases and strides are parameters, not literals
// ---------------------------------------------------------------------------
//
//   region  tensor      base        size       S_C  S_W    S_H    S_N      region end
//   RES4    64x64x32    0x00400000  131072       1   32   2048  131072   0x00420000
//   UPS0    128x128x16  0x00440000  262144       1   16   2048  262144   0x00480000
//   UPS1    256x256x8   0x00480000  524288       1    8   2048  524288   0x00500000
//   TAIL    256x256x4   0x00500000  262144       1    4   1024  262144   0x00540000
//   (element = 2 bytes, the U16 image; see RES4_ELEM_B, the one parameter that
//    moves any of these numbers)
//
//   every base is a 256 KiB multiple and every region end is one too, so the
//   four regions are adjacent at 256 KiB boundaries with no gap and no
//   straddle.  Sum of regions 1,179,648 B = 1.125 MiB.  The map spans
//   0x0040_0000..0x0054_0000 = 1.25 MiB; the difference is the alignment
//   slack RES4 leaves inside its 256 KiB slot.
//
// TAIL is one 32-bit word a pixel (F2 section 2.2, `0x00RRGGBB`).  Its inner
// extent of 4 is the four byte lanes of that word and is addressed as such:
// lane 0 is B, 1 is G, 2 is R, 3 is X for a little-endian 0x00RRGGBB.  The tail
// engine wants a word address for `out_wr_addr[17:0]`, and gets it from
// `tail_word()` -- which is exactly 18 bits wide for a 256x256 grid, which is
// where that port width came from.
//
// ---------------------------------------------------------------------------
// The boundary this must not cross, and where it was found
// ---------------------------------------------------------------------------
//
// The encoder/trunk chain's regions, read out of the RTL rather than out of a
// table:
//
//   0x0010_0000  HEAD     512 KiB  board/ddr3/d64_encoder_trunk_board_top.v:704
//   0x0018_0000  DOWN0    256 KiB  board/ddr3/d64_encoder_trunk_board_top.v:705
//   0x001C_0000  TRUNK_A  512 KiB  board/ddr3/d64_encoder_trunk_board_top.v:570
//   0x0024_0000  TRUNK_B  512 KiB  board/ddr3/d64_encoder_trunk_board_top.v:571
//
// The widths are the *addressed* widths, which is what a neighbour has to
// respect, and each one was checked against the file that walks it:
//
//   HEAD   control/d64_encoder_ddr_mem.v:544
//            wire [15:0] rd_total = rd_kind_axi ? 16'd16384 : 16'd32768;
//          from `rd_base_axi <= H_BASE` (:586), 32768 beats x 16 B = 512 KiB,
//          which is exactly the distance from H_BASE to D0_BASE.
//   DOWN0  the same expression's other arm, 16384 x 16 B = 256 KiB, exactly
//          the distance from D0_BASE to TRUNK_A_BASE.
//   TRUNK  board/ddr3/d64_trunk_row_ddr.v:54 takes `x_req_row` as [6:0] and
//          :271 forms `rd_addr = rd_base + {13'd0, rd_row, 12'd0}` -- 128 rows
//          of 4 KiB = 512 KiB per buffer, and TRUNK_B_BASE - TRUNK_A_BASE is
//          exactly 0x80000.  The RTL and the constant spacing agree, which is
//          the check that makes 512 KiB the number and not 256.
//
//          A *block* is 64 rows (BLK_BEATS = 16384, board/ddr3/d64_trunk_row_ddr.v:35),
//          so only 256 KiB of each buffer is used today -- but 512 KiB is
//          reachable, and a neighbour avoids what is reachable.
//
// So **the encoder/trunk chain ends at 0x002C_0000**, exclusive.  Two further
// claims sit beside it and are avoided as well: the HDMI display path's
// VIDEO_A/VIDEO_B at 0x0028_0000..0x0030_0000 (board/hdmi_tx/rtl/d64_video_row_ddr.v:204,
// `SRC_BASE = 32'h0028_0000`, two 256 KiB frames).  The highest address
// anything in this tree claims is 0x0030_0000, and this map starts a full
// 1 MiB above it at 0x0040_0000 -- so the boundary holds whichever reading of
// TRUNK_B's extent is the right one, and a later widening of the trunk's
// buffer cannot reach the decoder's tensors.
//
// The controller's window is `AXI_ADDR_WIDTH = 28` (board/ddr3/rtl/ddr3_controller/ddr3_parameter.vh:22),
// so the whole map is inside the 256 MiB the part actually has.
//
// ---------------------------------------------------------------------------
// The plan's budget, reconciled
// ---------------------------------------------------------------------------
//
// The project plan says "UPS0 about 256 KiB, UPS1 about 512 KiB, final RGBX
// 256 KiB".  Measured against the table above:
//
//   UPS0  128*128*16 = 262144 B = 256 KiB exactly -- nothing was rounded
//   UPS1  256*256*8  = 524288 B = 512 KiB exactly
//   TAIL  256*256*4  = 262144 B = 256 KiB exactly
//
// All three plan figures are exact, so the plan did not round and there is no
// conflict; the byte counts from the table are used because they are the same
// numbers before a unit is introduced, and because the checker enumerates
// bytes.  The plan does not budget RES4 at all; the table's 131072 B is used.
//
// **One discrepancy, and it is RES4.**  The table calls RES4 "as produced by
// the trunk chain today", and the trunk chain does not produce one byte per
// element.  It produces a **U16 transport image**: the conv3 output is 80 bits
// of 8 x 10-bit (`control/d64_encoder_trunk.v:302  output wire [79:0] out_data`)
// zero-extended lane by lane into 8 x U16 on the way to DDR
// (docs/D3C6D_ENCODER_DDR.md:99, docs/D3C6D1_ENCODER_DDR_BOARD.md:51).  That is
// 64 bytes a pixel and 256 KiB for a 64x64x32 tensor, not 128 KiB -- and it is
// also why the row buffer that carries it is 4 KiB
// (board/ddr3/d64_trunk_row_ddr.v:16, "A row is 4 KiB").
//
// So `RES4_ELEM_B` is a parameter and **2 is the default**, because 2 is the
// reading that has a producer.  1 would size RES4 at 131072 B, and the trap in
// that number is worth stating plainly: **131072 is RES4's element count, not
// its byte count.**  64 x 64 x 32 = 131072 elements -- the exported model's
// `B32>16>32` trunk block outputs 32 channels on a 64x64 grid, confirmed by
// `d64_trunk_scheduler.v`'s `A_WORDS=2`/`B_WORDS=2`/`C3_WORDS=4` (16/16/32) --
// and at one byte an element that is the size, which is why it looked right.
// The trunk does not write one byte an element.  Nothing does.
//
// At 2 the RES4 strides become S_C = 2, S_W = 64, S_H = 4096 and the region is
// 262144 B, which still fits inside the 256 KiB slot RES4 owns on this map, so
// **nothing else in the map moves** and both readings remain safe against
// everything above.  tools/check_net_addr.py enumerates every address of both,
// and it is the checker that keeps configuration 1 honest rather than a default
// that happens to be unused.
//
// ---------------------------------------------------------------------------
// The prefetch policy
// ---------------------------------------------------------------------------
//
// **A tap must never become a DDR read.**  A layer is 147,456 taps
// (docs/D3C4B_TCONV_SCHEDULE.md) and the scheduler offers one a cycle
// (control/d64_tconv_scheduler.v), while a DDR burst is 2 KiB and takes many
// cycles: a tap-per-read policy is not an addressing choice, it is two orders
// of magnitude of bus traffic.  So:
//
//   * the intermediate is read by ROW, in bursts, into an on-chip cache;
//   * the MAC and the store read the cache, and nothing downstream of the
//     cache knows DDR exists;
//   * a fetch is issued one row ahead of the row the window needs.
//
//   burst length   2048 bytes  -- 128 x 16-byte beats, AXI INCR
//   cache          2 rows      -- 8192 bytes for RES4 (4096 at RES4_ELEM_B = 1)
//
// Why 2048 bytes.  It is the burst this design already issues and already
// accepts, and it is not this module's to renegotiate: `BURST_LEN = 8'd127`
// with the comment "128 beats of 16 bytes = 2 KiB"
// (board/ddr3/d64_trunk_row_ddr.v:33, and the reason for it stated at :16),
// and the encoder's reader forms `rd_burst_offset = {13'd0, rd_burst_q, 11'd0}`
// -- 2 KiB a burst (control/d64_encoder_ddr_mem.v:543).  And a row of each of the three tensors
// the decoder reads is *exactly* one burst: 64*32 = 128*16 = 256*8 = 2048.
// One burst is one row is one cache fill: no partial row, no row straddling a
// burst boundary.  That is the property that makes this policy checkable
// rather than merely plausible, and the module states it as `sel_row_whole`.
//
// Why 2 rows.  The window is two input rows tall and the scheduler's loop is
// `for r, for c, for phase` with `src_row = r + dy`, dy in {0,1}
// (control/d64_tconv_scheduler.v:25 and :199).  One output row pair therefore
// touches input rows r and r+1 and nothing else, and the next pair touches
// r+1 and r+2.  Two rows is the minimum that holds a pair's window, and with
// two the step to the next pair evicts exactly one row and fetches exactly one
// -- so **each input row is fetched from DDR once per layer**, not twice.
// One row would stall every second pair; three would be a third more RAM10,
// which this project has none of (docs/D3C6F_RAM_BUDGET.md).
//
// What that costs, per pass of the decoder's three tconv layers, counting the
// reads only: RES4 131072 B (64 bursts), UPS0 262144 B (128), UPS1 524288 B
// (256).  Those are arithmetic from this policy and these shapes -- `sel_bursts`
// is the same number -- and they are **not** a measurement; nothing here has
// been run against a DDR, and no rate is claimed.
//
// ---------------------------------------------------------------------------
// What is verified and what is not
// ---------------------------------------------------------------------------
//
// tools/check_net_addr.py enumerates every address of every tensor through
// these functions and proves non-overlap, containment and injectivity, and
// reports the counts.  It is a mirror driven against the module, not a proof
// about it: see docs/NET_ADDR_2026-10-06.md for the exact claim.  What is *not*
// verified here: that a prefetcher built on this map meets any particular
// throughput, that these regions are free on a board that has been programmed
// (nothing has), and that the decoder's layers will want exactly these shapes.
// ===========================================================================

module d64_net_addr #(
    // ---- region 1: RES4, the decoder's input ------------------------------
    // `RES4_ELEM_B` is the only parameter here that is not a layout constant
    // chosen once: see the U16 note in the header.
    parameter [31:0]  RES4_BASE   = 32'h0040_0000,
    parameter integer RES4_N      = 1,
    parameter integer RES4_C      = 32,
    parameter integer RES4_H      = 64,
    parameter integer RES4_W      = 64,
    // 2, and the reason is in the header: it is the reading with a producer.
    // Set to 1 to reproduce the contract table's byte count -- the checker does,
    // and enumerates every address of both.
    parameter integer RES4_ELEM_B = 2,

    // ---- region 2: UPS0's output ------------------------------------------
    // **UPS0 and UPS1 are U16 transport images, decided 2026-10-07.**  The
    // decoder's activations are 8-bit values, but they are *transported* the
    // way the trunk transports its own: a channel in a 16-bit container, so
    // eight channels are 8 x 16 = 128 bits = exactly one AXI beat = exactly one
    // store word.  The cache is built at `IN_W = 8`, `DDR_LANE_W = 16`, and the
    // unpack committed in `d64_burst_prefetch` is reused unchanged -- one code
    // path for all three tensors instead of a second one that has never been
    // written.
    //
    // The cost is DDR, and it is paid deliberately: UPS0 262144 -> 524288 B and
    // UPS1 524288 -> 1048576 B.  **The values and the on-chip cache stay U8**;
    // only the transport container widens.  That is why the bases below move --
    // UPS0 no longer fits between its old base and UPS1's.
    //
    // RES4 was already 2 and TAIL is 1 by construction: a pixel there is a
    // 32-bit `0x00RRGGBB` word, which *is* its transport.
    parameter [31:0]  UPS0_BASE   = 32'h0044_0000,
    parameter integer UPS0_N      = 1,
    parameter integer UPS0_C      = 16,
    parameter integer UPS0_H      = 128,
    parameter integer UPS0_W      = 128,
    parameter integer UPS0_ELEM_B = 2,   // U16 transport; the value stays U8

    // ---- region 3: UPS1's output ------------------------------------------
    // Moved from 0x0048_0000: UPS0's U16 image now ends at 0x004C0000.
    parameter [31:0]  UPS1_BASE   = 32'h004C_0000,
    parameter integer UPS1_N      = 1,
    parameter integer UPS1_C      = 8,
    parameter integer UPS1_H      = 256,
    parameter integer UPS1_W      = 256,
    parameter integer UPS1_ELEM_B = 2,   // U16 transport; the value stays U8

    // ---- region 4: the tail's output, RGBX, one 32-bit word a pixel -------
    // The inner extent of 4 is the word's four byte lanes, not four feature
    // channels; `TAIL_ELEM_B` stays 1 and the byte stride of a lane is 1.
    // Moved from 0x0050_0000: UPS1's U16 image now ends at **0x005C0000**
    // (0x004C0000 + 1048576), so TAIL sits against it with no gap.  The
    // first draft of this put TAIL at 0x00600000, which passes every check
    // -- no overlap, in range, injective -- and quietly wastes 256 KiB and
    // states the wrong end address in this comment.  A gap is not a failure
    // the gate looks for; it is a thing a reader has to notice.
    parameter [31:0]  TAIL_BASE   = 32'h005C_0000,
    parameter integer TAIL_N      = 1,
    parameter integer TAIL_C      = 4,
    parameter integer TAIL_H      = 256,
    parameter integer TAIL_W      = 256,
    parameter integer TAIL_ELEM_B = 1,

    // ---- the prefetch policy (see the header) -----------------------------
    // These are on the interface because they are the contract the reader
    // side builds against, and because a port list cannot see a localparam.
    parameter [31:0]  BURST_B     = 2048,   // one AXI INCR burst, 128 x 16 B
    parameter [31:0]  CACHE_ROWS  = 2,      // rows resident while a pair runs

    // ---- derived strides, one set per tensor ------------------------------
    // Written as products of the parameters above and not as literals, so
    // that changing a shape cannot leave a stale stride behind.  The checker
    // re-derives these independently and requires agreement.
    parameter [31:0]  RES4_S_C    = RES4_ELEM_B,
    parameter [31:0]  RES4_S_W    = RES4_C * RES4_ELEM_B,
    parameter [31:0]  RES4_S_H    = RES4_W * RES4_S_W,
    parameter [31:0]  RES4_S_N    = RES4_H * RES4_S_H,
    parameter [31:0]  RES4_ROW_B  = RES4_W * RES4_S_W,
    parameter [31:0]  RES4_SIZE   = RES4_N * RES4_S_N,

    parameter [31:0]  UPS0_S_C    = UPS0_ELEM_B,
    parameter [31:0]  UPS0_S_W    = UPS0_C * UPS0_ELEM_B,
    parameter [31:0]  UPS0_S_H    = UPS0_W * UPS0_S_W,
    parameter [31:0]  UPS0_S_N    = UPS0_H * UPS0_S_H,
    parameter [31:0]  UPS0_ROW_B  = UPS0_W * UPS0_S_W,
    parameter [31:0]  UPS0_SIZE   = UPS0_N * UPS0_S_N,

    parameter [31:0]  UPS1_S_C    = UPS1_ELEM_B,
    parameter [31:0]  UPS1_S_W    = UPS1_C * UPS1_ELEM_B,
    parameter [31:0]  UPS1_S_H    = UPS1_W * UPS1_S_W,
    parameter [31:0]  UPS1_S_N    = UPS1_H * UPS1_S_H,
    parameter [31:0]  UPS1_ROW_B  = UPS1_W * UPS1_S_W,
    parameter [31:0]  UPS1_SIZE   = UPS1_N * UPS1_S_N,

    parameter [31:0]  TAIL_S_C    = TAIL_ELEM_B,
    parameter [31:0]  TAIL_S_W    = TAIL_C * TAIL_ELEM_B,
    parameter [31:0]  TAIL_S_H    = TAIL_W * TAIL_S_W,
    parameter [31:0]  TAIL_S_N    = TAIL_H * TAIL_S_H,
    parameter [31:0]  TAIL_ROW_B  = TAIL_W * TAIL_S_W,
    parameter [31:0]  TAIL_SIZE   = TAIL_N * TAIL_S_N,

    // ---- derived: an address must be inside its region ---------------------
    parameter [31:0]  RES4_END    = RES4_BASE + RES4_SIZE,
    parameter [31:0]  UPS0_END    = UPS0_BASE + UPS0_SIZE,
    parameter [31:0]  UPS1_END    = UPS1_BASE + UPS1_SIZE,
    parameter [31:0]  TAIL_END    = TAIL_BASE + TAIL_SIZE
) (
    // One selected tensor per access.  This is a map, not a reader: it says
    // where a thing is, and the reader decides when to ask.
    input  wire [1:0]  tsel,
    input  wire [15:0] t_n,
    input  wire [15:0] t_c,
    input  wire [15:0] t_h,
    input  wire [15:0] t_w,

    // the element
    output wire [31:0] addr,
    // the contiguous run the element's row occupies, and where it starts --
    // the row start IS the burst address for every tensor whose row is whole
    // bursts, which is every tensor here
    output wire [31:0] row_base,
    // the selected tensor's own numbers, so a reader never carries a copy
    output wire [31:0] sel_base,
    output wire [31:0] sel_size,
    output wire [31:0] sel_s_c,
    output wire [31:0] sel_s_w,
    output wire [31:0] sel_s_h,
    output wire [31:0] sel_s_n,
    output wire [15:0] sel_n,
    output wire [15:0] sel_c,
    output wire [15:0] sel_h,
    output wire [15:0] sel_w,
    output wire [31:0] sel_row_b,      // bytes in one row, all channels
    output wire [31:0] sel_rows,       // rows in the tensor: N*H
    output wire [31:0] sel_bursts,     // SIZE / BURST_B, exact for every tensor
    output wire [31:0] sel_row_bursts, // ROW_B / BURST_B
    output wire        sel_row_whole,  // ROW_B % BURST_B == 0: no row straddles
    output wire [31:0] sel_cache_b,    // CACHE_ROWS * ROW_B
    output wire        sel_in_region,  // is `addr` inside the selected region

    // the tail engine's word address for its `out_wr_addr[17:0]`
    output wire [17:0] tail_word_addr,

    // the policy, not the tensor
    output wire [31:0] burst_b,
    output wire [31:0] cache_rows,

    // the whole map, for whoever has to prove it fits
    output wire [31:0] map_lo,
    output wire [31:0] map_hi,
    output wire [31:0] map_span,
    output wire [31:0] map_used
);

    localparam [1:0] T_RES4 = 2'd0,
                     T_UPS0 = 2'd1,
                     T_UPS1 = 2'd2,
                     T_TAIL = 2'd3;

    // -----------------------------------------------------------------------
    // One function per tensor.  Each is the whole of that tensor's arithmetic
    // and there is nowhere else an address for it is formed.
    // -----------------------------------------------------------------------

    // RES4 -- 64x64x32, the trunk's U16 image at the default RES4_ELEM_B = 2
    function [31:0] res4_addr;
        input [15:0] fn;
        input [15:0] fc;
        input [15:0] fh;
        input [15:0] fw;
        begin
            res4_addr = RES4_BASE + (fn * RES4_S_N) + (fh * RES4_S_H)
                                  + (fw * RES4_S_W) + (fc * RES4_S_C);
        end
    endfunction

    // UPS0 -- 128x128x16
    function [31:0] ups0_addr;
        input [15:0] fn;
        input [15:0] fc;
        input [15:0] fh;
        input [15:0] fw;
        begin
            ups0_addr = UPS0_BASE + (fn * UPS0_S_N) + (fh * UPS0_S_H)
                                  + (fw * UPS0_S_W) + (fc * UPS0_S_C);
        end
    endfunction

    // UPS1 -- 256x256x8
    function [31:0] ups1_addr;
        input [15:0] fn;
        input [15:0] fc;
        input [15:0] fh;
        input [15:0] fw;
        begin
            ups1_addr = UPS1_BASE + (fn * UPS1_S_N) + (fh * UPS1_S_H)
                                  + (fw * UPS1_S_W) + (fc * UPS1_S_C);
        end
    endfunction

    // TAIL -- 256x256 words of RGBX; the inner 4 is the word's byte lanes
    function [31:0] tail_addr;
        input [15:0] fn;
        input [15:0] fc;
        input [15:0] fh;
        input [15:0] fw;
        begin
            tail_addr = TAIL_BASE + (fn * TAIL_S_N) + (fh * TAIL_S_H)
                                  + (fw * TAIL_S_W) + (fc * TAIL_S_C);
        end
    endfunction

    // -----------------------------------------------------------------------
    // The row each tensor's pixel sits in -- the unit the prefetch policy
    // fetches, and the address a burst is issued at.
    // -----------------------------------------------------------------------
    function [31:0] res4_row;
        input [15:0] fn;
        input [15:0] fh;
        begin
            res4_row = RES4_BASE + (fn * RES4_S_N) + (fh * RES4_S_H);
        end
    endfunction

    function [31:0] ups0_row;
        input [15:0] fn;
        input [15:0] fh;
        begin
            ups0_row = UPS0_BASE + (fn * UPS0_S_N) + (fh * UPS0_S_H);
        end
    endfunction

    function [31:0] ups1_row;
        input [15:0] fn;
        input [15:0] fh;
        begin
            ups1_row = UPS1_BASE + (fn * UPS1_S_N) + (fh * UPS1_S_H);
        end
    endfunction

    function [31:0] tail_row;
        input [15:0] fn;
        input [15:0] fh;
        begin
            tail_row = TAIL_BASE + (fn * TAIL_S_N) + (fh * TAIL_S_H);
        end
    endfunction

    // -----------------------------------------------------------------------
    // Containment, one per tensor.  A reader uses this to know it is inside the
    // region it thinks it is in; the checker uses it to cross-check its own
    // interval arithmetic.
    // -----------------------------------------------------------------------
    function res4_in;
        input [31:0] a;
        begin
            res4_in = (a >= RES4_BASE) && (a < RES4_END);
        end
    endfunction

    function ups0_in;
        input [31:0] a;
        begin
            ups0_in = (a >= UPS0_BASE) && (a < UPS0_END);
        end
    endfunction

    function ups1_in;
        input [31:0] a;
        begin
            ups1_in = (a >= UPS1_BASE) && (a < UPS1_END);
        end
    endfunction

    function tail_in;
        input [31:0] a;
        begin
            tail_in = (a >= TAIL_BASE) && (a < TAIL_END);
        end
    endfunction

    // -----------------------------------------------------------------------
    // TAIL in words, for the tail engine's out_wr_addr[17:0].  Pixel word
    // index, not byte: the engine writes 32 bits at a time.
    // -----------------------------------------------------------------------
    function [17:0] tail_word;
        input [15:0] fn;
        input [15:0] fh;
        input [15:0] fw;
        begin
            tail_word = ((fn * TAIL_H) + fh) * TAIL_W + fw;
        end
    endfunction

    // -----------------------------------------------------------------------
    // The whole map's extent and its used bytes.  Four terms, so no store and
    // no sorting -- two levels of a two-input min/max, which is a function
    // because Verilog-2001 has no reduction for it and a localparam because
    // the answer is a constant.
    //
    // The max is taken at 32 bits and is safe there: every end address is
    // below 0x0100_0000, which is a quarter of the controller's window and
    // nowhere near wrapping.  A map whose regions could reach the top of the
    // address space would need the extra bit; this one cannot, and adding a
    // wider temporary that no configuration can exercise is a wider temporary
    // nobody has tested.
    // -----------------------------------------------------------------------
    function [31:0] min2;
        input [31:0] a;
        input [31:0] b;
        begin
            min2 = (a < b) ? a : b;
        end
    endfunction

    function [31:0] max2;
        input [31:0] a;
        input [31:0] b;
        begin
            max2 = (a > b) ? a : b;
        end
    endfunction

    localparam [31:0] MAP_LO = min2(min2(RES4_BASE, UPS0_BASE),
                                    min2(UPS1_BASE, TAIL_BASE));
    localparam [31:0] MAP_HI = max2(max2(RES4_END, UPS0_END),
                                    max2(UPS1_END, TAIL_END));

    // -----------------------------------------------------------------------
    // One combinational block: select, then read the numbers out of the
    // function that owns them.
    // -----------------------------------------------------------------------
    assign addr     = (tsel == T_RES4) ? res4_addr(t_n, t_c, t_h, t_w) :
                      (tsel == T_UPS0) ? ups0_addr(t_n, t_c, t_h, t_w) :
                      (tsel == T_UPS1) ? ups1_addr(t_n, t_c, t_h, t_w) :
                                         tail_addr(t_n, t_c, t_h, t_w);

    assign row_base = (tsel == T_RES4) ? res4_row(t_n, t_h) :
                      (tsel == T_UPS0) ? ups0_row(t_n, t_h) :
                      (tsel == T_UPS1) ? ups1_row(t_n, t_h) :
                                         tail_row(t_n, t_h);

    assign sel_base = (tsel == T_RES4) ? RES4_BASE :
                      (tsel == T_UPS0) ? UPS0_BASE :
                      (tsel == T_UPS1) ? UPS1_BASE :
                                         TAIL_BASE;

    assign sel_size = (tsel == T_RES4) ? RES4_SIZE :
                      (tsel == T_UPS0) ? UPS0_SIZE :
                      (tsel == T_UPS1) ? UPS1_SIZE :
                                         TAIL_SIZE;

    assign sel_s_c  = (tsel == T_RES4) ? RES4_S_C :
                      (tsel == T_UPS0) ? UPS0_S_C :
                      (tsel == T_UPS1) ? UPS1_S_C :
                                         TAIL_S_C;

    assign sel_s_w  = (tsel == T_RES4) ? RES4_S_W :
                      (tsel == T_UPS0) ? UPS0_S_W :
                      (tsel == T_UPS1) ? UPS1_S_W :
                                         TAIL_S_W;

    assign sel_s_h  = (tsel == T_RES4) ? RES4_S_H :
                      (tsel == T_UPS0) ? UPS0_S_H :
                      (tsel == T_UPS1) ? UPS1_S_H :
                                         TAIL_S_H;

    assign sel_s_n  = (tsel == T_RES4) ? RES4_S_N :
                      (tsel == T_UPS0) ? UPS0_S_N :
                      (tsel == T_UPS1) ? UPS1_S_N :
                                         TAIL_S_N;

    assign sel_n    = (tsel == T_RES4) ? RES4_N :
                      (tsel == T_UPS0) ? UPS0_N :
                      (tsel == T_UPS1) ? UPS1_N :
                                         TAIL_N;

    assign sel_c    = (tsel == T_RES4) ? RES4_C :
                      (tsel == T_UPS0) ? UPS0_C :
                      (tsel == T_UPS1) ? UPS1_C :
                                         TAIL_C;

    assign sel_h    = (tsel == T_RES4) ? RES4_H :
                      (tsel == T_UPS0) ? UPS0_H :
                      (tsel == T_UPS1) ? UPS1_H :
                                         TAIL_H;

    assign sel_w    = (tsel == T_RES4) ? RES4_W :
                      (tsel == T_UPS0) ? UPS0_W :
                      (tsel == T_UPS1) ? UPS1_W :
                                         TAIL_W;

    assign sel_row_b = (tsel == T_RES4) ? RES4_ROW_B :
                       (tsel == T_UPS0) ? UPS0_ROW_B :
                       (tsel == T_UPS1) ? UPS1_ROW_B :
                                          TAIL_ROW_B;

    assign sel_rows  = (tsel == T_RES4) ? (RES4_N * RES4_H) :
                       (tsel == T_UPS0) ? (UPS0_N * UPS0_H) :
                       (tsel == T_UPS1) ? (UPS1_N * UPS1_H) :
                                          (TAIL_N * TAIL_H);

    assign sel_row_bursts = sel_row_b / BURST_B;

    // The precondition the burst policy rests on: a row is a whole number of
    // bursts, so no row is ever split across two fetches and no fetch ever
    // carries part of a row it is not there for.
    assign sel_row_whole  = ((sel_row_b % BURST_B) == 0) &&
                            (sel_row_b >= BURST_B);

    assign sel_bursts = sel_size / BURST_B;

    assign sel_cache_b = CACHE_ROWS * sel_row_b;

    assign sel_in_region = (tsel == T_RES4) ? res4_in(addr) :
                           (tsel == T_UPS0) ? ups0_in(addr) :
                           (tsel == T_UPS1) ? ups1_in(addr) :
                                              tail_in(addr);

    assign tail_word_addr = tail_word(t_n, t_h, t_w);

    assign burst_b    = BURST_B;
    assign cache_rows = CACHE_ROWS;

    assign map_lo     = MAP_LO;
    assign map_hi     = MAP_HI;
    assign map_span   = MAP_HI - MAP_LO;
    assign map_used   = RES4_SIZE + UPS0_SIZE + UPS1_SIZE + TAIL_SIZE;

endmodule
