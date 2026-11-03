`timescale 1ns/1ps

// ============================================================================
// d64_rgbx_display_adapter -- the one place the TAIL's byte order becomes the
// display's.
// ============================================================================
//
// The fault this module closes (root-project review F04,
// `E:\Ti60F225\fast_style_fpga\docs\root_project_2026-10-07\REVIEW_FINDINGS.md`)
// ---------------------------------------------------------------------------
//
// The TAIL packs a pixel as `{8'h00, B, G, R}` -- `control/d64_tail_engine.v`'s
// `out_wr_data`, and the same layout its integer oracle (`tools/d64_tail_ref.py`)
// computes and its own gate compares positionally.  The writer carries those
// four bytes into DDR verbatim and the display path carries them back out
// verbatim:
//
//   d64_rgbx_beat_writer.v        packs the 32-bit word into a 128-bit beat,
//                                 byte for byte, at `TAIL_BASE + pixel*4`
//   d64_video_row_ddr.v:496-497   lane0 = wq[23:0] ... load_wd <= pxw
//   d64_video_scaler.v:251,256    .wd(load_wd) into the row bank
//   d64_video_scaler.v:294,304    pix = ... ; rgb_o <= pix
//   board/preview/preview_top.v:466
//                                 .blue_din (s_rgb[ 7:0])
//                                 .green_din(s_rgb[15:8])
//                                 .red_din  (s_rgb[23:16])
//
// The display therefore reads the **top** byte of the 24-bit payload as red and
// the **bottom** byte as blue, and the TAIL wrote them the other way round.
// Wired straight through, a pixel the TAIL computes as pure red (its R byte,
// bits `[7:0]`, all ones) arrives at the TMDS encoder as `8'hFF` on `blue_din`
// and `8'h00` on `red_din`: **pure red displays as blue.**  The two channels
// that exchange are the two a viewer reads as "warm" and "cold", so a grey ramp
// looks perfect, a skin tone looks wrong and a red/blue picture looks
// plausible -- which is why this survived every per-module gate in the tree.
//
// Where the fix goes, and why it is here and in neither end
// ---------------------------------------------------------
//
// **Neither end is wrong, so neither end may be edited.**  The TAIL's layout is
// the TAIL's contract: its Python oracle, the beat writer's 262144-byte
// positional comparison and its `SWAP_BYTES_SABOTAGE` tooth are all built
// against it, and the review is explicit that the TAIL and its oracle are kept
// as they are.  The display's layout is the display's contract: `rgb_o` is
// `0x00RRGGBB`, `dvi_encoder`'s `red_din` is its byte `[23:16]`, and every
// other source on this board (`d64_video_test_src`, `rgbx_frame_writer`) already
// produces exactly that.  A byte order is a property of an *interface*; two
// interfaces that disagree do not make one of them wrong, and repairing either
// one alone would break the gates that measure it.
//
// So the conversion belongs **at the boundary, once, in one module**, and this
// is that module.  Two alternatives are worse, and they are named here so that
// a later hand does not reach for them:
//
//   * **in the reader** (`d64_video_row_ddr`).  It is the first place the TAIL's
//     bytes reappear after DDR, so it looks like the natural home -- but it is
//     not the only consumer of those bytes.  `d64_rgbx_beat_writer`'s own gate
//     compares the image **in DDR** positionally against the TAIL's oracle, so a
//     swap in the reader would fix the display and leave every other consumer to
//     re-derive what the region holds.
//   * **in the scaler or at the TMDS pins.**  That is the same edit made in
//     three files across two clock domains, where a later change can add a
//     second copy or lose the first.  **Two swaps cancel**, and a cancelling
//     pair looks exactly like no swap at all -- on every pattern, including the
//     asymmetric ones -- so it is the worse of the two failures, not the better.
//
// One module, one place, and the question "where is this byte order written?"
// has exactly one answer that `grep` can find.
// (`tools/run_rgb_display_chain.sh` asks that question of the whole tree and
// requires the answer to be this file, once.)
//
// What it is: a permutation, and nothing else
// -------------------------------------------
//
// Three `assign`s.  No clock, no reset, no state, no parameter, no generate:
// **there is nothing here to configure, so the swap cannot be switched off in a
// bitstream by a wrong instantiation.**  `load_we` and `load_wi` pass through
// untouched.  They are on the port list because the boundary this module draws
// is the whole of the loader's pixel port and not only its datum: a module
// carrying the datum alone would leave whoever joins the reader to the scaler to
// route the other two wires *around* it, and a boundary that can be routed
// around is a boundary that eventually will be -- with the swap on one side of
// the detour and the pixels on the other.
//
//     load_wd_o = { load_wd_i[7:0], load_wd_i[15:8], load_wd_i[23:16] }
//
// The green byte does not move, and that is not an accident of the layout: green
// is the one channel both ends already agree on, and the permutation is exactly
// the exchange of the two the fault exchanges.  A test that compares the *set*
// of bytes a pixel carries, or a checksum computed in one layout, cannot see
// this module at all -- a permutation of two equal-weight channels is invisible
// to any such test.  That is why `tb_d64_rgb_display_chain.v` compares the
// colour **channel by channel** and why its pattern is **asymmetric** (a pattern
// with R equal to B passes whichever way the bytes are ordered, so a symmetric
// pattern proves nothing).
//
// Verified, and not
// -----------------
//
// Verified by `tools/run_rgb_display_chain.sh`, every active pixel of a settled
// frame: pure red, pure green, pure blue and four all-distinct markers written
// through the **real** `d64_rgbx_beat_writer` into a DDR model, read back
// through the **real** `d64_video_row_ddr`, through this module, through the
// **real** `d64_video_scaler`, and compared at the **real** `dvi_encoder`'s
// `red_din`/`green_din`/`blue_din` pins.  The joint arm also carries the TAIL
// engine's own 65536-pixel frame the same way.  The swap removed from a scratch
// copy of this file is required to make the joint arm fail.  See
// `docs/RGB_BYTE_ORDER_2026-10-07.md`.
//
// **Not verified:** no synthesis, no place-and-route, no board, no DDR
// controller (the DDR is that run's own model), no monitor.  The monitor's own
// red/blue wiring is copied from `preview_top.v:466` and checked against that
// file by the runner rather than assumed.  Nothing here is a claim about the
// full network: F3 section 4 item 5.
// ============================================================================
module d64_rgbx_display_adapter (
    // ---- the loader's pixel port, as DDR drove it: {8'h00, B, G, R} ------
    input  wire        load_we_i,
    input  wire [7:0]  load_wi_i,
    input  wire [23:0] load_wd_i,

    // ---- the same port, as the display consumes it: {8'h00, R, G, B} ----
    output wire        load_we_o,
    output wire [7:0]  load_wi_o,
    output wire [23:0] load_wd_o
);

    // The write enable and the column index are the port's other two wires.
    // Neither is byte-ordered, so neither is touched.
    assign load_we_o = load_we_i;
    assign load_wi_o = load_wi_i;

    // **The swap.  It is the whole module.**  Byte 0 -- `[7:0]`, the TAIL's red,
    // which the display would read as blue -- becomes byte 2, where the display
    // reads red; byte 2 becomes byte 0; and the green byte in the middle stays
    // where it is, because both ends already agree about it.
    assign load_wd_o = {load_wd_i[7:0], load_wd_i[15:8], load_wd_i[23:16]};

endmodule
