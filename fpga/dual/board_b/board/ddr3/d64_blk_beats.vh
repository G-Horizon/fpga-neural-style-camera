// board/ddr3/d64_blk_beats.vh -- the residual block's beat count, derived once.
//
// F09, and the measurement is `docs/TRUNK_BANDING_2026-10-07.md` (section 3.19),
// the contract is `docs/F3_INTERFACES_2026-10-07.md` 6.5.
//
// `d64_trunk_row_ddr` raises `wr_done_tog` when `wblk == BLK_BEATS[14:0]`, and
// `BLK_BEATS` was the literal 16384 with the comment "64 rows" -- a parameter
// that **nothing in the hierarchy drove**, with the board top instantiating the
// adapter without overriding it.  `d64_trunk_block_crc` carried the same 16384
// in both of its instances.  A block at the rectangular shape is 36 rows =
// 9,216 beats, so:
//
//     64 rows, 16384 beats, adapter at its default:  wr_done_tog edges 1
//     36 rows,  9216 beats, adapter at its default:  wr_done_tog edges 0
//     36 rows,  9216 beats, BLK_BEATS told 9216:     wr_done_tog edges 1
//
// All 9,216 beats land and the read path is fine -- it is the *completion* that
// never happens.  The toggle never rises, `d64_trunk_blocks`'s barrier never
// completes, the trunk never leaves block 0, and `blk_end_o` never fires so the
// CRC accumulators never reset.
//
// A residual block is ROWS rows of 256 128-bit beats.  The 256 is not a choice:
// a row is 4 KiB and a beat is 16 bytes, and the DDR's burst is what made the
// trunk's row 256 beats in the first place (`d64_trunk_row_ddr.v`'s header).
// The ROWS is the shape.  Everything that needs the count takes it from here,
// so 16384 in one place and 9216 in another is not a shape this tree can be in.
//
//     board/ddr3/d64_trunk_row_ddr.v      `ROWS` in, the count derived
//     board/ddr3/d64_trunk_block_crc.v    the same
//     board/ddr3/d64_encoder_trunk_board_top.v
//                                         `parameter ROWS` is the one shape,
//                                         and `localparam BLK_BEATS` its
//                                         derivation, passed to the adapter,
//                                         to both CRC blocks and to this top's
//                                         own pass counters
//     board/ddr3/read_d64_encoder_trunk_board.py
//                                         reads `D64_ROW_BEATS` out of this file
//                                         and takes `--rows` (default 64)
//
// The two `16384` sites a sabotaged build pins back to the literal, so the
// divergence is demonstrated rather than asserted, are `D64_BLK_TOOTH_ADAPTER`
// and `D64_BLK_TOOTH_CRC` in the board top and in
// `tb_d64_blk_beats_path.v` (see `tools/run_blk_beats_path.sh`).

`ifndef D64_BLK_BEATS_VH
`define D64_BLK_BEATS_VH

// 4 KiB a row, 16 bytes a beat.
`define D64_ROW_BEATS 256

// **The one derivation.**  A caller with a shape writes `D64_ROWS_TO_BEATS(ROWS)`
// and does not write the number.
`define D64_ROWS_TO_BEATS(r) ((r) * `D64_ROW_BEATS)

`endif
