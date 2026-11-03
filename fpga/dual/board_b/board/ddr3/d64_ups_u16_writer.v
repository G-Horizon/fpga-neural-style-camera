`timescale 1ns/1ps

// ============================================================================
// d64_ups_u16_writer -- the decoder's two intermediates, into DDR as U16 images
// ============================================================================
//
// `docs/F3_INTERFACES_2026-10-07.md` section 6.10.  **The module did not
// exist.**  `d64_tconv_engine` emits `out_wr_en` / `out_wr_addr` /
// `out_wr_row_abs` / `out_wr_data`, both decoder intermediates are written to
// DDR by nothing, and the transport is settled (`afa71bd`): a **U16 image** --
// a channel in a 16-bit container, so eight channels are 8 x 16 = **128 bits =
// exactly one AXI beat**.
//
//     UPS0  base 0x0044_0000   524288 B   128 x 128 x 16
//     UPS1  base 0x004C_0000  1048576 B   256 x 256 x 8
//
// **The values stay U8.**  The container widens and the value does not, so each
// 16-bit lane is `{8'b0, value}` -- **zero-extended, not sign-extended**,
// because the decoder's activations are unsigned and the reader slices the
// *bottom* bits of the container (`d64_burst_prefetch.v:452`, `DDR_LANE_W = 16`
// with `IN_W = 8`).  A sign-extended lane reads back there as a value with its
// high byte set, which is a wrong activation that no address check can see.
//
// One word in, one beat out -- and there is no packing step at all
// ----------------------------------------------------------------
// `d64_tconv_engine` emits **one 64-bit word** a write: eight channels of eight
// bits, which is one output-channel group (`WORD_CH = 8`, `out_wr_data` is
// `[WORD_CH*OUT_W-1:0]`).  The transport containers that group as 8 x 16 = 128
// bits, so **one engine word is exactly one AXI beat** and this module's whole
// payload path is one expression:
//
//     beat[16*i +: 16] = { 8'b0, out_wr_data[8*i +: 8] }        i = 0..7
//
// There is no accumulator, no lane select, no partial beat and no straddle
// between words -- which is the same statement `d64_rgbx_beat_writer.v` makes
// one level down ("four pixels ARE one beat") and `d64_h0_block.v:17` makes
// about an imaginary 80-to-128 one ("the 80/128 straddle does not exist").  It
// is said here, first and at length, because this project has already spent a
// day on a repacker that had nothing to do.
//
// Where the beat goes: two numbers, added and shifted
// ----------------------------------------------------
// The engine gives the beat's place in the image as **two fields**, not one:
//
//   `out_wr_row_abs[8:0]`   the absolute output row, 2*r_in + dy
//   `out_wr_addr[15:0]`     the word's offset **within that row**
//
// (`d64_tconv_scheduler.v:240` forms the whole address as
// `out_row * OUT_W * OC_WORDS + out_word`; the engine splits the row out into
// `mf_rowabs` and hands the rest on `out_wr_addr`, `control/d64_tconv_engine.v:645-653`.)
// So the word's index in the tensor is
//
//     word    = out_wr_row_abs * ROW_WORDS + out_wr_addr
//     beat    = region_base   + word * 16
//
// and that is the whole of the address arithmetic.  `ROW_WORDS` is a
// parameter with the **map's** value behind it: `sel_row_b / 16` bytes to the
// row, which the testbench requires rather than assumes.
//
// This reproduces the map's own layout exactly, and it is worth writing down
// why, because the two look different.  `d64_net_addr`'s tensors are HWC --
// `addr(n,c,h,w) = BASE + n*S_N + h*S_H + w*S_W + c*S_C`, `S_C = ELEM_B` -- so
// for UPS1 one pixel is `C*2` = 16 contiguous bytes = one beat and the pixel
// column *is* the word.  For UPS0 one pixel is 32 bytes and holds **two** words
// (channels 0..7 then 8..15), so the word is `2*col + group`.  The engine forms
// exactly those two cases: `m_word = ((m_pix << 1) + odx) * OC_WORDS
// + (mp % OC_WORDS)` with `OC_WORDS = 2` at ups.0 and `1` at ups.1
// (`control/d64_tconv_engine.v:645-648`).  Both are `word = row*ROW_WORDS +
// out_wr_addr`, measured against the map rather than argued about -- the
// testbench builds its reference from the map's own `sel_s_c` / `sel_s_w` /
// `sel_s_h` strides and compares every byte.
//
// **UPS1 arrives out of order, and that is the main case**
// -------------------------------------------------------
// The transposed convolution runs four phases an input pixel and drains each
// phase's words during the *next* phase's slots.  At ups.1 the scheduler reads
// **two** adjacent input pixels a slot (`PIX_PAR = 2`) and `OC_WORDS = 1`, so
// `out_pix = out_c + out_idx` and the phase's two words are `2c + 2*idx + odx`:
//
//   phase EE (odx = 0) -> 2c, 2c+2      phase EO (odx = 1) -> 2c+1, 2c+3
//
// so a row's words arrive **0, 2, 1, 3, 4, 6, 5, 7, ...**, and the pair of
// output rows a phase feeds interleave four words at a time.  At ups.0
// (`PIX_PAR = 1`, `OC_WORDS = 2`) the same expression yields `4c, 4c+1` then
// `4c+2, 4c+3`, which **is** ascending -- so a writer built and gated at UPS0
// never meets the case, and that is why it is the arm this gate leads with.
//
// The writer is order-blind by construction and not by luck: **the slot a word
// occupies comes from the word's own address and not from an arrival count.**
// That is the same decision `d64_rgbx_beat_writer.v` records one level down
// ("the lane comes from the address and not from an arrival count"), made one
// level up: here it is the *panel slot* that comes from the address.  A word
// arriving early, late, or in a different order than its neighbours lands in
// its own 16-byte slot of the panel and in no other, and a word that arrives
// **twice** lands in an already-occupied slot and is counted
// (`duplicate_words`) instead of silently overwriting.
//
// Why a panel and not a FIFO, and why the burst length is the flow control
// -----------------------------------------------------------------------
// A FIFO in arrival order cannot be bursted: AXI fixes `awlen` at the address
// handshake, so before the AW is presented the writer must already know that
// the next `awlen+1` beats exist and are consecutive.  With an out-of-order
// producer that is only knowable if the writer holds the words in the slots
// they belong to -- so the buffer **is** the address space of one AXI burst:
//
//   a panel is `PANEL_BEATS` consecutive beats held in slot order, with a
//   present bitmap; it is issued when the bitmap is full, and its AW is the
//   panel's own base.
//
// `PANEL_BEATS` is **64 beats = 1024 B**, and it is a measured choice rather
// than a taste: see the flow-control section below, where the same 8 KiB of
// buffer is benchmarked at two burst lengths and the longer one survives half
// as much bus wait.  A row is 4096 B, so a row is exactly **four** bursts; a
// 1024-byte burst at a 1024-byte-aligned address **cannot cross a 4 KiB
// boundary** (an AXI INCR burst may not, F3 section 6.11), and the testbench
// checks both the alignment and the span.  The map's own prefetch unit is
// `burst_b` = 2048 B = 128 beats, twice this; the run reports the two numbers
// side by side rather than pretending they are the same.
//
// **The buffer depth is the flow control, and F3 section 6.13 is why.**  The
// engine has no stall input -- its port list is `out_wr_en`/`out_wr_addr`/
// `out_wr_data` and nothing else -- so this module cannot ask it to wait and
// does not invent a signal the engine would not answer.  What covers a bus wait
// is therefore the number of beats the writer can hold:
//
//     NBUF * PANEL_BEATS  beats   =  8 * 64 = 512 beats = 8 KiB = 65536 bits
//
// and the wait it covers is that divided by the engine's own rate -- for ups.1
// eight words an input pixel over eighteen slots, **0.444 beats a cycle**.  The
// measured figure is **about 700 cycles** of bus that has stopped moving, and
// it is *not* 512/0.444 = 1,153, because the buffer is not free to hold all
// 512 beats at a panel boundary: two panels are being filled (one per row of
// the pair, which fill in lockstep), one is stuck under the stalled bus, and
// only the rest is slack.  That overhead scales with `PANEL_BEATS`, and that is
// the whole reason for the burst length above:
//
//   | buffer              | burst     | a bus frozen 500 cycles |
//   |---------------------|-----------|-------------------------|
//   | 8 x 64 beats, 8 KiB | 1024 B    | **no word lost**        |
//   | 4 x 128 beats, 8 KiB| 2048 B    | 128 words lost          |
//   | 8 x 16 beats, 2 KiB | 256 B     | 128 words lost          |
//
// The first two rows are the same 8 KiB and the last is a quarter of it, and
// all three are gate arms.  **The burst length and the flow control are one
// decision**, which is F3 section 6.13's note put to a measurement instead of
// an agreement: 8 KiB holds for 700 cycles at a 1024-byte burst and only 300 at
// a 2048-byte one, so the shipping burst is the short one.
//
// The wait is finite and it is meant to be read that way: a bus that never
// returns is not a property the buffer can fix, and `panel_overflows` is the
// counter that says so.  The cost is paid in block RAM or in registers --
// `docs/RAM_BUDGET_2026-10-07.md` section 2 records that this design's arrays
// have been mapping to **registered arrays**, which is the worst of the three,
// and 8 KiB as registers is 65,536 flip-flops against the device's 60,800.  It
// is a real cost, it is not measured here, and it is listed as such below.
//
// `panel_overflows` is not decoration.  An overflow is a **lost word**: the
// engine cannot be asked to repeat it, so this is the number a run is failed on
// rather than a number to be explained away.
//
// The region comes from an instantiated `d64_net_addr`, never retyped
// ------------------------------------------------------------------
// `b74c9d4` is why.  A hardcoded `TAIL_BASE` and the map drifted apart, both
// sides of one check read the same stale constant, and the gate stayed green
// while measuring a region the map no longer had -- for two commits.
// `d64_net_addr` is a pure combinational map with no clock and no state, so
// this module **instantiates it** and takes `sel_base` (and, as outputs,
// `sel_size` / `sel_row_b` / `burst_b`) straight out of it.  There is no
// literal here for a base to drift from: `TSEL` picks the tensor by the map's
// own encoding (`2'd1` = UPS0, `2'd2` = UPS1) and everything else follows.
//
// Teeth
// -----
// Two builds, each required to fail, and each aimed at a different claim:
//
//   `SWAP_CH_SABOTAGE`     channels 0 and 1 of the assembled beat exchange
//                          places -- the two 16-bit containers swap, every
//                          address and every count stays right, and only the
//                          byte *position* moves.  This is the tooth that says
//                          the byte enumeration is load-bearing.
//   `DUP_GROUP_SABOTAGE`   a panel's burst is issued **twice**.  The addresses
//                          are all in the region and the data at each is
//                          right, so this one is *not* caught by the byte
//                          enumeration: it is caught by the slot enumeration
//                          ("written more than once"), which is the other half
//                          of "no gap and no duplicate".
//
// What is verified, and what is not
// ---------------------------------
// Verified by `tools/run_ups_u16_writer.sh` and nothing else: every byte of
// both tensors enumerated against a reference built from the same source, at
// UPS1's own out-of-order arrival, under a bus that stalls and a bus that
// bubbles, with the region read out of an instantiated `d64_net_addr`, and the
// two sabotages above each failing on a different check.  See
// `docs/UPS_U16_WRITER_2026-10-07.md` for the counts.
//
// **Not verified:** anything about the board.  No synthesis, no place-and-route,
// no resource figure and no timing figure; `NBUF * PANEL_BEATS` beats of buffer
// is a behavioural array here and a block RAM count there, and nothing in this
// file has measured one.  No DDR controller: the testbench's slave model does
// not have a real controller's BRESP behaviour or its AW/W/B latency.  No
// reader: `d64_burst_prefetch` is the consumer this container width was chosen
// for, and it is not wired to this module.  And **no claim about the full
// network** -- F3 section 4 item 5: these are pieces.
// ============================================================================
module d64_ups_u16_writer #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter integer RESERVE_PHASE = 0,
    // The tensor's region, from the instantiated map.  `2'd1` is the map's
    // `T_UPS0` and `2'd2` its `T_UPS1`; the map's own header lists the
    // encoding.  Everything else -- base, size, row bytes, burst unit -- is the
    // map's output and not a number here.
    parameter [1:0] TSEL          = 2'd2,
    parameter integer DYNAMIC_REGION = 0,

    // Words (beats) in one output row.  **256 for both tensors**, and it is
    // worth seeing why: UPS0's row is 128 pixels x 16 channels x 2 B = 4096 B,
    // and UPS1's is 256 x 8 x 2 = 4096 B, so the two layers are the two ends of
    // the same 4096-byte row -- 256 beats of sixteen bytes each.  A parameter
    // because a port cannot see a localparam, and checked by the testbench
    // against the map's `sel_row_b / 16` rather than trusted.
    parameter integer ROW_WORDS   = 256,

    // Beats in one AXI INCR burst, i.e. the panel: 64 beats = 1024 B.  **Not a
    // tuning knob in this module** -- it is also the number of beats one buffer
    // holds, the two are one decision (F3 6.13), and the module header carries
    // the measurement that picked it.  The testbench requires it to divide the
    // row and not to exceed the map's `burst_b`, and reports the two side by
    // side.
    parameter integer PANEL_BEATS = 64,

    // Panel buffers.  **Three is the structural minimum**: one panel under the
    // bus, one being filled for each row of the pair (the rows fill in
    // lockstep), so a fourth is the first one that is slack.  Everything above
    // that is tolerance of a stalled bus, one panel buffer each -- and since
    // the minimum is *panels* and the slack is what is left of the bytes, a
    // smaller panel leaves more slack for the same memory.  That is the trade
    // the module header measures.
    parameter integer NBUF        = 8,

    // The engine's word: eight channels of eight bits.
    parameter integer WORD_CH     = 8,
    parameter integer VALUE_W     = 8,
    // Bits in one DDR transport container.  The U16 image's 2 bytes; the value
    // stays `VALUE_W` bits and is **zero-extended** into this.
    parameter integer DDR_LANE_W  = 16,

    // ---- verification only, folded away at 0 ------------------------------
    parameter integer SWAP_CH_SABOTAGE   = 0,
    parameter integer DUP_GROUP_SABOTAGE = 0,

    // **The read pipeline's tooth.**  At 0 the payload read is issued one beat
    // ahead of the beat it feeds, which is what absorbs the block RAM's one
    // cycle of read latency.  At 1 the address follows the beat counter
    // *directly* -- the scratch diagnostic's shape -- so the beat presented on
    // the bus is the one the read issued a cycle ago: with `bcnt` advancing,
    // that is the beat before, every beat is its predecessor, and the burst
    // drives `PANEL_BEATS` beats of which the last is beat `PANEL_BEATS-2`.
    // Every count and every address stays right and only the bytes move, so
    // the arm fails on the byte enumeration and not on a counter.
    parameter integer RDPIPE_SABOTAGE    = 0
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire [1:0]                  region_sel,

    // ---- the engine's output, wired straight in ---------------------------
    // The names are the engine's own, so a wiring mistake is a name that does
    // not resolve rather than a name that resolves to the wrong thing.
    //
    // `out_wr_row` -- the engine's scratch row bit -- is **not** on this port
    // list: the scratch is two rows deep and `out_wr_row_abs` is the absolute
    // row the word belongs to, of which `out_wr_row` is the low bit
    // (`control/d64_tconv_engine.v:608`, `mf_rowabs <= {b_r, 1'b0} + b_dy` and
    // `mf_row <= b_dy`).  The image address is the absolute one, so the scratch
    // bit is a producer-side fact this writer has no use for.
    input  wire                        out_wr_en,
    input  wire [15:0]                 out_wr_addr,
    input wire reserve_en,
    input wire [8:0] reserve_row,
    input wire [15:0] reserve_addr,
    output wire reserve_ready,
    input  wire [8:0]                  out_wr_row_abs,
    input  wire [WORD_CH*VALUE_W-1:0]  out_wr_data,

    // ---- AXI4 write master, 128-bit beats --------------------------------
    // Raw AXI4, which F3 section 3.2 fixes the board's DDR as and which
    // `d64_axi_read_burst.v` and `d64_axi_arb2.v` spell.  One INCR burst is one
    // panel: `awsize` 4, `awburst` INCR, `awlen` = `PANEL_BEATS - 1`,
    // `wstrb` all ones (every byte of every beat is real -- there is no partial
    // beat to strobe, which is "no packing step" said in the write mask's
    // language).
    output reg  [31:0]                 axi_awaddr,
    output reg  [7:0]                  axi_awlen,     // beats MINUS ONE, as AXI4
    output wire [2:0]                  axi_awsize,
    output wire [1:0]                  axi_awburst,
    output reg                         axi_awvalid,
    input  wire                        axi_awready,
    output wire                        axi_wvalid,
    input  wire                        axi_wready,
    output wire [WORD_CH*DDR_LANE_W-1:0] axi_wdata,
    output wire [WORD_CH*DDR_LANE_W/8-1:0] axi_wstrb,
    output wire                        axi_wlast,
    input  wire                        axi_bvalid,
    output wire                        axi_bready,
    input  wire [1:0]                  axi_bresp,

    // ---- evidence ---------------------------------------------------------
    // Words handed in, W beats handed out, INCR bursts issued.  `beats_out`
    // must be `words_in` and `bursts_out` must be `words_in / PANEL_BEATS` on
    // the clean arms; a short count is a dropped word.
    output reg  [31:0]                 words_in,
    output reg  [31:0]                 beats_out,
    output reg  [31:0]                 bursts_out,
    // A committed write response: one pulse and one count, the shape
    // `d64_done_tracker.v` consumes.  **A response retires its transaction
    // whether or not it carried an error** (F03, `d64_rgbx_beat_writer.v`'s
    // header has the argument), so `wr_ack` is "the memory answered" and
    // `bresp_errors` is "and this many answers were not OKAY".
    output reg                         wr_ack,
    output reg  [31:0]                 wr_acks,
    output reg  [31:0]                 bresp_errors,
    // A word whose offset is past the end of a row.  The engine cannot produce
    // one -- `out_wr_addr` is `m_word` and its top is the row's -- so this is a
    // check on the producer and it is free.
    output reg  [31:0]                 oob_words,
    // **A word that landed in a slot that already held one.**  A producer
    // contract violation and a lost word at the same time: whatever should have
    // gone in the other slot is not in the tensor.  Reported, never repaired.
    output reg  [31:0]                 duplicate_words,
    // **A word that could not be buffered, i.e. a lost write.**  The engine
    // cannot be asked to repeat it, so this is a number a run is failed on.  It
    // is what a bus wait longer than `NBUF * PANEL_BEATS` beats produces, and
    // what the sabotaged burst length produces at a wait the real build
    // survives.
    output reg  [31:0]                 panel_overflows,
    // Cycles with a burst open and the bus not moving: AW presented and not
    // taken, W presented and not taken, or a response owed and not given.  In
    // this direction that is the honest description of "the bus is waiting for
    // the engine" and is reported, not required to be zero.
    output reg  [31:0]                 stall_cycles,
    // Panels held and complete, waiting for the bus.  Zero at the end of a
    // tensor unless the bus never came back.
    output wire [7:0]                  panels_ready,

    // ---- the region, read out of the instantiated map ---------------------
    // Exposed so a testbench and an integrator read the same numbers the
    // address path uses rather than a copy of them.
    output wire [31:0]                 region_base,
    output wire [31:0]                 region_size,
    output wire [31:0]                 region_row_b,
    output wire [31:0]                 region_burst_b,
    output wire                        drained
);

    // -----------------------------------------------------------------------
    // Shape.  Every number here is a `localparam` and none is on the port list,
    // because a port list cannot see one -- the trap
    // `docs/F2_INTERFACES_2026-10-06.md` section 0 names.
    // -----------------------------------------------------------------------

    localparam integer BEAT_B      = WORD_CH * DDR_LANE_W / 8;   // 16 bytes
    localparam integer BEAT_W      = WORD_CH * DDR_LANE_W;       // 128 bits
    localparam integer NBUF_W      = (NBUF <= 1) ? 1 : $clog2(NBUF);
    localparam integer SLOT_W      = (PANEL_BEATS <= 1) ? 1 : $clog2(PANEL_BEATS);
    localparam integer CNT_W       = SLOT_W + 1;                 // holds PANEL_BEATS
    localparam integer ROWW_W      = 9;                          // `out_wr_row_abs`

    // The buffer array, as a memory.  `PBUF_N` beats is what `NBUF x
    // PANEL_BEATS` always was; `RA_W` is the width of an address into it and it
    // is `$clog2` of that and not a hand count, because a parameter change would
    // leave a hand count behind.
    localparam integer PBUF_N      = NBUF * PANEL_BEATS;
    localparam integer RA_W        = (PBUF_N <= 1) ? 1 : $clog2(PBUF_N);

    localparam [1:0] S_FREE  = 2'd0,
                     S_FILL  = 2'd1,
                     S_READY = 2'd2,
                     S_BUSY  = 2'd3;

    localparam [1:0] B_IDLE = 2'd0,
                     B_ADDR = 2'd1,
                     B_DATA = 2'd2,
                     B_RESP = 2'd3;

    // A row/panel pair no producer can present, so the first word of a tensor
    // is always an allocation and never a "continue" against a buffer that has
    // never been set up.
    localparam [ROWW_W-1:0] ROW_NONE = {ROWW_W{1'b1}};
    localparam [15:0]       PAN_NONE = {16{1'b1}};

    // -----------------------------------------------------------------------
    // The map, instantiated -- not a base written down here
    // -----------------------------------------------------------------------
    // `b74c9d4`: a hardcoded base and the map drifted, and both sides of the
    // one check that existed read the same stale constant.  This module has no
    // such constant to drift: the address is added to `sel_base`, and the
    // tensor is chosen by the map's own `tsel` encoding.
    wire [31:0] map_addr, map_row_base;
    wire [31:0] map_s_base, map_s_size;
    wire [31:0] map_s_sc, map_s_sw, map_s_sh, map_s_sn;
    wire [15:0] map_s_n, map_s_c, map_s_h, map_s_w;
    wire [31:0] map_s_rowb, map_s_rows, map_s_bursts, map_s_rowbursts;
    wire        map_s_rowwhole;
    wire [31:0] map_s_cacheb;
    wire        map_s_inregion;
    wire [17:0] map_tailword;
    wire [31:0] map_burstb, map_cacher;
    wire [31:0] map_lo, map_hi, map_span, map_used;

    d64_net_addr u_map (
        .tsel(DYNAMIC_REGION ? region_sel : TSEL),
        .t_n(16'd0), .t_c(16'd0), .t_h(16'd0), .t_w(16'd0),
        .addr(map_addr),
        .row_base(map_row_base),
        .sel_base(map_s_base),
        .sel_size(map_s_size),
        .sel_s_c(map_s_sc), .sel_s_w(map_s_sw),
        .sel_s_h(map_s_sh), .sel_s_n(map_s_sn),
        .sel_n(map_s_n), .sel_c(map_s_c),
        .sel_h(map_s_h), .sel_w(map_s_w),
        .sel_row_b(map_s_rowb), .sel_rows(map_s_rows),
        .sel_bursts(map_s_bursts), .sel_row_bursts(map_s_rowbursts),
        .sel_row_whole(map_s_rowwhole), .sel_cache_b(map_s_cacheb),
        .sel_in_region(map_s_inregion),
        .tail_word_addr(map_tailword),
        .burst_b(map_burstb), .cache_rows(map_cacher),
        .map_lo(map_lo), .map_hi(map_hi),
        .map_span(map_span), .map_used(map_used)
    );

    assign region_base    = map_s_base;
    assign region_size    = map_s_size;
    assign region_row_b   = map_s_rowb;
    assign region_burst_b = map_burstb;

    // -----------------------------------------------------------------------
    // The word, taken apart.  Nothing below reads `out_wr_data` or
    // `out_wr_addr` except these wires.
    // -----------------------------------------------------------------------

    // The engine's word, container by container.  **The container is wider than
    // the value and the value is zero-extended into it** -- on the AXI beat.
    // The buffer below does not hold that form; see the next paragraph.
    //
    // **The extension is not stored.**  The engine's word is `WORD_CH` values of
    // `VALUE_W` bits -- 64 for the shipped parameters -- and the DDR beat is
    // `WORD_CH` containers of `DDR_LANE_W` bits, 128, with each value
    // zero-extended into its container.  The extension is a function of the
    // stored word alone, so it is done at the wire that leaves and the buffer
    // holds half the bits for the same tensor: `PBUF_N x WORD_CH x VALUE_W`
    // instead of `PBUF_N x BEAT_W`.  Measured, this module as its own root,
    // Efinity 2026.1.132.4.5: **7 RAM10 before, 4 after**.
    //
    // Both widths are parameters, so the two cases are worth naming:
    // `DDR_LANE_W == VALUE_W` is no extension at all and this is a no-op;
    // `DDR_LANE_W > VALUE_W` is the shipped eight-into-sixteen.
    localparam integer NWORD_W = WORD_CH * VALUE_W;             // 64 bits

    // **`SWAP_CH_SABOTAGE` exchanges containers 0 and 1 of the DDR beat, and
    // that is the same as exchanging their *values* in the stored word** --
    // every container's high `DDR_LANE_W-VALUE_W` bits are zero, so which value
    // sits in a container is the whole of what a container's position means.
    // The tooth is therefore applied here, at `VALUE_W` granularity, and the
    // byte enumeration it has to fail sees exactly the bytes it saw before.
    wire [NWORD_W-1:0] lane_data = out_wr_data;
    wire [NWORD_W-1:0] beat_data =
        (SWAP_CH_SABOTAGE != 0)
        ? {lane_data[NWORD_W-1:2*VALUE_W], lane_data[VALUE_W-1:0],
           lane_data[2*VALUE_W-1:VALUE_W]}
        : lane_data;

    // The word's place in the tensor, and the byte the beat lands on.  Both
    // are functions of where the word *belongs*; nothing below is a function
    // of how many words happened to arrive before this one.
    wire [31:0] word_index = (out_wr_row_abs * ROW_WORDS) + out_wr_addr;
    wire [31:0] byte_off   = word_index * BEAT_B;

    // ... and in its panel.  The panel is the AXI burst: `PANEL_BEATS`
    // consecutive words, aligned inside the row.
    wire [15:0] slot      = out_wr_addr % PANEL_BEATS;
    wire [15:0] panel_row = out_wr_addr / PANEL_BEATS;

    // The panel's byte base, which is the burst's AW.
    wire [31:0] panel_base =
        map_s_base + ((out_wr_row_abs * ROW_WORDS + panel_row * PANEL_BEATS) * BEAT_B);

    // A word past the end of its row, or a beat past the end of the region.
    // The first is a check on the producer -- the engine cannot form one -- and
    // the second is what makes "every address is inside the region" a property
    // of this module rather than of the map it was handed.
    wire row_oob   = (out_wr_addr >= ROW_WORDS);
    wire panel_oob = (byte_off >= region_size);
    wire oob_now   = out_wr_en && (row_oob || panel_oob);

    // -----------------------------------------------------------------------
    // State.  Every `reg` below is written from exactly one `always` block --
    // the arrays from the fill block, the scalars from the bus block -- which
    // is the property `tools/check_double_drivers.py` exists for and the
    // asymmetry (`vlog` accepts it, `efx_map` refuses) that cost this project
    // two board builds.
    // -----------------------------------------------------------------------

    // ---- the fill side ----------------------------------------------------
    // **`pbuf` is a block RAM, and it was not one.**  Mapped as its own root on
    // Efinity 2026.1.132.4.5 the array came back as
    // `Mapping into logic memory block 'pbuf' (65536 bits) ... because read port
    // is not synchronous` -- 26,243 LUT4 and 33,881 FF, against a Ti60F225's
    // 60,800 FF, for a 65,536-bit buffer.  Two instances of this module is
    // 67,762 FF and the device does not have them.
    //
    // Three things make it a memory now, and all three are the shape
    // `memory/d64_tc_ram.v` states and that `1a4c768` (F3-n) applied to the
    // burst cache's line RAM:
    //
    //   1. `ram_style = "block"`, which is an instruction and not a hint;
    //   2. a **synchronous** read, registered inside the memory's own clocked
    //      block.  The read is unconditional -- an enable on the address is
    //      what the mapper answers with logic memory -- and it lives in its own
    //      `always` block, which is not a choice: `pbuf` is written by the fill
    //      block and a second write to the same array from a second block is
    //      the double-driver shape `tools/check_double_drivers.py` refuses.
    //      Measured: the read in its own block infers, at 7 RAM10;
    //   3. the one cycle of read latency is **absorbed by issuing the address
    //      early** (`radr` below) and not by a mux in front of the output
    //      register.  See the bus block for what "early" means.
    (* ram_style = "block" *) reg [NWORD_W-1:0] pbuf [0:PBUF_N-1];
    reg [NWORD_W-1:0]        pbuf_q;             // the memory's own output register
    reg [PANEL_BEATS-1:0]   pbit  [0:NBUF-1];   // slot present
    reg [CNT_W-1:0]         pcnt  [0:NBUF-1];   // slots present, distinct
    reg [ROWW_W-1:0]        prow  [0:NBUF-1];   // the panel's row
    reg [15:0]              ppan  [0:NBUF-1];   // its panel index in the row
    reg [31:0]              pbase [0:NBUF-1];   // its byte base
    reg [1:0]               pst   [0:NBUF-1];
    reg                     pact  [0:NBUF-1];   // the buffer holds a live panel
    reg                     pdup  [0:NBUF-1];   // the duplicate tooth's flag
    // **The abandoned panel, one per output-row parity.**  When a word cannot
    // be buffered, the panel it belonged to is already short of the words
    // before it and the engine cannot be asked to produce any of them again --
    // so that panel is written off, and its *remaining* words are refused too
    // and counted in the same place.  Without this the buffer pool would take
    // the panel's later words, fill a bitmap that can never complete, and hold
    // that buffer until the end of the tensor: one lost word would take the
    // rest of the row with it, and the overflow count would report a cascade
    // instead of a loss.  The gate's `stall` arm found exactly that -- 48,498
    // overflows where the writer holds 512 beats.
    //
    // One entry a parity is enough because a row's panels are filled in order:
    // a panel's words are contiguous in arrival, so the first word that is not
    // one of them clears the record.  That is an assumption about the producer
    // and it is stated rather than hidden -- a producer that interleaved two
    // panels of one row would sometimes take a second loss where it could have
    // taken one.
    reg [ROWW_W-1:0]        drow  [0:1];
    reg [15:0]              dpan  [0:1];
    reg                     dv    [0:1];

    // ---- the bus side -----------------------------------------------------
    reg [1:0]               bst;
    reg [NBUF_W-1:0]        bcur;
    reg [SLOT_W-1:0]        bcnt;

    // **Which buffer holds this panel is a lookup and not an index.**  A panel
    // is found by what it *is* -- its row and its place in that row -- so a
    // word arriving out of order still finds the buffer its neighbours are in,
    // a word that arrives twice finds the same buffer and is counted there, and
    // an arriving panel takes whichever buffer is free.  Numbering buffers by
    // the panel's index (`panel % NBUF`) is the obvious alternative and it was
    // built first: the gate's `stall` arm measured it, and what it measured was
    // not the buffer's capacity but the *rotation* -- a buffer dedicated to one
    // panel index is blocked by any undrained panel of the same index, so an
    // 8 KiB buffer of four panels survived a freeze of **250 to 495 cycles
    // depending on the burst it began at**.  A pool of the same size and burst
    // gives about 400 at every burst, and making the figure a function of the
    // buffer and nothing else is what let the 8 KiB be turned into 700 cycles
    // by shortening the burst (the table above).  The pool is not a refinement;
    // it is what makes "NBUF x PANEL_BEATS beats is how much the writer holds"
    // a true sentence.
    //
    // The lookup is `NBUF` comparators over (row, panel) and a priority pick of
    // the free buffers.  It is combinational, it is not an arrival count, and
    // nothing about it is a round robin.
    reg [NBUF_W-1:0] bsel;
    reg              hit;
    reg              free_ok;
    integer          s2;
    always @* begin
        hit  = 1'b0;
        bsel = {NBUF_W{1'b0}};
        for (s2 = NBUF - 1; s2 >= 0; s2 = s2 - 1)
            if (pact[s2] && (prow[s2] == out_wr_row_abs) && (ppan[s2] == panel_row)) begin
                bsel = s2[NBUF_W-1:0];
                hit  = 1'b1;
            end
        free_ok = 1'b0;
        if (!hit)
            for (s2 = NBUF - 1; s2 >= 0; s2 = s2 - 1)
                if (!pact[s2]) begin
                    bsel    = s2[NBUF_W-1:0];
                    free_ok = 1'b1;
                end
    end

    // Panel identity is reserved before a phase can generate its outputs.
    // Empty reservations are ordinary S_FILL panels and consume the same pool.
    wire [15:0] reserve_panel=reserve_addr/PANEL_BEATS;
    wire [31:0] reserve_offset=(reserve_row*ROW_WORDS+reserve_panel*PANEL_BEATS)*BEAT_B;
    reg reserve_hit,reserve_free;
    reg [NBUF_W-1:0] reserve_pick;
    integer reserve_i;
    always @*begin
        reserve_hit=0;reserve_free=0;reserve_pick=0;
        for(reserve_i=NBUF-1;reserve_i>=0;reserve_i=reserve_i-1)
            if(pact[reserve_i] && prow[reserve_i]==reserve_row && ppan[reserve_i]==reserve_panel)begin
                reserve_hit=1;reserve_pick=reserve_i[NBUF_W-1:0];
            end
        if(!reserve_hit)for(reserve_i=NBUF-1;reserve_i>=0;reserve_i=reserve_i-1)
            if(!pact[reserve_i])begin reserve_free=1;reserve_pick=reserve_i[NBUF_W-1:0];end
    end
    assign reserve_ready=!RESERVE_PHASE ||
        ((reserve_addr<ROW_WORDS) && reserve_offset<region_size &&
         ((reserve_hit && pst[reserve_pick]==S_FILL) || reserve_free));
    wire reserve_new=RESERVE_PHASE && reserve_en && reserve_ready && !reserve_hit;

    // -----------------------------------------------------------------------
    // Which panel the bus takes, and when
    // -----------------------------------------------------------------------
    // A priority scan, and the reason it is a scan and not a counter: the
    // burst order is free.  A panel is written at its own base, so panels may
    // be issued in any order and the region is the same; what is *not* free is
    // opening a burst before its last word has arrived.
    reg [NBUF_W-1:0] pick;
    reg              found;
    reg        [7:0] ready_n;
    integer          s;
    always @* begin
        pick    = {NBUF_W{1'b0}};
        found   = 1'b0;
        ready_n = 8'd0;
        for (s = 0; s < NBUF; s = s + 1) begin
            if (pst[s] == S_READY) begin
                pick    = s[NBUF_W-1:0];
                found   = 1'b1;
                ready_n = ready_n + 8'd1;
            end
        end
    end

    assign panels_ready = ready_n;

    wire bitmap_dropped = dv[out_wr_row_abs[0]] &&
      drow[out_wr_row_abs[0]]==out_wr_row_abs && dpan[out_wr_row_abs[0]]==panel_row;
    wire bitmap_word=out_wr_en && !oob_now && !bitmap_dropped && (hit || (!RESERVE_PHASE && free_ok));
    genvar bitmap_panel,bitmap_slot;
    generate for(bitmap_panel=0;bitmap_panel<NBUF;bitmap_panel=bitmap_panel+1)begin:g_bitmap_panel
      for(bitmap_slot=0;bitmap_slot<PANEL_BEATS;bitmap_slot=bitmap_slot+1)begin:g_slot
        always @(posedge clk or negedge rst_n)begin
          if(!rst_n)pbit[bitmap_panel][bitmap_slot]<=1'b0;
          else pbit[bitmap_panel][bitmap_slot]<=
            (pbit[bitmap_panel][bitmap_slot] & !(bitmap_word && !hit && bsel==bitmap_panel) & !(reserve_new && reserve_pick==bitmap_panel)) |
            (bitmap_word && bsel==bitmap_panel && slot==bitmap_slot);
        end
      end
    end endgenerate

    // The fill side's view of the bus's two events.  `pick` and not `bcur` for
    // the take: `bcur` is registered on this same edge.
    wire bus_take   = (bst == B_IDLE) && found;
    wire bus_retire = (bst == B_RESP) && axi_bvalid;

    // -----------------------------------------------------------------------
    // The fill.  One `always` block owns every buffer array.
    // -----------------------------------------------------------------------
    integer ri;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            words_in        <= 32'd0;
            duplicate_words <= 32'd0;
            panel_overflows <= 32'd0;
            oob_words       <= 32'd0;
            for (ri = 0; ri < NBUF; ri = ri + 1) begin
                pst[ri]  <= S_FREE;

                pcnt[ri] <= {CNT_W{1'b0}};
                prow[ri] <= ROW_NONE;
                ppan[ri] <= PAN_NONE;
                pbase[ri]<= 32'd0;
                pdup[ri] <= 1'b0;
                pact[ri] <= 1'b0;
            end
            for (ri = 0; ri < 2; ri = ri + 1) begin
                drow[ri] <= ROW_NONE;
                dpan[ri] <= PAN_NONE;
                dv[ri]   <= 1'b0;
            end
        end
        else begin
            // ---- the bus's two events, seen from this side ---------------
            if (bus_take)
                pst[pick] <= S_BUSY;

            if (bus_retire) begin
                // **The duplicate tooth.**  The panel's burst is issued a
                // second time from the same buffer and the same base: the
                // buffer is kept, and its `pact` with it, so the pool offers it
                // again and the second burst goes to the same addresses.  A
                // freed buffer and a live-and-ready one would be two states
                // saying one thing, and the tooth would then be a build that
                // writes a *different* panel twice.
                if ((DUP_GROUP_SABOTAGE != 0) && !pdup[bcur]) begin
                    pdup[bcur] <= 1'b1;
                    pst[bcur]  <= S_READY;
                end
                else begin
                    pst[bcur]  <= S_FREE;
                    pact[bcur] <= 1'b0;
                end
            end

            if(reserve_new)begin
                pcnt[reserve_pick]<=0;
                prow[reserve_pick]<=reserve_row;
                ppan[reserve_pick]<=reserve_panel;
                pbase[reserve_pick]<=region_base+reserve_offset;
                pdup[reserve_pick]<=0;
                pact[reserve_pick]<=1;
                pst[reserve_pick]<=S_FILL;
            end

            // ---- the word --------------------------------------------------
            if (out_wr_en) begin
                words_in <= words_in + 32'd1;

                if (oob_now)
                    oob_words <= FAULT_STICKY ? 32'd1 : oob_words + 32'd1;
                else if (dv[out_wr_row_abs[0]]
                         && (drow[out_wr_row_abs[0]] == out_wr_row_abs)
                         && (dpan[out_wr_row_abs[0]] == panel_row)) begin
                    // A word of a panel already written off, counted with the
                    // word that wrote it off rather than apart from it.
                    panel_overflows <= FAULT_STICKY ? 32'd1 : panel_overflows + 32'd1;
                end
                else begin
                    // Any other word ends the record: the abandoned panel's
                    // words are contiguous, so this one is not one of them.
                    dv[out_wr_row_abs[0]] <= 1'b0;

                    if (hit) begin
                        // The buffer that already holds this panel.  A slot
                        // that is taken is a word arriving twice and is
                        // counted, never overwritten silently.
                        if (!pbit[bsel][slot]) begin

                            pcnt[bsel]       <= pcnt[bsel] + {{(CNT_W-1){1'b0}}, 1'b1};
                            if (pcnt[bsel] == PANEL_BEATS - 1) pst[bsel] <= S_READY;
                            pbuf[bsel*PANEL_BEATS + slot] <= beat_data;
                        end
                        else
                            duplicate_words <= FAULT_STICKY ? 32'd1 : duplicate_words + 32'd1;
                    end
                    else if (free_ok && !RESERVE_PHASE) begin
                        // A buffer for a panel nobody holds yet.


                        pcnt[bsel]       <= {{(CNT_W-1){1'b0}}, 1'b1};
                        prow[bsel]       <= out_wr_row_abs;
                        ppan[bsel]       <= panel_row;
                        pbase[bsel]      <= panel_base;
                        pdup[bsel]       <= 1'b0;
                        pact[bsel]       <= 1'b1;
                        pst[bsel]        <= (PANEL_BEATS == 1) ? S_READY : S_FILL;
                        pbuf[bsel*PANEL_BEATS + slot] <= beat_data;
                    end
                    else begin
                        // **The lost word.**  `NBUF` panels are held, none of
                        // them this one, and none of them can be written before
                        // the bus returns.  The engine cannot be asked to
                        // produce this word again, so this is a number a run is
                        // failed on and not a number to be explained.
                        panel_overflows <= FAULT_STICKY ? 32'd1 : panel_overflows + 32'd1;
                        dv[out_wr_row_abs[0]]   <= 1'b1;
                        drow[out_wr_row_abs[0]] <= out_wr_row_abs;
                        dpan[out_wr_row_abs[0]] <= panel_row;
                    end
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // The bus.  One burst at a time, one `always` block, and the payload off
    // the memory's **registered** read -- see the timing note below.  The
    // *combination* `rgbx_frame_writer`'s header warns about (a registered
    // `wdata`/`wlast` shipped the previous beat and dropped WLAST) is not what
    // this is: WLAST is still `bcnt`'s own comparison, and the register in front
    // of the payload is the memory's, whose address is chosen so that the beat
    // it holds is the beat this cycle's `bcnt` names.
    // -----------------------------------------------------------------------

    // Which beat the memory should be reading, and why it is not registered
    // -----------------------------------------------------------------------
    // `pbuf_q` is `pbuf[pbuf_ra]` sampled on the previous edge, so the beat the
    // bus presents in this cycle is the one whose address was on `pbuf_ra`
    // **last** cycle.  The address therefore has to be the beat *after* the one
    // on the bus, and the beat after is:
    //
    //     bcur*PANEL_BEATS + bcnt + (a beat is being taken this cycle)
    //
    // and not `bcnt`, and not `bcnt + 1`.  Every wrong version of this was
    // built and run before the right one, so the two ways to get it wrong are
    // worth naming:
    //
    //   * `bcur*PANEL_BEATS + bcnt`.  The address is one beat behind, so the
    //     bus is handed its own predecessor: the first beat of every burst is
    //     right by luck (it reads the panel's beat 0 for beat 0) and every
    //     beat after it is one late.  That is the scratch diagnostic's shape
    //     and it is what `RDPIPE_SABOTAGE` builds.
    //   * a **registered** counter that takes one step per accepted beat.  It
    //     looks right and it is one beat behind in a way no constant can fix:
    //     `pbuf_q(T) = pbuf[radr(T-1)]`, so the step taken in cycle T is not
    //     seen until T+2.  A register that is its own history cannot be asked
    //     what the *next* cycle's count will be, and the address needs exactly
    //     that.
    //
    // The expression above is a function of the **current** cycle only, and
    // that is the whole reason it can be right: `bcnt` is how many beats have
    // been taken, `axi_wready` this cycle says whether one more is about to be,
    // and the register in the memory samples the sum one cycle later.  It is
    // the same statement `d64_rgbx_beat_writer.v`'s `bq_ra` makes.
    //
    // `bcnt + (b_adv ? 1 : 0)` never reaches `PANEL_BEATS` -- the step is
    // suppressed on the last beat -- so `pbuf_ra` is inside
    // `[0, NBUF*PANEL_BEATS-1]` at every cycle, which is what lets the read be
    // unconditional.  And while `axi_wready` is low nothing is taken, the
    // address holds and `pbuf_q` holds the beat the burst has already declared
    // -- which is what the bus requires of a master that is stalled.
    wire b_adv = (bst == B_DATA) && axi_wready && (bcnt != PANEL_BEATS - 1);
    // The tooth: no step, so the memory keeps handing over the beat whose index
    // the bus is *already* presenting, and every beat of the burst is its own
    // predecessor.
    wire b_up  = b_adv && (RDPIPE_SABOTAGE == 0);
    wire [RA_W-1:0] pbuf_ra = ((bst == B_IDLE) && found)
                            ? (pick * PANEL_BEATS)
                            : (bcur * PANEL_BEATS + bcnt + (b_up ? 1'b1 : 1'b0));

    always @(posedge clk)
        // Unconditional, and in the memory's own clocked block: this is the
        // read the mapper needs to see as synchronous.  No reset on this
        // register -- `memory/d64_tc_ram.v` keeps its output register unreset
        // for the same reason, and it does not need one: `axi_wvalid` is low
        // until the first burst's first beat, and by then `pbuf_ra` has been a
        // real beat of the picked panel for two cycles.
        pbuf_q <= pbuf[pbuf_ra];

    assign axi_awsize  = 3'd4;              // 16 bytes: the whole 128-bit bus
    assign axi_awburst = 2'b01;             // INCR
    assign axi_wstrb   = {(WORD_CH*DDR_LANE_W/8){1'b1}};
    assign axi_wvalid  = (bst == B_DATA);
    // **The zero-extension, and the only place it happens.**  One value a
    // container, its own bits low and `DDR_LANE_W-VALUE_W` zeros above it, so
    // the beat on `axi_wdata` is bit for bit what it was when the buffer held
    // the extended form.
    genvar wi;
    generate
        for (wi = 0; wi < WORD_CH; wi = wi + 1) begin : g_wexp
            assign axi_wdata[wi*DDR_LANE_W +: DDR_LANE_W] =
                {{(DDR_LANE_W-VALUE_W){1'b0}}, pbuf_q[wi*VALUE_W +: VALUE_W]};
        end
    endgenerate
    assign axi_wlast   = (bcnt == PANEL_BEATS - 1);
    assign axi_bready  = (bst == B_RESP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bst          <= B_IDLE;
            bcur         <= {NBUF_W{1'b0}};
            bcnt         <= {SLOT_W{1'b0}};
            axi_awvalid  <= 1'b0;
            axi_awaddr   <= 32'd0;
            axi_awlen    <= 8'd0;
            beats_out    <= 32'd0;
            bursts_out   <= 32'd0;
            wr_ack       <= 1'b0;
            wr_acks      <= 32'd0;
            bresp_errors <= 32'd0;
            stall_cycles <= 32'd0;
        end
        else begin
            // One cycle wide, and it goes away unless a response lands.
            wr_ack <= 1'b0;

            case (bst)
                // **Not until there is a panel to write.**  `d64_trunk_row_ddr.v`
                // learned this on the board: an AW presented with nothing
                // behind it took the shared write channel and held it for the
                // whole run, and the first capture was all zeros with no
                // transaction issued.  Here "something behind it" is a panel
                // whose bitmap is full, which is the same statement made
                // addressable.
                B_IDLE: if (found) begin
                    bcur        <= pick;
                    axi_awaddr  <= pbase[pick];
                    axi_awlen   <= PANEL_BEATS - 1;
                    axi_awvalid <= 1'b1;
                    bcnt        <= {SLOT_W{1'b0}};
                    bst         <= B_ADDR;
                end

                B_ADDR: if (axi_awvalid && axi_awready) begin
                    axi_awvalid <= 1'b0;
                    bursts_out  <= bursts_out + 32'd1;
                    bst         <= B_DATA;
                end
                else
                    stall_cycles <= stall_cycles + 32'd1;

                B_DATA: if (axi_wready) begin
                    beats_out <= beats_out + 32'd1;
                    if (bcnt == PANEL_BEATS - 1) bst  <= B_RESP;
                    else                         bcnt <= bcnt + 1'b1;
                end
                else
                    stall_cycles <= stall_cycles + 32'd1;

                B_RESP: if (axi_bvalid) begin
                    // The response retires whatever it says; the verdict
                    // travels beside it.  Suppressing the pulse on an error
                    // would convert a reported failure into a tracker waiting
                    // for an answer that has already come.
                    wr_acks <= wr_acks + 32'd1;
                    wr_ack  <= 1'b1;
                    if (axi_bresp != 2'd0)
                        bresp_errors <= FAULT_STICKY ? 32'd1 : bresp_errors + 32'd1;
                    bst <= B_IDLE;
                end
                else
                    stall_cycles <= stall_cycles + 32'd1;

                default: bst <= B_IDLE;
            endcase
        end
    end

    // A count equality alone is not enough: an unissued partial panel and an
    // offered final engine word must also prevent a region handover/reset.
    wire [NBUF-1:0] panel_live;
    genvar di;
    generate for(di=0;di<NBUF;di=di+1)begin:g_drained
        assign panel_live[di]=pact[di];
    end endgenerate
    assign drained=(bst==B_IDLE) && !axi_awvalid && !(|panel_live) && !out_wr_en;
endmodule
