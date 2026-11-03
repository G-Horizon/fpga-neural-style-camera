`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// C2, word by word: the A request, the A read-back, and the B write.
// ---------------------------------------------------------------------------
//
// **What the chunk CRC left open.**  Page 8 says the board's first divergence is
// C2 -- conv2's output into the B ring -- somewhere inside its first 128 writes.
// A 128-transaction CRC cannot say *which* write, and it says nothing at all
// about what C2 read.  This probe answers both, and it is the difference between
// "the C2 stream is wrong" and a repairable boundary.
//
// **Two independent comparators, not one shared CRC.**  The three-stage monitor
// folded one packet per PE cycle and rested on A, B and C never colliding -- the
// measured block-0 trace has 0 collisions.  In the C2 window that premise is
// gone: the A consumption and the B write overlap on **126** cycles.  A shared
// network would have to drop one of them, so the two sides get their own
// synchronous ROMs and their own comparators, and overlapping costs nothing.
//
// **The expected side is computed, not stored.**  The reference's slot metadata
// is a closed-form function of the slot ordinal:
//
//     col = ord/18   phase = ord%18   tap = phase/2   icg = phase%2
//     dy  = tap/3    dx = tap%3       sr = (dy==1) ? 0 : 1
//     sc  = col+dx-1, except sc=1 at (col 0, dx 0) and sc=62 at (col 63, dx 2)
//     address = (sr*64 + sc)*2 + icg          weight address = 58 + phase
//
// so `col` and `phase` are two counters and the whole metadata check is LUTs.
// Only the A *data* (256 words) and the B *data* (128 words) need memory.
//
// **The address is checked independently of the data.**  If the expected address
// were used to *look up* the data and the design's own address were never
// compared, a wrong request address would still return the right word for the
// address it asked for, and the check would pass.  So the actual request address
// is compared against the expected one first, and the data ROM is indexed by the
// *expected* address -- a mismatch in either shows up as itself.
//
// **Out of range is an address fault, not a lookup.**  The reference covers
// A addresses 0..255.  A request outside that is reported as an address error
// and NOT truncated to its low eight bits, which would alias it onto a covered
// word and print a data mismatch that is really an addressing one.
//
// Timing: the request is at T, `a_q_d` holds that word at T+2.  The metadata is
// carried through the same two stages and the data ROM is read at T+1, so
// everything meets at T+2 with no extra delay in the design.
module d64_c2_diag #(
    parameter A_ROM_INIT = "",     // a_consumer_rows01_256x64.mem
    parameter B_ROM_INIT = ""      // b_head128_128x64.mem
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,            // window: block 0, first pass, no redo
    // ---- the A read request edge ----
    input  wire        a_req,         // s_a_rd_en
    input  wire [10:0] a_req_addr,    // a_ra
    input  wire [7:0]  a_req_waddr,   // w_ra_r
    input  wire [3:0]  a_req_tap,
    input  wire        a_req_icg,
    input  wire        a_req_first,
    input  wire        a_req_last,
    // ---- the A consumption edge ----
    input  wire        c_live,
    input  wire [3:0]  c_mode,
    input  wire [63:0] a_q_d,
    // ---- the B write edge ----
    input  wire        b_we,
    input  wire [10:0] b_wa,
    input  wire [63:0] b_wd,
    // ---- the first C2 accept, for the config blind spot ----
    input  wire        c2_accept,
    input  wire [15:0] rq_m0,         // pe_req_rq_m[15:0]   (lane 0)
    input  wire [4:0]  rq_s0,         // pe_req_rq_shift[4:0] (lane 0)
    output wire [287:0] c2,
    // Three stickies kept from the investigation that found the wrap: they say
    // *why* a count is zero when it is, which is the question a bare number
    // cannot answer.  The cycle counters that went with them are gone -- 160
    // flip-flops for a question that is now answered.
    output wire [2:0]    c2_dbg
);

    // ------------------------------------------------------------------
    // the slot the *next* request belongs to
    // ------------------------------------------------------------------
    //
    // 1152 slots = 64 pixels x 18.  `col` and `phase` are counters rather than a
    // division, because dividing by 18 in hardware is a ROM and counting is two
    // flip-flops.
    reg [5:0] col;
    reg [4:0] phase;
    reg [11:0] a_ord;         // A read requests seen
    reg [7:0]  b_ord;         // B writes seen, 0..128 (128 means the reference's 128 all arrived)
    reg [11:0] c_cnt;         // consumption edges seen, 0..1152

    wire [3:0] tap_e = phase[4:1];
    wire       icg_e = phase[0];
    wire [1:0] dy_e  = (tap_e >= 4'd6) ? 2'd2 : (tap_e >= 4'd3) ? 2'd1 : 2'd0;
    wire [1:0] dx_e  = tap_e - {dy_e, 1'b0} - dy_e;      // tap - 3*dy
    wire [3:0] sr_e  = (dy_e == 2'd1) ? 4'd0 : 4'd1;
    wire [6:0] sc_e  = ((col == 6'd0)  && (dx_e == 2'd0)) ? 7'd1
                     : ((col == 6'd63) && (dx_e == 2'd2)) ? 7'd62
                     : ({1'b0, col} + {5'd0, dx_e} - 7'd1);
    // **(sr*64 + sc)*2 + icg.**  The first version concatenated a nine-bit
    // `{0, sr, sc}` and shifted the whole thing left, which is sr*128 + sc*2 --
    // exactly twice the row term -- and reported slot 0 as address 0x002 instead
    // of 0x082.  The row term is sr at bit 6, so it is built at bit 6: `sc` is
    // 0..63 by construction (the two edge cases at col 0 and col 63 are what
    // keep it there) and `sr` is the bit above it.
    wire [7:0]  rowsel_e = (sr_e[0] ? 8'd64 : 8'd0) + {1'b0, sc_e};
    wire [7:0]  addr_e   = {rowsel_e[6:0], 1'b0} + {7'd0, icg_e};
    wire [7:0]  waddr_e = 8'd58 + {3'd0, phase};
    wire [6:0]  tag_e = {tap_e, icg_e, (phase == 5'd0), (phase == 5'd17)};

    // ------------------------------------------------------------------
    // the two reference memories
    // ------------------------------------------------------------------
    reg  [6:0] b_rom_addr;
    wire [63:0] b_rom_data;
    d64_c2_rom #(.DW(64), .AW(7), .INIT_FILE(B_ROM_INIT)) u_b_rom (
        .clk(clk), .addr(b_rom_addr), .dout(b_rom_data));

    // ------------------------------------------------------------------
    // stage 1: what the design actually asked for
    // ------------------------------------------------------------------
    reg        s1_v;
    reg [10:0] s1_addr;
    reg [7:0]  s1_waddr;
    reg [6:0]  s1_tag;
    reg [7:0]  s1_exp_addr;
    reg [7:0]  s1_exp_waddr;
    reg [6:0]  s1_exp_tag;
    reg [11:0] s1_ord;

    // stage 2: the ROM's answer for that slot, and the tags one cycle later
    reg        s2_v;
    reg [10:0] s2_addr;
    reg [7:0]  s2_waddr;
    reg [6:0]  s2_tag;
    reg [7:0]  s2_exp_addr;
    reg [7:0]  s2_exp_waddr;
    reg [6:0]  s2_exp_tag;
    reg [11:0] s2_ord;

    // B: two stages.  The write is at T, the ROM's answer for it is valid
    // during T+2, so the write's own fields have to still be there -- a single
    // stage compared the write at T against the answer for the write at T-1.
    reg        b1_v, b2_v;
    reg [6:0]  b1_ord, b2_ord;
    reg [10:0] b1_addr, b2_addr;
    reg [63:0] b1_data, b2_data;

    // ------------------------------------------------------------------
    // results
    // ------------------------------------------------------------------
    reg        a_seen, b_seen, a_over, b_over;
    reg        a_addr_bad, a_tag_bad, a_waddr_bad;
    // **The A *data* comparison is not in this build.**  Its 256x64 ROM and
    // its sixty-four-bit comparator were the widest nets in the probe, and
    // placement needed 62,808 XLR sites against 60,800 -- +3,131 over the
    // chunk-monitor bit, of which 2,995 was routing.  What is left answers
    // two of the three questions the decision table asks: is the request
    // right, and which B word is wrong.  Whether the ring returned the
    // right word for a right request is the one this build cannot say.
    reg        b_data_bad, b_addr_bad;
    reg        a_bad_v, b_bad_v;
    reg [11:0] a_bad_ord;
    reg [10:0] a_bad_act_addr, a_bad_exp_addr;
    reg [6:0]  a_bad_act_tag, a_bad_exp_tag;
    reg [7:0]  a_bad_act_waddr, a_bad_exp_waddr;
    reg [6:0]  b_bad_ord;
    reg [10:0] b_bad_act_addr, b_bad_exp_addr;
    reg [63:0] b_bad_act_data, b_bad_exp_data;

    // The config blind spot: the P7 instrument verified C1's requant sample and
    // nothing about C2's.  One accept edge, lane 0, against the reference's
    // OC0 values -- and OC0 alone is a weak sample (its accumulator rounds to
    // zero), so what this catches is a gross config mix-up, not a subtle one.
    localparam [15:0] C2_OC0_M = 16'd34464;
    localparam [4:0]  C2_OC0_S = 5'd23;
    reg        cfg_seen, cfg_m_bad, cfg_s_bad;
    reg        dbg_live, dbg_live3, dbg_en3;
    reg [15:0] cfg_m_got;
    reg [4:0]  cfg_s_got;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col <= 6'd0; phase <= 5'd0; a_ord <= 12'd0; b_ord <= 7'd0;
            c_cnt <= 11'd0;
            b_rom_addr <= 7'd0;
            s1_v <= 1'b0; s2_v <= 1'b0; b1_v <= 1'b0;
            s1_addr <= 11'd0; s1_waddr <= 8'd0; s1_tag <= 7'd0;
            s1_exp_addr <= 8'd0; s1_exp_waddr <= 8'd0; s1_exp_tag <= 7'd0;
            s1_ord <= 12'd0;
            s2_addr <= 11'd0; s2_waddr <= 8'd0; s2_tag <= 7'd0;
            s2_exp_addr <= 8'd0; s2_exp_waddr <= 8'd0; s2_exp_tag <= 7'd0;
            s2_ord <= 12'd0;
            b1_v <= 1'b0; b2_v <= 1'b0;
            b1_ord <= 7'd0; b2_ord <= 7'd0;
            b1_addr <= 11'd0; b2_addr <= 11'd0;
            b1_data <= 64'd0; b2_data <= 64'd0;
            a_seen <= 1'b0; b_seen <= 1'b0; a_over <= 1'b0; b_over <= 1'b0;
            a_addr_bad <= 1'b0; a_tag_bad <= 1'b0;
            a_waddr_bad <= 1'b0;
            b_data_bad <= 1'b0; b_addr_bad <= 1'b0;
            a_bad_v <= 1'b0; b_bad_v <= 1'b0;
            a_bad_ord <= 12'd0;
            a_bad_act_addr <= 11'd0; a_bad_exp_addr <= 11'd0;
            a_bad_act_tag <= 7'd0; a_bad_exp_tag <= 7'd0;
            a_bad_act_waddr <= 8'd0; a_bad_exp_waddr <= 8'd0;
            b_bad_ord <= 7'd0;
            b_bad_act_addr <= 11'd0; b_bad_exp_addr <= 11'd0;
            b_bad_act_data <= 64'd0; b_bad_exp_data <= 64'd0;
            cfg_seen <= 1'b0; cfg_m_bad <= 1'b0; cfg_s_bad <= 1'b0;
            dbg_live <= 1'b0; dbg_live3 <= 1'b0; dbg_en3 <= 1'b0;
            cfg_m_got <= 16'd0; cfg_s_got <= 5'd0;
        end
        else begin
            // ---- stage 1 <- stage 0 ------------------------------------
            // **The probe covers the reference's window, which is C2's first row.**
            // The board's window is the whole first pass -- 64 rows -- so the
            // first 1152 requests and the first 128 writes are the ones the
            // reference describes and everything past them is outside it.  The
            // first version compared *every* pass event against a row-0 reference,
            // which is not a mismatch but a category error, and it wrapped a
            // twelve-bit consumption counter at exactly 4096 x 18 = 73,728 --
            // the pass total -- so a working probe reported zero.
            s1_v <= a_req & en & (a_ord < 12'd1152);
            if (a_req & en) begin
                s1_addr <= a_req_addr;
                s1_waddr <= a_req_waddr;
                s1_tag <= {a_req_tap, a_req_icg, a_req_first, a_req_last};
                s1_exp_addr <= addr_e;
                s1_exp_waddr <= waddr_e;
                s1_exp_tag <= tag_e;
                s1_ord <= a_ord;
            end

            // ---- stage 2 <- stage 1, plus the ROM's answer --------------
            s2_v <= s1_v;
            s2_addr <= s1_addr;
            s2_waddr <= s1_waddr;
            s2_tag <= s1_tag;
            s2_exp_addr <= s1_exp_addr;
            s2_exp_waddr <= s1_exp_waddr;
            s2_exp_tag <= s1_exp_tag;
            s2_ord <= s1_ord;
            // **The ROM's answer is NOT registered here.**  The address was
            // written at edge T, so `mem` is addressed from cycle T+1 and its
            // output is valid during cycle T+2 -- which is exactly when this
            // block compares.  Latching it at T+1 and comparing the copy read
            // the *previous* slot's word: the same one-cycle-early defect the
            // chunk CRC ROM had, and it reported slot 0 against mem[0] instead
            // of mem[130].

            // Three stickies that say *why* the comparison never fired when it
            // does not: `c_live` at all, `c_live` with the C2 mode, and the C2
            // mode with the window open.  Without them a count of zero is a
            // number with no explanation.
            if (c_live && en) dbg_live <= 1'b1;
            if (c_live && (c_mode == 4'd3)) dbg_live3 <= 1'b1;
            if (en && (c_mode == 4'd3)) dbg_en3 <= 1'b1;

            // ---- the comparison, at the consumption edge ---------------
            if (c_live && (c_mode == 4'd3) && en) begin
                if (a_seen === 1'b0) a_seen <= 1'b1;
                if (s2_v) begin
                    // Counted only for the slots the reference covers, which is
                    // also what keeps the twelve-bit counter from wrapping.
                    c_cnt <= c_cnt + 12'd1;
                    // The address is compared first and against the full 11
                    // bits: a request outside the reference's 0..255 is an
                    // addressing fault, not a data one, and masking it to eight
                    // bits would alias it onto a covered word and report the
                    // wrong thing.
                    // **Three stickies, one record, and any of the three arms
                    // it.**  The record used to arm on an address mismatch
                    // alone, so when the board's weight address came back wrong
                    // the sticky was set and nothing was latched: the actual and
                    // expected weight addresses both read zero and the one value
                    // that round existed to obtain was not in the capture.  A
                    // sticky says *something* differed; a record says *what*.
                    // The round after this one needs the record.
                    //
                    // One event latches once, so the three fields in it are from
                    // the same slot and can be read against each other: if the
                    // weight is off by one slot, the tag is what says whether the
                    // request was late or the address was.
                    if ((s2_addr  !== {3'd0, s2_exp_addr}) ||
                        (s2_tag   !== s2_exp_tag) ||
                        (s2_waddr !== s2_exp_waddr)) begin
                        if (!a_bad_v) begin
                            a_bad_v <= 1'b1; a_bad_ord <= s2_ord;
                            a_bad_act_addr <= s2_addr; a_bad_exp_addr <= {3'd0, s2_exp_addr};
                            a_bad_act_tag <= s2_tag;   a_bad_exp_tag <= s2_exp_tag;
                            a_bad_act_waddr <= s2_waddr; a_bad_exp_waddr <= s2_exp_waddr;
                        end
                    end
                    if (s2_addr  !== {3'd0, s2_exp_addr}) a_addr_bad <= 1'b1;
                    if (s2_tag   !== s2_exp_tag)   a_tag_bad  <= 1'b1;
                    if (s2_waddr !== s2_exp_waddr) a_waddr_bad <= 1'b1;
                end
            end

            // ---- the request counter and the slot it belongs to --------
            if (a_req & en) begin
                // **One past the reference's coverage, and counted rather than
                // wrapped.**  The first version set this on the *last* covered
                // request and let a twelve-bit ordinal wrap, so an extra request
                // would have been invisible and a complete run would have been
                // reported as an overrun.
                if (a_ord >= 12'd1152) a_over <= 1'b1;
                if (a_ord < 12'd1152) a_ord <= a_ord + 12'd1;
                if (phase == 5'd17) begin
                    phase <= 5'd0;
                    if (col == 6'd63) col <= 6'd0;
                    else             col <= col + 6'd1;
                end
                else phase <= phase + 5'd1;
            end

            // ---- the B write, word by word -----------------------------
            if (b_we & en) begin
                if (b_seen === 1'b0) b_seen <= 1'b1;
                if (b_ord >= 8'd128) b_over <= 1'b1;
                if (b_ord < 8'd128) begin
                    b_rom_addr <= b_ord[6:0];
                    b1_v   <= 1'b1;
                    b1_ord <= b_ord[6:0];
                    b1_addr <= b_wa;
                    b1_data <= b_wd;
                    b_ord <= b_ord + 8'd1;
                end
            end
            else b1_v <= 1'b0;

            b2_v <= b1_v; b2_ord <= b1_ord;
            b2_addr <= b1_addr; b2_data <= b1_data;
            if (b2_v) begin
                // The reference's addresses for row 0 are 0..127 in write order,
                // but the address is *observed* rather than assumed: printing
                // the expected coordinate instead of the actual one would hide a
                // permutation, which is exactly what a wrong write order is.
                if (b2_data !== b_rom_data) begin
                    b_data_bad <= 1'b1;
                    if (!b_bad_v) begin
                        b_bad_v <= 1'b1; b_bad_ord <= b2_ord;
                        b_bad_act_addr <= b2_addr;
                        b_bad_exp_addr <= {4'd0, b2_ord};
                        b_bad_act_data <= b2_data; b_bad_exp_data <= b_rom_data;
                    end
                end
                if (b2_addr !== {4'd0, b2_ord}) begin
                    b_addr_bad <= 1'b1;
                    if (!b_bad_v) begin
                        b_bad_v <= 1'b1; b_bad_ord <= b2_ord;
                        b_bad_act_addr <= b2_addr;
                        b_bad_exp_addr <= {4'd0, b2_ord};
                        b_bad_act_data <= b2_data; b_bad_exp_data <= b_rom_data;
                    end
                end
            end

            // ---- the C2 config, at the first accept --------------------
            if (c2_accept && en && !cfg_seen) begin
                cfg_seen  <= 1'b1;
                cfg_m_got <= rq_m0;
                cfg_s_got <= rq_s0;
                if (rq_m0 !== C2_OC0_M) cfg_m_bad <= 1'b1;
                if (rq_s0 !== C2_OC0_S) cfg_s_bad <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // the flat result, in the order the decision table reads it
    // ------------------------------------------------------------------
    assign c2[0]        = a_seen;
    assign c2[1]        = b_seen;
    assign c2[2]        = a_over;
    assign c2[3]        = b_over;
    assign c2[4]        = a_addr_bad;
    assign c2[5]        = a_tag_bad;
    assign c2[6]        = a_waddr_bad;
    assign c2[7]        = 1'b0;   // the A data comparison is not built
    assign c2[8]        = b_data_bad;
    assign c2[9]        = b_addr_bad;
    assign c2[10]       = a_bad_v;
    assign c2[11]       = b_bad_v;
    assign c2[22:12]    = a_ord;
    assign c2[30:23]    = b_ord;
    assign c2[42:31]    = c_cnt;
    assign c2[53:43]    = a_bad_ord;
    assign c2[64:54]    = a_bad_act_addr;
    assign c2[75:65]    = a_bad_exp_addr;
    assign c2[82:76]    = a_bad_act_tag;
    assign c2[89:83]    = a_bad_exp_tag;
    assign c2[97:90]    = a_bad_act_waddr;
    assign c2[105:98]   = a_bad_exp_waddr;
    // **No A-data fields at all.**  The comparison is not built, and the two
    // 64-bit result registers it fed were still being latched and shipped to
    // telemetry -- 128 flip-flops and their routing for a value nothing writes.
    // Removing them is part of how this bit fits: placement wanted 62,808 XLR
    // sites against 60,800 before the cut, and 61,198 after the comparison went.
    assign c2[113:106]  = b_bad_ord;
    assign c2[124:114]  = b_bad_act_addr;
    assign c2[135:125]  = b_bad_exp_addr;
    assign c2[199:136]  = b_bad_act_data;
    assign c2[263:200]  = b_bad_exp_data;
    assign c2[264]      = cfg_seen;
    assign c2[265]      = cfg_m_bad;
    assign c2[266]      = cfg_s_bad;
    assign c2[282:267]  = cfg_m_got;
    assign c2[287:283]  = cfg_s_got;

    // **Driven, not left dangling.**  These three were declared and latched from
    // the first version of the probe and never connected to the port that
    // carries them, so the model printed `z` for all three -- an unassigned
    // output reads as "the probe observed nothing" when what happened is that
    // nothing was wired to say.  They are the window's own evidence: `c_live&en`
    // is the consumption edge, and `c_live&c_mode==3` and `en&c_mode==3` are the
    // two halves of the qualifier, so a count of zero can be read against which
    // half was missing rather than guessed at.
    assign c2_dbg = {dbg_en3, dbg_live3, dbg_live};
endmodule
