`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// Which stage, and which 128 write transactions, first differs from the model.
// ---------------------------------------------------------------------------
//
// **The question this answers.**  The board's five residual CRCs have been wrong
// and identical in every capture, the requant itself has been proven correct on
// hardware, and twenty-six RTL perturbations and fifty thousand tensor variants
// have failed to reproduce it.  What is left is a question the end-to-end CRC
// cannot answer: the design writes three distinct streams before packing --
// conv1's output into the A ring, conv2's into the B ring, and the requant's
// branch words into the trunk -- and the first divergence is in one of them.  A
// single CRC over the final tensor localises nothing.
//
// **The boundary is a real write transaction, not a schedule phase.**  The three
// streams are tapped where the RAM write actually happens (`a_we`/`a_wa`/`a_wd`,
// `b_we`/`b_wa`/`b_wd`, and the engine's registered `tw_en`/`tw_addr`/`tw_bank`
// with the branch word beside it).  A tap on the scheduler's `c_mode` would
// describe a write two or three cycles before it happens and would attribute it
// to the wrong pixel.
//
// **One packet is 128 bits and one packet is folded per PE cycle.**  The measured
// block-0 trace has A/B events in consecutive-cycle pairs (4,096 of them) and C
// back-to-back for 16,376 of its 16,384 -- so serialising a packet over several
// cycles is not available.  The packet is
//
//     [127:120] 8'hA5     [119:112] stage id (1 A, 2 B, 3 C)
//     [111:80]  address32 [79:0]    word80
//
// and its 16 bytes are consumed byte 0 first, which is `pkt[7:0]`.  A/B are
// 64-bit write data zero-extended into the 80-bit word; A/B's address is the
// physical ring address *including the bank bit*; C's is `(tw_addr << 2) |
// tw_bank`.  Data and address are in the same packet on purpose -- a stream of
// right values written to wrong places hashes differently, and a monitor that
// hashed only the data would call that a pass.
//
// **128 events per chunk, `4 * ROWS` chunks, one shared CRC network.**  The
// window is the shape's, so the split is the shape's: A and B get `ROWS` chunks
// each (`2 * ROWS * 64` events) and C gets `2 * ROWS` (`4 * ROWS * 64`).  At 64
// rows that is 64/64/128 chunks over 8,192/8,192/16,384 events and 256
// comparisons, which is what the constants said; at 36 rows it is 36/36/72 over
// 4,608/4,608/9,216 and 144.  Each stage's context is independent: a chunk's CRC
// starts from 32'hFFFFFFFF, and the whole point of the chunking is that a
// mismatch names an interval rather than a stream.
//
// **Interleaving is why the stages are judged in dependency order.**  A's chunks
// complete before B's merely by scheduling; the first *completed* mismatch is
// therefore not the first *causal* one.  Every comparison runs to the end of the
// window regardless, and each stage keeps its own first bad chunk, so the
// reading is "A's chunk k, and separately B's chunk j" rather than whichever
// finished first.
//
// **A collision is a failure, not a priority choice.**  With one CRC network,
// two events in one cycle means one of them is dropped.  The sticky `conflict`
// says so and is part of acceptance; what is forbidden is a priority mux that
// drops the second event silently and reports a clean run.
module d64_stage_chunk_crc #(
    // **The shape, and therefore the window.**  The monitor's window is
    // `(2+2+4) * PIX / 128` = `4 * ROWS` chunks with `PIX = ROWS * 64`, so the
    // three streams get `ROWS`, `ROWS` and `2*ROWS` chunks -- and *where* each
    // stream's chunks start in the table is `ROWS` too.  Those four numbers were
    // literals (0, 64, 128 and the budgets 63/63/127) and every one of them is
    // the 64-row window written down: at 36 rows A's chunks are 0..35, B's are
    // 36..71 and C's are 72..143, so the 64-row offsets would have B reading A's
    // expectations and C reading addresses that are not B's either.
    //
    // At `ROWS = 64` every localparam below is the literal it replaces, which is
    // the arm that says the derivation did not move the shipped shape.
    parameter integer ROWS = 64,
    parameter CHUNK_ROM_INIT = ""
)(
    input  wire        clk,
    input  wire        rst_n,
    // Fold only while this is high.  The caller arms it on the first block-0
    // engine start and drops it when the block index leaves 0 or a redo begins,
    // which is the window the model reference was taken over.
    input  wire        obs_en,
    // C1: the A ring's RAM write.
    input  wire        a_we,
    input  wire [10:0] a_wa,
    input  wire [63:0] a_wd,
    // C2: the B ring's RAM write.
    input  wire        b_we,
    input  wire [10:0] b_wa,
    input  wire [63:0] b_wd,
    // C3: the requant's write transaction.
    input  wire        c_en,
    input  wire [11:0] c_addr,
    input  wire [1:0]  c_bank,
    input  wire [79:0] c_word,
    // Results.
    output wire [13:0] cnt_a,
    output wire [13:0] cnt_b,
    output wire [14:0] cnt_c,
    output wire [2:0]  bad_seen,        // per stage: a chunk mismatch was latched
    output wire [7:0]  bad_chunk_a,
    output wire [7:0]  bad_chunk_b,
    output wire [7:0]  bad_chunk_c,
    output wire [31:0] bad_exp_a, output wire [31:0] bad_obs_a,
    output wire [31:0] bad_exp_b, output wire [31:0] bad_obs_b,
    output wire [31:0] bad_exp_c, output wire [31:0] bad_obs_c,
    output wire [8:0]  chunks_done,     // comparisons completed, out of 4*ROWS
    output wire        conflict,        // two stages wrote in one cycle
    output wire        overflow,        // a stage wrote past its chunk budget
    output wire        pipe_busy        // a chunk CRC is still in the ROM pipe
);

    // ------------------------------------------------------------------
    // which stage fired, and what its packet is
    // ------------------------------------------------------------------
    wire ev_a = obs_en & a_we;
    wire ev_b = obs_en & b_we;
    wire ev_c = obs_en & c_en;
    wire multi = (ev_a & ev_b) | (ev_a & ev_c) | (ev_b & ev_c);
    wire any   = ev_a | ev_b | ev_c;

    // C, then B, then A, and the choice is only ever about *which* packet is
    // folded -- the sticky below records that the other was dropped, so this is
    // not a silent priority.
    wire [1:0] sel = ev_c ? 2'd2 : (ev_b ? 2'd1 : 2'd0);

    wire [63:0] dw = (sel == 2'd2) ? 64'd0 : (sel == 2'd1) ? b_wd : a_wd;
    wire [10:0] aw = (sel == 2'd1) ? b_wa : a_wa;
    wire [79:0] w80 = (sel == 2'd2) ? c_word : {16'd0, dw};
    // **The widths are written out, not inferred.**  `{18'd0, c_addr, c_bank}`
    // is 18+12+2.  Writing `{c_addr << 2}` inside a concatenation would make the
    // shift 12 bits wide and drop the top of the result before the padding,
    // which is the class of bug the address widening was for.
    wire [31:0] a32 = (sel == 2'd2) ? {18'd0, c_addr, c_bank} : {21'd0, aw};
    wire [7:0]  sid = (sel == 2'd2) ? 8'd3 : (sel == 2'd1) ? 8'd2 : 8'd1;
    wire [127:0] pkt = {8'hA5, sid, a32, w80};

    // ------------------------------------------------------------------
    // the three contexts
    // ------------------------------------------------------------------
    reg [31:0] crc_q0, crc_q1, crc_q2;
    reg [31:0] crc_sel = 32'hFFFFFFFF;
    always @* crc_sel = (sel == 2'd2) ? crc_q2 : (sel == 2'd1) ? crc_q1 : crc_q0;

    wire [31:0] crc_nxt;
    d64_crc32_128 u_crc (.crc_i(crc_sel), .pkt(pkt), .crc_o(crc_nxt));

    // Events since the start of this stage's current chunk.
    reg [6:0] sub0, sub1, sub2;
    wire last_a = ev_a & (sub0 == 7'd127);
    wire last_b = ev_b & (sub1 == 7'd127);
    wire last_c = ev_c & (sub2 == 7'd127);

    // Index of the chunk now being filled.
    reg [7:0] chunk0, chunk1, chunk2;

    // **The three streams' chunk offsets, and their last chunk index.**  One
    // stream follows the other in the table: A owns 0..ROWS-1, B owns
    // ROWS..2*ROWS-1 and C owns 2*ROWS..4*ROWS-1, which is the 4*ROWS-entry
    // window `check_chunk_reference.py` refuses a table of the wrong length for.
    // The widths are 8 bits because the table's address is, and at `ROWS = 64`
    // these are 64, 128, 63, 63 and 127 -- the literals they replace.
    localparam [7:0] CH_B_BASE = ROWS;
    localparam [7:0] CH_C_BASE = 2 * ROWS;
    localparam [7:0] CH_AB_LAST = ROWS - 1;
    localparam [7:0] CH_C_LAST = 2 * ROWS - 1;

    // ------------------------------------------------------------------
    // the ROM and its one-cycle pipe
    // ------------------------------------------------------------------
    reg  [7:0]  rom_addr;
    wire [31:0] rom_dout;
    d64_chunk_crc_rom #(.INIT_FILE(CHUNK_ROM_INIT)) u_rom (
        .clk(clk), .addr(rom_addr), .dout(rom_dout));

    // **Two stages, because the ROM is synchronous.**  The address is
    // registered and so is the data, so an address written at edge T produces
    // `rom_dout` only after edge T+1 -- comparing at T+1 reads the *previous*
    // address's word.  The first version did exactly that and the standalone
    // testbench said so in one run: A and B interleave every cycle, so A's
    // comparison was reading B's address and every stage reported a mismatch
    // against another stage's expected value.  The tag has to travel the same
    // two cycles the data does.
    reg         pv, pv2;
    reg [1:0]   pstage, pstage2;
    reg [7:0]   pchunk, pchunk2;
    reg [31:0]  pobs, pobs2;

    reg [13:0] r_cnt_a, r_cnt_b;
    reg [14:0] r_cnt_c;
    reg [2:0]  r_bad_seen;
    reg [7:0]  r_bad_chunk_a, r_bad_chunk_b, r_bad_chunk_c;
    reg [31:0] r_bad_exp_a, r_bad_obs_a;
    reg [31:0] r_bad_exp_b, r_bad_obs_b;
    reg [31:0] r_bad_exp_c, r_bad_obs_c;
    reg [8:0]  r_chunks_done;
    reg        r_conflict, r_overflow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_q0 <= 32'hFFFFFFFF; crc_q1 <= 32'hFFFFFFFF; crc_q2 <= 32'hFFFFFFFF;
            sub0 <= 7'd0; sub1 <= 7'd0; sub2 <= 7'd0;
            chunk0 <= 8'd0; chunk1 <= 8'd0; chunk2 <= 8'd0;
            rom_addr <= 8'd0;
            pv <= 1'b0; pstage <= 2'd0; pchunk <= 8'd0; pobs <= 32'd0;
            pv2 <= 1'b0; pstage2 <= 2'd0; pchunk2 <= 8'd0; pobs2 <= 32'd0;
            r_cnt_a <= 14'd0; r_cnt_b <= 14'd0; r_cnt_c <= 15'd0;
            r_bad_seen <= 3'd0;
            r_bad_chunk_a <= 8'd0; r_bad_chunk_b <= 8'd0; r_bad_chunk_c <= 8'd0;
            r_bad_exp_a <= 32'd0; r_bad_obs_a <= 32'd0;
            r_bad_exp_b <= 32'd0; r_bad_obs_b <= 32'd0;
            r_bad_exp_c <= 32'd0; r_bad_obs_c <= 32'd0;
            r_chunks_done <= 9'd0;
            r_conflict <= 1'b0; r_overflow <= 1'b0;
        end
        else begin
            // ---- the comparison, one cycle behind the read -------------
            //
            // `pv` is cleared first and set again below if a chunk completes on
            // this same cycle, so a stage that ends one chunk and immediately
            // begins the next does not lose the second one's issue.
            pv <= 1'b0;
            // The second stage of the pipe: the tag and the observed CRC
            // follow the ROM's own one-cycle address-to-data latency.
            pv2 <= pv; pstage2 <= pstage; pchunk2 <= pchunk; pobs2 <= pobs;
            if (pv2) begin
                r_chunks_done <= r_chunks_done + 9'd1;
                if (rom_dout !== pobs2) begin
                    case (pstage2)
                        2'd0: if (!r_bad_seen[0]) begin
                            r_bad_seen[0] <= 1'b1; r_bad_chunk_a <= pchunk2;
                            r_bad_exp_a <= rom_dout; r_bad_obs_a <= pobs2;
                        end
                        2'd1: if (!r_bad_seen[1]) begin
                            r_bad_seen[1] <= 1'b1; r_bad_chunk_b <= pchunk2;
                            r_bad_exp_b <= rom_dout; r_bad_obs_b <= pobs2;
                        end
                        default: if (!r_bad_seen[2]) begin
                            r_bad_seen[2] <= 1'b1; r_bad_chunk_c <= pchunk2;
                            r_bad_exp_c <= rom_dout; r_bad_obs_c <= pobs2;
                        end
                    endcase
                end
            end

            // ---- the fold ----------------------------------------------
            if (any) begin
                if (multi) r_conflict <= 1'b1;
                // The completed chunk's CRC, complemented: the memory holds final
                // values and the running state is the raw one.
                if (last_a | last_b | last_c) begin
                    pv     <= 1'b1;
                    pstage <= sel;
                    pobs   <= ~crc_nxt;
                end
                case (sel)
                    2'd0: begin
                        r_cnt_a <= r_cnt_a + 14'd1;
                        crc_q0  <= last_a ? 32'hFFFFFFFF : crc_nxt;
                        sub0    <= last_a ? 7'd0 : (sub0 + 7'd1);
                        if (last_a) begin
                            rom_addr <= chunk0;              // 0..ROWS-1
                            pchunk   <= chunk0;
                            if (chunk0 > CH_AB_LAST) r_overflow <= 1'b1;
                            chunk0   <= chunk0 + 8'd1;
                        end
                    end
                    2'd1: begin
                        r_cnt_b <= r_cnt_b + 14'd1;
                        crc_q1  <= last_b ? 32'hFFFFFFFF : crc_nxt;
                        sub1    <= last_b ? 7'd0 : (sub1 + 7'd1);
                        if (last_b) begin
                            rom_addr <= CH_B_BASE + chunk1;   // ROWS..2*ROWS-1
                            pchunk   <= chunk1;
                            if (chunk1 > CH_AB_LAST) r_overflow <= 1'b1;
                            chunk1   <= chunk1 + 8'd1;
                        end
                    end
                    default: begin
                        r_cnt_c <= r_cnt_c + 15'd1;
                        crc_q2  <= last_c ? 32'hFFFFFFFF : crc_nxt;
                        sub2    <= last_c ? 7'd0 : (sub2 + 7'd1);
                        if (last_c) begin
                            rom_addr <= CH_C_BASE + chunk2;   // 2*ROWS..4*ROWS-1
                            pchunk   <= chunk2;
                            if (chunk2 > CH_C_LAST) r_overflow <= 1'b1;
                            chunk2   <= chunk2 + 8'd1;
                        end
                    end
                endcase
            end
        end
    end

    assign cnt_a = r_cnt_a;
    assign cnt_b = r_cnt_b;
    assign cnt_c = r_cnt_c;
    assign bad_seen = r_bad_seen;
    assign bad_chunk_a = r_bad_chunk_a;
    assign bad_chunk_b = r_bad_chunk_b;
    assign bad_chunk_c = r_bad_chunk_c;
    assign bad_exp_a = r_bad_exp_a; assign bad_obs_a = r_bad_obs_a;
    assign bad_exp_b = r_bad_exp_b; assign bad_obs_b = r_bad_obs_b;
    assign bad_exp_c = r_bad_exp_c; assign bad_obs_c = r_bad_obs_c;
    assign chunks_done = r_chunks_done;
    assign conflict = r_conflict;
    assign overflow = r_overflow;
    assign pipe_busy = pv | pv2;
endmodule
