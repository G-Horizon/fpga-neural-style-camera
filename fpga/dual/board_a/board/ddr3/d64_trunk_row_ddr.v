`timescale 1ns/1ps

// **The block's beat count comes from the shape, and the derivation is one
// file.**  This `include is before the module because `D64_ROWS_TO_BEATS` is a
// parameter default; the counter-width function it pairs with is included
// inside the body, where a declaration needs it.
`include "d64_blk_beats.vh"

// D3c6-e6 -- the trunk's row/beat contract on one side, AXI bursts on the other.
//
// `d64_trunk_blocks` speaks rows: it holds a row number stable, toggles a
// request, and the far side answers with a toggle and then delivers exactly 256
// beats of 128 bits -- four banks of sixty-four pixels.  It writes the same way
// and raises its completion toggle when the block's last beat has landed.
//
// That contract was modelled in `tb_d64_trunk_blocks` and measured at two DDR
// service rates.  On the board the far side is DDR3, so this is the module that
// turns it into the same 128-beat, 16-byte, INCR bursts `d64_encoder_ddr_mem`
// already issues -- because the bridge downstream and the controller's accepted
// shape are D3c6-d's and are not up for renegotiation.
//
// A row is 4 KiB and a burst is 2 KiB, so **a row is exactly two bursts**,
// which is the whole reason the trunk's row is 256 beats and not some other
// number: it was chosen when the DDR's burst was.
//
// The two directions
// ------------------
//
//   read    a request latches the row, two read bursts stream into `rf_wr_*`,
//           and the trunk's own FIFO does the crossing
//   write   `wf_rd_*` is popped into 128-beat write bursts at the destination
//           buffer, and the block's last beat raises `wr_done_tog`
//
// The buffers ping-pong, so both sides take `src_buf`/`dst_buf` from the trunk
// rather than deriving them: the trunk already owns that sequence and a second
// copy of it here is the defect D3c6-e3 and D3c6-e5 were both about.
module d64_trunk_row_ddr #(
    parameter integer TRUNK_FIRST_BLOCK = 0,
    parameter integer TRUNK_BLOCK_COUNT = 5,
    parameter integer LANES_PER_BEAT = 128,
    parameter [7:0]   BURST_LEN = 8'd127,      // 128 beats of 16 bytes = 2 KiB
    parameter integer ROW_BEATS = `D64_ROW_BEATS,   // 4 KiB a row
    // **The shape, and the count is derived from it.**  This parameter is what
    // the board drives, because the board is what knows how many rows a block
    // has.  F09: the literal `16384` stood here with the comment "64 rows" and
    // nothing in the hierarchy drove it, so at 36 rows (9,216 beats) the
    // completion toggle below never rose and the trunk never left block 0 --
    // measured in `docs/TRUNK_BANDING_2026-10-07.md`.
    parameter integer ROWS = 64,
    // **`BLK_BEATS` is derived and is not a shape.**  It stays a parameter so a
    // unit harness that has no shape -- `tb_d64_trunk_row_ddr`,
    // `tb_trunk_row_ddr_blk_beats` -- can still ask for a count directly, and
    // `tools/run_trunk_block_beats.sh` (3.19) does exactly that.  A caller with
    // a shape drives `ROWS`; the derivation is `D64_ROWS_TO_BEATS` here, which
    // is what stops this file's default disagreeing with the board's.
    parameter integer BLK_BEATS = `D64_ROWS_TO_BEATS(ROWS),
    // Legacy callers may leave wf_rd_level unconnected when this is0.
    // Final root enables full-burst reservation; all blocks are whole128s.
    parameter integer RESERVE_WRITE_BURST = 0,
    parameter integer WF_LEVEL_W = 12
) (
    input  wire         clk_pe,
    input  wire         clk_axi,
    input  wire         arst_n,

    // the two buffers, byte addresses
    // **P6i: the parity does not move while a block is being redone.**
    // `axi_blk` advances at the block's last write response, but a redo means the
    // PE is going to read and compute the *same* block again -- so if the parity
    // moves underneath it, the reread comes out of the other buffer and the
    // mechanism reads the wrong tensor.  Held high, the index stays put until
    // the redo is settled.
    input  wire         hold_blk,

    input  wire [31:0]  buf_a_base,
    input  wire [31:0]  buf_b_base,

    // ---- the trunk's side, `clk_pe` ---------------------------------------
    input  wire [6:0]   x_req_row,
    input  wire         x_req_tog,
    output reg          x_ack_tog,
    // **The buffer this request reads, sent with the request.**  `src_buf` is
    // the same bit but it arrives on its own, and this side used to derive the
    // read parity by comparing a synchronised completion toggle against the
    // value it last saw -- a race, and one the board loses differently on
    // different placements.  Latched at `rq_edge` beside `x_req_row`, which
    // crosses the same way and always has, it is not compared against anything.
    input  wire         x_req_buf,
    input  wire         src_buf,
    input  wire         dst_buf,

    output reg          rf_wr_en,
    output reg  [127:0] rf_wr_data,
    input  wire         rf_wr_full,

    output wire         wf_rd_en,
    input  wire [127:0] wf_rd_data,
    input  wire         wf_rd_valid,
    input  wire         wf_rd_empty,
    input  wire [WF_LEVEL_W-1:0] wf_rd_level,
    output reg          wr_done_tog,

    // The boundary monitor: at the instant this adapter says a block's writes
    // are answered, how many of the boundaries had the read side still busy.
    output reg  [7:0]   bnd_seen,
    output reg  [7:0]   bnd_busy,

    // What the run did, for telemetry: the parity this side ended on, how
    // many whole blocks it completed, and whether the PE side's select
    // ever disagreed with it.  A shift history was tried here first and
    // **the mapper removed it** -- `src_hist`/`dst_hist` are absent from
    // that build's netlist while `bnd_seen`/`bnd_busy`, same always block,
    // are present.  These drive pins directly and cannot be trimmed.
    // **How many times the read base actually changed.**  Five blocks must read
    // four different buffers between them -- block N reads what block N-1 wrote
    // -- so a working ping-pong moves the base four times.  A base that never
    // moves gives 0, and that is the direct measurement of blocks reading the
    // same tensor; every earlier attempt to reason about which buffer a block
    // read came down to what this number says outright.
    output reg  [31:0]  rd_base_chg,
    output wire [2:0]   axi_blk_out,
    output wire [3:0]   par_ones_out,
    output wire         par_ok_out,
    output wire         sel_mismatch_out,
    output wire [31:0]  rf_ovf_out,

    // ---- AXI read master, `clk_axi` ---------------------------------------
    output reg          axi_arvalid,
    input  wire         axi_arready,
    output reg  [31:0]  axi_araddr,
    output wire [7:0]   axi_arlen,
    output wire [2:0]   axi_arsize,
    output wire [1:0]   axi_arburst,
    input  wire         axi_rvalid,
    output reg          axi_rready,
    input  wire [127:0] axi_rdata,
    input  wire         axi_rlast,
    input  wire [1:0]   axi_rresp,

    // ---- AXI write master, `clk_axi` --------------------------------------
    output reg          axi_awvalid,
    input  wire         axi_awready,
    output reg  [31:0]  axi_awaddr,
    output wire [7:0]   axi_awlen,
    output wire [2:0]   axi_awsize,
    output wire [1:0]   axi_awburst,
    output wire         axi_wvalid,
    input  wire         axi_wready,
    output wire [127:0] axi_wdata,
    output wire [15:0]  axi_wstrb,
    output wire         axi_wlast,
    input  wire         axi_bvalid,
    output reg          axi_bready,
    input  wire [1:0]   axi_bresp,

    // ---- evidence ---------------------------------------------------------
    output reg  [31:0]  rd_beats,
    output reg  [31:0]  wr_beats,
    output reg  [31:0]  rd_bursts,
    output reg  [31:0]  wr_bursts,
    output wire [31:0]  axi_errors
);
    // The block's beat counter is sized to the block, not to a guess.
    `include "d64_blk_beats_width.vh"

    assign axi_arlen   = BURST_LEN;
    assign axi_awlen   = BURST_LEN;
    assign axi_arsize  = 3'd4;          // 16 bytes, the same as D3c6-d's
    assign axi_awsize  = 3'd4;
    assign axi_arburst = 2'b01;         // INCR
    assign axi_awburst = 2'b01;
    assign axi_wstrb   = 16'hFFFF;

    // ------------------------------------------------------------------
    // the request toggle, crossed
    // ------------------------------------------------------------------
    reg [1:0] rtog_s;
    reg       rtog_seen;
    wire      rq_edge = rtog_s[1] ^ rtog_seen;

    reg [6:0] rd_row;
    reg [1:0] rd_owed, rd_burst;
    reg [7:0] rd_sent;
    reg [31:0] rf_ovf;             // beats offered while the FIFO was full
    wire rd_burst_end = axi_rvalid && axi_rready && (rd_sent == BURST_LEN);

    // **Two counters, not one.**  The read and the write side are different
    // always blocks, and one register written from both is a multiply-driven
    // net: ModelSim tolerates it and the mapper does not.  Which of the two
    // found it first is the point -- the adapter's own testbench passed with
    // the double driver in place the whole time.
    reg  [31:0]  rd_errors, wr_errors;
    assign axi_errors = rd_errors + wr_errors;

    // ------------------------------------------------------------------
    // the read side
    // ------------------------------------------------------------------
    //
    // **One always block, and that is not a style choice.**  The row request and
    // the burst accounting both write `rd_owed`, and a register written from two
    // blocks is a multiply-driven net -- ModelSim tolerates it, and the mapper
    // refused to elaborate this file twice for exactly that reason.  The second
    // one was here: `rd_owed` was set by the request and decremented by the
    // burst, in different blocks, and the adapter's own testbench passed.
    // **`src_buf`/`dst_buf` are `clk_pe` signals and they are used here, in
    // `clk_axi`, to build addresses.**  `d64_trunk_blocks` drives them straight
    // off `blk_r`, so they change the instant the block index does, and a
    // `clk_pe` edge lands wherever it likes inside a `clk_axi` cycle.  Two
    // flops make every address below come from a value that was stable for a
    // whole `clk_axi` cycle.  The monitor at the end of this file reports
    // whether that was the only thing wrong.
    // ------------------------------------------------------------------
    // the ping-pong parity, owned here
    // ------------------------------------------------------------------
    //
    // **The AXI domain decides which buffer is which; the PE domain no longer
    // has a say.**  `src_buf`/`dst_buf` come from `d64_trunk_blocks` on
    // `clk_pe`, and building `clk_axi` addresses out of them made every read
    // and every write depend on a signal this side cannot see change.
    // Registering them was not enough: "which buffer is current" is an
    // AXI-domain question, because it is the AXI side that knows a block's
    // writes have been answered.  So the parity advances on that same event --
    // the whole-block write response -- and no address depends on a clock
    // boundary at all.
    //
    //     axi_blk 0   block 0 reads A writes B
    //             1   block 1 reads B writes A
    //             2   block 2 reads A writes B
    //             3   block 3 reads B writes A
    //             4   block 4 reads A writes B, then wraps to 0
    //
    // It wraps rather than saturating so a second frame starts where the first
    // did.  `d64_trunk_blocks` still drives `src_buf`/`dst_buf`; they are kept
    // as a cross-check and no longer reach an address.
    reg [2:0] axi_blk;
    // **A redo retires nothing**, so this stays at five however many passes the
    // block took -- it is the count of logical blocks whose buffer was handed
    // over, not of passes.  Widening it to eight bits was tried on the guess that
    // a redo would push it past seven; the model's `+REDO_TEST=1` run, twelve
    // passes and seven redos, retired five, and the count went back where it was.
    // The pass count on this side is the write beats, and the PE's is
    // `blk_starts`; those two are what have to agree.
    reg [2:0] blk_done;
    reg       sel_mismatch;
    // **An accumulator, and one the mapper has to compute.**  `blk_done` was
    // removed from this design's netlist for having no consumer but a report
    // slot -- the third register in this stage to be folded that way.  What
    // survives is `bnd_seen`, and what is different about it is that something
    // *compares* it.  So `par_ones` is compared too: it counts the block
    // completions whose parity was 1, and a working ping-pong over five blocks
    // completes at parities 0,1,0,1,0, giving 2.  A parity that never moves
    // gives 0, and one that moves but is not used gives something else.
    reg [3:0] par_ones;
    wire par_ok = (par_ones == ((TRUNK_BLOCK_COUNT + (TRUNK_FIRST_BLOCK & 1)) / 2));

    // **A row request carries the parity that was current when it was made.**
    // `axi_blk` advances at the block's last write response, but the AR for a
    // row is issued lazily -- gated on `rd_owed` and `rf_wr_full` -- so a
    // request the engine made before the flip could have its address formed
    // after it, and read the buffer the *next* block is about to fill.  That is
    // the window e6a left open: it closed the one between `src_buf` and the
    // address, and opened this one between the parity and the AR.  Latched at
    // the request, the two cannot disagree.
    // **The read parity advances on the request, not on a sampled value.**
    // Latching `axi_blk` at `rq_edge` still assumes the parity has already moved
    // by the time a new block's first request arrives, and the board says it has
    // not: every read came from buffer A while the writes ping-ponged, which
    // accounts for `in0 == in1` and `in2 == in3 == in4` exactly.  So the request
    // itself carries the transition -- the first request after a block
    // completion flips the parity, and every later request in that block keeps
    // it.  Nothing is sampled that can be mid-change.
    // **The first request adopts the toggle, it does not act on it.**  The
    // encoder's own CRCs are constant across runs and exact -- it writes trunk-A
    // as 0x343C03EE and reads it back identically every time -- while the
    // trunk's *first* read comes back as 0x343C03EE in one run and as two other
    // values in two others.  Since the encoder is deterministic, the address is
    // not.  If `wr_done_tog` already differs from `rd_tog_seen` when block 0
    // issues its first request, the old code flipped the parity before the first
    // block had read anything, and block 0 read a buffer the encoder never
    // wrote.  Adopting the value on the first request makes block 0 read buffer
    // A always, which is what the design says it does.
    reg rd_par, rd_tog_seen, rd_armed;
    // **The address is scoped to the request, not to the moment the AR is
    // issued.**  `rd_par` was already latched at `rq_edge`, but the AR is issued
    // lazily -- gated on `rd_owed`, `rf_wr_full` and `rd_live` -- so a request
    // whose AR went out after the *next* request had flipped the parity was
    // addressed with the wrong buffer.  One address bit is one buffer, and the
    // whole point of moving the parity into this domain was that no address
    // should depend on when something else happened.  Latching the base at the
    // request finishes that job.
    reg [31:0] rd_base_q;
    wire [31:0] rd_base = rd_base_q;
    // The parity this request will use, computed combinationally so the base
    // and the parity are latched from the *same* value.  Writing
    // `rd_base_q <= rd_par ? ...` beside `rd_par <= ~rd_par` latched the base
    // from the old parity, which the model caught as block 1 reading buffer A.
    wire rd_flip  = rd_armed & (rd_tog_seen != wr_done_tog);
    wire rd_par_n = rd_par ^ rd_flip;
    wire [31:0] rd_addr = rd_base + {13'd0, rd_row, 12'd0}
                        + {19'd0, rd_burst, 11'd0};

    reg rd_live;

    always @(posedge clk_axi or negedge arst_n) begin
        if (!arst_n) begin
            rtog_s <= 2'd0; rtog_seen <= 1'b0; x_ack_tog <= 1'b0;
            rd_par <= 1'b0; rd_tog_seen <= 1'b0; rd_armed <= 1'b0;
            rf_ovf <= 32'd0;
            rd_base_q <= buf_a_base;
            rd_base_chg <= 32'd0;
            rd_row <= 7'd0; rd_owed <= 2'd0; rd_burst <= 2'd0; rd_sent <= 8'd0;
            axi_arvalid <= 1'b0; axi_araddr <= 32'd0; axi_rready <= 1'b0;
            rf_wr_en <= 1'b0; rf_wr_data <= 128'd0; rd_live <= 1'b0;
            rd_beats <= 32'd0; rd_bursts <= 32'd0; rd_errors <= 32'd0;
        end
        else begin
            rf_wr_en <= 1'b0;
            rtog_s <= {rtog_s[0], x_req_tog};

            if (rq_edge) begin
                rtog_seen <= rtog_s[1];
                x_ack_tog <= ~x_ack_tog;
                rd_row    <= x_req_row;
                // Block N reads the buffer block N-1 wrote, and the request
                // says which that is.  `rd_par` still tracks the completion
                // toggle so that nothing else that reads it changes meaning;
                // it no longer selects an address.
                if ((x_req_buf ? buf_b_base : buf_a_base) != rd_base_q)
                    rd_base_chg <= rd_base_chg + 32'd1;
                rd_base_q   <= x_req_buf ? buf_b_base : buf_a_base;
                rd_par      <= rd_par_n;
                if (!rd_armed) begin
                    // Block 0: adopt whatever the far side is saying, do not act
                    // on it.  This request reads buffer A.
                    rd_armed    <= 1'b1;
                    rd_tog_seen <= wr_done_tog;
                end
                else if (rd_tog_seen != wr_done_tog) begin
                    // A later block's first request: the previous block's writes
                    // are answered, so this one reads what they wrote.
                    rd_tog_seen <= wr_done_tog;
                end
                rd_owed   <= 2'd2;          // 4 KiB is two 2 KiB bursts
                rd_burst  <= 2'd0;
                rd_sent   <= 8'd0;
            end
            // **A burst is 128 beats and `rf` is 32 deep.**  The AR is gated on
            // `!rf_wr_full`, but that only stops a burst being *started* -- once
            // it is in flight, `axi_rready` was held high for all 128 beats and
            // `rf_wr_en` did not look at the FIFO at all, so a reader that fell
            // behind had its write pointer walked past its read pointer and lost
            // the beats silently.  Beat counts stay exact because the pushes all
            // happen; the data is what goes.  Holding `axi_rready` low while the
            // FIFO is full is what AXI provides for exactly this, and it costs
            // the reader nothing.
            // **Measure first.**  Holding `axi_rready` low while the FIFO is full
            // is the textbook fix and it deadlocks this design in simulation --
            // `rf_wr_full` does assert, and stalling the R channel leaves the
            // engine unable to finish the row that would have drained it.  So the
            // backpressure is not applied; the counter is.  If `rf_ovf` is
            // non-zero on the board, the overflow is real and the fix is to size
            // `rf` for a whole burst rather than to stall the channel.
            // **Count at the edge where the FIFO actually writes, not at the
            // offer.**  `rf_wr_en` is registered, so the beat taken on this R
            // handshake reaches the FIFO's `wr_en` on the *next* cycle, against
            // the full flag as it is then.  Checking `rf_wr_full` here tests a
            // moment that is already past: the write registered from the previous
            // beat can fill the last free slot in between, and it is that write
            // -- the one being registered now -- that the FIFO rejects.  The
            // directed testbench measures exactly this shape -- 128 beats
            // returned, one write rejected, `rf_ovf` reading zero
            // (`sim/d3c6e6/transport_fix/tb_transport_fix.v`, driven by
            // `run_transport_fix.sh`).  That is a testbench measurement, not a
            // board observation: the defect is established by the
            // counterexample, not by a capture.  So the counter watches
            // `rf_wr_en && rf_wr_full`,
            // which is the same condition `d64_async_fifo` uses to drop a write,
            // sampled on the cycle that write happens.
            if (rf_wr_en && rf_wr_full)
                rf_ovf <= rf_ovf + 32'd1;

            if (axi_rvalid && axi_rready) begin
                rf_wr_en   <= 1'b1;
                rf_wr_data <= axi_rdata;
                rd_beats   <= rd_beats + 32'd1;
                if (axi_rresp != 2'd0) rd_errors <= rd_errors + 32'd1;
                if (rd_sent == BURST_LEN) begin
                    rd_sent  <= 8'd0;
                    rd_burst <= rd_burst + 2'd1;
                    rd_owed  <= rd_owed - 2'd1;
                    rd_live  <= 1'b0;
                    axi_rready <= 1'b0;
                    rd_bursts  <= rd_bursts + 32'd1;
                end
                else rd_sent <= rd_sent + 8'd1;
            end

            if (!axi_arvalid && (rd_owed != 2'd0) && !rf_wr_full && !rd_live
                && !rq_edge) begin
                axi_arvalid <= 1'b1;
                axi_araddr  <= rd_addr;
            end
            else if (axi_arvalid && axi_arready) begin
                axi_arvalid <= 1'b0;
                rd_live     <= 1'b1;
                axi_rready  <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // the write side
    // ------------------------------------------------------------------
    //
    // A block is `BLK_BEATS` -- `ROWS` rows of 256 -- and a burst is 128, so the
    // writer's whole job is to hold an AW, stream 128 beats out of the trunk's
    // FIFO, take the response, and do that BLK_BEATS/128 times.  The count is
    // derived at the top of this file and is not written down here.
    //
    // **The two-deep queue is not an optimisation, it is the rate.**
    // `d64_async_fifo` answers a read enable a cycle later and its `rd_valid` is
    // a one-cycle pulse, so a writer that reads one beat, waits for it, and then
    // offers it spends **four cycles a beat** -- read, answer, offer, transfer.
    // That is what this did, and the block barrier at the end of each of the
    // five blocks paid for it: 39,195 cycles against D3c6-e5's 4,886.
    //
    // `d64_encoder_ddr_mem`'s writer has had two slots and a `wf_rd_pending`
    // flag all along, which keeps two reads in flight and sustains a beat a
    // cycle.  This is that shape, because the same FIFO is in front of it.
    //
    // `wr_done_tog` is raised when the block's last beat has been **answered**,
    // not when it was handed over: the trunk's barrier waits on it, and a
    // barrier built on the handover is the defect D3c5-b1 found by counting
    // 16,383 words instead of 16,384.
    localparam [1:0] W_IDLE = 2'd0, W_BURST = 2'd1, W_RESP = 2'd2;

    reg [1:0]   wstate;
    reg [7:0]   wsent;
    // **The block's beat counter, sized to the block and compared against the
    // whole of it.**  It was `[14:0]` and the test was `wblk == BLK_BEATS[14:0]`
    // -- which reads as "fifteen bits is what a block needs" and is a silent
    // truncation the moment one does not: at 136 rows (34,816 beats) that slice
    // is 2,048, the equality can never be reached, and the completion stops
    // exactly as it does in the 36-row fault this change is about.  A test that
    // can only be right at one width is not a test.
    localparam integer BLK_W = d64_beats_w(BLK_BEATS);
    reg [BLK_W-1:0] wblk;
    reg [1:0]   wq_count;
    reg [127:0] wq0, wq1;
    reg         wf_rd_pending;

    // FIFO rd_level is gray_to_binary(wgray_s2)-rbin in clk_axi: producer
    // visibility may lag but cannot promise future words. A popped FIFO word
    // is counted in either pending or wq, so include both exactly once.
    localparam integer WRITE_CREDIT_W=WF_LEVEL_W+3;
    localparam [8:0] WRITE_BURST_BEATS={1'b0,BURST_LEN}+9'd1;
    wire [WRITE_CREDIT_W-1:0] write_available=
        {3'd0,wf_rd_level}+
        {{(WRITE_CREDIT_W-2){1'b0}},wq_count}+
        {{(WRITE_CREDIT_W-1){1'b0}},wf_rd_pending};
    wire [WRITE_CREDIT_W-1:0] write_required=
        {{(WRITE_CREDIT_W-9){1'b0}},WRITE_BURST_BEATS};
    wire write_start_ok=RESERVE_WRITE_BURST ? (write_available>=write_required) :
        (!wf_rd_empty||(wq_count!=2'd0));

    wire [31:0] wr_base = axi_blk[0] ? buf_a_base : buf_b_base;
    // **`wblk` shifted, not padded to a width someone counted.**  It was
    // `{13'd0, wblk, 4'd0}`, which is 32 bits only while `wblk` is fifteen --
    // the same assumption the old comparison made, in a second place.
    wire [31:0] wr_addr = wr_base + {wblk, 4'd0};

    wire        wf_pop_int    = (wstate == W_BURST) && axi_wvalid && axi_wready;
    wire [2:0]  wq_after_pop  = {1'b0, wq_count} - wf_pop_int;

    // Read whenever the burst engine is running and the queue has room for the
    // beat plus whatever read is already in flight.
    assign wf_rd_en = (wstate == W_BURST) && !wf_rd_empty &&
                      ((wq_after_pop + wf_rd_pending) < 2);

    assign axi_wvalid = (wstate == W_BURST) && (wq_count != 2'd0);
    assign axi_wdata  = wq0;
    assign axi_wlast  = (wsent == BURST_LEN);

    always @(posedge clk_axi or negedge arst_n) begin
        if (!arst_n) begin
            wq_count <= 2'd0; wq0 <= 128'd0; wq1 <= 128'd0; wf_rd_pending <= 1'b0;
        end
        else begin
            case ({wf_rd_en, wf_rd_valid})
                2'b10: wf_rd_pending <= 1'b1;
                2'b01: wf_rd_pending <= 1'b0;
                2'b11: wf_rd_pending <= 1'b1;
                default: wf_rd_pending <= wf_rd_pending;
            endcase

            case ({wf_rd_valid, wf_pop_int})
                2'b10: begin
                    if (wq_count == 2'd0)      wq0 <= wf_rd_data;
                    else if (wq_count == 2'd1) wq1 <= wf_rd_data;
                    if (wq_count < 2'd2) wq_count <= wq_count + 2'd1;
                end
                2'b01: begin
                    wq0 <= wq1;
                    wq_count <= wq_count - 2'd1;
                end
                2'b11: wq0 <= wf_rd_data;
                default: ;
            endcase
        end
    end

    always @(posedge clk_axi or negedge arst_n) begin
        if (!arst_n) begin
            wstate <= W_IDLE; axi_awvalid <= 1'b0; axi_awaddr <= 32'd0;
            axi_bready <= 1'b0; wsent <= 8'd0; wblk <= {BLK_W{1'b0}};
            wr_done_tog <= 1'b0; wr_beats <= 32'd0; wr_bursts <= 32'd0;
            axi_blk <= TRUNK_FIRST_BLOCK; blk_done <= 3'd0;
            wr_errors <= 32'd0;
        end
        else begin
            case (wstate)
                // **Not until there is a beat to write.**  Asserting AW the
                // cycle after reset took the shared write channel from
                // `d64_axi_arb2` with nothing to write and held it for the whole
                // run: the first board capture was all zeros with the write FIFO
                // at 254 of 256 and not one AXI transaction issued.
                W_IDLE: if (write_start_ok) begin
                    axi_awvalid <= 1'b1;
                    axi_awaddr  <= wr_addr;
                    wsent       <= 8'd0;
                    wstate      <= W_BURST;
                end

                W_BURST: begin
                    if (axi_awvalid && axi_awready) axi_awvalid <= 1'b0;
                    if (wf_pop_int) begin
                        wr_beats <= wr_beats + 32'd1;
                        wblk     <= wblk + 1'b1;
                        if (wsent == BURST_LEN) begin
                            axi_bready <= 1'b1;
                            wr_bursts  <= wr_bursts + 32'd1;
                            wstate     <= W_RESP;
                        end
                        else wsent <= wsent + 8'd1;
                    end
                end

                W_RESP: if (axi_bvalid && axi_bready) begin
                    axi_bready <= 1'b0;
                    if (axi_bresp != 2'd0) wr_errors <= wr_errors + 32'd1;
                    // **The block's last beat has now been answered.**  The
                    // count is the block's and so is the comparison: one whole
                    // block of beats, at any `ROWS`.  It is an exact equality --
                    // `wblk` reaches `BLK_BEATS` on the beat before the response
                    // and is cleared with the toggle -- so the trunk's own
                    // barrier, which hands over one block at a time, is what
                    // guarantees it is still the block's last beat when the
                    // response lands.  An over-delivery would leave `wblk` past
                    // the count and the toggle silent: that is the same failure
                    // mode as the 36-row fault, left visible rather than smoothed
                    // over with a `>=`.
                    if (wblk == BLK_BEATS) begin
                        wr_done_tog <= ~wr_done_tog;
                        wblk <= {BLK_W{1'b0}};
                        // The barrier is raised either way -- the PE has to be released
                        // to decide whether it is advancing or redoing -- but the buffer
                        // is only retired when it is advancing.  See `hold_blk`.
                        if (!hold_blk) begin
                            axi_blk <= (axi_blk == TRUNK_FIRST_BLOCK + TRUNK_BLOCK_COUNT - 1) ? TRUNK_FIRST_BLOCK : axi_blk + 3'd1;
                            blk_done <= blk_done + 3'd1;
                        end
                    end
                    wstate <= W_IDLE;
                end

                default: wstate <= W_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // the boundary monitor
    // ------------------------------------------------------------------
    //
    // The model answers "the read side is idle" at all four block advances and
    // its chain holds exactly.  The board's chain does not reproduce between
    // runs of the same bitstream, so before believing any explanation of it,
    // this counts how many of the boundaries had a read still owed, a burst
    // live, an AR asserted, or the write FIFO not yet drained.  A zero here
    // means the hazard is not at the boundary this design thinks it is.
    assign axi_blk_out = axi_blk;
    assign par_ones_out = par_ones;
    assign par_ok_out = par_ok;
    assign sel_mismatch_out = sel_mismatch;
    assign rf_ovf_out = rf_ovf;

    reg [1:0] wdt_s;
    always @(posedge clk_axi or negedge arst_n) begin
        if (!arst_n) begin
            wdt_s <= 2'd0; bnd_seen <= 8'd0; bnd_busy <= 8'd0;
            sel_mismatch <= 1'b0; par_ones <= 4'd0;
        end
        else begin
            wdt_s <= {wdt_s[0], wr_done_tog};
            // **Counted on the synchroniser's edge, like `bnd_seen`.**  The same
            // accumulator placed inside the write FSM was folded out of the
            // netlist -- 0 references, three builds running -- while `bnd_seen`,
            // in this block and on this edge, has survived every build and never
            // reported a wrong number.  A counter the mapper can delete is not a
            // measurement, and this one is what says whether the parity moves.
            if (wdt_s[1] ^ wdt_s[0])
                par_ones <= par_ones + {3'd0, axi_blk[0]};
            // Sticky, and the only thing `src_buf`/`dst_buf` still do:
            // if the PE side's select ever disagrees with the parity this
            // side is using, that is a fact about the run worth keeping.
            if (src_buf !== axi_blk[0]) sel_mismatch <= 1'b1;
            if (wdt_s[1] ^ wdt_s[0]) begin
                bnd_seen <= bnd_seen + 8'd1;
                if ((rd_owed != 2'd0) || rd_live || axi_arvalid || !wf_rd_empty)
                    bnd_busy <= bnd_busy + 8'd1;
            end
        end
    end

endmodule
