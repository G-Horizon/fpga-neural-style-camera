`timescale 1ns/1ps

// D3c4c-b -- the transposed convolution as one engine.
//
// Two things were proved separately and neither was proved together:
//
//   control/d64_tconv_scheduler.v   where every read and write goes, 147,456
//                                   slots a layer, with the MAC and the requant
//                                   left as a token pipeline
//   control/d64_tconv_mac.v         the arithmetic, exact against the frozen D2
//                                   digest, driven by a testbench that made up
//                                   its own taps
//
// This joins them, and the joining is where the remaining mistakes live, because
// every one of them is invisible to both halves alone.
//
// The pipeline, in cycles
// -----------------------
//
//   T     the scheduler is in S_RUN and commits slot N: address outputs,
//         `cur_*`, `cur_issue`
//   T+1   the address is presented to the store; `cur_*` describes slot N; stage
//         A registers it on the same edge the store latches the address
//   T+2   the store answers with slot N's data and stage A holds slot N's
//         control, so the MAC consumes slot N
//
// **Everything on that edge is aligned**: activation, weight and control all
// become valid on the same cycle.  The stores are registered-read, so their
// outputs are already one cycle behind their addresses; registering the
// activation again on top of that -- which the first version did, while the
// weight went straight in -- is exactly the one-cycle skew this module exists to
// prevent, committed by the module itself.  If timing ever forces another
// register it goes on all three at once, never on one.
//
// The phase rollover lands on the edge ending T+2: slot N is the phase's last,
// the accept latches it, and the bias for the next phase loads on the same edge.
// `pix_start` is therefore `a_slot & a_last & rq_ready` -- the same edge, and not
// the one after it.
//
// The two pixel ports
// -------------------
//
// At `PIX_PAR = 2` the scheduler reads two adjacent input pixels on the same
// cycle, one per port, each with its own address and its own valid.  At the right
// edge the second pixel is off the image while the first is on, so the slot is
// still live and the second lane must be fed *zero* -- not the word the store
// happens to still be holding, which is the previous slot's data and multiplies
// into the accumulator as if it were this one's.  The scheduler already says
// which ports are real; the engine carries that through instead of dropping it.
//
// The stall, and why it is there even though it should never fire
// ---------------------------------------------------------------
//
// The scheduler advances unconditionally and the MAC can only take a phase's
// accumulator on the cycle its last slot lands.  The arithmetic says the requant
// is always ready there -- it is `RQ_CYCLES` cycles and the shortest phase
// produces a slot for each of them -- but that is an argument and the design does
// not rest on it.  `next_last` gives one cycle of warning, the freeze is counted,
// and the gate requires zero.  The prime is once per layer, not on every rising
// edge of the slot stream: a stall stops that stream, and re-priming on its
// restart would reload the bias and wipe the phase being accumulated.

// The store's side of the input port
// ----------------------------------
//
// The scheduler's `stall` and the store's `in_stall` are not the same thing, and
// the difference is the whole of this section.  `stall` says "do not advance";
// `in_stall` says "the slot on the wires has not been taken, so present it
// again".  A store made of RAM always takes a read, so the second did not exist
// until the input store became `d64_burst_prefetch`, which can miss.
//
// The temptation is one OR gate -- the integration note's words -- and it is
// wrong in the direction that hides.  The scheduler's state has already moved
// past the slot it presented when the store's refusal arrives, so a freeze at
// that edge drops the refused read outright and the slot reaches the MAC on the
// store's **previous** word: a wrong operand inside a right number of writes and
// a right number of slots, with `done` still reached and no counter moved.  The
// scheduler therefore holds the whole presentation -- address, enables, phase,
// tap, group, coordinates, port mask -- and replays it, and `cur_issue` says "the
// store took the slot on the wires this cycle" rather than "the schedule advanced
// at the last edge".  Rule 1 of section 6.9, and `SAB_NAIVE_STALL` is the wrong
// build kept so that the tooth can be shown to fire.
//
// The number: `in_stall_cycles` is the cycles a read was presented and refused,
// so the stall is evidence and not an inference.  A stall cannot produce a
// completion -- the schedule cannot pass a read the store has not taken, so
// `done` implies every read was transferred.
module d64_tconv_engine #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter H          = 64,
    parameter W          = 64,
    parameter PIX_PAR    = 1,
    parameter IC_GROUPS  = 4,
    parameter OC_GROUPS  = 1,
    parameter OC_LANES   = 16,
    parameter IC_PAR     = 8,
    parameter WORD_CH    = 8,
    parameter IN_PORTS   = 1,
    parameter W_BASE     = 184,
    parameter W_ADDR_W   = 8,
    parameter IN_ADDR_W  = 14,       // the scheduler addresses PIXELS
    // The store is addressed in WORDS, so the engine's port is wider than the
    // scheduler's by the IC-group multiply.  At ups.1 that is 16384 pixels x 2
    // groups = 32768 words and fourteen bits holds 16384, so the address wrapped
    // at pixel 8192 -- the whole second half of the input read the first half
    // back, and every output from row 127 on was quietly wrong.
    parameter IN_WORD_W  = 16,
    parameter OUT_ADDR_W = 16,

    // what the input store holds, and what the MAC needs
    parameter IN_W       = 10,       // stored activation width (trunk 10, ReLU 8)
    parameter IN_ZERO    = 413,
    parameter integer RUNTIME_IN_ZERO = 0,
    parameter ACT_W      = 11,       // `q - zero` width (trunk 11, ReLU 9)
    parameter WGT_W      = 8,
    parameter ACC_W      = 32,
    parameter M_W        = 16,
    parameter SHIFT_W    = 5,
    parameter OUT_W      = 8,
    parameter RQ_LANES   = 8,
    parameter integer QMIN = 0,
    parameter integer QMAX = 255,

    // Verification only, folded away at 0.
    parameter integer SKEW_SABOTAGE  = 0,  // the store's data one cycle early
    parameter integer META_SABOTAGE  = 0,  // y_valid addressed from live coords
    parameter integer PORT1_SABOTAGE = 0,  // an invalid pixel port fed stale data
    // **The integration note's one OR gate, and nothing else**: the store's
    // `in_stall` folds into the scheduler's `stall` and the hold is not honoured,
    // so a refused read is dropped and its slot is computed on the store's
    // previous word.  Verification only, folded away at 0, and the tooth
    // `docs/TCONV_INPUT_STALL_2026-10-07.md` requires to fail **on data**.
    parameter integer SAB_NAIVE_STALL = 0,
    parameter integer ROLLOVER_SABOTAGE = 0, // the final MAC contribution dropped
    // Cycles between the scheduler's `next_last` warning and the multiplier.
    // D3c4c-d's extra stage made it four.
    parameter integer RQ_LOOKAHEAD = 4,

    // **Zero keeps the bank inside this engine; non-zero hands it out.**  The
    // arithmetic, the schedule, the stores, the scratch and the metadata do not
    // move either way -- only which side of the seam the multipliers live on.
    // The finished network has one bank because 144 DSP blocks cannot be spent
    // twice, so every decoder layer will run with this at 1 and an owner driving
    // `pe_req_*` into a bank it does not own.
    parameter integer EXTERNAL_PE = 0,

    // **The seam's widths, and they are parameters rather than localparams
    // because a port list cannot see a localparam.**  They belong on the
    // interface anyway: every owner meets the same bank, so the widths are the
    // contract and not this module's private business.
    parameter PE_LANES  = 128,
    parameter PE_OC_MAX = 16,
    parameter PE_OUT_W  = 10,
    // **`ACT_W` is not one of these and the first version wrongly used it.**
    // It is a property of the *layer*: 11 for the trunk stream, 9 for a ReLU
    // tensor, so an ups.1 seam was 1152 bits where the contract says 1408 and
    // an ups.0 seam was 1408 -- two different widths for one interface.  The
    // bank is built once at the network's widest, so the seam is too.
    parameter PE_ACT_W  = 11,
    parameter integer WRITE_RESERVATION = 0
) (
    // UPS0 follows the active trunk format; UPS1 keeps its fixed ReLU zero.
    input wire write_phase_ready,
    output wire write_phase_reserve,
    output wire [8:0] write_phase_row,
    output wire [15:0] write_phase_addr,
    input wire [ACT_W-1:0] runtime_in_zero,
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    output reg                     busy,
    output reg                     done,
    output reg  [31:0]             cycles,

    // ---- the layer's quantisation ------------------------------------------
    input  wire [OC_LANES*M_W-1:0]     rq_m,
    input  wire [OC_LANES*SHIFT_W-1:0] rq_shift,
    input  wire [OC_LANES*ACC_W-1:0]   rq_bias,

    // ---- the input store, external, registered read -------------------------
    // One port per input pixel, word addressed by (pixel, IC group): the
    // scheduler addresses the pixel and the group is the slot's.
    //
    // **`in_stall` is the store's side of the handshake and it is not optional
    // once the store is a cache.**  A store made of RAM always answers, which is
    // why this engine had no ready; `control/d64_burst_prefetch.v` can miss, and
    // "a cache that cannot stall is a cache that returns the wrong word
    // quietly".  A read is **transferred** in the cycle `in_stall` is low, and
    // the word for that address appears on `in_rd_data` in the following cycle;
    // while it is high the engine holds the address **and the metadata that
    // describes the slot**, and replays the slot whole.
    //
    // **The hold is not an OR gate into `stall`**, which is what the integration
    // note asked for and what a naive build does: the schedule has already moved
    // past the slot it presented, so a freeze at that edge drops the refused read
    // and the slot reaches the MAC on the store's previous word -- a wrong
    // operand inside a right number of writes, with `done` still reached.
    // Section 6.9 of `docs/F3_INTERFACES_2026-10-07.md` is the contract and
    // `docs/TCONV_INPUT_STALL_2026-10-07.md` is the measurement and the tooth.
    // `SAB_NAIVE_STALL` is the wrong build, kept and gated.
    //
    // A build that predates this port leaves it floating; the scheduler reads `z`
    // as "no stall", the same convention the cache's region handshake uses for an
    // unconnected `region_req`.
    input  wire                    in_stall,
    output wire [IN_PORTS-1:0]             in_rd_en,
    output wire [IN_PORTS*IN_WORD_W-1:0]   in_rd_addr,
    input  wire [IN_PORTS*IC_PAR*IN_W-1:0] in_rd_data,

    // ---- the weight store, external, registered read ------------------------
    output wire                    w_rd_en,
    output wire [W_ADDR_W-1:0]     w_rd_addr,
    input  wire [IC_PAR*OC_LANES*WGT_W-1:0] w_rd_data,

    // ---- the two-row scratch, written with real data ------------------------
    output wire                    out_wr_en,
    output wire [OUT_ADDR_W-1:0]   out_wr_addr,
    output wire                    out_wr_row,
    // The absolute output row, 2*r + dy.  The scratch is two rows deep, so
    // `out_wr_addr` alone says which of the two a word lands in and not where it
    // belongs in the image -- a consumer would have to track the input row to
    // know, and a checker that infers it is guessing where the rollover is.
    output wire [8:0]              out_wr_row_abs,
    output wire [WORD_CH*OUT_W-1:0] out_wr_data,

    // ---- counters ----------------------------------------------------------
    output reg  [31:0]             rq_requests,
    output reg  [31:0]             rq_accepts,
    output reg  [31:0]             rq_wait_cycles,
    // Cycles a read was presented and the store refused it.  Rule 4 of section
    // 6.9 asks for the stall to be a number on a port rather than an inference:
    // **a stall cannot produce a completion** -- the schedule cannot pass a read
    // the store has not taken, so `done` implies every read was transferred --
    // and this is the number that says what the walk was held for.
    output reg  [31:0]             in_stall_cycles,
    output reg  [31:0]             rq_overrun_count,
    output reg  [31:0]             slot_count,
    output reg  [31:0]             out_write_count,
    output reg  [31:0]             mfifo_peak,

    // ---- the PE seam -------------------------------------------------------
    //
    // **Fixed network widths, not this engine's.**  A shared bank is built once
    // and every request carries its own `qmin`/`qmax`, so ups.1's 8-bit
    // saturation is a `qmax` of 255 on a 10-bit lane and the value is identical.
    // That is what lets one bank serve every shape instead of one bank each.
    //
    // **`pe_req_mode` is on the seam and that is a change from the trunk.**  The
    // trunk's external testbench drives the bank with `u_ext.mac_mode`, reached
    // hierarchically -- which works in simulation and is not a signal a
    // synthesised arbiter can read.  An owner that does not own its bank has to
    // say which shape it is, out loud.
    output wire                      pe_req_slot_valid,
    output wire [PE_LANES*PE_ACT_W-1:0] pe_req_act,
    output wire [PE_LANES*WGT_W-1:0] pe_req_wgt,
    output wire                      pe_req_pix_start,
    output wire [PE_LANES*ACC_W-1:0] pe_req_bias,
    output wire [1:0]                pe_req_mode,
    output wire                      pe_req_rq_start,
    output wire [PE_OC_MAX*M_W-1:0]  pe_req_rq_m,
    output wire [PE_OC_MAX*SHIFT_W-1:0] pe_req_rq_shift,
    output wire [PE_OUT_W-1:0]       pe_req_rq_zero,
    output wire [PE_OUT_W-1:0]       pe_req_rq_qmin,
    output wire [PE_OUT_W-1:0]       pe_req_rq_qmax,
    input  wire                      pe_rsp_rq_ready,
    input  wire                      pe_rsp_rq_ready_n,
    input  wire [RQ_LANES*PE_OUT_W-1:0] pe_rsp_y,
    input  wire [RQ_LANES-1:0]       pe_rsp_y_valid,
    input  wire                      pe_rsp_rq_busy,
    input  wire                      pe_rsp_rq_done,
    input  wire                      pe_rsp_rq_overrun,
    input  wire [31:0]               pe_rsp_mode_viol,
    input  wire [PE_LANES*ACC_W-1:0] pe_rsp_acc,
    input  wire [PE_LANES*ACC_W-1:0] pe_rsp_snapshot,
    output wire                      mac_mode_safe
);

    localparam OC_WORDS = (OC_LANES + WORD_CH - 1) / WORD_CH;
    localparam NPIX     = PIX_PAR * OC_LANES;
    localparam RQ_PASS  = (NPIX + RQ_LANES - 1) / RQ_LANES;
    localparam RQW      = $clog2(RQ_PASS + 1);
    localparam OW       = 2 * W;    // the transposed convolution doubles the grid

    //: Requests whose passes have not all come back.  Two would almost do -- at
    //: ups.1 the accept rate is one per two cycles and a request's passes arrive
    //: on the two cycles after it -- but "almost" is not a pipeline depth, and
    //: four costs three flops a field.
    localparam MFIFO = 4;
    localparam MFW   = 2;

    // ------------------------------------------------------------------
    // the schedule
    // ------------------------------------------------------------------

    wire [IN_PORTS-1:0]           s_in_rd_en, s_in_rd_valid;
    wire [IN_PORTS*IN_ADDR_W-1:0] s_in_rd_addr;
    wire                    s_w_rd_en;
    wire [W_ADDR_W-1:0]     s_w_rd_addr;
    wire [1:0]              s_ph, s_tap;
    wire [3:0]              s_icg;
    wire [7:0]              s_r, s_c;
    wire [31:0]             s_cycles;
    wire                    s_live, s_last, s_next_last, s_issue, s_dy;
    wire                    s_busy, s_done;

    // Declared before use and driven later: an identifier ModelSim has not
    // yet seen becomes an implicit 1-bit net, which reads as a different
    // signal and not as something worth stopping for.
    wire mac_ready, mac_ready_n, prime_seen;
    reg  prime_done;

    // `s_next_last` is in scheduler time; the slot it warns about reaches the MAC
    // three cycles later.  Testing the present `rq_ready` against it froze the
    // schedule once per input position for a conflict that had already resolved --
    // 4095 cycles of stall in a design that needs none.
    wire mac_stall = s_next_last & ~mac_ready_n;

    // **The store's refusal is a hold, not a freeze.**  `stall` stops the
    // schedule; `sched_hold` says that the slot already on the wires has not been
    // taken, so the scheduler must keep its address *and its metadata* and present
    // it again.  ORing the store's `in_stall` into `stall` alone -- which is what
    // the integration note asked for -- loses the refused read, and
    // `SAB_NAIVE_STALL` is that build.
    //
    // Read through a copy, so that an input left floating by a build that
    // predates this port is "no stall": **a `case` and not an `if`**, because a
    // combinational `if` on a `z` leaves its target `x` and this one feeds the
    // scheduler.  `rst_n` and `start` are in the sensitivity list so that the
    // block settles even when `in_stall` is a constant `z` -- an `always @*`
    // whose only input never changes is never evaluated -- and they are signals
    // this block cannot be made to depend on, which a scheduler output would be.
    reg store_hold;
    always @(in_stall or rst_n or start) begin
        case (in_stall)
            1'b1:    store_hold = 1'b1;
            default: store_hold = 1'b0;
        endcase
    end

    wire stall      = mac_stall | (SAB_NAIVE_STALL ? store_hold : 1'b0);
    wire sched_hold = SAB_NAIVE_STALL ? 1'b0 : store_hold;

    d64_tconv_scheduler #(
        .H(H), .W(W), .PIX_PAR(PIX_PAR), .IC_GROUPS(IC_GROUPS),
        .OC_GROUPS(OC_GROUPS), .OC_LANES(OC_LANES), .IN_PORTS(IN_PORTS),
        .WORD_CH(WORD_CH), .IN_ADDR_W(IN_ADDR_W), .W_BASE(W_BASE),
        .W_ADDR_W(W_ADDR_W), .OUT_W(OW), .OUT_ADDR_W(OUT_ADDR_W),
        .REQ_LANES(RQ_LANES)
    ) u_sched (
        .clk(clk), .rst_n(rst_n), .start(start), .busy(s_busy), .done(s_done),
        .cycles(s_cycles),
        .in_rd_en(s_in_rd_en), .in_rd_valid(s_in_rd_valid),
        .in_rd_addr(s_in_rd_addr),
        .w_rd_en(s_w_rd_en), .w_rd_addr(s_w_rd_addr),
        .out_wr_en(), .out_wr_addr(), .out_wr_row(),
        .rq_valid(), .rq_values(),
        .cur_phase(s_ph), .cur_tap(s_tap), .cur_icg(s_icg),
        .cur_r(s_r), .cur_c(s_c), .cur_kh(), .cur_kw(),
        .cur_in_col0(), .out_cur_addr(),
        .mac_cycles(), .in_reads(), .w_reads(), .out_writes(),
        .rq_total(), .rq_peak(), .border_slots(), .emitted(), .rq_pend_o(),
        .dbg_st(),
        .cur_live(s_live), .cur_last(s_last), .next_last(s_next_last),
        .write_phase_ready(WRITE_RESERVATION ? write_phase_ready : 1'b1),
        .write_phase_reserve(write_phase_reserve),.write_phase_row(write_phase_row),.write_phase_addr(write_phase_addr),
        .cur_issue(s_issue), .cur_dy(s_dy), .stall(stall),
        .in_stall(sched_hold)
    );

    // ------------------------------------------------------------------
    // stage A: the control that travels with the address
    // ------------------------------------------------------------------
    //
    // Keyed on `cur_issue` and not on the state: while `stall` is asserted the
    // scheduler is still in S_RUN and `cur_*` still describes the slot it has
    // already issued, so a state-keyed register would capture that slot twice
    // and the MAC would accumulate it twice.
    //
    // **And `cur_issue` qualifies the slot bits as well as the strobe.**  The
    // scheduler holds a refused slot whole -- address, enables, phase, tap,
    // group, coordinates, port mask and these three bits -- until the store
    // takes it, so during a hold `s_live` is 1 and `s_in_rd_valid` is the slot's
    // mask.  Stage A keeps clocking through the hold, and with these three bits
    // registered unqualified the MAC would be told `slot_valid` **on every
    // cycle of the hold** with the store's previous word under it: an
    // accumulator full of the wrong operand, which the phase's own bias load
    // then destroys (`rq_overrun`).  A slot the store has not taken is not a
    // slot, and stage A is where that is decided -- on the same edge the
    // address is taken, so there is no cycle of lag and nothing to un-say.

    reg                a_slot, a_live, a_last, a_dy;
    reg [7:0]          a_r, a_c;
    reg [1:0]          a_ph;
    reg [IN_PORTS-1:0] a_pv;              // per-port: this pixel is real

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_slot <= 1'b0; a_live <= 1'b0; a_last <= 1'b0; a_dy <= 1'b0;
            a_r <= 8'd0; a_c <= 8'd0; a_ph <= 2'd0; a_pv <= {IN_PORTS{1'b0}};
        end
        else begin
            a_slot <= s_issue;
            a_live <= s_live & s_issue;
            a_last <= s_last & s_issue;
            a_dy   <= s_dy   & s_issue;
            a_r    <= s_r;
            a_c    <= s_c;
            a_ph   <= s_ph;
            a_pv   <= s_in_rd_valid;
        end
    end

    // ------------------------------------------------------------------
    // the read path: the stored value turned into an operand
    // ------------------------------------------------------------------

    // Per port: the scheduler addresses the input pixel, the store is word
    // addressed by (pixel, IC group).
    wire [IN_WORD_W-1:0] icg_w = s_icg;

    genvar gp;
    generate
        for (gp = 0; gp < IN_PORTS; gp = gp + 1) begin : g_addr
            // `s_icg` is four bits; selecting it at `IN_ADDR_W` reads past the
            // end and yields x, which makes the whole address x and the store
            // answer with an x word.
            assign in_rd_addr[gp*IN_WORD_W +: IN_WORD_W] =
                (s_in_rd_addr[gp*IN_ADDR_W +: IN_ADDR_W] * IC_GROUPS) + icg_w;
        end
    endgenerate

    assign in_rd_en  = s_in_rd_en;
    assign w_rd_en   = s_w_rd_en;
    assign w_rd_addr = s_w_rd_addr;

    // T1.  The store's data one cycle early while the control is not: the mirror
    // of the skew the pipeline exists to prevent.
    reg [IN_PORTS*IC_PAR*IN_W-1:0] in_late;
    always @(posedge clk) if (SKEW_SABOTAGE) in_late <= in_rd_data;
    wire [IN_PORTS*IC_PAR*IN_W-1:0] in_v = SKEW_SABOTAGE ? in_late : in_rd_data;

    // `q - zero`, widened to the operand width the arithmetic needs.  The trunk
    // stores 10 bits and its operand is 11 signed; a ReLU tensor stores 8 and its
    // operand is 9.  A port whose pixel is off the image contributes zero -- not
    // the store's residual word, which is the previous slot's data and would
    // multiply into this accumulator as if it belonged to it.
    reg [PIX_PAR*IC_PAR*ACT_W-1:0] act_bus;
    integer ai, ap;
    reg signed [ACT_W-1:0] act_one;
    wire [ACT_W-1:0] input_zero_live = RUNTIME_IN_ZERO ? runtime_in_zero : IN_ZERO[ACT_W-1:0];
    reg                    port_ok;

    always @* begin
        act_bus = {PIX_PAR*IC_PAR*ACT_W{1'b0}};
        for (ap = 0; ap < PIX_PAR; ap = ap + 1) begin
            port_ok = (ap < IN_PORTS) ? a_pv[ap] : 1'b0;
            if (PORT1_SABOTAGE && ap == 1) port_ok = (ap < IN_PORTS);
            for (ai = 0; ai < IC_PAR; ai = ai + 1) begin
                act_one = $signed({1'b0, in_v[(ap*IC_PAR + ai)*IN_W +: IN_W]})
                        - input_zero_live;
                act_bus[(ap*IC_PAR + ai)*ACT_W +: ACT_W] =
                    port_ok ? act_one : {ACT_W{1'b0}};
            end
        end
    end

    // ------------------------------------------------------------------
    // the accept, and the phase rollover
    // ------------------------------------------------------------------

    reg [PIX_PAR*IC_PAR*ACT_W-1:0] b_act;
    reg [IC_PAR*OC_LANES*WGT_W-1:0] b_wgt;
    reg                b_slot, b_live, b_last, b_dy, b_prime;
    reg [7:0]          b_r, b_c;
    reg [1:0]          b_ph;


    wire prime_now = prime_seen & ~prime_done & s_issue;
    wire b_accept  = b_slot & b_last & mac_ready;
    // `b_prime` is `prime_now` one cycle later, which is what keeps it exactly
    // one cycle ahead of the first `b_live` -- the bias has to be loaded before
    // the phase's first slot and not on it.
    wire pix_start = b_accept | b_prime;

    // ------------------------------------------------------------------
    // stage B: the operand, registered
    // ------------------------------------------------------------------
    //
    // **D3c4c-d.**  Stage A carries the control beside the address, and the store
    // answers with data a cycle later, so `act_bus` -- the store's output, the
    // `q - zero` subtract and the port mask -- ran combinationally into 128
    // multipliers.  The window RAM's clock-to-out plus that subtract plus the
    // routing is what the placement report named as the critical path, and it
    // named it on both layers:
    //
    //     ups.0 worst seed  g_win[0].u_win/mem__D$12|RCLK -> ...  +0.073 ns
    //     ups.1 worst seed  g_win[0].u_win/mem__D$c1|RCLK -> ...  +0.109 ns
    //
    // against a preferred +0.25.  So the operand gets a register of its own and
    // the path becomes store -> register, then register -> multiplier.
    //
    // **What is registered is everything the MAC sees, not just the activation.**
    // Registering `act` alone and leaving `slot_valid`, `pix_start` and the
    // accept in stage A would put the control a cycle ahead of the data it
    // qualifies: the MAC would take the previous slot's operand under this slot's
    // phase framing, which is a wrong tensor with a right write count.  So the
    // weight travels with the activation and so does the control, and the accept
    // -- which is the MAC's `rq_start` and half of its `pix_start` -- moves with
    // them.
    //
    // One consequence is not local: the scheduler warns one slot early and asks
    // whether the requant will be ready when that slot *arrives*, so the depth
    // between the warning and the multiplier is part of the MAC's contract.
    // It was three and is now four, and `RQ_LOOKAHEAD` carries that up to the
    // bank.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_act <= {PIX_PAR*IC_PAR*ACT_W{1'b0}};
            if(EXTERNAL_PE==0) b_wgt <= {IC_PAR*OC_LANES*WGT_W{1'b0}};
            b_slot <= 1'b0; b_live <= 1'b0; b_last <= 1'b0; b_dy <= 1'b0;
            b_prime <= 1'b0;
            b_r <= 8'd0; b_c <= 8'd0; b_ph <= 2'd0;
        end
        else begin
            b_act   <= act_bus;
            if(EXTERNAL_PE==0) b_wgt   <= w_rd_data;
            b_slot  <= a_slot;
            b_live  <= a_live;
            b_last  <= a_last;
            b_dy    <= a_dy;
            b_r     <= a_r;
            b_c     <= a_c;
            b_ph    <= a_ph;
            b_prime <= prime_now;
        end
    end


    // The layer's first phase has no preceding accept to roll over from, so it
    // is primed on the first slot the scheduler issues -- once, and once only.
    // Keying on the rising edge of the slot stream would re-prime after a stall,
    // reloading the bias and wiping the phase being accumulated.
    always @(posedge clk or negedge rst_n)
        if (!rst_n) prime_done <= 1'b0;
        else if (start) prime_done <= 1'b0;
        else if (prime_now) prime_done <= 1'b1;

    reg s_issue_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) s_issue_d <= 1'b0; else s_issue_d <= s_issue;
    assign prime_seen = s_issue | s_issue_d;

    // ------------------------------------------------------------------
    // the MAC
    // ------------------------------------------------------------------

    wire [RQ_LANES*OUT_W-1:0] mac_y;
    wire [RQ_LANES-1:0]       mac_y_valid;
    wire                      mac_overrun, mac_busy;

    // **The adapter split from its bank.**  `d64_mac128_compat` is the map plus
    // a local bank, and this engine used the whole thing; the map alone is what
    // a client needs when it is going to drive a bank someone else owns.  The
    // trunk's engine made this split in D3c6-e and this is the same seam with
    // one addition -- `pe_req_mode` -- because an arbiter cannot reach into a
    // peer's hierarchy the way a testbench can.
    wire [PE_LANES*ACT_W-1:0]     bank_act;
    wire [PE_LANES*WGT_W-1:0]     bank_wgt;
    wire [PE_LANES*ACC_W-1:0]     bank_bias;
    wire [PE_OC_MAX*M_W-1:0]      bank_m;
    wire [PE_OC_MAX*SHIFT_W-1:0]  bank_shift;
    wire [1:0]                    bank_mode;
    wire [OUT_W-1:0]              bank_qmin, bank_qmax;

    // Zero-extends to the seam's width whatever this engine's is.  Written as an
    // assignment through a wider function return rather than as a replication,
    // because `{{0{1'b0}}, v}` is not legal Verilog when the widths are equal
    // and ups.1 would be exactly that case if its output were ever widened.
    function [PE_OUT_W-1:0] seam_w;
        input [OUT_W-1:0] v;
        begin
            seam_w = v;
        end
    endfunction

    d64_mac128_compat_map #(
        .IC_PAR(IC_PAR), .OC_LANES(OC_LANES), .PIX_PAR(PIX_PAR),
        .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W), .SHIFT_W(SHIFT_W),
        .OUT_W(OUT_W), .RQ_LANES(RQ_LANES), .QMIN(QMIN), .QMAX(QMAX),
        .LANES(PE_LANES), .OC_MAX(PE_OC_MAX)
    ) u_map (
        .act(EXTERNAL_PE ? act_bus : b_act), .wgt(b_wgt), .bias({PIX_PAR{rq_bias}}),
        .rq_m(rq_m), .rq_shift(rq_shift),
        .b_act(bank_act), .b_wgt(bank_wgt), .b_bias(bank_bias),
        .b_m(bank_m), .b_shift(bank_shift), .b_mode(bank_mode),
        .b_qmin(bank_qmin), .b_qmax(bank_qmax),
        .b_acc(), .b_snapshot(),
        .acc_o(), .rq_snap_o()
    );

    // **A sign extension, not a zero extension.**  A lane is `q - zero` and is
    // read signed inside the bank; padding it with zeros turns every negative
    // operand into a large positive one.  `d64_mac128`'s own header records
    // that exact bug costing a build, so it is spelled lane by lane here rather
    // than left to a concatenation.
    genvar sl_a;
    generate
        for (sl_a = 0; sl_a < PE_LANES; sl_a = sl_a + 1) begin : g_act
            if (ACT_W < PE_ACT_W)
                assign pe_req_act[sl_a*PE_ACT_W +: PE_ACT_W] =
                    {{(PE_ACT_W-ACT_W){bank_act[sl_a*ACT_W + ACT_W - 1]}},
                     bank_act[sl_a*ACT_W +: ACT_W]};
            else
                assign pe_req_act[sl_a*PE_ACT_W +: PE_ACT_W] =
                    bank_act[sl_a*ACT_W +: PE_ACT_W];
        end
    endgenerate

    assign pe_req_slot_valid = b_live;
    assign pe_req_wgt        = bank_wgt;
    assign pe_req_wgt        = bank_wgt;
    assign pe_req_pix_start  = pix_start;
    assign pe_req_bias       = bank_bias;
    assign pe_req_mode       = bank_mode;
    assign pe_req_rq_start   = b_accept;
    assign pe_req_rq_m       = bank_m;
    assign pe_req_rq_shift   = bank_shift;
    assign pe_req_rq_zero    = seam_w({OUT_W{1'b0}});
    assign pe_req_rq_qmin    = seam_w(bank_qmin);
    assign pe_req_rq_qmax    = seam_w(bank_qmax);
    // Every valid, read and output drained, then it is safe to change `mode`.
    // A slot in flight under one shape means nothing under another.
    assign mac_mode_safe     = !b_live && !mac_busy;

    generate
        if (EXTERNAL_PE != 0) begin : g_external_pe
            assign mac_ready   = pe_rsp_rq_ready;
            assign mac_ready_n = pe_rsp_rq_ready_n;
            assign mac_busy    = pe_rsp_rq_busy;
            assign mac_overrun = pe_rsp_rq_overrun;
            // The seam's lane is `PE_OUT_W` wide and this engine's is `OUT_W`.
            // Saturation already happened in the bank against the `qmax` this
            // engine asked for, so the low bits are the value and not a
            // truncation of it.
            genvar sl;
            for (sl = 0; sl < RQ_LANES; sl = sl + 1) begin : g_y
                assign mac_y[sl*OUT_W +: OUT_W] =
                    pe_rsp_y[sl*PE_OUT_W +: OUT_W];
            end
            assign mac_y_valid = pe_rsp_y_valid;
            // `pe_rsp_mode_viol` is the bank's contract with whoever drives
            // `mode`, which is the owner and not this engine.
        end else begin : g_local_pe
            d64_mac128 #(
                .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .M_W(M_W),
                .SHIFT_W(SHIFT_W), .OUT_W(OUT_W), .RQ_LANES(RQ_LANES),
                .LANES(PE_LANES), .OC_MAX(PE_OC_MAX),
                .ROLLOVER_SABOTAGE(ROLLOVER_SABOTAGE),
                .RQ_LOOKAHEAD(RQ_LOOKAHEAD)
            ) u_bank (
                .clk(clk), .rst_n(rst_n), .mode(bank_mode),
                .slot_valid(b_live), .act(bank_act), .wgt(bank_wgt),
                .pix_start(pix_start), .bias(bank_bias),
                .rq_start(b_accept), .rq_ready(mac_ready),
                .rq_ready_n(mac_ready_n), .rq_m(bank_m), .rq_shift(bank_shift),
                .rq_zero({OUT_W{1'b0}}),
                .rq_qmin(bank_qmin), .rq_qmax(bank_qmax),
                .y(mac_y), .y_valid(mac_y_valid), .rq_busy(mac_busy),
                .rq_done(), .rq_overrun(mac_overrun), .mode_viol(),
                .acc_o(), .rq_snap_o()
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // the request metadata: a queue, not a register
    // ------------------------------------------------------------------
    //
    // **The coordinates travel with the request.**  `y_valid` arrives several
    // cycles after the accept, by which time the scheduler is phases ahead, so
    // reading its live coordinates writes the right number of words to the wrong
    // places.
    //
    // A single register is not enough even with the coordinates latched.  At
    // ups.1 the accept rate is one per two cycles and each request has two
    // passes, so request N's last pass and request N+1's accept land on the same
    // cycle: the register would take the new coordinates while the old request's
    // final word was still being addressed.  The value would be request N's and
    // the address request N+1's -- a wrong image with a right write count.

    reg [8:0]     mf_rowabs [0:MFIFO-1];
    reg [7:0]     mf_r    [0:MFIFO-1];
    reg [7:0]     mf_c    [0:MFIFO-1];
    reg           mf_row  [0:MFIFO-1];
    reg           mf_odx  [0:MFIFO-1];
    reg [RQW-1:0] mf_pass [0:MFIFO-1];
    reg [MFW-1:0] mf_head, mf_tail;
    reg [MFW:0]   mf_n;

    wire mf_pop  = (|mac_y_valid) && (mf_pass[mf_head] == RQ_PASS[RQW-1:0] - 1'b1);
    wire mf_push = b_accept && (mf_n != MFIFO[MFW:0]);

    integer mi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mf_head <= {MFW{1'b0}}; mf_tail <= {MFW{1'b0}}; mf_n <= {MFW+1{1'b0}};
            for (mi = 0; mi < MFIFO; mi = mi + 1) begin
                mf_rowabs[mi] <= 9'd0;
                mf_r[mi] <= 8'd0; mf_c[mi] <= 8'd0;
                mf_row[mi] <= 1'b0; mf_odx[mi] <= 1'b0; mf_pass[mi] <= {RQW{1'b0}};
            end
        end
        else begin
            if (mf_push) begin
                // Stage B's coordinates, because the push is stage B's accept:
                // `b_r`/`b_c` are `a_r`/`a_c` one cycle later and describe the
                // same phase, while `a_*` at that cycle already belongs to the
                // slot that follows it.
                mf_rowabs[mf_tail] <= {b_r, 1'b0} + {8'd0, b_dy};
                mf_r[mf_tail]    <= META_SABOTAGE ? s_r : b_r;
                mf_c[mf_tail]    <= META_SABOTAGE ? s_c : b_c;
                mf_row[mf_tail]  <= META_SABOTAGE ? 1'b0 : b_dy;
                mf_odx[mf_tail]  <= META_SABOTAGE ? s_ph[0] : b_ph[0];
                mf_pass[mf_tail] <= {RQW{1'b0}};
                mf_tail <= mf_tail + 1'b1;
            end
            // Independent of the push, and on the head rather than the tail.  A
            // pass arriving on the same cycle as the next request's accept is the
            // normal case at ups.1 -- the accept rate is one per two cycles and a
            // request is two passes -- and as an `else if` the increment lost to
            // every one of those, so the head's pass count stayed at zero and
            // every request wrote its first word twice.
            if (|mac_y_valid)
                mf_pass[mf_head] <= mf_pass[mf_head] + 1'b1;

            if (mf_pop) mf_head <= mf_head + 1'b1;

            if (mf_push && !mf_pop)      mf_n <= mf_n + 1'b1;
            else if (mf_pop && !mf_push) mf_n <= mf_n - 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // the scratch write
    // ------------------------------------------------------------------
    //
    // The pass index is the output word index, and both layers' mappings fall
    // out of one expression:
    //
    //   ups.0  PIX1 OC16  pass0 -> pixel P, channels 0..7   pass1 -> channels 8..15
    //   ups.1  PIX2 OC8   pass0 -> pixel P0, channels 0..7  pass1 -> pixel P1
    //
    // This restates the scheduler's own `out_pix`/`out_word` with the pass in
    // place of its `out_idx`, so the two cannot drift apart without one changing.

    wire [15:0] mp     = {{(16-RQW){1'b0}}, mf_pass[mf_head]};
    wire [15:0] m_pix  = {8'd0, mf_c[mf_head]} + (mp / OC_WORDS);
    wire [15:0] m_word = (((m_pix << 1) + {15'd0, mf_odx[mf_head]}) * OC_WORDS)
                       + (mp % OC_WORDS);

    assign out_wr_en   = |mac_y_valid;
    assign out_wr_addr = m_word[OUT_ADDR_W-1:0];
    assign out_wr_row  = mf_row[mf_head];
    assign out_wr_row_abs = mf_rowabs[mf_head];
    assign out_wr_data = mac_y;

    // ------------------------------------------------------------------
    // the engine's own completion
    // ------------------------------------------------------------------
    //
    // The scheduler's `done` is the end of *its* work, which now sits several
    // pipeline stages upstream of the end of the engine's.  Reporting it would
    // undercount the drain and, worse, would let a whole-network controller
    // switch layers while the last outputs were still in flight.

    // `s_done` is a one-cycle pulse and the drain outlasts it, so it has to be
    // latched: testing it live means the completion is only seen if the
    // pipeline happens to be empty on that exact cycle, which it is not.
    reg  sdone_seen;
    wire fin = sdone_seen && (mf_n == {MFW+1{1'b0}}) && !(|mac_y_valid) && !mac_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sdone_seen <= 1'b0;
        else if (start) sdone_seen <= 1'b0;
        else if (s_done) sdone_seen <= 1'b1;
    end

    reg running;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running <= 1'b0; cycles <= 32'd0; busy <= 1'b0; done <= 1'b0;
        end
        else begin
            done <= 1'b0;
            if (start) begin running <= 1'b1; cycles <= 32'd0; busy <= 1'b1; end
            else if (running) begin
                if (fin) begin
                    running <= 1'b0; busy <= 1'b0; done <= 1'b1;
                end
                else
                    cycles <= cycles + 32'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // counters
    // ------------------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rq_requests <= 32'd0; rq_accepts <= 32'd0;
            rq_wait_cycles <= 32'd0; rq_overrun_count <= 32'd0;
            in_stall_cycles <= 32'd0;
            slot_count <= 32'd0; out_write_count <= 32'd0; mfifo_peak <= 32'd0;
        end
        else if (start) begin
            rq_requests <= 32'd0; rq_accepts <= 32'd0;
            rq_wait_cycles <= 32'd0; rq_overrun_count <= 32'd0;
            in_stall_cycles <= 32'd0;
            slot_count <= 32'd0; out_write_count <= 32'd0; mfifo_peak <= 32'd0;
        end
        else begin
            if (a_slot & a_last) rq_requests    <= rq_requests + 32'd1;
            // **Stage B's accept.**  The request is still made at stage A -- that
            // is when the phase's slots run out -- but the accept is what the MAC
            // took, and it moved a stage with the operand.  Counting `a_accept`
            // here reported 24,577 accepts for 32,768 requests on a layer whose
            // tensor was exact, because at stage A the MAC is still a phase
            // behind and its `rq_ready` is often low.
            if (b_accept)        rq_accepts     <= rq_accepts + 32'd1;
            if (mac_stall)       rq_wait_cycles <= rq_wait_cycles + 32'd1;
            // **The store's refusal, visible as a number.**  Rule 4 of section
            // 6.9: a stall must never produce a completion -- it cannot, because
            // the schedule cannot pass a read the store has not taken -- and the
            // cycles it cost are reported rather than inferred from a difference
            // in `cycles`.
            //
            // A read presented and held, which is the same condition the cache
            // counts its own `stall_cycles` on: the toolchain has one definition
            // of this number and this is it.
            if ((|in_rd_en) && store_hold)
                                 in_stall_cycles <= in_stall_cycles + 32'd1;
            if (a_live)          slot_count     <= slot_count + 32'd1;
            if (|mac_y_valid)    out_write_count <= out_write_count + 32'd1;
            if (mac_overrun)     rq_overrun_count <= FAULT_STICKY ? 32'd1 : rq_overrun_count + 32'd1;
            if ({1'b0, mf_n} > mfifo_peak[MFW:0])
                mfifo_peak <= {{(32-MFW-1){1'b0}}, mf_n};
        end
    end

    // External bank consumes weights only with its valid slot. Sampling the
    // common response every clock, with no owner-specific mask/reset, allows
    // equivalent raw response bits to share FFs across mutually exclusive engines.
    // The standalone backend retains its original reset and capture contract.
    generate if(EXTERNAL_PE!=0)begin:g_shared_weight_capture
      always @(posedge clk) b_wgt <= w_rd_data;
    end endgenerate

    // synthesis translate_off
    wire [128*PE_ACT_W-1:0] act_registered_ghost;
    genvar ga;generate for(ga=0;ga<128;ga=ga+1)begin:g_act_ghost
      localparam AP=((ga/IC_PAR)/OC_LANES)*IC_PAR+(ga%IC_PAR);
      assign act_registered_ghost[ga*PE_ACT_W+:PE_ACT_W]={{(PE_ACT_W-ACT_W){b_act[AP*ACT_W+ACT_W-1]}},b_act[AP*ACT_W+:ACT_W]};
    end endgenerate
    // synthesis translate_on
endmodule
