`timescale 1ns/1ps

// D3c6-a2 -- one physical 128-multiplier bank that repartitions at runtime.
//
// The frozen budget in `docs/D64_60FPS_BUDGET.md` gives every layer a shape and
// every shape is a factorisation of 128.  What `tools/d64_encoder_packing_dse.py`
// measured is that those factorisations are **not** interchangeable at build
// time: `d64_conv_engine` physically builds `PIX 1 / IC 8 / OC 16`, downs.0 and
// downs.1 are exactly 100% on it, and head is 18.75% -- 589,824 cycles against a
// 110,592 budget.  head needs `IC_PAR 1`, which is different operand widths,
// different lane wiring and 128 accumulators against 16, and a second 128-
// multiplier array does not fit: 144 + 144 against 160 DSP blocks.
//
// So the array is built once and repartitioned.  Four modes, and every one of
// them is `PIX x IC x OC = 128`:
//
//     MAC_P1_I8_O16   the trunk, downs.0, downs.1, ups.0    16 accumulators
//     MAC_P2_I8_O8    ups.1                                 16 accumulators
//     MAC_P4_I8_O4    tail                                  16 accumulators
//     MAC_P4_I4_O8    head                                  32 accumulators
//
// The first three are the same machine -- sixteen 8-input reduction trees, 16
// accumulators -- and differ only in how the activation operand is spread over
// the pixel lanes.  head is the odd one: it reduces **four** input channels --
// R, G, B and a zero -- rather than eight, so it carries 32 accumulators.
//
// **head used to be `MAC_P16_I1_O8` -- sixteen pixels, no reduction at all --
// and that shape is the reason this array placed 128 accumulators.**  Sixteen
// pixels x eight output channels is 128 live results, and the array kept 128
// accumulators and 128 snapshots whatever the mode was, because a runtime
// accumulator count is a mux in front of every requant operand.
//
// The deployed head layer reads three input channels.  Four pixels x four input
// channels x eight output channels is still 128 multipliers, and the live
// result count is 4 x 8 = 32 -- so the mux `MAC_P16_I1_O8` was avoiding no
// longer exists to avoid, and the registers come out.  **The tree does not
// move**: `IC 4` is `l2`, a level the shared heap already had.
//
// The cost is cycles and it is exact.  The fourth input channel is zero, so a
// head slot does 96 useful multiplies in 128 and the layer's arithmetic goes
// from 110,592 slots to 147,456.  The requant moves the other way -- 32
// accumulators is four passes of eight lanes against the old sixteen -- so the
// wall clock does not grow by the slot ratio.
//
// Layout, and why it makes the tree fixed
// ---------------------------------------
//
// Multiplier `m` is numbered `(pixel, output channel, input channel)` with the
// **reduction innermost**:
//
//     m = (p*OC + o)*IC + i        accumulator k = p*OC + o
//
// so one accumulator's products are `m = k*IC .. k*IC+IC-1`, contiguous, and the
// reduction tree is a plain pairwise heap over adjacent multipliers with no
// per-mode regrouping.  `IC` is 8 or 1 and the heap's four levels cover 1, 2, 4
// and 8, so the mode picks a level and nothing else moves:
//
//     L0 = product[m]            IC 1
//     L1 = pairs                 IC 2
//     L2 = sums of 4             IC 4
//     L3 = sums of 8             IC 8
//
// The other three layouts the array would need -- pixel-major, or output channel
// innermost -- are all reachable by renumbering the *operands*, which is wiring
// in the caller and costs no logic.  That is the whole repartition: `act` and
// `wgt` arrive one lane per multiplier, in whatever order the mode wants, and
// the caller broadcasts.  For `MAC_P1_I8_O16` the trunk's own order is already
// this order -- the reference numbers products `(PP, II)` and both layouts put
// `II` innermost -- so the trunk's adapter is a pure permutation.
//
// 32 accumulators and 32 snapshots, whatever the mode
// ---------------------------------------------------
//
// `MAC_P4_I4_O8` is the widest live set -- 4 x 8 -- and the others use the
// first 16, so `ACC_N` is 32.  The array used to be 128 deep for head's old
// `P16 I1 O8` shape; `tools/d64_encoder_packing_dse.py` measured what that set
// costs on its own -- `PIX16 IC1 OC8` is 8432 FF and 15,789 LUT against 1238
// and 3557 for the trunk's shape, and **144 DSP either way** -- which is the
// bill this shape change is here to stop paying.
//
// The reduction is not the same story and is not built twice: the four levels
// are shared and `mode` selects one, so `IC 2` is still there for free the day
// a layer wants it.
//
// The observation ports stay `LANES` wide and the lanes past `ACC_N` read
// zero.  **Nothing in the network reads above lane 15** -- the P7 capture takes
// `mac_acc[63:0]`, the trunk's debug takes `mac_snapshot[511:0]` -- so those
// bits are constant and the mapper drops them.  Narrowing the ports instead
// would push a bus width through `d64_mac128_compat_map`, `d64_conv_engine`,
// `d64_down_engine`, `d64_encoder` and `d64_encoder_trunk` to save silicon
// that is already saved: the registers are what cost, and they are `ACC_N`
// deep now.
//
// What is deliberately not here
// -----------------------------
//
// No schedule, no address generation, no memory.  This is the array and its
// requant, and the differential testbench in `tb_d64_mac128.v` holds it against
// the parameterised `d64_tconv_mac` it generalises -- the same reference the
// trunk, ups.0 and ups.1 were all closed against -- so that a later change to
// the backend does not have to be localised by running a network.
module d64_mac128 #(
    // Legacy modules retain exact counts; the final root may select sticky faults.
    parameter integer FAULT_STICKY = 0,
    parameter ACT_W   = 11,       // the widest operand in the network, fixed
    parameter WGT_W   = 8,
    parameter ACC_W   = 32,
    parameter M_W     = 16,
    parameter SHIFT_W = 5,
    parameter OUT_W   = 10,       // the trunk's stored width, which conv3 writes
    parameter RQ_LANES = 8,
    parameter LANES   = 128,      // multipliers
    parameter ACC_N   = 32,       // accumulator slots: the widest mode is 4x8
    parameter OC_MAX  = 16,       // the widest output-channel group
    // Verification only, folded away at 0.  At 1 `done_acc` omits the final
    // slot, which is what the priority `if (pix_start) ... else if (slot_valid)`
    // did on its own before the reference separated them -- so the rollover edge
    // silently drops the last MAC contribution instead of folding it in.
    // `d64_mac128_compat` passes this through because `tb_d64_tconv_engine`
    // builds the variant and requires it to **fail**; an adapter that dropped it
    // would make the sabotaged build pass, which is a tooth that does not fire.
    parameter integer ROLLOVER_SABOTAGE = 0,
    // How many cycles ahead `rq_ready_n` predicts.  It is a property of the
    // **engine** and not of the array: the scheduler warns on `next_last` and
    // asks whether the requant will be ready when that slot arrives, so the
    // number has to equal the depth between the warning and the multiplier.
    // D3c4c-d put one more register in that path and the tconv engine passes 4;
    // the trunk's warning and its multiplier have not moved, so it keeps 3.
    parameter integer RQ_LOOKAHEAD = 3,
    parameter integer ACT_INPUT_REG = 0
) (
    input  wire                      clk,
    input  wire                      rst_n,

    // ---- the repartition ---------------------------------------------------
    // MAC_P1_I8_O16 / MAC_P2_I8_O8 / MAC_P4_I8_O4 / MAC_P4_I4_O8.  A layer
    // changes it between phases, never inside one: every mode is `128`
    // multipliers and the operand lanes are renumbered, so a slot in flight
    // under one mode means nothing under another.
    input  wire [1:0]                mode,

    // ---- the array operand -------------------------------------------------
    // One lane a multiplier, in the mode's own order.  The caller broadcasts:
    // `MAC_P1_I8_O16` presents eight activations each repeated sixteen times and
    // 128 weights once each; `MAC_P4_I4_O8` presents four pixels' four input
    // channels repeated eight times and eight output channels' four weights
    // repeated four times.  That
    // replication is wiring and not logic, which is why it is on this side of
    // the port.
    input  wire                      slot_valid,
    input  wire [LANES*ACT_W-1:0]    act,
    input  wire [LANES*WGT_W-1:0]    wgt,

    // ---- pixel framing -----------------------------------------------------
    // `pix_start` preloads the bias, which is what makes the exposed accumulator
    // the bias-included one the frozen digest covers.  It has priority over
    // `slot_valid`, so an edge carrying both must be qualified with `rq_accept`
    // or the phase's final contribution is destroyed -- `rq_overrun` reports
    // exactly that and does not prevent it.
    input  wire                      pix_start,
    input  wire [LANES*ACC_W-1:0]    bias,

    input  wire                      rq_start,
    output wire                      rq_ready,
    output wire                      rq_ready_n,
    // Per output channel, so these are indexed by the accumulator's channel and
    // not by its lane: `OC_MAX` entries and the mode picks the slice.
    input  wire [OC_MAX*M_W-1:0]     rq_m,
    input  wire [OC_MAX*SHIFT_W-1:0] rq_shift,
    input  wire signed [OUT_W-1:0]   rq_zero,
    // **Runtime, and that is the generalisation that matters.**  In the
    // parameterised MAC these are elaboration constants, which is why a trunk
    // tensor with zero point 413 and a head tensor with zero point 0 are two
    // different builds.  Here one clamp covers both -- and `downs.1`, which is
    // the case worth naming: it writes the trunk's U10 format, so its zero point
    // is 413, and it is still a ReLU, so its *low* clamp is 413 and not 0.
    input  wire [OUT_W-1:0]          rq_qmin,
    input  wire [OUT_W-1:0]          rq_qmax,

    output reg  [RQ_LANES*OUT_W-1:0] y,
    output reg  [RQ_LANES-1:0]       y_valid,
    output reg                       rq_busy,
    output reg                       rq_done,
    output reg                       rq_overrun,
    // **`mode` may only change while the pipeline is idle.**  It is combinational
    // control over three things -- the reduction depth, how many accumulators are
    // live, and how many requant passes a phase takes -- so a change mid-phase
    // reinterprets a slot already in flight under the new shape's numbering and
    // silently requantises the wrong number of accumulators.  `tb_d64_mac128`
    // proves a *drained* switch is safe; this is what says the integration does
    // the switch at a drained boundary rather than proving it could get away with
    // one that is not.  A violation is counted and not corrected: there is no
    // correct way to finish a phase whose shape changed underneath it.
    output reg  [31:0]               mode_viol,

    // ---- observability -----------------------------------------------------
    output wire [LANES*ACC_W-1:0]    acc_o,
    output wire [LANES*ACC_W-1:0]    rq_snap_o,

    // ---- observability: the requant's own operand registers, lane 0 --------
    //
    // **Why these are ports and not a hierarchical read.**  The P7r probe wants
    // the value the requant actually consumes -- `s_acc`, which the accept edge
    // loads from `done_acc` -- and `s_acc` has no reader outside this module.
    // Reading `u_bank.s_acc[0]` from the parent is legal Verilog and simulates
    // correctly, and the mapper **refuses it**:
    //
    //     [EFX-0010 VERI-ERROR] external connection to net 'acc_q[0][24]' is
    //     not allowed (VDB-1011)
    //
    // So a probe written that way is not a probe that gets folded -- it is a
    // build that does not elaborate, which is better than the alternative but
    // still not a probe.  Outputs are additive: every existing instantiation
    // leaves them unconnected and is unmoved.
    //
    // The accumulator and the snapshot are deliberately NOT repeated here --
    // `acc_o` and `rq_snap_o` already carry them, lane 0 at `[ACC_W-1:0]`.
    output wire [ACC_W-1:0]          obs_s_acc0,
    output wire [M_W-1:0]            obs_s_m0,
    output wire [SHIFT_W-1:0]        obs_s_shift0,
    output wire                      obs_s_ok0,
    output wire                      obs_run
);

    localparam [1:0] MAC_P1_I8_O16 = 2'd0,
                     MAC_P2_I8_O8  = 2'd1,
                     MAC_P4_I8_O4  = 2'd2,
                     // **Reused encoding.**  `P16_I1_O8` was `2'd3` and head was
                     // its only caller, which is why the mode port is still two
                     // bits.
                     //
                     // **"Nothing else in the design drove `2'd3`" was true of this
                     // module and false of the tree.**  `d64_mac128_compat_map`
                     // decodes `PIX_PAR/IC_PAR/OC_LANES` into this encoding and its
                     // table still said `P16/I1/O8 -> 2'd3`, with a fall-through of
                     // `2'd0` -- so a client built at the **current** head shape
                     // `P4/I4/O8` silently got mode 0 and computed a different layer.
                     // A comment that is true only because the reader was never
                     // updated is the shape of that bug: check the other end before
                     // writing "nothing else drives this".  Fixed in that table
                     // (`2'bxx` and a `b_mode_valid` flag rather than a silent
                     // fall-through); this note is the half that is not fixed by code.
                     MAC_P4_I4_O8  = 2'd3;

    // ---- what the mode decides ---------------------------------------------
    // Three things and no more: how deep the reduction goes, how many
    // accumulators are live, and which slice of the per-channel requant
    // constants an accumulator lane indexes.
    // `IC 4` is head and nothing else; every other mode reduces eight.
    wire        ic4    = (mode == MAC_P4_I4_O8);
    wire [7:0]  nacc   = ic4 ? 8'd32 : 8'd16;
    wire [4:0]  rq_cyc = ic4 ? 5'd4  : 5'd2;

    wire signed [ACT_W-1:0] a_s [0:LANES-1];
    wire signed [WGT_W-1:0] w_s [0:LANES-1];

    genvar gi;
    generate
        for (gi = 0; gi < LANES; gi = gi + 1) begin : g_in
            assign a_s[gi] = act[gi*ACT_W +: ACT_W];
            assign w_s[gi] = wgt[gi*WGT_W +: WGT_W];
        end
    endgenerate

    // ------------------------------------------------------------------
    // The multipliers and the shared reduction heap
    // ------------------------------------------------------------------


    // synthesis translate_off
    reg [LANES*ACT_W-1:0] act_registered_shadow;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)act_registered_shadow<=0;else act_registered_shadow<=act;
    // synthesis translate_on
    localparam PW = ACT_W + WGT_W;                // one product
    localparam SW = PW + 3;                       // a full eight-wide sum

    // Four native multipliers retain the exact signed sumof4 without fabric
    // L1/L2 adders. No primitive register is enabled, hence no cycle changes.
    // Runtime IC4 consumes these sums; IC8 adds two sums in unchanged L3.
    wire [47:0] quad_sum [0:31];
    genvar dsp_group,dsp_lane;
    generate for(dsp_group=0;dsp_group<32;dsp_group=dsp_group+1)begin:g_dsp_group
      wire [47:0] cascade [0:3];
      wire [47:0] sum_output [0:3];
      for(dsp_lane=0;dsp_lane<4;dsp_lane=dsp_lane+1)begin:g_lane
        wire [47:0] cascade_input;
        if(dsp_lane==0)assign cascade_input=48'd0;
        else assign cascade_input=cascade[dsp_lane-1];
        EFX_DSP48 #(.MODE("NORMAL"),.SIGNED(1'b1),
          .A_REG(ACT_INPUT_REG),.B_REG(0),.C_REG(0),.P_REG(0),.OP_REG(0),.W_REG(0),.O_REG(0),
          .SHIFTER(0),.M_SEL("P"),.N_SEL(dsp_lane==0?"CONST0":"CASCIN"),
          .W_SEL(dsp_lane==0?"P":"X"),.CASCOUT_SEL(dsp_lane==0?"P":"W")) u_dsp(
          .A({{(19-ACT_W){a_s[4*dsp_group+dsp_lane][ACT_W-1]}},a_s[4*dsp_group+dsp_lane]}),
          .B({{(18-WGT_W){w_s[4*dsp_group+dsp_lane][WGT_W-1]}},w_s[4*dsp_group+dsp_lane]}),
          .C(18'd0),.CASCIN(cascade_input),
          .OP(2'b00),.SHIFT_ENA(1'b0),.CLK(clk),.CE(1'b1),.RST(!rst_n),
          .O(sum_output[dsp_lane]),.CASCOUT(cascade[dsp_lane]),.OVFL());
      end
      assign quad_sum[dsp_group]=sum_output[3];
    end endgenerate

    // **One heap, four taps off it.**  Built once as a pairwise tree so the
    // mapper sees a tree rather than a chain -- eight 16-bit adds in series is
    // several nanoseconds and this sits beside a DSP at 120 MHz.  The modes read
    // different levels; none of them rebuilds a level, and `IC 2` and `IC 4`
    // come out of the same structure the day a layer wants them.
    // L1 native DSP cascade intermediates are internal to each group.
    wire signed [SW-1:0] l2 [0:31];
    reg signed [SW-1:0] l3 [0:15];

    // **The first level widens before it adds.**  `prod` is `ACT_W+WGT_W` bits
    // and the sum of two of them needs one more, so `prod[2j] + prod[2j+1]`
    // evaluated at `prod`'s width overflows -- silently, and only for operand
    // pairs whose sum leaves the product's range, which is most of them.  The
    // reference does not have this because its `red` array is `SW` wide from the
    // assignment that loads it, so every add it writes is already at sum width.
    function signed [SW-1:0] wid;
        input signed [PW-1:0] v;
        begin
            wid = {{(SW-PW){v[PW-1]}}, v};
        end
    endfunction

    generate for(gi=0;gi<32;gi=gi+1)begin:g_quad_sum
      assign l2[gi]=quad_sum[gi][SW-1:0];
    end endgenerate
    integer j;
    always @* begin


        for (j = 0; j < 16; j = j + 1)
            l3[j] = l2[2*j] + l2[2*j + 1];
    end

    // Accumulator `k` owns multipliers `k*IC .. k*IC+IC-1`.  At `IC 8` that is
    // one `l3` entry and there are sixteen of them; at `IC 1` it is the product
    // itself and there are 128.  Above sixteen there is nothing to accumulate at
    // `IC 8`, and the entries are zeroed rather than left to the level's
    // out-of-range behaviour.
    // **The zero is a signed wire and not a concatenation.**  Written inline as
    // `ic4 ? l2[gi] : {SW{1'b0}}` the concatenation makes the whole conditional
    // **unsigned**, because Verilog makes an expression unsigned if any operand
    // is -- and a concatenation always is.  `prod[gi]` is then zero-extended
    // instead of sign-extended, so a negative sum becomes `2^SW - |S|` instead
    // of `-|S|`: every accumulator that went negative is off by exactly 2^21,
    // on the lanes above sixteen only -- the `IC 8` modes' dead slots -- which is
    // the shape of a bug that reads as an arithmetic fault in the array and is
    // actually a signedness rule in the language.
    wire signed [SW-1:0] red_zero = {SW{1'b0}};

    wire signed [SW-1:0] red [0:ACC_N-1];
    generate
        for (gi = 0; gi < ACC_N; gi = gi + 1) begin : g_red
            if (gi < 16)
                assign red[gi] = ic4 ? l2[gi] : l3[gi];
            else
                assign red[gi] = ic4 ? l2[gi] : red_zero;
        end
    endgenerate

    // ------------------------------------------------------------------
    // The accumulators
    // ------------------------------------------------------------------

    reg signed [ACC_W-1:0] acc_q   [0:ACC_N-1];
    reg signed [ACC_W-1:0] done_acc[0:ACC_N-1];
    reg signed [ACC_W-1:0] acc_sum [0:ACC_N-1];
    integer d;

    always @* begin
        for (d = 0; d < ACC_N; d = d + 1) begin
            // One adder, two readers: `done_acc` and the accumulator want the
            // same sum on a slot edge and building it twice costs real slack.
            acc_sum[d]  = acc_q[d] + red[d];
            // `slot_valid` alone and not `pix_start && slot_valid`.  A slot on
            // the bus belongs to the phase being completed whether or not the
            // next one starts on the same edge.
            done_acc[d] = (slot_valid && !ROLLOVER_SABOTAGE) ? acc_sum[d] : acc_q[d];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            for (d = 0; d < ACC_N; d = d + 1)
                acc_q[d] <= {ACC_W{1'b0}};
        else
            for (d = 0; d < ACC_N; d = d + 1)
                if (pix_start)       acc_q[d] <= bias[d*ACC_W +: ACC_W];
                else if (slot_valid) acc_q[d] <= acc_sum[d];
    end

    generate
        for (gi = 0; gi < LANES; gi = gi + 1)
            assign acc_o[gi*ACC_W +: ACC_W] = (gi < ACC_N) ? acc_q[gi] : {ACC_W{1'b0}};
    endgenerate

    // ------------------------------------------------------------------
    // The requant: one sequencer, one set of lanes, both modes
    // ------------------------------------------------------------------

    reg signed [ACC_W-1:0] rq_snap [0:ACC_N-1];

    generate
        for (gi = 0; gi < LANES; gi = gi + 1)
            assign rq_snap_o[gi*ACC_W +: ACC_W] = (gi < ACC_N) ? rq_snap[gi] : {ACC_W{1'b0}};
    endgenerate

    reg signed [ACC_W-1:0] s_acc   [0:RQ_LANES-1];
    reg        [M_W-1:0]   s_m     [0:RQ_LANES-1];
    reg        [SHIFT_W-1:0] s_shift [0:RQ_LANES-1];
    reg        [RQ_LANES-1:0] s_ok;

    // `ph` counts the passes loaded.  Four is the deepest a pass sequence gets
    // now (`MAC_P4_I4_O8`: 32 accumulators, eight lanes) -- it was sixteen when
    // head carried 128 -- so five bits is slack that is left alone rather than
    // re-tuned.
    reg [4:0] ph;
    reg       run;

    // The P7r observation ports.  Lane 0 only, and taken as the register and
    // not through the flat bus, so nothing here can pick up a neighbouring
    // lane's bits: the bank's buses have stride `ACC_W` and a `[31:0]` slice of
    // one at `ACC_W == 25` carries seven bits of lane 1.
    assign obs_s_acc0   = s_acc[0];
    assign obs_s_m0     = s_m[0];
    assign obs_s_shift0 = s_shift[0];
    assign obs_s_ok0    = s_ok[0];
    assign obs_run      = run;


    assign rq_ready = ~run | (ph == rq_cyc);
    wire [5:0] ph_k = {1'b0, ph} + RQ_LOOKAHEAD[5:0];
    assign rq_ready_n = ~run | (ph_k >= {1'b0, rq_cyc});
    wire rq_accept = rq_start & rq_ready;

    // **Widths spelled out.**  Verilog evaluates the whole right-hand side at
    // the width of the left-hand side, so `ph * RQ_LANES` in a narrow context
    // truncates -- and at 128 accumulators `sel_base` reaches 120, which does
    // not fit seven bits.  A truncated base reloads pass 0 forever: it maps, it
    // simulates, and it looks like a working sequencer emitting the wrong
    // channels.
    wire [7:0] sel_base = {3'b0, ph} * 8'd8;

    // Which slice of the per-channel constants an accumulator lane reads.  The
    // parameterised MAC writes this as `idx % OC_LANES`, and with `OC_LANES` a
    // constant that is a bit slice -- 16, 8 and 4 are `idx[3:0]`, `idx[2:0]` and
    // `idx[1:0]`.  Generalised to a runtime mode it stays a bit slice, so it is
    // still not a divider.
    function [3:0] oidx;
        input [7:0] ix;
        input [1:0] md;
        begin
            case (md)
                MAC_P1_I8_O16: oidx = ix[3:0];
                MAC_P4_I8_O4:  oidx = {2'b0, ix[1:0]};
                default:       oidx = {1'b0, ix[2:0]};
            endcase
        end
    endfunction

    integer ri;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph <= 5'd0;
            run <= 1'b0;
            s_ok <= {RQ_LANES{1'b0}};
            for (ri = 0; ri < ACC_N; ri = ri + 1)
                rq_snap[ri] <= {ACC_W{1'b0}};
            for (ri = 0; ri < RQ_LANES; ri = ri + 1) begin
                s_acc[ri]   <= {ACC_W{1'b0}};
                s_m[ri]     <= {M_W{1'b0}};
                s_shift[ri] <= {SHIFT_W{1'b0}};
            end
        end
        else begin
            if (rq_accept)
                for (ri = 0; ri < ACC_N; ri = ri + 1)
                    rq_snap[ri] <= done_acc[ri];

            if (rq_accept) begin
                // Pass 0 reads `done_acc` and not the snapshot: on this edge the
                // snapshot register is still the previous request's value, and
                // `done_acc` is the same number -- including the final slot of a
                // rollover edge, which is the reason it exists at all.
                ph <= 5'd1;
                run <= 1'b1;
                for (ri = 0; ri < RQ_LANES; ri = ri + 1) begin
                    s_acc[ri]   <= done_acc[ri];
                    s_m[ri]     <= rq_m[oidx({5'b0, ri[2:0]}, mode)*M_W +: M_W];
                    s_shift[ri] <= rq_shift[oidx({5'b0, ri[2:0]}, mode)*SHIFT_W +: SHIFT_W];
                    s_ok[ri]    <= (ri < nacc);
                end
            end
            else if (run) begin
                if (ph == rq_cyc) begin
                    run  <= 1'b0;
                    s_ok <= {RQ_LANES{1'b0}};
                end
                else begin
                    // Every later pass comes from the snapshot, which the MAC
                    // cannot move.
                    ph <= ph + 5'd1;
                    for (ri = 0; ri < RQ_LANES; ri = ri + 1) begin
                        if (sel_base + ri[7:0] < nacc) begin
                            s_acc[ri]   <= rq_snap[sel_base + ri[7:0]];
                            s_m[ri]     <= rq_m[oidx(sel_base + ri[7:0], mode)*M_W
                                                +: M_W];
                            s_shift[ri] <= rq_shift[oidx(sel_base + ri[7:0], mode)
                                                   *SHIFT_W +: SHIFT_W];
                            s_ok[ri]    <= 1'b1;
                        end
                        else
                            s_ok[ri] <= 1'b0;
                    end
                end
            end
        end
    end

    // A phase that has received a slot and has not been accepted.  `pix_start`
    // destroys it, so `pix_start` without an accept is a lost accumulator.
    reg acc_dirty;
    reg [1:0] mode_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_q     <= mode;
            mode_viol  <= 32'd0;
            acc_dirty  <= 1'b0;
            rq_overrun <= 1'b0;
        end
        else begin
            mode_q <= mode;
            // A switch is legal only when nothing is in flight and nothing is
            // owed: no slot on the bus, no requant running, and no accumulator
            // waiting to be accepted.
            if ((mode !== mode_q) && (slot_valid || rq_busy || acc_dirty))
                mode_viol <= FAULT_STICKY ? 32'd1 : mode_viol + 32'd1;
            rq_overrun <= pix_start & acc_dirty & ~rq_accept;
            // The accept clears it too.  Clearing on `pix_start` alone leaves
            // every phase after the first looking dirty, and a warning that is
            // always on is a warning nobody reads.
            if (pix_start)       acc_dirty <= 1'b0;
            else if (rq_accept)  acc_dirty <= 1'b0;
            else if (slot_valid) acc_dirty <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // The lanes
    // ------------------------------------------------------------------
    //
    // **The lane clamps to its full output range and the narrow clamp is here.**
    // `d64_requant_lane` takes `QMIN`/`QMAX` as elaboration constants because
    // the parameterised MAC is built per layer; widening them into ports would
    // change a module the trunk, ups.0 and ups.1 are all closed against.  The
    // two clamps compose -- `clamp(clamp(x, 0, 2^OUT_W-1), qmin, qmax)` with
    // `[qmin, qmax]` inside that range is `clamp(x, qmin, qmax)` -- so the trunk
    // adapter drives 0 and 1023 and gets bit-identical behaviour, while head
    // drives 0 and 255 and `downs.1` drives 413 and 1023 for its ReLU.

    wire [RQ_LANES*OUT_W-1:0] lane_y;
    wire [RQ_LANES-1:0]       lane_ok;

    genvar rl;
    generate
        for (rl = 0; rl < RQ_LANES; rl = rl + 1) begin : g_rq
            wire [OUT_W-1:0] raw;
            d64_requant_lane #(.ACC_W(ACC_W), .M_W(M_W), .SHIFT_W(SHIFT_W),
                               .OUT_W(OUT_W), .QMIN(0),
                               .QMAX((1 << OUT_W) - 1)) u_lane (
                .acc   (s_acc[rl]),
                .bias  ({ACC_W{1'b0}}),   // already in the accumulator
                .m     (s_m[rl]),
                .shift (s_shift[rl]),
                .zero  (rq_zero),
                .y     (raw)
            );
            // **The bounds are read combinationally, like `rq_zero`.**  They are
            // a property of the layer and not of the pass, so registering them
            // buys nothing and costs the accept cycle: `s_qmin`/`s_qmax` are
            // loaded on the accept edge, so during that very cycle -- the one
            // whose `lane_y` becomes the first `y` anyone sees -- they still hold
            // their reset value of zero, and every lane above zero clamps to
            // `rq_qmax`.  The parameterised MAC has no such window because its
            // bounds are elaboration constants.
            wire signed [ACC_W-1:0] raw_s = {{(ACC_W-OUT_W){1'b0}}, raw};
            wire signed [ACC_W-1:0] qmin_s = {{(ACC_W-OUT_W){1'b0}}, rq_qmin};
            wire signed [ACC_W-1:0] qmax_s = {{(ACC_W-OUT_W){1'b0}}, rq_qmax};
            assign lane_y[rl*OUT_W +: OUT_W] =
                (raw_s < qmin_s) ? rq_qmin :
                (raw_s > qmax_s) ? rq_qmax : raw;
            // The operand register holds pass `ph - 1` and is live exactly while
            // `1 <= ph <= rq_cyc`, which `run` and `s_ok` already bracket.
            assign lane_ok[rl] = s_ok[rl] & run;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y        <= {RQ_LANES*OUT_W{1'b0}};
            y_valid  <= {RQ_LANES{1'b0}};
            rq_busy  <= 1'b0;
            rq_done  <= 1'b0;
        end
        else begin
            y       <= lane_y;
            y_valid <= lane_ok;
            rq_busy <= run;
            rq_done <= run & (ph == rq_cyc);
        end
    end

endmodule
