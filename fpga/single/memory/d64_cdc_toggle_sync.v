// One source-domain event, delivered once into the destination domain.
//
// ---------------------------------------------------------------------------
// The class: a toggle, and why it is the class and not a pulse
// ---------------------------------------------------------------------------
//
// An event that lasts one source cycle cannot be crossed as an event.  A pulse is
// narrow by construction -- it is one cycle of a faster clock, or a combinational
// decode that is narrower still -- and the destination samples on a clock that knows
// nothing about it, so a pulse that fits between two destination edges is simply not
// there.  Nothing is broken and nothing warns: the event is lost.
//
// This is not a hypothetical.  `docs/I3_TIMING_AND_RETIMING.md` section 5 records
// `control/d64_encoder_trunk.v:335` making `done` a *combinational* decode of
// `owner_q`, and the board top synchronising that decode into the AXI domain -- the
// mechanism it names for a build that read `encoder_done = 0`, `crc_done = 1` and
// zero trunk beats.  A combinational decode is as narrow as its logic depth.
//
// So the event is *widened into a level* by a flop that flips: `src_event` sets the
// toggle, the toggle is a level that stays where it was put, and the destination
// re-times it and edge-detects the change.  A level cannot be missed by sampling --
// only by sampling twice, which is what the contract below is about.
//
// ---------------------------------------------------------------------------
// The contract, stated rather than assumed
// ---------------------------------------------------------------------------
//
// **Two `src_event`s closer together than `STAGES` destination clocks are delivered
// as one.**  A toggle carries parity, not count: if it flips twice while the
// destination is not looking, the destination sees no change at all.  That is a real
// constraint on the caller and it is the reason this module is one bit and not a
// queue.  Where the source cannot promise the spacing, use
// `d64_cdc_descriptor_src`/`_dst`, whose acknowledge makes the spacing the
// destination's to give.
//
// `dst_event` is one destination cycle wide and is derived from the *settled* output
// of the synchroniser and a registered copy of it, not from an XOR of the two
// synchroniser stages -- the first stage of a synchroniser is the one that may be
// metastable, and XORing it into a pulse is a way of publishing that.
//
// `done_tog` in `board/preview/preview_top.v:271` is this module's shape written out
// by hand: a toggle in the AXI domain, `done_s` on `hdmi_tx_slow_clk`, and
// `fill_done_pe = done_s[1] ^ done_s[0]`.  It is left alone; see
// docs/CDC_CLOSURE_2026-10-07.md for what that costs and why it was not edited.
module d64_cdc_toggle_sync #(
    parameter STAGES = 2
) (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire src_event,      // one src cycle; see the contract above
    output wire src_tog,        // the level the source put down

    input  wire dst_clk,
    input  wire dst_rst_n,
    output wire dst_tog,        // the same level, settled in the dst domain
    output wire dst_event       // one dst cycle, per delivered toggle
);
    reg tog;

    always @(posedge src_clk or negedge src_rst_n)
        if (!src_rst_n)      tog <= 1'b0;
        else if (src_event)  tog <= ~tog;

    wire tog_q;
    d64_cdc_bit_sync #(.STAGES(STAGES)) u_tog (
        .clk(dst_clk), .rst_n(dst_rst_n), .d(tog), .q(tog_q)
    );

    reg tog_seen;

    always @(posedge dst_clk or negedge dst_rst_n)
        if (!dst_rst_n)    tog_seen <= 1'b0;
        else               tog_seen <= tog_q;

    assign src_tog   = tog;
    assign dst_tog   = tog_q;
    assign dst_event = tog_q ^ tog_seen;

endmodule
