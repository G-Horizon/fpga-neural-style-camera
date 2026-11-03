`timescale 1ns/1ps

// D3c6-e6 -- two AXI masters onto D3c6-d's one.
//
// `d64_encoder_ddr_mem` issues the encoder's transactions and
// `d64_trunk_row_ddr` issues the trunk's, and the catalog controller has one
// AXI slave port.  So the two are merged here, **upstream of
// `d64_efx_ddr_axi_bridge`**, which is unchanged: the bridge's own arbitration
// between the read and write address channels is a property of the controller's
// shared A/ATYPE port and has nothing to do with how many masters there are.
//
// Ownership, not round robin
// --------------------------
//
// A write burst is AW then W then B, and a read burst is AR then R.  The two
// data phases are *independent channels* in this protocol -- a master may have
// an AW accepted while it is still streaming W, and the two masters' channels
// must not interleave, or the controller pairs one master's data with the
// other's address.
//
// So each of the two directions keeps an owner from the address beat to the
// final response, and the other master is held off for that whole span.  It is
// not a throughput device: **the encoder and the trunk do not overlap in this
// design** -- the handover waits for the encoder's last trunk-A response -- and
// what this exists for is to make that a property of the wiring rather than an
// assumption the engine has to keep.
//
// **This is not the cause of the observed first-mismatch CRC.**  The encoder
// and the trunk are largely sequential today -- the handover waits for the
// encoder's last trunk-A response -- so the stalled-address substitution fixed
// below is not a state this design currently reaches.  It is a real defect in
// the arbiter, and it matters as soon as camera and HDMI traffic share the
// port -- a different question from the open first-mismatch investigation, and
// not evidence about it either way.
module d64_axi_arb2 (
    input  wire         clk,
    input  wire         rst_n,

    // ---- master 0: the encoder's DDR machinery ---------------------------
    input  wire         m0_awvalid,
    output wire         m0_awready,
    input  wire [31:0]  m0_awaddr,
    input  wire [7:0]   m0_awlen,
    input  wire [2:0]   m0_awsize,
    input  wire [1:0]   m0_awburst,
    input  wire         m0_wvalid,
    output wire         m0_wready,
    input  wire [127:0] m0_wdata,
    input  wire [15:0]  m0_wstrb,
    input  wire         m0_wlast,
    output wire         m0_bvalid,
    input  wire         m0_bready,
    output wire [1:0]   m0_bresp,
    input  wire         m0_arvalid,
    output wire         m0_arready,
    input  wire [31:0]  m0_araddr,
    input  wire [7:0]   m0_arlen,
    input  wire [2:0]   m0_arsize,
    input  wire [1:0]   m0_arburst,
    output wire         m0_rvalid,
    input  wire         m0_rready,
    output wire [127:0] m0_rdata,
    output wire         m0_rlast,
    output wire [1:0]   m0_rresp,

    // ---- master 1: the trunk's row adapter -------------------------------
    input  wire         m1_awvalid,
    output wire         m1_awready,
    input  wire [31:0]  m1_awaddr,
    input  wire [7:0]   m1_awlen,
    input  wire [2:0]   m1_awsize,
    input  wire [1:0]   m1_awburst,
    input  wire         m1_wvalid,
    output wire         m1_wready,
    input  wire [127:0] m1_wdata,
    input  wire [15:0]  m1_wstrb,
    input  wire         m1_wlast,
    output wire         m1_bvalid,
    input  wire         m1_bready,
    output wire [1:0]   m1_bresp,
    input  wire         m1_arvalid,
    output wire         m1_arready,
    input  wire [31:0]  m1_araddr,
    input  wire [7:0]   m1_arlen,
    input  wire [2:0]   m1_arsize,
    input  wire [1:0]   m1_arburst,
    output wire         m1_rvalid,
    input  wire         m1_rready,
    output wire [127:0] m1_rdata,
    output wire         m1_rlast,
    output wire [1:0]   m1_rresp,

    // ---- the slave the bridge sees ---------------------------------------
    output wire         s_awvalid,
    input  wire         s_awready,
    output wire [31:0]  s_awaddr,
    output wire [7:0]   s_awlen,
    output wire [2:0]   s_awsize,
    output wire [1:0]   s_awburst,
    output wire         s_wvalid,
    input  wire         s_wready,
    output wire [127:0] s_wdata,
    output wire [15:0]  s_wstrb,
    output wire         s_wlast,
    input  wire         s_bvalid,
    output wire         s_bready,
    input  wire [1:0]   s_bresp,
    output wire         s_arvalid,
    input  wire         s_arready,
    output wire [31:0]  s_araddr,
    output wire [7:0]   s_arlen,
    output wire [2:0]   s_arsize,
    output wire [1:0]   s_arburst,
    input  wire         s_rvalid,
    output wire         s_rready,
    input  wire [127:0] s_rdata,
    input  wire         s_rlast,
    input  wire [1:0]   s_rresp,

    output reg  [31:0]  write_owner_holds,
    output reg  [31:0]  read_owner_holds
);

    // ------------------------------------------------------------------
    // the write direction: one owner from AW to B
    // ------------------------------------------------------------------
    reg  w_owner;          // 0 = master 0, 1 = master 1
    reg  w_busy;           // an address beat has been taken and no B yet
    // **The grant, held for the whole address beat.**  `w_busy` is set by the
    // *accepted* handshake, so it is still low while a presented AW waits on
    // `s_awready`; keying the mux on `w_busy` alone let a higher-priority master
    // replace `s_awaddr` under a VALID beat whose READY had not come.  These two
    // latch the moment an address is presented and release only on the handshake.
    reg  w_aw_held;        // an AW is on `s_aw*` and not yet accepted
    reg  w_aw_owner;       // whose address it is

    wire w_take_1 = w_busy && (w_owner == 1'b1);
    wire w_take_0 = w_busy && (w_owner == 1'b0);

    // A grant is made once -- when the direction is free and a master presents
    // -- and then held until it is accepted, so the address on `s_aw*` cannot
    // change mid-beat.  Master 0 keeps priority for the initial grant.
    wire w_aw_free = !w_busy && !w_aw_held;
    wire m0_aw_take = w_aw_held ? (w_aw_owner == 1'b0)
                                : (w_aw_free && m0_awvalid);
    wire m1_aw_take = w_aw_held ? (w_aw_owner == 1'b1)
                                : (w_aw_free && !m0_awvalid && m1_awvalid);

    assign m0_awready = m0_aw_take && s_awready;
    assign m1_awready = m1_aw_take && s_awready;

    assign s_awvalid = m0_aw_take || m1_aw_take;
    assign s_awaddr  = m0_aw_take ? m0_awaddr  : m1_awaddr;
    assign s_awlen   = m0_aw_take ? m0_awlen   : m1_awlen;
    assign s_awsize  = m0_aw_take ? m0_awsize  : m1_awsize;
    assign s_awburst = m0_aw_take ? m0_awburst : m1_awburst;

    // The data and response channels follow the owner, not the request.
    assign s_wvalid = w_take_0 ? m0_wvalid : (w_take_1 ? m1_wvalid : 1'b0);
    assign s_wdata  = w_take_0 ? m0_wdata  : m1_wdata;
    assign s_wstrb  = w_take_0 ? m0_wstrb  : m1_wstrb;
    assign s_wlast  = w_take_0 ? m0_wlast  : m1_wlast;
    assign m0_wready = w_take_0 && s_wready;
    assign m1_wready = w_take_1 && s_wready;

    assign m0_bvalid = w_take_0 && s_bvalid;
    assign m1_bvalid = w_take_1 && s_bvalid;
    assign m0_bresp  = s_bresp;
    assign m1_bresp  = s_bresp;
    assign s_bready  = w_take_0 ? m0_bready : (w_take_1 ? m1_bready : 1'b0);

    // ------------------------------------------------------------------
    // the read direction: one owner from AR to RLAST
    // ------------------------------------------------------------------
    reg  r_owner;
    reg  r_busy;
    reg  r_ar_held;        // an AR is on `s_ar*` and not yet accepted
    reg  r_ar_owner;

    wire r_take_1 = r_busy && (r_owner == 1'b1);
    wire r_take_0 = r_busy && (r_owner == 1'b0);

    // The same held grant on the read side; an address beat is a beat whichever
    // channel carries it.
    wire r_ar_free = !r_busy && !r_ar_held;
    wire m0_ar_take = r_ar_held ? (r_ar_owner == 1'b0)
                                : (r_ar_free && m0_arvalid);
    wire m1_ar_take = r_ar_held ? (r_ar_owner == 1'b1)
                                : (r_ar_free && !m0_arvalid && m1_arvalid);

    assign m0_arready = m0_ar_take && s_arready;
    assign m1_arready = m1_ar_take && s_arready;

    assign s_arvalid = m0_ar_take || m1_ar_take;
    assign s_araddr  = m0_ar_take ? m0_araddr  : m1_araddr;
    assign s_arlen   = m0_ar_take ? m0_arlen   : m1_arlen;
    assign s_arsize  = m0_ar_take ? m0_arsize  : m1_arsize;
    assign s_arburst = m0_ar_take ? m0_arburst : m1_arburst;

    assign m0_rvalid = r_take_0 && s_rvalid;
    assign m1_rvalid = r_take_1 && s_rvalid;
    assign m0_rdata  = s_rdata;
    assign m1_rdata  = s_rdata;
    assign m0_rlast  = s_rlast;
    assign m1_rlast  = s_rlast;
    assign m0_rresp  = s_rresp;
    assign m1_rresp  = s_rresp;
    assign s_rready  = r_take_0 ? m0_rready : (r_take_1 ? m1_rready : 1'b0);

    // ------------------------------------------------------------------
    // the ownership registers
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_owner <= 1'b0; w_busy <= 1'b0;
            w_aw_held <= 1'b0; w_aw_owner <= 1'b0;
            r_owner <= 1'b0; r_busy <= 1'b0;
            r_ar_held <= 1'b0; r_ar_owner <= 1'b0;
            write_owner_holds <= 32'd0; read_owner_holds <= 32'd0;
        end
        else begin
            // The handshake consumes the held grant and sets ownership; a grant
            // that has not been accepted is latched the cycle it is made and
            // held until then.
            if (s_awvalid && s_awready) begin
                if (!w_busy) begin
                    w_owner <= m0_aw_take ? 1'b0 : 1'b1;
                end
                w_busy <= 1'b1;
                w_aw_held <= 1'b0;
            end
            else if (!w_aw_held && s_awvalid) begin
                w_aw_held  <= 1'b1;
                w_aw_owner <= m0_aw_take ? 1'b0 : 1'b1;
            end
            if (s_bvalid && s_bready) w_busy <= 1'b0;

            if (s_arvalid && s_arready) begin
                if (!r_busy) r_owner <= m0_ar_take ? 1'b0 : 1'b1;
                r_busy <= 1'b1;
                r_ar_held <= 1'b0;
            end
            else if (!r_ar_held && s_arvalid) begin
                r_ar_held  <= 1'b1;
                r_ar_owner <= m0_ar_take ? 1'b0 : 1'b1;
            end
            if (s_rvalid && s_rready && s_rlast) r_busy <= 1'b0;

            // How long each direction was held.  Two counters and not one,
            // because the read and write phases interleave by design -- the
            // bridge arbitrates them -- and a single "busy" number would say
            // nothing about either.
            if (w_busy) write_owner_holds <= write_owner_holds + 32'd1;
            if (r_busy) read_owner_holds  <= read_owner_holds  + 32'd1;
        end
    end

endmodule
