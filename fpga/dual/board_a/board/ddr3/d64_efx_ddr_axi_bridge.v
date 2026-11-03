`timescale 1ns/1ps

// Board-only adapter from the encoder's split AXI AW/AR channels to the
// catalog DDR3 controller's shared A/ATYPE channel.  Arbitration is per
// accepted address beat; W/B and R remain independent response/data channels.
module d64_efx_ddr_axi_bridge (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         m_awvalid,
    output wire         m_awready,
    input  wire [31:0]  m_awaddr,
    input  wire [7:0]   m_awlen,
    input  wire [2:0]   m_awsize,
    input  wire [1:0]   m_awburst,

    input  wire         m_arvalid,
    output wire         m_arready,
    input  wire [31:0]  m_araddr,
    input  wire [7:0]   m_arlen,
    input  wire [2:0]   m_arsize,
    input  wire [1:0]   m_arburst,

    output wire [31:0]  ddr_aaddr,
    output wire [7:0]   ddr_alen,
    output wire [2:0]   ddr_asize,
    output wire [1:0]   ddr_aburst,
    output wire         ddr_avalid,
    input  wire         ddr_aready,
    output wire         ddr_atype,

    input  wire [127:0] m_wdata,
    input  wire [15:0]  m_wstrb,
    input  wire         m_wlast,
    input  wire         m_wvalid,
    output wire         m_wready,
    output wire [127:0] ddr_wdata,
    output wire [15:0]  ddr_wstrb,
    output wire         ddr_wlast,
    output wire         ddr_wvalid,
    input  wire         ddr_wready,
    output wire         m_bvalid,
    output wire [1:0]   m_bresp,
    input  wire         m_bready,
    input  wire         ddr_bvalid,
    input  wire [1:0]   ddr_bresp,
    output wire         ddr_bready,

    output wire [127:0] m_rdata,
    output wire         m_rlast,
    output wire [1:0]   m_rresp,
    output wire         m_rvalid,
    input  wire         m_rready,
    input  wire [127:0] ddr_rdata,
    input  wire         ddr_rlast,
    input  wire [1:0]   ddr_rresp,
    input  wire         ddr_rvalid,
    output wire         ddr_rready,

    output reg  [31:0]  simultaneous_wait_cycles,
    output reg  [31:0]  write_address_accepts,
    output reg  [31:0]  read_address_accepts
);
    // rr_write=1 means the next contested address grant goes to AW; every
    // accepted grant hands the preference to the other pending class.
    reg rr_write;
    reg grant_locked, locked_write;
    wire choose_write = grant_locked ? locked_write :
                        (m_awvalid && (!m_arvalid || rr_write));
    wire grant_write = choose_write && m_awvalid;
    wire grant_read  = !choose_write && m_arvalid;

    assign ddr_avalid = grant_write || grant_read;
    assign ddr_aaddr  = grant_write ? m_awaddr  : m_araddr;
    assign ddr_alen   = grant_write ? m_awlen   : m_arlen;
    assign ddr_asize  = grant_write ? m_awsize  : m_arsize;
    assign ddr_aburst = grant_write ? m_awburst : m_arburst;
    assign ddr_atype  = grant_write;
    assign m_awready = ddr_aready && grant_write;
    assign m_arready = ddr_aready && grant_read;

    // These payload channels do not use the address arbiter. The client
    // serializes write response handling and tracks its one read burst at a time.
    assign ddr_wdata = m_wdata;
    assign ddr_wstrb = m_wstrb;
    assign ddr_wlast = m_wlast;
    assign ddr_wvalid = m_wvalid;
    assign m_wready = ddr_wready;
    assign m_bvalid = ddr_bvalid;
    assign m_bresp = ddr_bresp;
    assign ddr_bready = m_bready;
    assign m_rdata = ddr_rdata;
    assign m_rlast = ddr_rlast;
    assign m_rresp = ddr_rresp;
    assign m_rvalid = ddr_rvalid;
    assign ddr_rready = m_rready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_write <= 1'b1;
            grant_locked <= 1'b0;
            locked_write <= 1'b0;
            simultaneous_wait_cycles <= 32'd0;
            write_address_accepts <= 32'd0;
            read_address_accepts <= 32'd0;
        end else begin
            // Once AVALID has been presented under backpressure, a newly
            // arriving opposite request cannot replace that address/ATYPE.
            // Each selected master retains its payload until its own handshake.
            if (ddr_avalid && !ddr_aready && !grant_locked) begin
                grant_locked <= 1'b1;
                locked_write <= choose_write;
            end
            if (m_awvalid && m_arvalid && !ddr_aready)
                simultaneous_wait_cycles <= simultaneous_wait_cycles + 1'b1;
            if (ddr_avalid && ddr_aready) begin
                grant_locked <= 1'b0;
                if (grant_write) begin
                    rr_write <= 1'b0;
                    write_address_accepts <= write_address_accepts + 1'b1;
                end else begin
                    rr_write <= 1'b1;
                    read_address_accepts <= read_address_accepts + 1'b1;
                end
            end
        end
    end
endmodule
