// A write-only I2C master: 7-bit device address, 16-bit register address, 8-bit data.
//
// **Written rather than ported, and that is a deliberate choice.**  The vendor demo
// ships an OpenCores `i2c_master_top` with a register-file front end and a ROM
// sequencer -- about a thousand lines of someone else's state machine, and its
// failure mode is silence.  A camera bring-up lives or dies on being able to say
// *which* register the sensor refused, and the cheapest way to have that number is
// to own the byte_ix-level logic.
//
// So: one byte_ix at a time, with the ack sampled in the middle of the ack clock, a
// sticky `nack` the caller counts, and a `done` that says the transfer finished.  A
// sensor that is not there pulls no ack, which is the difference between "the table
// is wrong" and "the wire is wrong".
//
// **`bit` and `byte` are SystemVerilog keywords and ModelSim does not say so.**  The
// first version of this file named two counters `bit` and `byte`; `vlog` in its
// default mode compiled it without a murmur, and Efinity, which runs the synthesiser
// with `verilog_mode=sv_09`, refused the file outright.  A language server and a
// synthesiser disagreeing about what is an identifier is worth one line of comment,
// because the failing tool is the second one and it fails after the design is written.
//
// Open drain: the pads are inout with separate OUT and OE.  `scl_oe`/`sda_oe` high
// drives the line **low** (OUT is tied to 0 by the caller), low releases it to the
// board's pull-up.  Releasing is not driving high, and a design that drives high
// fights every other master on the bus.
//
// Timing is four quarter-phases per SCL period, which is what puts the ack sample in
// the middle of SCL high rather than on an edge.
module cam_i2c_master #(
    parameter integer CLK_HZ   = 50_000_000,
    parameter integer SCL_HZ   = 100_000,
    // 0x30 is the SmartSens default; the vendor demo passes 8'h60 to a core that
    // shifts it left itself, so the 7-bit form is the same address.
    parameter [6:0]   DEV_ADDR = 7'h30
) (
    input  wire        clk,
    input  wire        rst_n,
    // One transfer: assert `start` with `addr` and `data` stable.  `busy` is high
    // until the STOP, and `done` pulses for one cycle after it.
    input  wire        start,
    input  wire [15:0] addr,
    input  wire [7:0]  data,
    // **`rd` turns the same transfer into a read, and it is here because
    // "did the sensor accept that write" cannot be answered by writing.**
    // A read is: the write prefix (device+W, address high, address low), a
    // **repeated START** rather than a STOP so the sensor keeps its register
    // pointer, then device+R, then eight bits in and a NACK.
    input  wire        rd,
    output reg  [7:0]  rdata,
    output reg         busy,
    output reg         done,
    // Sticky for this transfer: the addressed device did not acknowledge.
    output reg         nack,
    input  wire        scl_i,
    output reg         scl_oe,
    input  wire        sda_i,
    output reg         sda_oe
);

    // Quarter period, floored at 1 so a silly parameter cannot make the counter
    // never advance.
    localparam integer DIV = (CLK_HZ / (SCL_HZ * 4) < 1) ? 1 : (CLK_HZ / (SCL_HZ * 4));

    localparam [2:0] S_IDLE = 3'd0, S_START = 3'd1, S_BYTE = 3'd2,
                     S_STOP = 3'd3, S_DONE = 3'd4,
                     S_RSTART = 3'd5, S_RBYTE = 3'd6;

    reg [2:0]  state;
    reg [15:0] q;          // quarter-phase counter
    reg [1:0]  ph;         // 0..3 within one SCL period
    reg [3:0]  bit_ix;        // 0..7 data bits, 8 = the ack
    reg [2:0]  byte_ix;       // 0 dev+W, 1 addr hi, 2 addr lo, 3 data or dev+R
    reg [7:0]  sh;         // the byte_ix being shifted out

    wire       is_ack    = (bit_ix == 4'd8);
    wire       sda_bit   = is_ack ? 1'b1 : sh[7];
    // **A function of the index it is asked about, and that is the whole fix.**
    //
    // This used to be `wire next_byte` read on the same edge that advanced
    // `byte_ix`.  A non-blocking assignment evaluates its right-hand side with
    // the *current* value, so `byte_ix <= byte_ix + 1; sh <= next_byte;` loaded
    // the byte for the index it was leaving, not the one it was entering.  On the
    // wire that is not a small skew: the transfer is device+W, address high,
    // address low, data, and the master sent **device+W, device+W, address high,
    // address low** -- four bytes, correct count, correct ACKs, and **the data
    // byte was never transmitted at all**.  Every "register write" this project
    // has ever sent wrote `<addr_lo>` into `0x60<addr_hi>`; nothing was
    // configured, and a sensor that is not configured does not stream.
    function [7:0] byte_at;
        input [2:0] i;
        begin
            case (i)
                3'd0:    byte_at = {DEV_ADDR, 1'b0};   // device + write
                3'd1:    byte_at = addr[15:8];
                3'd2:    byte_at = addr[7:0];
                // **Index 3 is where the two transfers differ.**  A write ends
                // with the data byte; a read ends its *address phase* with
                // device+R and then goes back to the slave for a byte.  The
                // original expression named only device+R here and had no `data`
                // arm at all, so even with the indexing corrected a write would
                // never have put its data on the wire.
                3'd3:    byte_at = rd ? {DEV_ADDR, 1'b1} : data;
                default: byte_at = data;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; q <= 16'd0; ph <= 2'd0; bit_ix <= 4'd0; byte_ix <= 2'd0;
            sh <= 8'd0; busy <= 1'b0; done <= 1'b0; nack <= 1'b0;
            rdata <= 8'h00;
            scl_oe <= 1'b0; sda_oe <= 1'b0;
        end
        else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    scl_oe <= 1'b0; sda_oe <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        nack  <= 1'b0;
                        rdata <= 8'h00;
                        byte_ix  <= 2'd0;
                        bit_ix   <= 4'd0;
                        ph    <= 2'd0;
                        q     <= 16'd0;
                        state <= S_START;
                    end
                end

                // START: both lines released, then SDA pulled low while SCL is high.
                S_START: begin
                    if (q != DIV - 1) q <= q + 16'd1;
                    else begin
                        q <= 16'd0;
                        case (ph)
                            2'd0: sda_oe <= 1'b0;                  // SDA high
                            2'd1: scl_oe <= 1'b0;                  // SCL high
                            2'd2: sda_oe <= 1'b1;                  // SDA low = START
                            2'd3: begin
                                scl_oe <= 1'b1;                    // SCL low
                                sh     <= byte_at(byte_ix);        // index 0: device+W
                                state  <= S_BYTE;
                            end
                        endcase
                        ph <= ph + 2'd1;
                    end
                end

                // One bit_ix per SCL period: SDA changes while SCL is low, is held
                // through SCL high, and the ack is sampled with SCL high.
                S_BYTE: begin
                    if (q != DIV - 1) q <= q + 16'd1;
                    else begin
                        q <= 16'd0;
                        case (ph)
                            2'd0: begin
                                scl_oe <= 1'b1;                    // SCL low
                                sda_oe <= ~sda_bit;                // drive or release
                            end
                            2'd1: scl_oe <= 1'b0;                  // SCL high
                            2'd2: begin
                                if (is_ack) begin
                                    // The slave drove it, or nobody did.  Sampled
                                    // here, in the middle of SCL high.
                                    sda_oe <= 1'b0;
                                    if (sda_i) nack <= 1'b1;
                                end
                            end
                            2'd3: begin
                                scl_oe <= 1'b1;                    // SCL low
                                if (is_ack) begin
                                    bit_ix <= 4'd0;
                                    if (byte_ix == 3'd3) begin
                                        // The address byte just sent was
                                        // acknowledged.  For a write that was the
                                        // data, so the transfer is over; for a read
                                        // it was device+R, so the slave now owns
                                        // SDA and the master must not drive it.
                                        if (rd) state <= S_RBYTE;
                                        else    state <= S_STOP;
                                    end
                                    else if ((byte_ix == 3'd2) && rd) begin
                                        // **Repeated START, not a STOP.**  A STOP
                                        // here would end the transaction and the
                                        // sensor would drop its register pointer,
                                        // so the read would return whatever the
                                        // pointer happens to default to rather
                                        // than the register that was asked for.
                                        byte_ix <= 3'd3;
                                        state   <= S_RSTART;
                                    end
                                    else begin
                                        // **`byte_ix + 1`, because that is the byte
                                        // being entered.**  `sh <= byte_at(byte_ix)`
                                        // here is the defect this function exists
                                        // to make impossible.
                                        byte_ix <= byte_ix + 3'd1;
                                        sh   <= byte_at(byte_ix + 3'd1);
                                    end
                                end
                                else begin
                                    bit_ix <= bit_ix + 4'd1;
                                    sh  <= {sh[6:0], 1'b0};
                                end
                            end
                        endcase
                        ph <= ph + 2'd1;
                    end
                end

                // STOP: SDA released while SCL is high.
                S_STOP: begin
                    if (q != DIV - 1) q <= q + 16'd1;
                    else begin
                        q <= 16'd0;
                        case (ph)
                            2'd0: begin sda_oe <= 1'b1; scl_oe <= 1'b1; end
                            2'd1: scl_oe <= 1'b0;                  // SCL high
                            2'd2: sda_oe <= 1'b0;                  // SDA high = STOP
                            2'd3: begin
                                scl_oe <= 1'b0;
                                state  <= S_DONE;
                            end
                        endcase
                        ph <= ph + 2'd1;
                    end
                end

                // Repeated START.  The same four phases as `S_START` and
                // deliberately not the same state: what makes a repeated START
                // a repeated START is that no STOP came before it, and keeping
                // them apart is what makes that visible in the state trace.
                S_RSTART: begin
                    if (q != DIV - 1) q <= q + 16'd1;
                    else begin
                        q <= 16'd0;
                        case (ph)
                            2'd0: sda_oe <= 1'b0;
                            2'd1: scl_oe <= 1'b0;
                            2'd2: sda_oe <= 1'b1;
                            2'd3: begin
                                scl_oe <= 1'b1;
                                sh     <= byte_at(byte_ix);        // index 3: device+R
                                state  <= S_BYTE;
                            end
                        endcase
                        ph <= ph + 2'd1;
                    end
                end

                // Eight bits in, MSB first, with SDA released throughout -- the
                // master drives SDA only for START, STOP and the ack slot.  The
                // ninth slot is released too, and *that release is the NACK*:
                // the master has had its byte and tells the slave to stop.
                S_RBYTE: begin
                    if (q != DIV - 1) q <= q + 16'd1;
                    else begin
                        q <= 16'd0;
                        case (ph)
                            2'd0: scl_oe <= 1'b1;                  // SCL low
                            2'd1: scl_oe <= 1'b0;                  // SCL high
                            2'd2: if (!is_ack) rdata <= {rdata[6:0], sda_i};
                            2'd3: begin
                                scl_oe <= 1'b1;                    // SCL low
                                if (is_ack) state  <= S_STOP;
                                else        bit_ix <= bit_ix + 4'd1;
                            end
                        endcase
                        ph <= ph + 2'd1;
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
