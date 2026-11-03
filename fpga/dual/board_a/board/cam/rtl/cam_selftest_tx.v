// A serial pattern with no input but the clock.
//
// The bring-up's first capture came back **zero bytes**.  The design was
// configured, the pinout had been checked against two independent sources, the
// flow reported PASS and produced a bitstream -- and the port was silent.  That
// reading cannot tell apart "the bitstream never came up", "the 25 MHz
// oscillator is not the clock this design thinks it is", "the baud divisor is
// wrong", "the pad is wrong" and "the host is reading the wrong FTDI channel",
// because the transmitter being read sits downstream of the snapshot handshake,
// the CSI receiver and the telemetry serialiser, and every one of those being
// dead produces exactly the same silence.
//
// So this module depends on exactly one thing: `clk`.  It emits one fixed line
// at a fixed rate, and it counts:
//
//     SELFTEST 0000000a\n
//     LANES    c0000001\n
//     SENSOR   cd6b0100\n
//
// **Two lines, alternating, same geometry**, because a beacon that says only
// "the clock is running" cannot say whether the camera did anything.  The
// `LANES` line carries `cam_csi_obs`'s 32-bit state word -- clock-lane and
// data-lane enable, the recovered byte clock, the FIFO flags -- and it is
// sampled in the *same* domain as this transmitter, so it is readable while the
// pixel domain is dead.  Same eight-byte prefix, same eight hex digits, same
// length: a host that parses one parses the other.
//
// **Three lines, and the third is the sensor's own answer.**  The I2C
// sequencer reads four bytes back out of the sensor once the table is out --
// ID high, ID low, `0x0100`, `0x302c` -- and packs them into one word.  A
// sensor that is configured and streaming answers `cd6b0100`: the datasheet's
// part ID `0xCD6B`, stream-on `0x01`, and the stream-control register back at
// its idle `0x00`.  That is the one reading a capture could not produce before,
// and it rides the beacon because the telemetry page's eight words are all
// spoken for, and because the beacon is readable while the pixel domain is dead
// -- which is the whole reason this module exists.
//
// Only the `SELFTEST` line advances the counter.  A host comparing the counter
// against the rate is comparing it against *that* line's period, and the two
// observation lines are further transmissions in every period.
//
// Every question a first capture can answer is in that line.  Bytes arriving at
// all settles the pad, the pinout and the host's channel.  The line parsing
// settles the baud divisor and CLK_HZ.  And the counter advancing at the rate the
// parameters claim settles the clock itself -- a fixed banner would say only
// "something is transmitting", which is a weaker statement than it looks.
//
// No RAM: a counter and a byte counter, in a project whose RAM10 budget is
// already spent elsewhere.
module cam_selftest_tx #(
    parameter CLK_HZ  = 25_000_000,
    parameter BAUD    = 115200,
    parameter LINE_HZ = 2
) (
    input  wire        clk,
    input  wire        rst_n,
    // `cam_csi_obs.state`.  A level, latched when a line starts, so the eight
    // digits on the wire are one moment rather than a word with its high half
    // from before a clock started.
    input  wire [31:0] state,
    // The four bytes the I2C sequencer read back, packed `{id_hi, id_lo,
    // 0x0100, 0x302c}`.  A level, latched with the rest when a line starts.
    input  wire [31:0] sensor,
    output wire        tx,
    // **High when no line is owed and none is in flight**, so a caller that has
    // to share this pad with another transmitter can hand it over at a byte
    // boundary instead of cutting a line in half.  The first version of the
    // sharing logic did exactly that and put a truncated line on the wire once
    // per window.
    output wire        idle
);

    localparam integer LINE_DIV = CLK_HZ / LINE_HZ;
    localparam integer LINE_W   = $clog2(LINE_DIV);
    localparam [4:0]   Q_LAST   = 5'd17;   // "SELFTEST " + 8 hex digits + LF

    reg [LINE_W-1:0] tick;
    reg [31:0]       seq;

    always @(posedge clk or negedge rst_n) begin
        // **`seq` is not reset here.**  It belongs to the line, and the line is
        // sent by the state machine below -- a register written in two `always`
        // blocks is two drivers, and `efx_map` rejects it (VDB-1000) where the
        // simulator accepts it.  Its reset lives with the rest of its logic.
        if (!rst_n) begin
            tick <= {LINE_W{1'b0}};
        end
        else if (tick == LINE_DIV - 1) begin
            tick <= {LINE_W{1'b0}};
        end
        else begin
            tick <= tick + 1'b1;
        end
    end

    reg  [4:0] q;
    reg        u_valid;
    reg  [7:0] u_data;
    wire       u_ready;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_utx (
        .clk(clk), .rst_n(rst_n),
        .data(u_data), .valid(u_valid), .ready(u_ready), .tx(tx)
    );

    // **Declared before the function that reads them.**  `line_byte` is a
    // function, and `vlog` resolves a name used inside one against what has been
    // declared *above* it -- so a register defined next to the state machine that
    // drives it is an undefined variable here, not a forward reference.
    reg [1:0]  kind;        // 0 = `SELFTEST`, 1 = `LANES`, 2 = `SENSOR`
    reg [31:0] state_held;  // the observation word this line will carry
    reg [31:0] sensor_held; // the read-back word this line will carry

    function [7:0] hexch;
        input [3:0] n;
        begin
            hexch = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h61 + {4'd0, n} - 8'd10);
        end
    endfunction

    // **One table, read at the index asked for.**  The byte for a position is a
    // function of the position, not a register loaded one step ahead of itself:
    // a register read a cycle after it was written is how the ROM comparisons in
    // this project's instruments came out one cycle early, twice.
    //
    // The nibble index is clamped, so a position that fell into this arm by
    // accident cannot produce a negative bit select -- and the select is a
    // variable shift on a wire rather than a variable part-select, because the
    // first version of the telemetry serialiser had five bytes of every record
    // vanish through an index expression that disagreed with itself.
    function [7:0] line_byte;
        input [4:0] qi;
        reg   [4:0] hi;
        reg  [31:0] sh;
        begin
            case (qi)
                // `SELFTEST ` / `LANES    ` / `SENSOR   ` -- nine bytes each,
                // so a host that parses one parses all three.  Written as the
                // three columns rather than as a table, because the column is
                // what a reader checks against.
                5'd0:  line_byte = (kind == 2'd0) ? "S" : (kind == 2'd1) ? "L" : "S";
                5'd1:  line_byte = (kind == 2'd0) ? "E" : (kind == 2'd1) ? "A" : "E";
                5'd2:  line_byte = (kind == 2'd0) ? "L" : (kind == 2'd1) ? "N" : "N";
                5'd3:  line_byte = (kind == 2'd0) ? "F" : (kind == 2'd1) ? "E" : "S";
                5'd4:  line_byte = (kind == 2'd0) ? "T" : (kind == 2'd1) ? "S" : "O";
                5'd5:  line_byte = (kind == 2'd0) ? "E" : (kind == 2'd1) ? " " : "R";
                5'd6:  line_byte = (kind == 2'd0) ? "S" : " ";
                5'd7:  line_byte = (kind == 2'd0) ? "T" : " ";
                5'd8:  line_byte = " ";
                5'd17: line_byte = 8'h0A;    // newline, so lines cannot run together
                default: begin
                    hi = (qi < 5'd9) ? 5'd0 : (5'd16 - qi);
                    sh = ((kind == 2'd1) ? state_held :
                          (kind == 2'd2) ? sensor_held : seq) >> {hi, 2'b00};
                    line_byte = hexch(sh[3:0]);
                end
            endcase
        end
    endfunction

    // One line is owed until it has been sent, and only then does the rate timer
    // start again.  That is the "pending flag" shape the snapshot handshake
    // needed: a condition compared against a copy of itself fires once and then
    // never again, and a link that reports one line looks exactly like a link
    // that is dead.
    localparam S_WAIT = 1'b0, S_PRESENT = 1'b1;

    reg st;
    reg owed;

    assign idle = (st == S_WAIT) && !owed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_WAIT; owed <= 1'b0; kind <= 2'd0;
            q <= 5'd0; u_valid <= 1'b0; u_data <= 8'h00; seq <= 32'd0;
            state_held <= 32'd0; sensor_held <= 32'd0;
        end
        else begin
            case (st)
                S_WAIT: begin
                    if (!owed && (tick == LINE_DIV - 1)) owed <= 1'b1;
                    if (owed) begin
                        state_held  <= state;
                        sensor_held <= sensor;
                        q       <= 5'd0;
                        u_data  <= line_byte(5'd0);
                        u_valid <= 1'b1;
                        st      <= S_PRESENT;
                    end
                end
                S_PRESENT: begin
                    // **Retired on the handshake, never on `u_ready` alone.**
                    // `u_ready` is high while the serialiser is *idle*, which is
                    // before it has taken the byte being presented, so advancing
                    // on it advances twice per byte and puts every second byte on
                    // the wire -- a defect the telemetry serialiser shipped once
                    // and its testbench was fitted to.
                    if (u_ready) begin
                        if (q == Q_LAST) begin
                            // **The counter advances when the line it names
                            // is finished.**  It used to advance on the rate
                            // timer, which fires one cycle *before* the line
                            // it starts -- so the first line on the wire read
                            // `00000001`, and a host counting from zero would
                            // have been off by one against the counter it was
                            // sent.  The acceptance caught it; nothing else
                            // would have.
                            if (kind == 2'd0) seq <= seq + 32'd1;
                            kind    <= (kind == 2'd2) ? 2'd0 : (kind + 2'd1);
                            u_valid <= 1'b0;
                            owed    <= 1'b0;
                            st      <= S_WAIT;
                        end
                        else begin
                            q      <= q + 5'd1;
                            u_data <= line_byte(q + 5'd1);
                        end
                    end
                end
            endcase
        end
    end

endmodule
