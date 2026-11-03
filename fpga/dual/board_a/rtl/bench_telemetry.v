// Board results, as one fixed-length JSON object per line, over the UART.
//
// Every later board measurement -- HyperRAM bandwidth, the ups.1 replay, the
// calibration soak -- has to survive the loan of the board.  A photograph of a screen
// does not; a file of JSON lines does.  So the transport is built first and
// everything else plugs into it, and this module carries no measurement of its own.
//
// The record is fixed at 23 bytes:
//
//     {"i":0,"v":0xdead0000}\n
//
//      byte  0   {              11   0
//            1   "              12   x
//            2   i              13..20   eight hex digits, most significant first
//            3   "              21   }
//            4   :              22   0x0A
//            5   the index, one lower-case hex digit
//            6   ,
//            7   "   8   v   9   "   10  :
//
// `i` is the record index and the host maps it to a name; the key names live in
// docs/B0_BOARD_INFRA.md and in tools/board_telemetry.py.  The first version emitted
// string keys from a table, which meant a variable-length record, a byte offset the
// key length decided, and a right-aligned string whose bytes are read from the low
// end -- three opportunities to be wrong and all three were taken.  A fixed
// single-digit index removes the whole class: no dynamic offsets, no string slicing,
// and a host can validate the length before it parses anything.
//
// **The second version got the tail wrong and its testbench agreed with it.**  It put
// the closing brace at byte 22 and the newline at byte 23, one past the end of a
// 23-byte record, so the newline was never sent -- records ran together with no
// separator -- and byte 21 fell through to the hex-digit arm with a negative bit
// index.  The testbench had been written to expect two closing braces and no newline,
// so it passed.  That is the failure this project keeps meeting: not a wrong answer,
// but a checker fitted to the wrong answer.  The arms below now carry the number of
// the byte they emit, and tb_bench_telemetry checks the stream against the table
// above rather than against this module.
//
// Hex rather than decimal for the value: decimal needs a divider and buys nothing the
// host cannot do in one line of Python.
//
// No RAM.  M1 closes at 252 of 256 RAM10 and none of them are going to a UART.
module bench_telemetry #(
    parameter N_RESULTS = 8,
    parameter CLK_HZ    = 50_000_000,
    parameter BAUD      = 115200,
    // Test-only; see tb_bench_telemetry.v.  0 is the design.  Teeth 1..7 misread the
    // record above, teeth 8 and 9 belong to the serialiser and are forwarded.
    parameter SABOTAGE  = 0
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    output reg                     busy,
    output reg                     done,
    input  wire [N_RESULTS*32-1:0] values,
    output wire                    tx
);

    localparam integer IDX_W     = (N_RESULTS <= 1) ? 1 : $clog2(N_RESULTS);
    localparam integer REC_BYTES = 23;

    reg        u_valid;
    reg  [7:0] u_data;
    wire       u_ready;

    uart_tx #(
        .CLK_HZ(CLK_HZ), .BAUD(BAUD),
        // Teeth 8 and 9 are the UART's; 8 -> 1, 9 -> 2.  Offsetting by 8 rather
        // than 7 silently mapped tooth 8 onto the clean design and tooth 9 onto
        // tooth 8, and the sweep reported a tooth that could not be caught.
        .SABOTAGE((SABOTAGE >= 8) ? (SABOTAGE - 7) : 0)
    ) u_uart (
        .clk(clk), .rst_n(rst_n),
        .data(u_data), .valid(u_valid), .ready(u_ready), .tx(tx)
    );

    reg  [4:0]       q;              // 0..22, the byte of the record being sent
    reg  [1:0]       st;
    reg  [IDX_W-1:0] idx;
    reg  [31:0]      cur;

    // Tooth 1 ends the record at the closing brace, the shape of the tail bug above:
    // 22 bytes, no separator.
    localparam [4:0] Q_LAST = (SABOTAGE == 1) ? 5'd21 : 5'd22;

    function [7:0] hexch;
        input [3:0] n;
        begin
            hexch = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h61 + {4'd0, n} - 8'd10);
        end
    endfunction

    // The record, selected byte by byte rather than assembled into a wide vector and
    // sliced back out.  The first version did the latter; the two index expressions
    // disagreed and five bytes of every record -- all the quote marks -- vanished off
    // the wire with no error anywhere.
    // The nibble select is a continuous assignment rather than a variable written
    // inside one arm of the case.  A variable assigned on only one path infers a
    // latch, which is what efx_map correctly warned about; and clamping the index
    // outside 13..20 means a byte that reached this arm by accident cannot produce a
    // negative bit select, which is what the tail bug above actually did.
    wire [4:0] hex_i = (q < 5'd13 || q > 5'd20) ? 5'd0 : (5'd20 - q);

    reg [7:0] byte_ok;
    always @* begin
        case (q)
            5'd0:  byte_ok = "{";
            5'd1:  byte_ok = 8'h22;      // "
            5'd2:  byte_ok = "i";
            5'd3:  byte_ok = 8'h22;      // "
            5'd4:  byte_ok = ":";
            5'd5:  byte_ok = hexch(idx);
            5'd6:  byte_ok = ",";
            5'd7:  byte_ok = 8'h22;      // "
            5'd8:  byte_ok = "v";
            5'd9:  byte_ok = 8'h22;      // "
            5'd10: byte_ok = ":";
            5'd11: byte_ok = "0";
            5'd12: byte_ok = "x";
            5'd21: byte_ok = "}";
            5'd22: byte_ok = 8'h0A;      // newline, so records cannot run together
            // 13..20 only.  The index is written out in full rather than as
            // ``7 - (q - 13)`` so that a byte falling in here by accident cannot
            // produce a negative bit select: byte 21 was the tail bug, and the
            // subtraction form turned it into an x on the wire.
            default: byte_ok = hexch(cur[{hex_i, 2'b00} +: 4]);
        endcase
    end

    // The teeth.  Each is a plausible misreading of the table at the top of this
    // file, and tools/run_b0_telemetry.sh requires the testbench to reject every one
    // of them.  Constant-folded away at SABOTAGE = 0.
    reg [7:0] byte_out;
    always @* begin
        byte_out = byte_ok;
        case (SABOTAGE)
            2: if (q >= 5'd13 && q <= 5'd20)
                   byte_out = hexch(cur[(q - 5'd13) * 4 +: 4]);  // most significant last
            3: if (byte_ok >= "a" && byte_ok <= "f")
                   byte_out = byte_ok - 8'h20;                   // upper-case hex
            4: if (q == 5'd5)  byte_out = "0";                   // index pinned to zero
            5: if (q == 5'd9)  byte_out = ":";                   // the key's quote lost
            6: if (q == 5'd12) byte_out = "X";                   // 0X rather than 0x
            7: if (q == 5'd0)  byte_out = " ";                   // the opening brace lost
        endcase
    end

    localparam S_IDLE = 2'd0, S_RUN = 2'd1, S_DONE = 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; idx <= 0; q <= 5'd0; busy <= 1'b0; done <= 1'b0;
            u_valid <= 1'b0; u_data <= 8'h00; cur <= 32'd0;
        end
        else begin
            done <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        idx  <= 0;
                        q    <= 5'd0;
                        cur  <= values[0 +: 32];
                        st   <= S_RUN;
                    end
                end

                // The byte handshake, and the second half of the bug above.
                // ``u_ready`` is high whenever the serialiser is *idle*, which is
                // before it has taken the byte being presented -- so advancing on
                // ``u_ready`` alone advances twice per byte and puts only every
                // second byte on the wire.  The old testbench was fitted to that,
                // which is why it reported content mismatches instead of a missing
                // half.  A byte is retired only on ``u_valid && u_ready``, the cycle
                // the serialiser actually latches it; the valid is dropped for one
                // cycle so the next byte cannot be taken in the same breath.
                S_RUN: begin
                    if (u_valid && u_ready) begin
                        u_valid <= 1'b0;
                        if (q == Q_LAST) begin
                            q <= 5'd0;
                            if (idx == N_RESULTS - 1) begin
                                st   <= S_DONE;
                                busy <= 1'b0;
                            end
                            else begin
                                idx <= idx + 1'b1;
                                cur <= values[(idx + 1'b1) * 32 +: 32];
                            end
                        end
                        else
                            q <= q + 5'd1;
                    end
                    else if (!u_valid) begin
                        u_valid <= 1'b1;
                        u_data  <= byte_out;
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
            endcase
        end
    end

endmodule
