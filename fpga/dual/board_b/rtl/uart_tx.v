// A UART transmitter, 8N1, with a byte handshake.
//
// This exists so that every board measurement this project takes can come back as
// machine-readable text instead of a photograph of a screen.  The devkit carries an
// FT4232H wired to the board's UART, so a bitstream can report its own results with
// no extra hardware and no logic analyser.
//
// No RAM.  A shift register and a baud counter, because M1 closes at 252 of 256
// RAM10 and none of them are going to a UART.
//
// Board pins, from the official demos at Ti60F225_DemoBoard_v4 (17_uart_demo):
//
//     txd   GPIOR_28   output, 3.3 V LVCMOS
//     rxd   GPIOL_02   input,  3.3 V LVCMOS
//
// An earlier version of this file said GPIOL_01/GPIOL_02, which is half wrong: the
// transmit pin is on the other side of the device.  A transmitter built to the wrong
// pin produces silence, which is the failure that looks most like a broken design.
//
// CLK_HZ is the frequency of whatever clock this is instantiated with, and it has to
// be the real one: the board has 25, 27 and 50 MHz oscillators selected by clk_sel,
// so the baud rate is only correct if the caller says which one is in use.
module uart_tx #(
    parameter CLK_HZ   = 50_000_000,
    parameter BAUD     = 115200,
    // Test-only.  Each value is a plausible way to get the serialiser wrong and
    // tb_bench_telemetry must reject every one of them.  0 is the design, and the
    // branches constant-fold away at 0 so the synthesised logic is unaffected.
    parameter SABOTAGE = 0
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] data,
    input  wire       valid,
    output reg        ready,

    output reg        tx
);

    localparam integer DIV_BASE = (CLK_HZ + BAUD / 2) / BAUD;

    // Tooth 2: a divisor one too long is a 1% baud error per bit at 1 MHz/100 kbaud
    // -- 10% accumulated over the ten bits of a frame, which a receiver sampling at
    // bit midpoints cannot survive.  It is the shape of a wrong CLK_HZ.
    localparam integer DIV = (SABOTAGE == 2) ? DIV_BASE + 1 : DIV_BASE;

    localparam integer DW = $clog2(DIV + 1);

    // Tooth 1: the data bits, most significant first.  UART is LSB-first, and a
    // reversal is invisible on a scope but fatal to every host that reads it.
    wire [7:0] tx_byte = (SABOTAGE == 1)
                       ? {data[0], data[1], data[2], data[3],
                          data[4], data[5], data[6], data[7]}
                       : data;

    reg [DW-1:0]  baud_cnt;
    reg [3:0]     bit_i;
    reg [9:0]     shift;      // start, 8 data, stop
    reg           busy;

    // Ready while idle.  The caller holds ``valid`` for one cycle.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready    <= 1'b1;
            busy     <= 1'b0;
            tx       <= 1'b1;      // idle high
            baud_cnt <= 0;
            bit_i    <= 4'd0;
            shift    <= 10'h3FF;
        end
        else if (!busy) begin
            tx    <= 1'b1;
            ready <= 1'b1;
            if (valid) begin
                shift    <= {1'b1, tx_byte, 1'b0};   // stop, data[7:0], start
                busy     <= 1'b1;
                ready    <= 1'b0;
                baud_cnt <= 0;
                bit_i    <= 4'd0;
            end
        end
        else begin
            ready <= 1'b0;
            if (baud_cnt == DIV - 1) begin
                baud_cnt <= 0;
                tx       <= shift[0];
                shift    <= {1'b1, shift[9:1]};
                if (bit_i == 4'd9) begin
                    busy  <= 1'b0;
                    ready <= 1'b1;
                    tx    <= 1'b1;
                end
                else
                    bit_i <= bit_i + 4'd1;
            end
            else
                baud_cnt <= baud_cnt + 1'b1;
        end
    end

endmodule
