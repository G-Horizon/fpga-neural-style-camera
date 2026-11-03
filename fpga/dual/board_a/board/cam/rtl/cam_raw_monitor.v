// What the CSI-2 receiver is actually delivering, as eight numbers.
//
// A camera bring-up has one question -- are pixels arriving -- and a board has no
// screen in front of whoever is reading the capture.  So the monitor answers it with
// counts and a fingerprint rather than with a picture:
//
//   beats, lines, frames        how much arrived, and in what shape
//   crc                         32-bit CRC32 over a decimated sample of the payload
//   beat0, beat1                the first two beats, verbatim
//   status                      datatype, pixel_per_clk, the marker levels, and nacks
//   i2c                         first NACK index, last NACK index, and how many
//                               register writes landed (the nack *count* is in
//                               `status`)
//
// **The CRC is over every 64th beat, and that is a design decision rather than a
// shortcut.**  A fair CRC32 of 64 bits needs eight byte-steps, and byte-stepping
// eight times in one cycle is thirty-two LUT levels: it does not close at 100 MHz.
// Decimating by 64 gives sixty-four cycles for eight byte-steps, which is one
// byte-step a cycle and trivial timing -- and it is a *defined* subset, so a model
// can reproduce the same number exactly.  An accumulator would have been cheaper
// still and would have thrown away the order, which is the only thing a fingerprint
// is for.
//
// `datatype` is latched rather than counted: RAW10 is 0x2B and anything else means
// the sensor is in a mode nobody asked for, which is worth one number.
module cam_raw_monitor (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        pix_valid,
    input  wire [63:0] pix_data,
    input  wire [3:0]  pix_per_clk,
    input  wire [5:0]  datatype,
    input  wire        hsync,
    input  wire        vsync,
    input  wire        irq,

    input  wire        i2c_done,
    input  wire [15:0] i2c_writes,
    input  wire [15:0] i2c_nacks,
    // **The ROM index of the first write the sensor did not acknowledge**, 255 when
    // there was none.  A count says a table has a bad entry; the index says which
    // one, which is the difference between a next step and a guess.
    input  wire [7:0]  i2c_first_nack,
    // **And the last one, because the first one is not the interesting one.**
    // Index 0 is `0x0100 = 0x00`, stream off.  Whether the *stream-on* write
    // near the end of the table landed is a different index, and a sensor that
    // was never told to stream enables no data lane and reads exactly like a
    // broken receiver.
    input  wire [7:0]  i2c_last_nack,

    output reg  [31:0] v_beats,
    output reg  [31:0] v_lines,
    output reg  [31:0] v_frames,
    output reg  [31:0] v_crc,
    output reg  [31:0] v_beat0,
    output reg  [31:0] v_beat1,
    output reg  [31:0] v_status,
    output reg  [31:0] v_i2c
);

    reg [31:0] beats, lines, frames, crc_beats;
    reg [5:0]  beat_ix;
    reg        beat_arm;
    reg        hs_q, vs_q;
    reg [3:0]  irq_cnt;
    reg        irq_sticky;
    reg [2:0]  crc_ph;
    reg [31:0] crc;
    reg [63:0] hold;
    reg        crc_run;
    reg [2:0]  by;
    integer    bi;

    wire hs_rise = hsync & ~hs_q;
    wire vs_rise = vsync & ~vs_q;
    wire take    = pix_valid && (beat_ix == 6'd0);

    // One byte of the CRC32 an iteration: the polynomial is the one the rest of this
    // project uses (0xEDB88320, reflected), so a model and a board agree on the
    // constant.
    wire [31:0] crc_x = crc ^ {24'd0, hold[by*8 +: 8]};
    reg  [31:0] crc_n;
    always @* begin
        crc_n = crc_x;
        for (bi = 0; bi < 8; bi = bi + 1)
            crc_n = crc_n[0] ? (crc_n >> 1) ^ 32'hEDB88320 : (crc_n >> 1);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beats <= 32'd0; lines <= 32'd0; frames <= 32'd0; crc_beats <= 32'd0;
            beat_ix <= 6'd0; hs_q <= 1'b0; vs_q <= 1'b0; beat_arm <= 1'b1;
            irq_cnt <= 4'd0; irq_sticky <= 1'b0;
            crc_ph <= 3'd0; crc <= 32'hFFFFFFFF; hold <= 64'd0; crc_run <= 1'b0;
            by <= 3'd0;
            v_beats <= 32'd0; v_lines <= 32'd0; v_frames <= 32'd0; v_crc <= 32'd0;
            v_beat0 <= 32'd0; v_beat1 <= 32'd0; v_status <= 32'd0; v_i2c <= 32'd0;
        end
        else begin
            hs_q <= hsync;
            vs_q <= vsync;
            if (irq && (irq_cnt != 4'hF)) irq_cnt <= irq_cnt + 4'd1;
            if (irq) irq_sticky <= 1'b1;

            if (pix_valid) begin
                beats   <= beats + 32'd1;
                beat_ix <= beat_ix + 6'd1;
            end
            // **One whole 64-bit beat per frame, and this time it is a beat.**
            //
            // It used to latch `pix_data[31:0]` on `beats == 0` and again on
            // `beats == 1`, so the pair was two *different* beats' low halves,
            // **the top 32 bits of every beat were never observed at all**, and
            // the value was frozen from reset onward.  A capture proved it: 902
            // telemetry pages carrying `i:4`, one distinct value.  The plan asks
            // for the full 64-bit sample recorded on the board, and this is what
            // stood in the way -- not a missing model.
            //
            // Re-armed on every vsync, so consecutive pages carry different
            // pixels and `{v_beat1, v_beat0}` is one beat rather than two.  A
            // single word could never have identified the packing: what decides
            // it is which bit positions are *constant across beats*, and that
            // needs more than one.
            if (vs_rise) beat_arm <= 1'b1;
            if (pix_valid && beat_arm) begin
                v_beat0  <= pix_data[31:0];
                v_beat1  <= pix_data[63:32];
                beat_arm <= 1'b0;
            end
            if (hs_rise) lines  <= lines + 32'd1;
            if (vs_rise) frames <= frames + 32'd1;
            // `[31:24]` stays zero, `[23:16]` is the first NACKed write's index and
            // `[15:0]` is the write count.  It used to be `{16'd0, i2c_writes}`, and
            // the high half being always zero is what a reader once took for a
            // field -- so the layout is written down here and read from here.
            // **Repacked, because 16 bits of write count for a 161-entry table
            // is 8 bits of field and 8 bits of nothing**, and the second NACK
            // index is worth more than the padding.  `[31:24]` first NACK,
            // `[23:16]` last NACK, `[15:8]` writes, `[7:0]` zero.  `i2c_writes`
            // is still compared against 161 in this module's callers, which is
            // why the zero half is at the bottom rather than the top.
            if (i2c_done)
                v_i2c <= {i2c_first_nack, i2c_last_nack,
                          i2c_writes[7:0], 8'd0};

            // **Started only when the CRC engine is idle**, which it always is:
            // eight byte-steps take eight cycles and the next sample is sixty-four
            // beats away.  v_crc is published at the end of each sample so a capture
            // taken at any moment is a real CRC of a real prefix.
            if (take && !crc_run) begin
                hold    <= pix_data;
                by      <= 3'd0;
                crc_run <= 1'b1;
                crc_ph  <= 3'd0;
                crc     <= 32'hFFFFFFFF;
            end
            else if (crc_run) begin
                crc   <= crc_n;
                by    <= by + 3'd1;
                crc_ph <= crc_ph + 3'd1;
                if (crc_ph == 3'd7) begin
                    crc_run   <= 1'b0;
                    crc_beats <= crc_beats + 32'd1;
                    v_crc     <= ~crc_n;
                end
            end

            v_beats  <= beats;
            v_lines  <= lines;
            v_frames <= frames;
            // **Exactly thirty-two bits, and the arithmetic is written down
            // because a concatenation one bit over silently truncates.**
            // 6 datatype + 4 pixel_per_clk + 1 irq_sticky + 1 vsync + 1 hsync
            // + 16 i2c_nacks + 3 spare = 32.
            v_status <= {datatype, pix_per_clk, irq_sticky, vsync, hsync,
                         i2c_nacks, 3'd0};
        end
    end

endmodule
