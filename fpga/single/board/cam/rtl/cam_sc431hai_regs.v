// The SC431HAI's power-up register table, as the vendor's own demo writes it.
//
// **Provenance, because this is not our table.**  It is transcribed from
// `Ti60F225_DemoBoard_v4/07_Ti60F225_MIPI_C2D_Demo/Ti60f225_csi2dsi_sc431hai_v3.4/
// rtl/cam_i2c_ctrl/i2c/i2c_master_reg_rom.v`, live (uncommented) entries only --
// 161 of them, a dense run of ROM indices, covering sensor addresses
// 0x0100..0x3213 and every one of them a write.
//
// **One entry was added, and it is the datasheet's, not the vendor's.**  The
// sensor manual gives the stream control as two writes whose order it says
// explicitly must not be changed:
//
//     stream off   16'h302c = 8'h0f,  then  16'h0100 = 8'h00
//     stream on    16'h302c = 8'h00,  then  16'h0100 = 8'h01
//
// The vendor's table writes `0x0100 = 0x01` with no `0x302c` write in front of
// it, so the pair is half there.  `0x302c` defaults to `0x00`, which is why the
// link works without it -- and that is exactly the reason to write it: the
// configuration currently depends on an unread reset value being what the
// datasheet says it is, and this removes the dependence.  The write is a no-op
// when the default holds, so it cannot change a working link.
//
// **The stream-off half was deliberately not added.**  Doing so would mean
// writing `0x302c = 0x0f` as the table's first entry -- a new state transition
// in a configuration that currently works, which the plan did not ask for and
// which would need its own board evidence.  The sensor is off at power-up and
// the table's existing `0x0100 = 0x00` is already a no-op there.
//
// **The distinction between live and commented is the whole trap in that file.**
// It carries a second, commented-out table of IMX185 registers (0x3002, 0x30eb,
// 0x0114 ...) left from an earlier revision of the project, and the two look
// alike at a glance.  Reading the wrong one configures an IMX185 that is not
// plugged in and produces exactly the symptom of a broken camera: silence, with
// nothing to say which table was sent.
//
// The first entry is 0x0100 = 0x00, stream off, which is what a table like this
// has to start with.  The last is 0x3213 = 0x05, and the added `0x302c`
// sits immediately before the stream-on write at index 145.
//
// A `case` and not a memory: an inferred ROM needs an initialisation file, and
// this project has already lost a day to a `$readmemh`/init path that was
// resolved against the working directory rather than the source tree.  161
// entries of 24 bits is about a thousand LUT4s, and this design has room.
module cam_sc431hai_regs #(
    // **The output window, and it is coupled to the link rate -- one decision.**
    //
    // 0 selects the vendor's table exactly as it shipped: a 1920x1080 window
    // positioned at column 324, row 5, inside the 2568x1448 array.  The datasheet's
    // table 2-12 (section 2.7.2) fixes the meaning of the four register pairs:
    //
    //     {0x3208,0x3209}  window WIDTH    default 0x0a00 = 2560
    //     {0x320a,0x320b}  window HEIGHT   default 0x05a0 = 1440
    //     {0x3210,0x3211}  column start    default 0x0004
    //     {0x3212,0x3213}  row start       default 0x0004
    //
    // so 1 restores the full array by writing those defaults back.
    //
    // **Do not set this without also changing the receiver's byte clock.**  At 60 fps
    // and four lanes, RAW10 needs 552.96 Mbit/s a lane for 2560x1440, which is a
    // 69.1 MHz byte clock before a single byte of CSI-2 overhead -- against a
    // declared `HS_BYTECLK_MHZ = 70`.  The 1080p window needs 38.9 MHz and is why 70
    // was chosen.  The byte clock is not generated here: it is the sensor's own clock
    // lane, recovered by the hard D-PHY and fed to the controller's `clk_byte_HS`, so
    // the window and the link rate have to move together or the link simply breaks --
    // and it breaks in a way that reads as a camera fault rather than as a
    // configuration mismatch.
    parameter integer FULL_ARRAY = 0
) (
    input  wire [7:0]  k,      // ROM index, one register a cycle
    output reg  [15:0] a,      // sensor register address
    output reg  [7:0]  d       // value to write
);
    always @* begin
        a = 16'h0000; d = 8'h00;
        case (k)
                10'd0: begin a = 16'h0100; d = 8'h00; end
                10'd1: begin a = 16'h36e9; d = 8'h80; end
                10'd2: begin a = 16'h37f9; d = 8'h80; end
                10'd3: begin a = 16'h301f; d = 8'h10; end
                10'd4: begin a = 16'h3058; d = 8'h21; end
                10'd5: begin a = 16'h3059; d = 8'h53; end
                10'd6: begin a = 16'h305a; d = 8'h40; end
                10'd7: begin a = 16'h3250; d = 8'h00; end
                10'd8: begin a = 16'h3301; d = 8'h0c; end
                10'd9: begin a = 16'h3304; d = 8'h50; end
                10'd10: begin a = 16'h3305; d = 8'h00; end
                10'd11: begin a = 16'h3306; d = 8'h50; end
                10'd12: begin a = 16'h3307; d = 8'h04; end
                10'd13: begin a = 16'h3308; d = 8'h0a; end
                10'd14: begin a = 16'h3309; d = 8'h60; end
                10'd15: begin a = 16'h330b; d = 8'hc8; end
                10'd16: begin a = 16'h330d; d = 8'h08; end
                10'd17: begin a = 16'h330e; d = 8'h38; end
                10'd18: begin a = 16'h331e; d = 8'h41; end
                10'd19: begin a = 16'h331f; d = 8'h51; end
                10'd20: begin a = 16'h3333; d = 8'h10; end
                10'd21: begin a = 16'h3334; d = 8'h40; end
                10'd22: begin a = 16'h3364; d = 8'h5e; end
                10'd23: begin a = 16'h338e; d = 8'he2; end
                10'd24: begin a = 16'h338f; d = 8'h80; end
                10'd25: begin a = 16'h3390; d = 8'h08; end
                10'd26: begin a = 16'h3391; d = 8'h18; end
                10'd27: begin a = 16'h3392; d = 8'hb8; end
                10'd28: begin a = 16'h3393; d = 8'h12; end
                10'd29: begin a = 16'h3394; d = 8'h14; end
                10'd30: begin a = 16'h3395; d = 8'h10; end
                10'd31: begin a = 16'h3396; d = 8'h88; end
                10'd32: begin a = 16'h3397; d = 8'h98; end
                10'd33: begin a = 16'h3398; d = 8'hb8; end
                10'd34: begin a = 16'h3399; d = 8'h10; end
                10'd35: begin a = 16'h339a; d = 8'h16; end
                10'd36: begin a = 16'h339b; d = 8'h1c; end
                10'd37: begin a = 16'h339c; d = 8'h40; end
                10'd38: begin a = 16'h33ac; d = 8'h0a; end
                10'd39: begin a = 16'h33ad; d = 8'h10; end
                10'd40: begin a = 16'h33ae; d = 8'h4f; end
                10'd41: begin a = 16'h33af; d = 8'h5e; end
                10'd42: begin a = 16'h33b2; d = 8'h50; end
                10'd43: begin a = 16'h33b3; d = 8'h10; end
                10'd44: begin a = 16'h33f8; d = 8'h00; end
                10'd45: begin a = 16'h33f9; d = 8'h50; end
                10'd46: begin a = 16'h33fa; d = 8'h00; end
                10'd47: begin a = 16'h33fb; d = 8'h50; end
                10'd48: begin a = 16'h33fc; d = 8'h48; end
                10'd49: begin a = 16'h33fd; d = 8'h78; end
                10'd50: begin a = 16'h349f; d = 8'h03; end
                10'd51: begin a = 16'h34a6; d = 8'h40; end
                10'd52: begin a = 16'h34a7; d = 8'h58; end
                10'd53: begin a = 16'h34a8; d = 8'h10; end
                10'd54: begin a = 16'h34a9; d = 8'h10; end
                10'd55: begin a = 16'h34f8; d = 8'h78; end
                10'd56: begin a = 16'h34f9; d = 8'h10; end
                10'd57: begin a = 16'h3633; d = 8'h44; end
                10'd58: begin a = 16'h363b; d = 8'h8f; end
                10'd59: begin a = 16'h363c; d = 8'h02; end
                10'd60: begin a = 16'h3641; d = 8'h08; end
                10'd61: begin a = 16'h3654; d = 8'h20; end
                10'd62: begin a = 16'h3674; d = 8'hc2; end
                10'd63: begin a = 16'h3675; d = 8'hb4; end
                10'd64: begin a = 16'h3676; d = 8'h88; end
                10'd65: begin a = 16'h367c; d = 8'h88; end
                10'd66: begin a = 16'h367d; d = 8'hb8; end
                10'd67: begin a = 16'h3690; d = 8'h34; end
                10'd68: begin a = 16'h3691; d = 8'h44; end
                10'd69: begin a = 16'h3692; d = 8'h54; end
                10'd70: begin a = 16'h3693; d = 8'h88; end
                10'd71: begin a = 16'h3694; d = 8'h98; end
                10'd72: begin a = 16'h3696; d = 8'h80; end
                10'd73: begin a = 16'h3697; d = 8'h83; end
                10'd74: begin a = 16'h3698; d = 8'h81; end
                10'd75: begin a = 16'h3699; d = 8'h81; end
                10'd76: begin a = 16'h369a; d = 8'h84; end
                10'd77: begin a = 16'h369b; d = 8'h82; end
                10'd78: begin a = 16'h36a2; d = 8'h80; end
                10'd79: begin a = 16'h36a3; d = 8'h88; end
                10'd80: begin a = 16'h36a4; d = 8'hf8; end
                10'd81: begin a = 16'h36a5; d = 8'hb8; end
                10'd82: begin a = 16'h36a6; d = 8'h98; end
                10'd83: begin a = 16'h36d0; d = 8'h15; end
                10'd84: begin a = 16'h36ea; d = 8'h23; end
                10'd85: begin a = 16'h36eb; d = 8'h0d; end
                10'd86: begin a = 16'h36ec; d = 8'h65; end
                10'd87: begin a = 16'h36ed; d = 8'h18; end
                10'd88: begin a = 16'h370f; d = 8'h01; end
                10'd89: begin a = 16'h3722; d = 8'h03; end
                10'd90: begin a = 16'h3724; d = 8'h92; end
                10'd91: begin a = 16'h3727; d = 8'h14; end
                10'd92: begin a = 16'h37b0; d = 8'h17; end
                10'd93: begin a = 16'h37b1; d = 8'h9b; end
                10'd94: begin a = 16'h37b2; d = 8'h9b; end
                10'd95: begin a = 16'h37b3; d = 8'h88; end
                10'd96: begin a = 16'h37b4; d = 8'hb8; end
                10'd97: begin a = 16'h37fa; d = 8'h23; end
                10'd98: begin a = 16'h37fb; d = 8'h54; end
                10'd99: begin a = 16'h37fc; d = 8'h21; end
                10'd100: begin a = 16'h37fd; d = 8'h1c; end
                10'd101: begin a = 16'h391f; d = 8'h41; end
                10'd102: begin a = 16'h3926; d = 8'he0; end
                10'd103: begin a = 16'h3933; d = 8'h80; end
                10'd104: begin a = 16'h3934; d = 8'hf8; end
                10'd105: begin a = 16'h3935; d = 8'h00; end
                10'd106: begin a = 16'h3936; d = 8'h45; end
                10'd107: begin a = 16'h3937; d = 8'h66; end
                10'd108: begin a = 16'h3938; d = 8'h66; end
                10'd109: begin a = 16'h3939; d = 8'h00; end
                10'd110: begin a = 16'h393a; d = 8'h03; end
                10'd111: begin a = 16'h393b; d = 8'h00; end
                10'd112: begin a = 16'h393c; d = 8'h00; end
                10'd113: begin a = 16'h393d; d = 8'h02; end
                10'd114: begin a = 16'h393e; d = 8'h80; end
                10'd115: begin a = 16'h3e00; d = 8'h00; end
                10'd116: begin a = 16'h3e01; d = 8'hba; end
                10'd117: begin a = 16'h3e02; d = 8'hd0; end
                10'd118: begin a = 16'h3e16; d = 8'h00; end
                10'd119: begin a = 16'h3e17; d = 8'hc5; end
                10'd120: begin a = 16'h3e18; d = 8'h00; end
                10'd121: begin a = 16'h3e19; d = 8'hc5; end
                10'd122: begin a = 16'h4509; d = 8'h20; end
                10'd123: begin a = 16'h450d; d = 8'h0b; end
                10'd124: begin a = 16'h5780; d = 8'h76; end
                10'd125: begin a = 16'h5784; d = 8'h0a; end
                10'd126: begin a = 16'h5785; d = 8'h04; end
                10'd127: begin a = 16'h5787; d = 8'h0a; end
                10'd128: begin a = 16'h5788; d = 8'h0a; end
                10'd129: begin a = 16'h5789; d = 8'h08; end
                10'd130: begin a = 16'h578a; d = 8'h0a; end
                10'd131: begin a = 16'h578b; d = 8'h0a; end
                10'd132: begin a = 16'h578c; d = 8'h08; end
                10'd133: begin a = 16'h578d; d = 8'h40; end
                10'd134: begin a = 16'h5790; d = 8'h08; end
                10'd135: begin a = 16'h5791; d = 8'h04; end
                10'd136: begin a = 16'h5792; d = 8'h04; end
                10'd137: begin a = 16'h5793; d = 8'h08; end
                10'd138: begin a = 16'h5794; d = 8'h04; end
                10'd139: begin a = 16'h5795; d = 8'h04; end
                10'd140: begin a = 16'h57ac; d = 8'h00; end
                10'd141: begin a = 16'h57ad; d = 8'h00; end
                10'd142: begin a = 16'h36e9; d = 8'h53; end
                10'd143: begin a = 16'h37f9; d = 8'h53; end
                10'd144: begin a = 16'h302c; d = 8'h00; end
                10'd145: begin a = 16'h0100; d = 8'h01; end
                10'd146: begin a = 16'h3200; d = 8'h00; end
                10'd147: begin a = 16'h3201; d = 8'h00; end
                10'd148: begin a = 16'h3202; d = 8'h00; end
                10'd149: begin a = 16'h3203; d = 8'hb3; end
                10'd150: begin a = 16'h3204; d = 8'h0a; end
                10'd151: begin a = 16'h3205; d = 8'h07; end
                10'd152: begin a = 16'h3206; d = 8'h04; end
                10'd153: begin a = 16'h3207; d = 8'hf4; end
                10'd154: begin a = 16'h3208; d = 8'h07; end
                10'd155: begin a = 16'h3209; d = 8'h80; end
                10'd156: begin a = 16'h320a; d = 8'h04; end
                10'd157: begin a = 16'h320b; d = 8'h38; end
                10'd158: begin a = 16'h3210; d = 8'h01; end
                10'd159: begin a = 16'h3211; d = 8'h44; end
                10'd160: begin a = 16'h3212; d = 8'h00; end
                10'd161: begin a = 16'h3213; d = 8'h05; end
                default: begin a = 16'h0000; d = 8'h00; end
        endcase

        // The window, applied after the table so the vendor's run is untouched and the
        // override is visible in one place.  Indices 154..161 are 0x3208..0x3213.
        if (FULL_ARRAY) begin
            case (k)
                10'd154: d = 8'h0a;    // 0x3208  width  [15:8] = 0x0a00 = 2560
                10'd155: d = 8'h00;    // 0x3209  width  [ 7:0]
                10'd156: d = 8'h05;    // 0x320a  height [15:8] = 0x05a0 = 1440
                10'd157: d = 8'ha0;    // 0x320b  height [ 7:0]
                10'd158: d = 8'h00;    // 0x3210  col start [15:8] = 0x0004
                10'd159: d = 8'h04;    // 0x3211
                10'd160: d = 8'h00;    // 0x3212  row start [15:8] = 0x0004
                10'd161: d = 8'h04;    // 0x3213
                default: begin end
            endcase
        end
    end
endmodule
