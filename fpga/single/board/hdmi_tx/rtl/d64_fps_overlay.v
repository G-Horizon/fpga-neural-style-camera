`timescale 1ns/1ps
// Compact3x5 glyphs, each pixel8x8. Digits24x40px, 32px pitch.
module d64_fps_overlay(
 input wire enabled,input wire[2:0]valid,input wire[15:0]camera_bcd,style_bcd,scan_bcd,
 input wire de_i,input wire[11:0]ax_i,ay_i,input wire[23:0]rgb_i,output wire[23:0]rgb_o,
 output wire overlay_active,overlay_ink
);
 wire[4:0]character=ax_i[9:5]-5'd1;wire[1:0]column=ax_i[4:3];
 reg[1:0]line;reg[2:0]glyph_row;reg[15:0]value;reg value_valid;
 reg[4:0]glyph;reg[14:0]font;reg[2:0]row;
 always @*begin
  if(ay_i<64)begin line=0;glyph_row=ay_i[5:3]-3'd2;value=camera_bcd;value_valid=valid[0];end
  else if(ay_i<112)begin line=1;glyph_row=ay_i[5:3];value=style_bcd;value_valid=valid[1];end
  else begin line=2;glyph_row=ay_i[5:3]-3'd6;value=scan_bcd;value_valid=valid[2];end
  glyph=5'd31;
  case(line)
   0:case(character)0:glyph=5'd13;1:glyph=5'd12;2:glyph=5'd20;4:glyph=5'd18;5:glyph=5'd21;default:glyph=31;endcase
   1:case(character)0:glyph=5'd23;1:glyph=5'd24;2:glyph=5'd26;3:glyph=5'd19;4:glyph=5'd15;6:glyph=5'd21;7:glyph=5'd15;8:glyph=5'd25;default:glyph=31;endcase
   2:case(character)0:glyph=5'd17;1:glyph=5'd14;2:glyph=5'd20;3:glyph=5'd18;5:glyph=5'd23;6:glyph=5'd13;7:glyph=5'd12;8:glyph=5'd21;default:glyph=31;endcase
  endcase
  case(character)
   11:glyph=!value_valid?5'd0:(value[15:12]==0?5'd31:(value[15:12]<10?value[15:12]+5'd2:5'd31));
   12:glyph=!value_valid?5'd0:(value[11:8]<10?value[11:8]+5'd2:5'd31);
   14:glyph=!value_valid?5'd0:(value[7:4]<10?value[7:4]+5'd2:5'd31);
   15:glyph=!value_valid?5'd0:(value[3:0]<10?value[3:0]+5'd2:5'd31);
   13:glyph=5'd1;
   17:glyph=line==2?5'd17:5'd16;
   18:glyph=line==2?5'd27:5'd22;
   19:glyph=line==2?5'd31:5'd23;
   default:begin end
  endcase
  case(glyph)
   5'd2:font=15'b111_101_101_101_111;
   5'd3:font=15'b010_110_010_010_111;
   5'd4:font=15'b110_001_010_100_111;
   5'd5:font=15'b110_001_010_001_110;
   5'd6:font=15'b101_101_111_001_001;
   5'd7:font=15'b111_100_110_001_110;
   5'd8:font=15'b011_100_111_101_111;
   5'd9:font=15'b111_001_010_010_010;
   5'd10:font=15'b111_101_111_101_111;
   5'd11:font=15'b111_101_111_001_110;
   5'd12:font=15'b010_101_111_101_101;
   5'd13:font=15'b011_100_100_100_011;
   5'd14:font=15'b110_101_101_101_110;
   5'd15:font=15'b111_100_110_100_111;
   5'd16:font=15'b111_100_110_100_100;
   5'd17:font=15'b101_101_111_101_101;
   5'd18:font=15'b111_010_010_010_111;
   5'd19:font=15'b100_100_100_100_111;
   5'd20:font=15'b101_111_111_101_101;
   5'd21:font=15'b101_111_111_111_101;
   5'd22:font=15'b110_101_110_100_100;
   5'd23:font=15'b011_100_010_001_110;
   5'd24:font=15'b111_010_010_010_010;
   5'd25:font=15'b101_101_111_111_101;
   5'd26:font=15'b101_101_010_010_010;
   5'd27:font=15'b111_001_010_100_111;
   5'd1:font=15'b000_000_000_000_010;
   5'd0:font=15'b000_000_111_000_000;
   default:font=0;
  endcase
  case(glyph_row)0:row=font[14:12];1:row=font[11:9];2:row=font[8:6];3:row=font[5:3];4:row=font[2:0];default:row=0;endcase
 end
 assign overlay_active=enabled&&de_i&&ax_i>=32&&ax_i<672&&ay_i>=16&&ay_i<160;
 assign overlay_ink=overlay_active&&column<3&&row[2-column];
 assign rgb_o=overlay_active?(overlay_ink?24'hffffff:24'h000000):rgb_i;
endmodule
