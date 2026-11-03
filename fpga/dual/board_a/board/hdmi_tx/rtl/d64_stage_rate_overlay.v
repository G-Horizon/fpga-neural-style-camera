`timescale 1ns/1ps
// 5x7 glyphs at 4x scale. 3 distinct measured rates, not scan Hz labelled as CNN fps.
module d64_stage_rate_overlay(
 input wire enabled,input wire[2:0] valid,
 input wire[19:0] camera_bcd,style_bcd,scan_bcd,
 input wire de_i,input wire[11:0] ax_i,ay_i,
 input wire[23:0] rgb_i,output wire[23:0] rgb_o,
 output wire overlay_active,overlay_ink
);
 wire[11:0] dx=ax_i-12'd16;
 wire[4:0] character=dx[9:5];wire[2:0] column=dx[4:2];
 reg[1:0] line;reg[11:0] dy;reg[19:0] value;reg value_valid;
 reg[7:0] glyph;reg[34:0] font;reg[4:0] row;
 always @*begin
  if(ay_i<56)begin line=0;dy=ay_i-16;value=camera_bcd;value_valid=valid[0];end
  else if(ay_i<96)begin line=1;dy=ay_i-56;value=style_bcd;value_valid=valid[1];end
  else begin line=2;dy=ay_i-96;value=scan_bcd;value_valid=valid[2];end
  glyph=" ";
  if(character<9)begin
   case(line)
    0:case(character)0:glyph="C";1:glyph="A";2:glyph="M";4:glyph="I";5:glyph="N";default:glyph=" ";endcase
    1:case(character)0:glyph="S";1:glyph="T";2:glyph="Y";3:glyph="L";4:glyph="E";6:glyph="N";7:glyph="E";8:glyph="W";default:glyph=" ";endcase
    2:case(character)0:glyph="H";1:glyph="D";2:glyph="M";3:glyph="I";5:glyph="S";6:glyph="C";7:glyph="A";8:glyph="N";default:glyph=" ";endcase
   endcase
  end else case(character)
   10:glyph=!value_valid?"-":(value[19:16]!=0?8'd48+value[19:16]:" ");
   11:glyph=!value_valid?"-":((value[19:12]!=0)?8'd48+value[15:12]:" ");
   12:glyph=!value_valid?"-":8'd48+value[11:8];
   13:glyph=".";
   14:glyph=!value_valid?"-":8'd48+value[7:4];
   15:glyph=!value_valid?"-":8'd48+value[3:0];
   17:glyph=line==2?"H":"F";
   18:glyph=line==2?"Z":"P";
   19:glyph=line==2?" ":"S";
   default:glyph=" ";
  endcase
  case(glyph)
   "0":font=35'b01110_10001_10011_10101_11001_10001_01110;
   "1":font=35'b00100_01100_00100_00100_00100_00100_01110;
   "2":font=35'b01110_10001_00001_00010_00100_01000_11111;
   "3":font=35'b11110_00001_00001_01110_00001_00001_11110;
   "4":font=35'b00010_00110_01010_10010_11111_00010_00010;
   "5":font=35'b11111_10000_10000_11110_00001_00001_11110;
   "6":font=35'b01110_10000_10000_11110_10001_10001_01110;
   "7":font=35'b11111_00001_00010_00100_01000_01000_01000;
   "8":font=35'b01110_10001_10001_01110_10001_10001_01110;
   "9":font=35'b01110_10001_10001_01111_00001_00001_01110;
   "A":font=35'b01110_10001_10001_11111_10001_10001_10001;
   "C":font=35'b01110_10001_10000_10000_10000_10001_01110;
   "D":font=35'b11110_10001_10001_10001_10001_10001_11110;
   "E":font=35'b11111_10000_10000_11110_10000_10000_11111;
   "F":font=35'b11111_10000_10000_11110_10000_10000_10000;
   "H":font=35'b10001_10001_10001_11111_10001_10001_10001;
   "I":font=35'b01110_00100_00100_00100_00100_00100_01110;
   "L":font=35'b10000_10000_10000_10000_10000_10000_11111;
   "M":font=35'b10001_11011_10101_10101_10001_10001_10001;
   "N":font=35'b10001_11001_10101_10011_10001_10001_10001;
   "P":font=35'b11110_10001_10001_11110_10000_10000_10000;
   "S":font=35'b01111_10000_10000_01110_00001_00001_11110;
   "T":font=35'b11111_00100_00100_00100_00100_00100_00100;
   "W":font=35'b10001_10001_10001_10101_10101_10101_01010;
   "Y":font=35'b10001_10001_01010_00100_00100_00100_00100;
   "Z":font=35'b11111_00001_00010_00100_01000_10000_11111;
   ".":font=35'b00000_00000_00000_00000_00000_00100_00100;
   "-":font=35'b00000_00000_00000_11111_00000_00000_00000;
   default:font=0;
  endcase
  case(dy[4:2])
   0:row=font[34:30];1:row=font[29:25];2:row=font[24:20];3:row=font[19:15];
   4:row=font[14:10];5:row=font[9:5];6:row=font[4:0];default:row=0;
  endcase
 end
 assign overlay_active=enabled&&de_i&&ax_i>=16&&ax_i<656&&
  ((ay_i>=16&&ay_i<48)||(ay_i>=56&&ay_i<88)||(ay_i>=96&&ay_i<128));
 assign overlay_ink=overlay_active&&column<5&&row[4-column];
 assign rgb_o=overlay_active?(overlay_ink?24'hffffff:24'h000000):rgb_i;
endmodule
