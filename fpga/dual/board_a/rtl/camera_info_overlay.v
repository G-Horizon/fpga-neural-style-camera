`timescale 1ns/1ps
`default_nettype none
module camera_info_overlay(
    input wire [7:0] fps_bcd,
    input wire geometry_valid,de_i,
    input wire [11:0] x_i,y_i,
    input wire [23:0] rgb_i,
    output wire [23:0] rgb_o,
    output wire active,ink
);
    // 5x7 font, four screen pixels per font pixel, 24px character pitch.
    // Two lines in a 288x88 dark panel, inset 24px from the screen edges.
    wire [11:0] dx=x_i-12'd32,dy=y_i-12'd32;
    wire second=dy>=12'd40;
    wire [11:0] ly=second?dy-12'd40:dy;
    wire [6:0] font_x=dx[8:2];
    wire [4:0] char_index=font_x/6;
    wire [2:0] col=font_x%6,row=ly[4:2];
    reg [7:0] ch;
    reg [34:0] font;
    reg [4:0] font_row;
    always @*begin
        ch=" ";
        if(!second)begin
            case(char_index)
                0:ch="C";1:ch="A";2:ch="M";4:ch="F";5:ch="P";6:ch="S";
                7:ch=":";9:ch=8'h30+{4'd0,fps_bcd[7:4]};10:ch=8'h30+{4'd0,fps_bcd[3:0]};
                default:ch=" ";
            endcase
        end else begin
            case(char_index)
                0:ch="1";1:ch="9";2:ch="2";3:ch="0";5:ch="x";
                7:ch="1";8:ch="0";9:ch="8";10:ch="0";
                default:ch=" ";
            endcase
            if(!geometry_valid&&((char_index<4)||(char_index>=7&&char_index<=10)))ch="-";
        end
        case(ch)
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
            "C":font=35'b01111_10000_10000_10000_10000_10000_01111;
            "F":font=35'b11111_10000_10000_11110_10000_10000_10000;
            "M":font=35'b10001_11011_10101_10101_10001_10001_10001;
            "P":font=35'b11110_10001_10001_11110_10000_10000_10000;
            "S":font=35'b01111_10000_10000_01110_00001_00001_11110;
            "x":font=35'b00000_00000_10001_01010_00100_01010_10001;
            ":":font=35'b00000_00100_00100_00000_00100_00100_00000;
            "-":font=35'b00000_00000_00000_11111_00000_00000_00000;
            default:font=0;
        endcase
        case(row)
            0:font_row=font[34:30];1:font_row=font[29:25];2:font_row=font[24:20];
            3:font_row=font[19:15];4:font_row=font[14:10];5:font_row=font[9:5];
            6:font_row=font[4:0];default:font_row=0;
        endcase
    end
    assign active=de_i&&x_i>=24&&x_i<312&&y_i>=24&&y_i<112;
    assign ink=active&&x_i>=32&&x_i<296&&y_i>=32&&ly<28&&col<5&&font_row[4-col];
    assign rgb_o=active?(ink?24'hffffff:24'h101820):rgb_i;
endmodule
`default_nettype wire
