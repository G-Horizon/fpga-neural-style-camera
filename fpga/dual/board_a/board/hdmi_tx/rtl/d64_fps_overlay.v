`timescale 1ns/1ps
module d64_fps_overlay(
    input wire enabled,input wire [7:0] fps_bcd,
    input wire de_i,input wire [11:0] ax_i,ay_i,
    input wire [23:0] rgb_i,output wire [23:0] rgb_o,
    output wire overlay_active,overlay_ink
);
    reg [3:0] glyph;
    reg [34:0] font;
    reg [4:0] row;
    wire [5:0] glyph_x=ax_i[7:2]-6'd4;
    wire [2:0] glyph_row=ay_i[4:2]^3'b100,glyph_col=glyph_x[2:0];
    always @* begin
        case(glyph_x[5:3])
            3'd0:glyph=4'd10;3'd1:glyph=4'd11;3'd2:glyph=4'd12;
            3'd3:glyph=4'd13;3'd4:glyph=fps_bcd[7:4];default:glyph=fps_bcd[3:0];
        endcase
        case(glyph)
            4'd0:font=35'b01110_10001_10011_10101_11001_10001_01110;
            4'd1:font=35'b00100_01100_00100_00100_00100_00100_01110;
            4'd2:font=35'b01110_10001_00001_00010_00100_01000_11111;
            4'd3:font=35'b11110_00001_00001_01110_00001_00001_11110;
            4'd4:font=35'b00010_00110_01010_10010_11111_00010_00010;
            4'd5:font=35'b11111_10000_10000_11110_00001_00001_11110;
            4'd6:font=35'b01110_10000_10000_11110_10001_10001_01110;
            4'd7:font=35'b11111_00001_00010_00100_01000_01000_01000;
            4'd8:font=35'b01110_10001_10001_01110_10001_10001_01110;
            4'd9:font=35'b01110_10001_10001_01111_00001_00001_01110;
            4'd10:font=35'b11111_10000_10000_11110_10000_10000_10000;
            4'd11:font=35'b11110_10001_10001_11110_10000_10000_10000;
            4'd12:font=35'b01111_10000_10000_01110_00001_00001_11110;
            default:font=35'b00000_00100_00100_00000_00100_00100_00000;
        endcase
        case(glyph_row)
            3'd0:row=font[34:30];3'd1:row=font[29:25];3'd2:row=font[24:20];
            3'd3:row=font[19:15];3'd4:row=font[14:10];3'd5:row=font[9:5];
            3'd6:row=font[4:0];default:row=0;
        endcase
    end
    assign overlay_active=enabled && de_i && ax_i>=12'd16 && ax_i<12'd208 && ay_i>=12'd16 && ay_i<12'd48;
    assign overlay_ink=overlay_active && glyph_col<5 && row[4-glyph_col];
    assign rgb_o=overlay_active?(overlay_ink?24'hffffff:24'h000000):rgb_i;
endmodule
