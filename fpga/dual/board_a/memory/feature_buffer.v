module feature_buffer #(
    parameter DATA_WIDTH = 8,

    parameter CIN        = 32,
    parameter IMG_WIDTH  = 256,
    parameter IMG_HEIGHT = 256,

    // 0: VALID window coordinates; 1: reflection padding
    parameter PADDING     = 0,

    // 32 * 256 * 256 = 2^21
    parameter ADDR_WIDTH = 21
)(
    input wire clk,
    input wire rst_n,


    // ========================================================
    // Write Port
    //
    // 用于把上一层 Feature Map 写入 Buffer
    // ========================================================

    input wire wr_en,

    input wire [4:0]  wr_ch,
    input wire [15:0] wr_row,
    input wire [15:0] wr_col,

    input wire signed [DATA_WIDTH-1:0] wr_data,


    // ========================================================
    // Window Request
    //
    // 请求读取某个 Channel 的一个 3x3 Window
    //
    // req_row / req_col = Window 左上角
    // ========================================================

    input wire req_valid,

    output wire req_ready,

    input wire [4:0]          req_ch,
    input wire signed [16:0]  req_row,
    input wire signed [16:0]  req_col,


    // ========================================================
    // Window Output
    //
    // valid / ready 握手
    // ========================================================

    output reg window_valid,

    input wire window_ready,


    output reg signed [DATA_WIDTH-1:0] x0,
    output reg signed [DATA_WIDTH-1:0] x1,
    output reg signed [DATA_WIDTH-1:0] x2,

    output reg signed [DATA_WIDTH-1:0] x3,
    output reg signed [DATA_WIDTH-1:0] x4,
    output reg signed [DATA_WIDTH-1:0] x5,

    output reg signed [DATA_WIDTH-1:0] x6,
    output reg signed [DATA_WIDTH-1:0] x7,
    output reg signed [DATA_WIDTH-1:0] x8
);


// ============================================================
// Memory parameters
// ============================================================

localparam integer IMG_SIZE =
    IMG_WIDTH * IMG_HEIGHT;

localparam integer DEPTH =
    CIN * IMG_SIZE;


// ============================================================
// Feature Memory
//
// 地址布局：
//
// addr = channel * IMG_SIZE
//      + row * IMG_WIDTH
//      + col
//
// 不要给整个 RAM 写 reset。
// ============================================================

reg signed [DATA_WIDTH-1:0] mem [0:DEPTH-1];


// ============================================================
// Write address
// ============================================================

wire [31:0] wr_addr_full;

wire [ADDR_WIDTH-1:0] wr_addr;

assign wr_addr_full =
      wr_ch  * IMG_SIZE
    + wr_row * IMG_WIDTH
    + wr_col;

assign wr_addr =
    wr_addr_full[ADDR_WIDTH-1:0];


// ============================================================
// Memory write
// ============================================================

always @(posedge clk) begin

    if (wr_en) begin

        mem[wr_addr] <= wr_data;

    end

end


// ============================================================
// ============================================================
// Request coordinate registers
//
// PADDING=1 时，窗口左上角可以是 -1，
// 所以 row/col 必须保留有符号坐标，不能直接用无符号地址相加。
// ============================================================

reg [4:0] req_ch_reg;

reg signed [16:0] req_row_reg;

reg signed [16:0] req_col_reg;


// ============================================================
// Reflection coordinate mapping
//
// PyTorch reflection padding 不复制边界像素：
//
//   ... 2 1 | 0 1 2 ... W-2 W-1 | W-2 W-3 ...
//
// 对当前 3x3、PADDING=1 的窗口，边界外只会出现 -1 或 W，
// 因此 -1 -> 1，W -> W-2。
// ============================================================

function [15:0] reflect_coord;

    input signed [16:0] coord;

    input integer       limit;

    integer coord_int;

    begin

        coord_int = coord;

        if (PADDING == 1) begin

            if (coord_int < 0) begin

                coord_int = -coord_int;

            end

            else if (coord_int >= limit) begin

                coord_int = 2 * limit - 2 - coord_int;

            end

        end

        reflect_coord = coord_int;

    end

endfunction


wire [15:0] mapped_row0;
wire [15:0] mapped_row1;
wire [15:0] mapped_row2;

wire [15:0] mapped_col0;
wire [15:0] mapped_col1;
wire [15:0] mapped_col2;

assign mapped_row0 =
    reflect_coord(req_row_reg, IMG_HEIGHT);

assign mapped_row1 =
    reflect_coord(req_row_reg + 17'sd1, IMG_HEIGHT);

assign mapped_row2 =
    reflect_coord(req_row_reg + 17'sd2, IMG_HEIGHT);

assign mapped_col0 =
    reflect_coord(req_col_reg, IMG_WIDTH);

assign mapped_col1 =
    reflect_coord(req_col_reg + 17'sd1, IMG_WIDTH);

assign mapped_col2 =
    reflect_coord(req_col_reg + 17'sd2, IMG_WIDTH);


// ============================================================
// Mapped address for each 3x3 tap
// ============================================================

wire [31:0] tap_addr0_full;
wire [31:0] tap_addr1_full;
wire [31:0] tap_addr2_full;

wire [31:0] tap_addr3_full;
wire [31:0] tap_addr4_full;
wire [31:0] tap_addr5_full;

wire [31:0] tap_addr6_full;
wire [31:0] tap_addr7_full;
wire [31:0] tap_addr8_full;

wire [ADDR_WIDTH-1:0] tap_addr0;
wire [ADDR_WIDTH-1:0] tap_addr1;
wire [ADDR_WIDTH-1:0] tap_addr2;

wire [ADDR_WIDTH-1:0] tap_addr3;
wire [ADDR_WIDTH-1:0] tap_addr4;
wire [ADDR_WIDTH-1:0] tap_addr5;

wire [ADDR_WIDTH-1:0] tap_addr6;
wire [ADDR_WIDTH-1:0] tap_addr7;
wire [ADDR_WIDTH-1:0] tap_addr8;

assign tap_addr0_full =
      req_ch_reg * IMG_SIZE
    + mapped_row0 * IMG_WIDTH
    + mapped_col0;

assign tap_addr1_full =
      req_ch_reg * IMG_SIZE
    + mapped_row0 * IMG_WIDTH
    + mapped_col1;

assign tap_addr2_full =
      req_ch_reg * IMG_SIZE
    + mapped_row0 * IMG_WIDTH
    + mapped_col2;

assign tap_addr3_full =
      req_ch_reg * IMG_SIZE
    + mapped_row1 * IMG_WIDTH
    + mapped_col0;

assign tap_addr4_full =
      req_ch_reg * IMG_SIZE
    + mapped_row1 * IMG_WIDTH
    + mapped_col1;

assign tap_addr5_full =
      req_ch_reg * IMG_SIZE
    + mapped_row1 * IMG_WIDTH
    + mapped_col2;

assign tap_addr6_full =
      req_ch_reg * IMG_SIZE
    + mapped_row2 * IMG_WIDTH
    + mapped_col0;

assign tap_addr7_full =
      req_ch_reg * IMG_SIZE
    + mapped_row2 * IMG_WIDTH
    + mapped_col1;

assign tap_addr8_full =
      req_ch_reg * IMG_SIZE
    + mapped_row2 * IMG_WIDTH
    + mapped_col2;

assign tap_addr0 = tap_addr0_full[ADDR_WIDTH-1:0];
assign tap_addr1 = tap_addr1_full[ADDR_WIDTH-1:0];
assign tap_addr2 = tap_addr2_full[ADDR_WIDTH-1:0];

assign tap_addr3 = tap_addr3_full[ADDR_WIDTH-1:0];
assign tap_addr4 = tap_addr4_full[ADDR_WIDTH-1:0];
assign tap_addr5 = tap_addr5_full[ADDR_WIDTH-1:0];

assign tap_addr6 = tap_addr6_full[ADDR_WIDTH-1:0];
assign tap_addr7 = tap_addr7_full[ADDR_WIDTH-1:0];
assign tap_addr8 = tap_addr8_full[ADDR_WIDTH-1:0];


// ============================================================
// Window Fetch Controller
// ============================================================

reg busy;

// 0 ~ 8
reg [3:0] issue_idx;


// RAM 是同步读取
//
// 所以要记住“上一拍读的是 Window 中哪个位置”
reg [3:0] read_idx_d;

reg read_pending;


// ============================================================
// Feature Buffer 只有完全空闲时才接受新请求
//
// 如果 window_valid=1，必须等消费者取走。
// ============================================================

assign req_ready =
    (!busy) &&
    (!window_valid);


// ============================================================
// Read address generator
//
// 一个 3x3 Window：
//
// 0 1 2
// 3 4 5
// 6 7 8
//
// ============================================================

reg [ADDR_WIDTH-1:0] rd_addr;

wire rd_en;

assign rd_en =
    busy &&
    (issue_idx < 9);


always @(*) begin

    case (issue_idx)

        4'd0:
            rd_addr = tap_addr0;

        4'd1:
            rd_addr = tap_addr1;

        4'd2:
            rd_addr = tap_addr2;


        4'd3:
            rd_addr = tap_addr3;

        4'd4:
            rd_addr = tap_addr4;

        4'd5:
            rd_addr = tap_addr5;


        4'd6:
            rd_addr = tap_addr6;

        4'd7:
            rd_addr = tap_addr7;

        4'd8:
            rd_addr = tap_addr8;


        default:
            rd_addr = tap_addr0;

    endcase

end


// ============================================================
// Synchronous RAM read
//
// Clock N:
// rd_addr 有效
//
// Clock N+1:
// rd_data 有效
// ============================================================

reg signed [DATA_WIDTH-1:0] rd_data;


always @(posedge clk or negedge rst_n) begin

    if (!rst_n) begin

        rd_data <= 0;

    end

    else begin

        if (rd_en) begin

            rd_data <= mem[rd_addr];

        end

    end

end


// ============================================================
// Main Controller
// ============================================================

always @(posedge clk or negedge rst_n) begin

    if (!rst_n) begin

        busy         <= 1'b0;

        issue_idx    <= 4'd0;

        read_idx_d   <= 4'd0;
        read_pending <= 1'b0;

        req_ch_reg   <= 5'd0;

        req_row_reg  <= 17'sd0;

        req_col_reg  <= 17'sd0;

        window_valid <= 1'b0;

        x0 <= 0;
        x1 <= 0;
        x2 <= 0;

        x3 <= 0;
        x4 <= 0;
        x5 <= 0;

        x6 <= 0;
        x7 <= 0;
        x8 <= 0;

    end


    else begin


        // ====================================================
        // Consumer 接收 Window
        // ====================================================

        if (window_valid && window_ready) begin

            window_valid <= 1'b0;

        end


        // ====================================================
        // 同步 RAM Pipeline
        //
        // 当前拍发地址
        // 下一拍才能拿数据
        // ====================================================

        read_pending <= rd_en;


        if (rd_en) begin

            read_idx_d <= issue_idx;

        end


        // ====================================================
        // 接收新的 Window Request
        // ====================================================

        if (req_valid && req_ready) begin

            req_ch_reg <= req_ch;

            req_row_reg <= req_row;

            req_col_reg <= req_col;

            busy <= 1'b1;

            issue_idx <= 4'd0;

            read_pending <= 1'b0;

        end


        // ====================================================
        // Window fetching
        // ====================================================

        else if (busy) begin


            // ------------------------------------------------
            // 每发出一个 RAM 地址
            // issue_idx + 1
            // ------------------------------------------------

            if (rd_en) begin

                issue_idx <=
                    issue_idx + 4'd1;

            end


            // ------------------------------------------------
            // 上一个 RAM 请求的数据现在返回
            // ------------------------------------------------

            if (read_pending) begin

                case (read_idx_d)

                    4'd0:
                        x0 <= rd_data;

                    4'd1:
                        x1 <= rd_data;

                    4'd2:
                        x2 <= rd_data;

                    4'd3:
                        x3 <= rd_data;

                    4'd4:
                        x4 <= rd_data;

                    4'd5:
                        x5 <= rd_data;

                    4'd6:
                        x6 <= rd_data;

                    4'd7:
                        x7 <= rd_data;

                    4'd8:
                        x8 <= rd_data;

                    default: begin
                    end

                endcase


                // =============================================
                // 第九个数读取完成
                // =============================================

                if (read_idx_d == 4'd8) begin

                    busy <= 1'b0;

                    window_valid <= 1'b1;

                    issue_idx <= 4'd0;

                end

            end

        end

    end

end


endmodule
