`timescale 1ns/1ps
// Restricted production contract: each round writes addresses 0..63 exactly
// once in order. Bases are aligned groups of 16. Ready forbids partial rounds.
// EFX_SRL8 Q is ~data[~A], not data[A]. Store inverted bits and select A=4+g.
module d64_trunk_cfg_srl_bank #(
    parameter integer BIAS_W=20,M_W=16,SHIFT_W=5
)(
    input wire clk,rst_n,cfg_we,
    input wire [5:0] cfg_wa,
    input wire [63:0] cfg_wd,
    input wire [5:0] bias_base,rq_base,
    output wire [16*BIAS_W-1:0] bias_o,
    output wire [16*M_W-1:0] m_o,
    output wire [16*SHIFT_W-1:0] shift_o,
    output wire ready,protocol_fault
);
    reg [5:0] expected_address;
    reg loaded,bad;
    wire bad_base=(|bias_base[3:0])||(|rq_base[3:0]);
    wire bad_address=cfg_we&&cfg_wa!=expected_address;
    assign protocol_fault=bad;
    assign ready=rst_n&&loaded&&!bad&&!bad_base&&!cfg_we;
    wire write_ok=rst_n&&cfg_we&&!bad&&!bad_base&&!bad_address;
    wire [BIAS_W-1:0] inverted_bias=~cfg_wd[BIAS_W-1:0];
    wire [M_W-1:0] inverted_m=~cfg_wd[32+:M_W];
    wire [SHIFT_W-1:0] inverted_shift=~cfg_wd[48+:SHIFT_W];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin expected_address<=0;loaded<=0;bad<=0;end
        else begin
            if(bad_base||bad_address)begin bad<=1;loaded<=0;end
            if(write_ok)begin
                expected_address<=cfg_wa==63 ? 0 : expected_address+1'b1;
                loaded<=cfg_wa==63;
            end
        end
    end
    genvar lane,bit_index;
    generate for(lane=0;lane<16;lane=lane+1)begin:g_lane
        wire lane_ce=write_ok&&cfg_wa[3:0]==lane;
        for(bit_index=0;bit_index<BIAS_W;bit_index=bit_index+1)begin:g_bias
            EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
                .D(inverted_bias[bit_index]),.A({1'b1,bias_base[5:4]}),.CLK(clk),.CE(lane_ce),
                .Q(bias_o[lane*BIAS_W+bit_index]),.Q7());
        end
        for(bit_index=0;bit_index<M_W;bit_index=bit_index+1)begin:g_m
            EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
                .D(inverted_m[bit_index]),.A({1'b1,rq_base[5:4]}),.CLK(clk),.CE(lane_ce),
                .Q(m_o[lane*M_W+bit_index]),.Q7());
        end
        for(bit_index=0;bit_index<SHIFT_W;bit_index=bit_index+1)begin:g_shift
            EFX_SRL8 #(.CLK_POLARITY(1'b1),.CE_POLARITY(1'b1),.INIT(8'hff)) u_srl(
                .D(inverted_shift[bit_index]),.A({1'b1,rq_base[5:4]}),.CLK(clk),.CE(lane_ce),
                .Q(shift_o[lane*SHIFT_W+bit_index]),.Q7());
        end
    end endgenerate
    // synthesis translate_off
    initial if(BIAS_W<1||BIAS_W>32||M_W<1||M_W>16||SHIFT_W<1||SHIFT_W>5)
        $fatal(1,"TRUNK_CFG_SRL_FIELD_DOMAIN");
    // synthesis translate_on
endmodule
