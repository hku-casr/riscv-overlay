`timescale 1ns / 1ps

`define RV_BIT_NUM_FOUR    32
`define MUX_WIDTH_FOUR  2
`define MUX_OPTION_FOUR 4

`define muxcasefour(caseNum, num) \
caseNum: q = d[num*RV_BIT_NUM_FOUR + RV_BIT_NUM_FOUR-1:num*RV_BIT_NUM_FOUR];

module FourToOneMux #(
    parameter integer RV_BIT_NUM_FOUR = `RV_BIT_NUM_FOUR)
    ( 
        input [`MUX_WIDTH_FOUR-1:0] sel,
        input [(RV_BIT_NUM_FOUR*`MUX_OPTION_FOUR)-1 : 0] d, 
        output reg [RV_BIT_NUM_FOUR-1:0]q);
    
    always@(*) begin
        case( sel )
            `muxcasefour(`MUX_WIDTH_FOUR'd0, 0)
            `muxcasefour(`MUX_WIDTH_FOUR'd1, 1)
            `muxcasefour(`MUX_WIDTH_FOUR'd2, 2)
            `muxcasefour(`MUX_WIDTH_FOUR'd3, 3)
            default: q = `RV_BIT_NUM_FOUR'd0;
        endcase
    end

endmodule