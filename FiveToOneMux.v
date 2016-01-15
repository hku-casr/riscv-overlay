`timescale 1ns / 1ps

`define RV_BIT_NUM_FIVE    32
`define MUX_WIDTH_FIVE  3
`define MUX_OPTION_FIVE  5

`define muxcasefive(caseNum, num) \
caseNum: q = d[num*RV_BIT_NUM_FIVE + RV_BIT_NUM_FIVE-1:num*RV_BIT_NUM_FIVE];

module FiveToOneMux #(
    parameter integer RV_BIT_NUM_FIVE = `RV_BIT_NUM_FIVE)
    ( 
        input [`MUX_WIDTH_FIVE-1:0] sel,
        input [(RV_BIT_NUM_FIVE*`MUX_OPTION_FIVE)-1 : 0] d, 
        output reg [RV_BIT_NUM_FIVE-1:0]q);
    
    always@(*) begin
        case( sel )
            `muxcasefive(`MUX_WIDTH_FIVE'd0, 0)
            `muxcasefive(`MUX_WIDTH_FIVE'd1,1)
            `muxcasefive(`MUX_WIDTH_FIVE'd2,2)
            `muxcasefive(`MUX_WIDTH_FIVE'd3,3)
            `muxcasefive(`MUX_WIDTH_FIVE'd4,4)
            default: q = `RV_BIT_NUM_FIVE'd0;
        endcase
    end

endmodule