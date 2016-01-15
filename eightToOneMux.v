`timescale 1ns / 1ps

`define RV_BIT_NUM_EIGHT    32
`define MUX_WIDTH_EIGHT  3
`define MUX_OPTION_EIGHT  8

`define muxcaseeight(caseNum, num) \
caseNum: q = d[num*RV_BIT_NUM_EIGHT + RV_BIT_NUM_EIGHT-1:num*RV_BIT_NUM_EIGHT];

module EightToOneMux #(
    parameter integer RV_BIT_NUM_EIGHT = `RV_BIT_NUM_EIGHT)
    ( 
        input [`MUX_WIDTH_EIGHT-1:0] sel,
        input [(RV_BIT_NUM_EIGHT*`MUX_OPTION_EIGHT)-1 : 0] d, 
        output reg [RV_BIT_NUM_EIGHT-1:0]q);
    
    always@(*) begin
        case( sel )
            `muxcaseeight(`MUX_WIDTH_EIGHT'd0,0)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd1,1)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd2,2)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd3,3)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd4,4)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd5,5)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd6,6)
            `muxcaseeight(`MUX_WIDTH_EIGHT'd7,7)
            //default: q = `RV_BIT_NUM_EIGHT'd0;
        endcase
    end

endmodule