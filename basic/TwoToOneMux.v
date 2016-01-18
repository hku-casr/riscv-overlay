`timescale 1ns / 1ps

`define RV_BIT_NUM_TWO    32
`define MUX_WIDTH_TWO  1
`define MUX_OPTION_TWO  2

`define muxcasetwo(caseNum, num) \
caseNum: q = d[num*RV_BIT_NUM_TWO + RV_BIT_NUM_TWO-1:num*RV_BIT_NUM_TWO];

module TwoToOneMux #(
    parameter integer RV_BIT_NUM_TWO = `RV_BIT_NUM_TWO)
    ( 
        input [`MUX_WIDTH_TWO-1:0] sel,
        input [(RV_BIT_NUM_TWO*`MUX_OPTION_TWO)-1 : 0] d, 
        output reg [RV_BIT_NUM_TWO-1:0]q);
    
    always@(*) begin
        case( sel )
            `muxcasetwo(`MUX_WIDTH_TWO'd0,0)
            `muxcasetwo(`MUX_WIDTH_TWO'd1,1)
            default: q = `RV_BIT_NUM_TWO'd0;
        endcase
    end

endmodule