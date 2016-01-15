`timescale 1ns / 1ps

`define RV_BIT_NUM_THREE    32
`define MUX_WIDTH_THREE  2
`define MUX_OPTION_THREE 3

`define muxcasethree(caseNum, num) \
caseNum: q = d[num*RV_BIT_NUM_THREE + RV_BIT_NUM_THREE-1:num*RV_BIT_NUM_THREE];

module ThreeToOneMux #(
    parameter integer RV_BIT_NUM_THREE = `RV_BIT_NUM_THREE)
    ( 
        input [`MUX_WIDTH_THREE-1:0] sel,
        input [(RV_BIT_NUM_THREE*`MUX_OPTION_THREE)-1 : 0] d, 
        output reg [RV_BIT_NUM_THREE-1:0]q);
    
    always@(*) begin
        case( sel )
            `muxcasethree(`MUX_WIDTH_THREE'd0, 0)
            `muxcasethree(`MUX_WIDTH_THREE'd1, 1)
            `muxcasethree(`MUX_WIDTH_THREE'd2, 2)
            default: q = `RV_BIT_NUM_THREE'd0;
        endcase
    end

endmodule