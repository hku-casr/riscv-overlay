`timescale 1ns / 1ps

`include "macro_para.v"


module regfile_altera(
    input clk,
    input rst_n,

    input [`ADDR_BIT_NUM-1:0] rs1_addr,
    input [`ADDR_BIT_NUM-1:0] rs2_addr,
    
    output reg [`RV_BIT_NUM-1:0] rs1_data,
    output reg [`RV_BIT_NUM-1:0] rs2_data,    
    
    input [`ADDR_BIT_NUM-1:0] waddr,
    input [`RV_BIT_NUM-1:0] wdata,
    input  wen
    );
    
    wire [`RV_BIT_NUM-1:0] wdata_real;
    assign wdata_real = (waddr == 0)? 0: wdata;
    
    
    reg [`RV_BIT_NUM-1:0] regfileReg [0:`RV_BIT_NUM-1];

    
    always@(posedge clk) begin
        
        rs1_data <= regfileReg[rs1_addr];
        rs2_data <= regfileReg[rs2_addr];


        if (wen) begin
            regfileReg[waddr] <= (waddr == 0)? 0: wdata;
        end  
   

    end

endmodule

