`timescale 1ns / 1ps

`include "macro_para.v"

module alu(
        input [`RV_BIT_NUM-1:0] op1,
        input [`RV_BIT_NUM-1:0] op2,
        input [`RV_BIT_NUM-1:0] exe_adder,
        input  [`ALU_FUN_BIT_NUM-1:0] ctrl_alu_fun,
        
        output reg [`RV_BIT_NUM-1:0] exe_alu_out
    );
    
    always@(*) begin
        case (ctrl_alu_fun)
            `ALU_ADD:           exe_alu_out = exe_adder;
            `ALU_SUB:           exe_alu_out = op1 - op2;
            `ALU_SLL:           exe_alu_out = op1 << op2[4:0];
            `ALU_SRL:           exe_alu_out = op1 >> op2[4:0];
            `ALU_SRA:           exe_alu_out = $signed(op1)  >>> op2[4:0];
            `ALU_AND:           exe_alu_out = op1 & op2;
            `ALU_OR:            exe_alu_out = op1 | op2;
            `ALU_XOR:           exe_alu_out = op1 ^op2;
            `ALU_SLT:           exe_alu_out = $signed(op1) < $signed(op2);
            `ALU_SLTU:          exe_alu_out = op1 < op2;
            `ALU_COPY_1:       exe_alu_out = op1;
            `ALU_COPY_2:       exe_alu_out = op2;
            default :               exe_alu_out = 0;
        endcase
    end


endmodule
