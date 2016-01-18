`timescale 1ns / 1ps

`include "macro_para.v"

module imem(
	input clka,
	input rsta,
	
	input wea,
	input [`IMEMM_ADDR_BIT_NUM-1:0] addra,
	
	input [`RV_BIT_NUM-1:0] dina,
	output reg [`RV_BIT_NUM-1:0] douta
	
    );
	
	reg [`RV_BIT_NUM-1:0] imemReg [0:`IMEMM_DEPTH-1];
	
	always@(posedge clka) begin

		if (wea) begin
			imemReg[addra] <= dina;
		end else begin
			douta <= imemReg[addra];
		end
	end

endmodule
