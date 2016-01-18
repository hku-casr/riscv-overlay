`timescale 1ns / 1ps

`include "macro_para.v"

module dmem(
	input clka,
	input rsta,
	
	input [`RV_BIT_NUM_DIVIV_NUM-1:0] wea,
	input [`DMEMM_ADDR_BIT_NUM-1:0] addra,
	
	input [`RV_BIT_NUM-1:0] dina,
	output reg [`RV_BIT_NUM-1:0] douta
	
    );
	
	`ifdef XILINX(* ram_style = "block" *) `endif  reg [`RV_BIT_NUM_DIVIV-1:0] dmemReg0 [0:`DMEMM_DEPTH-1];
	`ifdef XILINX(* ram_style = "block" *) `endif  reg [`RV_BIT_NUM_DIVIV-1:0] dmemReg1 [0:`DMEMM_DEPTH-1];
	`ifdef XILINX(* ram_style = "block" *) `endif  reg [`RV_BIT_NUM_DIVIV-1:0] dmemReg2 [0:`DMEMM_DEPTH-1];
	`ifdef XILINX(* ram_style = "block" *) `endif  reg [`RV_BIT_NUM_DIVIV-1:0] dmemReg3 [0:`DMEMM_DEPTH-1];
	
	always@(posedge clka ) begin
		douta[0+: `RV_BIT_NUM_DIVIV] <= dmemReg0[addra];
		douta[`RV_BIT_NUM_DIVIV*1+: `RV_BIT_NUM_DIVIV] <= dmemReg1[addra];
		douta[`RV_BIT_NUM_DIVIV*2+: `RV_BIT_NUM_DIVIV] <= dmemReg2[addra];
		douta[`RV_BIT_NUM_DIVIV*3+: `RV_BIT_NUM_DIVIV] <= dmemReg3[addra];
		
		if (wea[0:0]) begin
			dmemReg0[addra] <= dina[0+: `RV_BIT_NUM_DIVIV];
		end 
		
		if (wea[1:1]) begin
			dmemReg1[addra] <= dina[`RV_BIT_NUM_DIVIV*1+: `RV_BIT_NUM_DIVIV];
		end 
		
		if (wea[2:2]) begin
			dmemReg2[addra] <= dina[`RV_BIT_NUM_DIVIV*2+: `RV_BIT_NUM_DIVIV];
		end
		
		if (wea[3:3]) begin
			dmemReg3[addra] <= dina[`RV_BIT_NUM_DIVIV*3+: `RV_BIT_NUM_DIVIV];
		end
	end

endmodule
