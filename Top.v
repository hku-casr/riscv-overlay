`timescale 1ns / 1ps

`include "macro_para.v"

module Top(
    input clk,
    input rst,
    input rst_n,
    
    //input [`RV_BIT_NUM-1:0] imem_data_o
    input pc_valid_i,
    input [`RV_BIT_NUM-1:0] pc_start_minus4,   
    input [`RV_BIT_NUM-1:0] imem_wr_addr,
	input [`RV_BIT_NUM-1:0] imem_wr_data,
	input imem_wr_valid,
    	
	input [`RV_BIT_NUM-1:0] dmem_wr_addr,
	input [`RV_BIT_NUM-1:0] dmem_wr_data,
	input dmem_wr_valid
	
   , output [`RV_BIT_NUM-1:0] DtoC_dec_inst,
    output DtoC_exe_br_eq,
    output DtoC_exe_br_lt,
    output DtoC_exe_br_ltu,
    output [`BR_TYPE_BIT_NUM-1:0] DtoC_exe_br_type,
    output DtoC_exe_ctrl_dmem_val
    
    );

    wire [`EXE_PC_SEL-1:0] CtoD_exe_pc_sel;
    wire [`OPI_SEL-1:0] CtoD_sel_dec_op1;
    wire [`OPII_SEL-1:0] CtoD_sel_dec_op2_addr;
    
    wire CtoD_dec_stall;
    wire CtoD_full_stall;
    
    wire [`BR_TYPE_BIT_NUM-1:0] CtoD_br_type;
    
    wire CtoD_if_kill;
    wire CtoD_dec_kill;
   
    wire [`ALU_FUN_BIT_NUM-1:0] CtoD_alu_fun;
    wire [`WB_SEL_BIT_NUM-1:0] CtoD_wb_sel;    
    
    wire CtoD_rf_wen;
    wire CtoD_mem_val;
    
    wire [`MEM_FCN_BIT_NUM-1:0] CtoD_mem_fcn;
    wire [`MEM_TYP_BIT_NUM-1:0] CtoD_mem_typ;   
    wire [`CSR_CMD_BIT_NUM-1:0] CtoD_csr_cmd;
	 
	wire DtoC_exe_rc_done;
	wire CtoD_exe_rc_cmd;
    
    //wire [`RV_BIT_NUM-1:0] DtoC_dec_inst;
    //wire DtoC_exe_br_eq;
    //wire DtoC_exe_br_lt;
    //wire DtoC_exe_br_ltu;
    //wire [`BR_TYPE_BIT_NUM-1:0] DtoC_exe_br_type;
    //wire DtoC_exe_ctrl_dmem_val;
    
	dpath dpath(
        .clk(clk),
        .rst(rst),
        .rst_n(rst_n),
        
        //.imem_data_o(imem_data_o),
        .pc_valid_i(pc_valid_i),
        .pc_start_minus4(pc_start_minus4),
        
        .CtoD_exe_pc_sel(CtoD_exe_pc_sel),
        .CtoD_sel_dec_op1(CtoD_sel_dec_op1),
        .CtoD_sel_dec_op2_addr(CtoD_sel_dec_op2_addr),
        
        .CtoD_dec_stall(CtoD_dec_stall),
        .CtoD_full_stall(CtoD_full_stall),
        
        .CtoD_br_type(CtoD_br_type),
        
        .CtoD_if_kill(CtoD_if_kill),
        .CtoD_dec_kill(CtoD_dec_kill),
       
        .CtoD_alu_fun(CtoD_alu_fun),
        .CtoD_wb_sel(CtoD_wb_sel),    
        
        .CtoD_rf_wen(CtoD_rf_wen),
        .CtoD_mem_val(CtoD_mem_val),
        
        .CtoD_mem_fcn(CtoD_mem_fcn),
        .CtoD_mem_typ(CtoD_mem_typ),   
        .CtoD_csr_cmd(CtoD_csr_cmd),
        
        .DtoC_dec_inst(DtoC_dec_inst),
        .DtoC_exe_br_eq(DtoC_exe_br_eq),
        .DtoC_exe_br_lt(DtoC_exe_br_lt),
        .DtoC_exe_br_ltu(DtoC_exe_br_ltu),
        .DtoC_exe_br_type(DtoC_exe_br_type),
        .DtoC_exe_ctrl_dmem_val(DtoC_exe_ctrl_dmem_val),
		  
		  .DtoC_exe_rc_done(DtoC_exe_rc_done),
		  .CtoD_exe_rc_cmd(CtoD_exe_rc_cmd),
        
			.imem_wr_addr(imem_wr_addr),
			.imem_wr_data(imem_wr_data),
			.imem_wr_valid(imem_wr_valid),

			.dmem_wr_addr(dmem_wr_addr),
			.dmem_wr_data(dmem_wr_data),
			.dmem_wr_valid(dmem_wr_valid)

    );
	
    cpath cpath(
    .clk(clk),
    .rst_n(rst_n),

    .CtoD_exe_pc_sel(CtoD_exe_pc_sel),
    .CtoD_sel_dec_op1(CtoD_sel_dec_op1),
    .CtoD_sel_dec_op2_addr(CtoD_sel_dec_op2_addr),
    
    .CtoD_dec_stall(CtoD_dec_stall),
    .CtoD_full_stall(CtoD_full_stall),
    
    .CtoD_br_type(CtoD_br_type),
    
    .CtoD_if_kill(CtoD_if_kill),
    .CtoD_dec_kill(CtoD_dec_kill),
   
    .CtoD_alu_fun(CtoD_alu_fun),
    .CtoD_wb_sel(CtoD_wb_sel),    
    
    .CtoD_rf_wen(CtoD_rf_wen),
    .CtoD_mem_val(CtoD_mem_val),
    
    .CtoD_mem_fcn(CtoD_mem_fcn),
    .CtoD_mem_typ(CtoD_mem_typ),   
    .CtoD_csr_cmd(CtoD_csr_cmd),
    
    .DtoC_dec_inst(DtoC_dec_inst),
    .DtoC_exe_br_eq(DtoC_exe_br_eq),
    .DtoC_exe_br_lt(DtoC_exe_br_lt),
    .DtoC_exe_br_ltu(DtoC_exe_br_ltu),
    .DtoC_exe_br_type(DtoC_exe_br_type),
    .DtoC_exe_ctrl_dmem_val(DtoC_exe_ctrl_dmem_val),
	 
	.DtoC_exe_rc_done(DtoC_exe_rc_done),
	.CtoD_exe_rc_cmd(CtoD_exe_rc_cmd)
    
   // .dmem_resp_valid(dmem_resp_valid)
    );	

endmodule
