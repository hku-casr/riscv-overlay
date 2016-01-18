`timescale 1ns / 1ps

`include "macro_para.v"

module dpath #(
    parameter integer RV_BIT_NUM = `RV_BIT_NUM,
    parameter integer ADDR_BIT_NUM = `ADDR_BIT_NUM,
    
    parameter integer IMM_I_BIT_NUM = `IMM_I_BIT_NUM,
    
    parameter integer IMM_S_BIT_NUM = `IMM_S_LOW_BIT_NUM+`IMM_S_HIGH_BIT_NUM,
    parameter integer IMM_S_LOW_BIT_NUM = `IMM_S_LOW_BIT_NUM,
    parameter integer IMM_S_HIGH_BIT_NUM = `IMM_S_HIGH_BIT_NUM,
    
    parameter integer IMM_SB_BIT_NUM = `IMM_SB_LOW_BIT_NUM+`IMM_SB_MID_BIT_NUM+2,
    parameter integer IMM_SB_LOW_BIT_NUM = `IMM_SB_LOW_BIT_NUM,
    parameter integer IMM_SB_MID_BIT_NUM = `IMM_SB_MID_BIT_NUM,
    
    parameter integer IMM_U_BIT_NUM = `IMM_U_BIT_NUM,
    
    parameter integer IMM_UJ_BIT_NUM = `IMM_UJ_LOW_BIT_NUM + `IMM_UJ_MID_BIT_NUM+2,
    parameter integer IMM_UJ_LOW_BIT_NUM = `IMM_UJ_LOW_BIT_NUM,
    parameter integer IMM_UJ_MID_BIT_NUM = `IMM_UJ_MID_BIT_NUM)
    (
    input clk,
    input rst,
    input rst_n,
    
    //input [RV_BIT_NUM-1:0] imem_data_o,
    input pc_valid_i,
    input [`RV_BIT_NUM-1:0] pc_start_minus4,
    
    input [`EXE_PC_SEL-1:0] CtoD_exe_pc_sel,
    input [`OPI_SEL-1:0] CtoD_sel_dec_op1,
    input [`OPII_SEL-1:0] CtoD_sel_dec_op2_addr,
    
    input CtoD_dec_stall,
    input CtoD_full_stall,
    
    input [`BR_TYPE_BIT_NUM-1:0] CtoD_br_type,
    
    input CtoD_if_kill,
    input CtoD_dec_kill,
   
    input [`ALU_FUN_BIT_NUM-1:0] CtoD_alu_fun,
    input [`WB_SEL_BIT_NUM-1:0] CtoD_wb_sel,    
    
    input CtoD_rf_wen,
    input CtoD_mem_val,
    
    input [`MEM_FCN_BIT_NUM-1:0] CtoD_mem_fcn,
    input [`MEM_TYP_BIT_NUM-1:0] CtoD_mem_typ,   
    input [`CSR_CMD_BIT_NUM-1:0] CtoD_csr_cmd,
    
    output [`RV_BIT_NUM-1:0] DtoC_dec_inst,
    output DtoC_exe_br_eq,
    output DtoC_exe_br_lt,
    output DtoC_exe_br_ltu,
    output [`BR_TYPE_BIT_NUM-1:0] DtoC_exe_br_type,
    output DtoC_exe_ctrl_dmem_val,
    
	//input [RV_BIT_NUM-1:0] rf_rs1_data,
	
	//input [RV_BIT_NUM-1:0] rf_rs2_data
	input [`RV_BIT_NUM-1:0] imem_wr_addr,
	input [`RV_BIT_NUM-1:0] imem_wr_data,
	input imem_wr_valid,
	
	input [`RV_BIT_NUM-1:0] dmem_wr_addr,
	input [`RV_BIT_NUM-1:0] dmem_wr_data,
	input dmem_wr_valid
    
    );
    //reg in if
    reg [RV_BIT_NUM-1:0] if_reg_pc;
    
    //reg in dec
    reg [RV_BIT_NUM-1:0] dec_reg_inst;
    reg [RV_BIT_NUM-1:0] dec_reg_pc;
    
    //reg in exe
    reg [RV_BIT_NUM-1:0] exe_reg_inst;
    reg [RV_BIT_NUM-1:0] exe_reg_pc;
    
    reg [ADDR_BIT_NUM-1:0] exe_reg_wbaddr;
    reg exe_reg_ctrl_rf_wen;
    
    reg [RV_BIT_NUM-1:0] exe_reg_op1_data;
    reg [RV_BIT_NUM-1:0] exe_reg_op2_data;
    reg [RV_BIT_NUM-1:0] exe_reg_rs2_data;
    
    reg [`ALU_FUN_BIT_NUM-1:0] exe_reg_ctrl_alu_fun;
    reg [`WB_SEL_BIT_NUM-1:0] exe_reg_ctrl_wb_sel;
    
    reg exe_reg_ctrl_mem_val;
    reg [`MEM_FCN_BIT_NUM-1:0] exe_reg_ctrl_mem_fcn;
    reg [`MEM_TYP_BIT_NUM-1:0] exe_reg_ctrl_mem_typ;
    
    reg [`CSR_CMD_BIT_NUM-1:0]  exe_reg_ctrl_csr_cmd;
    reg [`BR_TYPE_BIT_NUM-1:0] exe_reg_ctrl_br_type;
    
    reg [RV_BIT_NUM-1:0] exe_reg_mem_addr;
    
    //reg in wb
    reg [ADDR_BIT_NUM-1:0] wb_reg_wbaddr;
    reg [RV_BIT_NUM-1:0] wb_reg_wbdata;
    reg wb_reg_ctrl_rf_wen;
    
    //reg [ADDR_BIT_NUM-1:0] exe_end_wbaddr;
    //reg [RV_BIT_NUM-1:0] exe_end_wbdata;
    reg exe_end_ctrl_rf_wen;  
    
    //if stage
    wire [RV_BIT_NUM-1:0] if_pc_plus4;
    
    wire [RV_BIT_NUM-1:0] if_pc_next;
    wire [RV_BIT_NUM-1:0] exe_brjmp_target;
    wire [RV_BIT_NUM-1:0] exe_jump_reg_target;
    
     //if regs
    always @(posedge clk) begin
        if (rst_n == 0) begin
            if_reg_pc <= pc_start_minus4;//`RV_BIT_NUM'hFFFFFFFC;
        end else begin
            if (!CtoD_dec_stall && !CtoD_full_stall && pc_valid_i == 1) begin
                if_reg_pc <= if_pc_next;
            end
        end
    end   
    
    assign if_pc_plus4 = if_reg_pc + `RV_BIT_NUM'd4;
    
    ThreeToOneMux Mux3_if_pc (
        .sel(CtoD_exe_pc_sel),
        .d({exe_jump_reg_target, exe_brjmp_target, if_pc_plus4}),
        .q(if_pc_next)
    );
    
    reg [RV_BIT_NUM-1:0] imem_addr;
    reg [RV_BIT_NUM-1:0] if_inst;
    wire [RV_BIT_NUM-1:0] imem_data_o;
    
    //assign imem_addr = if_pc_next;
    
    
    //assign imem_addr = ((imem_wr_valid == 1'b1) ? imem_wr_addr : if_pc_next);
    
    always@(*) begin
        if (!CtoD_dec_stall && !CtoD_full_stall) begin
            imem_addr = ((imem_wr_valid == 1'b1) ? imem_wr_addr : if_pc_next);
        end else begin
			imem_addr = if_reg_pc;
		end
    end
    
    imem imem(
        .clka(clk), // input clka
        .rsta(rst), // input rsta
        
        .addra(imem_addr[`IMEMM_ADDR_BIT_NUM-1+2:2]), 
        .douta(imem_data_o), // output [31 : 0] douta
                
        .wea(imem_wr_valid), // input [0 : 0] wea
        .dina(imem_wr_data) // input [31 : 0] dina        
    );
    
    
    //if-dec regs
    always @(posedge clk) begin
        if (rst_n == 0) begin
            dec_reg_inst <= 0;
            dec_reg_pc <= 0;
        end else begin
            if (!CtoD_dec_stall && !CtoD_full_stall) begin
                if (CtoD_if_kill) begin
                    dec_reg_inst <= 0;
                end else begin
                    dec_reg_inst <= imem_data_o;
                end
                dec_reg_pc <= if_reg_pc;
            end
        end
    end
    
    always @(*) begin
        
        if (!CtoD_dec_stall && !CtoD_full_stall) begin
            if (CtoD_if_kill) begin
                if_inst = 0;
            end else begin
                if_inst = imem_data_o;
            end
        end else begin
			if_inst = dec_reg_inst;
		end
    end   
     
    //decode stage
    wire [ADDR_BIT_NUM-1:0] if_rs1_addr;
    wire [ADDR_BIT_NUM-1:0] if_rs2_addr;
    
    assign if_rs1_addr = if_inst[15 +: ADDR_BIT_NUM];
    assign if_rs2_addr = if_inst[20 +: ADDR_BIT_NUM];
    
    wire [ADDR_BIT_NUM-1:0] dec_rs1_addr;
    wire [ADDR_BIT_NUM-1:0] dec_rs2_addr;
    wire [ADDR_BIT_NUM-1:0] dec_wbaddr;
    
    assign dec_rs1_addr = dec_reg_inst[15 +: ADDR_BIT_NUM];
    assign dec_rs2_addr = dec_reg_inst[20 +: ADDR_BIT_NUM];
    assign dec_wbaddr = dec_reg_inst[7 +: ADDR_BIT_NUM];
    
    
    wire [IMM_I_BIT_NUM-1:0] imm_itype;
    wire [IMM_S_BIT_NUM-1:0] imm_stype;
    wire [IMM_SB_BIT_NUM-1:0] imm_sbtype;
    wire [IMM_U_BIT_NUM-1:0] imm_utype;
    wire [IMM_UJ_BIT_NUM-1:0] imm_ujtype;
    wire [RV_BIT_NUM-1:0] imm_z;
    
    assign imm_itype = dec_reg_inst[20 +: IMM_I_BIT_NUM];
    assign imm_stype = {dec_reg_inst[25 +: IMM_S_HIGH_BIT_NUM], dec_reg_inst[7 +: IMM_S_LOW_BIT_NUM]};
    assign imm_sbtype = {dec_reg_inst[31], dec_reg_inst[7], 
                dec_reg_inst[25 +: IMM_SB_MID_BIT_NUM], dec_reg_inst[8 +: IMM_SB_LOW_BIT_NUM]};
    assign imm_utype = dec_reg_inst[12 +: IMM_U_BIT_NUM];
    assign imm_ujtype = {dec_reg_inst[31], dec_reg_inst[12 +: IMM_UJ_MID_BIT_NUM],
                                dec_reg_inst[20], dec_reg_inst[21 +: IMM_UJ_LOW_BIT_NUM]};
    assign imm_z = {27'b0, dec_reg_inst[19:15]};

    wire [RV_BIT_NUM-1:0] imm_itype_sext;
    wire [RV_BIT_NUM-1:0] imm_stype_sext;
    wire [RV_BIT_NUM-1:0] imm_sbtype_sext;
    wire [RV_BIT_NUM-1:0] imm_utype_sext;
    wire [RV_BIT_NUM-1:0] imm_ujtype_sext;
    
    assign imm_itype_sext = {{20{imm_itype[IMM_I_BIT_NUM-1]}}, imm_itype};
    assign imm_stype_sext = {{20{imm_stype[IMM_S_BIT_NUM-1]}}, imm_stype};
    assign imm_sbtype_sext = {{19{imm_sbtype[IMM_SB_BIT_NUM-1]}}, imm_sbtype, 1'b0};
    assign imm_utype_sext = {imm_utype, 12'b0};
    assign imm_ujtype_sext = {{11{imm_ujtype[IMM_UJ_BIT_NUM-1]}}, imm_ujtype, 1'b0};
    
    wire [RV_BIT_NUM-1:0] rf_rs1_data;
    wire [RV_BIT_NUM-1:0] rf_rs2_data;
    
    wire [RV_BIT_NUM-1:0] exe_wbdata;
    
	 `ifdef XILINX
		regfile_xilinx regfile(
	 `else
		regfile_altera regfile(
	 `endif
        .clk(clk),
        .rst_n(rst_n),

        .rs1_addr(if_rs1_addr),
        .rs2_addr(if_rs2_addr),
    
        .rs1_data(rf_rs1_data),
        .rs2_data(rf_rs2_data),
    
        .waddr(exe_reg_wbaddr),
        .wdata(exe_wbdata),
        .wen(exe_end_ctrl_rf_wen)
    );
    
    wire [RV_BIT_NUM-1:0] imm_addr_offst;
    
    TwoToOneMux Mux2_imm (
        .sel(CtoD_sel_dec_op2_addr[`OPII_SEL-3:0]),
        .d({imm_stype_sext, imm_itype_sext}),
        .q(imm_addr_offst)
    ); 

    
    wire [RV_BIT_NUM-1:0] exe_alu_out;
    
    wire [RV_BIT_NUM-1:0] dec_op1_data;
    wire [RV_BIT_NUM-1:0] dec_op2_data;
    wire [RV_BIT_NUM-1:0] dec_rs2_data;
    
    wire [RV_BIT_NUM-1:0] dec_mem_addr;

    //forwarding engine with data selection
    reg [`OPI_SEL_PLUS-1:0] op1_sel;
    always@(*) begin
        if (CtoD_sel_dec_op1 == `OPI_IMZ) begin
            op1_sel = `OPI_MUX_IMZ;
        end else if (CtoD_sel_dec_op1 == `OPI_PC ) begin
            op1_sel = `OPI_MUX_PC;
        end else if ((exe_reg_wbaddr == dec_rs1_addr && dec_rs1_addr != `ADDR_BIT_NUM'd0) && exe_reg_ctrl_rf_wen) begin
            op1_sel = `OPI_MUX_EXEFWD;
        end else if ((wb_reg_wbaddr == dec_rs1_addr && dec_rs1_addr != `ADDR_BIT_NUM'd0) && wb_reg_ctrl_rf_wen) begin
            op1_sel = `OPI_MUX_WBFWD;
        //end else if (CtoD_sel_dec_op1 == `OPI_DEFAULT) begin
            //op1_sel = `OPI_MUX_DEFAULT;
        end else begin
            op1_sel = `OPI_MUX_DEFAULT;	
		end
    end
    
    reg [`OPII_SEL-1:0] op2_sel;
    always@(*) begin
        if (CtoD_sel_dec_op2_addr == `OPII_IMM_ITYPE) begin
            op2_sel = `OPII_MUX_IMM_ITYPE;
        end else if (CtoD_sel_dec_op2_addr == `OPII_IMM_STYPE) begin
            op2_sel = `OPI_MUXI_IMM_STYPE;
        end else if (CtoD_sel_dec_op2_addr == `OPII_IMM_SBTYPE) begin
            op2_sel = `OPII_MUX_IMM_SBTYPE;
        end else if (CtoD_sel_dec_op2_addr == `OPII_IMM_UTYPE) begin
            op2_sel = `OPII_MUX_IMM_UTYPE;
        end else if (CtoD_sel_dec_op2_addr == `OPII_IMM_UJTYPE) begin
            op2_sel = `OPII_MUX_IMM_UJTYPE;
        end else if ((exe_reg_wbaddr == dec_rs2_addr && dec_rs2_addr != `ADDR_BIT_NUM'd0) && exe_reg_ctrl_rf_wen) begin
            op2_sel = `OPII_MUX_MUX_EXEFWD;
        end else if ((wb_reg_wbaddr == dec_rs2_addr && dec_rs2_addr != `ADDR_BIT_NUM'd0) && wb_reg_ctrl_rf_wen) begin
            op2_sel = `OPII_MUX_MUX_WBFWD; 
        //end else if (CtoD_sel_dec_op2_addr == `OPII_DEFAULT) begin
            //op2_sel = `OPII_MUX_DEFAULT;           
        end else begin
			op2_sel = `OPII_MUX_DEFAULT;  
		end
    end

    reg [`RSII_SEL-1:0] rs2_sel;
    always@(*) begin
        if ((exe_reg_wbaddr == dec_rs2_addr && dec_rs2_addr != `ADDR_BIT_NUM'd0) && exe_reg_ctrl_rf_wen) begin
            rs2_sel = `RS_MUX_EXEFWD;
        end else if ((wb_reg_wbaddr == dec_rs2_addr && dec_rs2_addr != `ADDR_BIT_NUM'd0) && wb_reg_ctrl_rf_wen) begin
            rs2_sel = `RS_MUX_WBFWD; 
        end else begin
            rs2_sel = `RS_MUX_DEFAULT; 
        end
    end
    
    reg [`ADDR_SEL-1:0] addr_sel;
    always@(*) begin
        if ((exe_reg_wbaddr == dec_rs1_addr && dec_rs1_addr != `ADDR_BIT_NUM'd0) && exe_reg_ctrl_rf_wen) begin
            addr_sel = `ADDR_MUX_EXEFWD;
        end else if ((wb_reg_wbaddr == dec_rs1_addr && dec_rs1_addr != `ADDR_BIT_NUM'd0) && wb_reg_ctrl_rf_wen) begin
            addr_sel = `ADDR_MUX_WBFWD; 
        end else begin
            addr_sel = `ADDR_MUX_DEFAULT; 
        end
    end    
    

 
    FiveToOneMux Mux5_dec_op1 (
        .sel(op1_sel),
        .d({rf_rs1_data, wb_reg_wbdata, exe_wbdata/*exe_alu_out*/, dec_reg_pc, imm_z}),
        .q(dec_op1_data)
    );
    
    
    EightToOneMux Mux8_dec_op2 (
        .sel(op2_sel),
        .d({rf_rs2_data, wb_reg_wbdata, exe_wbdata/*exe_alu_out*/, imm_ujtype_sext, imm_utype_sext, imm_sbtype_sext, imm_stype_sext, imm_itype_sext}),
        .q(dec_op2_data)
    );
    
    ThreeToOneMux Mux3_dec_rs2 (
        .sel(rs2_sel),
        .d({rf_rs2_data, wb_reg_wbdata, exe_wbdata/*exe_alu_out*/}),
        .q(dec_rs2_data)
    );
    
    ThreeToOneMux Mux3_dec_addr (
        .sel(addr_sel),
        .d({(imm_addr_offst + rf_rs1_data), (imm_addr_offst + wb_reg_wbdata), (imm_addr_offst + exe_wbdata/*exe_alu_out*/)}),
        .q(dec_mem_addr)
    );
    
    //dec-exe regs
    always @(posedge clk) begin
        if (rst_n == 0) begin
            exe_reg_pc <= 0;
            
            exe_reg_op1_data <= 0;
            exe_reg_op2_data <= 0;
            exe_reg_rs2_data <= 0;
            
            exe_reg_ctrl_alu_fun <= 0;
            exe_reg_ctrl_wb_sel <= 0;
            
            exe_reg_mem_addr <= 0;
            
            exe_reg_ctrl_mem_val <= 0;
            
        end else begin
             if (!CtoD_dec_stall && !CtoD_full_stall) begin
                exe_reg_pc <= dec_reg_pc;
                
                exe_reg_op1_data <= dec_op1_data;
                exe_reg_op2_data <= dec_op2_data;
                exe_reg_rs2_data <= dec_rs2_data;
                
                exe_reg_ctrl_alu_fun <= CtoD_alu_fun;
                exe_reg_ctrl_wb_sel <= CtoD_wb_sel;
                
                exe_reg_mem_addr <= dec_mem_addr;
            
            
                if (CtoD_dec_kill) begin
                    exe_reg_inst <= 0;
                    exe_reg_wbaddr <= 0;
                    
                    exe_reg_ctrl_rf_wen <= 0;
                    exe_reg_ctrl_mem_val <= 0;
                    
                    exe_reg_ctrl_mem_fcn <= `M_X;
                    exe_reg_ctrl_csr_cmd <= `CSR_N;
                    
                    exe_reg_ctrl_br_type <= `BR_N;
                
                end else begin
                    exe_reg_inst <= dec_reg_inst;
                    exe_reg_wbaddr <= dec_wbaddr;
                    
                    exe_reg_ctrl_rf_wen <= CtoD_rf_wen;
                    exe_reg_ctrl_mem_val <= CtoD_mem_val;
                    
                    exe_reg_ctrl_mem_fcn <= CtoD_mem_fcn;
                    exe_reg_ctrl_mem_typ <= CtoD_mem_typ;
                    
                    exe_reg_ctrl_csr_cmd <= CtoD_csr_cmd;
                    exe_reg_ctrl_br_type <= CtoD_br_type;
                end
                
            end else if (CtoD_dec_stall && !CtoD_full_stall) begin
                exe_reg_inst <= 0;
                exe_reg_wbaddr <= 0;
                
                exe_reg_ctrl_rf_wen <= 0;
                exe_reg_ctrl_mem_val <= 0;
                
                exe_reg_ctrl_mem_fcn <= `M_X;
                exe_reg_ctrl_csr_cmd <= `CSR_N;
                
                exe_reg_ctrl_br_type <= `BR_N;
                
            end
        end
    end
        
    //exe stage
    //exe-1st part: ALU
    wire [RV_BIT_NUM-1:0] exe_adder_out;
    assign exe_adder_out = exe_reg_op1_data + exe_reg_op2_data;
    
    wire [RV_BIT_NUM-1:0] exe_pc_plus4;
    
    alu alu_mod(
        .op1(exe_reg_op1_data),
        .op2(exe_reg_op2_data),
        .exe_adder(exe_adder_out),
        .ctrl_alu_fun(exe_reg_ctrl_alu_fun),
        .exe_alu_out(exe_alu_out)
    );
    
    assign exe_brjmp_target = exe_reg_pc + exe_reg_op2_data;
    assign exe_jump_reg_target = exe_adder_out;
    assign exe_pc_plus4 = exe_reg_pc + `RV_BIT_NUM'd4;
    
     //exe-2nd part: CSR
     
     //exe-3rd part: mem
    reg dec_end_ctrl_mem_val; 
    reg [RV_BIT_NUM-1:0] dec_end_mem_addr;

    reg [`MEM_FCN_BIT_NUM-1:0] dec_end_ctrl_mem_fcn;
    reg [`MEM_TYP_BIT_NUM-1:0] dec_end_ctrl_mem_typ;
    
    reg [RV_BIT_NUM-1:0] dec_end_dec_rs2_data;
     
    always@(*) begin
        if (!CtoD_dec_stall && !CtoD_full_stall) begin
        
            dec_end_mem_addr = dec_mem_addr;
            
            if (CtoD_dec_kill) begin
                dec_end_ctrl_mem_val = 0;
                dec_end_mem_addr = 0;
                
                dec_end_ctrl_mem_fcn = `M_X;
				dec_end_ctrl_mem_typ = exe_reg_ctrl_mem_typ;
				dec_end_dec_rs2_data = exe_reg_rs2_data;
                
            end else begin
                dec_end_ctrl_mem_val = CtoD_mem_val;
                
                dec_end_ctrl_mem_fcn = CtoD_mem_fcn;
                dec_end_ctrl_mem_typ = CtoD_mem_typ;
                
                dec_end_dec_rs2_data = dec_rs2_data;
                
            end       
        end else if (CtoD_dec_stall && !CtoD_full_stall) begin
            dec_end_ctrl_mem_val = 0;
            dec_end_mem_addr = 0;
            
            dec_end_ctrl_mem_fcn = `M_X;
			dec_end_ctrl_mem_typ = exe_reg_ctrl_mem_typ;
			dec_end_dec_rs2_data = exe_reg_rs2_data;
			
        end else begin
			dec_end_mem_addr = exe_reg_mem_addr;
			dec_end_ctrl_mem_val = CtoD_mem_val;
			dec_end_ctrl_mem_fcn = exe_reg_ctrl_mem_fcn;
			dec_end_ctrl_mem_typ = exe_reg_ctrl_mem_typ;
			dec_end_dec_rs2_data = exe_reg_rs2_data;
		end
    end
     
     
     wire dmem_valid;
     wire [RV_BIT_NUM-1:0] dmem_addr;
     wire [`MEM_FCN_BIT_NUM-1:0] dmem_fcn;
     wire [`MEM_TYP_BIT_NUM-1:0] dmem_typ;
     reg [RV_BIT_NUM-1:0] dmem_data_i;
     wire [RV_BIT_NUM-1:0] dmem_data_o;
     //assign dmem_data_o=32'hdeadbeef;
     reg [RV_BIT_NUM-1:0] exe_mem_data_o;
     
     assign dmem_valid = dec_end_ctrl_mem_val;
     assign dmem_addr = dec_end_mem_addr;
     assign dmem_fcn = dec_end_ctrl_mem_fcn;
     assign dmem_typ = dec_end_ctrl_mem_typ;
     //assign dmem_data = exe_reg_rs2_data;

    reg [`RV_BIT_NUM_DIVIV_NUM-1:0] dmem_keep;

    always@(*) begin
		dmem_keep = 0;
		//dmem_data_i = exe_reg_rs2_data;
        case (dmem_typ)
            `MT_W: begin
                dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b1111;
                dmem_data_i = dec_end_dec_rs2_data;
                //exe_mem_data_o = dmem_data_o;
            end
            
            `MT_B: begin
                if (dmem_addr[1:0] == 2'b00)  begin 
                    dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b0001;
                    //dmem_data_i[0 +: `RV_BIT_NUM_DIVIV] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV];
					`ifdef LEG_BOARD
						dmem_data_i = {24'd0, dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV]};
					`else
						dmem_data_i[0 +: `RV_BIT_NUM_DIVIV] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV];
					`endif                     
                    //exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV-1]}}, dmem_data_o[0 +: `RV_BIT_NUM_DIVIV]};
                    
                end if (dmem_addr[1:0] == 2'b01)  begin
                    dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b0010;
					`ifdef LEG_BOARD
						dmem_data_i = {16'd0, dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV], 8'd0};
					`else
						dmem_data_i[`RV_BIT_NUM_DIVIV +: `RV_BIT_NUM_DIVIV] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV];
					`endif                        
                    //exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV*2-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV +: `RV_BIT_NUM_DIVIV]};
                    
                end if (dmem_addr[1:0] == 2'b10) begin
                    dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b0100;
					`ifdef LEG_BOARD
						dmem_data_i = {8'd0, dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV], 16'd0};
					`else
						dmem_data_i[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV];
					`endif                    
                    //exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV*3-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV]};
                    
                end if (dmem_addr[1:0] == 2'b11)  begin
                    dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b1000;
					`ifdef LEG_BOARD
						dmem_data_i = {dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV], 24'd0};
					`else
						dmem_data_i[`RV_BIT_NUM_DIVIV*3 +: `RV_BIT_NUM_DIVIV] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV];
					`endif
                   
                    
                    //exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV*4-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV*3 +: `RV_BIT_NUM_DIVIV]};
                end
            end
            
            
            `MT_H: begin
                if (dmem_addr[1] == 1'b0)  begin 
                    dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b0011;
					`ifdef LEG_BOARD
						dmem_data_i = {16'd0, dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV*2]};
					`else
						dmem_data_i[0 +: `RV_BIT_NUM_DIVIV*2] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV*2];
					`endif
                    //exe_mem_data_o = {{16{dmem_data_o[`RV_BIT_NUM_DIVIV*2-1]}}, dmem_data_o[0 +: `RV_BIT_NUM_DIVIV*2]};
                    
                end if (dmem_addr[1] == 1'b1)  begin
                    dmem_keep = `RV_BIT_NUM_DIVIV_NUM'b1100;
                    //dmem_data_i[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV*2] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV*2];
					`ifdef LEG_BOARD
						dmem_data_i = {dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV*2], 16'd0};
					`else
						dmem_data_i[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV*2] = dec_end_dec_rs2_data[0 +: `RV_BIT_NUM_DIVIV*2];
					`endif
                    //exe_mem_data_o = {{16{dmem_data_o[`RV_BIT_NUM_DIVIV*4-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV*2]};
                    
                end
            end
            
            
            default: begin
                dmem_keep = 0;
                dmem_data_i = dec_end_dec_rs2_data;
                //exe_mem_data_o = 0;
            end
        endcase
    end


    always@(*) begin
		exe_mem_data_o = dmem_data_o;
        case (exe_reg_ctrl_mem_typ)
            `MT_W: begin
                exe_mem_data_o = dmem_data_o;
            end
            
            `MT_B: begin
                if (exe_reg_mem_addr[1:0] == 2'b00)  begin                 
                    exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV-1]}}, dmem_data_o[0 +: `RV_BIT_NUM_DIVIV]};
                    
                end if (exe_reg_mem_addr[1:0] == 2'b01)  begin                    
                    exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV*2-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV +: `RV_BIT_NUM_DIVIV]};
                    
                end if (exe_reg_mem_addr[1:0] == 2'b10) begin
                    exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV*3-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV]};
                    
                end if (exe_reg_mem_addr[1:0] == 2'b11)  begin   
                    exe_mem_data_o = {{24{dmem_data_o[`RV_BIT_NUM_DIVIV*4-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV*3 +: `RV_BIT_NUM_DIVIV]};
                end
            end
            
            `MT_BU: begin
                if (exe_reg_mem_addr[1:0] == 2'b00)  begin                     
                    exe_mem_data_o = {24'd0, dmem_data_o[0 +: `RV_BIT_NUM_DIVIV]};                       
                end if (exe_reg_mem_addr[1:0] == 2'b01)  begin                        
                    exe_mem_data_o = {24'd0, dmem_data_o[`RV_BIT_NUM_DIVIV +: `RV_BIT_NUM_DIVIV]};                        
                end if (exe_reg_mem_addr[1:0] == 2'b10) begin                        
                    exe_mem_data_o = {24'd0, dmem_data_o[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV]};                       
                end if (exe_reg_mem_addr[1:0] == 2'b11)  begin    
                    exe_mem_data_o = {24'd0, dmem_data_o[`RV_BIT_NUM_DIVIV*3 +: `RV_BIT_NUM_DIVIV]};
                end 
            end
            
            `MT_H: begin
                if (exe_reg_mem_addr[1] == 1'b0)  begin                  
                    exe_mem_data_o = {{16{dmem_data_o[`RV_BIT_NUM_DIVIV*2-1]}}, dmem_data_o[0 +: `RV_BIT_NUM_DIVIV*2]};
                    
                end if (exe_reg_mem_addr[1] == 1'b1)  begin                   
                    exe_mem_data_o = {{16{dmem_data_o[`RV_BIT_NUM_DIVIV*4-1]}}, dmem_data_o[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV*2]};
                    
                end
            end
            
            `MT_HU: begin
                if (exe_reg_mem_addr[1] == 1'b0)  begin 
                    exe_mem_data_o = {16'd0, dmem_data_o[0 +: `RV_BIT_NUM_DIVIV*2]};  
                end if (exe_reg_mem_addr[1] == 1'b1)  begin 
                    exe_mem_data_o = {16'd0, dmem_data_o[`RV_BIT_NUM_DIVIV*2 +: `RV_BIT_NUM_DIVIV*2]};  
                end
            end
            
            default: begin
                exe_mem_data_o = 0;
            end
        endcase
    end

	wire [`RV_BIT_NUM-1:0] dmem_data_tb;
	wire [`RV_BIT_NUM-1:0] dmem_addr_tb;
	wire [3:0] fake_dmem_keep_tb;
	wire [3:0] fake_dmem_keep;
	
	assign dmem_data_tb = (dmem_wr_valid == 1)? dmem_wr_data: dmem_data_i;
	assign dmem_addr_tb = (dmem_wr_valid == 1)? dmem_wr_addr: dmem_addr;
	assign fake_dmem_keep_tb = (dmem_wr_valid == 1)? 4'b1111: fake_dmem_keep;
    
    
    assign fake_dmem_keep = ((dmem_valid == 1 && dmem_fcn == `M_XWR)? dmem_keep : 4'b0);
    
    //fake dmem    
    dmem dmem (
        .clka(clk), // input clka
        .rsta(rst), // input rsta
 
        .wea(fake_dmem_keep_tb), // input [3 : 0] wea
        .addra(dmem_addr_tb[`DMEMM_ADDR_BIT_NUM-1+2:2]), // input [16 : 0] addra
        
        .dina(dmem_data_tb), // input [31 : 0] dina
 
        //.wea(fake_dmem_keep), // input [3 : 0] wea
        //.addra(dmem_addr[`DMEMM_ADDR_BIT_NUM-1+2:2]), // input [11 : 0] addra
        
        //.dina(dmem_data_i), // input [31 : 0] dina
        .douta(dmem_data_o) // output [31 : 0] douta
    );
    
 
 
    FourToOneMux Mux4_wb (
        .sel(exe_reg_ctrl_wb_sel),
        .d({32'd0, exe_mem_data_o, exe_pc_plus4, exe_alu_out}),
        .q(exe_wbdata)
    );
    //exe-wb stage
    always@(posedge clk) begin
        if (rst_n == 0) begin
            wb_reg_wbaddr <= 0;
            wb_reg_wbdata <= 0;
            wb_reg_ctrl_rf_wen <= 0;
            
        end else begin
             if (!CtoD_full_stall) begin
                wb_reg_wbaddr <= exe_reg_wbaddr;
                wb_reg_wbdata  <= exe_wbdata;
                wb_reg_ctrl_rf_wen <= exe_reg_ctrl_rf_wen;
                
            end else begin
                wb_reg_ctrl_rf_wen <= 0;
            end       
        end

    end
    
    always@(*) begin
            
        if (!CtoD_full_stall) begin
            //exe_end_wbaddr = exe_reg_wbaddr;
            exe_end_ctrl_rf_wen = exe_reg_ctrl_rf_wen;  
            //exe_end_wbdata = exe_wbdata;
        end else begin
            exe_end_ctrl_rf_wen = 0;
        end       

    end

    assign DtoC_dec_inst = dec_reg_inst;
    assign DtoC_exe_br_eq = ( exe_reg_op1_data == exe_reg_rs2_data);
    assign DtoC_exe_br_lt = ( $signed(exe_reg_op1_data) < $signed(exe_reg_rs2_data));
    assign DtoC_exe_br_ltu = ( exe_reg_op1_data < exe_reg_rs2_data);
    assign DtoC_exe_br_type = exe_reg_ctrl_br_type;
    
    assign DtoC_exe_ctrl_dmem_val = exe_reg_ctrl_mem_val;
    
    /*always@(posedge clk) begin
        if (rst_n == 0) begin
            dmem_data_o <= 32'hdeadbeaf;
        end else begin
            dmem_data_o <= dmem_data_o + 32'd1;
        end
    end*/
    
 endmodule
