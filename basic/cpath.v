`timescale 1ns / 1ps

`include "macro_para.v"

module cpath(
    input clk,
    input rst_n,

    output reg [`EXE_PC_SEL-1:0] CtoD_exe_pc_sel,
    output reg [`OPI_SEL-1:0] CtoD_sel_dec_op1,
    output reg [`OPII_SEL-1:0] CtoD_sel_dec_op2_addr,
    
    output CtoD_dec_stall,
    output CtoD_full_stall,
    
    output reg [`BR_TYPE_BIT_NUM-1:0] CtoD_br_type,
    
    output CtoD_if_kill,
    output CtoD_dec_kill,
   
    output reg [`ALU_FUN_BIT_NUM-1:0] CtoD_alu_fun,
    output reg [`WB_SEL_BIT_NUM-1:0] CtoD_wb_sel,    
    
    output reg CtoD_rf_wen,
    output reg CtoD_mem_val,
    
    output reg [`MEM_FCN_BIT_NUM-1:0] CtoD_mem_fcn,
    output reg [`MEM_TYP_BIT_NUM-1:0] CtoD_mem_typ,   
    output reg [`CSR_CMD_BIT_NUM-1:0] CtoD_csr_cmd,
    
    input [`RV_BIT_NUM-1:0] DtoC_dec_inst,
    input DtoC_exe_br_eq,
    input DtoC_exe_br_lt,
    input DtoC_exe_br_ltu,
    input [`BR_TYPE_BIT_NUM-1:0] DtoC_exe_br_type,
    input DtoC_exe_ctrl_dmem_val
    );
    
    wire [`OPCODE_BIT_NUM-1:0] opcode;
    wire [`FUNCTIII_BIT_NUM-1:0] funct3;
    wire [`FUNCTVII_BIT_NUM-1:0] funct7;
    wire [`SHIFT_BIT_NUM-1:0] shift6 ;
    
    assign opcode = DtoC_dec_inst[0 +: `OPCODE_BIT_NUM];
    assign funct3 = DtoC_dec_inst[12 +: `FUNCTIII_BIT_NUM];
    assign funct7 = DtoC_dec_inst[25 +: `FUNCTVII_BIT_NUM];
    assign shift6 = DtoC_dec_inst[26 +: `SHIFT_BIT_NUM];
    
    always@(*) begin
		CtoD_br_type = `BR_N;
		CtoD_sel_dec_op1 = `OPI_DEFAULT;
		CtoD_sel_dec_op2_addr = `OPII_DEFAULT;
		CtoD_alu_fun = `ALU_X;
		CtoD_wb_sel = `WB_X;
		CtoD_rf_wen = 0;
		CtoD_mem_val = 0;
		CtoD_mem_fcn = `M_X;
		CtoD_mem_typ = `MT_X;
		CtoD_csr_cmd = `CSR_N;
        case(opcode)
				/*`OPCODE_BAA_RPA: begin
					 case (funct3)
						`assignSignalsInst(`FUNCTIII_BAA, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_RC, 1, 0, `M_X, `MT_X, `CSR_N) end
					 endcase
				end*/
            `OPCODE_LD: begin
                case (funct3)
           //funct7, branch, op1_sel, op2_sel, alu_sel, wb_sel, rf_wen(wb_en), mem_val(dmem_en), mem_fcn(rd, wr), mem_type(b, w. h), csr
                    `assignSignalsInst(`FUNCTIII_LB, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_MEM, 1, 1, `M_XRD, `MT_B, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_LH, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_MEM, 1, 1, `M_XRD, `MT_H, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_LW, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_MEM, 1, 1, `M_XRD, `MT_W, `CSR_N) end
                    
                    `assignSignalsInst(`FUNCTIII_LBU, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_MEM, 1, 1, `M_XRD, `MT_BU, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_LHU, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_MEM, 1, 1, `M_XRD, `MT_HU, `CSR_N) end

/*                    `FUNCTIII_LB: begin
                        CtoD_br_type = `BR_N;
                        CtoD_sel_dec_op1 = `OPI_DEFAULT;
                        CtoD_sel_dec_op2_addr = `OPII_IMM_ITYPE;
                        CtoD_alu_fun = `ALU_X;
                        CtoD_wb_sel = WB_MEM;//this is to select from the mux
                        CtoD_rf_wen = 1;//this is to enable wb
                        CtoD_mem_val = 1;//this is to enable dmem
                        CtoD_mem_fcn = `M_XRD;//this is read_en or write_en signals to dmem
                        CtoD_mem_typ = `MT_B;//determine byte, word
                        CtoD_csr_cmd = `CSR_N;*/
                endcase
            end
            
            `OPCODE_ST: begin
                case (funct3)
                    `assignSignalsInst(`FUNCTIII_SB, `BR_N, `OPI_DEFAULT, `OPII_IMM_STYPE, `ALU_X, `WB_X, 0, 1, `M_XWR, `MT_B, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_SH, `BR_N, `OPI_DEFAULT, `OPII_IMM_STYPE, `ALU_X, `WB_X, 0, 1, `M_XWR, `MT_H, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_SW, `BR_N, `OPI_DEFAULT, `OPII_IMM_STYPE, `ALU_X, `WB_X, 0, 1, `M_XWR, `MT_W, `CSR_N) end
                endcase
            end
            //not_tested
            `OPCODE_AUIPC: begin
                `assignSignals(`BR_N, `OPI_PC, `OPII_IMM_UTYPE, `ALU_ADD, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N)
             end
             //not_tested
            `OPCODE_LUI: begin
                `assignSignals(`BR_N, `OPI_DEFAULT, `OPII_IMM_UTYPE, `ALU_COPY_2, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N)
            end
            
            `OPCODE_ADDI_ANDI_ORI_XORI_SLTI_SLTIU_SLLI_SRAI_SRLI: begin
                    case (funct3)
                        `FUNCTIII_SRAI_SRLI:
                            case (shift6)
                                 `assignSignalsInst(`SHIFT_SRLI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_SRL, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                                 `assignSignalsInst(`SHIFT_SRAI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_SRA, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            endcase
                            
                            `assignSignalsInst(`FUNCTIII_ADDI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_ADD, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTIII_ANDI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_AND, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTIII_ORI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_OR, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTIII_XORI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_XOR, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTIII_SLTI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_SLT, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTIII_SLTIU, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_SLTU, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTIII_SLLI, `BR_N, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_SLL, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end

                    endcase
            end
            
            `OPCODE_SLL_ADD_SUB_SLT_SLTU_AND_OR_XOR_XOR_SRA_SRL: begin
                case (funct3)
                    `FUNCTIII_ADD_SUB: begin
                        case (funct7)
                            `assignSignalsInst(`FUNCTVII_ADD, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_ADD, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTVII_SUB, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_SUB, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                        endcase
                    end
                    
                    `FUNCTIII_SRA_SRL: begin
                        case (funct7)
                            `assignSignalsInst(`FUNCTVII_SRA, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_SRA, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                            `assignSignalsInst(`FUNCTVII_SRL, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_SRL, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                        endcase
                    end    
                    `assignSignalsInst(`FUNCTIII_SLL, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_SLL, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_SLT, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_SLT, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_SLTU, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_SLTU, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_AND, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_AND, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_OR, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_OR, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_XOR, `BR_N, `OPI_DEFAULT, `OPII_DEFAULT, `ALU_XOR, `WB_ALU, 1, 0, `M_X, `MT_X, `CSR_N) end
                endcase                
            end
                      
            `OPCODE_JAL: begin
                `assignSignals(`BR_J, `OPI_DEFAULT, `OPII_IMM_UJTYPE, `ALU_X, `WB_PC, 1, 0, `M_X, `MT_X, `CSR_N)
            end

            `OPCODE_JALR: begin
                `assignSignals(`BR_JR, `OPI_DEFAULT, `OPII_IMM_ITYPE, `ALU_X, `WB_PC, 1, 0, `M_X, `MT_X, `CSR_N)
            end            
            //not yet tested 
            `OPCODE_BEQ_BNE_BGE_BGEU_BLT_BLTU: begin
                case (funct3)
                    `assignSignalsInst(`FUNCTIII_BEQ, `BR_EQ, `OPI_DEFAULT, `OPII_IMM_SBTYPE, `ALU_X, `WB_X, 0, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_BNE, `BR_NE, `OPI_DEFAULT, `OPII_IMM_SBTYPE, `ALU_X, `WB_X, 0, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_BGE, `BR_GE, `OPI_DEFAULT, `OPII_IMM_SBTYPE, `ALU_X, `WB_X, 0, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_BGEU, `BR_GE, `OPI_DEFAULT, `OPII_IMM_SBTYPE, `ALU_X, `WB_X, 0, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_BLT, `BR_LT, `OPI_DEFAULT, `OPII_IMM_SBTYPE, `ALU_X, `WB_X, 0, 0, `M_X, `MT_X, `CSR_N) end
                    `assignSignalsInst(`FUNCTIII_BLTU, `BR_LTU, `OPI_DEFAULT, `OPII_IMM_SBTYPE, `ALU_X, `WB_X, 0, 0, `M_X, `MT_X, `CSR_N) end
                endcase
            end            
            
            default: begin
                `assignSignals(`BR_N, `OPI_PC, `OPII_IMM_UTYPE, `ALU_ADD, `WB_ALU, 0, 0, `M_X, `MT_X, `CSR_N)
            end
            
        endcase
    end
    
    always@(*) begin
        case(DtoC_exe_br_type)
            `BR_N: CtoD_exe_pc_sel = `EXE_PC_SEL'b00;//<-2bits
            `BR_NE: CtoD_exe_pc_sel = (DtoC_exe_br_eq != 1) ? `EXE_PC_SEL'b01 : `EXE_PC_SEL'b00;
            `BR_EQ: CtoD_exe_pc_sel = (DtoC_exe_br_eq == 1) ? `EXE_PC_SEL'b01 : `EXE_PC_SEL'b00;
            `BR_GE: CtoD_exe_pc_sel = (DtoC_exe_br_lt != 1) ? `EXE_PC_SEL'b01 : `EXE_PC_SEL'b00;
            `BR_GEU: CtoD_exe_pc_sel = (DtoC_exe_br_ltu != 1) ? `EXE_PC_SEL'b01 : `EXE_PC_SEL'b00;
            `BR_LT: CtoD_exe_pc_sel = (DtoC_exe_br_lt == 1) ? `EXE_PC_SEL'b01 : `EXE_PC_SEL'b00;
            `BR_LTU: CtoD_exe_pc_sel = (DtoC_exe_br_ltu == 1) ? `EXE_PC_SEL'b01 : `EXE_PC_SEL'b00;
            `BR_J: CtoD_exe_pc_sel = `EXE_PC_SEL'b01;
            `BR_JR: CtoD_exe_pc_sel = `EXE_PC_SEL'b10;
            
            default: CtoD_exe_pc_sel = `EXE_PC_SEL'b00;
        endcase
    end
    
    assign CtoD_if_kill = (CtoD_exe_pc_sel != `EXE_PC_SEL'b00);//||imem_valid
    assign CtoD_dec_kill = (CtoD_exe_pc_sel != `EXE_PC_SEL'b00);
    
    assign CtoD_dec_stall = 0;
    
    wire dmem_resp_valid;
    /*reg [1:0] count;
    always @(posedge clk) begin
        if (rst_n == 0) begin
            dmem_resp_valid <= 0;
            count<=0;
        end else begin
            dmem_resp_valid <= 0 ;
            if (count == 2'b0) begin
                dmem_resp_valid <= 1 ;
            end
            count <= count + 1;
        end
    end*/
    assign dmem_resp_valid = 1;
    assign CtoD_full_stall = !((DtoC_exe_ctrl_dmem_val && dmem_resp_valid) || !DtoC_exe_ctrl_dmem_val);
endmodule
