`ifndef macros_para
`define macros_para

//Vendor Versions
`define XILINX

//Board Versions
//`define LEG_BOARD


//define MEM
`define IMEMM_ADDR_BIT_NUM    10
`define IMEMM_DEPTH    1024

`define DMEMM_ADDR_BIT_NUM    12
`define DMEMM_DEPTH    4096

//`define BLKRAM_REG

`define RV_BIT_NUM    32
`define RV_BIT_NUM_DIVIV    8
`define RV_BIT_NUM_DIVIV_NUM    4
`define ADDR_BIT_NUM    5


`define OPCODE_BIT_NUM  7
`define FUNCTIII_BIT_NUM  3
`define SHIFT_BIT_NUM   6
`define FUNCTVII_BIT_NUM    7

//decode state
`define IMM_I_BIT_NUM   12

`define IMM_S_LOW_BIT_NUM   5
`define IMM_S_HIGH_BIT_NUM   7

`define IMM_SB_LOW_BIT_NUM   4
`define IMM_SB_MID_BIT_NUM   6

`define IMM_U_BIT_NUM   20

`define IMM_UJ_LOW_BIT_NUM   10
`define IMM_UJ_MID_BIT_NUM   8



//decode state: the mux ctrl signal
`define EXE_PC_SEL  2
`define OPI_SEL     2
`define OPI_SEL_PLUS     3
`define OPII_SEL    3
`define RSII_SEL    2
`define ADDR_SEL    2
`define WB_SEL     3

//OP1
`define OPI_IMZ         `OPI_SEL'd0
`define OPI_PC          `OPI_SEL'd1
`define OPI_DEFAULT          `OPI_SEL'd2

`define OPI_MUX_IMZ         `OPI_SEL_PLUS'd0
`define OPI_MUX_PC          `OPI_SEL_PLUS'd1
`define OPI_MUX_EXEFWD          `OPI_SEL_PLUS'd2
`define OPI_MUX_WBFWD          `OPI_SEL_PLUS'd3
`define OPI_MUX_DEFAULT          `OPI_SEL_PLUS'd4


//OP2
`define OPII_IMM_ITYPE         `OPII_SEL'd0
`define OPII_IMM_STYPE         `OPII_SEL'd1
`define OPII_IMM_SBTYPE         `OPII_SEL'd2
`define OPII_IMM_UTYPE         `OPII_SEL'd3
`define OPII_IMM_UJTYPE         `OPII_SEL'd4
`define OPII_DEFAULT            `OPII_SEL'd5

`define OPII_MUX_IMM_ITYPE         `OPII_SEL'd0
`define OPI_MUXI_IMM_STYPE         `OPII_SEL'd1
`define OPII_MUX_IMM_SBTYPE         `OPII_SEL'd2
`define OPII_MUX_IMM_UTYPE         `OPII_SEL'd3
`define OPII_MUX_IMM_UJTYPE         `OPII_SEL'd4
`define OPII_MUX_MUX_EXEFWD          `OPII_SEL'd5
`define OPII_MUX_MUX_WBFWD          `OPII_SEL'd6
`define OPII_MUX_DEFAULT            `OPII_SEL'd7



//RS
`define RS_MUX_EXEFWD         `RSII_SEL'd0
`define RS_MUX_WBFWD          `RSII_SEL'd1
`define RS_MUX_DEFAULT         `RSII_SEL'd2


//mem_addr
`define ADDR_MUX_EXEFWD         `ADDR_SEL'd0
`define ADDR_MUX_WBFWD          `ADDR_SEL'd1
`define ADDR_MUX_DEFAULT         `ADDR_SEL'd2



`define CSR_N           `CSR_CMD_BIT_NUM'd0

//exe state:
`define ALU_FUN_BIT_NUM     4
`define WB_SEL_BIT_NUM       2
`define BR_TYPE_BIT_NUM     4
`define MEM_FCN_BIT_NUM     2
`define MEM_TYP_BIT_NUM     3
`define CSR_CMD_BIT_NUM     2

`define CSR_CMD_BIT_NUM     2


//alu operation signal
`define ALU_ADD     `ALU_FUN_BIT_NUM'd0
`define ALU_SUB     `ALU_FUN_BIT_NUM'd1
`define ALU_SLL     `ALU_FUN_BIT_NUM'd2
`define ALU_SRL     `ALU_FUN_BIT_NUM'd3
`define ALU_SRA     `ALU_FUN_BIT_NUM'd4
`define ALU_AND     `ALU_FUN_BIT_NUM'd5
`define ALU_OR     `ALU_FUN_BIT_NUM'd6
`define ALU_XOR     `ALU_FUN_BIT_NUM'd7
`define ALU_SLT     `ALU_FUN_BIT_NUM'd8
`define ALU_SLTU     `ALU_FUN_BIT_NUM'd9
`define ALU_COPY_1     `ALU_FUN_BIT_NUM'd10
`define ALU_COPY_2     `ALU_FUN_BIT_NUM'd11
`define ALU_X     `ALU_FUN_BIT_NUM'd12

//csr signal
`define CSR_ADDR_MSB    31
`define CSR_ADDR_LSB    20

//mem_fcn signal
`define M_X             `MEM_FCN_BIT_NUM'b00
`define M_XRD            `MEM_FCN_BIT_NUM'b01
`define M_XWR            `MEM_FCN_BIT_NUM'b10

//mem_typ_signal
`define MT_X            `MEM_TYP_BIT_NUM'd0
`define MT_W            `MEM_TYP_BIT_NUM'd1
`define MT_B            `MEM_TYP_BIT_NUM'd2
`define MT_BU            `MEM_TYP_BIT_NUM'd3
`define MT_H            `MEM_TYP_BIT_NUM'd4
`define MT_HU            `MEM_TYP_BIT_NUM'd5


//wb signal
`define WB_X         `WB_SEL'd0
`define WB_ALU         `WB_SEL'd0
`define WB_PC          `WB_SEL'd1
`define WB_MEM          `WB_SEL'd2
`define WB_CSR          `WB_SEL'd3
`define WB_RC          `WB_SEL'd4



//cpath
`define BR_N            `BR_TYPE_BIT_NUM'd0
`define BR_J            `BR_TYPE_BIT_NUM'd1
`define BR_JR            `BR_TYPE_BIT_NUM'd2
`define BR_EQ           `BR_TYPE_BIT_NUM'd3
`define BR_NE            `BR_TYPE_BIT_NUM'd4
`define BR_GE           `BR_TYPE_BIT_NUM'd5
`define BR_GEU            `BR_TYPE_BIT_NUM'd6
`define BR_LT           `BR_TYPE_BIT_NUM'd7
`define BR_LTU           `BR_TYPE_BIT_NUM'd8

//opcode
`define OPCODE_LD     `OPCODE_BIT_NUM'b0000011
`define OPCODE_ST     `OPCODE_BIT_NUM'b0100011

`define OPCODE_AUIPC    `OPCODE_BIT_NUM'b0010111
`define OPCODE_LUI    `OPCODE_BIT_NUM'b0110111


`define OPCODE_ADDI_ANDI_ORI_XORI_SLTI_SLTIU_SLLI_SRAI_SRLI     `OPCODE_BIT_NUM'b0010011
`define OPCODE_SLL_ADD_SUB_SLT_SLTU_AND_OR_XOR_XOR_SRA_SRL   `OPCODE_BIT_NUM'b0110011

`define OPCODE_JAL    `OPCODE_BIT_NUM'b1101111
`define OPCODE_JALR   `OPCODE_BIT_NUM'b1100111
`define OPCODE_BEQ_BNE_BGE_BGEU_BLT_BLTU   `OPCODE_BIT_NUM'b1100011

`define OPCODE_BAA_RPA	`SHIFT_BIT_NUM'b0001011
`define FUNCTIII_BAA     `FUNCTIII_BIT_NUM'b000
`define FUNCTIII_RPA     `FUNCTIII_BIT_NUM'b001


//funct3 for OPCODE_LD
`define FUNCTIII_LB     `FUNCTIII_BIT_NUM'b000
`define FUNCTIII_LH     `FUNCTIII_BIT_NUM'b001
`define FUNCTIII_LW     `FUNCTIII_BIT_NUM'b010

`define FUNCTIII_LBU     `FUNCTIII_BIT_NUM'b100
`define FUNCTIII_LHU     `FUNCTIII_BIT_NUM'b101
//`define FUNCTIII_LWU     `FUNCTIII_BIT_NUM'b110

//funct3 for OPCODE_ST
`define FUNCTIII_SB     `FUNCTIII_BIT_NUM'b000
`define FUNCTIII_SH     `FUNCTIII_BIT_NUM'b001
`define FUNCTIII_SW     `FUNCTIII_BIT_NUM'b010

//funct3 for OPCODE_ADDI_ANDI_ORI_XORI_SLTI_SLTIU_SLLI_SRAI_SRLI
`define FUNCTIII_ADDI     `FUNCTIII_BIT_NUM'b000
`define FUNCTIII_ANDI     `FUNCTIII_BIT_NUM'b111
`define FUNCTIII_ORI     `FUNCTIII_BIT_NUM'b110
`define FUNCTIII_XORI     `FUNCTIII_BIT_NUM'b100
`define FUNCTIII_SLTI     `FUNCTIII_BIT_NUM'b010
`define FUNCTIII_SLTIU    `FUNCTIII_BIT_NUM'b011
`define FUNCTIII_SLLI    `FUNCTIII_BIT_NUM'b001
`define FUNCTIII_SRAI_SRLI     `FUNCTIII_BIT_NUM'b101

//funct3 for OPCODE_SLL_ADD_SUB_SLT_SLTU_AND_OR_XOR_XOR_SRA_SRL
`define FUNCTIII_SLL     `FUNCTIII_BIT_NUM'b001
`define FUNCTIII_ADD_SUB     `FUNCTIII_BIT_NUM'b000
`define FUNCTIII_SLT     `FUNCTIII_BIT_NUM'b010
`define FUNCTIII_SLTU     `FUNCTIII_BIT_NUM'b011
`define FUNCTIII_AND     `FUNCTIII_BIT_NUM'b111
`define FUNCTIII_OR     `FUNCTIII_BIT_NUM'b110
`define FUNCTIII_XOR     `FUNCTIII_BIT_NUM'b100
`define FUNCTIII_SRA_SRL     `FUNCTIII_BIT_NUM'b101

//funct3 for OPCODE_BEQ_BNE_BGE_BGEU_BLT_BLTU
`define FUNCTIII_BEQ     `FUNCTIII_BIT_NUM'b000
`define FUNCTIII_BNE     `FUNCTIII_BIT_NUM'b001
`define FUNCTIII_BGE     `FUNCTIII_BIT_NUM'b101
`define FUNCTIII_BGEU     `FUNCTIII_BIT_NUM'b111
`define FUNCTIII_BLT     `FUNCTIII_BIT_NUM'b100
`define FUNCTIII_BLTU     `FUNCTIII_BIT_NUM'b110


//func7 for OPCODE_SLL_ADD_SUB_SLT_SLTU_AND_OR_XOR_XOR_SRA_SRL
`define FUNCTVII_ADD     `FUNCTVII_BIT_NUM'b0000000
`define FUNCTVII_SUB     `FUNCTVII_BIT_NUM'b0100000

`define FUNCTVII_SRL     `FUNCTVII_BIT_NUM'b0000000
`define FUNCTVII_SRA     `FUNCTVII_BIT_NUM'b0100000

//shift6
`define SHIFT_SRLI     `SHIFT_BIT_NUM'b000000
`define SHIFT_SRAI     `SHIFT_BIT_NUM'b010000

//RC
`define RC_0	0
`define RC_1	1

`define assignSignalsInst(inst, br_type, opI_sel, opII_addr_sel, alu_fun, wb_sel, rf_wen, mem_val, mem_fcn, mem_typ, csr_cmd, m_rc_cmd) \
                        inst: begin \
                        CtoD_br_type = br_type; \
                        CtoD_sel_dec_op1 = opI_sel; \
                        CtoD_sel_dec_op2_addr = opII_addr_sel; \
                        CtoD_alu_fun = alu_fun; \
                        CtoD_wb_sel = wb_sel;//this is to select from the mux \
                        CtoD_rf_wen = rf_wen;//this is to enable wb \
                        CtoD_mem_val = mem_val;//this is to enable dmem \
                        CtoD_mem_fcn = mem_fcn;//this is read_en or write_en signals to dmem \
                        CtoD_mem_typ = mem_typ;//determine byte, word \
                        CtoD_csr_cmd = csr_cmd; \
								rc_cmd = m_rc_cmd;

`define assignSignals( br_type, opI_sel, opII_addr_sel, alu_fun, wb_sel, rf_wen, mem_val, mem_fcn, mem_typ, csr_cmd, m_rc_cmd) \
                        CtoD_br_type = br_type; \
                        CtoD_sel_dec_op1 = opI_sel; \
                        CtoD_sel_dec_op2_addr = opII_addr_sel; \
                        CtoD_alu_fun = alu_fun; \
                        CtoD_wb_sel = wb_sel;//this is to select from the mux \
                        CtoD_rf_wen = rf_wen;//this is to enable wb \
                        CtoD_mem_val = mem_val;//this is to enable dmem \
                        CtoD_mem_fcn = mem_fcn;//this is read_en or write_en signals to dmem \
                        CtoD_mem_typ = mem_typ;//determine byte, word \
                        CtoD_csr_cmd = csr_cmd; \
								rc_cmd = m_rc_cmd;



`endif
