`timescale 1ns / 1ps

`include "macro_para.v"


module regfile(
    input clk,
    input rst_n,

    input [`ADDR_BIT_NUM-1:0] rs1_addr,
    input [`ADDR_BIT_NUM-1:0] rs2_addr,
    
    //`ifdef  BLKRAM_REG
    //output wire [`RV_BIT_NUM-1:0] rs1_data,
    //output wire [`RV_BIT_NUM-1:0] rs2_data,
    //`else
    output reg [`RV_BIT_NUM-1:0] rs1_data,
    output reg [`RV_BIT_NUM-1:0] rs2_data,    
    //`endif
    
    input [`ADDR_BIT_NUM-1:0] waddr,
    input [`RV_BIT_NUM-1:0] wdata,
    input  wen
    );
    
    wire [`RV_BIT_NUM-1:0] wdata_real;
    assign wdata_real = (waddr == 0)? 0: wdata;
    
//`ifdef  BLKRAM_REG
    
/*    regfileblkmem regfile0 (
      .clka(clk), // input clka
      .wea(wen), // input [0 : 0] wea
      .addra(waddr), // input [4 : 0] addra
      .dina(wdata_real), // input [31 : 0] dina
      
      .clkb(clk), // input clkb
      .rstb(!(rst_n)), // input rstb
      .addrb(rs1_addr), // input [4 : 0] addrb
      .doutb(rs1_data) // output [31 : 0] doutb
    );
    
    regfileblkmem regfile1 (
      .clka(clk), // input clka
      .wea(wen), // input [0 : 0] wea
      .addra(waddr), // input [4 : 0] addra
      .dina(wdata_real), // input [31 : 0] dina
      
      .clkb(clk), // input clkb
      .rstb(!(rst_n)), // input rstb
      .addrb(rs2_addr), // input [4 : 0] addrb
      .doutb(rs2_data) // output [31 : 0] doutb
    );*/
    
//`else
    
    reg [`RV_BIT_NUM-1:0] regfileReg [0:`RV_BIT_NUM-1];

    
    always@(posedge clk) begin
        
        rs1_data <= (rs1_addr == 0)? 0 : regfileReg[rs1_addr];
        rs2_data <= (rs2_addr == 0)? 0 : regfileReg[rs2_addr];


        if (wen) begin
            regfileReg[waddr] <= wdata;
        end  
        
        /*if (rs1_addr == 0) begin
            rs1_data <= 0;
        end else begin
            rs1_data <= regfileReg[rs1_addr];
        end

        if (rs2_addr == 0) begin
            rs2_data <= 0;
        end else begin        
            rs2_data <= regfileReg[rs2_addr];
        end
        

    end*/
      

    end

//`endif
endmodule

