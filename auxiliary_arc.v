`timescale 1ns / 1ps

`include "macro_para.v"

module auxiliary_arc(
    input clk,
    input rst,
    input rst_n,

    output [`RV_BIT_NUM_DIVIV_NUM-1:0] aux_mem_keep,
    output [`RV_BIT_NUM-1:0] aux_mem_datai,
    output [`RV_BIT_NUM-1:0] aux_mem_addr,   

    input [`RV_BIT_NUM-1:0] aux_mem_datao,

    input [`RV_BIT_NUM-1:0] aux_start_addr,
    input aux_en,
    output reg aux_done
    );
	 
    always@(posedge clk) begin
        if (rst_n == 0) begin
            aux_done <= 0;
        end else begin
            if (aux_en == 1) begin
                aux_done <= 1;
            end else begin
                aux_done <= 0;
            end
        end
              
    end


endmodule
