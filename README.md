# riscv_overlay

This soft processor is designed to be tightly-coupled with a custom coarse-grained architecture to become part of an FPGA overlay framework.

RISC-V RV32I is chosen as the instruction set for its openness and portability, and the soft processor is designed as a 4-stage pipeline to minimize the resource consumption when implemented on FPGAs.

The processor is generically implemented so as to promote design portability and compatibility across different FPGA platforms. Current implemenations support Spartan-3 to Virtex-7 and Cyclone IV to V.


To use this soft processor, instantiate a new project in ISE or Quartus II and put all the files in the basic directory in there. Ok, now, you are basically done.

If you want to implement the design onto Xilinx platforms that are order than Virtex5, please uncomment the macro LEG_BOARD in macro.para.v.

If you want to put the soft processor on Altrea FPGA, comment macros XILINX and LEG_BOARD in macro.para.v.

Note that the IMEM and DMEM are implemented as block RAM. You can change their size by modifying macro: IMEMM_ADDR_BIT_NUM, IMEMM_DEPTH and DMEMM_ADDR_BIT_NUM, DMEMM_DEPTH respectively.

