# RISCV-Overlay

This soft processor is designed to be tightly-coupled with a custom coarse-grained architecture to become part of an FPGA overlay framework.

RISC-V `RV32I` is chosen as the instruction set for its openness and portability, and the soft processor is designed as a 4-stage pipeline to minimize the resource consumption when implemented on FPGAs.

The processor is generically implemented so as to promote design portability and compatibility across different FPGA platforms. Current implementations support Spartan-3 to Virtex-7 and Cyclone IV to V.

## Directory

`basic` consists of the RISCV core without the BAA and RPA instructions.

If you want to use the RISCV core that supports the BAA and RPA instructions, please ignore the directory `basic`.

You can refer to the details of the BAA and RPA instructions by visiting

    http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6927421

## Installation

You can specify the device, IMEM and DMEM size by executing `install.sh`

OR you can do it manually with the following steps:

1. If you want to implement the design onto Xilinx platforms that are order than Virtex5, please uncomment the macro LEG_BOARD in macro.para.v.

2. If you want to put the soft processor on Altrea FPGA, comment macros XILINX and LEG_BOARD in macro.para.v.

3. The IMEM and DMEM are implemented as block RAM. You can change their size by modifying macro: IMEMM_ADDR_BIT_NUM, IMEMM_DEPTH and DMEMM_ADDR_BIT_NUM, DMEMM_DEPTH respectively.

## Synthesize

You can synthesize the processor by first instantiating a new project in ISE or Quartus II. Depending on whether you need the BAA and RPA instructions, you can then place the files from either the `basic` or the files other than `basic` into the project.
