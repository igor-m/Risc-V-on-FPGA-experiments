# Risc-V-on-FPGA-experiments
My naive experiments with RudoIV and picorv32

This is my playground with risc-v SOCs..

RudoIV - see Joerg's https://github.com/bobbl/rudolv
Picorv32 - see Clair's https://github.com/cliffordwolf/picorv32


24.03.2021
rudolv - includes essential files only..
MODs for
 - enabling full 128kB of SPRAM in UPduinos (ice40UP5k fpgas)
 - new bootloader which boots from FLASH instead of UART
 - a test C 125kB large - it tests the full 128kB capabilities

The bootloader does bitbanged SPI via CSRs 
 - added reading of fpga pins in csr.v
 - modded up5k.pcf with UPduino externa 4MB flash wiring


