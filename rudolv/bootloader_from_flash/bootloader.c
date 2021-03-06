

// BOOTLOADER FROM UPduino's FLASH (loads SPRAM with a binary from address 0x10_000 in flash)
// SPI bitbanging
// igor-m 03/2021

#include "uart.h"


void main(int argc, char **argv) __attribute__ ((noreturn));
// _Noreturn gives a warning with main()

//  #### verilog SPI pin defs
//    assign spics =    leds[4];
//    assign spiclk =   leds[6];
//    assign spido =    leds[5];
//    assign spidi = buttons[7]; 

//                                         76543210
#define SPI_CS_0        clear_csr(0xbc1, 0b00010000)    // CS - chip select
#define SPI_CS_1        set_csr(0xbc1,   0b00010000)    // CS - chip select
#define SPI_DO_0        clear_csr(0xbc1, 0b00100000)    // data to FLASH
#define SPI_DO_1        set_csr(0xbc1,   0b00100000)    // data to FLASH
#define SPI_DI     ((read_csr(0xfc1)>>7)&0b00000001)    // data from FLASH
#define SPI_CK_0        clear_csr(0xbc1, 0b01000000)    // SCK - SPI clock
#define SPI_CK_1        set_csr(0xbc1,   0b01000000)    // SCK - SPI clock

char spitrx(char Data)
{
  char SPIDataIN, SPIDataOUT;                            
  unsigned char Bit;
  SPIDataOUT = Data; 
  SPIDataIN = 0;                           

  SPI_CK_0;                                        

  for (Bit=0x80; Bit; Bit>>=1)                       // Prepare to clock out the byte
  {
    SPIDataIN <<= 1;                                 // Rotate to be ready for the next bit 
    if (SPIDataOUT & Bit)                            // Check for a 1
      SPI_DO_1;                                      // and set the line appropriately
    else
      SPI_DO_0;
    SPI_CK_1;                                        // Set the clock line  
    SPIDataIN += SPI_DI;                             // Read the data from Flash
    SPI_CK_0;                                        // Clear the clock line  } 
     } 
     
  SPI_DO_0;  
  return ((char)SPIDataIN & 0xFF);          // Finally return the read data
}

void main(int argc, char **argv)
{
    unsigned long i;
    unsigned long length = 0;
    unsigned long *p = 0;
    unsigned long word = 0;


    unsigned long start = read_cycle();
    while ((read_cycle() - start) < 30000); // wait on fpga ready

    set_csr(0xbc1,   0b100);

    unsigned long Fl_address = 0x00100000 ;    // 1MB up

    SPI_CK_0; SPI_CS_1; SPI_CS_0;  spitrx(0xAB); SPI_CS_1;     // Wake up from sleep mode
 
    SPI_CK_0; SPI_CS_1; SPI_CS_0;  spitrx(0x03);               // read command is 0x03
   
    spitrx(Fl_address>>16);                                    // write flash starting address
    spitrx(Fl_address>>8);
    spitrx(Fl_address);


    for (i=0; i<0x20000; i++) {            // always read all 128kB
        word = (word >> 8) | ((uint32_t)spitrx(0) << 24);
        if ((i&3)==3) *p++ = word;
    }
    if ((i&3)!=0) {
        // length is not a multiple of 4
        // fill last word correctly (not done in grubby.S)
        *p = word >> (8*(4-(i&3)));
    }

    SPI_CS_1; 
    clear_csr(0xbc1,   0b100);
    set_csr(0xbc1,   0b001);

    asm volatile ("jr x0");
    while (1); // avoid warning
}

// SPDX-License-Identifier: ISC


    

