/* wrapper for iCE40 UP5K MDP board

Memory map
0000'0000h 64KiB main memory (SPRAM)
0002'0000h  1KiB boot loader (BRAM)

CSR
BC0h    UART
BC2h    Timer

Copyright (c) 2018-2019 Jörg Mische <bobbl@gmx.de>

MOD:  
23.3.2021 JoergM - for use with full 128kB of SPRAM
          igor-m - for accessing the UPduino flash
*/


module top (

    input uart_rx,
    output uart_tx,

    input resetbutt,
    input testbutt,

	output led_r,
	output led_g,
	output led_b,

//  SPI flash

    output spics,
    output spiclk,
    output spido,
    input  spidi   

);
    localparam integer CLOCK_RATE = 24_000_000;
    //localparam integer CLOCK_RATE = 12_000_000;
    localparam integer BAUD_RATE = 115200;

    wire clk;

    SB_HFOSC OSCInst0(
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF(clk)   );
    defparam OSCInst0.CLKHF_DIV = "0b01"; // 48 MHz / 2
    //defparam OSCInst0.CLKHF_DIV = "0b10"; // 48 MHz / 4

  // ######   Reset logic   #####################################

    reg [8:0] reset_cnt = 0;
    wire rstn = &reset_cnt;

    always @(posedge clk) begin
      if (resetbutt) reset_cnt <= reset_cnt + !rstn;
      else        reset_cnt <= 0;
    end

  // ######   RGB LED indicator   ############################

  defparam RGBA_DRIVER.CURRENT_MODE = "0b1";

  defparam RGBA_DRIVER.RGB0_CURRENT = "0b000001";
  defparam RGBA_DRIVER.RGB1_CURRENT = "0b000001";
  defparam RGBA_DRIVER.RGB2_CURRENT = "0b000001";


    SB_RGBA_DRV RGBA_DRIVER (
        .CURREN(1'b1),
        .RGBLEDEN(1'b1),
        .RGB0PWM(leds[0]),
        .RGB1PWM(leds[1]),
        .RGB2PWM(leds[2]),
        .RGB0(led_r),
        .RGB1(led_g),
        .RGB2(led_b)
    );

	// #### Hearth Beat
	//wire hbeat;
	//reg [22:0] cntrhb;

    //always @ (posedge clk) begin
    //    cntrhb <= cntrhb + 1;
    //end

    // CSR_UART  = 12'hBC0,
    // CSR_OUTPINS  = 12'hBC1,
    // CSR_INPINS  = 12'hFC1,
    // CSR_TIMER = 12'hBC2,
    // CSR_KHZ   = 12'hFC0

    // RGB LEDS via CSR 

    wire [7:0] leds ;

    // INPUT testbutt    0x00000001

    wire [7:0] buttons ;
    assign testbutt = buttons[0];
//

// ################## External UPduino FLASH SPI  #############
// 03/2021 igor-m
    assign spics = leds[4];
    assign spiclk = leds[6];
    assign spido =  leds[5];
    assign  spidi = buttons[7];  

//  #### RUDOLV change for all 4 SPRAM modules ##############################
//  03/2021 JoergM

    wire        mem_valid;
    wire        mem_write;
    wire        mem_write_main0 = mem_write & ~mem_addr[17] & ~mem_addr[16];
    wire        mem_write_main1 = mem_write & ~mem_addr[17] &  mem_addr[16];
    wire        mem_write_boot = mem_write & mem_addr[17];
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire        mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_main0;
    wire [31:0] mem_rdata_main1;
    wire [31:0] mem_rdata_boot;
    wire [31:0] mem_rdata = q_SelBootMem ? mem_rdata_boot
                                 : (q_SelWhichMainMem ? mem_rdata_main1 
                                    : mem_rdata_main0);

    reg q_SelBootMem;
    reg q_SelWhichMainMem;
    always @(posedge clk) begin
        q_SelBootMem <= rstn ? mem_addr[17] : 0;
        q_SelWhichMainMem <= mem_addr[16];
    end

    wire        regset_we;
    wire  [5:0] regset_wa;
    wire [31:0] regset_wd;
    wire        regset_wg;
    wire  [5:0] regset_ra1;
    wire  [5:0] regset_ra2;
    wire [31:0] regset_rd1;
    wire        regset_rg1;
    wire [31:0] regset_rd2;
    wire        regset_rg2;

    wire        irq_software = 0;
    wire        irq_timer;
    wire        irq_external = 0;
    wire        retired;

    wire        csr_read;
    wire  [2:0] csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata;
    wire        csr_valid;

    CsrDefault #(
        .OUTPINS_COUNT(8),
        .INPINS_COUNT(8),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE),
        .TIMER_WIDTH(32)
    ) csrs (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (csr_rdata),
        .valid  (csr_valid),

        .retired(retired),
        .rx     (uart_rx),
        .tx     (uart_tx),

        .outpins(leds),
        .inpins(buttons),

        .irq_timer(irq_timer),

        .AVOID_WARNING()
    );

    Pipeline #(
        .START_PC       (32'h0002_0000)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

        .irq_software   (irq_software),
        .irq_timer      (irq_timer),
        .irq_external   (irq_external),
        .retired        (retired),

        .csr_read       (csr_read),
        .csr_modify     (csr_modify),
        .csr_wdata      (csr_wdata),
        .csr_addr       (csr_addr),
        .csr_rdata      (csr_rdata),
        .csr_valid      (csr_valid),

        .mem_valid      (mem_valid),
        .mem_write      (mem_write),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_wgrubby    (mem_wgrubby),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata),
        .mem_rgrubby    (mem_rgrubby),

        .regset_we      (regset_we),
        .regset_wa      (regset_wa),
        .regset_wd      (regset_wd),
        .regset_wg      (regset_wg),
        .regset_ra1     (regset_ra1),
        .regset_ra2     (regset_ra2),
        .regset_rd1     (regset_rd1),
        .regset_rg1     (regset_rg1),
        .regset_rd2     (regset_rd2),
        .regset_rg2     (regset_rg2)
    );

SPRAMMemory mainmem0 (
    .clk    (clk),
    .write  (mem_write_main0),
    .wmask  (mem_wmask),
    .wdata  (mem_wdata),
    .addr   (mem_addr[15:2]),
    .rdata  (mem_rdata_main0)
);

SPRAMMemory mainmem1 (
    .clk    (clk),
    .write  (mem_write_main1),
    .wmask  (mem_wmask),
    .wdata  (mem_wdata),
    .addr   (mem_addr[15:2]),
    .rdata  (mem_rdata_main1)
);

    Memory32 #(
        .ADDR_WIDTH(9), // 512 words??? igor-m
        .CONTENT("../../sw/uart/build/bootloader_char.hex")
	//.CONTENT("../../sw/uart/build/test_char.hex")
    ) bootmem (
        .clk    (clk),
        .write  (mem_write_boot),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[10:2]),
        .rdata  (mem_rdata_boot)
    );

`ifdef ENABLE_GRUBBY
    wire mem_rgrubby_main;
    wire mem_rgrubby_boot;
    wire mem_rgrubby = q_SelBootMem ? mem_rgrubby_boot : mem_rgrubby_main;

    Memory1 #(
        .ADDR_WIDTH(14)
    ) mainmem_grubby (
        .clk    (clk),
        .write  (mem_write_main),
        .wdata  (mem_wgrubby),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rgrubby_main)
    );
    Memory1 #(
        .ADDR_WIDTH(8)
    ) bootmem_grubby (
        .clk    (clk),
        .write  (mem_write_boot),
        .wdata  (mem_wgrubby),
        .addr   (mem_addr[9:2]),
        .rdata  (mem_rgrubby_boot)
    );
`else
    wire mem_rgrubby = 0;
`endif

`ifdef ENABLE_GRUBBY
    RegSet32g1
`else
    RegSet32
`endif
    regset (
        .clk    (clk),
        .we     (regset_we),
        .wa     (regset_wa),
        .wd     (regset_wd),
        .wg     (regset_wg),
        .ra1    (regset_ra1),
        .ra2    (regset_ra2),
        .rd1    (regset_rd1),
        .rg1    (regset_rg1),
        .rd2    (regset_rd2),
        .rg2    (regset_rg2)
    );
endmodule


module SPRAMMemory (
    input clk,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [13:0] addr,
    output reg [31:0] rdata
);

SB_SPRAM256KA spram_lo(
    .DATAIN     (wdata[15:0]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[1], wmask[1], wmask[0], wmask[0]}),
    .WREN       (write),
    .CHIPSELECT (1'b1),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[15:0])
    );

SB_SPRAM256KA spram_hi(
    .DATAIN     (wdata[31:16]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[3], wmask[3], wmask[2], wmask[2]}),
    .WREN       (write),
    .CHIPSELECT (1'b1),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[31:16])
    );

endmodule


// SPDX-License-Identifier: ISC
