/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include <stdio.h>
#include <MK53D10.h>
#include <fsl_dspi.h>
//#include <fsl_clock.h>
#include "st7565.h"
#include "Graphics_Bmp.h"
#include "graphics.h"
#include "drag_invert_glcd_glcd.h"
#include "Nautilus_glcd.h"
#include "Nautilus_inv_glcd.h"
void spi_init(void);
void SPISendByte(unsigned char byte);



/*!
 * @brief Application entry point.
 */
int main(void) {
  /* Init board hardware. */
	int x;
	unsigned char byte = 0xCC;
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  //GPIOA->PDDR|=0x1f000000;
  //GPIOD->PDDR|=0x7000;
  GPIOE->PDDR|=0x300;
  //GPIOD->PDDR|=0xB000;
  spi_init();
  glcd_init();
  draw_bmp(3,0,nautilus_glcd_bmp);
  glcd_refresh();
  //draw_bmp(3,28,nautilus_inv_glcd_bmp);
  //glcd_refresh();

  for(;;) { /* Infinite loop to avoid leaving the main function */
	  x=0x0;
/*
	  while (x<50)
	  {
		  glcd_pixel(x,10,0xFF);
		  x++;
	  }
	  while(x<100)
	  {
		  glcd_pixel(x,10,0x00);
		  x++;
	  }

	  draw_text_bmp((INT8U *)"INVALID ",16,PAGE3,MyFont,1);
	  glcd_refresh();
*/

	  /*
		SPI2->PUSHR = (byte | SPI_PUSHR_CONT(0) | SPI_PUSHR_PCS(2) | SPI_PUSHR_CTAS(0));// write a single byte to the output FIFO - assert CS line
		while (!(SPI2->SR & SPI_SR_RFDF_MASK)) {}// wait for byte to be sent and a byte to be read in
		SPI2->SR |= SPI_SR_RFDF(1);// clear the reception flag (not self-clearing)
		*/
	  //SPISendByte(byte);
	  //x=0;
    __asm("NOP"); /* something to use as a breakpoint stop while looping */
  }
}

void spi_init(void)
{
	/* clock gate */
	//SIM->SCGC5 |= 0x1000;
	SIM->SCGC3 |= SIM_SCGC3_SPI2_MASK;
	//SIM->SCGC7 &= (~0x1);


	//Define config
	dspi_master_config_t	masterConfig;
	//dspi_master_handle_t	masterHandle;
	//dspi_transfer_t			masterTransfer;
	char byte;

	//Set config
	masterConfig.whichCtar                                = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate                      = 500000000;
	masterConfig.ctarConfig.bitsPerFrame                  = 8;
	masterConfig.ctarConfig.cpol                          = kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha                          = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction                     = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec        = 1000000000 / masterConfig.ctarConfig.baudRate ;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec    = 1000000000 / masterConfig.ctarConfig.baudRate ;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.ctarConfig.baudRate ;
	masterConfig.whichPcs                                 = kDSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow                       = kDSPI_PcsActiveLow;
	masterConfig.enableContinuousSCK                      = false;
	masterConfig.enableRxFifoOverWrite                    = false;
	masterConfig.enableModifiedTimingFormat               = false;
	masterConfig.samplePoint                              = kDSPI_SckToSin0Clock;
	//init with above values

//	  //PORT_SetPinMux(PORTD, PIN12_IDX, kPORT_MuxAlt2);           /* PORTD12 (pin 141) is configured as SPI2_SCK */
//	  PORTD->PCR[12] = ((PORTD->PCR[12] &
//	    (~(PORT_PCR_PS_MASK | PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_MUX_MASK))) /* Mask bits to zero which are setting */
//	      | PORT_PCR_PS(1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding Port Pull Enable Register field is set. */
//		  | PORT_PCR_PE(1)
//		  | PORT_PCR_DSE(1)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
//		  | PORT_PCR_MUX(2)
//	  );
//	  //PORT_SetPinMux(PORTD, PIN13_IDX, kPORT_MuxAlt2);           /* PORTD13 (pin 142) is configured as SPI2_SOUT */
//	  PORTD->PCR[13] = ((PORTD->PCR[13] &
//	    (~(PORT_PCR_PS_MASK | PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_MUX_MASK))) /* Mask bits to zero which are setting */
//	      | PORT_PCR_PS(1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding Port Pull Enable Register field is set. */
//		  | PORT_PCR_PE(1)
//		  | PORT_PCR_DSE(1)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
//		  | PORT_PCR_MUX(2)
//	  );
//	  PORTD->PCR[14] = ((PORTD->PCR[14] &
//	    (~(PORT_PCR_PS_MASK | PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_MUX_MASK))) /* Mask bits to zero which are setting */
//	      | PORT_PCR_PS(1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding Port Pull Enable Register field is set. */
//		  | PORT_PCR_PE(1)
//		  | PORT_PCR_DSE(1)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
//		  | PORT_PCR_MUX(2)
//	  );
//	  //PORT_SetPinMux(PORTD, PIN15_IDX, kPORT_MuxAlt2);           /* PORTD15 (pin 144) is configured as SPI2_PCS1 */
//	  PORTD->PCR[15] = ((PORTD->PCR[15] &
//	    (~(PORT_PCR_PS_MASK | PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
//	      | PORT_PCR_PS(1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding Port Pull Enable Register field is set. */
//	      | PORT_PCR_DSE(1)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
//	    );



/*
	DSPI_MasterInit(SPI2, &masterConfig, 32768U);
	//DSPI_SetMasterSlaveMode(SPI2,kDSPI_Master);
	DSPI_MasterTransferCreateHandle(SPI2, &masterHandle, NULL, NULL);
	masterTransfer.txData = 0xFF;
	masterTransfer.dataSize = 0x2;
	//masterHandle.txData = 0xFF;
	masterHandle.totalByteCount = 4;
	DSPI_MasterTransferNonBlocking (SPI2_BASE, &masterHandle, &masterTransfer);
*/
	//POWER_UP(3, SIM_SCGC3_SPI2);
	//_CONFIG_PERIPHERAL(D, 11, (PD_11_SPI2_PCS0 | PORT_SRE_FAST | PORT_DSE_HIGH));
	//_CONFIG_PERIPHERAL(D, 12, (PD_12_SPI2_SCK | PORT_SRE_FAST | PORT_DSE_HIGH));
	//_CONFIG_PERIPHERAL(D, 13, (PD_13_SPI2_SOUT | PORT_SRE_FAST | PORT_DSE_HIGH));
	//_CONFIG_PERIPHERAL(D, 14, PD_14_SPI2_SIN);
	SPI2->MCR = (SPI_MCR_MSTR(1) | 		//Sets DSPI to master mode
			SPI_MCR_DCONF(0) | 			// Sets SPI configuration to SPI
			SPI_MCR_CLR_RXF(1) | 		// Clears the RX FIFO counter
			SPI_MCR_CLR_TXF(1) | 		// Clears the TX FIFO counter
			SPI_MCR_PCSIS(2) );			// Sets the inactive state of PCS1 to High (2 is actually PCSIS 1)
	// for 50MHz bus, 25MHz speed and 140ns min de-select time
	SPI2->CTAR[0] = (SPI_CTAR_DBR(0) | 	// Sets the Double Baud Rate to 1	// was 1
			SPI_CTAR_PBR(0) | 			//
			SPI_CTAR_FMSZ(7) | 			// Sets the Frame Size to 8 bits
			SPI_CTAR_PDT(3) | 			// Sets the delay after Transfer Prescaler to a value of 11(7)
			SPI_CTAR_BR(2) | 			// Sets the Baud Rate Scaler of 0010(6) // was 2
			SPI_CTAR_CPHA(1) | 			// Sets the Clock Phase to data change on leading edge and captured on following edge
			SPI_CTAR_CSSCK(3) |
			SPI_CTAR_LSBFE(0) |
			SPI_CTAR_CPOL(1));			// Sets the Clock Polarity inactive state of High

	//byte = 0x55;
	// test single byte transmission
	/*
	SPI2->PUSHR = (byte | SPI_PUSHR_CONT(0) | SPI_PUSHR_PCS(2) | SPI_PUSHR_CTAS(0));// write a single byte to the output FIFO - assert CS line
	while (!(SPI2->SR & SPI_SR_RFDF_MASK)) {}// wait for byte to be sent and a byte to be read in
	SPI2->SR |= SPI_SR_RFDF(1);// clear the reception flag (not self-clearing)
	*/
/*
	POWER_UP(3, SIM_SCGC3_SPI2);
	_CONFIG_PERIPHERAL(D, 11, (PD_11_SPI2_PCS0 | PORT_SRE_FAST | PORT_DSE_HIGH));
	_CONFIG_PERIPHERAL(D, 12, (PD_12_SPI2_SCK | PORT_SRE_FAST | PORT_DSE_HIGH));
	_CONFIG_PERIPHERAL(D, 13, (PD_13_SPI2_SOUT | PORT_SRE_FAST | PORT_DSE_HIGH));
	_CONFIG_PERIPHERAL(D, 14, PD_14_SPI2_SIN);
	SPI2_MCR = (SPI_MCR_MSTR | SPI_MCR_DCONF_SPI | SPI_MCR_CLR_RXF | SPI_MCR_CLR_TXF | SPI_MCR_PCSIS_CS0 | SPI_MCR_PCSIS_CS1 | SPI_MCR_PCSIS_CS2 | SPI_MCR_PCSIS_CS3 | SPI_MCR_PCSIS_CS4 | SPI_MCR_PCSIS_CS5);
	\SPI2_CTAR0 = (SPI_CTAR_DBR | SPI_CTAR_FMSZ_8 | SPI_CTAR_PDT_7 | SPI_CTAR_BR_2 | SPI_CTAR_CPHA | SPI_CTAR_CPOL);
	// for 50MHz bus, 25MHz speed and 140ns min de-select time
	byte = 0x55;
	// test single byte transmission
	SPI2_PUSHR = (byte | SPI_PUSHR_CONT | SPI_PUSHR_PCS0 | SPI_PUSHR_CTAS_CTAR0)
	// write a single byte to the output FIFO - assert CS line
			while (!(SPI2_SR & SPI_SR_RFDF)) {}
	// wait for byte to be sent and a byte to be read in
	SPI2_SR |= SPI_SR_RFDF;
	// clear the reception flag (not self-clearing)
	 */

}

void SPISendByte(unsigned char byte)
{
	SPI2->PUSHR = (byte | SPI_PUSHR_CONT(0) | SPI_PUSHR_PCS(2) | SPI_PUSHR_CTAS(0));// write a single byte to the output FIFO - assert CS line
	while (!(SPI2->SR & SPI_SR_TCF_MASK)) {}// wait for byte to be sent and a byte to be read in
	SPI2->SR |= SPI_SR_TCF(0);// clear the reception flag (not self-clearing)
}
