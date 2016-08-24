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
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	GPIOE->PDDR|=0x300;

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

    __asm("NOP"); /* something to use as a breakpoint stop while looping */
  }
}

void spi_init(void)
{
	/* clock gate */
	SIM->SCGC3 |= SIM_SCGC3_SPI2_MASK;

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

}

void SPISendByte(unsigned char byte)
{
	SPI2->PUSHR = (byte | SPI_PUSHR_CONT(0) | SPI_PUSHR_PCS(2) | SPI_PUSHR_CTAS(0));// write a single byte to the output FIFO - assert CS line
	while (!(SPI2->SR & SPI_SR_TCF_MASK)) {}// wait for byte to be sent and a byte to be read in
	SPI2->SR |= SPI_SR_TCF(0);// clear the reception flag (not self-clearing)
}
