/*
 * Tritium MSP430 SPI interface
 * Copyright (c) 2006, Tritium Pty Ltd.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products 
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE. 
 *
 * Last Modified: J.Kennedy, Tritium Pty Ltd, 18 December 2006
 *
 * - Implements the following SPI interface functions
 *	- init
 *	- transmit
 *	- exchange
 *
 */

// Include files
#include <msp430x13x.h>
#include "tri63.h"
#include "spi.h"

/*
 * Initialise SPI port
 * 	- Master, 8 bits, mode 0:0, max speed, 3 wire
 * 	- Clock = 0: SMCLK /2 (works with no external clock input, DCO only)
 *	- Clock = 1: ACLK  /2 (fastest possible, from external clock input)
 */
void spi_init( unsigned char clock )
{
	P3SEL |= CAN_MOSI | CAN_MISO | CAN_SCLK;	// Set pins to peripheral function, not GPIO
	U0CTL = CHAR | SYNC | MM | SWRST;			// 8-bit, SPI, Master
	if( clock == 0 ) U0TCTL = SSEL1 | STC;	// Mode 0:0, BRCLK = SMCLK, 3-wire Mode
	else U0TCTL = SSEL0 | STC;				// Mode 0:0, BRCLK = ACLK, 3-wire Mode
	U0BR0 = 0x02;								// SPICLK = BRCLK/2
	U0BR1 = 0x00;
	U0MCTL = 0x00;								// No modulation
	ME1 |= USPIE0;								// Module enable
	U0CTL &= ~SWRST;							// SPI enable
}

/*
 * Transmits data on SPI connection
 *	- Busy waits until entire shift is complete
 *	- On devices with hardware SPI support, this function is identical to spi_exchange,
 *	  with the execption of not returning a value
 *	- On devices with software (bit-bashed) SPI support, this function can run faster
 *	  because it does not require data reception code
 */
void spi_transmit( unsigned char data )
{
	U0TXBUF = data;
	while(( IFG1 & URXIFG0 ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	U0RXBUF;
}

/*
 * Exchanges data on SPI connection
 *	- Busy waits until entire shift is complete
 *	- This function is safe to use to control hardware lines that rely on shifting being finalised
 */
unsigned char spi_exchange( unsigned char data )
{
	U0TXBUF = data;
	while(( IFG1 & URXIFG0 ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	return( U0RXBUF );
}
