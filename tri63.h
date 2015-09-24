/*
 * Tritium TRI63 CAN Driver Controls header
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
 * Last Modified: J.Kennedy, Tritium Pty Ltd, 11 August 2006
 *
 */

// Pin Definitions
// Port 1
#define ENC2_SW			0x01
#define ENC2_B			0x02
#define ENC2_A			0x04
#define ENC1_B			0x08
#define ENC1_SW			0x10
#define ENC1_A			0x20
#define ENC1_LED		0x40
#define ENC2_LED		0x80
#define P1_UNUSED		0x00

// Port 2
#define DEBUG_nLED		0x04
#define DEBUG_nSW		0x08
#define CAN_nINT		0x80
#define P2_UNUSED		0x01 | 0x02 | 0x10 | 0x20 | 0x40

// Port 3
#define CAN_nCS			0x01
#define CAN_MOSI		0x02
#define CAN_MISO		0x04
#define CAN_SCLK		0x08
#define FLASH_nCS		0x10
#define FLASH_MOSI		0x20
#define FLASH_MISO		0x40
#define FLASH_SCLK		0x80
#define P3_UNUSED		0x00

// Port 4
#define nGREEN_A		0x01
#define nGREEN_B		0x02
#define nYELLOW			0x04
#define nRED			0x08
#define LIGHT_HIGH		0x10
#define LIGHT_LOW		0x20
#define LIGHT_PARK		0x40
#define REGEN			0x80
#define P4_UNUSED		0x00

// Port 5
#define BRAKE_1			0x01
#define REVERSE			0x02
#define IGN_ON			0x04
#define IGN_ACC			0x08
#define HORN			0x10
#define HAZARD			0x20
#define LH_IND			0x40
#define RH_IND			0x80
#define P5_UNUSED		0x00

// Port 6
#define ENC1_POT		0x40
#define ENC2_POT		0x80
#define P6_UNUSED		0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x20

// Constant Definitions
#define	TRUE			1
#define FALSE			0

// Quadrature encoder states (reflects combinations of ENCx_A and ENCx_B input pins)
#define STATE_1_A		0x00				// 00 - AB
#define STATE_1_B		0x20				// 10
#define STATE_1_C		0x28				// 11
#define STATE_1_D		0x08				// 01

#define STATE_2_A		0x00				// 00 - AB
#define STATE_2_B		0x04				// 10
#define STATE_2_C		0x06				// 11
#define STATE_2_D		0x02 				// 01

#define CRUISE_STEP		1

// Pushbutton switch states
#define PUSHED			1
#define RELEASED		0
#define BOUNCECNT       3

// Indicator (flasher) LED blink states
#define BLINK_SET		2
#define BLINK_CLR		1
#define BLINK_OFF		0

// LED indicator PWM setup (8 bits)
#define MAX_LED_PWM 	255

// Event timing
#define INPUT_CLOCK		4000000				// Hz
#define TICK_RATE		100 				// Hz
#define BLINK_SPEED		33					// Number of ticks per event: 33*2 = 1.5 Hz = 90 flashes/min (Aust Design Rule 13-00)
#define COMMS_SPEED		10					// Number of ticks per event: 100ms = 10 Hz
#define ACTIVITY_SPEED	2					// Number of ticks to blink CAN activity LED, per trigger event
#define INPUT_SPEED		2					// Number of ticks per event: 20ms = 50 Hz

// Control parameters
#define MAX_CURRENT		1.0					// %, absolute value
#define MIN_CURRENT		0.0					// %, absolute value
#define MAX_VELOCITY_F	100.0					// Forwards max speed, metres/second (100m/s = 360km/h)
#define MAX_VELOCITY_R	-10.0					// Reverse max speed, m/s

// ADC scaling parameters - 12 bit ADC (4096 counts)
#define ADC_MIN			0x0270					// Below this value counts as 0%, to make sure that off is really OFF
#define ADC_MAX			0x0E90  				// Above this value counts as 100%

// Typedefs for quickly joining multiple bytes/ints/etc into larger values
// These rely on byte ordering in CPU & memory - i.e. they're not portable across architectures
typedef union _group_64 {
	float data_fp[2];
	unsigned char data_u8[8];
	unsigned int data_u16[4];
	unsigned long data_u32[2];
} group_64;

typedef union _group_32 {
	float data_fp;
	unsigned char data_u8[4];
	unsigned int data_u16[2];
	unsigned long data_u32;
} group_32;

typedef union _group_16 {
	unsigned char data_u8[2];
	unsigned int data_u16;
} group_16;
