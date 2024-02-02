/*****************************************************************************
   Title:     AVR Monitor
   Author:    Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
   Compiler:  avr-gcc 3.4.5 or 4.1 / avr-libc 1.7.1
   Hardware:  All AVRs
   License:   GNU General Public License

   Modified:  K Scott Vitale ksv_prj@gmx.com
   Date:      18 July 2016
   Compiler:  avr-gcc 4.5.3
   Description: Convert stk500v2 to stand-alone AVR Monitor

   LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software;	you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation;	either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY;	without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*****************************************************************************/

//************************************************************************
//*	Edit History
//************************************************************************
//*	Jul  8,	2010	<MLS> Adding monitor code
//*	Jul 18,	2016	<KSV> Issue 544: stk500v2 bootloader doesn't support reading fuses
//************************************************************************

#include	<inttypes.h>
#include	<avr/io.h>
#include	<avr/interrupt.h>
#include	<avr/boot.h>
#include	<avr/pgmspace.h>
#include	<util/delay.h>
#include	<avr/eeprom.h>
#include	<avr/common.h>
#include	<stdlib.h>

static void	RunMonitor(void);

#ifndef EEWE
  #define EEWE	1
#endif
#ifndef EEMWE
  #define EEMWE	2
#endif

/*
 * define CPU frequency in Mhz here if not defined in Makefile
 */
#ifndef F_CPU
  #define F_CPU 16000000UL
#endif

/*
 * UART Baudrate, AVRStudio AVRISP only accepts 115200 bps
 */

#ifndef BAUDRATE
  #define BAUDRATE 38400
#endif

/*
 *  Enable (1) or disable (0) USART double speed operation
 */
#ifndef UART_BAUDRATE_DOUBLE_SPEED
  #if defined (__AVR_ATmega32__)
    #define UART_BAUDRATE_DOUBLE_SPEED 0
  #else
    #define UART_BAUDRATE_DOUBLE_SPEED 1
  #endif
#endif

/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			1
#define CONFIG_PARAM_SW_MINOR			0

#if defined(__AVR_AT90USB1287__) || defined(__AVR_AT90USB1286__)
  #define UART_BAUD_RATE_LOW			UBRR1L
  #define UART_STATUS_REG				UCSR1A
  #define UART_CONTROL_REG			UCSR1B
  #define UART_ENABLE_TRANSMITTER		TXEN1
  #define UART_ENABLE_RECEIVER		RXEN1
  #define UART_TRANSMIT_COMPLETE		TXC1
  #define UART_RECEIVE_COMPLETE		RXC1
  #define UART_DATA_REG				UDR1
  #define UART_DOUBLE_SPEED			U2X1

#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) \
	|| defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__)
/* ATMega8 with one USART */
  #define UART_BAUD_RATE_LOW			UBRRL
  #define UART_STATUS_REG				UCSRA
  #define UART_CONTROL_REG			UCSRB
  #define UART_ENABLE_TRANSMITTER		TXEN
  #define UART_ENABLE_RECEIVER		RXEN
  #define UART_TRANSMIT_COMPLETE		TXC
  #define UART_RECEIVE_COMPLETE		RXC
  #define UART_DATA_REG				UDR
  #define UART_DOUBLE_SPEED			U2X

#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__) || defined(__AVR_ATmega162__) \
	|| defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
/* ATMega with two USART, use UART0 */
  #define UART_BAUD_RATE_LOW			UBRR0L
  #define UART_STATUS_REG				UCSR0A
  #define UART_CONTROL_REG			UCSR0B
  #define UART_ENABLE_TRANSMITTER		TXEN0
  #define UART_ENABLE_RECEIVER		RXEN0
  #define UART_TRANSMIT_COMPLETE		TXC0
  #define UART_RECEIVE_COMPLETE		RXC0
  #define UART_DATA_REG				UDR0
  #define UART_DOUBLE_SPEED			U2X0

#elif defined(UBRR0L) && defined(UCSR0A) && defined(TXEN0)
/* ATMega with two USART, use UART0 */
  #define UART_BAUD_RATE_LOW			UBRR0L
  #define UART_STATUS_REG				UCSR0A
  #define UART_CONTROL_REG			UCSR0B
  #define UART_ENABLE_TRANSMITTER		TXEN0
  #define UART_ENABLE_RECEIVER		RXEN0
  #define UART_TRANSMIT_COMPLETE		TXC0
  #define UART_RECEIVE_COMPLETE		RXC0
  #define UART_DATA_REG				UDR0
  #define UART_DOUBLE_SPEED			U2X0
  #elif defined(UBRRL) && defined(UCSRA) && defined(UCSRB) && defined(TXEN) && defined(RXEN)
//* catch all
  #define UART_BAUD_RATE_LOW			UBRRL
  #define UART_STATUS_REG				UCSRA
  #define UART_CONTROL_REG			UCSRB
  #define UART_ENABLE_TRANSMITTER		TXEN
  #define UART_ENABLE_RECEIVER		RXEN
  #define UART_TRANSMIT_COMPLETE		TXC
  #define UART_RECEIVE_COMPLETE		RXC
  #define UART_DATA_REG				UDR
  #define UART_DOUBLE_SPEED			U2X

#else
  #error "no UART definition for MCU available"
#endif

/*
 * Macro to calculate UBBR from XTAL and baudrate
 */
#if defined(__AVR_ATmega32__) && UART_BAUDRATE_DOUBLE_SPEED
  #define UART_BAUD_SELECT(baudRate, xtalCpu) ((xtalCpu / 4 / baudRate - 1) / 2)
#elif defined(__AVR_ATmega32__)
  #define UART_BAUD_SELECT(baudRate, xtalCpu) ((xtalCpu / 8 / baudRate - 1) / 2)
#elif UART_BAUDRATE_DOUBLE_SPEED
  #define UART_BAUD_SELECT(baudRate, xtalCpu) (((float)(xtalCpu)) / (((float)(baudRate)) * 8.0) - 1.0 + 0.5)
#else
  #define UART_BAUD_SELECT(baudRate, xtalCpu) (((float)(xtalCpu)) / (((float)(baudRate)) * 16.0) - 1.0 + 0.5)
#endif

/*
 * use 16bit address variable for ATmegas with <= 64K flash
 */
#if defined(RAMPZ)
typedef uint32_t address_t;
#else
typedef uint16_t address_t;
#endif

/*
 * function prototypes
 */
static void sendchar(char c);
static unsigned char recchar(void);

/*
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain(void) __attribute__ ((naked)) __attribute__ ((section(".init9")));
#include <avr/sfr_defs.h>

//#define	SPH_REG	0x3E
//#define	SPL_REG	0x3D

//*****************************************************************************
void __jumpMain(void)
{
	//*	July 17, 2010	<MLS> Added stack pointer initialzation
	//*	the first line did not do the job on the ATmega128

	asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );

	//*	set stack pointer to top of RAM

	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );

	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );

	asm volatile ( "clr __zero_reg__" );					// GCC depends on register r1 set to 0
	asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );	// set SREG to 0
	asm volatile ( "jmp main");						// jump to main()
}


//*****************************************************************************
void delay_ms(unsigned int timedelay)
{
	unsigned int i;

	for (i = 0;	i < timedelay;	i++) {
		_delay_ms(0.5);
	}
}


//*****************************************************************************
/*
 * send single byte to USART, wait until transmission is completed
 */
static void sendchar(char c)
{
	UART_DATA_REG	=	c;					// prepare transmission
	while (!(UART_STATUS_REG & (1 << UART_TRANSMIT_COMPLETE))) ;	// wait until byte sent
	UART_STATUS_REG |= (1 << UART_TRANSMIT_COMPLETE);		// delete TXCflag
}


//************************************************************************
static int	Serial_Available(void)
{
	return(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE));	// wait for data
}


//*****************************************************************************
/*
 * Read single byte from USART, block if no data available
 */
static unsigned char recchar(void)
{
	while (!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE))) {
		// wait for data
	}
	return UART_DATA_REG;
}


//*	for watch dog timer startup
void (*app_start)(void) = 0x0000;


//*****************************************************************************
int main(void)
{
	//*	some chips dont set the stack properly
	asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );

#ifdef _FIX_ISSUE_181_
	//************************************************************************
	//*	Dec 29,	2011	<MLS> Issue #181, added watch dog timmer support
	//*	handle the watch dog timer
	uint8_t mcuStatusReg;
	mcuStatusReg	=	MCUSR;

	__asm__ __volatile__ ("cli");
	__asm__ __volatile__ ("wdr");
	MCUSR	=	0;
	WDTCSR	|=	_BV(WDCE) | _BV(WDE);
	WDTCSR	=	0;
	__asm__ __volatile__ ("sei");
	// check if WDT generated the reset, if so, go straight to app
	if (mcuStatusReg & _BV(WDRF)) {
		app_start();
	}
	//************************************************************************
#endif

	/*
	* Init UART
	* set baudrate and enable USART receiver and transmiter without interrupts
	*/
#if UART_BAUDRATE_DOUBLE_SPEED
	UART_STATUS_REG		|=	(1 << UART_DOUBLE_SPEED);
#endif
	UART_BAUD_RATE_LOW	=	UART_BAUD_SELECT(BAUDRATE, F_CPU);
	UART_CONTROL_REG	=	(1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);

	asm volatile ("nop");					// wait until port has changed

	RunMonitor();

	asm volatile ("nop");					// wait until port has changed

	/*
	* Now leave bootloader
	*/

	UART_STATUS_REG &=	0xfd;
	boot_rww_enable();					// enable application section


	asm volatile (
		"clr	r30		\n\t"
		"clr	r31		\n\t"
		"ijmp	\n\t"
		);
	//	asm volatile ( "push r1" "\n\t"		// Jump to Reset vector in Application Section
	//					"push r1" "\n\t"
	//					"ret"	"\n\t"
	//					::);

	/*
	* Never return to stop GCC to generate exit return code
	* Actually we will never reach this point, but the compiler doesn't
	* understand this
	*/
	for (;;	) ;
}


//************************************************************************
#include	<math.h>

unsigned long gRamIndex;
unsigned long gFlashIndex;
unsigned long gEepromIndex;

#define true	1
#define false	0

#include	"avr_cpunames.h"

#ifndef _AVR_CPU_NAME_
 #error cpu name not defined
#endif

#ifdef _VECTORS_SIZE
#define kInterruptVectorCount (_VECTORS_SIZE / 4)
#else
#define kInterruptVectorCount 23
#endif


void	PrintDecInt(int theNumber, int digitCnt);

#ifdef _AVR_CPU_NAME_
const char gTextMsg_CPU_Name[]			PROGMEM =	_AVR_CPU_NAME_;
#else
const char gTextMsg_CPU_Name[]			PROGMEM =	"UNKNOWN";
#endif

const char gTextMsg_Explorer[]			PROGMEM =	"Arduino explorer stk500V2 by MLS";
const char gTextMsg_Prompt[]			PROGMEM =	"Bootloader>";
const char gTextMsg_HUH[]				PROGMEM =	"Huh?";
const char gTextMsg_COMPILED_ON[]		PROGMEM =	"Compiled on = ";
const char gTextMsg_CPU_Type[]			PROGMEM =	"CPU Type	= ";
const char gTextMsg_AVR_ARCH[]			PROGMEM =	"__AVR_ARCH__= ";
const char gTextMsg_AVR_LIBC[]			PROGMEM =	"AVR LibC Ver= ";
const char gTextMsg_GCC_VERSION[]		PROGMEM =	"GCC Version = ";
const char gTextMsg_CPU_SIGNATURE[]		PROGMEM =	"CPU ID	= ";
const char gTextMsg_FUSE_BYTE_LOW[]		PROGMEM =	"Low fuse	= ";
const char gTextMsg_FUSE_BYTE_HIGH[]	PROGMEM =	"High fuse	= ";
const char gTextMsg_FUSE_BYTE_EXT[]		PROGMEM =	"Ext fuse	= ";
const char gTextMsg_FUSE_BYTE_LOCK[]	PROGMEM =	"Lock fuse	= ";
const char gTextMsg_GCC_DATE_STR[]		PROGMEM =	__DATE__;
const char gTextMsg_AVR_LIBC_VER_STR[]	PROGMEM =	__AVR_LIBC_VERSION_STRING__;
const char gTextMsg_GCC_VERSION_STR[]	PROGMEM =	__VERSION__;
const char gTextMsg_VECTOR_HEADER[]		PROGMEM =	"V#	ADDR	op code	instruction addr   Interrupt";
const char gTextMsg_noVector[]			PROGMEM =	"no vector";
const char gTextMsg_rjmp[]				PROGMEM =	"rjmp  ";
const char gTextMsg_jmp[]				PROGMEM =	"jmp ";
const char gTextMsg_WHAT_PORT[]			PROGMEM =	"What port:";
const char gTextMsg_PortNotSupported[]	PROGMEM =	"Port not supported";
const char gTextMsg_MustBeLetter[]		PROGMEM =	"Must be a letter";
const char gTextMsg_SPACE[]				PROGMEM =	" ";
const char gTextMsg_WriteToEEprom[]		PROGMEM =	"Writting EE";
const char gTextMsg_ReadingEEprom[]		PROGMEM =	"Reading EE";
const char gTextMsg_EEPROMerrorCnt[]	PROGMEM =	"EE err cnt=";
const char gTextMsg_PORT[]				PROGMEM =	"PORT";


//************************************************************************
//*	Help messages
const char gTextMsg_HELP_MSG_0[]	PROGMEM =	"0 = Zero addr";
const char gTextMsg_HELP_MSG_QM[]	PROGMEM =	"? = CPU stats";
const char gTextMsg_HELP_MSG_AT[]	PROGMEM =	"@ = EEPROM test";
//const char gTextMsg_HELP_MSG_B[]	PROGMEM =	"B = Blink LED";
const char gTextMsg_HELP_MSG_E[]	PROGMEM =	"E = Dump EEPROM";
const char gTextMsg_HELP_MSG_F[]	PROGMEM =	"F = Dump FLASH";
const char gTextMsg_HELP_MSG_H[]	PROGMEM =	"H = Help";
const char gTextMsg_HELP_MSG_L[]	PROGMEM =	"L = List I/O Ports";
//const char gTextMsg_HELP_MSG_Q[]	PROGMEM	=	"Q = Quit & jump to user pgm";
const char gTextMsg_HELP_MSG_Q[]	PROGMEM =	"Q = Quit";
const char gTextMsg_HELP_MSG_R[]	PROGMEM =	"R = Dump RAM";
const char gTextMsg_HELP_MSG_V[]	PROGMEM =	"V = show interrupt Vectors";
const char gTextMsg_HELP_MSG_Y[]	PROGMEM =	"Y = Port blink";

const char gTextMsg_END[]			PROGMEM =	"*";


//************************************************************************
void	PrintFromPROGMEM(const void *dataPtr, unsigned char offset)
{
	char theChar;

	dataPtr		+=	offset;

	do {
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#if (FLASHEND > 0x10000)
		theChar =	pgm_read_byte_far((uint16_t)dataPtr++);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#else
		theChar =	pgm_read_byte_near((uint16_t)dataPtr++);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif
		if (theChar != 0) {
			sendchar(theChar);
		}
	} while (theChar != 0);
}

//************************************************************************
void	PrintNewLine(void)
{
	sendchar(0x0d);
	sendchar(0x0a);
}


//************************************************************************
void	PrintFromPROGMEMln(const void *dataPtr, unsigned char offset)
{
	PrintFromPROGMEM(dataPtr, offset);

	PrintNewLine();
}


//************************************************************************
void	PrintString(char *textString)
{
	char theChar;
	int ii;

	theChar		=	1;
	ii			=	0;
	while (theChar != 0) {
		theChar =	textString[ii];
		if (theChar != 0) {
			sendchar(theChar);
		}
		ii++;
	}
}

//************************************************************************
void	PrintHexByte(unsigned char theByte)
{
	char theChar;

	theChar =	0x30 + ((theByte >> 4) & 0x0f);
	if (theChar > 0x39) {
		theChar +=	7;
	}
	sendchar(theChar );

	theChar =	0x30 + (theByte & 0x0f);
	if (theChar > 0x39) {
		theChar +=	7;
	}
	sendchar(theChar );
}

//************************************************************************
void	PrintDecInt(int theNumber, int digitCnt)
{
	int theChar;
	int myNumber;

	myNumber	=	theNumber;

	if ((myNumber > 100) || (digitCnt >= 3)) {
		theChar		=	0x30 + myNumber / 100;
		sendchar(theChar );
	}

	if ((myNumber > 10) || (digitCnt >= 2)) {
		theChar =	0x30  + ((myNumber % 100) / 10 );
		sendchar(theChar );
	}
	theChar =	0x30 + (myNumber % 10);
	sendchar(theChar );
}

//************************************************************************
static void	PrintCPUstats(void)
{
	unsigned char fuseByte;

	PrintFromPROGMEMln(gTextMsg_Explorer, 0);

	PrintFromPROGMEM(gTextMsg_COMPILED_ON, 0);
	PrintFromPROGMEMln(gTextMsg_GCC_DATE_STR, 0);

	PrintFromPROGMEM(gTextMsg_CPU_Type, 0);
	PrintFromPROGMEMln(gTextMsg_CPU_Name, 0);

	PrintFromPROGMEM(gTextMsg_AVR_ARCH, 0);
	PrintDecInt(__AVR_ARCH__, 1);
	PrintNewLine();

	PrintFromPROGMEM(gTextMsg_GCC_VERSION, 0);
	PrintFromPROGMEMln(gTextMsg_GCC_VERSION_STR, 0);

	//*	these can be found in avr/version.h
	PrintFromPROGMEM(gTextMsg_AVR_LIBC, 0);
	PrintFromPROGMEMln(gTextMsg_AVR_LIBC_VER_STR, 0);

#if defined(SIGNATURE_0)
	PrintFromPROGMEM(gTextMsg_CPU_SIGNATURE, 0);
	//*	these can be found in avr/iomxxx.h
	PrintHexByte(SIGNATURE_0);
	PrintHexByte(SIGNATURE_1);
	PrintHexByte(SIGNATURE_2);
	PrintNewLine();
#endif


#if defined(GET_LOW_FUSE_BITS)
	//*	fuse settings
	PrintFromPROGMEM(gTextMsg_FUSE_BYTE_LOW, 0);
	fuseByte	=	boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
	PrintHexByte(fuseByte);
	PrintNewLine();

	PrintFromPROGMEM(gTextMsg_FUSE_BYTE_HIGH, 0);
	fuseByte	=	boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
	PrintHexByte(fuseByte);
	PrintNewLine();

	PrintFromPROGMEM(gTextMsg_FUSE_BYTE_EXT, 0);
	fuseByte	=	boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
	PrintHexByte(fuseByte);
	PrintNewLine();

	PrintFromPROGMEM(gTextMsg_FUSE_BYTE_LOCK, 0);
	fuseByte	=	boot_lock_fuse_bits_get(GET_LOCK_BITS);
	PrintHexByte(fuseByte);
	PrintNewLine();

#endif

}

enum {
	kDUMP_FLASH	=	0,
	kDUMP_EEPROM,
	kDUMP_RAM
};

//************************************************************************
static void	DumpHex(unsigned char dumpWhat, unsigned long startAddress, unsigned char numRows)
{
	unsigned long myAddressPointer;
	uint8_t ii;
	unsigned char theValue;
	char asciiDump[18];
	unsigned char	*ramPtr;


	ramPtr				=	0;
	theValue			=	0;
	myAddressPointer	=	startAddress;
	while (numRows > 0) {
		if (myAddressPointer > 0x10000) {
			PrintHexByte((myAddressPointer >> 16) & 0x00ff);
		}
		PrintHexByte((myAddressPointer >> 8) & 0x00ff);
		PrintHexByte(myAddressPointer & 0x00ff);
		sendchar(0x20);
		sendchar('-');
		sendchar(0x20);

		asciiDump[0]		=	0;
		for (ii = 0;	ii < 16;	ii++) {
			switch (dumpWhat) {
			case kDUMP_FLASH:
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#if (FLASHEND > 0x10000)
				theValue	=	pgm_read_byte_far(myAddressPointer);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#else
				theValue	=	pgm_read_byte_near(myAddressPointer);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif
				break;

			case kDUMP_EEPROM:
				theValue	=	eeprom_read_byte((uint8_t*)(uint16_t)myAddressPointer);
				break;

			case kDUMP_RAM:
				theValue	=	ramPtr[myAddressPointer];
				break;

			}
			PrintHexByte(theValue);
			sendchar(0x20);
			if ((theValue >= 0x20) && (theValue < 0x7f)) {
				asciiDump[ii % 16]	=	theValue;
			}
			else{
				asciiDump[ii % 16]	=	'.';
			}

			myAddressPointer++;
		}
		asciiDump[16]	=	0;
		PrintString(asciiDump);
		PrintNewLine();

		numRows--;
	}
}

//************************************************************************
//*	returns amount of extended memory
static void	EEPROMtest(void)
{
	int ii;
	char theChar;
	char theEEPROMchar;
	int errorCount;

	PrintFromPROGMEMln(gTextMsg_WriteToEEprom, 0);
	PrintNewLine();
	ii			=	0;
#if (FLASHEND > 0x10000)
	while (((theChar = pgm_read_byte_far(((uint16_t)gTextMsg_Explorer) + ii)) != '*') && (ii < 512))
#else
	while (((theChar = pgm_read_byte_near(((uint16_t)gTextMsg_Explorer) + ii)) != '*') && (ii < 512))
#endif
	{
		eeprom_write_byte((uint8_t*)ii, theChar);
		if (theChar == 0) {
			PrintFromPROGMEM(gTextMsg_SPACE, 0);
		}
		else{
			sendchar(theChar);
		}
		ii++;
	}

	//*	no go back through and test
	PrintNewLine();
	PrintNewLine();
	PrintFromPROGMEMln(gTextMsg_ReadingEEprom, 0);
	PrintNewLine();
	errorCount	=	0;
	ii			=	0;
#if (FLASHEND > 0x10000)
	while (((theChar = pgm_read_byte_far((uint16_t)gTextMsg_Explorer + ii)) != '*') && (ii < 512))
#else
	while (((theChar = pgm_read_byte_near((uint16_t)gTextMsg_Explorer + ii)) != '*') && (ii < 512))
#endif
	{
		theEEPROMchar	=	eeprom_read_byte((uint8_t*)ii);
		if (theEEPROMchar == 0) {
			PrintFromPROGMEM(gTextMsg_SPACE, 0);
		}
		else{
			sendchar(theEEPROMchar);
		}
		if (theEEPROMchar != theChar) {
			errorCount++;
		}
		ii++;
	}
	PrintNewLine();
	PrintNewLine();
	PrintFromPROGMEM(gTextMsg_EEPROMerrorCnt, 0);
	PrintDecInt(errorCount, 1);
	PrintNewLine();
	PrintNewLine();

	gEepromIndex	=	0;				//*	set index back to zero for next eeprom dump

}



//************************************************************************
static void	VectorDisplay(void)
{
	unsigned long byte1;
	unsigned long byte2;
	unsigned long byte3;
	unsigned long byte4;
	unsigned long word1;
	unsigned long word2;
	int vectorIndex;
	unsigned long myMemoryPtr;
	unsigned long wordMemoryAddress;
	unsigned long realitiveAddr;
	unsigned long myFullAddress;
	unsigned long absoluteAddr;

#if defined(_INTERRUPT_NAMES_DEFINED_)
	long stringPointer;
#endif

	myMemoryPtr		=	0;
	vectorIndex		=	0;
	PrintFromPROGMEMln(gTextMsg_CPU_Name, 0);
	PrintFromPROGMEMln(gTextMsg_VECTOR_HEADER, 0);
	//					V#   ADDR   op code
	//					1 - 0000 = C3 BB 00 00 rjmp 03BB >000776 RESET
	while (vectorIndex < kInterruptVectorCount) {
		wordMemoryAddress	=	myMemoryPtr / 2;
		//					01 - 0000 = 12 34
		PrintDecInt(vectorIndex + 1, 2);
		sendchar(0x20);
		sendchar('-');
		sendchar(0x20);
		PrintHexByte((wordMemoryAddress >> 8) & 0x00ff);
		PrintHexByte((wordMemoryAddress) & 0x00ff);
		sendchar(0x20);
		sendchar('=');
		sendchar(0x20);


		//*	the AVR is LITTLE ENDIAN, swap the byte order
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#if (FLASHEND > 0x10000)
		byte1	=	pgm_read_byte_far(myMemoryPtr++);
		byte2	=	pgm_read_byte_far(myMemoryPtr++);
		byte3	=	pgm_read_byte_far(myMemoryPtr++);
		byte4	=	pgm_read_byte_far(myMemoryPtr++);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#else
		byte1	=	pgm_read_byte_near(myMemoryPtr++);
		byte2	=	pgm_read_byte_near(myMemoryPtr++);
		byte3	=	pgm_read_byte_near(myMemoryPtr++);
		byte4	=	pgm_read_byte_near(myMemoryPtr++);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif
		word1	=	(byte2 << 8) + byte1;
		word2	=	(byte4 << 8) + byte3;


		PrintHexByte(byte2);
		sendchar(0x20);
		PrintHexByte(byte1);
		sendchar(0x20);
		PrintHexByte(byte4);
		sendchar(0x20);
		PrintHexByte(byte3);
		sendchar(0x20);

		if (word1 == 0xffff) {
			PrintFromPROGMEM(gTextMsg_noVector, 0);
		}
		else if ((word1 & 0xc000) == 0xc000) {
			//*	rjmp instruction
			realitiveAddr	=	word1 & 0x3FFF;
			absoluteAddr	=	wordMemoryAddress + realitiveAddr;	 //*	add the offset to the current address
			absoluteAddr	=	absoluteAddr << 1;		 //*	multiply by 2 for byte address

			PrintFromPROGMEM(gTextMsg_rjmp, 0);
			PrintHexByte((realitiveAddr >> 8) & 0x00ff);
			PrintHexByte((realitiveAddr) & 0x00ff);
			sendchar(0x20);
			sendchar('>');
			PrintHexByte((absoluteAddr >> 16) & 0x00ff);
			PrintHexByte((absoluteAddr >> 8) & 0x00ff);
			PrintHexByte((absoluteAddr) & 0x00ff);

		}
		else if ((word1 & 0xfE0E) == 0x940c) {
			//*	jmp instruction, this is REALLY complicated, refer to the instruction manual (JMP)
			myFullAddress	=	((byte1 & 0x01) << 16) +
							((byte1 & 0xf0) << 17) +
							((byte2 & 0x01) << 21) +
							word2;

			absoluteAddr	=	myFullAddress << 1;

			PrintFromPROGMEM(gTextMsg_jmp, 0);
			PrintHexByte((myFullAddress >> 16) & 0x00ff);
			PrintHexByte((myFullAddress >> 8) & 0x00ff);
			PrintHexByte((myFullAddress) & 0x00ff);
			sendchar(0x20);
			sendchar('>');
			PrintHexByte((absoluteAddr >> 16) & 0x00ff);
			PrintHexByte((absoluteAddr >> 8) & 0x00ff);
			PrintHexByte((absoluteAddr) & 0x00ff);
		}

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#if defined(_INTERRUPT_NAMES_DEFINED_)
		sendchar(0x20);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#if (FLASHEND > 0x10000)
		stringPointer	=	pgm_read_word_far(&(gInterruptNameTable[vectorIndex]));
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#else
		stringPointer	=	pgm_read_word_near(&(gInterruptNameTable[vectorIndex]));
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif
		PrintFromPROGMEM((char*)stringPointer, 0);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif
		PrintNewLine();

		vectorIndex++;
	}
}

//************************************************************************
static void	PrintAvailablePort(char thePortLetter)
{
	PrintFromPROGMEM(gTextMsg_PORT, 0);
	sendchar(thePortLetter);
	PrintNewLine();
}

//************************************************************************
static void	ListAvailablePorts(void)
{

#ifdef DDRA
	PrintAvailablePort('A');
#endif

#ifdef DDRB
	PrintAvailablePort('B');
#endif

#ifdef DDRC
	PrintAvailablePort('C');
#endif

#ifdef DDRD
	PrintAvailablePort('D');
#endif

#ifdef DDRE
	PrintAvailablePort('E');
#endif

#ifdef DDRF
	PrintAvailablePort('F');
#endif

#ifdef DDRG
	PrintAvailablePort('G');
#endif

#ifdef DDRH
	PrintAvailablePort('H');
#endif

#ifdef DDRI
	PrintAvailablePort('I');
#endif

#ifdef DDRJ
	PrintAvailablePort('J');
#endif

#ifdef DDRK
	PrintAvailablePort('K');
#endif

#ifdef DDRL
	PrintAvailablePort('L');
#endif

}

//************************************************************************
static void	AVR_PortOutput(void)
{
	char portLetter;
	char getCharFlag;

	PrintFromPROGMEM(gTextMsg_WHAT_PORT, 0);

	portLetter	=	recchar();
	portLetter	=	portLetter & 0x5f;
	sendchar(portLetter);
	PrintNewLine();

	if ((portLetter >= 'A') && (portLetter <= 'Z')) {
		getCharFlag	=	true;
		switch (portLetter) {
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRA
		case 'A':
			DDRA	=	0xff;
			while (!Serial_Available()) {
				PORTA	^=	0xff;
				delay_ms(200);
			}
			PORTA	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRB
		case 'B':
			DDRB	=	0xff;
			while (!Serial_Available()) {
				PORTB	^=	0xff;
				delay_ms(200);
			}
			PORTB	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRC
		case 'C':
			DDRC	=	0xff;
			while (!Serial_Available()) {
				PORTC	^=	0xff;
				delay_ms(200);
			}
			PORTC	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRD
		case 'D':
			DDRD	=	0xff;
			while (!Serial_Available()) {
				PORTD	^=	0xff;
				delay_ms(200);
			}
			PORTD	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRE
		case 'E':
			DDRE	=	0xff;
			while (!Serial_Available()) {
				PORTE	^=	0xff;
				delay_ms(200);
			}
			PORTE	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRF
		case 'F':
			DDRF	=	0xff;
			while (!Serial_Available()) {
				PORTF	^=	0xff;
				delay_ms(200);
			}
			PORTF	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRG
		case 'G':
			DDRG	=	0xff;
			while (!Serial_Available()) {
				PORTG	^=	0xff;
				delay_ms(200);
			}
			PORTG	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRH
		case 'H':
			DDRH	=	0xff;
			while (!Serial_Available()) {
				PORTH	^=	0xff;
				delay_ms(200);
			}
			PORTH	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRI
		case 'I':
			DDRI	=	0xff;
			while (!Serial_Available()) {
				PORTI	^=	0xff;
				delay_ms(200);
			}
			PORTI	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRJ
		case 'J':
			DDRJ	=	0xff;
			while (!Serial_Available()) {
				PORTJ	^=	0xff;
				delay_ms(200);
			}
			PORTJ	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRK
		case 'K':
			DDRK	=	0xff;
			while (!Serial_Available()) {
				PORTK	^=	0xff;
				delay_ms(200);
			}
			PORTK	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#ifdef DDRL
		case 'L':
			DDRL	=	0xff;
			while (!Serial_Available()) {
				PORTL	^=	0xff;
				delay_ms(200);
			}
			PORTL	=	0;
			break;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																#endif

		default:
			PrintFromPROGMEMln(gTextMsg_PortNotSupported, 0);
			getCharFlag	=	false;
			break;
		}
		if (getCharFlag) {
			recchar();
		}
	}
	else{
		PrintFromPROGMEMln(gTextMsg_MustBeLetter, 0);
	}
}


//*******************************************************************
static void PrintHelp(void)
{
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_0, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_QM, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_AT, 0);
	//	PrintFromPROGMEMln(gTextMsg_HELP_MSG_B, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_E, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_F, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_H, 0);

	PrintFromPROGMEMln(gTextMsg_HELP_MSG_L, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_Q, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_R, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_V, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_Y, 0);
}

//************************************************************************
static void	RunMonitor(void)
{
	char keepGoing;
	unsigned char theChar;
	int ii, jj;

/*
	for (ii = 0;	ii < 5;	ii++) {
		for (jj = 0;	jj < 25;	jj++) {
			sendchar('!');
		}
		PrintNewLine();
	}
*/

	gRamIndex			=	0;
	gFlashIndex			=	0;
	gEepromIndex		=	0;

	PrintFromPROGMEMln(gTextMsg_Explorer, 0);

	keepGoing	=	1;
	while (keepGoing) {
		PrintFromPROGMEM(gTextMsg_Prompt, 0);
		theChar =	recchar();
		if (theChar >= 0x60) {
			theChar =	theChar & 0x5F;
		}

		if (theChar >= 0x20) {
			sendchar(theChar);
			sendchar(0x20);
		}

		switch (theChar) {
		case '0':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_0, 2);
			gFlashIndex		=	0;
			gRamIndex		=	0;
			gEepromIndex	=	0;
			break;

		case '?':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_QM, 2);
			PrintCPUstats();
			break;

		case '@':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_AT, 2);
			EEPROMtest();
			break;

		//		case 'B':
		//			PrintFromPROGMEMln(gTextMsg_HELP_MSG_B, 2);
		//			BlinkLED();
		//			break;

		case 'E':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_E, 2);
			DumpHex(kDUMP_EEPROM, gEepromIndex, 16);
			gEepromIndex	+=	256;
			if (gEepromIndex > E2END) {
				gEepromIndex	=	0;
			}
			break;

		case 'F':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_F, 2);
			DumpHex(kDUMP_FLASH, gFlashIndex, 16);
			gFlashIndex	+=	256;
			break;

		case 'H':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_H, 2);
			PrintHelp();
			break;

		case 'L':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_L, 2);
			ListAvailablePorts();
			break;

		case 'Q':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_Q, 2);
			keepGoing	=	false;
			break;

		case 'R':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_R, 2);
			DumpHex(kDUMP_RAM, gRamIndex, 16);
			gRamIndex	+=	256;
			break;

		case 'V':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_V, 2);
			VectorDisplay();
			break;

		case 'Y':
			PrintFromPROGMEMln(gTextMsg_HELP_MSG_Y, 2);
			AVR_PortOutput();
			break;

		default:
			PrintFromPROGMEMln(gTextMsg_HUH, 0);
			break;
		}
	}
}
