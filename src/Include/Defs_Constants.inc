; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;                       AttoBASIC V2.34
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Copyright (C)2011-2017 Kenneth Scott Vitale <ksv_prj@gmx.com>
; Note: All code is maintained by Kenneth Scott Vitale since 2011
;
; All Versions of AttoBASIC from V2.00 forward, including ATtiny84,
;	ATtiny85, ATmega16(A)/32(A), ATmega32U4, ATmega88/168/328,
;	AT90USB1286, ATmega644P/1284P and Mega2560 ports and merging of
;   original AttoBASIC code for AT90S2313, AT90S8515 and ATmega163 are
;	Copyright (C)2011-2017 by: K. Scott Vitale, Florida, uSA
;			email: ksv_prj@gmx.com
;	Plese let me know of any bugs or (especially) improvements.
;		Thank you - KSV
; Original AttoBASIC code for AT90S2313, AT90S8515 and ATmega163
;	Copyright 2003 Richard Cappels projects@cappels.org
;	http://projects.cappels.org
;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;  NOTICE:
;	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;   "AS IS"AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
;   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
;   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;	POSSIBILITY OF SUCH DAMAGE.
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;	This source code may be used for personal use only.
;	Commercial License is available.
;	This source code may be republished provided this notice and all
;	  support files are kept intact.
; ~~~~~~~~~~~~~~~~~~~~~~~~~ End of Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;#if defined(__ATmega88__) || defined(__ATmega88PA__)
;#elif defined(__ATmega168__) || defined(__ATmega168PA__)
;#elif defined(__ATmega328__) || defined(__ATmega328P__)
;#elif defined(__ATmega32U4__)
;#elif defined(__AT90USB1286__)
;#elif defined(__ATmega2560__)
;#endif
;
.set	f_clk		= FCLK				;set CPU clock frequency
;
;------------------------------------------------------------------
.include "Include/Defs_Register.inc"	;registers defined
.include "Include/Defs_Pinout.inc"		;pinouts defined
.include "Include/Defs_Timers.inc"		;Timer specific dividers defined
.include "Include/Defs_ADC.inc"			;ADC and ACO constants
;
;~~~~~ Code enabling feature ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Code enabling feature.  Select "1" to enable code generation,
;	"0" to disable.  Note: the size difference is processor dependant.
#define	BIGRAM	  1		;"1" to allow more program RAM (+0 bytes)
.set	DATAREC	= 0		;DON'T MODIFY THIS! (needed to process correct bootloader)
.set	DEBUG	= 1		;"1" to enable [EV]DUMP routines (+562 bytes)
.set	CFG		= 1		;"1" to enable non-volatile config (+110 bytes)
.set	DDS		= 1		;"1" to enable DDS routines (+740 bytes)
.set	DSI		= 1		;"1" to enable DSI routines (+274 bytes)
.set	PWM		= 1		;"1" to enable PWM routines (+286 bytes)
.set	SPI		= 1		;"1" to enable SPI routines (+518 or 364 bytes)
.set	DFR		= 1		;"1" to enable DataFile routines (+576 bytes)
.set	SPGM	= 0		;"1" to enable FLASH data store/fetch routines (+590/728/798 bytes)
.set	TWI		= 1		;"1" to enable TWI routines (+498 bytes)
.set	OWR		= 1		;"1" to enable 1-Wire routines (+302 bytes)
.set	ICP		= 1		;"1" to enable ICP routines (+316 bytes)
.set	EFS		= 1		;"1" to enable E2P File System routines (+840 bytes)
.set	LPMD	= 1		;"1" to enable Low-Power routines (+310 bytes)
.set	DHT		= 1		;"1" to enable DHTxx routines (+962 bytes)
.set	RTC		= 1		;"1" to enable Real-time Counter routines (+550 bytes)
.set	NRF		= 1		;"1" to enable nRF24L01 routines (+1324 bytes)
.set	PIO		= 1		;"1" to enable Port I/O routines (+746 or 274 bytes)
.set	SCLK	= 1		;"1" to enable System Clock Prescaler routines (+xxx bytes)
.set	ADConv	= 1		;"1" to enable ADConv routines (+128 bytes)
.set	AComp	= 1		;"1" to enable Analog Comp. routines (+74 bytes)
.set	INTREG	= 1		;"1" to enable internal register routines (+82 bytes)
.set	XTRAOPS	= 1		;"1" to enable additional operators (+76 bytes)
.set	CRC16R	= 1		;"1" to enable internal CCITT CRC16 routines (+68 bytes)
.set	HELP	= 1		;"1" to enable HELP command (+146 bytes)
.set	RENUMBR	= 0		;"1" to enable the Renumber routine (future release)
.if CFG
 .set	SStst	= 0		;disble self-start test if E2P CONFIG
.else
 .set	SStst	= 1		;enable self-start test at RESET (+40 bytes)
.endif
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
.set	TESTING		= 0		;
.set	USBSUPP		= 0		;for AVRASM2 conditionals usage
.set	USISUPP		= 0		;for AVRASM2 conditionals usage
.set	UARTSUPP	= 0		;for AVRASM2 conditionals usage

.set	AVRhasMUL	= 1		;set for cores with MUL instructions
;
#if TEST
  #message !! Test code enabled !!
  .set		TESTING	= 	1			;used for preprocessor and ASM compat.
#endif
;
;~~~~~~~ MCU Specific Restrictions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifdef USI
 #undef USI
 #define USI	0	;start with no USI on all parts
#endif

#if ( defined(__ATmega88__) || defined(__ATmega88PA__) || \
	defined(__ATmega168__) || defined(__ATmega168PA__) || \
	defined(__ATmega328__) || defined(__ATmega328P__) || \
	defined(__ATmega2560__) )
#ifdef USB
 #undef USB
 #define USB	0	;no USB allowed on Mega88/168/328/2560
#endif
#endif
;
;=========================================================================
#if ( defined(__ATmega16__) || defined(__ATmega16L__) || \
	  defined(__ATmega32__) || defined(__ATmega32A__) )

#define	BIGRAM	  1		;BIGRAM (+0 bytes)
.set	DATAREC	= 0		;ALWAYS disable DATARECORDER routines for Mega16/32
.set	DEBUG	= 1		;DEBUG routines (+622 bytes)
.set	CFG		= 1		;CFG routines (+60 bytes)
.set	DDS		= 0		;DDS routines (+636 bytes)
.set	PWM		= 1		;PWM routines (+48 bytes)
.set	SPI		= 1		;SPI routines (+346 bytes)
.set	DFR		= 1		;DataFile routines (+426 bytes)
.set	SPGM	= 1		;FLASH data store/fetch routines (+534 bytes)
.set	TWI		= 1		;TWI routines (+488 bytes)
.set	OWR		= 1		;1-Wire routines (+380 bytes)
.set	ICP		= 1		;ICP routines (+238 bytes)
.set	EFS		= 1		;E2P routines (+854 bytes)
.set	RTC		= 0		;RTC routines (+480 bytes)
.set	NRF		= 1		;nRF24L01 routines (+976 bytes)
.set	PIO		= 1		;Port I/O routines (+584 bytes)
.set	ADConv	= 1		;ADConv routines (+102 bytes)
.set	AComp	= 1		;Analog Comp. routines (+74 bytes)
.set	SCLK	= 1		;System Clock Prescaler routines (+46 bytes)
.set	INTREG	= 1		;internal register (+162)
.set	XTRAOPS	= 1		;Additional operators (+xx bytes)
.set	CRC16R	= 1		;CRC16 routines (+62 bytes)
.set	HELP	= 1		;HELP command (+142 bytes)
.set	RENUMBR	= 0		;Renumber routine
.if CFG
 .set	SStst	= 0		;disble self-start test if E2P CONFIG
.else
 .set	SStst	= 1		;enable self-start test at RESET (+46 bytes)
.endif

;!! NOT SUPPORTED !! ATmega16/32 do not support the enhanced
;  watchdog timer, which these modules require.
.set	LPMD	= 0		;Low-Power routines (+xxx bytes)
.set	DHT		= 0		;DHT routines (+xxx bytes)
.set	DSI		= 0		;DSI routines (+xxx bytes)
;
#undef	DREC
#define	DREC	  0
#endif
;
;=========================================================================
#if ( defined(__ATmega88__) || defined(__ATmega88PA__) )
#define	BIGRAM	  1		;BIGRAM (+0 bytes)
.set	DATAREC	= 0		;ALWAYS disable DATARECORDER routines for Mega88
.set	DEBUG	= 0		;DEBUG routines (+536 bytes)
.set	CFG		= 1		;CFG routines (+36 bytes)
.set	DDS		= 0		;DDS routines (+656 bytes)
.set	DSI		= 0		;DSI routines (+268 bytes)
.set	PWM		= 1		;PWM routines (+162 bytes)
.set	SPI		= 1		;SPI routines (+262 bytes)
.set	DFR		= 0		;DataFile routines (+392 bytes)
.set	SPGM	= 0		;FLASH data store/fetch routines (+534 bytes)
.set	TWI		= 1		;TWI routines (+580 bytes)
.set	OWR		= 0		;1-Wire routines (+334 bytes)
.set	ICP		= 0		;ICP routines (+290 bytes)
.set	EFS		= 0		;E2P routines (+754 bytes)
.set	LPMD	= 1		;Low-Power routines (+180 bytes)
.set	DHT		= 0		;DHT routines (+1324 bytes)
.set	RTC		= 0		;RTC routines (+480 bytes)
.set	NRF		= 0		;nRF24L01 routines (+976 bytes)
.set	PIO		= 1		;Port I/O routines (+468 bytes)
.set	ADConv	= 1		;ADConv routines (+172 bytes)
.set	AComp	= 1		;Analog Comp. routines (+148 bytes)
.set	SCLK	= 0		;System Clock Prescaler routines (+86 bytes)
.set	XTRAOPS	= 1		;Additional operators (+66 bytes)
.set	INTREG	= 0		;internal register (+74)
.set	CRC16R	= 0		;CRC16 routines (+64 bytes)
.set	HELP	= 1		;HELP command (+142 bytes)
.set	RENUMBR	= 0		;Renumber routine
.if CFG
 .set	SStst	= 0		;disble self-start test if E2P CONFIG
.else
 .set	SStst	= 1		;enable self-start test at RESET (+46 bytes)
.endif
#undef	DREC
#define	DREC	  0
;
;=========================================================================
#elif ( defined(__ATmega168__) || defined(__ATmega168PA__) )
.set	DATAREC	= 0		;disable Data Recorder code generation
#undef	DREC
#define	DREC	  0
.set	SPGM	= 0		;disable FLASH data store/fetch routines
.set	DSI		= 0		;disable DSI routines for Mega168
;
;=========================================================================
#elif defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) ; (dis)enable various code modules for Tiny84/85
.set	AVRhasMUL	= 0		;set for cores with MUL instructions

 #if BTLDR	;bootloader has priority over USI
  #undef 	USI
  #define	USI	0	;disable USI UART, use SWUART in bootloader
;#else
; #undef 	USI
; #define	USI	1	;enable USI UART
 #endif

 #ifndef SIO_EINT
  #undef SIO_EINT
  #define	SIO_EINT 0		;set to "1" for RX on INT0
 #endif 					; or "0" for RX on pin-change

#ifdef USB
 #undef 	USB
 #define	USB		0	;no USB SIO
#endif

#ifdef TEENSY
 #undef 	TEENSY
 #define	TEENSY	0	;not a TEENSY
#endif

#ifdef DREC
 #undef 	DREC
 #define	DREC	0	;not a data recorder (yet!)
#endif
;
#ifdef BIGRAM
 #undef BIGRAM
 #define BIGRAM	  0		;no BIGRAM
#endif

#if defined(__ATtiny84__) || \
	defined(__ATtiny84A__) ; (dis)enable various code modules for Tiny84
; Supported on ATtiny84
 #if BTLDR	; if no bootloader then enable the DDS routines
 .set	DDS		= 0		;"1" to enable DDS routines (+662 bytes)
 .set	XTRAOPS	= 1		;"1" to enable additional operators (+66 bytes)
 #else
 .set	DDS		= 1		;"1" to enable DDS routines (+662 bytes)
 .set	XTRAOPS	= 0		;"1" to enable additional operators (+66 bytes)
 #endif

  .set	DEBUG	= 0		;"1" to enable [EV]DUMP routines (+528 bytes)
  .set	HELP	= 0		;"1" to enable HELP command (+142 bytes)
  .set	CFG		= 1		;"1" to enable non-volatile config (+34 bytes)
  .set	PWM		= 1		;"1" to enable PWM routines (+92 bytes) 
  .set	OWR		= 1		;"1" to enable 1-Wire routines (+332 bytes)
  .set	LPMD	= 1		;"1" to enable Low-Power routines (+134 bytes)
  .set	PIO		= 1		;"1" to enable Port I/O routines (+370 bytes)
  .set	ADConv	= 1		;"1" to enable ADConv routines (+172 bytes)
  .set	AComp	= 1		;"1" to enable Analog Comp. routines (+124 bytes)
  .set	SCLK	= 1		;"1" to enable System Clock Prescaler routines (+78 bytes)
  .set	SPGM	= 0		;"1" to enable FLASH data store/fetch routines (+272 bytes)
  .set	INTREG	= 0		;"1" to enable internal register routines [+74 bytes]
  .set	CRC16R	= 0		;"1" to enable internal CCITT CRC16 routines (+64 bytes)
 .if CFG
  .set	SStst	= 0		;disable self-start test if E2P CONFIG
 .else
  .set	SStst	= 1		;enable self-start test at RESET (+36 bytes)
 .endif
;
 #elif defined(__ATtiny85__) ; (dis)enable various code modules for Tiny85
; Supported on ATtiny85
  #if BTLDR	; if no bootloader then enable the DDS routines
  .set	DDS		= 0		;"1" to enable DDS routines (+662 bytes)
  #else
  .set	DDS		= 1		;"1" to enable DDS routines (+662 bytes)
  #endif

  .set	DEBUG	= 0		;"1" to enable [EV]DUMP routines (+528 bytes)
  .set	HELP	= 1		;"1" to enable HELP command (+142 bytes)
  .set	CFG		= 1		;"1" to enable non-volatile config (+24 bytes)
  .set	PWM		= 1		;"1" to enable PWM routines 8-bit only (+176 bytes) 
  .set	OWR		= 1		;"1" to enable 1-Wire routines (+332 bytes)
  .set	LPMD	= 1		;"1" to enable Low-Power routines (+136 bytes)
  .set	PIO		= 1		;"1" to enable Port I/O routines (+194 bytes)
  .set	ADConv	= 1		;"1" to enable ADConv routines (+144 bytes)
  .set	AComp	= 1		;"1" to enable Analog Comp. routines (+110 bytes)
  .set	SCLK	= 1		;"1" to enable System Clock Prescaler routines (+80 bytes)
  .set	SPGM	= 0		;"1" to enable FLASH data store/fetch routines (+272 bytes)
  .set	INTREG	= 0		;"1" to enable internal register routines [+74 bytes]
  .set	XTRAOPS	= 1		;"1" to enable additional operators (+66 bytes)
  .set	CRC16R	= 1		;"1" to enable internal CCITT CRC16 routines (+62 bytes)
  .if CFG
   .set	SStst	= 0		;disble self-start test if E2P CONFIG
  .else
   .set	SStst	= 1		;enable self-start test at RESET (+36 bytes)
  .endif
;
; Not yet supported on tiny84/85
 #endif
  .set	DSI		= 0		;"1" to enable DSI routines [+294 bytes]
  .set	RTC		= 0		;"1" to enable Real-time Counter routines [conflicts w/ SIO]
  .set	ICP		= 0		;"1" to enable ICP routines [not supported]
  .set	SPI		= 0		;"1" to enable SPI routines [not supported]
  .set	EFS		= 0		;"1" to enable E2P File System routines [will never be supported]
  .set	DFR		= 0		;"1" to enable DataFile routines [will never be supported]
  .set	TWI		= 0		;"1" to enable TWI routines [not supported]
  .set	DHT		= 0		;"1" to enable DHTxx routines [not supported]
  .set	NRF		= 0		;"1" to enable nRF24L01 routines [not supported]
  .set	RENUMBR	= 0		;"1" to enable the Renumber routine (future release)
;
;=========================================================================
#elif defined(__AT90USB1286__) | \
	( defined(__ATmega32U4__) | \
	  defined(__ATmega2560__) )
.set	DATAREC	= 0		;disable Data Recorder code generation
#undef	DREC
#define	DREC	  0
;
#endif
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
#if DREC	;enable/disable AVR Data Recorder features
 #undef FCLK
 #define FCLK	  8000000	;set Fclk (preprocessor variable)
 #define BTLDR	  1			;"1" to include boot loader code.
 #ifdef BIGRAM
 #undef BIGRAM
 #endif
 #define BIGRAM	  0			;No BIGRAM
 .set	f_clk	= FCLK		;set CPU clock frequency
 .set	DATAREC	= 1			;needed to process correct bootloader
 .set	DEBUG	= 1			;disable [EV]DUMP routines
 .set	CFG		= 0			;ALWAYS disable non-volatile config
 .set	SStst	= 1			;ALWAYS use self-start pin
 .set	DSI		= 0			;disable DSI routines
 .set	TWI		= 0			;disable routines
 .set	SPI		= 1			;enable SPI routines
 .set	DFR		= 1			;enable DataFile routines
 .set	OWR		= 0			;disble 1-Wire routines
 .set	ICP		= 0			;disable ICP routines
 .set	DDS		= 1			;disable DDS routines
 .set	PWM		= 1			;enable PWM routines
 .set	DHT		= 1			;disable DataFile routines
 .set	LPMD	= 1			;enable Low-Power routines
 .set	RTC		= 1			;enable Real-time Counter routines
 .set	EFS		= 1			;enable E2P File System routines
 .set	PIO		= 1			;enable Port I/O routines
 .set	NRF		= 0			;disble nRF24L01 routines
 .set	INTREG	= 1			;enable internal register routines
 .set	ADConv	= 1			;enable ADConv routines
 .set	AComp	= 1			;enable Analog Comp. routines
 .set	CRC16R	= 0			;disable internal CCITT CRC16 routines
 .set	SStst	= 1			;enable self-start pin test at RESET
;
 .equ	ADCGAIN	= PORTD		;Port D is gain select
 .equ	ADC0_R0	= PORTD2	;PD2 - Channel 0 gain bits
 .equ	ADC0_R1	= PORTD3	;PD3
 .equ	ADC1_R0	= PORTD4	;PD4 - Channel 1 gain bits
 .equ	ADC1_R1	= PORTD5	;PD5
;
 .equ	AD536IO	= PORTC		;Port C is AD536 Output select
 .equ	AD536CH	= PORTC4	;AD536 Channel select bit
 .equ	AD536OS	= PORTC5	;AD536 output select bit

 .equ	DIGPORT	= PORTB		;digital out port
 .equ	DIGOUT	= PORTB0	;digital output pin
 .equ	DIGALF	= 'A'+(DIGPORT/3);alphanumeric character of port

#endif
;
;* Debug Messages ***********************************************
;#if ((FCLK_PS != 1) && (FCLK_PS != 2) && (FCLK_PS != 4) && (FCLK_PS != 8) && (FCLK_PS != 16) && (FCLK_PS != 32) && (FCLK_PS != 64) && (FCLK_PS != 128) && (FCLK_PS != 256))
#if ((256 % FCLK_PS ) != 0)
  #error "!! Invalid System Clock Prescaler Value !!"
#else
  #message "System clock prescaler set to " FCLK_PS
#endif

#if  FCLK == 1000000
 .message "Using FCLK @ 1MHz"
#elif  FCLK == 2000000
 .message "Using FCLK @ 2MHz"
#elif  FCLK == 4000000
 .message "Using FCLK @ 4MHz"
#elif  FCLK == 5000000
 .message "Using FCLK @ 5MHz"
#elif  FCLK == 6000000
 .message "Using FCLK @ 6MHz"
#elif  FCLK == 8000000
 .message "Using FCLK @ 8MHz"
#elif  FCLK == 10000000
 .message "Using FCLK @ 10MHz"
#elif  FCLK == 12000000
 .message "Using FCLK @ 12MHz"
#elif  FCLK == 16000000
 .message "Using FCLK @ 16MHz"
#elif  FCLK == 20000000
 .message "Using FCLK @ 20MHz"
#else
 .message "No FCLK selected!"
#endif
;
;****************************************************************
.equ	PROMPT		= '>'		;here's our prompt character
.equ	LNLEN		= 54		;max line length for system msgs
;
.equ	NULL		= 0			;define a NULL
.equ	CRLFCHAR	= '~'		;CR/LF character used by the PRINT command
.equ	SPACE		= ' '		;a space character
.equ	LF			= 0x0a		;Line Feed character
.equ	CR			= 0x0d		;Carriage Return character
.equ	BELL		= 0x07		;"Bell" character
.equ	CTRL_C		= 0x03		;^C for halt
.equ	CTRL_H		= 0x08		;^H for delete previous character
.equ	CTRL_I		= 0x09		;^I for tab
.equ	CTRL_S		= 0x13		;^S for suspend
.equ	CTRL_U		= 0x15		;^U for delete line from line buffer
.equ	DELETE		= 0x7f		;"BACKSPACE" key
;
; CRC-8 Polynomial Defs
; These values are changed depending on the algorithm used.
; See the Include/Code_Operators.inc file where "CRC8" is housed.
.equ	CRCBufSz	= 32		;just the 1st 32 bytes
.set	CRC8Init    = 0x00
.set	CRC8Poly 	= 0x18		;CRC-8 Polynomial, 0x18 = (X^8 + X^5 + X^4 + 1)
;.set	CRC8Poly 	= 0x8C		;CRC-8 Polynomial, 0x8C = (X^8 + X^5 + X^4 + 1)
;
;.equ	CRC16Poly 	= 0x8005	;CRC-16 Polynomial. (X^16 + X^15 + X^2 + 1)
;.equ	CRC16Init 	= 0x0000	;CRC-16 initial value, 0x0000
;.equ	CRC16Poly 	= 0x8408	;CRC-XMODEM Polynomial, (X^16 + X^15 + X^10 + X^3)
;.equ	CRC16Init 	= 0x0000	;CRC-16 initial value, 0x0000
.equ	CRC16Poly 	= 0x1021	;CRC-16 CCITT Polynomial, (X^16 + X^12 + X^5 + 1)
.equ	CRC16Init 	= 0xFFFF	;CRC-16 initial value, 0xFFFF
;
.equ	RNDPoly		= 0xB4		;polynomial to seed for RND command
;
.equ	DUMPBYTES	= 16		;# of bytes/line for "dump" command
;
.equ	EOP			= 0			;End of Program token
;
; RENUMBER command setup
.equ	STLNNO		= 5			;default starting line number
.equ	LNINC		= 10			;default line increment
;
;**** Character Cast Type Definitions ***************************
.equ	QC_OPER		= 0		;Operator (not one of the other casts
.equ	QC_DELIM	= 1		;Delimiter -space ($20) or comma ($2C)
.equ	QC_NUM		= 2		;Numeral 0..9 ($30 through $39)
.equ	QC_LTR		= 3		;Letter [A..Z] uppercase ($41 throught $5A)
.equ	QC_CR		= 4		;Carriage return
.equ	QC_QUOT		= 5		;quote character (string support)
							;  or "@" for a constant
;****************************************************************
; TEENSY++ 2.0 support
;****************************************************************
#if defined(__AT90USB1286__) \
	&& TEENSY
 #undef		USB
 #define	USB		1			;include USB serial I/O support code

 #undef		BTLDR
 #define	BTLDR	0			;disable bootloader generation
 								; for TEENSY++ 2.0 loader
 #undef		FCLK
 #define	FCLK	(16000000/FCLK_PS)	;default TEENSY clock

 #message "Disabling BootLoader Support on TEENSY++ 2.0"
 #message "Enabling USB Serial I/O Support on TEENSY++ 2.0"

#else
 #undef		TEENSY
 #define	TEENSY	0			;insure TEENSY 2.0 ++ support code
 								; disabled on M88/168/328/M32u4
#endif
;
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
;****************************************************************
; USI as UART support
;****************************************************************
  #if USI
	#message "ATtiny84/85 with USI enabled"
    .set	USISUPP		= 1		;for AVRASM2 conditionals usage
;    .equ	USI_SERIAL_RAM	= 0x0209	;start of RAM variables
  #endif
;
;****************************************************************
; Tiny84 and Tiny85 Soft UART support
;****************************************************************
 #if !USI
	#message "ATtiny84/85 with UART enabled"
    .set	UARTSUPP	= 1		;for AVRASM2 conditionals usage
;    .equ	UART_SERIAL_RAM	= 0x0209	;start of RAM variables
 #endif
#endif
;
;****************************************************************
; USB support
;****************************************************************
#if USB
  .equ	USBConTmr	= 15	;USB connection timeout (N * 8 sec)
  .set	USBSUPP		= 1		;for AVRASM2 conditionals usage
  #if ( defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) || \
	defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__)  || \
	defined(__ATmega2560__) ) && USB
   #undef	USB
   #define	USB		0		;disable USB support code on M88/168/328
  .set	USBSUPP		= 0		;for AVRASM2 conditionals usage
  #endif

;!!!  DO NOT MODIFY THESE VALUES !!!
  #if defined(__ATmega32U4__)
  .equ	USB_SERIAL_RAM	= 0x0AA0	;start of RAM variables
  #elif defined(__AT90USB1286__)
  .equ	USB_SERIAL_RAM	= 0x20A0	;start of RAM variables
  #endif
#endif
;
#if ( (USB) && ( (FCLK != 8000000) && (FCLK != 16000000) ) )
  #error "USB support is enabled but FCLK is not 8MHz or 16MHz!"
#endif
;
;****************************************************************
; Boot-loader support
;****************************************************************
#if BTLDR
.equ	BLDMJKEY		= 0xE5		;majick byte to invoke bootloader
									; at system reset
#endif
#if ( defined(__ATmega88__) || \
	defined(__ATmega88PA__) ) && BTLDR;no bootloader support for M88
#undef	BTLDR
#define	BTLDR	0
#endif
;
; Fusebyte Constants (for bootloader support)
#if defined(__ATmega168__) || \
	defined(__ATmega168PA__)
.equ	FUSEBYTE		= 0x02		;Ext Fuse for M168 (0x02)
#elif ( defined(__ATmega328__) || \
		defined(__ATmega328P__) || \
		defined(__ATmega16__) || \
		defined(__ATmega16A__) || \
		defined(__ATmega16L__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__) || \
		defined(__ATmega32L__) || \
		defined(__ATmega644P__) || \
		defined(__ATmega1284P__) || \
		defined(__ATmega32U4__) || \
		defined(__ATmega2560__) || \
		defined(__AT90USB1286__) )
.equ	FUSEBYTE		= 0x03		;High Fuse for M32A/328/32U4,
									; M644P/1284P, M2560 and USB1286
									; (0x03)
#endif
;
;****************************************************************
;				MEMORY ALLOCATION
; !!Refer to file "Include\Data_RAM.inc" for memory setup!!
;
;****************************************************************
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
 .set	MCUSTKSZ	= 48		;reserve space in RAM for stack growth
#else
 .set	MCUSTKSZ	= 80		;reserve space in RAM for stack growth
#endif
.ifdef RAMPZ					;24-bit addresses on stack for >64KW FLASH
  .set	MCUSTKSZ	= (MCUSTKSZ*2)	;reserve space in RAM for stack growth
.endif
;
;+++++++++++++++++++++++++++++++++++++
;define flag bits in AB_CFG0
.if CFG
 .message "Enabling non-volatile option storage (CFG)"
 .equ	CFG_SSTRT	= 0		;Flag0 = 1 = self-start enabled
; .equ				= 1		;Flag1 = 1 =
; .equ	CFG_		= 2		;Flag2 = 1 =
; .equ	CFG_		= 3		;Flag3 = 1 =
 .equ	CFG_APP0	= 4		;Flag4 = Application use 0
 .equ	CFG_APP1	= 5		;Flag5 = Application use 1
 .equ	CFG_APP2	= 6		;Flag6 = Application use 2
 .equ	CFG_APP3	= 7		;Flag7 = Application use 3
.endif
;
;+++++++++++++++++++++++++++++++++++++
;define flag bits in GPIOR0
.equ	STRGDET	= 0					;Flag0 = 1 = String detected
.equ	IGNCHAR	= 1					;Flag1 = 1 = ignore characters (parser)
.equ	NRFIRQ	= 2					;Flag2 = 1 = NRF generated interrupt
.equ	ACOIRQ	= 3					;Flag3 = 1 = ACO generated interrupt
.equ	USBIRQ	= 4					;Flag4 = 1 = USB generated interrupt
.equ	WDTIRQ	= 5					;Flag5 = 1 = WDT generated interrupt
.equ	DDSIRQ	= 6					;Flag6 = 1 = DDS generated interrupt
.equ	RTCIRQ	= 7					;Flag7 = 1 = RTC generated interrupt
.ifndef GPIOR0
  .message "This MCU does not support the GPIOR0 register"
.endif
;
;+++++++++++++++++++++++++++++++++++++
;define flag bits in GPIOR1
 .equ	ACOCh0	= 0					;Flag[3:0] = ADC Channel for Analog Comp.
 .equ	ACOCh1	= 1					; 0=ADC0, 1=ADC1, 2=ADC2, 3=ADC3, 4=AIN-
 .equ	ACOCh2	= 2					
; .equ			= 3					;Flag3 =
; .equ			= 4					;Flag4 =
 .equ	BIO_RW	= 5					;Flag5 = 0=read address, 1=write addressed (for PEEK,POKE,EER,EEW)
 .equ	EEP_WE	= 6					;Flag6 = 0=EEP erase/write combo, 1=EEP erase
 .equ	AOVErr	= 7					;Flag7 = 0 = disable arithmetic
									; overflow/underflow errors
.ifndef GPIOR1
  .message "This MCU does not support the GPIOR1 register"
.endif
;
;+++++++++++++++++++++++++++++++++++++
;define flag bits in GPIOR2
.equ	SSTEn	= 0				;Flag0 = Self-Start pin was detected
; .equ	NRF_EN	= 1				;Flag1 = 0=NRF disabled, 1=NRF enabled,
.equ	TWRSngl	= 2				;Flag2 = 1= Read a single TWI byte to data stack
.equ	DebugEn	= 3				;Flag3 = 0=disable, 1=enable debug
.equ	DS_Cmd	= 4				;Flag4 = 0=DATA, 1=CMD
.equ	DHTUnit	= 5				;Flag5 = 0=DHT in Farenheit, 1=DHT in Celcius
.equ	DHTBusy	= 6				;Flag6 = 0=DHT ready to read, 1=DHT Busy (1.5 sec between reads)
.equ	DHTSign	= 7				;Flag7 = 0=ignore sign, 1=allow sign
;
.ifndef GPIOR2
  .message "This MCU does not support the GPIOR2 register"
.endif
;
;+++++++++++++++++++++++++++++++++++++
;define general purpose bit positions
.equ	Bit0	= 0
.equ	Bit1	= 1
.equ	Bit2	= 2
.equ	Bit3	= 3
.equ	Bit4	= 4
.equ	Bit5	= 5
.equ	Bit6	= 6
.equ	Bit7	= 7
;
;****************************************************************
; Watchdog Timer Constants
;****************************************************************
#if defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
.equ	WDTCSR	= WDTCR				
.equ	WDMAX	= 7					;maximum prescaler value

#else
.equ	WDMAX	= 9					;maximum prescaler value

#endif
;
.equ WDT_OFF    = 0x00
.equ WDT_16MS   = (0<<WDP0)
.equ WDT_32MS   = (1<<WDP0)
.equ WDT_64MS   = (1<<WDP1)
.equ WDT_125MS  = (1<<WDP1) | (1<<WDP0)
.equ WDT_250MS  = (1<<WDP2)
.equ WDT_500MS  = (1<<WDP2) | (1<<WDP0)
.equ WDT_1S     = (1<<WDP2) | (1<<WDP1)
.equ WDT_2S     = (1<<WDP2) | (1<<WDP1) | (1<<WDP0)
.ifdef WDP3
 .equ WDT_4S     = (1<<WDP3)
 .equ WDT_8S     = (1<<WDP3) | (1<<WDP0)
.endif
;
;****************************************************************
;	EEPROM storage
;****************************************************************
#if defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
 .equ	EEPE		= EEWE				;redefine bit name
 .equ	EEMPE		= EEMWE				;redefine bit name

#endif
.equ	E2P_RESV	= 32				;reserve 32 bytes of E2P
.equ	AB_CFG0		= 0x00				;AttoBASIC configuration byte 0
.equ	AB_CFG1		= 0x01				;AttoBASIC configuration byte 1

;.equ	eeprogstortop	= LOW(E2END)		;First (top) byte in EEPROM
;.equ	eepromstortopage= HIGH(E2END)		;MSB of highest address in EEPROM
;
.equ	EEPBLNK		= 0xff				;erased EEPROM byte value

.set	EFSHNDSZ	= 1					;for RAM calculations
.set 	EFSBLKSZ	= 0
.if EFS
  .set  EFSIDXOFS	= 0x18				;location of start of file index
  .set	EFSBLKSZ	= 32
  .set	EFSHNDSZ	= 1					;default 1 byte per handle
  .set	HANDLES_MAX	= ((E2END+1)/256)	;1 file handle per 256
 											; bytes of EEPROM.
  .if (HANDLES_MAX > 8)
    .set	HANDLES_MAX	= 8				;8 handles max
  .endif
  .equ	EFSIDXSZ	= EFSHNDSZ * 8		;reserve Index header
  .equ	EFSERAREC	= 0xFF				;Unused Record Type
  .equ	EFSDATREC	= 0x01 				;Data Record Type
  .equ	EFSPAYSZ	= EFSBLKSZ - 2		;size of data payload
  .equ	EFSMAXBLK	= ((E2END+1)/EFSBLKSZ);max # of blocks available
.endif

;****************************************************************
;	RPM & WPM Constants
;****************************************************************
	.equ	PGMmaxEl	= 2*PAGESIZE;maximum data elements
									; per page in bytes
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || \
	defined(__ATmega88__) || defined(__ATmega88PA__)
	.equ	RES_PAGE_CNT	= 1	;loop counter for # of reserved pages
#elif defined(__ATmega168__) || defined(__ATmega168PA__) || \
	defined(__ATmega16__) || defined(__ATmega16L__) || \
		defined(__ATmega328__) || defined(__ATmega328P__) || \
		defined(__ATmega328__) || defined(__ATmega328P__) || \
		defined(__ATmega32__) || defined(__ATmega32A__) || \
		defined(__ATmega32U4__)	; PAGESIZE = 64 words (128 bytes)
	.equ	RES_PAGE_CNT	= (128 / PAGESIZE)	;loop counter for
												;# of reserved pages
#elif 	defined(__ATmega644P__) || \
		defined(__ATmega1284P__)  || \
		defined(__AT90USB1286__) || \
		defined(__ATmega2560__)	; PAGESIZE = 128 words (256 bytes)
	.equ	RES_PAGE_CNT	= (128 / PAGESIZE)	;loop counter for
												;# of reserved pages
#endif
;
;****************************************************************
;	PWM Constants
;****************************************************************
.if PWM
	.include "Include/Defs_PWM.inc"			;include PWM timer defs
.endif
;
;****************************************************************
;	Self-Start Constants
;****************************************************************
#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
  .if SStst
    .if DATAREC	;for AVR Data Recorder
	  .equ	SS_PORT		= PINB		;self-start port
	  .equ	SS_PIN		= 7			;self-start pin
	.else
	  .equ	SS_PORT		= PINC		;self-start port
	  .equ	SS_PIN		= 3			;self-start pin
  	.endif
  .endif
;==================================================
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32U4__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__ATmega2560__) || \
	defined(__AT90USB1286__)
  .if SStst
	.equ	SS_PORT		= PIND		;self-start port
	.equ	SS_PIN		= 7			;self-start pin
  .endif
;==================================================
#elif defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
  .if SStst
	.equ	SS_PORT		= PINB		;self-start port
	.equ	SS_PIN		= 1			;self-start pin
  .endif
#endif
;
;****************************************************************
;	SPI port pins
;****************************************************************
#if DREC
 #define CTRL_SS			1		;Always allow AttoBASIC to control the SS pin  
 #message "AttoBASIC controls SPI_SS line"
#else
 #define CTRL_SS			0		;set to "1" to allow AttoBASIC to control the SS pin  
 #message "AttoBASIC does not control SPI_SS line"
#endif

#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
.if (SPI || DFR || NRF)	;support pins for DATA file, SPI or NRF
  .equ	SPI_PORT		= PORTB		;SPI on this port
  .equ	SPI_SS			= PORTB2	;port pin for SPI Slave select
  .equ	SPI_MOSI		= PORTB3	;port pin for MOSI
  .equ	SPI_MISO		= PORTB4	;port pin for MISO
  .equ	SPI_SCK			= PORTB5	;port pin for SCK
.endif
;-----------------------------------------------
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__)
.if (SPI || DFR || NRF)	;support pins for DATA file, SPI or NRF
  .equ	SPI_PORT		= PORTB		;SPI on this port
  .equ	SPI_SS			= PORTB4	;port pin for SPI Slave select
  .equ	SPI_MOSI		= PORTB5	;port pin for MOSI
  .equ	SPI_MISO		= PORTB6	;port pin for MISO
  .equ	SPI_SCK			= PORTB7	;port pin for SCK
.endif
;-----------------------------------------------
#elif defined(__ATmega32U4__) || \
	defined(__ATmega2560__) || \
	defined(__AT90USB1286__)
.if (SPI || DFR || NRF)	;support pins for DATA file, SPI or NRF
  .equ	SPI_PORT		= PORTB		;SPI on this port
  .equ	SPI_SS			= PORTB0	;port pin for SPI Slave select
  .equ	SPI_SCK			= PORTB1	;port pin for SCK
  .equ	SPI_MOSI		= PORTB2	;port pin for MOSI
  .equ	SPI_MISO		= PORTB3	;port pin for MISO
.endif
#endif
;
;****************************************************************
;	 DS port pins
;****************************************************************
 #if ( defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) )
  .if DSI
   .equ	DS_PORT			= PORTD		;DS on this port
   .equ	DS_SDA			= PORTD4	;port pin for ATTENTION
   .equ	DS_SDD			= PORTD3	;port pin for DATA
  .endif
 #elif ( defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) )
  .if DSI
   .equ	DS_PORT			= PORTC		;DS on this port
   .equ	DS_SDA			= PORTC7	;port pin for ATTENTION
   .equ	DS_SDD			= PORTC6	;port pin for DATA
  .endif
 #elif defined(__ATmega32U4__)
  .if DSI
   .equ	DS_PORT			= PORTC		;DS on this port
   .equ	DS_SDA			= PORTC7	;port pin for ATTENTION
   .equ	DS_SDD			= PORTC6	;port pin for DATA
  .endif
 #elif 	defined(__AT90USB1286__) || \
 	defined(__ATmega2560__)
  .if DSI
   .equ	DS_PORT			= PORTC		;DS on this port
   .equ	DS_SDA			= PORTC5	;port pin for ATTENTION
   .equ	DS_SDD			= PORTC4	;port pin for DATA
  .endif
 #elif defined(__ATtiny84__) ||	defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
  .if DSI
   .equ	DS_PORT			= PORTB		;DS on this port
   .equ	DS_SDA			= PORTB4	;port pin for ATTENTION
   .equ	DS_SDD			= PORTB3	;port pin for DATA
  .endif
#endif
;
;**** Leave the following alone ********************************************
.if DSI
 .equ	DS_PUPS			= 1			;"1" to enable port pin pullups
 .equ	settletime		= $50		;Time for sampling, setting of data lines
 .equ	attline			= 4			;B4	Attention line
 .equ	dataline		= 5			;B5	Data line
.endif
;
.if DHT
  .set	RTC		= 1					;insure Real-time Counter
  .include "Include/Defs_DHT.inc"	;include DHT definitions
.endif
;
;****************************************************************
; One-Wire Port Pins and such
;****************************************************************
.if OWR
  .include "Include/Defs_OWR.inc"	;include OWR definitions
.endif
;
;****************************************************************
; nRF24L01(+) Support
;****************************************************************
.equ	NRF24_MAX_MSG_LEN	= 32	;maximum message length of nRF24

.set	NRFBUFFSZ	= 0
.if NRF
;  .set	SPI			= 1				;insure SPI routines
;  .equ	NRFARDLY	= 8				;10mS delay to allow for auto-tx
  .equ	NRFARDLY	= 15			;10mS delay to allow for auto-tx
  .include "Include/Defs_NRF.inc"	;include NRF definitions
.endif
;
;****************************************************************
; RTC Support
;****************************************************************
.if RTC
  .include "Include/Defs_RTC.inc"			;include DEF's
.endif;
;
;****************************************************************
; DDS Constants
;****************************************************************
.if DDS
.include "Include\Defs_DDS.inc"		;include DDS defs
.endif
;
