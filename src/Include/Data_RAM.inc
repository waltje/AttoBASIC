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
;******************************************************************
;		MEMORY ALLOCATION
;******************************************************************
.dseg
.org		SRAM_START
;******************************************************************
;
.set	DATASTKSZ	= 17						;Data stack size
.set	VARSTORSZ	= 26						;variable storage size

.set	READSTKSZ	= DATASTKSZ					;DATA statement size (w/o NRF)
												; for A - Z
.set	SCRPADSZ	= 0							;Scratchpad RAM size

.set	LNBUFFSZ	= 102						;line buffer size

.set	GOSUDEPTH	= 4							;GOSUB depth
.set	GOSUCHUNK	= 2							;GOSUB chunk size (bytes per level)
.set	GOSUSTKSZ	= (GOSUCHUNK * GOSUDEPTH)	;GOSUB return address stack size

.set	LOOPDEPTH	= 4							;FOR_NEXT loop depth
.set	LOOPCHUNK	= 4							;FOR_NEXT loop chunk size (bytes per level)
.set	LOOPSTKSZ	= (LOOPCHUNK * LOOPDEPTH)	;FOR_NEXT loop stack size

.set	MISCVARSZ	= 7							;starting size of internal variables

.set	USBCONFSZ	= 0							;preset to no USB RAM required
.set	USICONFSZ	= 0							;preset to no USI RAM required
.set	UARTCONFSZ	= 0							;preset to no UART RAM required

.if SRAM_SIZE <= 512	;reduce memory usage on RAM-limited MCU's
  .set	DATASTKSZ	= 9							;Data stack size
  .set	READSTKSZ	= 8							;DATA statement size
  .set	VARSTORSZ	= 13						;reduce variable storage size to half
  .set	GOSUDEPTH	= 2							;GOSUB depth
  .set	GOSUSTKSZ	= (GOSUCHUNK * GOSUDEPTH)	;GOSUB return address stack size
  .set	LOOPDEPTH	= 2							;FOR_NEXT loop depth
  .set	LOOPSTKSZ	= (LOOPCHUNK * LOOPDEPTH)	;FOR_NEXT loop stack size
  .set	LNBUFFSZ	= 54						;smaller line buffer
.endif

#if defined (__ATmega16__) || defined (__ATmega16L__) || \
	defined (__ATmega32__) || defined (__ATmega32A__)
;for Mega16/32, there are no GPIOR registers, so we use RAM instead
  .set	GPIORegs	= 3 
#else
  .set	GPIORegs	= 0 
#endif

#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined (__ATtiny85__)
 .if ( TWI )
 .set	MISCVARSZ	= MISCVARSZ + 1				;add USI-TWI RAM usage
 .endif
#else
 .if ( OWR || SPI || TWI )
 .set	DATASTKSZ	= 2 * DATASTKSZ				;re-size Data stack
 .set	READSTKSZ	= DATASTKSZ					;resize DATA statement buffer
 .endif
#endif

.if ( NRF )
 .set	READSTKSZ	= NRF24_MAX_MSG_LEN			;resize DATA statement for NRF use
 .set	DATASTKSZ	= NRF24_MAX_MSG_LEN			;re-size Data stack size
.endif

.if DHT
 .set	MISCVARSZ	= MISCVARSZ + 7				;add DHT RAM usage
.endif
;
.if RTC
 .set	MISCVARSZ	= MISCVARSZ + 5				;add RTC RAM usage
.endif

.if ICP
 .set	MISCVARSZ	= MISCVARSZ + 1				;add ICP RAM usage
.endif

.if DDS
 .set	MISCVARSZ	= MISCVARSZ + 7				;add DDS RAM usage
.endif

.if DFR
 .set	MISCVARSZ	= MISCVARSZ + 4				;add DATA RAM usage
.endif
;
.if EFS
 .set	MISCVARSZ	= MISCVARSZ + 2				;add EFS RAM usage
.endif

#if USB
 .set	USBCONFSZ	= 15						;USB lib needs 16 bytes of RAM
#endif

#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)	;limit some RAM usage for ATtiny84/85
 #if USI
  .set	USICONFSZ	= 13						;USI RAM requirements
;
 #elif !USI
  .set	UARTCONFSZ	= 6							;USART RAM requirements
 #endif
#endif

#if BIGRAM
   .set	PROGMEMSZ	= 	( SRAM_SIZE - \
						( DATASTKSZ + VARSTORSZ + \
						  READSTKSZ + LNBUFFSZ + \
						  GOSUSTKSZ + LOOPSTKSZ + \
						  SCRPADSZ + MISCVARSZ + \
						  EFSBLKSZ + \
						  GPIORegs + \
						  MCUSTKSZ + \
						  USBCONFSZ ) ) 		;Program memory size
#else
  #if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
  .set	PROGMEMSZ	= 	( SRAM_SIZE - MCUSTKSZ ) - \
						 ( DATASTKSZ + VARSTORSZ + \
						  READSTKSZ + LNBUFFSZ + \
						  GOSUSTKSZ + LOOPSTKSZ + \
						  MISCVARSZ + \
						  USICONFSZ + UARTCONFSZ );Program memory size
  #else
   .set	PROGMEMSZ = (EEPROMEND)
  #endif
#endif
;
;******************************************************************
;					MEMORY ALLOCATION
;******************************************************************
;System RAM starts at the location just below the reserved stack
; space and grows donwward.  Thus the amount of program memory
; is determined by the amount of RAM used by AttoBASIC.
;
;The system memory layout is as follows:
;
PROGMEM:	.byte	PROGMEMSZ	;PROGRAM memory storage
VARSTOR:	.byte	VARSTORSZ	;VARIABLE storage
READSTOR:	.byte	READSTKSZ	;DATA statement storage
SCRPAD:		.byte	SCRPADSZ	;Scratchpad RAM
DATASTK:	.byte	DATASTKSZ	;DATA stack
GOSUSTK:	.byte	GOSUSTKSZ	;DGOSUB-RETURN stack
LOOPSTK:	.byte	LOOPSTKSZ	;FOR-NEXT loop stack
LNBUFF:		.byte	LNBUFFSZ	;LINE buffer
;
;++++++++++++++++++++++++++++++++++++++++++
VARstart:	;beginning of RAM storage
PCLO:		.byte	1		;Marks position in program counter for virtual machine
PCHI:		.byte	1		;This pair points to next available space
;
;=============================================================================
ErrCode:	.byte	1			;holding location for error codes
CurrLine:	.byte	1			;holding location for current program line#
DATAptr:	.byte	2			;holding location for DATA statement pointer
								; plus number of elements
RNDSeed:	.byte	1			;save the current random # seed here
;
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
 .if ( TWI )
  TWI_state:	.byte	1			;Status of USI-TWI
 .endif
#endif
;
.if ICP
 ICPCfg:	.byte	1			;[0..2] 	= ICP gate time coefficient
.endif							;[7]		= fall/rise edge detect
;
.if RTC
 RTCMul:	.byte	1			;holds RTC time multiplier count ("N" x 1mS")
 RTCReg:	.byte	4			;reserve 32-bits for Real-time counter
.endif
;
.if DFR
 DFDfmt:	.byte	1			;[0..7] = 1 to 8 data values per record
 DFIbyte:	.byte	1			;value to use for bulk erase
 DFLaddr:	.byte	2			;internal data logger address counter
.endif
;
.if DHT
RH_I:		.byte	1			;%RH integer
RH_D:		.byte	1			;%RH decimal
RT_I:		.byte	1			;Temperature integer (degrees C)
RT_D:		.byte	1			;Temperature decimal (degrees C)
RT_TMR:		.byte	2			;16-bit DHT timer (in mS)
RT_NRC:		.byte	1			;DHT no-response counter
.endif
;
;=============================================================================
.if EFS	;EEP file system support
 EFSlab:	.byte	1			;last available block
 EFSblkadr:	.byte	1			;desired block number
 EFSbuff:	.byte	EFSBLKSZ	;reserve EFSBLKSZ bytes for buffer
.endif
;
;=============================================================================
#if defined (__ATmega16__) || \
defined (__ATmega16L__) || \
defined (__ATmega32__) || \
defined (__ATmega32A__)
;for Mega16/32, there are no GPIOR registers, so we use RAM instead
 GPIOR0:	.byte	1			;simulated GPIOR0
 GPIOR1:	.byte	1			;simulated GPIOR1
 GPIOR2:	.byte	1			;simulated GPIOR2
#endif
;=============================================================================
.if DDS
DDSbcdL:	.byte	1			;packed BCD of desired DDS frequency
DDSbcdM:	.byte	1
DDSbcdH:	.byte	1

DDSaddrL:	.byte	1			;setup DDS accumulator adder low-byte
DDSaddrM:	.byte	1			;setup DDS accumulator adder mid-byte
DDSaddrH:	.byte	1			;setup DDS accumulator adder high-byte

DDSacch:	.byte	1			;DDS accumulator high-byte (ZL)
.endif
;
;=============================================================================
; USB Serial I/O support
;=============================================================================
#if defined(__ATmega32U4__) || defined(__AT90USB1286__)
 #if USB
USBstart:		;start of USB RAM usage
 .equ	USB_SERIAL_DTR		= 0			;bit 0
 .equ	USB_SERIAL_RTS		= 1			;bit 1
;
 .org 	USB_SERIAL_RAM		;start of USB serial I/O RAM usage
 .org	cdc_line_coding
							.byte	8	;.data:0000
 .org	usb_configuration
							.byte	1	;.bss
 .org	cdc_line_rtsdtr
							.byte	1	;.bss
 .org	transmit_flush_timer
							.byte	1	;.bss
 .org	transmit_previous_timeout
							.byte	1	;.bss
 .org	usb_suspended
							.byte	1	;.bss
 .org	USB_SERIAL_RAM + 13
 usb_irqstate:				.byte	1	;don't remove this, for internal use!
 usb_openforuse:			.byte	1	;don't remove this, for internal use!
USBend:			;end of USB RAM usage
 #endif
#endif
;
;=============================================================================
; USB Serial I/O support
;=============================================================================
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
 #if USI
; These are bit assignments in the RAM variable named "UART_status"
 .equ	TransmittingFromBuffer	=	0	;defined as "ongoing_Transmission_From_Buffer"
 .equ	TransmittingPackage		=	1	;defined as "ongoing_Transmission_Of_Package"
 .equ	ReceivingPackage		=	2	;defined as "ongoing_Reception_Of_Package"
 .equ	ReceiveBufferOverflow	=	3	;defined as "reception_Buffer_Overflow"
;
USIstart:		;start of USI RAM usage
;start of USB serial I/O RAM usage
;.org	(UART_status - 1 )	.byte	1	; reserved
 .org	UART_status			.byte	1	;.data:0000
 .org	UART_TxHead			.byte	1	;.bss
 .org	UART_TxTail			.byte	1	;.bss
 .org	UART_TxBuf			.byte	4	;.bss
 .org	UART_RxHead			.byte	1	;.bss
 .org	UART_RxTail			.byte	1	;.bss
 .org	UART_RxBuf			.byte	4	;.bss
;usi_irqstate:				.byte	2	;don't remove this, for internal use!
USIend:			;end of USI RAM usage
;
 #elif !USI	; Soft-UART RAM usage
;Bit positions definition in state variable
  .equ	DATA_EMPTY 			= 0			;defined as "VUART_DATA_EMPTY"
  .equ	TX_COMPLETE			= 1			;defined as "VUART_TRANSMIT_COMPLETE"
  .equ	RX_COMPLETE			= 2			;defined as "VUART_RECEIVE_COMPLETE"
;start of UART serial I/O RAM usage
UARTstart:		;start of UART RAM usage
  tx_data:					.byte	1
  tx_bit_count:				.byte	1
  rx_data_complete:			.byte	1
  rx_data:					.byte	1
  rx_bit_count:				.byte	1
  vuart_state:				.byte	1
UARTend:		;end of UART RAM usage
 #endif
#endif
;
VARend:		;end of data storage
;
;=============================================================================
	.org	RAMEND-MCUSTKSZ+1		;reserve STACK space
MCUSTK:			.byte	MCUSTKSZ	;uP RETURN stack
;=============================================================================
