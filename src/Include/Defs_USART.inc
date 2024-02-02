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
;****************************************************************
;				UART SETUP
;****************************************************************
.equ	CHAR_SIZE	= 8			;data size in bits
.equ	STOP_BITS	= 1			;number of stop bits
;
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if  FCLK == 1000000	;Set for 0.16% error
  .message "Using baud rate of 4800 for FCLK = 1MHz"
  .set	baudrate 		=  4800	;choose a baud rate
#elif  FCLK == 2000000	;Set for 0.16% error
  .message "Using baud rate of 9600 for FCLK = 2MHz"
  .set	baudrate 		=  9600	;choose a baud rate
#elif  FCLK == 4000000	;Set for 0.16% error
  .message "Using baud rate of 19200 for FCLK = 4MHz"
  .set	baudrate 		=  19200	;choose a baud rate
#else					;Set for 0.16% error
  .message "Using baud rate of 38400 for FCLK > 4MHz"
  .set	baudrate 		=  38400	;choose a baud rate
#endif
;
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;These are redefs for tiny84 and tiny85 devices since there are 
; no USARTs, we use software emulated.
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;************* Attiny84 Support *******************************************
#if ( defined(__ATtiny84__) || defined(__ATtiny84A__) ) && !USI	;for Soft-UART
;redefine timer0 registers
  .equ	TIMSK		= TIMSK0
  .equ	TIFR		= TIFR0
 #if SIO_EINT
;  #define SIOIRQ	INT0_int	;External Interrupt Request 0
  .equ	UART_Port	= PORTA		;UART port
  .equ	UART_DDR	= DDRA		;UART DDR
  .equ	UART_Rx		= 0			;UART RX pin (INT0)
  .equ	UART_Tx		= 1			;UART TX pin (PBx)
 #else
  .equ	UART_Port	= PORTB		;UART port
  .equ	UART_DDR	= DDRB		;UART DDR
  .equ	UART_Rx		= 0			;UART RX pin (PCINT0)
  .equ	UART_Tx		= 1			;UART TX pin (PBx)
;redefine PCMSK reg based on UART port
 .if UART_Port == PORTA
;  #define SIOIRQ	PCINT0_int	;Pin Change Interrupt Request 0
  .equ	PCIE		= PCIE0
  .equ	PCIF		= PCIF0
  .equ	PCMSK		= PCMSK0
 .elif UART_Port == PORTB
;  #define SIOIRQ	PCINT1_int	;Pin Change Interrupt Request 1
  .equ	PCIE		= PCIE1
  .equ	PCIF		= PCIF1
  .equ	PCMSK		= PCMSK1
 .endif
 #endif

;************* Attiny85 Support *******************************************
#elif defined(__ATtiny85__) && !USI	;for Soft-UART
 #if SIO_EINT
  #define SIOIRQ	INT0_int	;External Interrupt Request 0
  .equ	UART_Port	= PORTB		;UART port
  .equ	UART_DDR	= DDRB		;UART DDR
  .equ	UART_Rx		= 2			;UART RX pin (INT0)
  .equ	UART_Tx		= 1			;UART TX pin (PBx)
 #else
  #define SIOIRQ	PCINT0_int	;Pin Change Interrupt Request 0
  .equ	UART_Port	= PORTB		;UART port
  .equ	UART_DDR	= DDRB		;UART DDR
  .equ	UART_Tx		= 1			;UART TX pin (PBx)
  .equ	UART_Rx		= 0			;UART RX pin (PBx)
 #endif
#endif
;
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;these are redefs for Mega88/168 and Mega32U4 devices since the
; USARTs have different names
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
	.equ	UART_PRR	= PRR
	.equ	UART_PRUART	= PRUSART0	;USART in PRR

	.equ	UBRRH		= UBRR0H
	.equ	UBRRL		= UBRR0L
	.equ	UDR			= UDR0

; UCSRxA - USART Control and Status Register A
	.equ	UCSRA		= UCSR0A
	;UCSR0A bits
	.equ	MPCM		= MPCM0
	.equ	U2X			= U2X0
	.equ	UPE			= UPE0
	.equ	DOR			= DOR0
	.equ	FE			= FE0
	.equ	UDRE		= UDRE0
	.equ	TXC			= TXC0
	.equ	RXC			= RXC0

; UCSRxB - USART Control and Status Register B
	.equ	UCSRB		= UCSR0B
	;UCSR0B bits
	.equ	TXB8		= TXB80
	.equ	RXB8		= RXB80
	.equ	UCSZ2		= UCSZ02
	.equ	TXEN		= TXEN0
	.equ	RXEN		= RXEN0
	.equ	UDRIE		= UDRIE0
	.equ	TXCIE		= TXCIE0
	.equ	RXCIE		= RXCIE0

; UCSRxC - USART Control and Status Register C
	.equ	UCSRC		= UCSR0C
	;UCSR0C bits
	.equ	UCPOL		= UCPOL0
	.equ	UCSZ0		= UCSZ00
	.equ	UCPHA		= UCPHA0
	.equ	UCSZ1		= UCSZ01
	.equ	UDORD		= UDORD0
	.equ	USBS		= USBS0
	.equ	UPM0		= UPM00
	.equ	UPM1		= UPM01
	.equ	UMSEL_0		= UMSEL00
	.equ	UMSEL_1		= UMSEL01

;************* Ameaga16/32 Support *******************************************
#elif defined(__ATmega16__) || \
	defined(__ATmega168L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) 

;Most USART registers already mapped in Mega16/32
;	.equ	UART_PRR	= PRR
;	.equ	UART_PRUART	= PRUSART0	;USART in PRR

;	.equ	UBRRH		= UBRRH
;	.equ	UBRRL		= UBRRL
;	.equ	UDR			= UDR

; UCSRxA - USART Control and Status Register A
;	.equ	UCSRA		= UCSRA
	;UCSR0A bits
;	.equ	MPCM		= MPCM
;	.equ	U2X			= U2X
;	.equ	UPE			= UPE
;	.equ	DOR			= DOR
;	.equ	FE			= FE
;	.equ	UDRE		= UDRE
;	.equ	TXC			= TXC
;	.equ	RXC			= RXC

; UCSRxB - USART Control and Status Register B
;	.equ	UCSRB		= UCSRB
	;UCSR0B bits
;	.equ	TXB8		= TXB8
;	.equ	RXB8		= RXB8
;	.equ	UCSZ2		= UCSZ2
;	.equ	TXEN		= TXEN
;	.equ	RXEN		= RXEN
;	.equ	UDRIE		= UDRIE
;	.equ	TXCIE		= TXCIE
;	.equ	RXCIE		= RXCIE

; UCSRxC - USART Control and Status Register C
;	.equ	UCSRC		= UCSRC
	;UCSR0C bits
;	.equ	UCPOL		= UCPOL
;	.equ	UCSZ0		= UCSZ0
;	.equ	UCSZ1		= UCSZ1
;	.equ	USBS		= USBS
;	.equ	UPM0		= UPM0
;	.equ	UPM1		= UPM1
	.equ	UMSEL_0		= UMSEL
	.equ	URSEL_1		= URSEL

;************* AT90USB1286 and ATmeag32U4 Support *********************
#elif defined(__ATmega32U4__) || \
	defined(__AT90USB1286__)
	.equ	UART_PRR	= PRR1
	.equ	UART_PRUART	= PRUSART1	;USART in PRR

	.equ	UBRRH		= UBRR1H
	.equ	UBRRL		= UBRR1L
	.equ	UDR			= UDR1

; UCSRxA - USART Control and Status Register A
	.equ	UCSRA		= UCSR1A
	;UCSR1A bits
	.equ	MPCM		= MPCM1
	.equ	U2X			= U2X1
	.equ	UPE			= UPE1
	.equ	DOR			= DOR1
	.equ	FE			= FE1
	.equ	UDRE		= UDRE1
	.equ	TXC			= TXC1
	.equ	RXC			= RXC1

; UCSRxB - USART Control and Status Register B
	.equ	UCSRB		= UCSR1B
	;UCSR1B bits
	.equ	TXB8		= TXB81
	.equ	RXB8		= RXB81
	.equ	UCSZ2		= UCSZ12
	.equ	TXEN		= TXEN1
	.equ	RXEN		= RXEN1
	.equ	UDRIE		= UDRIE1
	.equ	TXCIE		= TXCIE1
	.equ	RXCIE		= RXCIE1

; UCSRxC - USART Control and Status Register C
	.equ	UCSRC		= UCSR1C
	;UCSR1C bits
	.equ	UCPOL		= UCPOL1
	.equ	UCSZ0		= UCSZ10
	.equ	UCPHA		= UCPHA1
	.equ	UCSZ1		= UCSZ11
	.equ	UDORD		= UDORD1
	.equ	USBS		= USBS1
	.equ	UPM0		= UPM10
	.equ	UPM1		= UPM11
	.equ	UMSEL_0		= UMSEL10
	.equ	UMSEL_1		= UMSEL11

;************* ATmega2560 Support *******************************************
#elif 	defined(__ATmega1284P__) || \
		defined(__ATmega644P__) || \
		defined(__ATmega2560__)
	.equ	UART_PRR	= PRR0
	.equ	UART_PRUART	= PRUSART0	;USART in PRR

	.equ	UBRRH		= UBRR0H
	.equ	UBRRL		= UBRR0L
	.equ	UDR			= UDR0

; UCSRxA - USART Control and Status Register A
	.equ	UCSRA		= UCSR0A
	;UCSR1A bits
	.equ	MPCM		= MPCM0
	.equ	U2X			= U2X0
	.equ	UPE			= UPE0
	.equ	DOR			= DOR0
	.equ	FE			= FE0
	.equ	UDRE		= UDRE0
	.equ	TXC			= TXC0
	.equ	RXC			= RXC0

; UCSRxB - USART Control and Status Register B
	.equ	UCSRB		= UCSR0B
	;UCSR1B bits
	.equ	TXB8		= TXB80
	.equ	RXB8		= RXB80
	.equ	UCSZ2		= UCSZ02
	.equ	TXEN		= TXEN0
	.equ	RXEN		= RXEN0
	.equ	UDRIE		= UDRIE0
	.equ	TXCIE		= TXCIE0
	.equ	RXCIE		= RXCIE0

; UCSRxC - USART Control and Status Register C
	.equ	UCSRC		= UCSR0C
	;UCSR1C bits
	.equ	UCPOL		= UCPOL0
	.equ	UCSZ0		= UCSZ00
	.equ	UCPHA		= UCPHA0
	.equ	UCSZ1		= UCSZ10
	.equ	UDORD		= UDORD0
	.equ	USBS		= USBS0
	.equ	UPM0		= UPM00
	.equ	UPM1		= UPM01
	.equ	UMSEL_0		= UMSEL00
	.equ	UMSEL_1		= UMSEL01
;
#elif defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	 defined(__ATtiny85__)		;for Soft-UART
  #message ATtiny84(A) or ATtiny85 selected, no hardware USART.
#else
  #error "!! Invalid Target MCU !!"
#endif
