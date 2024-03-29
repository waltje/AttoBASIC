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
;******************************************************************************
; AVR Pinout Definitions
;******************************************************************************
;
#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
;Port B pin definitions
;Note: These are PCINT[7..0] on PCINT0 vector
;.equ			= PINB0			; 
;.equ			= PINB1			; 
;.equ			= PINB2			; 
;.equ			= PINB3			; 
;.equ			= PINB4			; 
;.equ			= PINB5			; 
;.equ			= PINB6			; 
;.equ			= PINB7			; 
;Set Port B
.equ	PrtBDir	= 0b00000000	; Ouput Direction Register
.equ	PrtBPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtBPdn	= 0b00000000	; Default Pin settings (Power-Down)
;---------------------------------------------
;Note: These are PCINT[15..8] on PCINT1 vector
;Port C pin definitions
;.equ			= PINC0			; 
;.equ			= PINC1			; 
;.equ			= PINC2			; 
;.equ			= PINC3			; 
;.equ			= PINC4			; TWI SDA 
;.equ			= PINC5			; TWI SCL 
;.equ			= PINC6			; 
;.equ			= PINC7			; 
;Set Port C
.equ	PrtCDir	= 0b00000000	; Ouput Direction Register
.equ	PrtCPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtCPdn	= 0b0000000		; Default Pin settings (Power-Down)
;---------------------------------------------
;Port D pin definitions
;Note: These are PCINT[23..16] on PCINT2 vector
;.equ	RXD		= PIND0			; DIn: RS-232 RX
;.equ	TXD		= PIND1			; DOut: RS-232 TX
;.equ	USBDN	= PIND1			; not implimented on this rev (USB D-)
;.equ	USBDP	= PIND2			; not implimented on this rev (USB D+) 
;.equ			= PIND3			; 
;.equ			= PIND4			; 
;.equ			= PIND5			; 
;.equ			= PIND6			; 
;.equ			= PIND7			; 
;Set Port D
.equ	PrtDDir	= 0b00000000	; Ouput Direction Register
.equ	PrtDPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtDPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PIND			; Port for 
;.equ			= PORTD			; Port for 
;
;********************************************************************************
#elif defined(__ATmega16__) ||defined(__ATmega32__) || defined(__ATmega32A__) || \
	defined(__ATmega1284P__) || defined(__ATmega644P__)
;********************************************************************************
.ifdef PORTA
;Port A pin definitions
;.equ			= PINA0			; ADC0
;.equ			= PINA1			; ADC1
;.equ			= PINA2			; ADC2
;.equ			= PINA3			; ADC3
;.equ			= PINA4			; ADC4
;.equ			= PINA5			; ADC5
;.equ			= PINA6			; ADC6
;.equ			= PINA7			; ADC7
;Set Port A
.equ	PrtADir	= 0b00000000	; Ouput Direction Register
.equ	PrtAPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtAPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif

.ifdef PORTB
;---------------------------------------------
;Port B pin definitions
;.equ			= PINB0			; T0
;.equ			= PINB1			; T1
;.equ			= PINB2			; AIN0/T2
;.equ			= PINB3			; AIN1/OC0
;.equ			= PINB4			; \S\S
;.equ			= PINB5			; MOSI
;.equ			= PINB6			; MISO
;.equ			= PINB7			; SCK
;Set Port B
.equ	PrtBDir	= 0b00000000	; Ouput Direction Register
.equ	PrtBPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtBPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif

.ifdef PORTC
;---------------------------------------------
;Port C pin definitions
;.equ	SCL		= PINC0			; TWI SCL  
;.equ	SDA		= PINC1			; TWI SDA 
;.equ			= PINC2			; JTAG TCK
;.equ			= PINC3			; JTAG TMS
;.equ			= PINC4			; JTAG TDO
;.equ			= PINC5			; JTAG TDI
;.equ			= PINC6			; TOSC1 
;.equ			= PINC7			; TOSC2

; Define JTAG pins
.equ	J_TCK	= PINC2
.equ	J_TMS	= PINC3
.equ	J_TDO	= PINC4
.equ	J_TDI	= PINC5

;Set Port C
.equ	PrtCDir	= 0b00000000	; Ouput Direction Register
.equ	PrtCPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtCPdn	= 0b0000000		; Default Pin settings (Power-Down)
.endif

.ifdef PORTD
;---------------------------------------------
;Port D pin definitions
;.equ	RXD		= PIND0			; RS-232 RX 
;.equ	TXD		= PIND1			; RS-232 TX 
;.equ	INT0	= PIND2			; INT0
;.equ	INT1	= PIND3			; INT1
;.equ			= PIND4			; OC1B
;.equ			= PIND5			; OC1B
;.equ			= PIND6			; ICP1
;.equ			= PIND7			; OC2
;Set Port D
.equ	PrtDDir	= 0b00000000	; Ouput Direction Register
.equ	PrtDPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtDPdn	= 0b0000000		; Default Pin settings (Power-Down)
.endif
;
;********************************************************************************
#elif defined(__ATmega32U4__)
;********************************************************************************
;Port B pin definitions
;Note: These are PCINT[7..0] on PCINT0 vector
;.equ			= PINB0			; 
;.equ			= PINB1			; 
;.equ			= PINB2			; 
;.equ			= PINB3			; 
;.equ			= PINB4			; 
;.equ			= PINB5			; 
;Set Port B
.equ	PrtBDir	= 0b00100110	; Ouput Direction Register
.equ	PrtBPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtBPdn	= 0b00000000	; Default Pin settings (Power-Down)
;---------------------------------------------
;Note: These are PCINT[15..8] on PCINT1 vector
;Port C pin definitions
;.equ			= PINC0			; 
;.equ			= PINC1			; 
;.equ			= PINC2			; 
;.equ			= PINC3			; 
;.equ			= PINC4			; TWI SDA 
;.equ			= PINC5			; TWI SCL 
;.equ			= PINC6			; 
;.equ			= PINC7			; 
;Set Port C
.equ	PrtCDir	= 0b00000000	; Ouput Direction Register
.equ	PrtCPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtCPdn	= 0b0000000	; Default Pin settings (Power-Down)
;---------------------------------------------
;Port D pin definitions
;.equ			= PIND0			; 
;.equ			= PIND1			; 
;.equ			= PIND1			; 
;.equ	RXD		= PIND2			; DIn: RS-232 RX
;.equ	TXD		= PIND3			; DOut: RS-232 TX
;.equ			= PIND4			; 
;.equ			= PIND5			; 
;.equ			= PIND6			; 
;.equ			= PIND7			; 
;Set Port D
.equ	PrtDDir	= 0b00001000	; Ouput Direction Register
.equ	PrtDPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtDPdn	= 0b00000000	; Default Pin settings (Power-Dn)
;
;.equ			= PIND			; Port for 
;.equ			= PORTD			; Port for 

;---------------------------------------------
;Port E pin definitions
;.equ			= PINE0			; 
;.equ			= PINE1			; 
;.equ			= PINE1			; 
;.equ			= PINE2			; 
;.equ			= PINE3			; 
;.equ			= PINE4			; 
;.equ			= PINE5			; 
;.equ			= PINE6			; 
;.equ			= PINE7			; 
;Set Port E
.equ	PrtEDir	= 0b00000000	; Ouput Direction Register
.equ	PrtEPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtEPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINE			; Port for 
;.equ			= PORTE			; Port for 

;---------------------------------------------
;Port F pin definitions
;.equ			= PINF0			; 
;.equ			= PINF1			; 
;.equ			= PINF1			; 
;.equ			= PINF2			; 
;.equ			= PINF3			; 
;.equ			= PINF4			; 
;.equ			= PINF5			; 
;.equ			= PINF6			; 
;.equ			= PINF7			; 

; Define JTAG pins
.equ	J_TCK	= PINF4
.equ	J_TMS	= PINF5
.equ	J_TDO	= PINF6
.equ	J_TDI	= PINF7

;Set Port F
.equ	PrtFDir	= 0b00000000	; Ouput Direction Register
.equ	PrtFPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtFPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINF			; Port for 
;.equ			= PORTF			; Port for DDS 
;********************************************************************************
#elif ( defined(__AT90USB1286__) || defined(__ATmega2560__) )
;********************************************************************************
.ifdef PORTA
;Port A pin definitions
;.equ			= PINA0			; 
;.equ			= PINA1			; 
;.equ			= PINA2			; 
;.equ			= PINA3			; 
;.equ			= PINA4			; 
;.equ			= PINA5			; 
;.equ			= PINA6			; 
;.equ			= PINA7			; 
;Set Port A
.equ	PrtADir	= 0b00000000	; Ouput Direction Register
.equ	PrtAPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtAPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif

.ifdef PORTB
;---------------------------------------------
;Port B pin definitions
;Note: These are PCINT[7..0] on PCINT0 vector
;.equ			= PINB0			; 
;.equ			= PINB1			; 
;.equ			= PINB2			; 
;.equ			= PINB3			; 
;.equ			= PINB4			; 
;.equ			= PINB5			; 
;Set Port B
.equ	PrtBDir	= 0b00100110	; Ouput Direction Register
.equ	PrtBPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtBPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif

.ifdef PORTC
;---------------------------------------------
;Note: These are PCINT[15..8] on PCINT1 vector
;Port C pin definitions
;.equ			= PINC0			; 
;.equ			= PINC1			; 
;.equ			= PINC2			; 
;.equ			= PINC3			; 
;.equ			= PINC4			; TWI SDA 
;.equ			= PINC5			; TWI SCL 
;.equ			= PINC6			; 
;.equ			= PINC7			; 
;Set Port C
.equ	PrtCDir	= 0b00000000	; Ouput Direction Register
.equ	PrtCPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtCPdn	= 0b0000000	; Default Pin settings (Power-Down)
.endif

.ifdef PORTD
;---------------------------------------------
;Port D pin definitions
;.equ			= PIND0			; 
;.equ			= PIND1			; 
;.equ			= PIND1			; 
;.equ	RXD		= PIND2			; DIn: RS-232 RX
;.equ	TXD		= PIND3			; DOut: RS-232 TX
;.equ			= PIND4			; 
;.equ			= PIND5			; 
;.equ			= PIND6			; 
;.equ			= PIND7			; 
;Set Port D
.equ	PrtDDir	= 0b00001000	; Ouput Direction Register
.equ	PrtDPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtDPdn	= 0b00000000	; Default Pin settings (Power-Dn)
;
;.equ			= PIND			; Port for 
;.equ			= PORTD			; Port for 
.endif

.ifdef PORTE
;---------------------------------------------
;Port E pin definitions
;.equ			= PINE0			; 
;.equ			= PINE1			; 
;.equ			= PINE1			; 
;.equ			= PINE2			; 
;.equ			= PINE3			; 
;.equ			= PINE4			; 
;.equ			= PINE5			; 
;.equ			= PINE6			; 
;.equ			= PINE7			; 
;Set Port E
.equ	PrtEDir	= 0b00000000	; Ouput Direction Register
.equ	PrtEPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtEPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINE			; Port for 
;.equ			= PORTE			; Port for 
.endif

.ifdef PORTF
;---------------------------------------------
;Port F pin definitions
;.equ			= PINF0			; 
;.equ			= PINF1			; 
;.equ			= PINF1			; 
;.equ			= PINF2			; 
;.equ			= PINF3			; 
;.equ			= PINF4			; 
;.equ			= PINF5			; 
;.equ			= PINF6			; 
;.equ			= PINF7			; 

; Define JTAG pins
.equ	J_TCK	= PINF4
.equ	J_TMS	= PINF5
.equ	J_TDO	= PINF6
.equ	J_TDI	= PINF7

;Set Port F
.equ	PrtFDir	= 0b00000000	; Ouput Direction Register
.equ	PrtFPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtFPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINF			; Port for 
;.equ			= PORTF			; Port for DDS 
.endif
;
#elif ( defined(__AT90USB1286__) || defined(__ATmega2560__) )
;---------------------------------------------
.ifdef PORTG
;Port G pin definitions
;.equ			= PING0			; 
;.equ			= PING1			; 
;.equ			= PING1			; 
;.equ			= PING2			; 
;.equ			= PING3			; 
;.equ			= PING4			; 
;.equ			= PING5			; 
;.equ			= PING6			; 
;.equ			= PING7			; 
;Set Port G
.equ	PrtGDir	= 0b00000000	; Ouput Direction Register
.equ	PrtGPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtGPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PING			; Port for 
;.equ			= PORTG			; Port for DDS 
.endif

.ifdef PORTH
;---------------------------------------------
;Port H pin definitions
;.equ			= PINH0			; 
;.equ			= PINH1			; 
;.equ			= PINH1			; 
;.equ			= PINH2			; 
;.equ			= PINH3			; 
;.equ			= PINH4			; 
;.equ			= PINH5			; 
;.equ			= PINH6			; 
;.equ			= PINH7			; 
;Set Port H
.equ	PrtHDir	= 0b00000000	; Ouput Direction Register
.equ	PrtHPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtHPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINH			; Port for 
;.equ			= PORTH			; Port for DDS 
.endif

.ifdef PORTJ
;---------------------------------------------
;Port J pin definitions
;.equ			= PINJ0			; 
;.equ			= PINJ1			; 
;.equ			= PINJ1			; 
;.equ			= PINJ2			; 
;.equ			= PINJ3			; 
;.equ			= PINJ4			; 
;.equ			= PINJ5			; 
;.equ			= PINJ6			; 
;.equ			= PINJ7			; 
;Set Port J
.equ	PrtJDir	= 0b00000000	; Ouput Direction Register
.equ	PrtJPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtJPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINJ			; Port for 
;.equ			= PORTJ			; Port for DDS 
.endif

.ifdef PORTK
;---------------------------------------------
;Port K pin definitions
;.equ			= PINK0			; 
;.equ			= PINK1			; 
;.equ			= PINK1			; 
;.equ			= PINK2			; 
;.equ			= PINK3			; 
;.equ			= PINK4			; 
;.equ			= PINK5			; 
;.equ			= PINK6			; 
;.equ			= PINK7			; 
;Set Port K
.equ	PrtKDir	= 0b00000000	; Ouput Direction Register
.equ	PrtKPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtKPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINK			; Port for 
;.equ			= PORTK			; Port for DDS 
.endif

.ifdef PORTL
;---------------------------------------------
;Port L pin definitions
;.equ			= PINL0			; 
;.equ			= PINL1			; 
;.equ			= PINL1			; 
;.equ			= PINL2			; 
;.equ			= PINL3			; 
;.equ			= PINL4			; 
;.equ			= PINL5			; 
;.equ			= PINL6			; 
;.equ			= PINL7			; 
;Set Port L
.equ	PrtLDir	= 0b00000000	; Ouput Direction Register
.equ	PrtLPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtLPdn	= 0b10101000	; Default Pin settings (Power-Dn)
;
;.equ			= PINL			; Port for 
;.equ			= PORTL			; Port for DDS 
.endif
#elif defined(__ATtiny84__) || defined(__ATtiny84A__)
;---------------------------------------------
.ifdef PORTA
;Port A pin definitions
;.equ			= PINA0			; ADC0
;.equ			= PINA1			; ADC1
;.equ			= PINA2			; ADC2
;.equ			= PINA3			; ADC3
;.equ			= PINA4			; ADC4
;.equ			= PINA5			; ADC5
;.equ			= PINA6			; ADC6
;.equ			= PINA7			; ADC7
;Set Port A
.equ	PrtADir	= 0b00000000	; Ouput Direction Register
.equ	PrtAPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtAPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif
;---------------------------------------------
.ifdef PORTB
;Port B pin definitions
;Note: These are PCINT[6..0] on PCINT0 vector
;.equ			= PINB0			; 
;.equ			= PINB1			; 
;.equ			= PINB2			; 
;.equ			= PINB3			; 
;.equ			= PINB4			; 
;.equ			= PINB5			; 
;Set Port B
.equ	PrtBDir	= 0b00000000	; Ouput Direction Register
.equ	PrtBPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtBPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif
#elif defined(__ATtiny85__)
.ifdef PORTB
;---------------------------------------------
;Port B pin definitions
;Note: These are PCINT[6..0] on PCINT0 vector
;.equ			= PINB0			; 
;.equ			= PINB1			; 
;.equ			= PINB2			; 
;.equ			= PINB3			; 
;.equ			= PINB4			; 
;.equ			= PINB5			; 
;Set Port B
.equ	PrtBDir	= 0b00000010	; Ouput Direction Register
.equ	PrtBPup	= 0b00000000	; Default Pin settings (Power-Up)
;.equ	PrtBPdn	= 0b00000000	; Default Pin settings (Power-Down)
.endif
#else
  #error	"!! Invalid Target MCU !!"
#endif
