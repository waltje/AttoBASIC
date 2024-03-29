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

;***************************************************************************
; DHT Constants
;***************************************************************************
.message "Enabling DHT Sensor Support"
#define	DHT_TEST 0		;set to "1" to enable DHT testing code
;
#if (FCLK < 2000000)
	#error "!! DHT functions not supported for FCLK less than 2MHz !!"
#endif

#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)

#if DREC
	.equ	DHTPIN	= PINB6				;Pin for DHT Data (use PCINT's)
	.equ	DHPORT	= PORTB				;Port for DHT data pin
#else
	.equ	DHTPIN	= PIND7				;Pin for DHT Data (use PCINT's)
	.equ	DHPORT	= PORTD				;Port for DHT data pin
#endif

;use TIMER1 for for pulse width measurement on Mega88/168/328
; TIMER1 Prescaler Options
	.equ	HTCK_0		= CK1_0			; stop clock
	.equ	HTCK_1		= CK1_1			; Prescaler 1x
	.equ	HTCK_8		= CK1_8			; Prescaler 8x
	.equ	HTCK_64		= CK1_64		; Prescaler 64x
	.equ	HTCK_256	= CK1_256		; Prescaler 256x
	.equ	HTCK_1K		= CK1_1K		; Prescaler 1024x

	.equ	HT_PRR		= PRR
	.equ	HT_PRTIM	= PRTIM1

	.equ	HT_TCNTH	= TCNT1H
	.equ	HT_TCNTL	= TCNT1L
	.equ 	HT_OCRAH	= OCR1AH
	.equ 	HT_OCRAL	= OCR1AL
	.equ 	HT_OCRBH	= OCR1AH
	.equ	HT_OCRBL	= OCR1BL

	.equ	HT_TIMSK	= TIMSK1
	;bit defs
	.equ	HT_TOIE		= TOIE1
	.equ	HT_OCIEA	= OCIE1A
	.equ	HT_OCIEB	= OCIE1B

	.equ	HT_TIFR		= TIFR1
	;bit defs
	.equ	HT_OCFA		= OCF1A
	.equ	HT_OCFB		= OCF1B
	.equ	HT_TOV		= TOV1

	.equ	HT_TCCRA	= TCCR1A
	;bit defs
	.equ	HT_WGM0		= WGM10
	.equ	HT_WGM1		= WGM11
	.equ	HT_COMB0	= COM1B0
	.equ	HT_COMB1	= COM1B1
	.equ	HT_COMA0	= COM1A0
	.equ	HT_COMA1	= COM1A1

	.equ	HT_TCCRB	= TCCR1B
	;bit defs
	.equ	HT_CS0		= CS10
	.equ	HT_CS1		= CS11
	.equ	HT_CS2		= CS12
	.equ	HT_WGM2		= WGM12
	.equ	HT_WGM3		= WGM13

	.equ	HT_TCCRC	= TCCR1C
	;bit defs
	.equ	HT_FOCB		= FOC1B
	.equ	HT_FOCA		= FOC1A
;-----------------------------------------
#elif	defined(__ATmega16__) || \
		defined(__ATmega16L__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__) 
	.equ	DHTPIN	= PIND3				;Pin for DHT Data (use IRQ1)
	.equ	DHPORT	= PORTD				;Port for DHT data pin

;use TIMER1 for for pulse width measurement on Mega88/168/328
; TIMER1 Prescaler Options
	.equ	HTCK_0		= CK1_0			; stop clock
	.equ	HTCK_1		= CK1_1			; Prescaler 1x
	.equ	HTCK_8		= CK1_8			; Prescaler 8x
	.equ	HTCK_64		= CK1_64		; Prescaler 64x
	.equ	HTCK_256	= CK1_256		; Prescaler 256x
	.equ	HTCK_1K		= CK1_1K		; Prescaler 1024x

	.equ	HT_TCNTH	= TCNT1H
	.equ	HT_TCNTL	= TCNT1L
	.equ 	HT_OCRAH	= OCR1AH
	.equ 	HT_OCRAL	= OCR1AL
	.equ 	HT_OCRBH	= OCR1AH
	.equ	HT_OCRBL	= OCR1BL

	.equ	HT_TIMSK	= TIMSK
	;TIMSK bit defs
	.equ	HT_TOIE		= TOIE1
	.equ	HT_OCIEA	= OCIE1A
	.equ	HT_OCIEB	= OCIE1B

	.equ	HT_TIFR		= TIFR
	;TIFR bit defs
	.equ	HT_OCFA		= OCF1A
	.equ	HT_OCFB		= OCF1B
	.equ	HT_TOV		= TOV1

	.equ	HT_TCCRA	= TCCR1A
	;TCCR1Abit defs
	.equ	HT_WGM0		= WGM10
	.equ	HT_WGM1		= WGM11
	.equ	HT_COMB0	= COM1B0
	.equ	HT_COMB1	= COM1B1
	.equ	HT_COMA0	= COM1A0
	.equ	HT_COMA1	= COM1A1

	.equ	HT_TCCRB	= TCCR1B
	;TCCR1B bit defs
	.equ	HT_CS0		= CS10
	.equ	HT_CS1		= CS11
	.equ	HT_CS2		= CS12
	.equ	HT_WGM2		= WGM12
	.equ	HT_WGM3		= WGM13
;
;-----------------------------------------
#elif	defined(__ATmega644P__) || \
		defined(__ATmega1284P__)

	.equ	DHTPIN	= PINB3				;Pin for DHT Data (PCINT11)
	.equ	DHPORT	= PORTB				;Port for DHT data pin

;use TIMER1 for for pulse width measurement on Mega88/168/328
; TIMER1 Prescaler Options
	.equ	HTCK_0		= CK1_0			; stop clock
	.equ	HTCK_1		= CK1_1			; Prescaler 1x
	.equ	HTCK_8		= CK1_8			; Prescaler 8x
	.equ	HTCK_64		= CK1_64		; Prescaler 64x
	.equ	HTCK_256	= CK1_256		; Prescaler 256x
	.equ	HTCK_1K		= CK1_1K		; Prescaler 1024x

	.equ	HT_PRR		= PRR0
	.equ	HT_PRTIM	= PRTIM1

	.equ	HT_TCNTH	= TCNT1H
	.equ	HT_TCNTL	= TCNT1L
	.equ 	HT_OCRAH	= OCR1AH
	.equ 	HT_OCRAL	= OCR1AL
	.equ 	HT_OCRBH	= OCR1AH
	.equ	HT_OCRBL	= OCR1BL

	.equ	HT_TIMSK	= TIMSK1
	;TIMSK bit defs
	.equ	HT_TOIE		= TOIE1
	.equ	HT_OCIEA	= OCIE1A
	.equ	HT_OCIEB	= OCIE1B

	.equ	HT_TIFR		= TIFR1
	;TIFR bit defs
	.equ	HT_OCFA		= OCF1A
	.equ	HT_OCFB		= OCF1B
	.equ	HT_TOV		= TOV1

	.equ	HT_TCCRA	= TCCR1A
	;TCCR1Abit defs
	.equ	HT_WGM0		= WGM10
	.equ	HT_WGM1		= WGM11
	.equ	HT_COMB0	= COM1B0
	.equ	HT_COMB1	= COM1B1
	.equ	HT_COMA0	= COM1A0
	.equ	HT_COMA1	= COM1A1

	.equ	HT_TCCRB	= TCCR1B
	;TCCR1B bit defs
	.equ	HT_CS0		= CS10
	.equ	HT_CS1		= CS11
	.equ	HT_CS2		= CS12
	.equ	HT_WGM2		= WGM12
	.equ	HT_WGM3		= WGM13

	.equ	HT_TCCRC	= TCCR1C
	;bit defs
	.equ	HT_FOCB		= FOC1B
	.equ	HT_FOCA		= FOC1A
;
;-----------------------------------------
#elif defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__) 
	.equ	DHTPIN	= PINB2				;Pin for DHT Data (use PCINT's)
	.equ	DHPORT	= PORTB				;Port for DHT data pin

;use TIMER3 for for pulse width measurement on Mega32U4
;  USB1286, and Mega2560
; TIMER3 Prescaler Options
	.equ	HTCK_0		= CK3_0			; stop clock
	.equ	HTCK_1		= CK3_1			; Prescaler 1x
	.equ	HTCK_8		= CK3_8			; Prescaler 8x
	.equ	HTCK_64		= CK3_64		; Prescaler 64x
	.equ	HTCK_256	= CK3_256		; Prescaler 256x
	.equ	HTCK_1K		= CK3_1K		; Prescaler 1024x

	.equ	HT_PRR		= PRR1  
	.equ	HT_PRTIM	= PRTIM3

	.equ	HT_TCNTH	= TCNT3H
	.equ	HT_TCNTL	= TCNT3L
	.equ 	HT_OCRAH	= OCR3AH
	.equ 	HT_OCRAL	= OCR3AL
	.equ	HT_OCRBH	= OCR3BH
	.equ	HT_OCRBL	= OCR3BL
	.equ	HT_OCRCH	= OCR3BH
	.equ	HT_OCRCL	= OCR3BL

	.equ	HT_TIMSK	= TIMSK3
	;bit defs
	.equ	HT_TOIE		= TOIE3
	.equ	HT_OCIEA	= OCIE3A
	.equ	HT_OCIEB	= OCIE3B
	.equ	HT_OCIEC	= OCIE3C

	.equ	HT_TIFR		= TIFR3
	;bit defs
	.equ	HT_OCFA		= OCF3A
	.equ	HT_OCFB		= OCF3B
	.equ	HT_OCFC		= OCF3C
	.equ	HT_TOV		= TOV3

	.equ	HT_TCCRA	= TCCR3A
	;bit defs
	.equ	HT_WGM0		= WGM30
	.equ	HT_WGM1		= WGM31
	.equ	HT_COMB0	= COM3B0
	.equ	HT_COMB1	= COM3B1
	.equ	HT_COMA0	= COM3A0
	.equ	HT_COMA1	= COM3A1

	.equ	HT_TCCRB	= TCCR3B
	;bit defs
	.equ	HT_CS0		= CS30
	.equ	HT_CS1		= CS31
	.equ	HT_CS2		= CS32
	.equ	HT_WGM2		= WGM32
	.equ	HT_WGM3		= WGM33

	.equ	HT_TCCRC	= TCCR3C
	;bit defs
	.equ	HT_FOCA		= FOC3A
	.equ	HT_FOCB		= FOC3B
	.equ	HT_FOCC		= FOC3C
;
#else
#error	"!! Invalid Target MCU !!"
#endif

;
;**** Leave the following alone ********************************************
 .equ	PCINT_E	= PCICR				;PCINT control register for DHTPIN
 .equ	PCINT_I	= PCIFR				;PCINT flag register for DHTPIN

#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
 .if (DHPORT == PORTB)
  .equ	PCINT_B	= 0
  .equ	PCINT_M	= PCMSK0			;PCINT mask register for DHTPIN, PORTB
 .elif (DHPORT == PORTC)
  .equ	PCINT_B	= 1
  .equ	PCINT_M	= PCMSK1			;PCINT mask register for DHTPIN, PORTC
 .elif (DHPORT == PORTD)
  .equ	PCINT_B	= 2
  .equ	PCINT_M	= PCMSK2			;PCINT mask register for DHTPIN, PORTD
 .endif
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
 .if (DHPORT == PORTD)
  .equ	PCINT_B	= INT0
  .equ	PCINT_M	= GICR			;GICR mask register for DHTPIN, PORTD3
 .endif
;#-------------------------------
#elif defined(__ATmega644P__) || \
	defined(__ATmega1284P__)
 .if (DHPORT == PORTA)
	.equ	PCINT_M	= PCMSK0		;PCINT mask register for DHTPIN, PORTB
	.equ	PCINT_B	= 0				;PORTA = bit 0
 .elif (DHPORT == PORTB)
	.equ	PCINT_M	= PCMSK1		;PCINT mask register for DHTPIN, PORTB
	.equ	PCINT_B	= 1				;PORTB = bit 1
 .elif (DHPORT == PORTC)
 	.equ	PCINT_M	= PCMSK2		;PCINT mask register for DHTPIN, PORTC
	.equ	PCINT_B	= 2				;PORTC = bit 2
 .elif (DHPORT == PORTD)
	.equ	PCINT_M	= PCMSK3		;PCINT mask register for DHTPIN, PORTD
	.equ	PCINT_B	= 3				;PORTD = bit 3
 .endif
#elif defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__)
    .equ	PCINT_B	= 0
    .equ	PCINT_M	= PCMSK0		;PCINT mask register for DHTPIN, PORTB
  .if (DHPORT != PORTB)
    .error "DHT sensor can only use PORTB on ATMega32U4 and AT90USB1286!"
  .endif
#else
  #error	"!! Invalid Target MCU !!"
#endif
;
.equ	HTRdTmr	= 2000				;2000mS delay between readings
.equ	HTPulse	= 3					;N x 1mS for DHTxx init pulse width
.equ	HTNoRsp = 3					;number of non-responses before issuing
									; an error message.

.equ	HTBytes	= 5					;5 data bytes
.equ	HTBits	= 8					;5 * 8 (40) bits of data
;
.set	HTZero	= 22				;minimum 22uS bit duration for a "0"
.set	HTOne	= 70				;minimum 70uS bit duration for a "1"
;
;determine the incremental time resolution (in uS)
;.equ	RTC_RES	= (1000000 / (f_clk / RTC_PSCLR))
;.set	HTZero	= (HTZero /	RTC_RES);minimum 23uS bit duration for a "0"
;.set	HTOne	= (HTOne / RTC_RES)	;minimum 68uS bit duration for a "1"
;
.equ	HT_IPS			= 1000000	;# of interrupts per second

.set	HT_CLK			= HTCK_1	;actual bit value of prescaler
.set	HT_PSCLR		= 1			;TMR0 prescaler set to 1 for start
;
;Calculate OCRA register load value for "HT_IPS" interupt interval
;.set	HTOne			= ((10*FCLK)/HT_PSCLR/HT_IPS)
.set	HT_OCRA_L		= (((10*(FCLK/HT_IPS))/HT_PSCLR)*HTOne)

.if ((HT_OCRA_L /10) > 255)
 .set	HT_PSCLR		= 8			;increase TMR1 prescaler to 16
 .set	HT_CLK			= HTCK_8	;actual bit value of prescaler
; .set	HT_OCRA_L		= (((10*(FCLK/HT_IPS))/HT_PSCLR)*HTOne)/10
 .message "Using divide by 8 for DHT timing."
.elif ((HT_OCRA_L /10) > 255)
 .set	HT_PSCLR		= 64		;increase TMR1 prescaler to 64
 .set	HT_CLK			= HTCK_64	;actual bit value of prescaler
 ;.set	HT_OCRA_L		= (((10*(FCLK/HT_IPS))/HT_PSCLR)*HTOne)/10
 .message "Using divide by 64 for DHT timing."
.elif ((HT_OCRA_L /10) > 255)
 .set	HT_PSCLR		= 256		;increase TMR1 prescaler to 256
 .set	HT_CLK			= HTCK_256	;actual bit value of prescaler
 ;.set	HT_OCRA_L		= (((10*(FCLK/HT_IPS))/HT_PSCLR)*HTOne)/10
 .message "Using divide by 256 for DHT timing."
.elif ((HT_OCRA_L /10) > 255)
 .set	HT_PSCLR		= 1024		;increase TMR1 prescaler to 1024
 .set	HT_CLK			= HTCK_1K	;actual bit value of prescaler
 ;.set	HT_OCRA_L		= (((10*(FCLK/HT_IPS))/HT_PSCLR)*HTOne)/10
 .message "Using divide by 1024 for DHT timing."
.else
 .message "Using divide by 1 for DHT timing."
.endif

.set	HT_OCRA_L		= ((((10*(FCLK/HT_IPS))/HT_PSCLR)*HTOne)/10)

