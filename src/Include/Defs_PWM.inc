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
; PWM equates
;******************************************************************
#if defined(__ATtiny84__) || defined(__ATtiny84A__)
 .equ		OCRAH		= OCR1AH	;TIMER1 OCA for 10-bit
 .equ		OCRAL		= OCR1AL
 .equ		OCRBH		= OCR1BH	;TIMER1 OCB for 10-bit
 .equ		OCRBL		= OCR1BL
 .equ		TCCRA		= TCCR1A
 .equ		TCCRB		= TCCR1B
 .equ		TCCRC		= TCCR1C
#elif defined(__ATtiny85__)
 .equ		OCRA		= OCR1A		;TIMER1 OCA for 8-bit
 .equ		OCRB		= OCR1B		;TIMER1 OCB for 8-bit
 .equ		OCRC		= OCR1C		;TIMER1 OCC for 8-bit
 .equ		TCCR		= TCCR1
#else
 .equ		OCRAH		= OCR1AH	;TIMER1 OCA for 10-bit
 .equ		OCRAL		= OCR1AL
 .equ		OCRBH		= OCR1BH	;TIMER1 OCB for 10-bit
 .equ		OCRBL		= OCR1BL
 .equ		TCCRA		= TCCR1A
 .equ		TCCRB		= TCCR1B
 .ifdef TCCR1C
 .equ		TCCRC		= TCCR1C
 .endif
#endif

; PWM pin definitions
#if defined(__ATtiny84__) || defined(__ATtiny84A__)
;output only on OC1A 
	.equ	PWMPORT		= PORTA		;port for 10-bit PWM output
	.equ	PWMBITA		= PORTA6	;port pin for PWM OC1A output
	.equ	PWMBITB		= PORTA5	;port pin for PWM OC1B output

#elif defined(__ATtiny85__) 
;output on OC1B only
	.equ	PWMPORT		= PORTB		;port for 8-bit PWM output
;	.equ	PWMBITA		= PORTB1	;port pin for PWM OC1A output;
	.equ	PWMBITB		= PORTB4	;port pin for PWM OC1B output
;
#elif defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
;output on OC1A or OC1B
	.equ	PWMPORT		= PORTB		;port for 10-bit PWM output
	.equ	PWMBITA		= PORTB1	;port pin for PWM OC1A output
	.equ	PWMBITB		= PORTB2	;port pin for PWM OC1B output
#elif	defined(__ATmega16__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) || \
	defined(__ATmega1284P__) || \
	defined(__ATmega644P__)
;output on OC1A or OC1B
	.equ	PWMPORT		= PORTD		;port for 10-bit PWM output
	.equ	PWMBITA		= PORTD4	;port pin for PWM OC1A output
	.equ	PWMBITB		= PORTD5	;port pin for PWM OC1B output
#elif ( defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__) )
;output on OC1A or OC1B
	.equ	PWMPORT		= PORTB		;port for 10-bit PWM output
	.equ	PWMBITA		= PORTB5	;port pin for PWM OC1A output
	.equ	PWMBITB		= PORTB6	;port pin for PWM OC1B output
;
#else
#error	"!! Invalid Target MCU !!"
#endif

.equ	PWMCK_0		= 0			; stop clock
.equ	PWMCK_1		= 1			; Prescaler 1x
.equ	PWMCK_8		= 2			; Prescaler 8x
.equ	PWMCK_64	= 3			; Prescaler 64x
.equ	PWMCK_256	= 4			; Prescaler 256x
.equ	PWMCK_1024	= 5			; Prescaler 1024x
;

.equ	PWMCK		= PWMCK_1	;fastest clock

;set the PWM frequency to 250KHz < PWM < 125KHz
;.equ	PWMCLK_MAX	= 250000
;.if 	f_clk/EXP2(1) <= PWMCLK_MAX	;calculate appropriate prescale value
;	.equ		PWMCK	= PWMCK_1
;.elif 	f_clk/EXP2(3) <= PWMCLK_MAX
;	.equ		PWMCK	= PWMCK_8
;.elif 	f_clk/EXP2(6) <= PWMCLK_MAX
;	.equ		PWMCK	= PWMCK_64
;.elif 	f_clk/EXP2(8) <= PWMCLK_MAX
;	.equ		PWMCK	= PWMCK_256
;.elif 	f_clk/EXP2(10) <= PWMCLK_MAX
;	.equ		PWMCK	= PWMCK_1024
;.endif
