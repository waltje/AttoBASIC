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
;**********************************************************************
;	Definitions for ICP registers
;**********************************************************************
.if ICP
 #if ( defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) )
 .equ	ICP_PORT	= PORTD			;define T1 on PD5
 .equ	ICP_PIN		= PIND5

 .equ	ICP_PRR		= PRR			;PRR register

 .equ	ICP_TIMSK	= TIMSK1
 .equ	ICP_TIFR	= TIFR1
 .equ	ICP_TCCRA	= TCCR1A
 .equ	ICP_TCCRB	= TCCR1B
 .equ	ICP_TCCRC	= TCCR1C
 .equ	ICP_TCNTH	= TCNT1H
 .equ	ICP_TCNTL	= TCNT1L
 .equ	ICP_ICRH	= ICR1L
 .equ	ICP_ICRL	= ICR1L

;	TIFR register
 .equ	ICP_ICF		= ICF1
 .equ	ICP_TOV		= TOV1
;
;	TIMSK1 register
 .equ	ICP_OCIE	= OCIE1A
 .equ	ICP_TOIE	= TOIE0
;
;--------------------------------------
 #elif ( defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) )
 .equ	ICP_PORT	= PORTB			;define T1 on PB1
 .equ	ICP_PIN		= PINB1

; .equ	ICP_PRR		= PRR			;no PRR register

 .equ	ICP_TIMSK	= TIMSK
 .equ	ICP_TIFR	= TIFR
 .equ	ICP_TCCRA	= TCCR1A
 .equ	ICP_TCCRB	= TCCR1B
 .equ	ICP_TCNTH	= TCNT1H
 .equ	ICP_TCNTL	= TCNT1L
 .equ	ICP_ICRH	= ICR1L
 .equ	ICP_ICRL	= ICR1L

;	TIFR register
 .equ	ICP_ICF		= ICF1
 .equ	ICP_TOV		= TOV1
 ;
;	TIMSK register
 .equ	ICP_OCIE	= OCIE0
 .equ	ICP_TOIE	= TOIE0
;
;--------------------------------------
 #elif 	defined(__ATmega644P__) || \
		defined(__ATmega1284P__)

 .equ	ICP_PORT	= PORTB			;define T1 on PB1
 .equ	ICP_PIN		= PINB1

 .equ	ICP_PRR		= PRR0			;PRR register

 .equ	ICP_TIMSK	= TIMSK1
 .equ	ICP_TIFR	= TIFR1
 .equ	ICP_TCCRA	= TCCR1A
 .equ	ICP_TCCRB	= TCCR1B
 .equ	ICP_TCCRC	= TCCR1C
 .equ	ICP_TCNTH	= TCNT1H
 .equ	ICP_TCNTL	= TCNT1L
 .equ	ICP_ICRH	= ICR1H
 .equ	ICP_ICRL	= ICR1L

;	TIFR1 register
 .equ	ICP_ICF		= ICF1
 .equ	ICP_TOV		= TOV1
 ;
;	TIMSK1 register
 .equ	ICP_OCIE	= OCIE1A
 .equ	ICP_TOIE	= TOIE0
;
;--------------------------------------
 #elif defined(__ATmega32U4__)
 .equ	ICP_PORT	= PORTD			;define T1 on PD6
 .equ	ICP_PIN		= PIND6

 .equ	ICP_PRR		= PRR0			;PRR register

 .equ	ICP_TIMSK	= TIMSK1
 .equ	ICP_TIFR	= TIFR1
 .equ	ICP_TCCRA	= TCCR1A
 .equ	ICP_TCCRB	= TCCR1B
 .equ	ICP_TCCRC	= TCCR1C
 .equ	ICP_TCNTH	= TCNT1H
 .equ	ICP_TCNTL	= TCNT1L
 .equ	ICP_ICRH	= ICR1L
 .equ	ICP_ICRL	= ICR1L

;	TIFR1 register
 .equ	ICP_ICF		= ICF1
 .equ	ICP_TOV		= TOV1
 ;
;	TIMSK1 register
 .equ	ICP_OCIE	= OCIE1A
 .equ	ICP_TOIE	= TOIE0
;
;--------------------------------------
 #elif defined(__AT90USB1286__)
 .equ	ICP_PORT	= PORTE			;define T3 on PE6
 .equ	ICP_PIN		= PINE6

 .equ	ICP_PRR		= PRR0			;PRR register

 .equ	ICP_TIMSK	= TIMSK3
 .equ	ICP_TIFR	= TIFR3
 .equ	ICP_TCCRA	= TCCR3A
 .equ	ICP_TCCRB	= TCCR3B
 .equ	ICP_TCCRC	= TCCR3C
 .equ	ICP_TCNTH	= TCNT3H
 .equ	ICP_TCNTL	= TCNT3L
 .equ	ICP_ICRH	= ICR3L
 .equ	ICP_ICRL	= ICR3L

;	TIFR register
 .equ	ICP_ICF		= ICF3
 .equ	ICP_TOV		= TOV3

;	TIMSK3 register
 .equ	ICP_OCIE	= OCIE3A
 .equ	ICP_TOIE	= TOIE3
;
;--------------------------------------
#elif defined(__ATmega2560__)
 .equ	ICP_PORT	= PORTC			;define T3 on PC3
 .equ	ICP_PIN		= PINC3

 .equ	ICP_PRR		= PRR0			;PRR register

 .equ	ICP_TIMSK	= TIMSK3
 .equ	ICP_TIFR	= TIFR3
 .equ	ICP_TCCRA	= TCCR3A
 .equ	ICP_TCCRB	= TCCR3B
 .equ	ICP_TCCRC	= TCCR3C
 .equ	ICP_TCNTH	= TCNT3H
 .equ	ICP_TCNTL	= TCNT3L
 .equ	ICP_ICRH	= ICR3L
 .equ	ICP_ICRL	= ICR3L

;	TIFR register
 .equ	ICP_ICF		= ICF3
 .equ	ICP_TOV		= TOV3

;	TIMSK3 register
 .equ	ICP_OCIE	= OCIE3A
 .equ	ICP_TOIE	= TOIE3
;
#else
  #error	"!! Invalid Target MCU !!"
#endif
.endif


