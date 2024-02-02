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
.message "PWM enabled."
;***********************************************************************
; PWMcommand:	8-bit PWM.  Sets channel 0 or 1 to the value passed
;***********************************************************************
PWMcommand:
ACRcommand:
#if defined(__ATtiny85__)
	PUSHY							;save Y
	rcall	Get1stParm				;get the last parameter

	SETB	PWMPORT-1,PWMBITB,temp	;set DDR to output for OCxB

	STOREI	OCRA,0,r1				;clear the OC1A register
	STOREI	OCRC,0xff,r1			;set OC1C register to max count
	STORE	OCRB,U,temp				;store the desired value to OCRB

	ldi		temp,(	(1<<PWM1B) |\
					(2 << COM1B0))	;enable OC1B as 8-bit PWM
	STORE	GTCCR,temp

	ldi		temp,(PWMCK << CS10)	;enable PWM clock
	STORE	TCCR1,temp
;
#else
	PUSHY						;save Y
	rcall	Get1stParm			;get the last parameter

.if (!SPI && !DFR && !NRF)	;only if SPI is not enabled as OC1B shares SS pin on
 #if ( !defined(__ATmega88__) && \
	!defined(__ATmega88PA__) && \
	!defined(__ATmega168__) && \
	!defined(__ATmega168PA__) && \
	!defined(__ATmega328__) && \
	!defined(__ATmega328P__) )
			;ATmega88/168/328 devices
	tst		DStkPtr					;DStkPtr = 0?
	brne	PWMcommand1				;no, more data on stack
	mov		T,U						;xfer PWM value to T
	clr		U						;default channel 0 (OC1A)
	rjmp	PWMcommand2				;set the registers

PWMcommand1:	;there are two values on the dstack
	push	U						;save channel # on stack
	GoSub	popU					;get PWM value into U
	mov		T,U						;xfer to T
	pop		U						;restore channel # from stack

	GoSub	CheckUisBitVal			;only values of [0..1] allowed

 #endif
.endif

PWMcommand2:	;there are two values on the dstack
	clr		r1						;zero a register
 #if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__)
	STORE	TCCRC,r1				;zero TCCRC
 #endif

.if !SPI && !DFR && !NRF	;only if SPI is not enabled as OC1B shares SS pin on
 #if ( !defined(__ATmega88__) && \
	!defined(__ATmega88PA__) && \
	!defined(__ATmega168__) && \
	!defined(__ATmega168PA__) && \
	!defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega328__) && \
	!defined(__ATmega328P__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__) )		;ATmega88/168/328 devices

	sbrs	U,0						;skip next if bit 1 set
	rjmp	PWMcommand2a			;set channel 0 (OC1A)
	rjmp	PWMcommand2b			;set channel 1 (OC1B)
 #endif
.endif


PWMcommand2a:	;set channel 0 (OCxA)
	LOAD	temp,TCCRA				;fetch current TCCRA value
	ori		temp,(	(2 << COM1A0)	| \
					(1 << WGM10) )
	SETB	PWMPORT-1,PWMBITA,temp2	;set DDR to output for OCxA
	STORE16	OCRAL,r1,T
	rjmp	PWMcommand3				;return

PWMcommand2b:	;set channel 1 (OCxB)
	LOAD	temp,TCCRA				;fetch current TCCRA value
	ori		temp,(	(2 << COM1B0)	| \
					(1 << WGM10) )
	SETB	PWMPORT-1,PWMBITB,temp2	;set DDR to output for OCxB
	STORE16	OCRBL,r1,T

PWMcommand3:
	STORE	TCCRA,temp
	ldi		temp,(	(1 << WGM12) | \
					(PWMCK << CS10) )
	STORE	TCCRB,temp 				; prescaler = 1
#endif
;
PWMcommand8:
	Goto	PopYret					;restore Y and return
;
#if !defined(__ATtiny84__) && !defined(__ATtiny84A__) && \
	!defined(__ATtiny85__)
;***********************************************************************
; PWEcommand:	10-bit PWM.  Sets channel 0 or 1 to the value passed
;***********************************************************************
PWEcommand:
	PUSHY							;save Y
	rcall	Get1stParm				;get the last parameter
	mov		H,U						;xfer channel or lo-PWM to H
	GoSub	popU					;get hi-PWM or lo-PWM into U

.if !SPI && !DFR && !NRF	;only if SPI is not enabled as OC1B shares SS pin on
 #if ( !defined(__ATmega88__) && \
	!defined(__ATmega88PA__) && \
	!defined(__ATmega168__) && \
	!defined(__ATmega168PA__) && \
	!defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega328__) && \
	!defined(__ATmega328P__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__) )		;ATmega88/168/328 devices
	tst		DStkPtr					;DStkPtr = 0?
	brne	PWEcommand1				;no, more data on stack (infer channel 0)
	mov		T,H						;xfer lo-PWM value to T
	mov		H,U						;xfer hi-PWM value to H
	clr		U						;default channel 0 (OC1A)
	rjmp	PWEcommand2				;set the registers

PWEcommand1:	;there are three values on the dstack
	push	H						;save channel # on stack
	mov		T,U						;xfer lo-PWM to T
	GoSub	popU					;get hi-PWM value into U
	mov		H,U						;xfer hi-PWM to H
	pop		U						;restore channel # from stack

	GoSub	CheckUisBitVal			;only values of [0..1] allowed

 #endif
.endif


PWEcommand2:	;there are two values on the dstack
	clr		r1						;clear a register
#if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__)
	STORE	TCCRC,r1
#endif

.if !SPI && !DFR && !NRF	;only if SPI is not enabled as OC1B shares SS pin on
 #if ( !defined(__ATmega88__) && \
	!defined(__ATmega88PA__) && \
	!defined(__ATmega168__) && \
	!defined(__ATmega168PA__) && \
	!defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega328__) && \
	!defined(__ATmega328P__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__) )		;ATmega88/168/328 devices

	sbrs	U,0						;skip next if channel bit 1 set
	rjmp	PWEcommand2a			;set channel 0 (OC1A)
	rjmp	PWEcommand2b			;set channel 1 (OC1B)
 #endif
.endif


PWEcommand2a:	;set channel 0 (OCxA)
	LOAD	temp,TCCRA				;fetch current TCCRA value
	ori		temp,(	(2 << COM1A0)	| \
					(3 << WGM10) )
	SETB	PWMPORT-1,PWMBITA,temp	;set DDR to output for OCxA
	STORE16	OCRAL,H,T
	rjmp	PWEcommand8

PWEcommand2b:	;set channel 1 (OCxB)
	LOAD	temp,TCCRA				;fetch current TCCRA value
	ori		temp,(	(2 << COM1B0)	| \
					(3 << WGM10) )
	SETB	PWMPORT-1,PWMBITB,temp	;set DDR to output for OCxB
	STORE16	OCRBL,H,T

PWEcommand8:
	STORE	TCCRA,temp 				; Fast 10-bit PWM

	ldi		temp,(	(1 << WGM12) | \
					(PWMCK << CS10) )
	STORE	TCCRB,temp 				; prescaler = 1
	Goto	PopYret					;restore Y and return
;
#endif
;
;***************************************************************
; PWOcommand:	;Turn Pulse Width Modulation OFF.  Does not
;	tamper with port direction register
;***************************************************************
PWOcommand:				;Pulse Width Modulation OFF
	clr		temp				;zero a register
#if defined(__ATtiny85__)
	STORE	TCCR1,temp 			;disable clock
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	STORE	TCCR1B,temp 		;disable all registers
	STORE	TCCR1A,temp
#else
	STORE	TCCR1B,temp 		;disable all registers
	STORE	TCCR1A,temp
	STORE	TCCR1C,temp
#endif
	ret
;
