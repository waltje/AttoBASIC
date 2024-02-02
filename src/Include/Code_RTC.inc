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
.message "RTC enabled."
;***********************************************************************
;OC0A_int:  ISR for TMR0 Compare Match A (OCR0A).  Increments the
;	32-bit sequential counter stored in RAM@RTTReg.  The clock source
;	is momentarily switch off while the low byte register is read.
;***********************************************************************
OC0A_int:
OC0_int:
	push	temp				;save all registers used
	PUSHY
	PUSHZ

	LOAD	ZH,SREG				;fetch SREG
	push	ZH					;save SREG

OC0A_int1:	;no need to stop TMR0 as interrupts are disabled during ISR
	tst		RTCcntr				;[RTCcntr] == 0?
	breq	OC0A_int1a			;yes, process RTC count
	dec		RTCcntr				;decrement RTCcntr
	rjmp	OC0A_int1z			;skip processing RTC count

OC0A_int1a:
	LOAD16	ZL,ZH,RTCReg+0		;load 32-bits to registers ZH:ZL,YH:YL
	LOAD16	YL,YH,RTCReg+2

	ADDI32	ZH,ZL,YH,YL,1		;add 1 to count

	STORE16	RTCReg,ZL,ZH		;store new count (upper bytes)
	STORE16	RTCReg+2,YL,YH		;store new count (lower bytes)

	LOAD	RTCcntr,RTCMul		;reload RTC multiplier counter
OC0A_int1z:
;
.if DHT
OC0A_int2:	;if the DHT routines are enabled then keep track of BUSY flag
	LOAD16	ZH,ZL,RT_TMR		;load 16-bits to registers ZH:ZL
	CPI16	ZH,ZL,0				;timer = 0?
	breq	OC0A_int2a			;timer is zero

	Subi16	ZH,ZL,1				;decrement timer
	STORE16	RT_TMR,ZH,ZL
	SETB	GPIOR2,DHTBusy,temp	;set DHT busy flag
	rjmp	OC0A_int2b			;jump
;
OC0A_int2a:
	CLRB	GPIOR2,DHTBusy,temp	;
;
OC0A_int2b:
.endif

OC0A_int_out:
	SETB	GPIOR0,RTCIRQ,temp	;set the RTCIRQ flag to let others know
								; how we generated the IRQ
	pop		ZH					;restore SREG
	STORE	SREG,ZH

	POPZ						;restore all registers used
	POPY
	pop		temp
	reti						;return from interupt
;
;***********************************************************************
; RTIcommand: RTI - Set real-time counter interval
;***********************************************************************
RTIcommand:
	Gosub	interpretlinev		;Get the value onto data stack
	Gosub	popU				;retrieve # from data stack

	GoSub	CheckUisNiblVal		;check for proper range [0..3]

RTIcommand0:
;	rcall	RTC_TIMINIT			;re-init TMR0 and 32-bit counter

	tst		U					;zero?
	brne	RTIcommand1			;branch if not
	ldi		U,RTCMul_0			;default load value
	rjmp	RTIcommand9			;save the new value

RTIcommand1:
	cpi		U,1					;one?
	brne	RTIcommand2			;branch if not
	ldi		U,RTCMul_1			;10x load value
	rjmp	RTIcommand9			;save the new value

RTIcommand2:
	ldi		U,RTCMul_2			;100x load value

RTIcommand9:
	STORE	RTCMul,U			;save multiplier value in RAM
	mov		RTCcntr,U			;load RTC interrupt counter

	ret							;return to caller
;
;***********************************************************************
;RTRcommand: RTR - Resets the real-time counter to zero
;***********************************************************************
RTRcommand:
	PUSHY						;save Y-ptr
	SETYPTR	RTCReg				;point Y to RTCReg
	clr		temp				;init to zero

	st		Y+,temp				;clear 32-bits (4 bytes)
	st		Y+,temp
	st		Y+,temp
	st		Y,temp
	Goto	PopYret					;restore Y and return
;
;***********************************************************************
;RTPcommand: RTP - Print real-time counter value or assign to variable
;***********************************************************************
RTPcommand:
	push	outchar				;save all used registers
	PUSHY
	PUSHZ

	SETYPTR	RTCReg				;point Y to RTCReg
	sbrc	XL,VARSTK			;skip next if VARSTK flag clear
	rjmp	RTPcommand2			;just print full 32-bit RTC value

RTPcommand1:
	ldd		U,Y+3				;fetch lowest 8-bits
	GoSub	pushU				;save value on stack
	rjmp	RTPcommand9			;exit

RTPcommand2:
	LOAD16	ZL,ZH,RTCReg+0		;load 32-bits to registers ZH:ZL,YH:YL
	LOAD16	YL,YH,RTCReg+2
	GoSub	BIN4ASC				;convert 32-bit RTC count to ASCII
	GoSub	crlf				;send a CR/LF combo

RTPcommand9:
	POPZ						;restore all pointer registers
	POPY
	pop		outchar
	ret							;return to caller
;
.if INTREG	;only if internal variables
;************************************************************
; RT3const: returns the value of the byte of RTCReg@RAM
;	The lookup table is included for ease of decoding.
;************************************************************
RTCidxtbl:	;index table
	.db		LOW(RTCReg+3), \
			LOW(RTCReg+2), \
			LOW(RTCReg+1), \
			LOW(RTCReg+0)		;MSD to LSD
;
RTCcommand:
	rcall	Get1stParm			;get the last parameter
	GoSub	CheckUisNiblVal		;check for proper range [0..3]

	SetZPtr	(2*RTCidxtbl)		;point to index table above
	GoSub	PGMReadByte			;datastack = FLASH @Z
	ret
;
;RTCcommand9:
;	Goto	Error_0D			;error code "D"
;
.endif	;from INTREG
;
;***********************************************************************
;INITIALIZES TIM0 COUNTER AND OCR0A ENABLES INTERUPT
;***********************************************************************
RTC_TIMINIT:
	PUSHY						;save Y-ptr
	SETYPTR	RTCReg				;point Y to RTCReg
	clr		temp				;init to zero

	st		Y+,temp				;clear 32-bits (4 bytes)
	st		Y+,temp
	st		Y+,temp
	st		Y,temp

;clear interupt and zero the counter register
;enable TIMx in PRRx
#if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__)
	CLRB	RTC_PRR,RTC_PRTIM,temp	;enable TIM0 in PRR
#endif
	clr		temp				;clear all registers
	STORE	RTC_TIFR,temp
	STORE	RTC_TCNT,temp
	STORE	RTC_TCCRA,temp
	STORE	RTC_OCRA,temp
#if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__)
	STORE	RTC_TCCRB,temp
	com		temp				;set OCR0B to max count
	STORE	RTC_OCRB,temp
#endif

	;load compare count register RTC_OCR0A with default
	STOREI	RTC_OCRA,RTC_OCRA_L,temp

#if defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	;Set Waveform Generator Mode to OCR0A Interupt
	STOREI	RTC_TCCRA,(1<<WGM01|0<<COM00|0<<WGM00|RTC_CLK),temp

#else
	;Set Waveform Generator Mode to OCR0A Interupt
	STOREI	RTC_TCCRA,(0<<COM0A0|0<<COM0B0|2<<WGM00),temp

;	;Set clock source and waveform mode
	STOREI	RTC_TCCRB,(0<<RTC_WGM2|RTC_CLK),temp
#endif
	;enable Timer OCRA Interupts
	SETB	RTC_TIMSK,RTC_OCIEA,temp

RTC_TIMINIT_out:
	Goto	PopYret					;restore Y and return
;
;***********************************************************************
;DEINITIALIZES TIM0 COUNTER AND OCR0A DISABLES INTERUPT
;***********************************************************************
;RTC_TIMDINIT:
;	clr		temp				;clear registers
;	STORE	RTC_TIFR,temp
;	STORE	RTC_TCCRA,temp
;#if !defined(__ATmega16__) && \
;	!defined(__ATmega16L__) && \
;	!defined(__ATmega32__) && \
;	!defined(__ATmega32A__)
;	STORE	RTC_TCCRB,temp
;	STORE	RTC_OCRB,temp
;#endif
;	STORE	RTC_TCNT,temp
;	STORE	RTC_OCRA,temp

;	CLRB	RTC_TIMSK,RTC_OCIEA,temp
;#if !defined(__ATmega16__) && \
;	!defined(__ATmega16L__) && \
;	!defined(__ATmega32__) && \
;	!defined(__ATmega32A__)
;	SETB	RTC_PRR,RTC_PRTIM,temp		;disable TIMx in PRRx
;#endif
;RTC_TIMDINIT_out:
;	ret
;
