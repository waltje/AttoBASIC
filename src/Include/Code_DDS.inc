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
.message "DDS enabled."
;*****************************************************************
; DDS: - Outputs a frequency on the defined port pin at the
;	6-BCD-digit freqency held in the X/Y/Z variables.  Called with
;	0 on the stack to disable DDS 1 on the stack to enable
;	Without a variable on the stack disables DDS as well as
;	X/Y/Z = 0.
DDScommand:
	PUSHY						;save Y (program line pointer)
	clr		U					;default to '0'
	Gosub	PushU				;save on datastack
	GoSub	Get1stParm			;get the last parameter

	GoSub	CheckUisBitVal		;branch if allowable exceeded
	sbrc	U,0					;skip next if bit 0 clear
	rjmp	DDScommand1			;enable DDS and set frequency

	rcall	DDS_TIMDINIT		;disable DDS TIMER

;*************************
; do not need to do this
;	SETYPTR	DDSaddrL			;point Y to DDS registers in RAM
;	st		Y+,r1				;clear them (r1 = 0 from TIM2_DINIT)
;	st		Y+,r1
;	st		Y,r1
;*************************

	CLRB	DDSPort,DDSOut		;clear output bit
	CLRB	DDSPort-1,DDSOut	;change DDR to input

	rjmp	DDScommand9			;exit

DDScommand1:	;enable DDS
	rcall	FRQ2DDSADD			;convert BCD (H,T,U) to DDS accumulator
								; adder values
	clr		temp				;zero DDSaccmH
	STORE	DDSacch,temp		;store to high byte of DDS accumulator

	SETB	DDSPort-1,DDSOut	;change DDR to output
	CLRB	DDSPort,DDSOut		;clear output bit

	rcall	DDS_TIMINIT			;enable DDS TIMER

DDScommand9:
	Goto	PopYret					;restore Y and return
;
;***********************************************************************
;TimerX  OCRxA Service
;	DDS Accululator Update
;***********************************************************************
;output frequency (using 24 bit accumulator) :
;
;	f = deltaPhase * fClock/2^24
;
;	fClock is in this case the CPU clock divided by the
;	number of cycles to output the data ( 5uS or 50 cycles @ 10MHz)
;
;	f = addrl/addrm/addrh * (10000000/50)/16777216
;	f = addrl/addrm/addrh * 0.0119 Hz resolution

; Original DDS code written by Jesper Hansen jesperh@telia.com
;	http://www.myplace.nu/avr/minidds/index.htm
;***********************************************************************
;	accl,accm,acch is the phase accumulator
; 	addrl,addrm,addrh is the adder value determining frequency
;
;	add value to accumulator
;	load byte from current table in RAM
;	output high bit to port
;
; Timer/Counter Compare Match A
;
DDS_int:
	push	H					;2 - save H
	LOAD	H,SREG				;2 - save SREG
	push	H					;2
	push	T					;2 - save T
	push	U					;2 - save U
	PUSHZ						;4 - save Z

.ifdef RAMPZ					;need RAMPZ to access >64KW FLASH
	ldi		H,(BYTE3(2*sqrwave));1 - set 64K page number
	STORE	RAMPZ,H				;2
.endif
	ldi		ZH,high(2*sqrwave)	;2 - point Z to last wavetable index
	LOAD	ZL,DDSacch			;1 - get accmH from RAM

	LOAD	U,DDSaddrL			;1 - get table pointer increment low byte
	LOAD	T,DDSaddrM			;1 - get table pointer increment mid byte
	LOAD	H,DDSaddrH			;1 - get table pointer increment high byte

	ADD24	acch,accm,accl,H,T,U;3 - add it all up

.ifdef RAMPZ					;need RAMPZ to access >64KW FLASH
	elpm	U,Z					;3 - get value from table
.else
	lpm		U,Z					;3 - get value from table
.endif
	STORE	DDSacch,ZL			;1 - save new table pointer to RAM
	sbrs	U,0					;2/3 - skip next if bit 0 = 1
	rjmp	DDS_int1			;2 - bit 0 = 1
	nop							;1 - for symetry
	SETB	DDSPort,DDSout		;1 Set DDS Bit out
	rjmp	DDS_int_out			;2 - exit

DDS_int1:
	CLRB	DDSPort,DDSout		;1 Clear DDS Bit out
	nop							;1 - for symetry
	nop							;1 - for symetry
DDS_int_out:
	POPZ						;4 - restore Z
	pop		U					;2 - restore U
	pop		T					;2 - restore T
	pop		H					;restore SREG
	STORE	SREG,H				;2 - save SREG
	pop		H					;2 - restore H
	reti						;4
;
;***********************************************************************
;INITIALIZES TIMx COUNTER AND OCRxA ENABLES INTERUPT
;***********************************************************************
;set up Timer
DDS_TIMINIT:
;clear interupt and zero the counter register
	clr		r1				;clear a register
#if ( !defined(__ATmega16__) && \
	  !defined(__ATmega16L__) && \
	  !defined(__ATmega32__) && \
	  !defined(__ATmega32A__) )
;enable TIMx in PRRx
	CLRB	DDS_PRR,DDS_PRTIM,temp	;enable TIMx in PRR
#endif
;	clr		temp
	STORE	DDS_TIFR,r1

#if defined (__ATtiny84__) || defined (__ATtiny84A__)
	STORE16	DDS_TCNTL,r1,r1			;16-bit register
#else
	STORE	DDS_TCNT,r1				;8-bit register
#endif
#if defined (__ATtiny85__)
	STORE	DDS_TCCR,r1
	STORE	DDS_OCRC,r1
	STORE	DDS_OCRA,r1
	STORE	DDS_OCRB,r1

;=======================================================
#elif ( defined(__ATmega16__) || \
	  defined(__ATmega16L__) || \
	  defined(__ATmega32__) || \
	  defined(__ATmega32A__) )
	STORE	DDS_TCCR,temp
	STORE	DDS_OCRA,temp

;=======================================================
#else
	STORE	DDS_TCCRA,r1
	STORE	DDS_TCCRB,r1
 #if defined (__ATtiny84__) || defined (__ATtiny84A__)
	STORE16	DDS_OCRAL,r1,r1			;16-bit regiters
	STORE16	DDS_OCRBL,r1,r1
 #else
	STORE	DDS_OCRA,r1				;8-bit regiters
	STORE	DDS_OCRB,r1
 #endif
#endif

#if defined(__ATmega32U4__)
	STORE	DDS_TCCRC,r1
	STORE	DDS_TCCRD,r1
	STORE	DDS_TCCRE,r1
;	STORE	DDS_OCRC,r1
#endif

;-----------------------------------
#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__)
	;load compare count register DDS_OCRxA with DDSCnt-1
	ldi		temp,DDSCnt-1
	STORE	DDS_OCRA,temp

	;Set Waveform Generator Mode to OCRA Interupt
	ldi		temp,(0<<COM2A0|0<<COM2B0|2<<WGM20)
	STORE	DDS_TCCRA,temp

	;Set clock source and waveform mode
	ldi		temp,(0<<DDS_WGM2|DDSPre)
	STORE	DDS_TCCRB,temp

	;enable Timer OCRA Interupts
	SETB	DDS_TIMSK,DDS_OCIEA,temp
#elif	defined(__ATmega16__) || \
		defined(__ATmega16L__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__)
	;load compare count register DDS_OCRxA with DDSCnt-1
	ldi		temp,DDSCnt-1
	STORE	DDS_OCRA,temp

	;Set Waveform Generator Mode to OCRA Interupt
	ldi		temp,(1<<WGM21|0<<COM20|0<<WGM21|DDSPre)
	STORE	DDS_TCCR,temp

	;enable Timer OCRA Interupts
	SETB	DDS_TIMSK,DDS_OCIEA,temp
;
;=======================================================
#elif 	defined(__ATtiny84__) || defined(__ATtiny84A__)
	;load compare count register DDS_OCRxA with DDSCnt-1
	ldi		temp,DDSCnt-1
	STORE16	DDS_OCRAL,r1,temp
	;enable Timer overflow interupt
	;Set clock source and waveform mode
	ldi		temp,(  (0<<DDS_COMA0)| \
					(0<<DDS_COMB0) | \
					(0<<DDS_WGM1) | \
					(0<<DDS_WGM0) )
	STORE	DDS_TCCRA,temp		;write TCCRA
	ldi		temp,(  (0<<DDS_ICNC)| \
					(0<<DDS_ICES) | \
					(0<<DDS_WGM3) | \
					(1<<DDS_WGM2) | \
					(DDSPre) )
	STORE	DDS_TCCRB,temp		;write TCCRB
	SETB	DDS_TIMSK,DDS_OCIEA,temp
;
;=======================================================
#elif 	defined(__ATtiny85__)
	;load compare count register DDS_OCRxA with DDSCnt-1
	ldi		temp,DDSCnt-1
	STORE	DDS_OCRA,temp
	STORE	DDS_OCRC,temp
	;enable Timer overflow interupt
	SETB	DDS_TIMSK,DDS_OCIEA,temp
	;Set clock source and waveform mode
	ldi		temp,(  (1<<CTC1) |\
					(0<<PWM1A) | \
					(0<<COM1A0)| \
					(DDSPre) )
	STORE	DDS_TCCR,temp
;
;=======================================================
#elif defined(__ATmega32U4__)
	;load compare count register DDS_OCRxA with DDSCnt-1
	ldi		temp,DDSCnt-1
	STORE	DDS_OCRC,temp
	;Set Waveform Generator Mode to OCRA Interupt
	ldi		temp,(0<<DDS_COMA0|0<<DDS_COMB0|0<<DDS_PWMA)
	STORE	DDS_TCCRA,temp
	;Set clock source and waveform mode
	ldi		temp,DDSPre
	STORE	DDS_TCCRB,temp
	;enable Timer OCRC Interupts
	SETB	DDS_TIMSK,DDS_OCIEA,temp
#endif

DDS_TIMINIT_out:
	ret							;return from subroutine
;
;***********************************************************************
;DEINITIALIZES TIM2 COUNTER AND OCR2A DISABLES INTERUPT
;***********************************************************************
DDS_TIMDINIT:
	clr		r1					;clear registers

	STORE	DDS_TIFR,r1
#if defined(__ATtiny85__)
	STORE	DDS_TCCR,r1
	STORE	DDS_TCNT,r1
	STORE	DDS_OCRB,r1
	STORE	DDS_OCRA,r1
#elif ( defined(__ATmega16__) || \
	  defined(__ATmega16L__) || \
	  defined(__ATmega32__) || \
	  defined(__ATmega32A__) )
	STORE	DDS_TCCR,r1
	STORE	DDS_TCNT,r1
#else
	STORE	DDS_TCCRA,r1
	STORE	DDS_TCCRB,r1

 #if defined (__ATtiny84__) || defined (__ATtiny84A__)
	STORE16	DDS_TCNTL,r1,r1		;16-bit registers
	STORE16	DDS_OCRBL,r1,r1
	STORE16	DDS_OCRAL,r1,r1
 #else
	STORE	DDS_TCNT,r1			;8-bit registers
	STORE	DDS_OCRB,r1
	STORE	DDS_OCRA,r1
 #endif
#endif

#if defined(__ATmega32U4__)
	STORE	DDS_TCCRC,r1
	STORE	DDS_TCCRD,r1
	STORE	DDS_TCCRE,r1
#endif
	CLRB	DDS_TIMSK,DDS_OCIEA,temp

#if ( !defined(__ATmega16__) && \
	  !defined(__ATmega16L__) && \
	  !defined(__ATmega32__) && \
	  !defined(__ATmega32A__) )
	SETB	DDS_PRR,DDS_PRTIM,temp		;disable TIMx in PRRx
#endif

DDS_TIMDINIT_out:
	ret
;
;***************************************************************************
;* FRQ2DDSADD - sets up the DDS frequency adder registers from
;	the 6-digit packed-BCD "current" frequency stored in H,T,U when
;	Gosubed and saves the accumulator addresses in RAM.
;***************************************************************************
FRQ2DDSADD:							;set up registers for new frequency
	SetYPtr	(VARSTOR+VARSTORSZ-3)	;point Y to last variable less 3 ('L' or 'X')
	GoSub	ClearHTU			;clear H, T & U

	ldi		temp,0xf0				;set up mask
	mov		r1,temp					;put it in r1

FRQ2DDSADD100KHZ:		;get upper digits, 100K and 10K
	ld		r0,Y					;load variable 'X' to r0 (BCDH digits)
	and		r0,r1					;strip low nible
	swap	r0						;and swap nible, 100K
FRQ2DDSADD100KHZa:
	tst		r0						;check for zero
	breq	FRQ2DDSADD10KHZ			;is zero, so skip
	ldi		inbytel,LOW(_100KHz)		;add up the count values
	ldi		inbyteh,BYTE2(_100KHz)
	ldi		temp,BYTE3(_100KHz)
	rcall	FRQ2DDSADD_24Bsub		;24-bit subtraction
	rjmp	FRQ2DDSADD100KHZa		;and loop till zero

FRQ2DDSADD10KHZ:		;get upper digits, 100K and 10K
	ld		r0,Y+					;re-load variable 'X' to r0 (BCDH digits)
	swap	r1
	and		r0,r1					;strip high nible, 10K
FRQ2DDSADD10KHZa:
	tst		r0						;check for zero
	breq	FRQ2DDSADD1KHZ			;is zero, so skip
	ldi		inbytel,LOW(_10KHz)		;add up the count values
	ldi		inbyteh,BYTE2(_10KHz)
	ldi		temp,BYTE3(_10KHz)
	rcall	FRQ2DDSADD_24Bsub		;24-bit subtraction
	rjmp	FRQ2DDSADD10KHZa		;and loop till zero

FRQ2DDSADD1KHZ:		;get middle digits, 1K and 100
	ld		r0,Y					;load variable 'Y' to r0 (BCDM digits)
	swap	r1
	and		r0,r1					;strip low nible
	swap	r0						;and swap nible, 1K
FRQ2DDSADD1KHZa:
	tst		r0						;check for zero
	breq	FRQ2DDSADD100HZ			;is zero, so skip
	ldi		inbytel,LOW(_1KHz)		;add up the count values
	ldi		inbyteh,BYTE2(_1KHz)
	ldi		temp,BYTE3(_1KHz)
	rcall	FRQ2DDSADD_24Bsub		;24-bit subtraction
	rjmp	FRQ2DDSADD1KHZa			;and loop till zero

FRQ2DDSADD100HZ:	;get middle digits, 1K and 100
	ld		r0,Y+					;re-load variable 'Y' to r0 (BCDM digits)
	swap	r1
	and		r0,r1					;strip high nible, 100
FRQ2DDSADD100HZa:
	tst		r0						;check for zero
	breq	FRQ2DDSADD10HZ			;is zero, so skip
	ldi		inbytel,LOW(_100Hz)		;add up the count values
	ldi		inbyteh,BYTE2(_100Hz)
	ldi		temp,BYTE3(_100Hz)
	rcall	FRQ2DDSADD_24Bsub		;24-bit subtraction
	rjmp	FRQ2DDSADD100HZa		;and loop till zero

FRQ2DDSADD10HZ:		;get low digits, 10 and 1
	ld		r0,Y					;load variable 'Z' to r0 (BCDL digits)
	swap	r1
	and		r0,r1					;strip low nible
	swap	r0						;and swap nible, 10
FRQ2DDSADD10HZa:
	tst		r0						;check for zero
	breq	FRQ2DDSADD1HZ			;is zero, so skip
	ldi		inbytel,LOW(_10Hz)		;add up the count values
	ldi		inbyteh,BYTE2(_10Hz)
	ldi		temp,BYTE3(_10Hz)
	rcall	FRQ2DDSADD_24Bsub		;24-bit subtraction
	rjmp	FRQ2DDSADD10HZa			;and loop till zero

FRQ2DDSADD1HZ:		;get low digits, 10 and 1
	ld		r0,Y					;re-load variable 'Z' to r0 (BCDL digits)
	swap 	r1
	and		r0,r1					;strip high nible, 1
FRQ2DDSADD1HZa:
	tst		r0						;check for zero
	breq	FRQ2DDSADD_out			;is zero, so skip
	ldi		inbytel,LOW(_1Hz)			;add up the count values
	ldi		inbyteh,BYTE2(_1Hz)
	ldi		temp,BYTE3(_1Hz)
	rcall	FRQ2DDSADD_24Bsub		;24-bit subtraction
	brne	FRQ2DDSADD1HZa			;and loop till zero

FRQ2DDSADD_out:
	STORE	DDSaddrL,U				;save addrl to RAM
	STORE	DDSaddrM,T				;save addrm to RAM
	STORE	DDSaddrH,H				;save addrh to RAM
	ret
;

;*******************************************************
; FRQ2DDSADD_24Bsub: Supports the FRQ2DDSADD routine
;*******************************************************
FRQ2DDSADD_24Bsub:
	adc		U,inbytel				;24-bit subtraction
	adc		T,inbyteh
	adc		H,temp
	dec		r0						;dec register
	ret								;return to caller
;
