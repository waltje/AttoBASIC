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
; This file holds the code for the ADC and the Analog Comparator
;
;******************************************************************************
; ADCcommand:	;Perform 8 bit A to D conversion for channel specified and put 
;	result on stack.  If no channel is specified, convert channel zero.
; Uses: temp, temp2, inbyteh, inbytel, H, T and U registers
;******************************************************************************
ADCcommand:	
	push	DStkPtr				;save data stack pointer before interpreter is
								; called
	GoSub	interpretlinev		;process line
	clr 	U					;default to channel "0"
	pop		temp				;restore DStkPtr to temp
	sub		temp,DStkPtr		;DStkPtr = 0?
	breq	ADCcommand1			;yes, no parameters on the data stack, continue
	GoSub	popU				;fetch the value on the data statck

ADCcommand1:
	GoSub	ADCen				;enable ADC in PRR
	STOREI	ADCSRB,ADCSRB_L,temp;Set ADC control reg B

#if defined(__ATmega32U4__) || \
	  defined(__ATmega2560__)
	;64 channel combinations for Mega32U4 and Mega2560
	LOAD 	temp,ADMUX			;get ADCMUX
	andi	temp,~MUXMASK		;strip off MUX bits
	CLRB	ADCSRB,MUX5,T		;clear MUX5 bit
	sbrs	U,bit5				;skip next if bit 5 set
	rjmp	PC+5				;skip MACRO
	SETB	ADCSRB,MUX5,T		;set MUX5 bit
	andi	U,MUXMASK			;64/2 channel combinations for Mega32U4
#else
	LOAD 	temp,ADMUX			;get ADCMUX
	andi	temp,~MUXMASK		;strip off MUX bits
	andi	U,MUXMASK			;64 channels for tiny84/85
#endif
	or		temp,U				;mask in desired MUX channel
	STORE	ADMUX,temp			;save it

	cpi		U,0x08				;with 8 channels? (ADC[7:0])
	brpl	ADCcommand2			;yes, ignore DIDR register

	;prepare to set the ADC pin in the DIRD register
	andi	U,0x07				;only for ADC[7:0] pins
	GoSub	BitPositiontoORmask1;convert channel to bit mask
	GoSub	PopU				;retreive the bit mask off the dstack
.ifdef DIDR
	LOAD	temp,DIDR			;get the existing DIDR contents
	or		U,temp				;OR the new setting
	STORE	DIDR,U				;disable the digital input buffer
.endif
.ifdef DIDR0
	LOAD	temp,DIDR0			;get the existing DIDR contents
	or		U,temp				;OR the new setting
	STORE	DIDR0,U				;disable the digital input buffer
.endif

ADCcommand2:
	;set up to average "ADC_Avg" reads
	clr		U					;zero registers
	clr		T
	ldi		H,ADC_Avg			;load loop counter
	
	cli							;disable global IRQ's

ADCcommand3:
	STOREI 	ADCSRA,ADCSRA_L,temp;Set ADC control reg A and start a conv.
	WAIT4ADCRDY					;wait for ADC ready

	LOAD16	inbyteh,inbytel,ADCL;Get the 10 bit result
	ADD16	T,U,inbyteh,inbytel	;accumulate the readings
	dec		H					;decrement loop cpunter
	brne	ADCcommand3			;loop till zero

	sei							;re-enable global IRQ's
	
	;shift result right for 8-bit result
	ldi		H,ADC_Shft			;load loop counter
ADCcommand4:
	lsr		T					;shift result right
	ror		U
	dec		H					;decrement counter
	brne	ADCcommand4

	Gosub	pushU				;Push the data onto the stack

	GoSub	ADCden				;diable ADC hardware
	ret
;
;*********************************************************
ADRcommand:
	clr 	U
	Gosub	pushU				;Put default channel on stack
	rcall	Get1stParm			;get the last parameter
	GoSub	CheckUisBitVal		;only values of [0..1] allowed
	GoSub	ADCen				;initialize ADC
.ifdef RAMPZ					;insure lower 64K data space
	STOREI	RAMPZ,0				;clear RAMPZ register
.endif
	LOAD	temp,ADMUX			;get current MUX value
	andi	temp,~(ADC_RefM)	;mask out reference bits
	sbrc	U,0					;skip next if bit 0 set
	ori		temp,(ADC_Ext)		;ref to external
	sbrs	U,0					;skip next if bit 0 clear
	ori		temp,(ADC_Int)		;ref to internal
ADRcommand1:
	STORE	ADMUX,temp			;save to ADMUX
	GoSub	Delay1mS			;delay time for ref to settle
	ret
;
;*********************************************************
; ADCen:	Enables the ADC hardware by clearing the PRR
;*********************************************************
; This routine was moved into the file "Code_Init.inc"
;
;*********************************************************
; ADCden:	Dis-enables the ADC hardware by setting the PRR
;*********************************************************
; This routine was moved into the file "Code_Init.inc"
;
.if ( (DATAREC) && (ADConv) )
;*********************************************************
;	ADGcommand:	select CH0 or CH1 gains
;*********************************************************
ADGcommand:
	rcall	Get1stParm		;get the last parameter
	GoSub	CheckUisNiblVal		;only values of [0..3] allowed

ADGcommand1:
	mov		T,U					;move gain to T
	Gosub	popU				;get channel into U
	GoSub	CheckUisBitVal			;only values of [0..1] allowed

ADGcommand2:	;channel 0
	LOAD	temp,ADCGAIN		;get current port bits into temp
	sbrc	U,0					;skip next if bit 0 clear
	rjmp	ADGcommand3			;process channel 1
	lsl		T					;shift gain bits into bit position 2 and 3
	lsl		T
	andi	temp,~( (1<<ADC0_R0) | \
					(1<<ADC0_R1) )	;strip gain bits 0 and 1
	rjmp	ADGcommand9			;jump to combine gain

ADGcommand3:	;channel 1
	swap	T					;swap gain bits into bit position 4 and 5
	andi	temp,~( (1<<ADC1_R0) | \
					(1<<ADC1_R1) )	;strip gain bits 0 and 1
ADGcommand9:
	or		temp,T				;set new gain bits
	STORE	ADCGAIN,temp		;set gain bits to port
	ret
;
;*********************************************************
;	ADScommand: select CH0 or CH1 to AD536
;*********************************************************
ADScommand:
	clr 	U
	Gosub	pushU				;Put default channel on stack
	rcall	Get1stParm			;get the last parameter
	GoSub	CheckUisBitVal			;only values of [0..1] allowed

ADScommand1:
	CLRB	AD536IO,AD536CH		;default is channel 0
	sbrc	U,0					;skip next if bit 0 clear
	SETB	AD536IO,AD536CH		;set to channel 1
	ret
;
;*********************************************************
;	ADUcommand:	select TRMS or dBV from AD536
;*********************************************************
ADUcommand:
	clr 	U
	Gosub	pushU				;Put default channel on stack
	rcall	Get1stParm		;get the last parameter
	GoSub	CheckUisBitVal			;only values of [0..1] allowed

ADUcommand1:
	CLRB	AD536IO,AD536OS		;default is TRMS
	sbrc	U,0					;skip next if bit 0 clear
	SETB	AD536IO,AD536OS		;set to dBV readings
	ret
;
;*********************************************************
.endif
;
;
