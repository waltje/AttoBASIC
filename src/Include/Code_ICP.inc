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
.message "ICP enabled."
.include "Include/Defs_ICP.inc"			;include DEF's
;
;****************************************************************
;	ICP commands
;****************************************************************
;ICG [x] - set gate time for capture [0..7].  [x] is optional and
;			defaults to '0' to disable.
ICGcommand:
	clr 	U
	Gosub	pushU				;Put default on stack
	rcall	Get1stParm			;get the last parameter
	GoSub	CheckUisByteVal		;only values of [0..7] allowed
	LOAD	temp,ICPCfg			;get ICPCfg
	andi	temp,$F8			;clear gate coefficient bits
	or		U,temp				;merge new gate coefficient
	STORE	ICPCfg,U			;save gate coefficient in RAM

	;enable TIMR in PRR
 #if defined(__AT90USB1286__) || defined(__ATmega2560__)
	CLRB	ICP_PRR,PRTIM3,temp	;enable TIMER in PRR
 #elif  !defined(__ATmega16__) && \
		!defined(__ATmega16L__) && \
		!defined(__ATmega32__) && \
		!defined(__ATmega32A__)
	CLRB	ICP_PRR,PRTIM1,temp	;enable TIMER in PRR
 #endif

	;disable ICP timer interrupts, leave all others alone
	LOAD	temp,ICP_TIMSK		;fetch TIMER IRQ mask register
	andi	temp,~(1<<ICP_OCIE)	;disable ICP TIMER IRQ
	STORE	ICP_TIMSK,temp		;no interrrupts

	STORE	ICP_TIFR,temp		;clear interrupt flags
 #if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__) && \
	!defined(__ATmega644P__) && \
	!defined(__ATmega1284P__)
	STORE	ICP_TCCRC,temp
#endif
	STORE	ICP_TCCRA,temp 	;CTS mode
#if TEST	;use system clock for testing
	ldi		temp,(	(0 << ICNC1) | \
					(0 << ICES1) | \
					(0 << WGM12) | \
					(1 << CS10) )
#endif
	STORE	ICP_TCCRB,temp 		; CTC, clock on rising edge
	ret

;ICGcommand9:
;	Goto	Error_0A			;error code "A"
;
;****************************************************************
;ICE - set input capture edge select to [0..1] where 0 = falling
;		and 1 = rising edge.  [x] is optional and defaults rising
;		edge (1).
ICEcommand:
	ldi 	U,1					;default '1' for rising edge
	Gosub	pushU				;Put default channel on stack
	rcall	Get1stParm			;get the last parameter
;	ldi		temp,1				;only values of [0..1] allowed
;	cp		temp,U	
;	brmi	ICEcommand9			;branch if allowable exceeded
	GoSub	CheckUisBitVal		;only values of [0..1] allowed
	LOAD 	temp,ICPCfg			;get ICPCfg from RAM
	andi	temp,$7F			;mask off out edge select bit
	sbrs	U,0					;skip next if bit 0 set
	rjmp	ICEcommand1
	ori		temp,$80			;rising edge select
ICEcommand1:
	STORE	ICPCfg,temp			;save ICPCfg in RAM
	ret

;ICEcommand9:
;	Goto	Error_0C			;error code "C"
;
;
;****************************************************************
;ICP - start capture with 'ICG' gatetime and return low-byte of
;	ICP register, store high-byte in 'Z'
ICPcommand:
	PUSHY						;save Y since it is the program
								; pointer when running a program
	ser		T					;set T
	clr		U					;zero U
	STORE	ICP_TIFR,T			;clear TIFR flags
	STORE	ICP_TCCRA,U			;clear TCCRx registers
	STORE	ICP_TCCRB,U
 #if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__) && \
	!defined(__ATmega644P__) && \
	!defined(__ATmega1284P__)
	STORE	ICP_TCCRC,U
#endif
	STORE16	ICP_TCNTL,U,U		;zero TCNT

	CLRB	ICP_PORT-1,ICP_PIN,temp	;insure port to input

.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT			;disable USB ints
.endif

.if DDS
	LOAD	temp,DDS_TIMSK		;always disable DDS ints
	push	temp
	STOREI	DDS_TIMSK,0,temp
.endif

	LOAD 	temp,ICPCfg			;get ICPCfg from RAM
	mov		H,temp				;copy in H
	andi	H,$80				;mask out edge select bit
	clc							;clear carry flag
	rol		H					;rotate edge select bit into bit 0
	rol		H
	ori		H,$06				;merge in EXT clock bits

	andi	temp,$07			;mask out gate coefficient bits

	SetZPtr	ICPcommandTbl		;point to jump table
	ADD16	ZH,ZL,U,temp		;add gate index as offset
	ijmp						;indirect jump Z-Ptr

ICPcommandTbl:
	rjmp	ICPcommand8			;0 = ICP disabled
	rjmp	ICPcommand_0		;1 =   10mS gate time
	rjmp	ICPcommand_0		;2 =   25mS gate time
	rjmp	ICPcommand_0		;3 =   50mS gate time
	rjmp	ICPcommand_1		;4 =  100mS gate time
	rjmp	ICPcommand_1		;5 =  250mS gate time
	rjmp	ICPcommand_1		;6 =  500mS gate time
	rjmp	ICPcommand_2		;7 = 1000mS gate time

ICPcommand_0:	;10, 20 or 50 mS gate time
	cpi		temp,3				;need 50mS?
	brcs	ICPcommand_0a		;skip next if not 50mS
	ldi		temp,5				;adjust for 50mS
ICPcommand_0a:
	mov		r0,temp				;move counter into r0
	rjmp	ICPcommand_7		;execute delay

ICPcommand_1:	;100, 200 or 500 mS gate time
	ldi		T,10				;set up for multiply
	subi	temp,3				;get to 100's
	cpi		temp,3				;need 500mS?
	brcs	ICPcommand_1a		;skip next if not 500mS
	ldi		temp,5				;adjust for 500mS
ICPcommand_1a:
	mul		temp,T				;multiply
	rjmp	ICPcommand_7		;execute delay

ICPcommand_2:	;10000mS gate time
	ldi		temp,100			;set up loop counter
	mov		r0,temp				;move counter into r0

ICPcommand_7:
	STORE16	ICP_TCNTL,U,U		;zero TCNT
	STORE	ICP_TCCRB,H			;enable counting
ICPcommand_7a:
	GoSub	Delay10mS			;wait 10mS
	dec		r0					;decrement counter
	brne	ICPcommand_7a		;loop till zero

ICPcommand8:
.if DDS
	pop		temp				;restore DDS ints
	STORE	DDS_TIMSK,temp
.endif

.if USBSUPP
	GoSub	E_USBINT			;re-enable USB ints
.endif
	LOAD16	T,U,ICP_TCNTL		;get TCNT
	clr		temp				;stop ICP clock
	STORE	ICP_TCCRB,temp
	SKBC	ICP_TIFR,ICP_TOV,temp;skip next if no timer overflow
	rjmp	ICPcommand9
	SETYPTR	(VARSTOR+25)		;point Y to variable 'Z'
	st		Y,T					;save to variable 'Z'
	POPY
	Goto	PushURet			;exit

ICPcommand9:
	POPY
	STOREI	ERRCode,$0E			;error code "E"
	rcall	error
;
