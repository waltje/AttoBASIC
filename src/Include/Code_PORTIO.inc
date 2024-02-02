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
;it is more efficient to use original PORT I/O for tiny84/85
#if defined(__ATtiny85__) || \
	defined(__ATtiny84__) || defined(__ATtiny84A__)	
  .message "Single-port PIO enabled."
;  .ifdef PORTA || ifdef PORTB
;*****************************
 .ifdef PORTA
inAcommand:					;input from PINA
	LOAD	U,PINA
	Goto	PushURet		; save U and return
 .endif

 .ifdef PORTB
inBcommand:					;input from PINB
	LOAD	U,PINB
	Goto	PushURet		; save U and return
 .endif
;
;*****************************
outpAcommand:				;Output to PORTB
outpBcommand:				;Output to PORTB
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParm
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for DDRB
	rjmp	outpBcommand1	;for DDRB
	STORE	PORTA,U
	ret
 .endif
outpBcommand1:
	STORE	PORTB,U
	ret
;
;*****************************
 .ifdef PORTA
inDirectionA:				;input from DDRA
	LOAD	U,DDRA
	Goto	PushURet		; save U and return
 .endif

 .ifdef PORTB
inDirectionB:				;input from DDRB
	LOAD	U,DDRB
	Goto	PushURet		; save U and return
 .endif
 ;
;*****************************
outdAcommand:				;Output to DDRA/B
outdBcommand:
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParm
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for DDRB
	rjmp	outdBcommand1	;for DDRB
	STORE	DDRA,U
	ret
 .endif
outdBcommand1:
	STORE	DDRB,U
	ret
;
;*****************************
setdirectionA:				;Set bit in DDRA/B
setdirectionB:
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParmOred
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for DDRB
	rjmp	setdirection1	;for DDRB
	SetZPtr	DDRA+32			;Z = DDRA
	rjmp	ORandSTORE		;XOR U and data@port then store it
 .endif
 setdirection1:
	SetZPtr	DDRB+32			;Z = DDRB
	rjmp	ORandSTORE		;XOR U and data@port then store it
;
;*****************************
cleardirectionA:			;Clear bit in DDRA/B
cleardirectionB:
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParmOred
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for DDRB
	rjmp	cleardirection1	;for DDRB
	SetZPtr	DDRA+32			;Z = DDRA
	rjmp	ANDandSTORE		;AND U and data@port then store it (and return)
 .endif
cleardirection1:
	SetZPtr	DDRB+32			;Z = DDRB
	rjmp	ANDandSTORE		;AND U and data@port then store it (and return)
;
;*****************************
setbitinA:				;Set bit in position inidicated at top of stack.
setbitinB:
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParmOred
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for PORTB
	rjmp	setbitin1		;for PORTB
	SetZPtr	PORTA+32		;Z = PORTA
	rjmp	ORandSTORE		;XOR U and data@port then store it
 .endif
setbitin1:				;Set bit in position inidicated at top of stack.
	SetZPtr	PORTB+32		;Z = PORTB
	rjmp	ORandSTORE		;XOR U and data@port then store it
;
;*****************************
clearbitinA:				;Clear bit in position inidicated at top of stack.
clearbitinB:
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParmOred
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for PORTB
	rjmp	clearbitin1		;for PORTB
	SetZPtr	PORTA+32		;Z = PORTA
	rjmp	ANDandSTORE		;AND U and data@port then store it (and return)
 .endif
clearbitin1:
	SetZPtr	PORTB+32		;Z = PORTB
	rjmp	ANDandSTORE		;AND U and data@port then store it (and return)
;
;*****************************
testbitinA:					;Test bit in PIN
testbitinB:					;Test bit in PIN
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParmOred
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for PORTB
	rjmp	testbitin1		;for PORTB
	SetZPtr	PORTA+32		;Z = PORTA
	rjmp	testbitin2
 .endif
testbitin1:
	SetZPtr	PORTB+32		;Z = PORTB
testbitin2:
	ld		temp,Z			;get PINx data into temp
	and		temp,U			;strip mask desired bit from PINx data
	ldi		U,0				;preload U = 0
	tst		temp			;PINx bit cleared?
;	SKBS	SREG,SREG_Z,r0	;skip next if Z is set
	brne	testbitin3		;branch if not zero
	ldi		U,1				;set U = 1
testbitin3:
	Goto	PushURet		; save U and return
;
;*****************************
xorbitinA:					;XOR bit in B
xorbitinB:					;XOR bit in B
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	rcall	Get1stParmOred
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for PORTB
	rjmp	xorbitin1		;for PORTB
	SetZPtr	PORTA+32		;Z = PORTA
	rjmp	XORandSTORE		;XOR U and data@port then store it
 .endif
xorbitin1:
	SetZPtr	PORTB+32		;Z = PORTB
	rjmp	XORandSTORE		;XOR U and data@port then store it
;
;*****************************
plsbitinA:					;Pulse bit in position inidicated at top of stack.
plsbitinB:					;Pulse bit in position inidicated at top of stack.
 .ifdef PORTA
 	mov		temp2,U			;copy last character of command to temp2
 .endif
	GoSub	Get1stParmOred	;get bit position
 .ifdef PORTA
	sbrs	temp2,bit0		;bit0=0 for PORTB
	rjmp	plsbitin1		;for PORTB
	SetZPtr	PORTA+32		;Z = PORTA
	rjmp	plsbitin2
 .endif
plsbitin1:
	SetZPtr	PORTB+32		;Z = PORTB
plsbitin2:
	rcall	XORandSTORE		;XOR U and data@port then store it
	GoSub	Delay10mS		;delay 10mS
	rjmp	XORandSTORE		;XOR U and data@port then store it (and return)
;
;**********************************************************
; Calls line interpreter, calculates the bit-OR mask, fetches 1st parameter
;	on the and returns with it in U.
Get1stParmOred:
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	ret
;  .endif
;**********************************************************
;
#else
 .message "Multi-Port PIO enabled."
 .if DATAREC
;**********************************************************
;DIGcommand - Digital OUT for AVR Data Recorder
;**********************************************************
DIGcommand:
	rcall	Get1stParm		;get the last parameter
	GoSub	CheckUisBitVal	;check for proper range [0..1]

	SetZPtr	DIGPORT+$20		;Point Z to port (+ offset)
	mov		T,U				;copy to T
	ldi		U,(1<<DIGOUT)	;set bit mask
	sbrc	T,0				;skip next if clearing port bit
	rjmp	ORandSTORE		;OR U and data@port then store it
	rjmp	ANDandSTORE		;AND U and data@port then store it
 .endif
;
;***********************************************************
inDirectionX:
	rcall	GetPortOffset	;calculate port offset
.ifndef PORTH
	ADDI16	ZH,ZL,$21		;add $21 for DDRx offset
.else
	ADDI16	ZH,ZL,1			;add 1 for DDRx offset
.endif
	ld		U,Z				;get data into U
	Goto	PushURet		; save U and return
;
;***********************************************************
outdXcommand:					;Output to DDRx
	rcall	GetPortOffset		;calculate port offset
	PUSHZ						;save z pointer
	rcall	Get1stParm			;get the last parameter
	POPZ						;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$21			;add $21 for DDRx offset
.else
	ADDI16	ZH,ZL,1				;add 1 for DDRx offset
.endif

#if (defined(__ATmega32U4__) || \
		defined(__ATmega16__) || defined(__ATmega16L__) || \
	  	defined(__ATmega32__) || defined(__ATmega32A__) || \
		defined(__ATmega644P__) || defined(__ATmega1284P__) || \
 		defined(__ATmega2560__) || \
 		defined(__AT90USB1286__) )
  .ifdef DDRC
	CPI16	ZH,ZL,(DDRC+0x20)	;is it port C?
	breq	outdXcommand1		;yes, continue
  .endif
  .ifdef DDRF
	CPI16	ZH,ZL,(DDRF+0x20)	;is it port F?
	brne	outdXcommand3		;not, continue
  .endif

outdXcommand1:					;Output to DDRx
	PUSHZ						;save Z pointer
	;accessing DDRF so look at the JTAG interface
	SETZPTR	FUSEBYTE			;set Z to High Fuse Byte
  #if 	defined(__ATmega16__) || \
		defined(__ATmega16A__) || \
		defined(__ATmega16L__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__) || \
		defined(__ATmega32L__)	;
	STOREI	SPMCR,(1<<SPMEN | 1<<BLBSET);SPM and BLB Enabled
  #else
	STOREI	SPMCSR,(1<<SPMEN | 1<<BLBSET);SPM and BLB Enabled
  #endif
	lpm		temp,Z				;get high fuse into temp
	POPZ						;restore Z pointer
	sbrs 	temp,OCDEN			;skip next if OCDEN fuse is not programmed, 
								;  which could mean we are debugging code
	jmp		outdXcommand2		;continue
	;not debugging, so disable the JTAG interface
	SETB	MCUCR,JTD,temp		;set the JTD flag to
	SETB	MCUCR,JTD,temp		;disable the JTAG interface
	jmp		outdXcommand3		;continue

outdXcommand2:					;Output to DDRx
	;JTAG pins being used for debug, only adjust non-JTAG pins
	andi	U,~(1<<J_TCK | 1<<J_TMS | 1<<J_TDO | 1<<J_TDI)

#endif
;
outdXcommand3:					;Output to DDRx
	st		Z,U					;store to port
	ret
;
;***********************************************************
setdirectionX:				;Set bit in position inidicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$21		;add $21 for DDRx offset
.else
	ADDI16	ZH,ZL,1			;add 1 for DDRx offset
.endif
	rjmp	ORandSTORE		;OR U and data@port then store it
;
;***********************************************************
cleardirectionX:			;Clear bit in position inidicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$21		;add $21 for DDRx offset
.else
	ADDI16	ZH,ZL,1			;add 1 for DDRx offset
.endif
	rjmp	ANDandSTORE		;AND U and data@port then store it
;
;***********************************************************
outpXcommand:				;Output to PORTx
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	rcall	Get1stParm		;get the last parameter
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$22		;add $22 for PORTx offset
.else
	ADDI16	ZH,ZL,2			;add 2 for PORTx offset
.endif
	st		Z,U				;store to port
	ret
;
;***********************************************************
inXcommand:
	rcall	GetPortOffset	;calculate port offset
.ifndef PORTH
	ADDI16	ZH,ZL,$20		;add $20 for PINx offset
.endif
	ld		U,Z				;get data into U
	Goto	PushURet		; save U and return
;
;***********************************************************
setbitinX:					;Set bit in position inidicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$22		;add $22 for PORTx offset
.else
	ADDI16	ZH,ZL,2			;add 2 for PORTx offset
.endif
	rjmp	ORandSTORE		;OR U and data@port then store it
;
;***********************************************************
clearbitinX:				;Clear bit in position inidicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$22		;add $22 for PORTx offset
.else
	ADDI16	ZH,ZL,2			;add 2 for PORTx offset
.endif
	rjmp	ANDandSTORE		;AND U and data@port then store it

;***********************************************************
xorbitinX:					;XOR bit in position inidicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$22		;add $22 for PORTx offset
.else
	ADDI16	ZH,ZL,2			;add 2 for PORTx offset
.endif
	rjmp	XORandSTORE		;XOR U and data@port then store it
;
;***********************************************************
plsbitinX:					;Pulse bit in position inidicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$22		;add $22 for PORTx offset
.else
	ADDI16	ZH,ZL,2			;add 2 for PORTx offset
.endif
	rcall	XORandSTORE		;XOR U and data@port then store it
	GoSub	Delay10mS		;delay 10mS
	rcall	XORandSTORE		;XOR U and data@port then store it
	ret						;return to caller
;
;***********************************************************
testbitinX:					;Test bit in specified position indicated at top of stack.
	rcall	GetPortOffset	;calculate port offset
	PUSHZ					;save z pointer
	Gosub	interpretlinev
	Gosub	bitpositiontoormask
	Gosub	popU
	POPZ					;restore Z
.ifndef PORTH
	ADDI16	ZH,ZL,$20		;add $20 for PINx offset
.endif
	ld		temp,Z			;get PINx data into temp
	and		temp,U			;strip mask desired bit from PINx data
	ldi		U,0				;preload U = 0
	tst		temp			;PINx bit cleared?
	SKBS	SREG,SREG_Z,r0	;skip next if Z is set
	ldi		U,1				;set U = 1
	Goto	PushURet		; save U and return
;
;****************************************************************************
GetPortOffset:				;U holds port [A..L]
	subi	U,'A'			;get port addressed by subtracting 'A'
  #if	defined(__ATtiny84__) || \
		defined(__ATtiny84A__) || \
		defined(__ATmega16__) || \
		defined(__ATmega16L__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__) || \
		defined(__ATmega2560__)
	clr		r1				;clear r1
	SetZPtr	(2*PortOffsetTbl);point ZH:ZL to lookup table
	lsl		U				;multiply U by 2 for index
	ADD16	ZH,ZL,r1,U		;add to Z-Ptr
	lpm		r0,Z+			;fetch port low address to r0
	lpm		r1,Z			;fetch port high address to r1
  #else
	ldi		temp,3			;load port to port offset address
.if AVRhasMUL	;for cores with MUL instruction
	mul		U,temp			;get port index into r1:r0
.else			;for cores without MUL instruction
	GoSub	MPY8u			;get port index into r1:r0
.endif
  #endif
	movw	ZH:ZL,r1:r0		;move r1:r0 into Z pointer
	ret						;return
;
;****************************************************************************
  .ifdef PORTH
PortOffsetTbl:	;table of port locations for "GetPortOffset" routine
	.dw		PINA+32			;Port A
	.dw		PINB+32			;Port B
	.dw		PINC+32			;Port C
	.dw		PIND+32			;Port D
	.dw		PINE+32			;Port E
	.dw		PINF+32			;Port F
	.dw		PING+32			;Port G
	.dw		PINH			;Port H
	.dw		0x0000			;Port I (no port I ! )
	.dw		PINJ			;Port J
	.dw		PINK			;Port K
	.dw		PINL			;Port L
  .endif

;****************************************************************************
  #if defined(__ATtiny84__) || \
		defined(__ATtiny84A__)
PortOffsetTbl:	;table of port locations for "GetPortOffset" routine
	.dw		PINA			;Port A
	.dw		PINB			;Port B
  #elif defined(__ATmega16__) || \
		defined(__ATmega16L__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__)
PortOffsetTbl:	;table of port locations for "GetPortOffset" routine
	.dw		PINA			;Port A
	.dw		PINB			;Port B
	.dw		PINC			;Port C
	.dw		PIND			;Port D
  #endif
#endif
;
;****************************************************************************
ORandSTORE:					;Z holds port pointer.  OR U with data@port
	ld		temp,Z			;get data in register
	or		temp,U			;set the appropriate bit
	st		Z,temp			;save it back in the register
	ret
;
;****************************************************************************
ANDandSTORE:				;Z holds port pointer.  AND U with data@port
	com		U				;compliment for AND mask
	ld		temp,Z			;get data in register
	and		temp,U			;clear the appropriate bit
	st		Z,temp			;save it back in the register
	ret

;****************************************************************************
XORandSTORE:				;Z holds port pointer. XOR U with data@port
	ld		temp,Z			;get data in register
	eor		temp,U			;set the appropriate bit
	st		Z,temp			;save it back in the register
	ret
;
;****************************************************************************
