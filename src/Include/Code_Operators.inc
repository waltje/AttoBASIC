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
;********************************************************
; This file holds the operator commands.
;********************************************************
MULTIPLYcommand:
	rcall	GetTwoParm			;get two parameters from dstack
.if AVRhasMUL
	mul		U,temp				;8x8 unsigned hardware multiply
.else
	GoSub	MPY8u				;8x8 unsigned software multiply
.endif
	tst		r1					;check for 8-bit overflow
	SKBC	GPIOR1,AOVErr,temp	;skip next if AOVErr disabled
	brne	multiplycommand9	;branch if overflow
	mov		U,r0
	rjmp	PushURet			;exit

multiplycommand9:
	STOREI	ERRCode,$0F			;error code "F"
	Goto	error
;
;*************************************
;*	drem8u		=H			;remainder
;*	dres8u		=T			;result
;*	dd8u		=T			;dividend
;*	dv8u		=temp		;divisor
;*	dcnt8u		=U			;loop counter
;*************************************
DIVIDEcommand:
MODULUScommand:
	push	U					;save the operator we used to get here
	rcall	Get1stParm			;get the last parameter
	tst		U					;test for divide by zero
	SKBC	GPIOR1,AOVErr,r0	;skip next if AOVErr disabled
	breq	dividecommand9		;branch if divide by zero
	mov		temp,U
	GoSub	popU				;divisor 2nd off stack
	mov		T,U

	clr		H					;clear remainder and carry
	ldi		U,9					;init loop counter
dividecommand1:
	rol		T					;shift left dividend
	dec		U					;decrement counter
	breq	dividecommand4		;if done
dividecommand2:
	rol		H					;shift dividend into remainder
	sub		H,temp				;remainder = remainder - divisor
	brcc	dividecommand3		;if result negative
	add		H,temp				;restore remainder
	clc							;clear carry to be shifted into result
	rjmp	dividecommand1		;else
dividecommand3:
	sec							;set carry to be shifted into result
	rjmp	dividecommand1

dividecommand4:
	pop		U					;get out operator back
	cpi		U,('/' & $5F)		;was it a "/" (divide)?
	brne	dividecommand4a		;no
	mov		U,T					;get result into U (divide)
	rjmp	dividecommand4b

dividecommand4a:
	mov		U,H					;get remainder into U (modulus)

dividecommand4b:				;save result to data stack
	rjmp	PushURet			;exit

dividecommand9:
	STOREI	ERRCode,$11			;error code "11"
	Goto	error
;
;*************************************
ADDcommand:
	rcall	GetTwoParm			;get two parameters from dstack
	add		U,temp
	SKBC	GPIOR1,AOVErr,temp	;skip next if AOVErr disabled
	brcs	addcommand9			;branch if overflow
	rjmp	PushURet			;exit

addcommand9:
	STOREI	ERRCode,$0F			;error code "F"
	Goto	error
;
;*************************************
SUBTRACTcommand:
	rcall	GetTwoParm			;get two parameters from dstack
	sub		U,temp
	SKBC	GPIOR1,AOVErr,temp	;skip next if AOVErr disabled
;	brvs	subtractcommand9	;branch if underflow
	brcs	subtractcommand9	;branch if underflow/carry
	rjmp	PushURet			;exit

subtractcommand9:
	STOREI	ERRCode,$10			;error code "10"
	Goto	error
;
;******************************************************
EQUALcommand:		;Returns "1" if true, else "0"
	rcall	GetTwoParm			;get two parameters from dstack
	cp		U,temp
	SKBS	SREG,SREG_Z,temp	;skip next if Z set
	rjmp	makeU0				;set U to "0"
	rjmp	makeU1				;set U to "1"
;
;******************************************************
NOTEQUALcommand:	;Returns "1" if true, else "0"
	rcall	GetTwoParm			;get two parameters from dstack
	cp		U,temp
	SKBS	SREG,SREG_Z,temp	;skip next if Z set
	rjmp	makeU1				;set U to "1"
	rjmp	makeU0				;set U to "0"
;
;*************************************
DIFFcommand:
	push	U					;save operator on stack
	rcall	GetTwoParm			;get two parameters from dstack

	pop		T					;restore operator
	cpi		T,('<' & $5F)		;is it greater-than?
	brne	diffcommand1		;no, branch
	cp		U,temp				;test for less-than
	rjmp	diffcommand2

DIFFcommand1:
	cp		temp,U				;test for greater-than

DIFFcommand2:
	brcs	makeU1
;
;************************************************************************
; Supports the DIFFcommand
;************************************************************************
makeU0:	;sets U to return a "0"
	clr		U
	sbrc	U,0					;skip next if U = 0
;
makeU1:	;sets U to return a "1"
	ldi		U,1
;
makeUx:
	rjmp	PushURet			;exit
;
;********************************************************************
; LSRcommand:	Shift value right "N" bits.
;********************************************************************
LSRcommand:
	GoSub	LogShftSetup		;setup the registers
	tst		T					;did user want "0" for shift?
	breq	LSRcommand2			;yes, just pass original value back

LSRcommand1:
	lsr		U					;shift right
	dec		T					;decrement counter
	brne	LSRcommand1			;loop till zero

LSRcommand2:
	rjmp	PushURet			;exit
;
;********************************************************************
; LSLcommand:	Shift value left "N" bits.
;********************************************************************
LSLcommand:
	GoSub	LogShftSetup		;setup the registers
	tst		T					;did user want "0" for shift?
	breq	LSLcommand2			;yes, just pass original value back

LSLcommand1:
	lsl		U					;shift left
	dec		T					;decrement counter
	brne	LSLcommand1			;loop till zero

LSLcommand2:
	rjmp	PushURet			;exit
;
;************************************************************************
;LogShftSetup: Supports the logical shift operators by pulling the data
;	and shift count off the datastack.  If the shift count is invalid
;	then an error is issued.
;	- Returns with value to shift in U, shift count less one in T
;************************************************************************
LogShftSetup:
	rcall	ChkCmdParms			;check for more paramters
	GoSub	popU				;fetch parameter
	tst		cmdparms			;any parameter left?
	brne	LogShftSetup1		;yes, process
	ldi		T,1					;T = 1 to start
	ret
;
LogShftSetup1:	;assume U holds bit count
	GoSub	CheckUisByteVal		;only values of [0..7] allowed
	mov		T,U					;move count to register
	GoSub	popU				;fetch U
	ret							;return to caller
;
;************************************************************************
ANDcommand:
	rcall	GetTwoParm			;get two parameters from dstack
	and		U,temp
	rjmp	PushURet			;exit
;
;************************************************************************
ORcommand:
	rcall	GetTwoParm			;get two parameters from dstack
	or		U,temp
	rjmp	PushURet			;exit
;
;************************************************************************
XORcommand:
	rcall	GetTwoParm			;get two parameters from dstack
	eor		U,temp
	rjmp	PushURet			;exit
;
;************************************************************************
COMcommand:
	rcall	Get1stParm			;get the last parameter
	com		U
	rjmp	PushURet			;exit
;
;************************************************************************
NEGcommand:
	rcall	Get1stParm			;get the last parameter
	neg		U
	rjmp	PushURet			;exit
;
;************************************************************************
; SWAPcommand:	SWAP the high and low nibbles
;************************************************************************
SWAPcommand:
	rcall	Get1stParm			;get the last parameter
	swap	U					;swap nibbles
	rjmp	PushURet			;exit
;
;************************************************************************
;NBHcommand:	Return the high nibble
;************************************************************************
NBHcommand:
	rcall	Get1stParm			;get the last parameter
	swap	U					;swap nibbles
	andi	U,0x0F				;strip off high nibble
	rjmp	PushURet			;exit
;
;************************************************************************
.if XTRAOPS	;support for byte-size additional operators
;***********************************************
;Mark Jeronimus of Digital Mosular shares this code, which is an
;  implementation of the Galois LFSR (Linear Feedback Shift Register)
RNDcommand:	;generate an 8-bit random number
	ldi		temp,RNDPoly		;get our polynomial
	LOAD	U,RNDSeed			;get the random # from RAM
	ror		U					;rotate seed through carry
	brcc	RNDcommand1			;branch if carry clear
	eor		U,temp				;xor value and seed
RNDcommand1:
	STORE	RNDSeed,U			;save our new seed
	GoSub	pushU				;put new random # on stack
	GoSub	interpretlinev		;execute any further commands
	ret
;
;************************************************************************
;NBLcommand:	Return the low nibble
;************************************************************************
NBLcommand:
	rcall	Get1stParm			;get the last parameter
	andi	U,0x0F				;strip off high nibble
	rjmp	PushURet			;exit
;
;************************************************************************
;REVcommand:	Reverses the bit locations
;************************************************************************
REVcommand:				;reverse bit order in pwmvale.
	rcall	Get1stParm			;get the last parameter
	ldi 	temp,$80			;uses carry bit for 8 bits shift

REVcommand1:
	rol		U
	ror		temp
	brcc	REVcommand1			;loop till all bits shifted
	mov		U,temp
	rjmp	PushURet			;exit
;
.endif
;
;****************************************************************************
;CRBcommand: calculates the CRC-8 of a series of data passed on the dstack.
;CRWcommand: calculates the CRC-16 of a series of data passed on the dstack.
;	If there are no arguments passed, then the data in the data statement
;	  buffer is used.
;	Returns with:
;	 CRC8 on dstack
;	 CRC16 on dstack as CRC16L, CRC16H (little endian)
;****************************************************************************
CRCcommand:
.if	CRC16R
	clt							;clear T-Flag for CRC8
	rjmp	CRCcommand0			;calculate

CRWcommand:
	set							;set T-Flag for CRC16
.endif

CRCcommand0:
	PUSHZ						;save Z-Ptr
	clr		r1
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	brne	CRCcommand3			;parameters on dstack, continue

CRCcommand1:	;no command line parms, use DATA buffer
	LOAD	r0,DATAptr+1		;fetch number of bytes in DATA buffer
	tst		r0					;r0 = 0?
	brne	CRCcommand2

	Goto	Error_07			;value not available

CRCcommand2:
	SetZPtr	READSTOR+READSTKSZ	;Z = DATA buffer
	rjmp	CRCcommand5			;calculate CRC-8/CRC-16

CRCcommand3:
	ldi		temp,DATASTKSZ		;max DATASTKSZ parms in direct mode
	cp		cmdparms,temp		;too many parms?
	brmi	CRCcommand4			;good

	Goto	Error_06			;too many arguments

CRCcommand4:
	mov		r0,cmdparms			;copy of # of parms into r0
	SetZPtr	DATASTK+DATASTKSZ	;Z = dstack

CRCcommand5:
.if	CRC16R
	brts	CRCcommand6			;CRC16 if T-flag set
.endif
	rcall	CRC8				;calculate CRC-8

.if	CRC16R
	sub		DStkPtr,cmdparms	;adjust DStkPtr
	mov 	cmdparms,DStkPtr	;copy current position of DStkPtr
	rjmp	CRCcommand9			;save CRC8 and exit

CRCcommand6:
	rcall	CRC16				;calculate CRC-16
	mov		U,H					;save CRC16H
	GoSub	pushU				;save value on dstack
	mov		U,T					;save CRC16L
.endif

CRCcommand9:
	GoSub	PushU				;save on dstack
	POPZ						;restore Z-Ptr
	ret							;return to caller
;
;****************************************************************************
;CRC8: calculates the CRC-8 of a series of data ( x^8 + x^5 + x^4 + 1 ).
;	- Call with Y pointing to start of data in RAM.
;	- Call with byte count in r1:r0
;	- Returns with CRC-8 in U
;	Destroys temp, U, H, T, inchar, and Z pointers.
;****************************************************************************
#define	CRC8TYPE	1
#if CRC8TYPE == 1
; This code written by  E. Nicholas Cupery  Farba Research  29 April 2000
; http://www.farbaresearch.com/examples/crc8.htm
; 17 bytes, 685 cycles for 8 bytes
.set	CRC8Poly 	= 0x8C		;CRC-8 Polynomial, 0x8C = (X^8 + X^5 + X^4 + 1)
;
CRC8:
	ldi		U,CRC8Init				;start with a zero CRC-8
	ldi		H,CRC8POLY				;load CRC8POLY

CRC8a:	;begin loop to do each byte in the string
	ld		temp,-Z					;load data and decrement pointer
	ldi		T,8						;load bit counter

CRC8b:	;begin loop to do each bit in the byte
	mov		inchar,temp				;get a temporary copy of current data
	eor		inchar,U				;XOR the data byte with the current CRC
	lsr		U						;position to the new CRC
	lsr		temp					;position to next bit of this byte
	lsr		inchar					;get low bit of old result into c-bit
	brcc	CRC8c					;branch if low bit was clear (no adjustment)
	eor		U,H						;crc = crc ^ CRC8POLY;
CRC8c:
	dec		T						;count the previous bit done
	brne	CRC8b					;branch if not done all bits in this byte
		;end loop to do each bit in the byte
	dec		r0						;decrement byte counter
	brne	CRC8a					;loop till byte count reached
		;end loop to do each byte in the string
	ret								;return w/ crc in U
;
#elif CRC8TYPE == 2
; http://avr-asm.tripod.com/dalcrc.html
; 18 bytes, 813 cycles for 8 bytes
.set	CRC8Poly 	= 0x18		;CRC-8 Polynomial, 0x18 = (X^8 + X^5 + X^4 + 1)
;
CRC8:
	ldi		U,CRC8Init				;start with a zero CRC-8
	ldi		H,CRC8POLY				; CRC8POLY = 0x18
CRC8a:								;Y points to data in RAM
	ld		temp,-Z					;load data and decrement pointer
	ldi		T,8						;load bit counter
CRC8b:
	mov		inchar,temp				;feedback_bit = (crc ^ data) & 0x01
	eor		inchar,U
	ror		inchar
	mov		inchar, U
	brcc	CRC8c
	eor		inchar, H
CRC8c:
	ror		inchar
	mov		U, inchar
	lsr		temp
	dec		T						;decrement bit counter
	brne	CRC8b      				;loop till (bit_counter > 0)
	dec		r0						;decrement byte counter
	brne	CRC8a					;loop till byte count reached
	ret								;return w/ crc in U
;
#elif CRC8TYPE == 3
; This code is from Colin O'Flynn - Copyright (c) 2002
; http://www.mikrocontroller.net/attachment/131464/edm.c
; C source converted to asm by K. Scott Vitale
; 18 bytes, 805 cycles for 8 bytes
.set	CRC8Poly 	= 0x18		;CRC-8 Polynomial, 0x18 = (X^8 + X^5 + X^4 + 1)
;
CRC8:
	ldi		U,CRC8Init				;start with a zero CRC-8
	ldi		H,CRC8POLY				; CRC8POLY = 0x18
CRC8a:								;Y points to data in RAM
	ld		temp,-Z					;load data and decrement pointer
	ldi		T,8						;load bit counter
CRC8b:
	mov		inchar,temp				;feedback_bit = (crc ^ data) & 0x01
	eor		inchar,U
	sbrc	inchar,bit0    			;skip next if (feedback_bit != 1)
	eor		U,H						;crc = crc ^ CRC8POLY;
CRC8c:
 	lsr		U						;crc = (crc >> 1) & 0x7F;
	sbrc	inchar,bit0    			;skip next if (feedback_bit != 1)
	ori		U,0x80					;crc = crc | 0x80;
CRC8d:
	dec		T						;decrement bit counter
	breq	CRC8e      				;loop till (bit_counter > 0)
	lsr		temp					;data = data >> 1
	rjmp	CRC8b
CRC8e:
	dec		r0						;decrement byte counter
	brne	CRC8a					;loop till byte count reached
	ret								;return w/ crc in U
#endif
;
.if CRC16R
;****************************************************************************
;CRC16: calculates the CRC-16 of a series of data.  Set the polynomial and
; initial value in Include/Defs_Constants.inc
;	- Call with Y pointing to start of data in RAM.
;	- Call with byte count in r1:r0
;	- Returns with CRC-16 in H:T
;	Destroys temp, H, T, inbyteh, inbytel and Z pointers.
;****************************************************************************
; Author: John C. Wren
; Origin: http://www.kd4dts.com/download/zip/crc16.zip  [Crc16.asm] {broken link}
; Modified from MCS-51 to AVR by Göran Nordahl on 2000-0717
; Modified for AttoBASIC by K. Scott Vitale on 2013-1007
;****************************************************************************
CRC16:
	ldi		H,HIGH(CRC16Init)	;initialize CRC16 accumulator
	ldi		T,LOW(CRC16Init)

CRC16a:
	ldi		temp,8				;seed bit counter
	mov		bytecntr,temp
	ld		U,-Z				;load data and decrement pointer
	eor		H,U

CRC16b:
	lsl		T					;shift a bit position
	rol		H
	brcc	CRC16c				;branch if MSB = 0
	ldi		temp,HIGH(CRC16Poly);use high byte of polynomial
	eor		H,temp
	ldi		temp,LOW(CRC16Poly)	;use low byte of polynomial
	eor		T,temp
CRC16c:
	dec		bytecntr			;decrement bit counter
	brne	CRC16b				;loop till zero

	dec		r0					;decrement byte counter
	brne	CRC16a				;loop till byte count reached

	ret
.endif
;
;*****************************************************************
; Get1stParm: used by many commands to interpret the command line
;	parameters and return with the last value in U
;*****************************************************************
Get1stParm:
	GoSub	interpretlinev
	GoSub	popU				;fetch data
	ret
;
;*****************************************************************
; Calls line interpreter, fetches 1st parameter, places it in
;	temp, fetches the 2nd parameter and returns with it in U.
;*****************************************************************
GetTwoParm:
	rcall	Get1stParm			;fetch 1st parameter
	mov		temp,U				;save in temp
	GoSub	popU				;fetch 2nd parameter
	ret
;
;****************************************************************************
; ChkCmdParms: Zeros cmdparm register, calls line interpreter and tests
;	for zero parameters passed.
;****************************************************************************
ChkCmdParms:
	clr		cmdparms			;clear relative counter
	GoSub	interpretlinev		;process line parameters
	tst		cmdparms			;any new data on stack?
	ret
;
;****************************************************************************
; PopYret: Pops YH:YL off the stack and returns to caller.
;	This routine is implemented to save a few bytes for the 8KB MCU's
;****************************************************************************
PopYret:
	POPY							;restore Y
	ret								;return to caller
;
;*****************************************************************
; Pushes the contents of U onto datastack and returns.
;	DOES NOT CLOBBER Y-Ptr
;*****************************************************************
PushURet:
	GoSub	pushU				;fetch data from dstack
	ret
;
; End ************************************************************

