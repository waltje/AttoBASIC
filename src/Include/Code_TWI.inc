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
.message "TWI enabled."
.ifdef TWDR
.include "Include/Defs_TWI.inc"			;include TWI definitions
;*******************************************************************
;	TWI interface routines
;****************************************************************************/
;  TWIcommand: TWI must be called first to initialize the TWI interface.
;	X = 0 for 400Kbps and x = 1 for 100Kbps clock.  Without [x] is same
;	as x = 0. Defaults to Master @ 400Kbps with PORT pull-ups enabled.
;	[Note: If it is desired to use alternate pull-ups, disable the PORT
;	pull-ups by clearing the SCL/SDA pins in the PORT register]
;****************************************************************************/
TWIcommand:
#if ( !defined(__ATmega16__) && \
	  !defined(__ATmega16L__) && \
	  !defined(__ATmega32__) && \
	  !defined(__ATmega32A__) )
	CLRB	PRRTWI,PRTWI,temp		;enable TWI in PRR
#endif
	clr		U						;default to 400K
	Gosub	pushU					;save it on the stack
	GoSub	Get1stParm				;interpret line and fetch 1st variable
;	ldi		temp,$01				;only values of [0..1] allowed
;	cp		temp,U
;	brmi	TWIcommand9				;branch if allowable exceeded
	GoSub	CheckUisBitVal			;only values of [0..1] allowed

;Is 400K TWI bit rate available?
.if (TWBCLK400K > 9 && TWBCLK400K <255)
	ldi		temp,TWBCLK400K			;Clock = 400K
.else
	ldi		temp,TWBCLK				;Clock = maximum determined
.endif

	sbrc	U,0						;skip next if bit 0 clear

;Is 100K TWI bit rate available?
.if (TWBCLK100K > 9 && TWBCLK100K <255)
	ldi		temp,TWBCLK100K			;Clock = 100K
.else
	ldi		temp,TWBCLK				;Clock = maximum determined
.endif

	STORE	TWBR,temp				;save bit rate

	CLRB	TWI_PORT-1,TWI_SCL 		;set SCK pin to input
	SETB	TWI_PORT,TWI_SCL 		;enable SCK pin PUP

	CLRB	TWI_PORT-1,TWI_SDA 		;set SDA pin to input
	SETB	TWI_PORT,TWI_SDA 		;enable SDA pin PUP

	clr		temp
 	STORE   TWSR, temp				;set prescaler
	com		temp					;Default content = SDA released
	STORE	TWDR,temp
	ldi		temp,((1<<TWEN)|(0<<TWIE)|(0<<TWINT)| \
				 (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC))
									;Enable TWI and release TWI pins,
									;Disable Interupt,
									;No Signal requests.
 	STORE	TWCR,temp
	TWISndStop						;reset bus by sending a STOP condition

	rcall	TWIgetstat				;Get the current bus status
	ret

TWIcommand9:
	rjmp	TWIErr0C		;error code "0C"
;
;****************************************************************************/
;  TWWcommand: TWW [x...z] or TWW "Hello" - TWW sends data to the bus.
;	Returns with the bus status on the stack.
;	This command is also used to address a slave on the bus and will
;	  continue transferring data until a NACK is received, at which time
;	  it will abort and return the bus status.
;	  Ex 1: TWW B send the data held in variable B to the previously
;	    selected slave for a write operation.
;	  Ex 2: A:= TWW $A0 $55 $00 sends $A0, $55 and $00 to the slave for
;	    writing and returns the bus status in variable A.
;	  Ex 3: A:= TWW "Hello" send the string "Hello" to the slave previously
;			addressed and returns the bus status in variable A.
;	[Note issue this command after a "TWA [x]" (SLA+W) has been issued
;	and acknowledged by the slave].
;****************************************************************************/
TWWcommand:
	rcall	ChkTWIEn			;is TWI enabled

TWWcommand1:
	PUSHY						;save Y
	SUBI16	YH,YL,1				;decrement Y pointer to next character
	ld		U,Y					;get the character
	cpi		U,'"'				;is it the end quote mark?
	brne	TWWcommand2			;no, process as numeric parms

;	GoSub	advanclbpointer		;advance past character
TWWcommand1a:	;stuff the string into the data stack
	ld		U,-Y				;fetch character, decrement Y
	cpi		U,'"'				;is it the end quote mark?
	breq	TWWcommand1b		;yes, exit loop
	GoSub	pushU				;save the character on the stack
	rjmp	TWWcommand1a		;loop for all the string data

TWWcommand1b:
	pop		U					;remove stored Y from stack
	pop		U
	GoSub	advanclbpointer		;advance past character
	mov		cmdparms,DStkPtr	;adjust cmdparms
	rjmp	TWWcommand2a		;now process as numeric parms

TWWcommand2:
	POPY						;restore Y
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	tst		cmdparms			;zero parms passed?
	brne	TWWcommand2a		;parameters on dstack, process
	rjmp	TWIErr0D			;not enough arguments

TWWcommand2a:
	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	PUSHZ						;save Z Ptr
	clr		r1					;clear a register
	SetZPtr	(DATASTK+DATASTKSZ)	;point Z to TOS
	sub		DStkPtr,cmdparms	;adjust DStkPtr
	SUB16	ZH,ZL,r1,DStkPtr	;adjust Z to point to the data

TWWcommand2b:	;disable USB or USART
.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT				;disable USB ints
.else		;USART interrupts do too!
	CLRB	UCSRB,RXCIE,temp		;disable USART RX complete IRQ
.endif

TWWcommand2c:	;set up bit counter and fetch a data byte
	ld		U,-Z				;fetch the value from the dstack
	dec		cmdparms			;decrement byte counter
	STORE	TWDR,U				;save the address in TWI data reg

 	ldi		temp,(	(1<<TWINT) | \
					(1<<TWEN) | \
					(0<<TWIE) | \
					(1<<TWEA) )	;Enable TWI-interface, ACK, clear
								; the flag to send byte
 	STORE	TWCR,temp			;init the xfer

	rcall	TWIwaitInt			;wait for command to finish

	LOAD 	temp,TWSR			;get bus status
	andi	temp,$F8			;strip all but status
	cpi		temp,TW_MTX_ADR_ACK	;TX ACK from addressed slave?
	breq	TWWcommand2d		;yes, continue

	cpi		temp,TW_MRX_ADR_ACK	;RX ACK from addressed slave?
	breq	TWWcommand2d		;yes, continue

	cpi		temp,TW_MTX_DATA_ACK;ACK on data from slave?
	brne	TWWcommand3			;no, abort

TWWcommand2d:
	tst		cmdparms			;byte counter = 0?
	brne	TWWcommand2c		;loop till zero

TWWcommand3:
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif

	POPZ						;restore Z Ptr
	POPSTAT						;restore status register
	rjmp	TWIgetstat			;Get the current bus status and return
;
;****************************************************************************/
;  TWRcommand: Receives a single byte or a series of data bytes from a slave
;		on the TWI bus, which is set to read from, and either places it onto
;		the data stack or into the DATA statement's buffer for subsequent
;		reading by the READ command.
;	[Note issue this command after a "TWS ; TWW [x]" (SLA+R) has been
;	  issued and acknowledged by the slave].
;****************************************************************************/
TWRcommand:
	rcall	ChkTWIEn			;is TWI enabled

	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	brne	TWRcommand1			;parameters on dstack, process as DATA
	SETB	GPIOR2,TWRSngl,temp2;set the TWI single byte flag
;	rjmp	TWIErr0D			;not enough arguments

TWRcommand1:	;fetch multiple bytes and stuff in DATA buffer
.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT			;disable USB ints
.else		;USART interrupts do too!
	CLRB	UCSRB,RXCIE,temp	;disable USART RX complete IRQ
.endif
	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	PUSHZ						;save Z
	clr		r1					;clear a register

	ldi		U,1					;pre-load byte xfer count to 1
	SKBC	GPIOR2,TWRSngl,temp2;skip next if multi-byte TWR
	rjmp	TWRcommand1a		;continue as single-byte TWR

	ldi		temp,DATASTKSZ+1	;set max data buffer size
	GoSub	popU				;fetch # of bytes from stack
	cp		U,temp				;is the value provided larger than buffer?
	brmi	TWRcommand1a		;no, continue
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif
	rjmp	TWIErr16			;specified value exceeded range

TWRcommand1a:
	mov		H,U					;copy payload size into H
	SKBC	GPIOR2,TWRSngl,temp2;skip next if multi-byte TWR
	rjmp	TWRcommand1b		;continue with data retrieval

	SetZPtr	(READSTOR+READSTKSZ);Z = top of DATA buffer
	STORE	DATAptr,r1			;clear DATA read pointer
	STORE	DATAptr+1,U			;save payload size as # of DATA elements

TWRcommand1b:
	;set up registers for proper slave response - normally an ACK
	ldi		temp,((1<<TWEN)|(1<<TWINT)|(1<<TWEA));load TWCR (with ACK)
	ldi   	T,TW_MRX_DATA_ACK	;ACK for "N" data bytes

TWRcommand2:	;start of data xfer loop
	dec		H					;decrement byte counter
	brne	TWRcommand2a		;no, NACK only on last byte

	;set up registers for proper slave response - NACK
	ldi		temp,((1<<TWEN)|(1<<TWINT)|(0<<TWEA));load TWCR (with NACK)
	ldi   	T,TW_MRX_DATA_NACK	;NACK for last data byte
	ser		H					;set 0xff to H (as last byte indicator)

TWRcommand2a:
	STORE	TWCR,temp			;init the xfer

	rcall	TWIwaitInt			;wait for command to finish

	LOAD 	U,TWSR				;get bus status
	andi	U,$F8				;strip all but status
	cpse	U,T					;test for proper response
	rjmp  	TWRcommand2d		;bus error, process it

TWRcommand2b:	;TW_MRX_DATA_ACK/NACK recieved
	LOAD	U,TWDR				;get the data byte
	SKBC	GPIOR2,TWRSngl,temp2;skip next if multi-byte TWR
	rjmp	TWRcommand2c		;data byte in U, finish
	st		-Z,U				;store in DATA space
	sbrs	H,bit7				;skip next if bit7 set (last byte)
	rjmp	TWRcommand2			;loop for more
	rjmp	TWRcommand3			;done, cleanup and exit

TWRcommand2c:
	GoSub	pushU				;save on stack

TWRcommand2d:
	SKBS	GPIOR2,TWRSngl,temp2;skip next if single-byte TWR
	STORE	DATAptr+1,r1		;save payload size as 0, failed read

TWRcommand3:
	POPZ						;restore Z

.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif

	POPSTAT						;restore STATUS reg
	SKBS	GPIOR2,TWRSngl,temp2;skip next if single-byte TWR
	rjmp	TWIgetstat			;get bus status and return
	CLRB	GPIOR2,TWRSngl,temp2;clear the TWI single byte flag
	ret							;return to caller
;
;****************************************************************************/
;  TWScommand: Assert a START condition on the bus.  When the TWI interface
;	is initialized, a START condition is asserted.  Returns with the bus status
;	on the stack.  However, the user must re-assert a START condition after a
;	STOP condition to ready the bus for the next message sequence.
;****************************************************************************/
TWScommand:
	rcall	ChkTWIEn			;is TWI enabled

	andi	temp,~TWINT			;clear TWINT flag
	STORE	TWCR,temp

	TWISndStart					;assert a STOP condition

	rcall	TWIwaitInt			;wait for command to finish

	rcall	TWIgetstat			;get bus status
	ret
;
;****************************************************************************/
;  TWPcommand: Assert a STOP condition on the bus.  The user must assert
;	a STOP condition after the last message byte has been sent to or received
;	from the slave or to abort a transfer in progress.
;****************************************************************************/
TWPcommand:
	rcall	ChkTWIEn			;is TWI enabled

	TWISndStop					;assert a STOP condition

	ldi		temp,$FF			;write data reg ro release SDA
	STORE	TWDR,temp
	rcall	TWIgetstat			;Get the current bus status
	ret							;return to caller
;
;****************************************************************************/
;  TWBcommand: Queries the TWI status register for the last detected
;	condition of the bus.  [Note: the byte returned is right-
;	shifted 3 bit positions. If a STOP condition has been detected,
;	$01 is returned to indicate so. Ex: A:= TWB (if A = 3 then SLA+W
;	has been transmitted and an ACK received).
;****************************************************************************/
TWBcommand:
	LOAD	U,TWCR				;get TWCR
	sbrs 	U,TWEN				;skip next if TWEN set
	rjmp	TWIErr13			;TWI not enabled, notify user
	rcall	TWIgetstat			;Get the current bus status
	ret							;return to caller
;
;*****************************************************************
;TWIgetstat: fetch the TWI status register
;*****************************************************************
TWIgetstat:
	LOAD	U,TWSR				;Get current bus status
	andi	U,$F8				;strip all but status
;	lsr		U					;shift status right 3 bit positions
;	lsr		U
;	lsr		U
	Goto	PushURet			;save on dstack then exit
;
;*****************************************************************
;TWIwaitInt: wait for TWINT flag to set.  Timeout if no response
;*****************************************************************
TWIwaitInt:
	PUSHZ						;save Z Ptr
	SetZPtr	TWI_TIMEOUT*1000/5	;load timeout with 5uS delays

 TWIwaitInt1:
	SKBC	TWCR,TWINT,r0		;skip next if TWINT flag clear
 	rjmp	TWIwaitInt9			;loop
	GoSub	Delay5uS			;delay 5uS
	SUBI16	ZH,ZL,1				;decrement timeout counter
	brne	TWIwaitInt1			;loop till zero

	;no response from device, abort
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif
	STOREI	ERRCode,$18		;error code "18"
	Goto	error

 TWIwaitInt9:
	POPZ						;restore Z Ptr
	ret							;return to caller
;

;*****************************************************************
; ChkTWIEn: is TWI enabled?
;*****************************************************************
ChkTWIEn:
	LOAD	temp,TWCR			;get TWCR
	sbrs 	temp,TWEN			;skip next if TWEN set
	rjmp	TWIErr13			;TWI not enabled, notify user
	ret
;
;*****************************************************************
; TWI error processing.
;*****************************************************************
;TWIErr12:	;Bus error
;	Goto	Error_12		;error code "12"

TWIErr13:	;TWI not enabled
	STOREI	ERRCode,$13		;error code "13"
	Goto	error

TWIErr16:	;Specified value exceeded range
	Goto	Error_16		;error code "16"

TWIErr0C:	;Exceeded range of 0:1
	Goto	Error_0C		;error code "C"

TWIErr0D:	;not enough arguments
	Goto	Error_1D
;
;****************************************************************
  .endif
