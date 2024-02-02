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
.include "Include/Defs_DATAFILE.inc"	;include needed defs
;
;*******************************************************************
; DFRcommand: Read data from random address
;	inbyteh:l	= address counter
; Uses: U, inbyteh and inbytel
;*******************************************************************
DFRcommand:
	GoSub	Get1stParm			;get the last parameter
	mov		inbyteh,U			;move page # to inbyteh
	Gosub	popU				;retrieve page address from stack
	mov		inbytel,U			;move address to inbytel

	;init the SPI interface to Mode 0, MSB first, Fclk/16
	Gosub	SPIinit				;init the SPI interface to "mode 0"
	CLRB	SPI_PORT,SPI_SS		;drop SS line low

	ldi		U,DA_READ			;read instruction
	GoSub	SPI_write			;send it
	GoSub	SPI_WrtAddr			;write the address to the device
	GoSub	SPI_read			;read data value into "U"
	SETB	SPI_PORT,SPI_SS		;raise SS line to end command
	Goto	PushURet			;exit
;
;*******************************************************************
; DFWcommand: Write data to random address
;	inbyteh:l	= address counter
; Uses: U, T, inbyteh and inbytel
;*******************************************************************
DFWcommand:
	GoSub	Get1stParm			;get the last parameter
	mov		inbyteh,U			;move page # to inbyteh
	Gosub	popU				;retrieve page address from stack
	mov		inbytel,U			;move address to inbytel

DFWcommand1:
	;init the SPI interface to Mode 0, MSB first, Fclk/16
	Gosub	SPIinit				;init the SPI interface to "mode 0"
	CLRB	SPI_PORT,SPI_SS		;drop SS line low

	ldi		U,DA_WREN			;enable writes
	GoSub	SPI_write			;send it
	SETB	SPI_PORT,SPI_SS		;raise SS line to end instruction

	ldi		U,DA_WRIT			;read instruction
	GoSub	SPI_write			;send it
	GoSub	SPI_WrtAddr			;write the address to the device
	Gosub	popU				;retrieve data byte from stack
	GoSub	SPI_write			;send data in "U"

	SETB	SPI_PORT,SPI_SS		;raise SS line to end command

DFWcommand2:
	GoSub	Delay1mS			;delay 1mS
	ldi		U,DA_RDSR			;read status register instruction
	GoSub	SPI_write			;send data in "U"
	GoSub	SPI_read			;read status value into "U"
	SETB	SPI_PORT,SPI_SS		;raise SS line to end command
	tst		U					;is zero?
	brne	DFWcommand2			;no, loop till zero
	ret							;return to caller
;
;*******************************************************************
; DFLcommand: log data to the address held in the internal
;	inbyteh:l	= byte counter
; Uses: U, inbyteh and inbytel
;*******************************************************************
DFLcommand:
	clr		U					;set default to "0"
	Gosub	pushU				;save on stack
	Gosub	interpretlinev		;Get the value onto data stack

	LOAD16	inbyteh,inbytel,DFLaddr	;get current address
	Gosub	DFWcommand1			;write the data
	ADDI16	inbyteh,inbytel,1	;increment pointer
	STORE16	DFLaddr,inbyteh,inbytel	;save new address
	ret							;return to caller
;
;*******************************************************************
; DFXcommand: reset the internal address counter to "0"
;	- Destroys r0
;*******************************************************************
DFXcommand:
	clr		r0					;clear register
	STORE16	DFLaddr,r0,r0		;zero the internal address counter
	ret							;return to caller
;
;*******************************************************************
; DFFcommand: set data dump format to "n" bytes
; Uses: U
;*******************************************************************
DFFcommand:
	ldi		U,1					;set default to "1"
	Gosub	pushU				;save on stack
	GoSub	Get1stParm			;get the last parameter
	tst		U					;user entered "0"
	breq	DFFcommand9			;branch if allowable exceeded
	STORE	DFDfmt,U			;save the value
	ret							;return to caller
;
DFFcommand9:
	Goto	Error_16		;error code "16"
;
;*******************************************************************
; DFDcommand: Read sequential data from address and print to output
;	device using the format specified with the DFF command.
;	inbyteh:l	= byte counter
; Uses: r0, r1, H, T, U, inbyteh and inbytel.
;*******************************************************************
DFDcommand:
	GoSub	Get1stParm			;get the last parameter
	tst		U					;did user enter 0?
	breq	DFDcommand9			;error condition
	mov		inbyteh,U			;move page count to inbyteh
	clr		inbytel				;clear byte counter for page

	Gosub	popU				;retrieve page count from stack
	COPY16	r1,r0,inbyteh,inbytel;move to r1:r0
	mov		inbyteh,U			;move page # to inbyteh
								;address low is already "0" in inbytel
	;init the SPI interface to Mode 0, MSB first, Fclk/16
	Gosub	SPIinit				;init the SPI interface to "mode 0"
	GoSub	Delay1ms			;wait for I/O to settle
	CLRB	SPI_PORT,SPI_SS		;drop SS line low

	ldi		U,DA_READ			;read instruction
	GoSub	SPI_write			;send it

	GoSub	SPI_WrtAddr			;write the address to the device
	COPY16	inbyteh,inbytel,r1,r0	;get byte counter into inbyteh:l

	LOAD	bytecntr,DFDfmt		;set the field counter

DFDcommand1:
	GoSub	SPI_read			;read data value into "U"
	GoSub	binarytodecimal		;convert U to decimal in H:T:U
	GoSub	sendHTUasdecimal	;xmit H:T:U value
	SUBI16	inbyteh,inbytel,1	;decrement byte counter
	breq	DFDcommand2			;branch if zero

	dec		bytecntr			;decrement field counter
	brne	DFDcommand1a		;no, branch
	GoSub	crlf				;send a CR/LF combo
	LOAD	bytecntr,DFDfmt		;reload the field counter
	rjmp	DFDcommand1b		;CR/LF sent, check for keys

DFDcommand1a:
	ldi		outchar,DFD_SEP		;data separator
	GoSub	emitchar			;send it!

DFDcommand1b:
	sbrs	XH,HALT				;skip next if "Halt flag" set
	rjmp	DFDcommand1			;loop for more
	GoSub	breakmessage		;xmit "break" message
	andi	XH,~(1<<HALT)		;Clear the "halt flag"

DFDcommand2:
	SETB	SPI_PORT,SPI_SS		;raise SS line to end command
	GoSub	crlf				;ending CR/LF combo
	ret							;return to caller
;
DFDcommand9:
	Goto	Error_11			;error code "11"
;
;*******************************************************************
; DFVcommand: set the initialization value for the DFI command
; Uses: U
;*******************************************************************
DFVcommand:
	clr		U					;set default to "0"
	Gosub	pushU				;save on stack
	GoSub	Get1stParm			;get the last parameter
	STORE	DFIbyte,U			;save the value
	ret							;return to caller
;
;*******************************************************************
; DFIcommand: Initialize Data File by programming zero to each byte
;	in a page for "n" pages.
; 	H			= byte counter for device page buffer
;	r1:r0		= # of 256-byte pages to erase
;	inbyteh:l	= address counter
; Uses: r0, r1, temp, U, H, YH and YL
;*******************************************************************
DFIcommand:
	clr		U					;default to page "0"
	Gosub	pushU				;save it on the stack
	ldi		U,1					;default to "1" page
	Gosub	pushU				;save it on the stack
	GoSub	Get1stParm			;get the last parameter
	tst		U					;"0" is illegal!
	brne	DFIcommand1

	Goto	Error_11			;"0" not allowed, error code "11"

DFIcommand1:
	ldi		temp,(256/DA_PGSZ)	;256-byte page size / device page size
	mul		temp,U				;multiply for # of device pages

	Gosub	popU				;retrieve page # from data stack
	mov		inbyteh,U			;set starting address
	clr		inbytel

	;init the SPI interface to Mode 0, MSB first, Fclk/16
	Gosub	SPIinit				;init the SPI interface to "mode 0"
	CLRB	SPI_PORT,SPI_SS		;drop SS line low

DFIcommand2:
	ldi		U,DA_WREN			;enable writes
	GoSub	SPI_write			;send it
	SETB	SPI_PORT,SPI_SS		;raise SS line to end instruction

	ldi		U,DA_WRIT			;write instruction
	GoSub	SPI_write			;send it

	GoSub	SPI_WrtAddr			;write the address to the device
	LOAD	U,DFIbyte			;use value stored in RAM@DFIbyte
	ldi		H,DA_PGSZ			;load device's internal page buffer size
DFIcommand2a:	; Use device's "page write" mode
	GoSub	SPI_write			;send data in "U"
	dec		H
	brne	DFIcommand2a		;loop till byte counter is zero
	SETB	SPI_PORT,SPI_SS		;raise SS line to write buffer

DFIcommand3:
	GoSub	Delay10uS
	ldi		U,DA_RDSR			;read status register instruction
	GoSub	SPI_write			;send data in "U"
	GoSub	SPI_read			;read status value into "U"
	SETB	SPI_PORT,SPI_SS		;raise SS line to end command
	tst		U					;is zero?
	brne	DFIcommand3			;no, loop till zero
	ADDI16	inbyteh,inbytel,DA_PGSZ	;add device page size to address

	PUSHY						;save Y-pointer
	COPY16	YH,YL,r1,r0			;copy page counter to YH:L
	SUBI16	YH,YL,1				;decrement page counter
	COPY16	r1,r0,YH,YL			;copy page counter back to r1:r0
	POPY						;restore Y-pointer
	brne	DFIcommand2			;loop if not zero
	ret							;return to caller
;
.if INTREG	;enabled only if the internal variable are
;************************************************************
; DFAcommand: Read data from random address
;	The lookup table is included for ease of decoding.
;************************************************************
DFAidxtbl:	;index table
	.db		LOW(DFLaddr+0), \
			LOW(DFLaddr+1)

DFAcommand:
	GoSub	Get1stParm			;get the last parameter
	GoSub	CheckUisNiblVal		;only values of [0..3] allowed

	SetZPtr	(2*DFAidxtbl)		;point to index table above
	GoSub	PGMReadByte			;datastack = FLASH @Z
	ret
;
DFAcommand9:
	Goto	Error_0C			;error code "C"
;
.endif
;

;***************************************************************
;SPI_WrtAddr: Writes the 16-bit address held in inbyteh:inbytel
;	to the device.  Calls the local SPI write data routine.
;***************************************************************
SPI_WrtAddr:
	mov		U,inbyteh			;high byte of starting address
	GoSub	SPI_write			;send data in "U"
	mov		U,inbytel			;low byte of starting address
	GoSub	SPI_write			;send data in "U"
	ret
;
