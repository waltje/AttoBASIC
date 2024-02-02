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
#if ( defined(__ATmega88__) || \
	  defined(__ATmega88PA__) )
 .message "SPI enabled."
;*************************************************************
;	SPW: write a value to and read a value from the SPI port
SPWcommand:
	rcall	ChkSPIEn			;is SPI enabled?
	rcall	Get1stParm			;get the last parameter
#if CTRL_SS	;if AttoBASIC is allowed to control the SS pin
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
	nop							;delay
	nop
#endif
	STORE	SPDR,U				;send data
SPWcommand1:
	SKBS	SPSR,SPIF,romreg	;skip next if SPIF set
	rjmp	SPWcommand1			;loop till data shifted out
	LOAD	U,SPDR				;get return byte
	ret
;
;*************************************************************
;	SPR: read a value from the SPI port
SPRcommand:
	rcall	ChkSPIEn			;is SPI enabled?
#if CTRL_SS	;if AttoBASIC is allowed to control the SS pin
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
	nop							;delay
	nop
#endif
	STORE	SPDR,U				;send dummy data
SPRcommand1:
	SKBS	SPSR,SPIF,romreg	;skip next if SPIF set
	rjmp	SPRcommand1			;loop till data shifted out
	LOAD	U,SPDR				;get return byte
	Goto	PushURet			;exit
;
#else
;****************************************************************************/
;  SPWcommand: SPW [x...z] or "SPW "hello" - SPW sends data or string
;		to the SPI bus.
;****************************************************************************/
 SPWcommand:
 	rcall	ChkSPIEn			;is SPI enabled?

SPWcommand1:
	PUSHY						;save Y
	SUBI16	YH,YL,1				;decrement Y pointer to next character
	ld		U,Y					;get the character
	cpi		U,'"'				;is it the end quote mark?
	brne	SPWcommand2			;no, process as numeric parms

;	GoSub	advanclbpointer		;advance past character
SPWcommand1a:	;stuff the string into the data stack
	ld		U,-Y				;fetch character, decrement Y
	cpi		U,'"'				;is it the end quote mark?
	breq	SPWcommand1b		;yes, exit loop
	GoSub	pushU				;save the character on the stack
	rjmp	SPWcommand1a		;loop for all the string data

SPWcommand1b:
	pop		U					;remove stored Y from stack
	pop		U
	GoSub	advanclbpointer		;advance past character
	mov		cmdparms,DStkPtr	;adjust cmdparms
	rjmp	SPWcommand2a		;now process as numeric parms

SPWcommand2:
	POPY						;restore Y
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	brne	SPWcommand2a		;parameters on dstack, process
	Goto	Error_1D			;error code $1D, "Invalid number of arguments"

SPWcommand2a:
	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	PUSHZ						;save Z Ptr
	clr		r1					;clear a register
	SetZPtr	(DATASTK+DATASTKSZ)	;point Z to TOS
	sub		DStkPtr,cmdparms	;adjust DStkPtr
	SUB16	ZH,ZL,r1,DStkPtr	;adjust Z to point to the data

.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT				;disable USB ints
.else		;USART interrupts do too!
	CLRB	UCSRB,RXCIE,temp		;disable USART RX complete IRQ
.endif

#if CTRL_SS	;if AttoBASIC is allowed to control the SS pin
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
	nop
	nop
#endif

SPWcommand2b:	;set up bit counter and fetch a data byte
	ld		U,-Z				;fetch the value from the dstack
	STORE	SPDR,U				;send data
SPWcommand2c:
	SKBS	SPSR,SPIF,temp		;skip next if SPIF set
	rjmp	SPWcommand2c		;loop till SPIF set
	LOAD	U,SPDR				;get return byte
	dec		cmdparms			;decrement byte counter
	brne	SPWcommand2b		;loop till zero

SPWcommand3:
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif

	POPZ						;restore Z Ptr
	POPSTAT						;restore status register
	ret							;return to caller
;
;****************************************************************************/
;  SPRcommand: Receives a a series of data bytes from an SPI device and
;	places it into the DATA statement's buffer for subsequent reading by the
;	READ command.
;	[Note issue this command after an "SPM" command has been issued.]
;****************************************************************************/
SPRcommand:
	rcall	ChkSPIEn			;is SPI enabled?
.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT			;disable USB ints
.else		;USART interrupts do too!
	CLRB	UCSRB,RXCIE,temp	;disable USART RX complete IRQ
.endif

	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	PUSHZ						;save Z
	clr		r1					;clear a register
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	brne	SPRcommand1			;parms detected
	clr		U					;clear U
	rjmp	SPRcommand1a		;bypass parm test

SPRcommand1:	;fetch multiple bytes and stuff in DATA buffer
	ldi		temp,DATASTKSZ+1	;set max data buffer size
	GoSub	popU				;fetch # of bytes from stack
	cp		U,temp				;is the value provided larger than buffer?
	brmi	SPRcommand1a		;no, continue
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif
	Goto	Error_16			;error code $16, "Specified value exceeded range"

SPRcommand1a:
	mov		H,U					;copy payload size into H
	LOAD	r0,SPDR				;clear data from buffer

#if CTRL_SS	;if AttoBASIC is allowed to control the SS pin
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
	nop
	nop
#endif

	tst		U					;any command line parameters?
	brne	SPRcommand2			;yes, read data into buffer

	STORE	SPDR,r1				;send dummy data
SPRcommand1b:
	SKBS	SPSR,SPIF,temp		;skip next if SPIF set
	rjmp	SPRcommand1b		;loop till SPIF set
	LOAD	U,SPDR				;get return byte
	GoSub	pushU				;save on dstack
	rjmp	SPRcommand3			;clean up and exit

SPRcommand2:
	SetZPtr	(READSTOR+READSTKSZ);Z = top of DATA buffer
	STORE	DATAptr,r1			;clear DATA read pointer
	STORE	DATAptr+1,U			;save payload size as # of DATA elements

SPRcommand2a:	;start of data xfer loop
	STORE	SPDR,r1				;send dummy data
SPRcommand2b:
	SKBS	SPSR,SPIF,temp		;skip next if SPIF set
	rjmp	SPRcommand2b		;loop till SPIF set
	LOAD	U,SPDR				;get return byte
	st		-Z,U				;store in DATA space
	dec		H					;decrement byte counter
	brne	SPRcommand2a		;loop till zero

SPRcommand3:	;clean up
	POPZ						;restore Z

.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.endif

	POPSTAT						;restore STATUS reg
	ret							;return to caller
;
#endif
;
;*************************************************************
;	SPM: set the SPI mode [0..2].
SPMcommand:
.ifdef PRR
	CLRB	PRR,PRSPI,temp		;enable SPI in PRR
.endif
.ifdef PRR0
	CLRB	PRR0,PRSPI,temp		;enable SPI in PRR0
.endif
	ldi		U,2					;default to Mode 2
	Gosub	pushU				;save it on the stack
	rcall	Get1stParm			;get the last parameter
;	ldi		temp,$03			;only values of [0..3] allowed
;	cp		temp,U
;	brmi	SPMcommand9			;branch if allowable exceeded
	GoSub	CheckUisNiblVal		;only values of [0..3] allowed
	lsl		U					;shift value 2 bits left
	lsl		U

	rcall	SPIinit				;init the SPI
	LOAD	temp,SPCR			;get SPCR
	andi	temp,0xF3			;zero the CPOL and CPHA bits
	or		temp,U				;OR new value
	STORE	SPCR,temp			;save SPCR
	ret
;
;*************************************************************
;	SPO: set the data order (SPO 0/1) 0 = MSB, 1 = LSB.
SPOcommand:
	rcall	Get1stParm			;get the last parameter
;	ldi		temp,$1				;only values of [0..1] allowed
;	cp		temp,U
;	brmi	SPOcommand9			;branch if allowable exceeded
	GoSub	CheckUisBitVal		;only values of [0..1] allowed

	SETB	SPCR,DORD,temp		;set LSB mode
	sbrc	U,0					;skip next if bit 0 set
	rjmp	SPOcommand1
	CLRB	SPCR,DORD,temp		;set LSB mode
SPOcommand1:
	ret

;SPOcommand9:
;	Goto	Error_0C			;error code "C"
;
;*************************************************************
;	SPC: set the SPI clock rate.
SPCKcommand:
	rcall	Get1stParm		;get the last parameter
;	ldi		temp,$7				;only values of [0..7] allowed
;	cp		temp,U
;	brmi	SPCKcommand9		;branch if allowable exceeded
	GoSub	CheckUisByteVal		;only values of [0..7] allowed

	mov		r0,U				;copy of U in r0
	andi	U,0x03				;strip off bit 2 (SPI2X)
	LOAD	temp,SPCR			;get SPCR
	andi	temp,0xF8			;clear clock bits
	or		temp,U				;OR new clock bits
	STORE	SPCR,temp			;save SPCR

	SETB	SPSR,SPI2X,temp		;set 2X clock mode
	sbrc	r0,2				;skip next if bit 2 clear
	rjmp	SPCKcommand1
	CLRB	SPSR,SPI2X,temp		;set 1X clock mode
SPCKcommand1:
	ret
;
;*************************************************************
;	SPS: set SPI_SS pin to level to U where U = [0..1].  Defaults
;		 to '1' when Gosubed without a value.
SPScommand:
	rcall	ChkSPIEn			;is SPI enabled?
	ldi		U,1					;Put default level on stack
	GoSub	pushU				;save on the dstack
	rcall	Get1stParm			;get the last parameter
	SETB	SPI_PORT,SPI_SS		;drop SS line low
	sbrc	U,0					;skip next if bit 0 set
	rjmp	SPScommand1
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
SPScommand1:
	ret
;

;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;
.if (SPI || DFR || NRF)	;primitives for DATA file, SPI or nRF support
;***********************************************************************
; ChkSPIEn: checks to see if the SPI is enabled.  Error if not
;***********************************************************************
ChkSPIEn:
	SKBC	SPCR,SPE,temp		;skip next if SPI enabled
	ret							;return to caller
	STOREI	ERRCode,$1A			;error code "0x1A"
	Goto	Error
;
;***********************************************************************
; SPIinit: SPI port initialization routine.  Defaults to SPI Master in
;	mode 0 with fck/16.
; Destroys: temp
;***********************************************************************
SPIinit:
.ifdef PRR
	CLRB	PRR,PRSPI,temp		;enable SPI in PRR
.endif
.ifdef PRR0
	CLRB	PRR0,PRSPI,temp		;enable SPI in PRR0
.endif
	;Set MOSI, SCK and SS output, all others input
	LOAD	temp,SPI_PORT-1
	ori		temp,(	(1 << SPI_MOSI) | \
					(1 << SPI_SCK) | \
					(1 << SPI_SS) )
	STORE	SPI_PORT-1,temp

	;make SS output high
	SETB	SPI_PORT,SPI_SS		;insure SS is high

	;Enable SPI, Master, set clock rate fck/16, mode 0
	ldi		temp,(	(1 << SPE) | \
					(1 << MSTR) | \
					(0 << CPOL) | \
					(0 << CPHA) | \
					(1 << SPR0) )
	STORE	SPCR,temp
	CLRB	SPSR,SPI2X,temp			;insure 1X clock

	ret
;
;***************************************************************
; SPI_write: Local SPI write data routine.  Writes value in "U"
;	to SPDR, waits for xmit to finish before returning.
; Destroys: temp
;***************************************************************
SPI_write:
#if CTRL_SS	;if AttoBASIC is allowed to control the SS pin
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
	nop							;delay
	nop
#endif
	STORE	SPDR,U				;send data
SPI_write1:
	SKBS	SPSR,SPIF,temp		;skip next if SPIF set
	rjmp	SPI_write1			;loop till data shifted out
	LOAD	temp,SPDR			;read SPDR to clear SPIF flag
	ret							;return to caller
;
;*************************************************************
; SPI_read: read a value from the SPI port into "U"
; Destroys: temp
;*************************************************************
SPI_read:
#if CTRL_SS	;if AttoBASIC is allowed to control the SS pin
	CLRB	SPI_PORT,SPI_SS		;drop SS line low
	nop							;delay
	nop
#endif
	STORE	SPDR,U				;send dummy data
SPI_read1:
	SKBS	SPSR,SPIF,temp		;skip next if SPIF set
	rjmp	SPI_read1			;loop till data shifted out
	LOAD	U,SPDR				;get return byte
	ret							;return to caller
;
.endif
