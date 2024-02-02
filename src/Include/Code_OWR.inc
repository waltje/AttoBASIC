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
.message "OWR enabled."
;***********************************************************
; OWIcommand:	One-Wire initialize and detect presence.
;	Returns with "1" in U if device presence detected
;***********************************************************
OWIcommand:
.if USBSUPP		;USB interrupts will screw us up!
	GoSub	D_USBINT			;disable USB ints
.elif USISUPP	;USI interrupts will screw us up!
	GoSub	D_USIINT			;disable USI ints
.elif ! __ATtiny85__	;USART interrupts do too!
	CLRB	UCSRB,RXCIE,temp	;disable USART RX complete IRQ
.endif
	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	SETB	OWRPORT,OWRPIN		;set pin high
	SETB	OWRPORT-1,OWRPIN	;set pin to output
	GoSub	Delay1mS			;delay 1mS for settling time
	CLRB	OWRPORT,OWRPIN		;set pin low

	ldi		temp,50				;50x loop for 500uS
OWIcommand1:
	GoSub	Delay10uS			;delay 10uS
	dec		temp				;decement counter
	brne	OWIcommand1			;loop till zero

	;change to pin to input and check for presence
	SETB	OWRPORT,OWRPIN		;set pin PUP
	CLRB	OWRPORT-1,OWRPIN	;set pin to input

	ldi		temp,6				;6x loop for 60uS

OWIcommand2:
	GoSub	Delay10uS			;delay 10uS
	dec		temp				;decement counter
	brne	OWIcommand2			;loop till zero

	ldi		U,1					;preset U to "1"
	LOAD	temp,OWRPORT-2		;fetch pin level
	sbrc	temp,OWRPIN			;skip next if device present
	clr		U					;set U to "0"
	GoSub	pushU				;save on dstack

	;need to allow 500uS recovery time before next bus transaction
	ldi		temp,50				;50x loop for 500uS
OWIcommand3:
	GoSub	Delay10uS			;delay 10uS
	dec		temp				;decement counter
	brne	OWIcommand3			;loop till zero

OWIcommand9:
	POPSTAT						;restore status register
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.elif USISUPP	;USI interrupts will screw us up!
	GoSub	E_USIINT		;re-enable USI ints
.endif
	GoSub	interpretlinev		;execute any further commands
	ret
;
;***********************************************************
; OWWcommand:	One-Wire Write data
;***********************************************************
OWWcommand:
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	brne	OWWcommand1			;parameters on dstack, process

	Goto	Error_1D			;not enough arguments

OWWcommand1:	;set up pointer to data on dstack
.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT			;disable USB ints
.elif USISUPP	;USI interrupts will screw us up!
	GoSub	D_USIINT		;disable USB ints
.elif ! __ATtiny85__	;USART interrupts do too!
	CLRB	UCSRB,RXCIE,temp	;disable USART RX complete IRQ
.endif
	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	PUSHZ						;save Z Ptr
	clr		r1					;clear a register
	SetZPtr	(DATASTK+DATASTKSZ)	;point Z to TOS
	sub		DStkPtr,cmdparms	;adjust DStkPtr

OWWcommand1a:	;set up bit counter and fetch a data byte
	ldi		U,8					;8 bits per byte
	mov		bytecntr,U			;move to bit counter
	ld		U,-Z				;fetch the value from the dstack

OWWcommand1b:	;process each bit value LSB -> MSB
	ror		U					;rotate bit into C flag
	brcc	OWWcommand1c		;carry clear, write a "0"
	rcall	OWRwr1ts			;write a "1"
	rjmp	OWWcommand1d		;jump

OWWcommand1c:
	rcall	OWRwr0ts			;write a "0"

OWWcommand1d:
	GoSub	Delay5uS			;5uS bit-to-bit recovery delay
	dec		bytecntr			;decrement bit counter
	brne	OWWcommand1b		;loop till zero

	GoSub	Delay15uS			;15uS byte-to-byte recovery delay
	dec		cmdparms			;decrement byte counter
	brne	OWWcommand1a		;loop till zero

	POPZ						;restore Z
	POPSTAT						;restore status register
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.elif USISUPP	;USI interrupts will screw us up!
	GoSub	E_USIINT		;re-enable USI ints
.endif
	ret
;
;***********************************************************
; OWRcommand:	One-Wire Read data
;	Returns with 1 byte on dstack
;***********************************************************
OWRcommand:
.if USBSUPP	;USB interrupts will screw us up!
	GoSub	D_USBINT			;disable USB ints
.elif USISUPP	;USI interrupts will screw us up!
	GoSub	D_USIINT		;disable USI ints
.endif
	PUSHSTAT					;save status register
	cli							;disable global IRQ's
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	brne	OWRcommand1			;parameter on dstack, stuff buffer
	rcall	OWRrdbyte			;read a byte
	GoSub	pushU				;save on dstack
	rjmp	OWRcommand2			;exit
;
OWRcommand1:
	PUSHY						;save Y
	clr		r1					;clear a register
	GoSub	popU				;fetch # of bytes from stack
	mov		H,U

	SetYPtr	(READSTOR+READSTKSZ);Y = top of DATA buffer
	STORE	DATAptr,r1			;clear DATA read pointer
	STORE	DATAptr+1,U			;save payload size as # of DATA elements
;
OWRcommand1a:
	rcall	OWRrdbyte			;read a byte
	st		-Y,U				;store in DATA space
	GoSub	Delay15uS			;delay 15uS for recovery (tREC)
	dec		H					;decrement byte counter
	brne	OWRcommand1a		;loop till zero
	POPY						;restore Y

OWRcommand2:
	POPSTAT						;restore status register
.if USBSUPP	;re-enable USB interrupts, USART are done in main loop
	GoSub	E_USBINT			;re-enable USB ints
.elif USISUPP	;USI interrupts will screw us up!
	GoSub	E_USIINT		;re-enable USI ints
.endif
	ret							;return to caller
;
;***********************************************************
; OWRrdbyte:	One-Wire read a byte from device.
;	Returns with byte value in U
;	Destroys temp, bytecntr
;***********************************************************
OWRrdbyte:
	ldi		temp,8				;bit counter
	mov		bytecntr,temp		;move to bytecntr

OWRrdbyte1:
	rcall	OWRrdts				;fetch bit value
	clc							;clear carry flag
	sbrc	temp,bit0			;skip next if bit0 = 0
	sec							;set carry flag
	ror		U					;shift bit into U LSB -> MSB
	GoSub	Delay5uS			;delay 5uS for recovery (tREC)
	dec		bytecntr			;decrement counter
	brne	OWRrdbyte1			;loop till zero
	ret
;
;***********************************************************
; OWRrdts:	One-Wire read time slot.  75uS time slot.
;	Returns with bit value in temp
;***********************************************************
OWRrdts:
	CLRB	OWRPORT,OWRPIN		;set pin low
	SETB	OWRPORT-1,OWRPIN	;set pin to output

	GoSub	Delay5uS			;delay 5uS (tRL)

	SETB	OWRPORT,OWRPIN		;set pin PUP
	CLRB	OWRPORT-1,OWRPIN	;set pin to input

	GoSub	Delay5uS			;delay 5uS (tRL)

	ser		temp				;preset temp to 0xFF
	LOAD	inchar,OWRPORT-2	;fetch pin level
	sbrs	inchar,OWRPIN		;skip next if bus high
	clr		temp				;clear temp
	push	temp				;save temp

	;wait out the time slot
	ldi		temp,10				;10x loop for ~60uS
OWRrdts1:
	GoSub	Delay5uS			;delay 5uS (tRL)
	dec		temp				;decement counter
	brne	OWRrdts1			;loop till zero
	pop		temp				;restore temp
	ret
;
;***********************************************************
; OWRwr1ts:	One-Wire write 1 time slot.  70uS time slot w/
;	port pin low for 10uS.
;***********************************************************
OWRwr1ts:
	CLRB	OWRPORT,OWRPIN		;set pin low
	SETB	OWRPORT-1,OWRPIN	;set pin to output

	GoSub	Delay10uS			;delay 10uS (tW1L)

	SETB	OWRPORT,OWRPIN		;set pin PUP
	CLRB	OWRPORT-1,OWRPIN	;set pin to input

	;wait out the time slot
	ldi		temp,6				;6x loop for ~60uS
OWRwr1ts1:
	GoSub	Delay10uS			;delay 10uS
	dec		temp				;decement counter
	brne	OWRwr1ts1			;loop till zero

	ret
;
;***********************************************************
; OWRwr0ts:	One-Wire write 0 time slot.  70uS time slot.
;***********************************************************
OWRwr0ts:
	SETB	OWRPORT-1,OWRPIN	;set pin to output
	CLRB	OWRPORT,OWRPIN		;set pin low

	ldi		temp,7				;7x loop for 70uS
OWRwr0ts1:
	GoSub	Delay10uS			;delay 10uS
	dec		temp				;decement counter
	brne	OWRwr0ts1			;loop till zero

	SETB	OWRPORT,OWRPIN		;set pin low
	CLRB	OWRPORT-1,OWRPIN	;set pin to input
	GoSub	Delay5uS			;5uS bit-to-bit recovery time (tREC)
	ret
;
;****************************************************************************
; DELAY15US: Delays 15uS.
;****************************************************************************
Delay15uS:
	GoSub	Delay5uS			;15 uS delay
	GoSub	Delay5uS
	GoSub	Delay5uS
	ret
;
