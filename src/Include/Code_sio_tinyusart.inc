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
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
; Support code for SIO routines - Interrupt-driven Soft-UART
;******************************************************************
;calculate timer prescaler and load value
.if (FCLK / BAUDRATE) > 200
 .set	TPSCL			= CK0_8
 .set	ONE_BIT_COUNT	= (FCLK / BAUDRATE / 8)
.else
 .set	TPSCL			= CK0_64
 .set	ONE_BIT_COUNT	= (FCLK / BAUDRATE / 64)
.endif
.equ	ONE_PLUS_BIT_COUNT	= ((((100 * ONE_BIT_COUNT) / 8 ) * 10) / 100)

; For baud >= 38400, set RX to TX delay of 1 TX + 2 RX frames
#if BAUDRATE >= 38400
 .equ	FRAMEDELAY		= (3000000 / BAUDRATE)
#else	; for <= 19200, set RX to TX delay of 1 TX + 1 RX frames
 .equ	FRAMEDELAY		= (2000000 / BAUDRATE)
#endif
;
;************************************************************
; KEYcommand: Receive a byte from the terminal and place it
;	on the stack.
;************************************************************
KEYcommand:
	GoSub	RecvChar			;wait for and fetch a character
	mov		U,inchar			;copy character to U
	cpi		U,CTRL_C			;Is it control-C?
	brne	notermhalt1
	ori		XH,(1<<HALT)		;set flag to halt program
notermhalt1:
	Goto	PushURet			;exit
;
;******************************************************************
; RecvChar:	;	Receive a byte from the terminal.  This is the main
;	receive routine.  This is a blocking routine.  If non-blocking
;	behaviour is required ,check the flag RX_COMPLETE@vuart before
;	calling this function.
;	Return: INCHAR contains the character received
;******************************************************************
RecvChar:
	LOAD	inchar,vuart_state			;get the UART status
	sbrc	inchar,RX_COMPLETE			;skip next if no RX data received
	rjmp	RecvChar1					;RX data received, retrieve it
	sleep								;sleep till IRQ
	nop
	rjmp	RecvChar					;loop till RX data received
RecvChar1:
	andi	inchar,~(1<<RX_COMPLETE)	;clear RX_COMPLETE flag
	STORE	vuart_state,inchar			;save status
	LOAD	inchar,rx_data_complete		;fetch the received byte
	ret									;return to caller
;
;******************************************************************
; EmitChar:	;this is the main transmit routine.
;	Emits a single character but tests for ^S to pause
;	  output.
;	Enter: OUTCHAR contains the character to emit
;******************************************************************
EmitChar:
	rcall	Checkkeybd				;see if control-c or control-s is pressed
	push	outchar					;save outchar
	sbrs	XH,SUSP					;skip next if suspend
	rjmp	EmitItNow				;don't wait for a key
;
waitforkey:
	sleep								;sleep till IRQ
	nop
	LOAD	outchar,vuart_state		;check the UART status
	sbrs	outchar,RX_COMPLETE 	;skip next if byte available
	rjmp	waitforkey				;no characters received, loop
	andi	XH,~(1<<SUSP)			;clear the suspend flag

EmitItNow:
	rcall	VUART_WRITE_DATA		;write the byte in outchar
#if defined(__ATtiny84__) || \
	defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
	ldi		outchar,FRAMEDELAY		;load N * 10uS delay count
EmitItNow1:	;delay for time to allow a ^S or ^C to be received
	GoSub	Delay10uS				;delay 10uS
	dec		outchar					;decrement counter
	brne	EmitItNow1				;loop till zero
#endif
	pop		outchar					;retrieve outchar
	ret								;return to caller
;
;******************************************************************
;Checkkeybd:	See if there is any reason to stop output or
;		program execution.  The interpreter is responsible
;		for handling ^C, while the "EMITCHAR" routine handles ^S.
;	If ^S is found in keyboard buffer, set XH.6 and return.
;	If ^C is found in keyboard buffer, set XH.1 and return.
;******************************************************************
Checkkeybd:
;	push	inbytel
	LOAD	inbytel,vuart_state	;check the UART status
	sbrs	inbytel,RX_COMPLETE ;skip next if byte available
	ret							;return to caller
;
Checkkeybd1:	;look for ^S
	rcall	RecvChar			;fetch the character
	cpi		inchar,CTRL_S		;Is it control-S?
	brne	Checkkeybd2
	ori		XH,(1<<SUSP)		;Set the SUSPEND flag as suspend was requested
	ret							;return to caller
;
Checkkeybd2:	;look for ^C
	cpi		inchar,CTRL_C		;Is it control-C?
	brne	Checkkeybd3
	ori		XH,(1<<HALT)		;set HALT flag to halt program.

Checkkeybd3:
;	pop		inbytel
	ret							;return to caller
;
;******************************************************************
;VUART_WRITE_DATA:  Transmits the character held in outchar.  If
;	transmit buffer is not empty, it blocks and waits until it can
;	transmit new data.
;******************************************************************
VUART_WRITE_DATA:					;send the data in outchar
	LOAD	inbytel,vuart_state		;get the UART status
	sbrs	inbytel,DATA_EMPTY		;skip next if data registers empty
	rjmp	VUART_WRITE_DATA		;loop till empty

;	LOAD	inbyteh,vuart_state
	andi	inbytel,~(1<<DATA_EMPTY);clear DATA_EMPTY flag
	STORE	vuart_state,inbytel		;save status

	STORE	tx_data,outchar			;tx_data = data
	STOREI	tx_bit_count,0,inbytel	;tx_bit_count = 0

	cli								;disable IRQ's
	CLRB	UART_Port,UART_Tx		;clear_tx_pin to make a start bit

	;TX_NEXT_COUNT = COUNTER + ONE_BIT_COUNT;
	LOAD	inbytel,TCNT0			;fetch current TCNT0
	subi	inbytel,-ONE_BIT_COUNT	;add one bit count to TX counter
	STORE	OCR0B,inbytel			;save it in OCR0B

	SETB	TIFR,OCF0B,inbytel		;clear tx_interrupt flag
	SETB	TIMSK,OCIE0B,inbytel	;enable tx_timer interrupt

	sei								;enable IRQ's
	ret								;return to caller
;
;******************************************************************
; InitSIO:	configure RX and TX pins and set to their initial
;	state.  Also enables pin IRQ settings.
; USES: inbytel
;******************************************************************
InitSIO:
	push	inbytel					;save register
#if defined(__ATtiny84__) || defined(__ATtiny85__)
	CLRB	PRR,PRTIM0,inbytel		;enable TIMR0
#endif

#if SIO_EINT	;set the INT0 edge detect for RX on INT0 vector
	STOREI	MCUSR,~((1<<ISC01|1 << ISC01)) | (1<<ISC01),inbytel
	SETB	GIMSK,INT0,inbytel		;enable INT0 interrupt
#else			;for RX on PCINT0 vector
	SETB	PCMSK,UART_RX,inbytel	;enable PINx for PCINT's
	SETB	GIFR,PCIF,inbytel		;clear PCIE interrupt flag
	SETB	GIMSK,PCIE,inbytel		;enable INT0 interrupt
#endif
	CLRB	TIMSK,OCIE0A,inbytel	;disable rx_timer interrupt
	CLRB	UART_DDR,UART_Rx		;rx_pin to input

	CLRB	TIMSK,OCIE0B,inbytel	;disable tx_timer interrupt
	SETB	UART_DDR,UART_Tx		;tx_pin to output
	SETB	UART_Port,UART_Tx		;set tx_pin to high

	STOREI	vuart_state,(1<<DATA_EMPTY),inbytel;set DATA_EMPTY flag

	STOREI	TCCR0A,0,inbytel		;config timer register A
	STOREI	TCCR0B,TPSCL,inbytel	;config timer prescaler
	pop		inbytel					;restore register
	sei								;enable IRQ's
	ret
;
;******************************************************************
; INT0_int or PCINT0_int service routine
;******************************************************************
#if SIO_EINT && ( defined(__ATtiny85__) || \
				  defined(__ATtiny84__) || \
				  defined(__ATtiny84A__) )	;for RX on INT0 vector
   INT0_int:
#else
 #if defined(__ATtiny84__) || defined(__ATtiny84A__)
  .if UART_Port == PORTA
   PCINT0_int:	;for RX on PCINT pin
  .elif UART_Port == PORTB
   PCINT1_int:	;for RX on PCINT pin
  .endif
 #else
   PCINT0_int:	;for RX on PCINT pin
 #endif
	sbic	UART_PORT-2,UART_RX		;skip next if I/O pin low (START bit)
	reti							;not our pin
#endif
	push	temp
	PUSHSTAT						;save status register

;	RX_NEXT_COUNT = COUNTER + ONE_PLUS_BIT_COUNT; // set next timer
	LOAD	temp,TCNT0				;fetch current TCNT0 count
	subi	temp,-ONE_PLUS_BIT_COUNT;add one plua a bit to TCNT
	STORE	OCR0A,temp				;seed the RX Timer

	SETB	TIFR,OCF0A,temp			;clear the rx_interrupt flag
	SETB	TIMSK,OCIE0A,temp		;enable_rx_timer_interrupt

#if SIO_EINT	;for RX on INT0 vector
	CLRB	GIMSK,INT0,temp			;disable external interrupt
#else
	CLRB	GIMSK,PCIE,temp			;disable pin-change interrupts
#endif
	clr		temp					;insure inbytel = 0
	STORE	rx_data,temp			;clear rx_data
	STORE	rx_bit_count,temp		;clear rx_bit_count;

	POPSTAT							;restore status register
	pop		temp
	reti							;return from interrupt
;
;******************************************************************
; TIMER0_COMPA_int: RX_TIMER_INTERRUPT
;	After storing all the bits into rx_data, this routine disables
;	itself and re-enables the external interrupt to listen for
;	a new data frame.
;******************************************************************
TIMER0_COMPA_int:
    push	r18						;save some registers
	LOAD	r18,SREG				;save SREG
	push	r18
    push	r19
    push	inbytel
    push	inbyteh

;	RX_NEXT_COUNT += ONE_BIT_COUNT;
    LOAD	inbytel,OCR0A
    subi	inbytel,-ONE_BIT_COUNT	;add one bit count to RX counter
    STORE	OCR0A,inbytel

;	if (read_rx_pin())
	sbis	UART_PORT-2,UART_Rx		;skip next if I/O pin high
    rjmp	TM0OCA3

;	rx_data |= (1 << rx_bit_count);
    LOAD	r18,rx_data
    LOAD	r19,rx_bit_count
    ldi		inbytel,1
    clr		inbyteh
    rjmp	TM0OCA2

TM0OCA1:
    add		inbytel,inbytel
    adc		inbyteh,inbyteh

TM0OCA2:
    dec		r19
    brpl	TM0OCA1
    or		r18,inbytel
    STORE	rx_data,r18

TM0OCA3:	; increment rx_bit_count
	LOAD	inbytel,rx_bit_count
    inc		inbytel
    STORE	rx_bit_count,inbytel

;	if (rx_bit_count == CHAR_SIZE + STOP_BITS)
    LOAD	inbytel,rx_bit_count
    cpi		inbytel,(CHAR_SIZE + STOP_BITS)
    brne	TM0OCA4

    LOAD	inbytel,rx_data			;fetch received data
    STORE	rx_data_complete,inbytel	;store to buffer

	SETB	vuart_state,RX_COMPLETE,inbytel	;set RX_COMPLETE flag
	CLRB	TIMSK,OCIE0A,inbytel	;disable rx_timer interrupt

#if SIO_EINT	;for RX on INT0 vector
	SETB	GIFR,INT0,inbytel		;clear external_interrupt flag
	SETB	GIMSK,INT0,inbytel		;enable external_interrupt
#else
	SETB	GIFR,PCIF,inbytel		;clear PCIE interrupt flag
	SETB	GIMSK,PCIE,inbytel		;enable INT0 interrupt
#endif

TM0OCA4:
	pop		inbyteh					;restore registers
    pop		inbytel
    pop		r19
    pop		r18						;restore SREG
	STORE	SREG,r18
    pop		r18
    reti
;
;******************************************************************
; TIMER0_COMPB_int: TX_TIMER_INTERRUPT - transmit bits in tx_data
;	in correct intervals.  After transmitting all the bits, sets
;	flags and disable itself.
;******************************************************************
TIMER0_COMPB_int:
	push	inbytel					;save register
	LOAD	inbytel,SREG			;save SREG
	push	inbytel

;	TX_NEXT_COUNT += ONE_BIT_COUNT;
	LOAD	inbytel,OCR0B
	subi	inbytel,-ONE_BIT_COUNT	;add one bit count to TX counter
	STORE	OCR0B,inbytel

;	if (tx_bit_count < CHAR_SIZE)
	LOAD	inbytel,tx_bit_count
	cpi		inbytel,CHAR_SIZE
	brcc	TM0OCB3
;	if (tx_data & 0x01)
	LOAD	inbytel,tx_data
	sbrs	inbytel,0
	rjmp	TM0OCB1
	sbi		UART_Port,UART_Tx		;set tx_pin high
	rjmp	TM0OCB2

TM0OCB1:
	cbi		UART_Port,UART_Tx		;clear tx_pin low

TM0OCB2:	;tx_data >>= 1;
	LOAD	inbytel,tx_data			;fetch current TX data
	lsr		inbytel					;shift one bit position
	STORE	tx_data,inbytel			;save it back
	rjmp	TM0OCB4

TM0OCB3:
	SETB	UART_Port,UART_Tx		;set tx_pin to high

TM0OCB4:
	LOAD	inbytel,tx_bit_count	;increment TX bit counter
	inc		inbytel
	STORE	tx_bit_count,inbytel

;	if (tx_bit_count > CHAR_SIZE + STOP_BITS)
	LOAD	inbytel,tx_bit_count	;test for end of TX data transmission
	cpi		inbytel,(CHAR_SIZE + STOP_BITS + 1)
	brcs	TM0OCB5
	STOREI	vuart_state,(1<<TX_COMPLETE|1<<DATA_EMPTY),inbytel
	CLRB	TIMSK,OCIE0B,inbytel	;disable tx_timer interrupt

TM0OCB5:
    pop		inbytel					;restore SREG
	STORE	SREG,r18
	pop		inbytel					;restore register
    reti
;
