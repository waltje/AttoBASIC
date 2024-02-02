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
; Support code for SIO routines - Hardware USART
;************************************************************
; KEYcommand: Receive a byte from the terminal and place it
;	on the stack.
;************************************************************
KEYcommand:		;Receive a byte from the terminal
	sleep						;go to to sleep
	nop
	SKBS	UCSRA,RXC,temp		;Wait for byte to be received
	rjmp 	keycommand
	LOAD	U,UDR	;Read byte
	cpi		U,CTRL_C			;Is it control-C?
	brne	notermhalt1
	ori		XH,(1<<HALT)		;set flag to halt program
notermhalt1:
	Goto	PushURet			;exit
;
;************************************************************
; RecvChar:	;this is the main receive routine.
;	Receive a byte from the terminal.
;	Return: INCHAR contains the character received
;************************************************************
RecvChar:
#if ( defined(__ATmega16__) || \
	  defined(__ATmega16L__) || \
	  defined(__ATmega32__) || \
	  defined(__ATmega32A__) )
	SETB	MCUCR,SE,temp		;insure SE bit is set
#else
	SETB	SMCR,SE,temp		;insure SE bit is set
#endif

RecvChar1:
	sei							;insure global interrupts allowed
	sleep						;go to to sleep
	nop
;
	SKBS	UCSRA,RXC,temp		;skip next if RX ready with character
	rjmp	recvchar1			;not RX, loop
	LOAD	inchar,UDR			;Read byte
	ret
;
;************************************************************
; EmitChar:	;this is the main transmit routine.
;	Emits a single character but tests for ^S to pause
;	  output.
;	Enter: OUTCHAR contains the character to emit
;************************************************************
EmitChar:
	SKBS	UCSRA,UDRE,romreg	;skip next if Data Register Empty
	Goto	emitchar			;loop till Data Register Empty
	GoSub	Checkkeybd			;see if control-c or control-s is pressed

	sbrs	XH,SUSP				;if XH:SUSP (suspend) is clear, don't wait for key
	Goto	waitdone

waitforkey:
	SKBS	UCSRA,RXC,romreg	;skip next if RX char waiting
	rjmp 	waitforkey			;wait for a key before continuing
	andi	XH,~(1<<SUSP)		;clear the suspend flag

waitdone:
	STORE	UDR,outchar			;send the byte
	ret							;return to caller
;
;********************************************************************
;Checkkeybd:	See if there is any reason to stop output or
;		program execution.  The interpreter is responsible
;		for handling ^C, while the "EMITCHAR" routine handles ^S.
;	If ^S is found in keyboard buffer, set XH.6 and return.
;	If ^C is found in keyboard buffer, set XH.1 and return.
;********************************************************************
Checkkeybd:
;	andi	XH,~((1<<HALT)|(1<<SUSP));insure flags cleared
	SKBS	UCSRA,RXC,romreg	;if byte received, from terminal, get it
	Goto	nokeydown
	LOAD	temp,UDR			;Read byte
	cpi		temp,CTRL_S			;Is it control-S?
	brne	notermstop
	ori		XH,(1<<SUSP)		;Set the SUSPEND flag as suspend was requested
	ret							;return to caller

notermstop:
	cpi		temp,CTRL_C			;Is it control-C?
	brne	notermhalt
	ori		XH,(1<<HALT)		;set HALT flag to halt program.

notermhalt:
nokeydown:
	ret							;return to caller
;
;************************************************************
; InitSIO:	;init hardware USART
;************************************************************
InitSIO:
;enable TIMx in PRRx
#if ( !defined(__ATmega16__) && \
	  !defined(__ATmega16L__) && \
	  !defined(__ATmega32__) && \
	  !defined(__ATmega32A__) )
	CLRB	UART_PRR,UART_PRUART,temp	;enable UART in PRR
#endif
	ldi     temp,high(((f_clk/16)/baudrate)-1)
	STORE	UBRRH,temp     	;Load UART baud rate
	ldi     temp,low(((f_clk/16)/baudrate)-1)
	STORE	UBRRL,temp     	;Load UART baud rate

; USART0: RX Interupts, enable TX and RX
	ldi		temp,(1<<RXCIE|0<<TXCIE|0<<UDRIE| \
					1<<RXEN|1<<TXEN|0<<UCSZ2)
	STORE	UCSRB,temp

	ret							;return to caller
;

