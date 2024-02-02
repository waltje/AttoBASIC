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
; Support code for SIO routines - USI
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
; RecvChar:	;this is the main receive routine.
;	Receive a byte from the terminal.         
;	Return: INCHAR contains the character received
;************************************************************
RecvChar:
	SETB	MCUCR,SE,temp		;insure SE bit is set

RecvChar1:
	sei							;insure global interrupts allowed
	sleep						;go to to sleep
	nop

	push	r18					;save register
	GoSub	USI_UART_Available	;check for character
	pop		r18					;restore register

	tst		inbytel				;any characters?
	breq	RecvChar1			;nope, loop

	PUSHZ						;save register
	GoSub	USI_UART_GetChar	;fetch the character
	POPZ						;restore register

	mov		inchar,inbytel		;move to inchar
	ret							;return to caller
;
;************************************************************
; EmitChar:	;this is the main transmit routine.  
;	Emits a single character but tests for ^S to pause 
;	  output.
;	Enter: OUTCHAR contains the character to emit
;************************************************************
EmitChar:
	GoSub	Checkkeybd			;see if control-c or control-s is pressed
	sbrs	XH,SUSP				;if XH:SUSP (suspend) is clear, don't wait for key
	Goto	waitdone

waitforkey:	
	push	r18					;save register
	GoSub	USI_UART_Available	;data in buffer?
	pop		r18					;restore register
	tst		inbytel				;test for zero characters received
	breq	waitforkey			;no characters received, loop
	andi	XH,~(1<<SUSP)		;clear the suspend flag

waitdone:
	mov		inbytel,outchar		;copy character to USB register
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	PUSHZ	
	GoSub	USI_UART_PutChar	;send it
	POPZ
	pop	r1						;save r1
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
	push	r18
	GoSub	USI_UART_Available	;data in buffer?
	pop		r18					;restore register
	tst		inbytel				;test for zero characters received
	brne	Checkkeybd1			;characters received, continue
	ret							;return to caller
Checkkeybd1:
	PUSHZ						;save registers
	GoSub	USI_UART_GetChar	;fetch the character
	mov		temp,inbytel		;move received char to temp
	POPZ						;restore registers
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
; InitSIO:	;initialize the USI as a UART
;************************************************************
InitSIO:
	Gosub	USI_UART_Close		;shutdown USI UART and flush the buffers
	GoSub	Delay100mS			;delay 100 mS
	GoSub	USI_UART_Init		;initialise USI UART
;	Gosub	USI_UART_Initialise_Transmitter	;initialize the transmitter
;	Gosub	USI_UART_Initialise_Receiver	;initialize the receiver

	STOREI	UART_status,0,temp	;zero the UART_status flags
	ret							;return to caller
;
