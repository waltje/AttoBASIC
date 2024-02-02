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
;************************************************************
; Support code for SIO routines - USB
;************************************************************
; KEYcommand: Receive a byte from the terminal and place it
;	on the stack.
;************************************************************
keycommand:		;Receive a byte from the terminal
	sleep						;go to to sleep
	nop
	LOAD	inbytel,usb_configuration;get status of USB
	tst		inbytel				;is it 0?
	breq	keycommand1			;USB is not config'd so inbytel=0
	;TX a character via USB
	push	r18					;save registers clobbered by USB
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	call	usb_serial_available	;any characters available?
	pop		r1					;restore r1
	pop		r18
	tst		inbytel				;test for zero characters received
	breq	keycommand			;zero characters received, loop

	push	r18					;save register clobbered by USB
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	call	usb_serial_getchar	;get the character
	pop		r1					;restore r1
	pop		r18					;restore register
keycommand1:
	mov		U,inbytel			;mov byte to U register
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
	SETB	SMCR,SE,temp		;insure SE bit is set

RecvChar1:
	sei							;insure global interrupts allowed
	sleep						;go to to sleep
	nop
;	Check to see if USB interface is configured
	LOAD	inbytel,usb_configuration;get status of USB
	tst		inbytel				;is it 0?
	breq	recvchar			;USB not config'd, loop

;	Receive the character via USB
	push	r18					;save registers clobbered by USB
	push	r19
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	call	usb_serial_getchar	;get the character
	pop		r1					;restore r1
	pop		r19					;restore registers
	pop		r18
	tst		inbyteh				;zero is good character
	brne	recvchar1			;not RX, loop
	mov		inchar,inbytel		;mov byte to inchar register
	ret
;
;************************************************************
; EmitChar:	;this is the main transmit routine.
;	Emits a single character but tests for ^S to pause
;	  output.
;	Enter: OUTCHAR contains the character to emit
;************************************************************
EmitChar:
	push	r1					;save registers
	clr		r1					;insure r1 = 0
	push	r25					;save r25
	push	r18
	LOAD	r18,usb_configuration;get status of USB
	tst		r18					;is it 0?
	brne	EmitChar1			;if not, USB is config'd
	pop		r18					;restore registers
	pop		r25					;restore r25
	pop		r1
	ret
;
EmitChar1:	;TX a character via USB
	GoSub	Checkkeybd 			;see if control-c or control-s is pressed
	sbrs	XH,SUSP				;if XH:SUSP (suspend) is clear, don't wait for key
	Goto	waitdone

waitforkey:
	call	usb_serial_available	;check for character(s) available
	tst		inbytel				;test for zero characters received
	breq	waitforkey			;zero characters received, loop
	andi	XH,~(1<<SUSP)		;clear the suspend flag
waitdone:
	push	r19					;USB clobbers r19
	push	r20					;USB clobbers r20
waitdone1:
	mov		inbytel,outchar		;copy character to USB register
	call	usb_serial_putchar	;put char in USB buffer
;	call	usb_serial_putchar_nowait;put char in USB buffer
	tst		inbytel				;success = 0
	brne	waitdone1			;loop till character is buffered
	pop		r20					;restore r20
	pop		r19					;restore r19
	pop		r18					;restore r18
	pop		r25					;restore r25
	pop		r1					;restore r1
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

	LOAD	inbytel,usb_configuration;get status of USB
	tst		inbytel				;is it 0?
	breq	nokeydown			;USB is not config'd so inbytel=0
	;RX a character via USB
	push	r18					;save registers clobbered by USB
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	call	usb_serial_available;any characters available?
	pop		r1					;restore r1
	pop		r18
	tst		inbytel				;test for zero characters received
	brne	Checkkeybd1			;characters received, continue
	ret							;return to caller
Checkkeybd1:
	push	r18					;save register clobbered by USB
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	call	usb_serial_getchar	;get the character
	pop		r1					;restore r1
	pop		r18					;restore register
	mov		temp,inbytel		;mov byte to temp register
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
; InitSIO:
;************************************************************
InitSIO:
;	GoSub	usb_configured		;wait till configed
;;	LOAD	inbytel,usb_configuration;wait till configed
;	tst		inbytel				;test inbytel for zero
;	breq	Init1				;branch if not configured yet
;	clr		r1					;insure r1 = 0

	GoSub	usb_shutdown		;insure USB interface has been terminated
								; if we were restarted by the bootloader,
								; we may not have had a proper USB disconnect
  #if !TEST	;bypass delay in TEST mode
	GoSub	Delay100mS			;delay 100 mS
	GoSub	Delay100mS			;delay 100 mS
  #endif

InitSIOa:	;init USB if enabled
	GoSub	usb_init			;init USB interface
								;sei set by usb_serial_init routine
	GoSub	Delay1S				;delay 1 Second

;=====================================================================
;#if defined(__ATmega32U4__) || defined(__AT90USB1286__) ; not needed
 .if CFG 	;only if configuration register routines are enabled
	; test for self-start and connection timeout limit options set
	;  inchar register still holds the config byte value
	ldi		temp,USBConTmr		;load counter with normal USB connection timeout
	;preload watchdog timer for 8 seconds IRQ
	ldi		temp2,((1<<WDIE)|WDT_8S|(0<<WDE))

	SKBS	GPIOR2,SSTEn,r0		;skip next if Self-Start is desired
;	sbrs	inchar,CFG_SSTRT	;skip next if self-start enabled
	rjmp	InitSIOa1			;no self-start, process normal USB timeout
	ldi		temp,1				;load counter to short USB connection timeout
	;preload watchdog timer for 4 seconds IRQ
	ldi		temp2,((1<<WDIE)|WDT_4S|(0<<WDE))

InitSIOa1:
	mov		bytecntr,temp

InitSIOb:	;init USB if enabled
	CLRB	GPIOR0,WDTIRQ		;insure WDTIRQ flag is clear
	;enable watchdog timer for 8 seconds IRQ
	wdr							;reset watchdog
	STOREI	WDTCSR,(1<<WDCE|1<<WDE)
	STORE	WDTCSR,temp2
;=====================================================================
 .else	; config register routines not enabled
	ldi		temp,USBConTmr		;load normal USB connection timeout counter
	mov		bytecntr,temp

InitSIOb:	;init USB if enabled
	CLRB	GPIOR0,WDTIRQ		;insure WDTIRQ flag is clear
	;enable watchdog timer for 8 seconds IRQ
	wdr							;reset watchdog
	STOREI	WDTCSR,(1<<WDCE|1<<WDE)
	STOREI	WDTCSR,((1<<WDIE)|WDT_8S|(0<<WDE))
 .endif
;#endif
;=====================================================================

InitSIOc:
	sleep						;sleep as USB IRQ's will wake
	nop
	SKBC	GPIOR0,WDTIRQ		;skip next if WDTIRQ flag clear
	rjmp	InitSIOe			;WDT tripped, no connection
	LOAD	inbytel,usb_configuration;load config status
	tst		inbytel				;if zero, we have a connection
	breq	InitSIOc			;loop till not zero
	;check if the user's terminal has connected and set DTR
	SKBS	cdc_line_rtsdtr,USB_SERIAL_RTS,temp;skip next if USB_SERIAL_RTS bit is set
	rjmp	InitSIOc			;loop till DTR set

	GoSub	usb_serial_flush_input 	;flush any junk input
	GoSub	usb_serial_flush_output ;flush output buffer
	rjmp	InitSIOf			;continue

InitSIOe:	;we arrived here because of a good connection or a WDT timeout
	dec		bytecntr			;decrement timeout counter
	brne	InitSIOb			;loop till 0
	GoSub	WDDisable			;disable watchdog timer
	SKBC	GPIOR0,WDTIRQ		;skip next if WDTIRQ flag clear
	GoSub	usb_shutdown		;give up but insure USB interface has
								; been terminated
	SKBS	GPIOR2,SSTEn,temp	;skip next if Self-Start is desired
	rjmp	InitSIOa			;no self-start so try USB forever

InitSIOf:	;we either got a USB connection or didn't, clear the WDT flag anyhow
	wdr							;reset watch-dog
	GoSub	WDDisable			;disable watchdog timer
	CLRB	GPIOR0,WDTIRQ		;insure WDTIRQ flag is clear
	ret							;return to caller
;
