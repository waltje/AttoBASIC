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
.message "LPMD enabled."
;***********************************************
;SLP command - Enter Sleep Mode
;*****************************************************
SLPcommand:
	clr		U					;default to "0"
	Gosub	pushU				;save it on the stack
	rcall	Get1stParm			;get the last parameter
	tst		U					;test for zero
	breq	SLPcommand2			;branch if zero (hardware IRQ only)

	;user has passed a number for use with the watchdog timeout
	cpi		U,WDMAX+1			;greater than max prescaler +1?
	brcs	SLPcommand1			;branch if allowable
	Goto	Error_16			;error code "16"

SLPcommand1:
	wdr							;reset watchdog
	CLRB	MCUSR,WDRF,temp		;clear WDRF flag
	mov		temp,U				;copy of U in temp register
	andi	U,0x07				;mask bits 2:0

	lsl		temp				;shift bit 3 left by 2 bits
	lsl		temp
	andi	temp,(1<<WDP3)		;strip all other bits
	ori		temp,(1<<WDIE)		;set "interupt mode"
	or		U,temp				;timout now in U

	;set the WD to "interrupt mode" (0<<WDE | 1<<WDIE)
	cli							;disable interrupts to set WDT
	STOREI	WDTCSR,(1<<WDCE|1<<WDE),temp
	STORE	WDTCSR,U			;set Watchdog timer to timeout

	LOAD	temp,GPIOR0			;get IRQ flags
	andi	temp,~( 1<<RTCIRQ | \
					1<<DDSIRQ | \
					1<<WDTIRQ | \
					1<<USBIRQ | \
					1<<ACOIRQ )	;clear and
	STORE	GPIOR0,temp			;store them

	rjmp	SLPcommand3			;jump and execute sleep
;
SLPcommand2:
.if ADConv || AComp	;only if ADC or AC routines enabled.
	GoSub	ADCen				;enable ADC hardware for register change
	CLRB	ADCSRB,ACME,T		;insure ACME bit cleared to start
	SKBC	GPIOR1,ACOCh1,U		;skip next if user has ADCn selected
	rjmp	SLPcommand2a		;using analog comp input, continue

	STOREI	ADCSRA,(1<<ADIF),T	;clear ADCSRA to disable ADC
	STOREI	ADCSRB,(1<<ACME),T	;set ACME bit for ACO MUX select
;********************************
; not sure why this code was here
;	LOAD	temp,ADMUX			;get current ADMUX
;	andi	temp,~(15<<MUX0)	;strip off MUX bits
;	or		temp,U				;add in AC MUX select bits
;	STORE	ADMUX,temp			;save it
;********************************

SLPcommand2a:
	SETB	ACSR,ACI,T			;insure no pending AC interrupts
	GoSub	ADCden				;disable ADC hardware and short delay
.endif

SLPcommand3:	;enter "sleep", all hardware interrrupts should be enabled
	sei							;insure global interrupts are enabled
	sleep						;sleep
	nop							;for benefit of the aleep instruction

.if (RTC || DDS)	;only if DDS or RTC enabled
	LOAD	U,GPIOR0			;get IRQ flags into U
	andi	U,(1<<RTCIRQ|1<<DDSIRQ)	;mask out IRQ flags
	breq	SLPcommand3a			;not DDS or RTC IRQ, continue
  .if RTC			;only if RTC enabled
	SKBC	GPIOR0,RTCIRQ,U		;skip next if RTC did not interrupt
	CLRB	GPIOR0,RTCIRQ,U		;clear the RTCIRQ flag
  .endif
  .if DDS			;only if DDS enabled
	SKBC	GPIOR0,DDSIRQ,U		;skip next if DDS did not interrupt
	CLRB	GPIOR0,DDSIRQ,U		;clear the DDSIRQ flag
  .endif
.endif

SLPcommand3a:
; Exclude for ATtiny84/85 - odd that I cannot use preprocessor conditional here!
.if !((SIGNATURE_000 == 0x1e) && (SIGNATURE_001 == 0x93) &&(SIGNATURE_002 == 0x0b)) && \
	!((SIGNATURE_000 == 0x1e) && (SIGNATURE_001 == 0x93) &&(SIGNATURE_002 == 0x0c)) 
	;for hardware UART
	SKBC	UCSRA,RXC,temp		;skip next RX_RDY did not interrupt
	ret							;was RX_RDY, so exit
.endif
SLPcommand3b:	;waiting for watchdog to timeout
	SKBS	GPIOR0,WDTIRQ,U		;skip next if WDTIRQ
	rjmp	SLPcommand3c		;check for ACO trigger
	CLRB	GPIOR0,WDTIRQ,U		;clear WDTIRQ flag
	ret							;return to caller
;
SLPcommand3c:	;waiting for ACO to trigger
.if ADConv || AComp	;only if ADC or AC routines enabled.
	SKBS	GPIOR0,ACOIRQ,U		;skip next if ACOIRQ
	rjmp	SLPcommand3			;loop back to sleep
	CLRB	GPIOR0,ACOIRQ,U		;clear WDTIRQ flag
.endif
	ret							;return to caller
;
