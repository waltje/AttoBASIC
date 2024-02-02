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
;
; This file holds the code for the Analog Comparator
;#if !defined(__ATtiny85__)	;AIN pins are used on TINY85
;
;********************************************************************
; ACScommand: select the ADC channel for the Analog 
;	Comparator's "AIN1"(-) input.
;	This routine only sets or clears the flag as the "ACOcommand"
;	routine is responsible for selecting the channel
;********************************************************************
ACScommand:
	ldi		U,2
	Gosub	pushU				;Put default channel "2" (AIN-) on stack
	rcall	Get1stParm			;get the last parameter
	cpi		U,5					;only values of 0 through 4 allowed
	brcs	ACScommand1			;carry set, branch
	Goto	Error_16			;error code "16"

ACScommand1:
	LOAD	temp,GPIOR1			;get current flags
	andi	temp,~(7<<ACOCh0)	;strip off ACO select bits
	or		temp,U				;OR in ACO select bits
	STORE	GPIOR1,temp			;save GPIOR1
	ret
;
;********************************************************************
; ACOcommand:	leaves the value of the analog comparator output
;	 on the stack, normally "1" for high and "0" for low but is
;	 inverted for use with the data recorder.
;	Also checks the state of the ACOCh1 and ACOCh0 flags to select
;	 the proper source pin for the analog comp's "AIN0(+)" pin.
;********************************************************************
ACOcommand:		;analog comparator output on stack ("1" for high)
	GoSub	ADCen				;enable ADC hardware for register change

;#endif ;(DREC)

ACOcommand1:
	rcall	ACOinit				;insure Analog Comp is initialized

 #if DREC	;invert output for data recorder
	ldi		U,1
 #else
	clr		U
 #endif
	SKBC	ACSR,ACO,romreg
 #if DREC	;invert output for data recorder
	clr		U
 #else
	ldi		U,1
 #endif
	SETB	ACSR,ACD,temp		;disable analog-comp
	GoSub	ADCden				;disable ADC hardware
	GoSub	pushU				;put value on data stack
	GoSub	interpretlinev		;execute any further commands
	ret
;
;********************************************************************
; ACOinit:	disables the ADC, initialize the analog comparator, sets
;	the AIN1 input pin and disables the croresponding digital input
;	buffer.
; Uses: U and temp
;********************************************************************
ACOinit:	;initialize analog comparator
	STOREI	ADCSRA,0,temp		;turn off ADC
	STOREI	ADCSRA,(1<<ADIF),temp;clear the ADC IRQ flag
	CLRB	ADCSRB,ACME,U		;disable ADCMUX to start

	;enable the AComp
	LOAD	temp,ACSR			;get current value of ACSR
	andi	temp,(1<<ACIE)		;leave ACIE bit alone
	ori		temp,ACSR_L			;OR in the init bits
	STORE	ACSR,temp			;save ACSR

	;determine if an ADC pin is selected for AIN1
	SKBS	GPIOR1,ACOCh2,U		;skip next if AIN1 pin selected
	rjmp	ACOinit1			;using an AIN1 pin directly
#if defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	rjmp	ACOinit3
#else
	ldi		U,(1<<ADC2D)		;bit position for AIN1
	rjmp	ACOinit2			;using the AIN1 pin directly
#endif

ACOinit1:	;determine the selected input pin for AIN1(-)
	SETB	ADCSRB,ACME,U		;enable ADCMUX
	LOAD	U,GPIOR1			;get GPIOR1 flags
	andi	U,(3<<ACOCh0)		;mask ACOCh[1:0] bits

	;select the ADC channel
	LOAD	temp,ADMUX			;get current ADMUX
	andi	temp,~MUXMASK		;strip off MUX bits
	or		temp,U				;add in AC MUX select bits
	STORE	ADMUX,temp			;save it

	GoSub	BitPositiontoORmask1;convert channel in U to bit mask
	dec		DStkPtr				;value already in U, decrement DStkPtr
	dec		cmdparms			; and cmdparms

ACOinit2:
#if defined(__ATtiny85__)  || \
	defined(__ATtiny84__) || defined(__ATtiny84A__)
	STORE	DIDR0,U
#elif !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__)
	STORE	DIDR1,U
#endif

ACOinit3:
	GoSub	Delay100uS			;short delay for bandgap to settle
	ret							;return to caller
;
#if DREC
;********************************************************************
; ACIcommand:	enable or disable the analog comparator's interrupt
;	and set the edge on which to trigger.  Tests for the AC reference
;	being set to > 0, errors if not.
;********************************************************************
ACIcommand:
	rcall	ACOinit				;insure Analog Comp is initialized

	clr		U					;default is to disable interrupts
	GoSub	pushU				;put value on data stack
	rcall	Get1stParm		;get the last parameter
	cpi		U,2					;check for 2 (error if so)
	brcs	ACIcommand1			;not "2+", branch
	Goto	Error_16			;value higher than "1", error

ACIcommand1:
	LOAD	temp,ACSR			;get current ACSR into temp
	andi	temp,(1<<ACD)		;clear all but ACD
	STORE	ACSR,temp			;set ACSR to disable spurious interrupts

	tst		U					;check for 0
	brne	ACIcommand2			;not "0", branch
	ret							;interrupts already disabled, return

ACIcommand2:
	load	T,OCRAL				;fetch the current PWM value
	tst		T					;is it zero?
	brne	ACIcommand3			;greater than 0 so branch

	STOREI	ERRCode,$17			;PWM not set, error
	Goto	error
	ret
ACIcommand3:	;set ACO to trigger on change of state
	ori		temp,(1<<ACI);enable interrupt
	STORE	ACSR,temp			;set ACSR
	ori		temp,((1<<ACI)|(1<<ACIE));enable interrupt
	STORE	ACSR,temp			;set ACSR
	ret
;
#endif ;(DREC)
;
;********************************************************************
; ACI_int:	executed when analog comparator's interrupt triggers.
;	This routine only sets the ACOIRQ flag, which is cleared by the
;	"SLP" command.
;********************************************************************
ACI_int:
	SETB	GPIOR0,ACOIRQ,temp	;set ACOIRQ flag
	reti						;return to caller
;
