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
 ;Note that the stack pointer is init'ed upon hardware reset
;
Init:
	cli							;disable global interrupts
	clr		r0					;zero some registers
	clr		r1					;zero some registers
	rcall	WDDisable			;disable watchdog timer
	rcall	SYSCLKINIT			;set system clock prescaler
	ser		temp				;set 0xff for PRR settings

;enable power reduction on all modules
.ifdef PRR
	STORE	PRR,temp			;disable all hardware modules
.endif
.ifdef PRR0
	STORE	PRR0,temp			;disable all hardware modules
.endif
.ifdef PRR1
	STORE	PRR1,temp			;disable all hardware modules
.endif

Init1:
#if !TEST
 .if SStst	;check for self-start
	CLRB	GPIOR2,SSTEn,temp	;insure SSTEn flag is cleared
	SETB	SS_PORT+2,SS_PIN	;enable port pull-up
	GoSub	Delay100uS			;delay 100uS
	LOAD	temp,SS_PORT		;get input port
	GoSub	Delay100uS			;delay 100uS
	CLRB	SS_PORT+2,SS_PIN	;disable port pull-up
	sbrc	temp,SS_PIN			;skip next if SS_PORT,SS_PIN is low (self-start)
	rjmp	init2
	SETB	GPIOR2,SSTEn,temp	;set Self-Start Enable flag
 .endif
#endif

 .if CFG		; Check CONFIG register for status of self-start
	CLRB	GPIOR2,SSTEn,temp	;insure SSTEn flag is cleared
	SetYPtr	AB_CFG0				;point Y to AB_CFG0 byte in EEP
	GoSub	ReadEEP				;fetch config byte into "temp"
	mov		inchar,temp			;save a copy in inchar
	sbrs	temp,CFG_SSTRT		;skip next if CFG_SSTRT flag set
	rjmp	init2
	SETB	GPIOR2,SSTEn,temp	;set Self-Start Enable flag
 .endif

Init2:
;Port setup for Power-Up
;------------------------
.ifdef PORTA
	ldi		temp,PrtAdir		;port A DDR
	STORE	DDRA,temp
	ldi		temp,PrtAPup
	STORE	PORTA,temp
.endif
;------------------------
#if DREC	;enable/disable AVR Data Recorder features
 .ifdef PORTB
	ldi		temp,PrtBdir		;port B DDR
	ori		temp,(1<<DIGOUT)	;port bit for digital outpuy
	STORE	DDRB,temp
	ldi		temp,PrtBPup
	ori		temp,(0<<DIGOUT)	;port bit for digital output
	STORE	PORTB,temp
 .endif
#else
 .ifdef PORTB
	ldi		temp,PrtBdir		;port B DDR
	STORE	DDRB,temp
	ldi		temp,PrtBPup
	STORE	PORTB,temp
 .endif
#endif
;------------------------
#if DREC	;enable/disable AVR Data Recorder features
 .ifdef PORTC
	ldi		temp,PrtCdir		;port C DDR
	ori		temp,(  (1<<AD536CH) | \
					(1<<AD536OS) )	;port bits for AD536
	STORE	DDRC,temp
	ldi		temp,PrtCPup
	STORE	PORTC,temp

	ldi		temp,PrtDdir		;port D DDR
	ori		temp,(  (1<<ADC0_R0) | \
					(1<<ADC0_R1) | \
					(1<<ADC1_R0) | \
					(1<<ADC1_R1) )	;port bits for DREC gain
 .endif
#else
 .ifdef PORTC
	ldi		temp,PrtCdir		;port C DDR
	STORE	DDRC,temp
	ldi		temp,PrtCPup
	STORE	PORTC,temp
	ldi		temp,PrtDdir		;port D DDR
 .endif
#endif
;------------------------
.ifdef PORTD
	ldi		temp,PrtDdir		;port D DDR
	STORE	DDRD,temp
	ldi		temp,PrtDPup
	STORE	PORTD,temp
.endif
;------------------------
.ifdef PORTE
	ldi		temp,PrtEdir		;port E DDR
	STORE	DDRE,temp
	ldi		temp,PrtEPup
	STORE	PORTE,temp
.endif
;------------------------
.ifdef PORTF
	ldi		temp,PrtFdir		;port F DDR
	STORE	DDRF,temp
	ldi		temp,PrtFPup
	STORE	PORTF,temp
.endif

;------------------------
.if DSI	;initialize the DS pins
	LOAD	temp,DS_PORT-1
	andi	temp,~( 1<<DS_SDA | \
					1<<DS_SDD )
	STORE	DS_PORT-1,temp		;set DS port bits as inputs

  .if DS_PUPS
  .message "Enabling Port-Pin Pullups for DS interface."
	LOAD	temp,DS_PORT-2
	ori		temp,(  1<<DS_SDA | \
					1<<DS_SDD )
	STORE	DS_PORT,temp		;set pullups on DS port bits
  .endif
.endif
;
;------------------------
.if (DHT && RTC)	;initiate the DHT pin as input w/ pullup
	CLRB	DHPORT-1,DHTPIN		;set DATA pin to output
	SETB	DHPORT,DHTPIN		;enable DATA pin pullup

	LOAD	temp,GPIOR2			;get DHT flags
	andi	temp,~((1<<DHTSign) | (1<<DHTUnit))
	STORE	GPIOR2,temp			;clear flags to default

	STOREI	RT_NRC,3			;preset no-response counter

	SetZPtr	HTRdTmr 			;seed DHT read timeout
	STORE16	RT_TMR,ZH,ZL
.endif
;
Init3:	; Set up system variables
	clr		r1					;clear a register
	STORE	MCUSR,r1			;clear MCUSR flags

	STORE	GPIOR0,r1			;clear all GPIOR0 flags
#if DREC	;enable AVR Data Recorder features
	STOREI	GPIOR1,(1<<ACOCh1)	;clear all GPIOR1 flags
#else
	STORE	GPIOR1,r1			;clear all GPIOR1 flags
#endif
	LOAD	temp,GPIOR2			;get GPIOR2
	andi	temp,(1<<SSTEn)		;preserve SSTEn flag
	STORE	GPIOR2,temp			;save GPIOR2 flags
;
.ifdef RAMPZ					;need RAMPZ to access >64KW FLASH
	STORE	RAMPZ,r1				;clear RAMPZ register
.endif
	STORE	CurrLine,r1			;clear current line counter
;
	GoSub	MkRndSeed			;generate random seed before clearing RAM

.if DFR	;data file variables
	clr		r1					;clear register
	STORE	DFLaddr,r1			;zero the internal address counter
	STORE	DFLaddr+1,r1
	STOREI	DFDfmt,1			;set data dump format to "1"
	STOREI	DFIbyte,DFI_BYTE	;set the bulk erase value
.endif
;
.if ICP
	STOREI	ICPCfg,1,temp		;init ICPCfg RAM variable
.endif
;
.if EFS
;====================================================================
;File system support.  Check the EFS index and initialize with all
;  file handles and sizes being "0" if they contain 0xFF (erased EEP?).
;NOTE: does not check for valid programs stored at each file handle.
;====================================================================
	ldi		U,HANDLES_MAX		;load byte counter
	SETYPTR	EFSIDX0				;Y = 1st byte of file index pointers
Init3a:
	GoSub	ReadEEP				;get the byte
	cpi		temp,0xFF			;is it blank?
	brne	Init3b				;no, branch
	clr		temp				;zero the byte
	GoSub	WriteEEP			;write it
Init3b:
	ADDI16	YH,YL,1				;increment EEP pointer
	dec		U					;decrement loop counter
	brne	Init3a				;loop
;
.endif
;
Init4:	; Set up serial I/O
	GoSub	InitSIO				;initialize the Serial I/O

	sei							;enable global interrupts otherwise no serial I/O
;
Init5:	; Initialize other hardware
;set sleep mode to idle
	ldi		temp,(0<<SM0|1<<SE)
#if defined(__ATtiny84__) || \
	defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || \
	defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	STORE	MCUCR,temp
#else
	STORE	SMCR,temp
#endif
;
.if ADConv	;preload ADC registers
	STOREI	ADMUX,ADMUX_L,temp
	STOREI	ADCSRB,ADCSRB_L,temp
	STOREI	ADCSRA,ADCSRA_L,temp
.endif

#if DREC	;enable AVR Data Recorder features
	ldi		temp,(  (1<<ADCH0) | \
					(1<<ADCH1) | \
					(1<<ADCH2) | \
					(1<<ADCH3) )	;port bits for DREC DIDR0
	STORE	DIDR0,temp				;disable digital inputs
#endif

.if RTC	;Real-time Counter init
	Gosub	RTC_TIMINIT			;enable real-time counter
	STOREI	RTCMul,RTCMul_0		;save default multiplier value in RAM
.endif
;
Init6:		; Check for self-start and execute if so
.if CFG || SStst
	; Check flag for status of self-start pin
	SKBS	GPIOR2,SSTEn,temp	;skip next if SSTEn flag clear
	rjmp	ReadyToRun			;Self-Start request, execute from EEPROM
;
 .if !EFS		; load program direct from EEP (no EFS)
	GoSub	loadcommand			;load program (no EFS)
 .else			; load program "0" from EFS
	INITDSTACK					;Initialize data stack pointer
	clr		U					;default to program '0'
	sbr		XH,(1<<RUNACT)		;set flag to indicate we are executing a command
	GoSub	LOADcommanda		;load program (EFS)
 .endif
	GoSub   runcommand

	GoSub	crlf				;print two lines
	GoSub	crlf
	SETZPTR (2*signonmessage)	;Print 1st part of sign-on message
	GoSub	sendlromline		;sent it.
	ret							;return to caller
.endif
;
ReadyToRun:
	GoSub	newprogram			;Initialize program memory
								; also spits out short sign-on
	ret
;
;*************************************************************************************
; WDDisable:  Disabled watch-dog timer.
;	Uses: temp
;*************************************************************************************
WDDisable:
	PushStat					;save status register (preserve I flag)
	wdr							;reset WDR
	LOAD	temp,MCUSR			;clear WDRF flag
 	andi	temp,~(1<<WDRF)
	STORE	MCUSR,temp

	; Write logical one to WDCE and WDE
	; Keep old prescaler setting to prevent unintentional time-out
	cli							;disable interrupts to set WDT
	LOAD	temp,WDTCSR			;get WDTCSR

#if defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	ori		temp,(1<<WDTOE) | (1<<WDE)
#else
	ori		temp,(1<<WDCE) | (1<<WDE)
#endif
	STORE	WDTCSR,temp

	; Turn off WDT
	andi	temp,0<<WDE
	STORE	WDTCSR,temp

	PopStat						;restore I flag
	ret
;
;*************************************************************************
; SYSCLKINIT - Sets the system clock prescaler
;	Uses: U and T
;*************************************************************************
;
SYSCLKINIT:
#if !defined(__ATmega16__) && \
	!defined(__ATmega16L__) && \
	!defined(__ATmega32__) && \
	!defined(__ATmega32A__)
	ldi		U,(1<<CLKPCE)			;load CLKPS enable bit
	ldi		T,LOG2(FCLK_PS)			;load CLKPS divisor
	STORE	CLKPR,U
	STORE	CLKPR,T					;set it
#endif
	ret
;
;//////// Moved from "Code_Analog.inc" ///////////////////
;*********************************************************
; ADCen:	Enables the ADC hardware by clearing the PRR
;*********************************************************
ADCen:
#if defined(__ATtiny84__) || \
	defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || \
	defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
	CLRB	PRR,PRADC,temp		;enable ADC in PRR
#elif defined(__ATmega32U4__) || \
		defined(__ATmega644P__) || \
		defined(__ATmega1284P__) || \
		defined(__AT90USB1286__) || \
		defined(__ATmega2560__)
	CLRB	PRR0,PRADC,temp		;enable ADC in PRR0
#endif
	STOREI 	ADCSRA,ADCSRA_L,temp;Start an initial conversion
	WAIT4ADCRDY					;wait for ADC ready
	GoSub	Delay10uS			;delay a short time
	ret							;return to caller
;
;//////// Moved from "Code_Analog.inc" ///////////////////
;*********************************************************
; ADCden:	Dis-enables the ADC hardware by setting the PRR
;*********************************************************
ADCden:
#if defined(__ATtiny84__) || \
	defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || \
	defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
	SETB	PRR,PRADC,temp		;disable ADC in PRR
#elif defined(__ATmega32U4__) || \
		defined(__ATmega644P__) || \
		defined(__ATmega1284P__) || \
		defined(__AT90USB1286__) || \
		defined(__ATmega2560__)
 .if LPMD || ADConv || AComp
	SETB	PRR0,PRADC,temp		;disable ADC in PRR0
 .endif
#endif
	GoSub	Delay10uS			;delay a short time
	ret							;return to caller
;
;***************************************************************
