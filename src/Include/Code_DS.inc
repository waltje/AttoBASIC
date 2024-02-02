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
.message "DSI enabled."
;*****************************************************
; dsdcommand:	Send DATA byte via DS port
;*****************************************************
dsdcommand:
	GoSub	Get1stParm			;get the last parameter
	mov		inchar,U
	clc							;clear carry flag
	rcall	SendDSByte
	ret
;
;***********************************************
; dsccommand:	Send COMMAND byte via DS port
;***********************************************
dsccommand:
	GoSub	Get1stParm			;get the last parameter
	mov		inchar,U
	sec							;set carry flag
	rcall	SendDSByte
	ret
;
;***********************************************
dsrcommand:		;Receive a byte vai DS port
	Gosub	ReceiveDSByte
	mov		U,inchar
	Goto	PushURet			;exit

;***********************************************
;	SEND A BYTE
; Send T flag as first bit, then send inchar
;	bit-by-bit. inchar destroyed.
;***********************************************
SendDSByte:
	rcall	WDEnable			;enable WD


	CLRB	GPIOR0,WDTIRQ,temp	;insure WDTIRQ flag is clear

	LOAD	temp,DS_PORT-1
	andi	temp,~( 1<<DS_SDA | \
					1<<DS_SDD )
	STORE	DS_PORT-1,temp		;set DS port bits as inputs

.if DS_PUPS
	rcall	DS_PUPSon			;enable port pull-ups
.endif

	ldi		H,$09				;load bit counter
S1:
;	sleep						;sleep till IRQ
;	nop							;benefit of the sleep instruction
	SKBC	GPIOR0,WDTIRQ,temp	;skip next if WDTIRQ flag clear
	rjmp	S4					;watchdog timeout, exit
	SKBS	DS_PORT-2,DS_SDA	;Wait for ATT line to go high =--SEND A BIT
	rjmp	S1

	CLRB	DS_PORT-1,DS_SDD	;DAT pin to input
.if DS_PUPS
	SETB	DS_PORT,DS_SDD		;DAT pin pullup
.endif
	brcs	S2
	SETB	DS_PORT-1,DS_SDD	;DAT pin to output
	CLRB	DS_PORT,DS_SDD		;DAT pin low
S2:
	SKBC	GPIOR0,WDTIRQ,temp	;skip next if WDTIRQ flag clear
	rjmp	S4					;watchdog timeout, exit
	SETB	DS_PORT-1,DS_SDA	;Pin to output
	CLRB	DS_PORT,DS_SDA		;ATT pin low
	GoSub	Delay1mS			;delay 1mS so other chip can see ATT line is low

	;Release Attention line for a peak
	CLRB	DS_PORT-1,DS_SDA	;ATT pin to input
.if DS_PUPS
	SETB	DS_PORT,DS_SDA		;ATT pin pullup
.endif
	GoSub	Delay1mS			;delay 1mS so other chip can see ATT line is low

	SKBC	DS_PORT-2,DS_SDA	;If ATT line isn't low, go back and put it low again, else continue
	rjmp	S2

	brcs	S2a					;Invert data line
	CLRB	DS_PORT-1,DS_SDD	;DAT pin to input
.if DS_PUPS
	SETB	DS_PORT,DS_SDD		;DAT pin pullup
.endif
	rjmp	S3
S2a:
	CLRB	DS_PORT,DS_SDD		;DAT pin low
	SETB	DS_PORT-1,DS_SDD	;DAT pin output
S3:
;	sleep						;sleep till IRQ
;	nop							;benefit of the sleep instruction
	SKBC	GPIOR0,WDTIRQ,temp	;skip next if WDTIRQ flag clear
	rjmp	S4					;watchdog timeout, exit
	SKBS	DS_PORT-2,DS_SDA	;Wait for Attention line to go high
	rjmp	S3

	CLRB	DS_PORT-1,DS_SDD	;DAT pin input
.if DS_PUPS
	SETB	DS_PORT,DS_SDD		;DAT pin pullup
.endif
	;finished sending a bit

	rol		inchar				;Shift inchar through carry

	dec		H
	brne	S1					;Continue until all bits sent
	rjmp	SendDSByteOut

S4:
	SetZPtr	(2*emessage18)		;Z = message start
	GoSub	sendlromline		;inform user of error

SendDSByteOut:
	rcall	WDDisable			;disable watchdog timer
	rcall	DS_RestPins			;restoere SDA/SDD to inputs
	ret

;***********************************************
;	Recieve a byte
ReceiveDSByte: ;Get a byte into inchar, and "start bit" into carry bit.
	rcall	WDEnable			;enable WD

	LOAD	temp,DS_PORT-1
	andi	temp,~( 1<<DS_SDA | \
					1<<DS_SDD )
	STORE	DS_PORT-1,temp		;set DS port bits as inputs

.if DS_PUPS
	rcall	DS_PUPSon			;enable port pull-ups
.endif

	ldi		H,$09				;load bit counter

W3:					;Get a bit from the input into the carry
	SKBC	GPIOR0,WDTIRQ,temp	;skip next if WDTIRQ flag clear
	rjmp	W4					;watchdog timeout, exit
	SKBC	DS_PORT-2,DS_SDA	;Wait for Attention line to go low
	rjmp	w3

	clc				;Latch DS_SDD into carry bit
	SKBS	DS_PORT-2,DS_SDD
	rjmp	NotaOne
	sec

	CLRB	DS_PORT,DS_SDA		;ATT pin low
	SETB	DS_PORT-1, DS_SDA	;ATT pin output

W1:					;Wait for data line to go low
	SKBC	GPIOR0,WDTIRQ,temp	;skip next if WDTIRQ flag clear
	rjmp	W4					;watchdog timeout, exit
	SKBC	DS_PORT-2,DS_SDD
	rjmp	W1
	rjmp	RelesaseDS_SDAAndGo

NotaOne:
	CLRB	DS_PORT,DS_SDA		;ATT pin low
	SETB	DS_PORT-1, DS_SDA	;ATT pin output

W2:					;Wait for data line to go high
	SKBC	GPIOR0,WDTIRQ,temp	;skip next if WDTIRQ flag clear
	rjmp	W4					;watchdog timeout, exit
	SKBS	DS_PORT-2,DS_SDD
	rjmp	W2

RelesaseDS_SDAAndGo:			;Release attention line
	CLRB	DS_PORT-1, DS_SDA	;ATT pin input
.if DS_PUPS
 	SETB	DS_PORT,DS_SDA		;ATT pin PUP
.endif
	;DONE RECEIVING BIT
	rol		inchar		;Shift inchar through carry
	dec		H
	brne	W3			;Continue until all bits received
	rjmp	ReceiveDSByteOut

W4:
	SetZPtr	(2*emessage18)		;Z = message start
	GoSub	sendlromline		;inform user of error
	ser		U					;set U to 0xFF - error

ReceiveDSByteOut:
	rcall	WDDisable			;disable watchdog timer
	rcall	DS_RestPins			;restoere SDA/SDD to inputs
	ret
;
;***********************************************
; DS_RestPins: Restore SDD and SDA pins to inputs
;***********************************************
DS_RestPins:
	CLRB	DS_PORT-1,DS_SDA 	;SDA to input
	CLRB	DS_PORT-1,DS_SDD	;SDD to input
.if DS_PUPS
 	SETB	DS_PORT,DS_SDA		;ATT pin PUP
.endif
	ret							;return to caller
;
.if DS_PUPS
;***********************************************
; DS_PUPSon: Enable Pull-ups on port pins
;***********************************************
DS_PUPSon:
	LOAD	temp,DS_PORT-2
	ori		temp,(  1<<DS_SDA | \
					1<<DS_SDD )
	STORE	DS_PORT-2,temp		;set pullups on DS port bits
	ret
.endif
;
;***********************************************
; WDEnable: Enable WDT for device non-response.
;			Set for 512mS timeout.
;***********************************************
WDEnable:
	CLRB	GPIOR0,WDTIRQ,temp	;clear WDTIRQ flag
	STOREI	WDTCSR,(1<<WDCE|1<<WDE)
	STOREI	WDTCSR,((1<<WDIE)|WDT_500MS|(0<<WDE))
;	STOREI	WDTCSR,((1<<WDIE)|WDT_1S|(0<<WDE))
;	STOREI	WDTCSR,((1<<WDIE)|WDT_4S|(0<<WDE))
	wdr							;reset watchdog
	ret
;
;***********************************************
;shortdelay:
	;19444 cycles [1+(1+((80*3)+3)*80)+4]
;	ldi		inbytel,settletime	;1 cycles
;outerloop:
;	ldi		inbyteh,settletime	;1 cycles
;innerloop:
;	dec		inbyteh				;1 cycles
;	brne	innerloop			;1/2 cycles
;	dec		inbytel				;1 cycles
;	brne	outerloop			;1/2 cycles
;	ret							;4/5 cycles
;
