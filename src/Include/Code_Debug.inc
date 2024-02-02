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
.message "DEBUG enabled."
;*******************************************************************
;DebugCommand: enable or disable program line display.
;*******************************************************************
DebugCommand:
	clr 	U
	Gosub	pushU				;Put default channel on stack
	GoSub	Get1stParm			;get the last parameter
	GoSub	CheckUisBitVal			;only values of [0..1] allowed

	SETZPTR	(2*MSG_DEBUG)		;print "Debug"
	GoSub	sendromstring		;sent it.

	sbrs	U,bit0				;skip next if bit0 set ("1")
	rjmp	DebugCommand1
	SETZPTR	(2*MSG_DBG_ON)		;print "on"
	SETB	GPIOR2,DebugEn,temp	;enable debug
	rjmp	DebugCommand2		;continue
DebugCommand1:
	SETZPTR	(2*MSG_DBG_OFF)		;print "off"
	CLRB	GPIOR2,DebugEn,temp	;disable debug
DebugCommand2:
	GoSub	sendromstring		;sent it.
	ret
;
;*******************************************************************
; dumpio:	Dumps the the contents of I/O from "0" to IOEND
;*******************************************************************
dumpio:	;dumps 256 bytes of I/O for debug only.
	SETZPTR	(2*MSG_PROG)		;set string pointer
	GoSub	sendromstring		;sent it.
	rcall	Dump_Header			;print the header

	ldi		temp,DUMPBYTES		;# of data bytes per line
	mov		bytecntr,temp		;save it
	SetZPtr	0					;set Z-ptr to start of I/O

dumpio2:
	rcall	Dump_Z				;print value of Z register
	rcall	Dump_Page			;dump a page

	CPI16	ZH,ZL,IOEND+1		;check for end of I/O
	breq	dumpio3

	GoSub	Checkkeybd 			;check for ^S is pressed
	sbrc	XH,HALT				;skip next if no HALT flag
	rjmp	endofpmem2			;user abort, exit
	GoSub	Delay10ms			;delay 10ms

	ldi		temp,DUMPBYTES		;reload byte counter
	mov		bytecntr,temp
	rjmp	dumpio2
;
dumpio3:
	andi	XH,~(1<<HALT)		;clear HALT bit
	rjmp	Dump_Footer			;print the footer and exit
;	ret
;
;*******************************************************************
; dumppmembuffer:	Dumps the contents of RAM starting from
;	0x0000.to RAMEND
;*******************************************************************
dumppmembuffer:	;dumps pmem buffer for debug only.
	SETZPTR	(2*MSG_PROG)		;set string pointer
	GoSub	sendromstring		;sent it.
	rcall	Dump_Header			;print the header

	ldi		temp,DUMPBYTES		;# of data bytes per line
	mov		bytecntr,temp		;save it
	SetZPtr	SRAM_START			;set Z-ptr to start of RAM

anotherchar2:
	rcall	Dump_Z				;print value of Z register
	rcall	Dump_Page			;dump a page

	CPI16	ZH,ZL,RAMEND+1		;check for end of RAM
	breq	endofpmem2

	GoSub	Checkkeybd 			;check for ^S is pressed
	sbrc	XH,HALT				;skip next if no HALT flag
	rjmp	endofpmem2			;user abort, exit
	GoSub	Delay10ms			;delay 10ms

	ldi		temp,DUMPBYTES		;reload byte counter
	mov		bytecntr,temp
	rjmp	anotherchar2
;
endofpmem2:
	andi	XH,~(1<<HALT)		;clear HALT bit
	GoSub	Dump_Perf

	SETZPTR	(2*pc_nextavail)	;print "next available"
	GoSub	sendromstring		;sent it.
	LOAD	inbytel,PCHI		;get PC high byte
	GoSub	sendbyte
	LOAD	inbytel,PCLO		;get PC low byte
	GoSub	sendbyte
	GoSub	crlf				;next line
	rjmp	Dump_Footer			;print the footer and exit
;	ret

;*******************************************************************
; dumpemembuffer:	Dumps the contents of the EEPROM starting from
;	0x0000 to E2END.
;*******************************************************************
dumpemembuffer:	;dumps emem buffer for debug only.
;	GoSub	Dump_Perf			;print perforation to screen
	SETZPTR	(2*MSG_E2PROM)		;set string pointer
	GoSub	sendromstring		;sent it.

	rcall	Dump_Header			;print the header

	ldi		temp,DUMPBYTES		;# of data bytes per line
	mov		bytecntr,temp		;save it
	SetZPtr	0					;Initialize PC position inidcator

	rcall	Dump_Z				;print value of Z register

dumpemembuffer1:
	STORE16	EEARL,ZH,ZL			;store address byte
	SETB	EECR,EERE			;set EEPROM Read strobe
	LOAD	inbytel,EEDR		;get data into outchar
	GoSub	sendbyte
	ldi		outchar,$20			;add "space" between bytes
	GoSub	emitchar
	ADDI16	ZH,ZL,1				;increment EEP addr pointer
	dec		bytecntr			;decrement byte counter
	brne	dumpemembuffer1		;branch if not zero
	GoSub	crlf				;next line

	CPI16	ZH,ZL,E2END+1		;end of EEP?
	breq	endofemem2			;yes, done with dump

	GoSub	Checkkeybd 			;check for ^S is pressed
	sbrc	XH,HALT				;skip next if no HALT flag
	rjmp	endofemem2			;user abort, exit
	rcall	Dump_Z				;print value of Z register
	GoSub	Delay10ms			;delay 10ms

	ldi		temp,DUMPBYTES		;reload byte counter
	mov		bytecntr,temp
	rjmp	dumpemembuffer1
endofemem2:
	andi	XH,~(1<<HALT)		;clear HALT bit
	rjmp	Dump_Footer			;print the footer and exit
;	ret
;
;*******************************************************************
dumpvmembuffer:	;dumps variables in RAM for debug only.
	GoSub	Dump_Perf			;print perforation to screen
	SETZPTR	(2*MSG_VRAM)		;set string pointer
	GoSub	sendromstring		;sent it.

	ldi		temp,DUMPBYTES		;# of data bytes per line
	mov		bytecntr,temp		;save it
	SETZPTR	VARSTOR

dumpvmembuffer1:
	CPI16	ZH,ZL,(VARSTOR+VARSTORSZ);at beginning of variables?
	brsh	endofvmem2			;yes, done with dump
dumpvmembuffer2:
	rcall	Dump_Z				;print value of Z register
	rcall	Dump_Page			;dump a page

	ldi		temp,DUMPBYTES		;reload byte counter
	mov		bytecntr,temp
dumpvmembuffer3:
	rjmp	dumpvmembuffer1
endofvmem2:
	rcall	Dump_Footer			;print the footer
	ret
;
;*************************************************************
; Dump_Page:	Dumps a page (256 bytes) to the console
;	- Enter with start of buffer in ZH:ZL
;	- Enter with number of bytes to dump in "bytecntr"
;*************************************************************
Dump_Page:
	ld		inbytel,Z+			;get data, post increment Z
	GoSub	sendbyte
	ldi		outchar,$20			;add "space" between bytes
	GoSub	emitchar
	dec		bytecntr			;decrement byte counter
	brne	Dump_Page			;jump if not zero
	GoSub	crlf				;next line
	ret							;return to caller
;
;*************************************************************
; Dump_Z:	Dumps the value of the Z register pair to screen
;*************************************************************
Dump_Z:
	mov		inbytel,ZH
	GoSub	sendbyte
	mov		inbytel,ZL
	GoSub	sendbyte
	ldi		outchar,':'
	GoSub	emitchar
	ldi		outchar,' '
	GoSub	emitchar
	ret

;*************************************************************
; Dump_Header:	Dumps the header to console
;*************************************************************
Dump_Header:
	SETZPTR	(2*MSG_HEADER)		;set string pointer
	GoSub	sendromstring		;sent it.
	GoSub	Dump_Perf			;print perforation to screen
	ret
;
;*************************************************************
; Dump_Footer:	Dumps the footer to console
;*************************************************************
Dump_Footer:
	GoSub	Dump_Perf			;print perforation to screen
	GoSub	crlf
	ret
;
;*******************************************************************
; Fillmembuffer: Fills a range of RAM with a byte value
;*******************************************************************
FILLmembuffer:
	GoSub	Get1stParm			;get the last parameter on stack
	mov		inbytel,U			;low byte of count
	GoSub	popU
	mov		inbyteh,U			;high byte of count
	GoSub	popU
	push	U					;save low byte on stack
	GoSub	popU
	push	U					;save high byte on stack
	GoSub	popU				;get the fill value
	mov		temp,U				;save in temp
	pop		ZH					;restore high byte of start address
	pop		ZL					;restore low byte of start address
	GoSub	FillMem				;execute the RAM fill
	ret
;
