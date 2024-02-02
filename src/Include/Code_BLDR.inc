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
;#if (! DREC)
.equ	EraseByte	= 0xFF		;Erased FLASH byte
.equ	NOPByte		= 0x00		;OpCode for NOP

BLDRcommand:	;determine and jump to the bootloader if it exists
	;it is prudent to read the lock bits and determine where the
	; bootloader is located before attempting to access it
	; HIGH fuse bits: BOOTSZ1 & BOOTSZ0
	; For ATmega32U4 and ATmega328
	;	 256 words = FIRSTBOOTSTART		= 0x3f00
	;	 512 words = SECONDBOOTSTART	= 0x3e00
	;	1024 words = THIRDBOOTSTART		= 0x3c00
	;	2048 words = FOURTHBOOTSTART	= 0x3800
	; For ATmega2560
	;	 512 words = FIRSTBOOTSTART		= 0xfe00
	;	1024 words = SECONDBOOTSTART	= 0xfc00
	;	2048 words = THIRDBOOTSTART		= 0xf800
	;	4096 words = FOURTHBOOTSTART	= 0xf000
	; For AT90usb1286
	;	 512 words = FIRSTBOOTSTART		= 0x1fe00
	;	1024 words = SECONDBOOTSTART	= 0x1fc00
	;	2048 words = THIRDBOOTSTART		= 0x1f800
	;	4096 words = FOURTHBOOTSTART	= 0x1f000
	; For ATmega168
	;	 256 words = FIRSTBOOTSTART		= 0x1f00
	;	 512 words = SECONDBOOTSTART	= 0x1f00
	;	1024 words = THIRDBOOTSTART		= 0x1e00
	;	2048 words = FOURTHBOOTSTART	= 0x1c00

	rcall	BTSZdetect			;get the bootloader start address
	PUSHZ						;save Z-Ptr
;
BLDRcommand1:	;valid code was detected
	SETZPTR (2*invokebldr1)		;Point Z to message
	GoSub	sendromstring		;send it

	SETZPTR (2*invokebldr2)		;Point Z to message
	GoSub	sendromstring		;send it

	POPZ						;restore Z
	mov		inbytel,ZH			;get ZH
	GoSub	sendbyte			;print it
	mov		inbytel,ZL			;get ZL
	GoSub	sendbyte			;print it
	GoSub	crlf				;print CR/LF combo
	GoSub	crlf

	clr		r1					;zero a register

#if ( USB && ( defined(__ATmega32U4__) || defined(__AT90USB1286__) ) )
	Gosub	usb_serial_flush_output	;flush the output buffer
	Gosub	Delay100mS			;delay 100mS

BLDRcommand1b:
	SKBS	UEINTX,TXINI,temp	;skip next if USB buffer has flushed
	rjmp	BLDRcommand1b		;loop till last message sent

	Gosub	usb_shutdown		;terminate current USB connection
#endif
	cli							;disable global interrupts
;
.ifdef EIMSK
	STORE	EIMSK,r1			;disable all interupt sources
.endif
.ifdef GIMSK
	STORE	GIMSK,r1			;disable all interupt sources
.endif
.ifdef PCMSK
	STORE	PCMSK,r1
.endif
.ifdef USICR
	STORE	USICR,r1
.endif
;
.ifdef ACSR
	STORE	ACSR,r1
.endif
.ifdef EECR
	STORE	EECR,r1
.endif
;
.ifdef TIMSK0
	STORE	TIMSK0,r1
.endif
.ifdef TIMSK1
	STORE	TIMSK1,r1
.endif
.ifdef TIMSK2
	STORE	TIMSK2,r1
.endif
.ifdef TIMSK3
	STORE	TIMSK3,r1
.endif
.ifdef TIMSK4
	STORE	TIMSK4,r1
.endif
.ifdef TIMSK5
	STORE	TIMSK5,r1
.endif
.ifdef UCSR0B
	STORE	UCSR0B,r1
.endif
.ifdef UCSR1B
	STORE	UCSR1B,r1
.endif
.ifdef TWCR
	STORE	TWCR,r1
.endif
.ifdef ADCSRA
	STORE	ADCSRA,r1
.endif
.ifdef DDRA
	STORE	DDRA,r1
.endif
.ifdef DDRB
	STORE	DDRB,r1
.endif
.ifdef DDRC
	STORE	DDRC,r1
.endif
.ifdef DDRD
	STORE	DDRD,r1
.endif
.ifdef DDRE
	STORE	DDRE,r1
.endif
.ifdef DDRF
	STORE	DDRF,r1
.endif
.ifdef DDRG
	STORE	DDRG,r1
.endif
.ifdef DDRH
	STORE	DDRH,r1
.endif
.ifdef DDRJ
	STORE	DDRJ,r1
.endif
.ifdef DDRK
	STORE	DDRK,r1
.endif
.ifdef DDRL
	STORE	DDRL,r1
.endif
.ifdef PORTA
	STORE	PORTA,r1
.endif
.ifdef PORTB
	STORE	PORTB,r1
.endif
.ifdef PORTC
	STORE	PORTC,r1
.endif
.ifdef PORTD
	STORE	PORTD,r1
.endif
.ifdef PORTE
	STORE	PORTE,r1
.endif
.ifdef PORTF
	STORE	PORTF,r1
.endif
.ifdef PORTG
	STORE	PORTG,r1
.endif
.ifdef PORTH
	STORE	PORTH,r1
.endif
.ifdef PORTJ
	STORE	PORTJ,r1
.endif
.ifdef PORTK
	STORE	PORTK,r1
.endif
.ifdef PORTL
	STORE	PORTL,r1
.endif

;#if BTLDR
;	STOREI	RAMEND,BLDMJKEY		;store majick byte
;#endif

	STOREI	MCUSR,(1<<PORF),temp	;set PORF flag before boot

	cli							;disable interrupts to set WDT
;
;#if defined(__ATtiny85__)
;	ijmp						;jump to Bootloader
;
;#else
	;set the WD to "interrupt mode" (0<<WDE | 1<<WDIE) and
	; set to interrupt in 8 seconds
	STOREI	WDTCSR,(1<<WDCE|1<<WDE),temp
	STOREI	WDTCSR,(1<<WDCE|1<<WDE|WDT_8S),temp
	sei							;enable global interrupts
	sleep						;sleep
	nop							;for benefit of the aleep instruction

	rjmp	PC+0				;loop forever in case of spurious interrupt
;#endif
;
;*************************************************************************
;BTSZdetect: determine the bootloader address, if it exists.
;	It is prudent to read the lock bits and determine where the
; 	bootloader is located before attempting to access it.
; 	HIGH fuse bits: BOOTSZ1 & BOOTSZ0
;		 256 words = FIRSTBOOTSTART		= 0x3f00
;		 512 words = SECONDBOOTSTART	= 0x3e00
;		1024 words = THIRDBOOTSTART		= 0x3c00
;		2048 words = FOURTHBOOTSTART	= 0x3800
;*************************************************************************
BTSZdetect:
;#if defined(__ATtiny85__)
;	SetZPtr	Optiboot			;point Z to Optiboot vector (s.b. 0x1E00)
;	ret
;#else
	SETZPTR	FUSEBYTE			;set Z to BtSz FuseByte
 #if 	defined(__ATmega16A__) || \
		defined(__ATmega168__) || \
		defined(__ATmega168PA__) || \
		defined(__ATmega32A__) || \
		defined(__ATmega328__) || \
		defined(__ATmega328P__)	;
	STOREI	SPMCSR,(1<<SELFPRGEN | 1<<BLBSET);SPM and BLB Enabled
 #elif (defined(__ATmega32U4__) || \
 		defined(__ATmega2560__) || \
		defined(__ATmega644P__) || \
		defined(__ATmega1284P__) || \
 		defined(__AT90USB1286__) )
	STOREI	SPMCSR,(1<<SPMEN | 1<<BLBSET);SPM and BLB Enabled
 #else
 	#error	"Please select Mega168/328, Mega644P/1284P, Mega16A/32A, Mega2560, Mega32U4 or USB1286 as Target!"
 #endif
;
BTSZdetect1:
	lpm		temp,Z				;get low fuse into temp
	lsr		temp				;shift BOOTSZ1:0 right
	andi	temp,(BOOTSZ1 | BOOTSZ0)	;mask the bits

	cpi		temp,0				;2048 words?
	brne	BTSZdetect1a		;no, check next
	SetZPtr	FOURTHBOOTSTART		;point Z to bootstart
	rjmp	BTSZdetect2			;be sure boot-loader exists

BTSZdetect1a:
	cpi		temp,1				;1024 words?
	brne	BTSZdetect1b		;no, check next
	SetZPtr	THIRDBOOTSTART		;point Z to bootstart
	rjmp	BTSZdetect2			;be sure boot-loader exists

BTSZdetect1b:
	cpi		temp,2				;512 words?
	brne	BTSZdetect1c		;no, check next
	SetZPtr	SECONDBOOTSTART		;point Z to bootstart
	rjmp	BTSZdetect2			;be sure boot-loader exists

BTSZdetect1c:
	cpi		temp,3				;256 words?
	brne	BTSZdetect1d		;no, eroor?
	SetZPtr	FIRSTBOOTSTART		;point Z to bootstart
	rjmp	BTSZdetect2			;be sure boot-loader exists

BTSZdetect1d:	;this is an error condition!!
	STOREI	ERRCode,$0B			;error code "B"
	Goto	error

BTSZdetect2:	;be sure boot-loader exists
	PUSHZ						;save Z-Ptr
	ldi		temp,8				;"not good code" counter
	mov		r1,temp				;copy to r0
.ifdef RAMPZ
	STOREI	RAMPZ,BYTE3(FLASHEND+1)	;set 64K page number
.endif
	rol		ZL					;multiply Z by 2 for LPM instr.
	rol		ZH
BTSZdetect2a:
.ifdef RAMPZ
	elpm	temp,Z+				;get byte @ RAMPZ:Z and increment
.else
	lpm		temp,Z+				;get byte @ Z and increment
.endif
	cpi		temp,EraseByte		;is it an erased byte?
	breq	BTSZdetect2b		;branch if erased byte
	cpi		temp,NOPByte		;is it a NOP instruction?
	breq	BTSZdetect2b		;branch if NOP instr.
	POPZ						;restore Z-Ptr
	ret							;return with Bootloader in Z-Ptr
;
BTSZdetect2b:
	dec		r1					;decrement counter
	brne	BTSZdetect2a		;loop till zero
BTSZdetect2g:	;no valid code detected so error
	STOREI	ERRCode,$15			;error code "21"
	Goto	error
;#endif
;#endif
