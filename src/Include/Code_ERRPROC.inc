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
Error_06:
	STOREI	ERRCode,$06
	Goto	error
;
Error_07:
	STOREI	ERRCode,$07
	Goto	error
;
Error_08:
	STOREI	ERRCode,$08
	Goto	error
;
Error_0A:
	STOREI	ERRCode,$0A
	Goto	error
;
Error_0C:
	STOREI	ERRCode,$0C
	Goto	error
;
Error_0D:
	STOREI	ERRCode,$0D
	Goto	error
;
Error_11:
	STOREI	ERRCode,$11
	Goto	error
;
Error_12:
	STOREI	ERRCode,$12
	Goto	error
;
Error_16:
	STOREI	ERRCode,$16
	Goto	error
;
Error_1B:
	STOREI	ERRCode,$1B
	Goto	error
;
Error_1D:
	STOREI	ERRCode,$1D
	Goto	error

.if SCLK || (SPGM && ((PGMmaxEl * RES_PAGE_CNT) < 256))
Error_1E:
	STOREI	ERRCode,$1E
	Goto	error
 .endif

.if SPGM
Error_1F:
	STOREI	ERRCode,$1F
	Goto	error
.endif
;
;**********************************************************
;ERROR: Error Processing routine
;***********************************************************
Error:
	GoSub	crlf
	clr		r1					;for 16-bit addition to Z-ptr

	LOAD	r0,ERRCode			;get error code into temp
	mov		ecode,r0			;save a copy
	lsl		r0					;multiply by 2 for byte address
	SetZPtr	(2*ERRORTbl)		;point to string table
	ADD16	ZH,ZL,r1,r0			;add to Z-ptr
	lpm		temp,Z+				;get low word of error string
	lpm		ZH,Z				;get high word of error string
	mov		ZL,temp

	rol		ZL					;multiply address by two for byte offset
	rol		ZH

	GoSub	sendromstring		;sent it.
	GoSub	crlf

	GoSub	PrnLnNo				;print the line number of the error

Error9:
	InitSP						;init stack pointer to RAMEND
	clr		XL					;Clear interpreter mode flags
	clr		XH
	STOREI	CurrLine,0			;clear current line counter
	GoSub	crlf
	Goto	main

ERRORTbl:
	.dw		emessage00				;Error code 0
	.dw		emessage01				;Error code 1
	.dw		emessage02				;Error code 2
	.dw		emessage03				;Error code 3
	.dw		emessage04				;Error code 4
	.dw		emessage05				;Error code 5
	.dw		emessage06				;Error code 6
	.dw		emessage07				;Error code 7
	.dw		emessage08				;Error code 8
	.dw		emessage09				;Error code 9
	.dw		emessage0A				;Error code A
	.dw		emessage0B				;Error code B
	.dw		emessage0C				;Error code C
	.dw		emessage0D				;Error code D
	.dw		emessage0E				;Error code E
	.dw		emessage0F				;Error code F
	.dw		emessage10				;Error code 10
	.dw		emessage11				;Error code 11
	.dw		emessage12				;Error code 12
	.dw		emessage13				;Error code 13
	.dw		emessage14				;Error code 14
	.dw		emessage15				;Error code 15
	.dw		emessage16				;Error code 16
	.dw		emessage17				;Error code 17
	.dw		emessage18				;Error code 18
	.dw		emessage19				;Error code 19
	.dw		emessage1A				;Error code 1A
	.dw		emessage1B				;Error code 1B
	.dw		emessage1C				;Error code 1C
	.dw		emessage1D				;Error code 1D
	.dw		emessage1E				;Error code 1D
	.dw		emessage1F				;Error code 1F
