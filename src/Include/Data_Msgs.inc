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
;*****************************************************************
;***	Strings stored in program memory
;*****************************************************************
; Tokens are used for enhanced printing as follows:
;	0xf8 before a string will center it using the constant LNLEN
;		as the maximum line length.
;	0xf9 followed by a number then a printable character will
;		repeat that character the number of times specified.
;	0xfa emits a CR/LF combination.
;	0xfb emits the string "Error ".
;
; !!!! WARNING: do not allow the assembler to pad these !!!!
;		strings with 0x00, fill with a space at the end.
;*****************************************************************
;
.equ	TOK_CTR		= 0xF8		;the "center line" token
.equ	TOK_REP		= 0xF9		;the "repeat character" token
.equ	TOK_CRLF	= 0xFA		;the CR/LF token
.equ	TOK_ERR		= 0xFB		;the "Error" string token
.equ	TOK_RNG		= 0xFC		;the "value exceeded range" string token
.equ	TOK_NOT		= 0xFD		;the " not " string token
.equ	TOK_SUP		= 0xFE		;the " support (C)" string token
;
#if ( (FCLK/10000000) == 0 )
  #define CLOCKRATE	$20,(FCLK/1000000)+$30
#else
  #define CLOCKRATE	(FCLK/10000000)+$30,((FCLK/1000000)%10)+$30
#endif

signonmessage:
#if (defined(__ATtiny84__) || defined(__ATtiny84A__)) && TEST
	.db     "AttoBASIC ATtiny84(A) Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,,TOK_CRLF
#elif defined(__ATtiny85__) && TEST
	.db     "AttoBASIC ATtiny85 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30," ",TOK_CRLF
#elif defined(__ATmega88__) && TEST
	.db     "AttoBASIC ATmega88 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30," ",TOK_CRLF
#elif defined(__ATmega88PA__) && TEST
	.db     "AttoBASIC ATmega88PA Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30," ",TOK_CRLF
#elif defined(__ATmega168__) && TEST
	.db     "AttoBASIC ATmega168 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__ATmega168PA__) && TEST
	.db     "AttoBASIC ATmega168PA Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif (defined(__ATmega16__) || defined(__ATmega16A__)) && TEST
	.db     "AttoBASIC ATmega16(A) Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__ATmega328__) && TEST
	.db     "AttoBASIC ATmega328 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__ATmega328P__) && TEST
	.db     "AttoBASIC ATmega328PA Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif (defined(__ATmega32__) || defined(__ATmega32A__)) && TEST
	.db     "AttoBASIC ATmega32(A) Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__ATmega32U4__) && TEST
	.db     "AttoBASIC ATmega32U4 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30," ",TOK_CRLF
#elif defined(__ATmega644P__) && TEST
	.db     "AttoBASIC ATmega644P Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__ATmega1284P__) && TEST
	.db     "AttoBASIC ATmega1284P Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__AT90USB1286__) && TEST
	.db     "AttoBASIC AT90USB1286 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30,TOK_CRLF
#elif defined(__ATmega2560__) && TEST
	.db     "AttoBASIC ATmega2560 Ver: ", \
		(Version/100)+$30,'.',\
		((Version%100)/10)+$30, \
		((Version%100)%10)+$30," ",TOK_CRLF
#endif
;
#if !TEST
	.db		TOK_REP,LNLEN,"=",TOK_CRLF
#endif
;
#if (defined(__ATtiny84__) || defined(__ATtiny84A__)) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATtiny84(A) @",CLOCKRATE,"MHz",TOK_CRLF
#elif defined(__ATtiny85__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATtiny85 @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__ATmega88__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega88 @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__ATmega88PA__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega88PA @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__ATmega168__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega168 @",CLOCKRATE,"MHz",TOK_CRLF
#elif defined(__ATmega168PA__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega168PA @",CLOCKRATE,"MHz",TOK_CRLF
#elif defined(__ATmega328__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega328P @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__ATmega328P__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega328PA @",CLOCKRATE,"MHz",TOK_CRLF
#elif (defined(__ATmega16__) || defined(__ATmega16A__)) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega16(A) @",CLOCKRATE,"MHz",TOK_CRLF
#elif (defined(__ATmega32__) || defined(__ATmega32A__)) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega32(A) @",CLOCKRATE,"MHz",TOK_CRLF
#elif defined(__ATmega32U4__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega32U4 @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__ATmega644P__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega644P @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__ATmega1284P__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega1284 @",CLOCKRATE,"MHz ",TOK_CRLF
#elif defined(__AT90USB1286__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for AT90USB1286 @",CLOCKRATE,"MHz",TOK_CRLF
#elif defined(__ATmega2560__) && !TEST && !DREC
	.db     TOK_CTR,"AttoBASIC for ATmega2560 @",CLOCKRATE,"MHz ",TOK_CRLF
#elif !TEST && DREC
	.db		TOK_CTR,"AVR Data Recorder ",TOK_CRLF
	.db		TOK_CTR,"Using AttoBASIC for ATmega328P @",CLOCKRATE,"MHz ",TOK_CRLF
#endif

#if !TEST	; generate the build code
	.db		TOK_CTR,"Ver: ", \
			(Version/100)+$30,'.',\
			((Version%100)/10)+$30, \
			((Version%100)%10)+$30," ", \
			"Build code: ", \
			(__CENTURY__/10)+0x30, \
			(__CENTURY__%10)+0x30, \
			(__YEAR__/10)+0x30,(__YEAR__%10)+0x30, \
			(__MONTH__/10)+0x30,(__MONTH__%10)+0x30, \
			(__DAY__/10)+0x30,(__DAY__%10)+0x30, \
			".",(__HOUR__/10)+0x30, \
			(__HOUR__%10)+0x30,(__MINUTE__/10)+0x30, \
			(__MINUTE__%10)+0x30,TOK_CRLF,TOK_REP,LNLEN,"="
#endif
	.dw	0
;
hellomessage:
#if !TEST && DREC
	.db		TOK_CTR,"Original AttoBASIC (C)2002 by Richard Cappels ",TOK_CRLF
	.db		TOK_CTR,"http://projects.cappels.org  projects@cappels.org ",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
	.db		TOK_CTR,"Data Recorder & Enhancements (C)2011, 2012",TOK_CRLF
	.db		TOK_CTR," by K. Scott Vitale (ksv_prj@gmx.com) ",TOK_CRLF
	.db		TOK_REP,LNLEN,"=",TOK_CRLF
;----------------------------------
#elif !TEST && !DREC
  #if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
 	defined(__ATtiny85__) || \
	defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_CTR,"Enhanced by K Scott Vitale (ksv_prj@gmx.com)",TOK_CRLF
 #else
	.db		TOK_CTR,"Enhancements by K Scott Vitale (ksv_prj@gmx.com)",TOK_CRLF
 #endif
 #if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
 	defined(__ATtiny85__)
	.db		TOK_CTR,"ATtiny84/85 port (C)2016",TOK_CRLF
 #elif ( defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) ); reduce sign-on message size, be specific
	.db		TOK_CTR,"ATmega88/168/328 port (C)2011 ",TOK_CRLF
 #elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
 	defined(__ATmega32A__)	; reduce sign-on message size, be specific
	.db		TOK_CTR,"ATmega16(L)/32(A) port (C)2016",TOK_CRLF
 #elif defined(__ATmega644P__) || \
	defined(__ATmega1284P__)	; reduce sign-on message size, be specific
	.db		TOK_CTR,"ATmega644P/1284P port (C)2016 ",TOK_CRLF
 #elif defined(__ATmega32U4__)	; reduce sign-on message size, be specific
	.db		TOK_CTR,"ATmega32U4 port (C)2012 ",TOK_CRLF
 #elif defined(__ATmega2560__)	; reduce sign-on message size, be specific
	.db		TOK_CTR,"ATmega2560 port (C)2013 ",TOK_CRLF
 #elif defined(__AT90USB1286__); reduce sign-on message size, be specific
	.db		TOK_CTR,"AT90USB1286 port (C)2013",TOK_CRLF
 #endif
;
 #if ( BTLDR && defined(__AT90USB1286__) ); reduce sign-on message size, be specific
	.db		TOK_CTR,"AT90USB1286 USB & DFU Bootloader",TOK_SUP,"2013 ",TOK_CRLF
 #elif ( BTLDR && (defined(__ATtiny84__) || defined(__ATtiny84A__) || defined(__ATtiny85__)) ); reduce sign-on message size, be specific
	.db		TOK_CTR,"Tiny84/85 Bootloader",TOK_SUP,"2016 ",TOK_CRLF
 #elif ( BTLDR && defined(__ATmega32U4__) ); reduce sign-on message size, be specific
	.db		TOK_CTR,"Mega32U4 USB & DFU Bootloader",TOK_SUP,"2012",TOK_CRLF
 #endif
  .if SPGM
	.db		TOK_CTR,"WPM/RPM",TOK_SUP,"2016",TOK_CRLF
  .endif
  .if USISUPP	; reduce sign-on message size, be specific
	.db		TOK_CTR,"USI UART",TOK_SUP,"2014",TOK_CRLF
  .endif
  .if OWR		; reduce sign-on message size, be specific
	.db		TOK_CTR,"1-Wire(R)",TOK_SUP,"2013",TOK_CRLF
  .endif
  .if NRF		; reduce sign-on message size, be specific
	.db		TOK_CTR,"nRF24L01(+) RF Transceiver",TOK_SUP,"2013 ",TOK_CRLF
  .endif
  .if DHT		; reduce sign-on message size, be specific
	.db		TOK_CTR,"DHT Humidity and Temperature",TOK_SUP,"2013 ",TOK_CRLF
  .endif
  .if EFS		; reduce sign-on message size, be specific
	.db		TOK_CTR,"EEP File System",TOK_SUP,"2013",TOK_CRLF
  .endif
 #if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
  .if (RTC || LPMD || DFR); reduce sign-on message size, be specific
	.db		TOK_CTR,"Sleep",TOK_SUP,"2012",TOK_CRLF
  .endif
  .if (TWI || SPI || DDS || ICP); reduce sign-on message size, be specific
	.db		TOK_CTR,"DDS",TOK_SUP,"2011",TOK_CRLF
  .endif
 #else
  .if (RTC || LPMD || DFR)
	.db		TOK_CTR,"Sleep, RTC and Data-file",TOK_SUP,"2012 ",TOK_CRLF
  .endif
  .if (TWI || SPI || DDS || ICP)
	.db		TOK_CTR,"TWI/SPI/DDS & Input Capture",TOK_SUP,"2011",TOK_CRLF
  .endif
 #endif
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
 #if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
 	defined(__ATtiny85__) || \
	defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_CTR,"Base Code (C)2002 Dick Cappels",TOK_CRLF
 #else
	.db		TOK_CTR,"Original code (C)2002 by Richard Cappels",TOK_CRLF
 #endif
	.db		TOK_CTR,"http://projects.cappels.org  projects@cappels.org ",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
  .if USISUPP 	; reduce sign-on message size, be specific
	.db		TOK_CTR,"USI SIO (C)2003 ATMEL ",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
  .endif
 #if USB && !DREC	; reduce sign-on message size, be specific
	.db		TOK_CTR,"USB Serial I/O (C)2008,2010 PJRC.COM, LLC ",TOK_CRLF
	.db		TOK_CTR,"http://www.pjrc.com/teensy/ ",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
 #endif
 #if BTLDR && ( defined(__ATmega168__) || \
  		defined(__ATmega168PA__) || \
		defined(__ATmega328__) || \
		defined(__ATmega328P__) || \
		defined(__ATmega16__) || \
	  	defined(__ATmega16L__) || \
	  	defined(__ATmega32__) || \
	  	defined(__ATmega32A__) )	; reduce sign-on message size, be specific
  	.db		TOK_CTR,"OptiBoot Bootloader based on stk500boot.c by",TOK_CRLF
	.db		TOK_CTR,"Jason Kyle, Peter Knight, Bill Westfield & Ladyada",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
 #elif BTLDR && (defined(__ATtiny84__) || defined(__ATtiny84A__) || \
 				defined(__ATtiny85__)); reduce sign-on message size, be specific
  	.db		TOK_CTR,"OptiBoot85 Bootloader based on optiboot.c by",TOK_CRLF
	.db		TOK_CTR,"J. Kyle, P. Knight, B. Westfield & Ladyada",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
 #elif BTLDR && (defined(__ATmega644P__) || \
 				defined(__ATmega1284P__) || \
				defined(__ATmega2560__))	; reduce sign-on message size, be specific
	.db     TOK_CTR,"STK500v2 Bootloader Peter Fleury <pfleury@gmx.ch> ",TOK_CRLF
	.db     TOK_CTR," and Worapoht Kornkaewwattanakul <dev@avride.com> ",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
 #elif BTLDR &&  ( defined(__ATmega32U4__) || \
  					defined(__AT90USB1286__) ); reduce sign-on message size, be specific
	.db     TOK_CTR,"LUFA Bootloader Written by Dean Camera",TOK_CRLF
	.db     TOK_CTR,"dean@fourwalledcubicle.com - http://www.lufa-lib.org",TOK_CRLF
	.db		TOK_REP,LNLEN,"-",TOK_CRLF
 #endif
	.db     TOK_CTR,"All rights and remedies expressly reserved",TOK_CRLF
	.db		TOK_CTR,"Not for commercial use",TOK_CRLF
	.db		TOK_REP,LNLEN,"=",TOK_CRLF
#endif
	.dw     0
;
perferate:	;for the dump commands
	.db		TOK_REP,((DUMPBYTES * 3) + 5),"-",TOK_CRLF
	.dw		0
;
MSG_DEBUG:
	.db		"Debug "
	.dw		0

MSG_DBG_ON:
	.db		"on ",TOK_CRLF
	.dw		0

MSG_DBG_OFF:
	.db		"off",TOK_CRLF
	.dw		0

.if DEBUG
MSG_HEADER:
	.db		TOK_REP,6," ","00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F",TOK_CRLF,NULL
;	.db		0

MSG_VRAM:
	.db		"Variables in RAM:",TOK_CRLF
	.dw		0
;
MSG_E2PROM:
	.db		"EEP Contents:",TOK_CRLF
	.dw		0
;
MSG_PROG:
	.db		"PROGRAM Memory Contents:",TOK_CRLF,NULL
;	.dw		0
;
pc_nextavail:
	.db		"Next available location: ",NULL
;	.dw		0
.endif
;
MSG_E2PROMSAVE2BIG:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,": Pgm too large for EEP! ",TOK_CRLF,NULL
#else
	.db		TOK_ERR,": Program too large for EEP! ",TOK_CRLF,NULL
#endif
;
MSG_EEPERASE:
	.db		"Erasing EEP",NULL
;	.dw		0

.if !EFS
MSG_E2PROMSAVE:
	.db		"Saving to EEP.",TOK_CRLF,NULL
;	.dw		0
.endif
;
.if EFS
MSG_SAVEPROGNUM:
	.db		"Saving File # "
	.dw		0
.endif
;
.if !EFS
MSG_E2PROMLOAD:
	.db		"Loading from EEP",TOK_CRLF,NULL
;	.dw		0
.endif
;
.if EFS
MSG_LOADPROGNUM:
	.db		"Loading File # ",NULL
;	.dw		0
.endif
;
.if EFS
MSG_CATEMPTY:
	.db		"Empty",NULL
;	.dw		0
.endif
;
.if EFS
MSG_BYTES:
	.db		" bytes > ",NULL
;	.dw		0
.endif
;
breakmsgtxt:
	.db		"Break",TOK_CRLF
	.dw		0
;
usedmem:
	.db		" bytes used.  "
	.dw		0
;
freemem:
	.db		" bytes free. ",TOK_CRLF
	.dw		0

clrchar:
	.db		CTRL_H,SPACE,CTRL_H,NULL
;	.dw		0
;
;BLDR command messages strings
#if defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
	invokebldr1:
	  .db		TOK_CRLF,"Exit your terminal program."
	  .dw		0
	invokebldr2:
	  .db		TOK_CRLF,"Invoking bootloader (in 8 secs) at 0x"
	  .dw		0
#endif
;
#if ( defined(__ATmega328__) || \
	  defined(__ATmega328P__) || \
	  defined(__ATmega168__) || \
	  defined(__ATmega168PA__) || \
	  defined(__AT90USB1286__)  || \
	  defined(__ATmega2560__) || \
	  defined(__ATmega32U4__) )
WTFmsg1:
	.db		TOK_CTR,"42 is the Answer to the Ultimate Question ",TOK_CRLF
	.db		TOK_CTR,"of Life, the Universe, and Everything ",TOK_CRLF
	.db		TOK_CTR,"(http://www.betemunah.org/fortytwo.html)",TOK_CRLF
	.dw		0
;
WTFmsg2:
	.db		TOK_CTR,"So Long, and Thanks for All the Fish",TOK_CRLF
	.dw		0
#endif
;
; Common words found in the strings
valexcdrng:
	.db		"Value exceeded range"
	.dw		0

notstring:
	.db		" not ",NULL
;	.dw		0

supportstrg:
	.db		" support (C)"
	.dw		0
;
;==================================================================================
;ERROR messages
;==================================================================================
errlinenum:
	.db		"Line # ",NULL

emessage:
	.db		"Error:"
	.dw		0

emessage00:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"CMD/OP Unknown",NULL
#else
	.db		TOK_ERR,"Unknown command or operator.",NULL
#endif
;
emessage01:
	.db		TOK_ERR,TOK_RNG," of 0:255",NULL
;
emessage02:
	.db		TOK_ERR,"Program memory is full",NULL
;
emessage03:
	.db		TOK_ERR,"Character",TOK_NOT,"a recognized type.",NULL
;
emessage04:
	.db		TOK_ERR,"Buffer limit exceeded.",NULL
;
emessage05:
	.db		TOK_ERR,"Bad variable name.",NULL
;
emessage06:
	.db		TOK_ERR,"Too many arguments. ",NULL
;
emessage07:
	.db		TOK_ERR,"Value was",TOK_NOT,"available.",NULL
;
emessage08:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"Stack error ",TOK_CRLF
	.db		TOK_REP,9," ","Func/calls too complex/nested too deep",NULL
#else
	.db		TOK_ERR,"Machine stack error ",TOK_CRLF
	.db		TOK_REP,9," ","Functions too complex or calls nested too deeply",NULL
#endif
;
emessage09:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"GOT or GOS dest",TOK_NOT,"found.",NULL
#else
	.db		TOK_ERR,"GOTO or GOSUB destination",TOK_NOT,"found.",NULL
#endif
;
emessage0A:
	.db		TOK_ERR,"",TOK_RNG," of 0:7",NULL
;
emessage0B:
	.db		TOK_ERR,"Line number > 3 digits",NULL
;
emessage0C:
	.db		TOK_ERR,"",TOK_RNG," of 0:1",NULL
;
emessage0D:
	.db		TOK_ERR,TOK_RNG," of 0:3",NULL
;
emessage0E:
.if ICP
	.db		TOK_ERR,"ICP register overflow.",NULL
.else
	.dw		0
.endif;
;
emessage0F:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"Math overflow.",NULL
#else
	.db		TOK_ERR,"Arithmetic overflow.",NULL
#endif

;
emessage10:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"Math underflow. ",NULL
#else
	.db		TOK_ERR,"Arithmetic underflow. ",NULL
#endif
;
emessage11:
	.db		TOK_ERR,"Divide by zero. ",NULL
;
emessage12:
	.db		TOK_ERR,"Empty File",NULL
;
emessage13:
.if TWI
	.db		TOK_ERR,"TWI",TOK_NOT,"enabled.",NULL
.else
	.dw		0
.endif;
;
emessage14:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.dw		0
#else
	.db		TOK_ERR,"14: ",NULL
#endif
;
emessage15:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.dw		0
#else
	.db		TOK_ERR,": No Boot-loader found."
#endif
;
emessage16:
	.db		TOK_ERR,"Specified ",TOK_RNG,".",NULL
;
emessage17:
	.db		TOK_ERR,"Set AC Ref > 0. ",NULL
;
emessage18:
.if ( DHT || OWR || DSI || TWI)
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"DEV no resp.",NULL
#else
	.db		TOK_ERR,"Device non-response.",NULL
#endif
.else
	.dw		0
.endif
;
emessage19:
.if ( DHT || OWR || DSI )
	.db		TOK_ERR,"Device checksum.",NULL
.else
	.dw		0
.endif
;
emessage1A:
.if SPI
	.db		TOK_ERR,"SPI ",TOK_NOT,"enabled",NULL
.else
	.dw		0
.endif

emessage1B:
.if SPGM
	.db		TOK_ERR,"WPM exec not found! ",NULL
	.dw		0
.endif

emessage1C:
	.db		TOK_ERR,TOK_NOT,"a binary number",NULL

emessage1D:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	.db		TOK_ERR,"Invalid # of args.",NULL
#else
	.db		TOK_ERR,"Invalid number of arguments.",NULL
#endif

emessage1E:	;only needed for MCU's supporting less than 256 bytes of WPM memory
.if SCLK && (SPGM && ((PGMmaxEl * RES_PAGE_CNT) < 256))
	.db		TOK_ERR," Out of range ",NULL
.else
	.dw		0
.endif

emessage1F:
.if SPGM
	.db		TOK_ERR," Even address only! ",NULL
.else
	.dw		0
.endif

