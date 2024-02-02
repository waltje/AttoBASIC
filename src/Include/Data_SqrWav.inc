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
.cseg
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
 #if BTLDR && !USI	;DDS table must go before the bootloader library
	.org	( ( ( ( 2 * Optiboot ) / 256) * 256 ) - 256 ) / 2
 #else 	;DDS table must go at end of FLASH
	.org	(FLASHEND + 1) - 128
;	.org	( ( ( ( 2 * FLASHEND ) / 256) * 256 ) - 256 ) / 2
 #endif
;
	;---------------------------------------------------------------------------
	;on builds supporting a bootloadr, always assume there is a bootloader,
	;  even if we don't include one
	;---------------------------------------------------------------------------
#elif defined(__ATmega32U4__)
	.org	LARGEBOOTSTART-128		; Leave room for LUFA bootloader
;
#elif defined(__AT90USB1286__)
  #if TEENSY
	  .org	FIRSTBOOTSTART-128		; The TEENSY++ 2.0 bootloader is 4K words
  #else
;	 .org	THIRDBOOTSTART-128		; LUFA DFU is 2K words
	 .org	FOURTHBOOTSTART-128		; LUFA DFU is 4K words
  #endif
#elif defined(__ATmega2560__)
	 .org	FOURTHBOOTSTART - 128	; bootloader on m2560 is 4K words
;
#elif defined(__ATmega168__) || \
  		defined(__ATmega168PA__) || \
		defined(__ATmega16__) || \
		defined(__ATmega16L__)
	.org	SECONDBOOTSTART - 128	; Leave room for bootloader
;
#elif defined(__ATmega328__) || \
		defined(__ATmega328P__) || \
		defined(__ATmega32__) || \
		defined(__ATmega32A__) || \
		defined(__ATmega644P__) || \
		defined(__ATmega1284P__)

	.org	FIRSTBOOTSTART - 128	; Leave room for bootloader
;
#else
	.org	(FLASHEND + 1) - 8		; word addresses
;
#endif
;
;Lookup table for squarewave output.  256 bytes
.cseg
sqrwave:				; must be page aligned
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
.db	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
sqrwave_end:
