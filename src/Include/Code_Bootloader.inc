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
#if defined(__ATtiny84__) || defined(__ATtiny84A__)
	.org	Optiboot	;0x1E00 for OptiBootTiny85-Loader
	 BOOTIMAGE:
   .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for tn84 @ 4MHz enabled"
		.include	"Include/Code_optiboot85-attiny84-4M.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for tn84 @ 8MHz enabled"
		.include	"Include/Code_optiboot85-attiny84-8M.inc"
    .elif f_clk == 10000000
		.message	"Notice: BootloaderAVR for tn84 @ 10MHz enabled"
		.include	"Code_optiboot85-attiny84-10M.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for tn84 @ 16MHz enabled"
		.include	"Code_optiboot85-attiny84-16M.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for tn84 @ 20MHz enabled"
		.include	"Code_optiboot85-attiny84-20M.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATtiny85__)
	.org	Optiboot	;0x1E00 for OptiBootTiny85-Loader
	 BOOTIMAGE:
   .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for tn85 @ 4MHz enabled"
		.include	"Include/Code_optiboot85-attiny85-4M.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for tn85 @ 8MHz enabled"
		.include	"Include/Code_optiboot85-attiny85-8M.inc"
    .elif f_clk == 10000000
		.message	"Notice: BootloaderAVR for tn85 @ 10MHz enabled"
		.include	"Include/Code_optiboot85-attiny85-10M.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for tn85 @ 16MHz enabled"
		.include	"Include/Code_optiboot85-attiny85-16M.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for tn85 @ 20MHz enabled"
		.include	"Include/Code_optiboot85-attiny85-20M.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega168__) || defined(__ATmega168PA__)
   .org	SECONDBOOTSTART		;0x1F00 for OptiBoot-Loader
   BOOTIMAGE:
   .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for m168 @ 4MHz enabled"
		.include	"Include/Code_BootloaderAVR-m168-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for m168 @ 8MHz enabled"
		.include	"Include/Code_BootloaderAVR-m168-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for m168 @ 16MHz enabled"
		.include	"Include/Code_BootloaderAVR-m168-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for m168 @ 20MHz enabled"
		.include	"Include/Code_BootloaderAVR-m168-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega16__) || defined(__ATmega16L__)
   .org	SECONDBOOTSTART		;0x1F00 for OptiBoot-Loader
   BOOTIMAGE:
   .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for m16 @ 4MHz enabled"
		.include	"Include/Code_BootloaderAVR-m16-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for m16 @ 8MHz enabled"
		.include	"Include/Code_BootloaderAVR-m16-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for m16 @ 16MHz enabled"
		.include	"Include/Code_BootloaderAVR-m16-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for m16 @ 20MHz enabled"
		.include	"Include/Code_BootloaderAVR-m16-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega328__) || defined(__ATmega328P__)
    .org	FIRSTBOOTSTART		;0x3F00 for OptiBoot-Loader
    BOOTIMAGE:
    .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for m328 @ 4MHz enabled"
		.include	"Include/Code_BootloaderAVR-m328-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for m328 @ 8MHz enabled"
		.include	"Include/Code_BootloaderAVR-m328-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for m328 @ 16MHz enabled"
		.include	"Include/Code_BootloaderAVR-m328-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for m328 @ 20MHz enabled"
		.include	"Include/Code_BootloaderAVR-m328-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega644P__)
    .org	FIRSTBOOTSTART		;0x3F00 for OptiBoot-Loader
    BOOTIMAGE:
    .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for m644P @ 4MHz enabled"
		.include	"Include/Code_BootloaderAVR-m644p-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for m644P @ 8MHz enabled"
		.include	"Include/Code_BootloaderAVR-m644p-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for m644P @ 16MHz enabled"
		.include	"Include/Code_BootloaderAVR-m644p-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for m644P @ 20MHz enabled"
		.include	"Include/Code_BootloaderAVR-m644p-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega1284P__)
    .org	FIRSTBOOTSTART		;0x3F00 for OptiBoot-Loader
    BOOTIMAGE:
    .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for m1284P @ 4MHz enabled"
		.include	"Include/Code_BootloaderAVR-m1284p-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for m1284P @ 8MHz enabled"
		.include	"Include/Code_BootloaderAVR-m1284p-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for m1284P @ 16MHz enabled"
		.include	"Include/Code_BootloaderAVR-m1284p-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for m1284P @ 20MHz enabled"
		.include	"Include/Code_BootloaderAVR-m1284p-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega32__) || defined(__ATmega32A__)
    .org	FIRSTBOOTSTART		;0x3F00 for OptiBoot-Loader
    BOOTIMAGE:
    .if f_clk == 4000000
		.message	"Notice: BootloaderAVR for m32 @ 4MHz enabled"
		.include	"Include/Code_BootloaderAVR-m32-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderAVR for m32 @ 8MHz enabled"
		.include	"Include/Code_BootloaderAVR-m32-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderAVR for m32 @ 16MHz enabled"
		.include	"Include/Code_BootloaderAVR-m32-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderAVR for m32 @ 20MHz enabled"
		.include	"Include/Code_BootloaderAVR-m32-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega32U4__) && USB && BTDFU	; Use LUFA DFU (4KB in size)
  .org		FOURTHBOOTSTART
  BOOTIMAGE:
  .if f_clk == 8000000
		.message	"Notice: BootloaderDFU for m32u4 @ 8MHz enabled"
		.include	"Include/Code_BootloaderDFU_atmega32u4-8M.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderDFU for m32u4 @ 16MHz enabled"
		.include	"Include/Code_BootloaderDFU_atmega32u4-16M.inc"
    .else
    	.message "Bootloader DFU is not supported for this clock speed!"
    	.message "Bootloader DFU will not be included."
  .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega32U4__) && USB && !BTDFU	; Use LUFA CDC  (4KB in size)
  .org		FOURTHBOOTSTART	
  BOOTIMAGE:
  .if f_clk == 8000000
		.message	"Notice: BootloaderCDC for m32u4 @ 8MHz enabled"
		.include	"Include/Code_BootloaderCDC_atmega32u4-8M.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderCDC for m32u4 @ 16MHz enabled"
		.include	"Include/Code_BootloaderCDC_atmega32u4-16M.inc"
    .else
    	.message "Bootloader CDC is not supported for this clock speed!"
    	.message "Bootloader CDC will not be included."
  .endif
;--------------------------------------------------------------------------------
#elif defined(__AT90USB1286__) && USB && BTDFU
	#if TEENSY
	  .org		FIRSTBOOTSTART	;The TEENSY++ 2.0 bootloader is 2K words
	#else
	  .org		FOURTHBOOTSTART	;LUFA DFU is 4K bytes
	#endif
   BOOTIMAGE:
   .if f_clk == 8000000
		.message	"Notice: BootloaderDFU for AT90usb1286 @ 8MHz enabled"
		.include	"Include/Code_BootloaderDFU_at90usb1286-8M.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderDFU for AT90usb1286 @ 16MHz enabled"
		.include	"Include/Code_BootloaderDFU_at90usb1286-16M.inc"
    .else
    	.message "Bootloader DFU is not supported for this clock speed!"
    	.message "Bootloader DFU will not be included."
  .endif
;--------------------------------------------------------------------------------
#elif defined(__AT90USB1286__) && USB && !BTDFU
	#if TEENSY
	  .org		FIRSTBOOTSTART	;The TEENSY++ 2.0 bootloader is 2K words
	#else
	  .org		FOURTHBOOTSTART	;LUFA CDC is 4K bytes
	#endif
   BOOTIMAGE:
   .if f_clk == 8000000
		.message	"Notice: BootloaderCDC for AT90usb1286 @ 8MHz enabled"
		.include	"Include/Code_BootloaderCDC_at90usb1286-8M.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderCDC for AT90usb1286 @ 16MHz enabled"
		.include	"Include/Code_BootloaderCDC_at90usb1286-16M.inc"
    .else
    	.message "Bootloader CDC is not supported for this clock speed!"
    	.message "Bootloader CDC will not be included."
  .endif
;--------------------------------------------------------------------------------
#elif defined(__ATmega2560__)
  .org		FOURTHBOOTSTART			;STK500V2 is 4K bytes
   BOOTIMAGE:
    .if f_clk == 4000000
		.message	"Notice: BootloaderSTK for m2560 @ 4MHz enabled"
		.include	"Include/Code_BootloaderSTK-m2560-4mhz.inc"
    .elif f_clk == 8000000
		.message	"Notice: BootloaderSTK for m2560 @ 8MHz enabled"
		.include	"Include/Code_BootloaderSTK-m2560-8mhz.inc"
    .elif f_clk == 16000000
		.message	"Notice: BootloaderSTK for m2560 @ 16MHz enabled"
		.include	"Include/Code_BootloaderSTK-m2560-16mhz.inc"
    .elif f_clk == 20000000
		.message	"Notice: BootloaderSTK for m2560 @ 20MHz enabled"
		.include	"Include/Code_BootloaderSTK-m2560-20mhz.inc"
    .else
		.error "Bootloader is not supported for this clock speed!"
    .endif
#endif
