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
;******************************************************************
;	Select your controller type here.  Don't forget to set the
;	appropriate fuse bits.
;
; The fallowing MCU's are fully supported
;.include "m2560def.inc"		;256K FLASH, 8K DATA, 4K EEP
.include "usb1286def.inc"	;128K FLASH, 8K DATA, 4K EEP, USB
;.include "m1284pdef.inc"	;128K FLASH, 16K DATA, 4K EEP, USB
;.include "m644pdef.inc"	;64K FLASH, 4K DATA, 2K EEP, USB
;.include "m32u4def.inc"		;32K FLASH, 2.5K DATA, 1K EEP, USB
;.include "m32adef.inc"		;32K FLASH, 2K DATA, 1K EEP
;.include "m32def.inc"		;32K FLASH, 2K DATA, 1K EEP
;.include "m328def.inc"		;32K FLASH, 2K DATA, 1K EEP
;.include "m328pdef.inc"		;32K FLASH, 2K DATA, 1K EEP
;.include "m168def.inc"		;16K FLASH, 1K DATA, 512 EEP
;.include "m168padef.inc"		;16K FLASH, 1K DATA, 512 EEP
;.include "m16def.inc"		;16K FLASH, 1K DATA, 512 EEP
;.include "m88def.inc"		;8K FLASH, 1K DATA, 512 EEP
;.include "m88padef.inc"		;8K FLASH, 1K DATA, 512 EEP
;.include "tn85def.inc"		;8K FLASH, 512 DATA, 512 EEP
;.include "tn84def.inc"		;8K FLASH, 512 DATA, 512 EEP


; The following MCU's are only supported by old V:1.0 code 
;.include "m163def.inc"		;16K FLASH, 1K DATA, 512 EEP
;.include "8515def.inc"		;8K FLASH, 512 DATA, 512 EEP
;.include "2313def.inc"  	;2K FLASH, 128 DATA, 128 EEP
;.include "tn2313def.inc"	;2K FLASH, 128 DATA, 128 EEP  
;.include "tn2313adef.inc"  ;2K FLASH, 128 DATA, 128 EEP

#define BTLDR	1			;"1" to include boot loader code.
#define BTDFU	0			;"0" = CDC Bootloader, "1" = DFU bootloader
#define USI		0			;Leave alone as it is automatically set later
#define USB		1			;"1" to include USB support code (also
							; disables the UART).  Only for M32U4
							; and USB1286!
#define TEENSY	0			;set to "1" to enable TEENSY++ 2.0 build
#define	DREC	0			;set to "1" to enable AVR Data Recorder specific code
#define	TEST	1			;set to "1" to enable code for testing
;******************************************************************
;!!! Minimum clock frequency for 400K TWI operation is 6.4MHz !!!

#define	FCLK_PS  1		;System clock prescaler (1,2,4,8,16,32,64,128 or 256)
;
#ifndef FCLK
;	#warning !!! Minimum clock frequency for 400K TWI operation is 6.4MHz !!!
;	#define FCLK	4000000
#endif
#ifndef FCLK
;	#define FCLK	(8000000/FCLK_PS)	//set Fclk (preprocessor variable)
#endif
#ifndef FCLK
;	#define FCLK	(10000000/FCLK_PS)
#endif
#ifndef FCLK
;	#define FCLK	(12000000/FCLK_PS)
#endif
#ifndef FCLK
	#define FCLK	(16000000/FCLK_PS)
#endif
#ifndef FCLK
	#define FCLK	(20000000/FCLK_PS)
#endif
;
;+++ Break Here +++ Do not move or remove this line, used for automated processing
;
