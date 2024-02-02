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
;***************************************************************
; SPM_DATA:  This is the memory space reserved in FLASH for
;  the RPM and WPM commands.  We reserve memory based on the 
;  MCU's FLASH size.  For 8KB or less parts, we reserve one 
;  "PAGESIZE" of memory, which is the block size of a FLASH 
;  block erase.  For parts with 16KB+, reserve a full 256 bytes.
;  The starting address MUST be PAGESIZE page aligned, thus it
;  must be strategically place in program memory to minimize 
;  program memory waste.
;***************************************************************
.org PC + (PAGESIZE - (PC % PAGESIZE)); set program counter for page alignment
SPM_DATA:		; reserve FLASH memory for data storage (MCU specific)
#if TEST
	.dd	0x03020100,0x07060504,0x0B0A0908,0x0F0E0D0C
	.dd	0x13121110,0x17161514,0x1B1A1918,0x1F1E1D1C
	.dd	0x23222120,0x27262524,0x2B2A2928,0x2F2E2D2C
	.dd	0x33323130,0x37363534,0x3B3A3938,0x3F3E3D3C
#else
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	;minimum of 32 words reserved
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	; for all parts.  For 8KB parts
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	;  PAGESIZE=32 or 64 bytes.
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
#endif
.if 2*FLASHEND > 8192	; for 16KB+ parts 
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	;additional reserved words 
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	; for 16+KB parts as
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	; or PAGESIZE=64 (128 bytes)
	.dd	0x00000000,0x00000000,0x00000000,0x00000000	; or PAGESIZE=128 (256 bytes)
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
	.dd	0x00000000,0x00000000,0x00000000,0x00000000
.endif
