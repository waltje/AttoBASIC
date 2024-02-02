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
;******************************************************************************
; SPM support for storing data in FLASH memory.
;  On parts containing a bootloader, the SPM command MUST
;  be executed from the NRWW memory area, otherwise SPM can be executed
;  from anywhere in the FLASH program space.
; - Call with DO_SPM with SPM register value in temp register
;******************************************************************************
#if defined(__ATmega88__) || defined(__ATmega88PA__) || \
		defined(__ATmega168__) || defined(__ATmega168PA__) || \
		defined(__ATmega328__) || defined(__ATmega328P__)
	#define SPMEN	SELFPRGEN			;redefine SPMEN bit
#endif
;
;******************************************************************************
SPM_test:
	ldi		inbyteh,0			;point to 1st page
	GoSub	SPGM_PageErase		;erase the data space
	GoSub	SPGM_DataWrite		;write the data
	ret

;******************************************************************************
; SPGM_DataWrite: Writes the data to the data space.
;	- YH:YL holds pointer to data in scratchpad RAM
;	- inbytel holds the number of elements being passed
;	+ uses temp, temp2, U, r0, r1 and Z pointer
;******************************************************************************
SPGM_DataWrite:
	PUSHY						;save Y pointer
	SetYPtr	2*SPM_DATA			;Y pointer to data region (byte address)
	ldi 	U,PAGESIZE			; load page size counter (64 bytes)
	ldi		inbyteh,RES_PAGE_CNT;load loop counter for # of reserved pages

;	ldi		temp,0x55
;	mov		r0,temp
;	ldi		temp,0xaa
;	mov		r1,temp
	clr		r0
	clr		r1

SPGM_DataWrite1:
	SetZPtr	0					;pointer beginning of temp buffer

SPGM_DataWrite2:	;load the temporary buffer
	ldi		temp2, (1<<SPMEN)
	GoSub	Do_spm
	adiw	ZH:ZL,2				;increment temp buffer pointer
	dec		U					;data counter
	brne	SPGM_DataWrite2		;loop till zero

	; SPM - write to FLASH
	CopyYtoZ					;copy Y to Z
	ldi		temp2, (1<<PGWRT) | (1<<SPMEN)
	GoSub	Do_spm				;write the data to FLASH
	ADDI16	YH,YL,2*PAGESIZE	;increment page pointer
	dec		inbyteh				;decrement page counter
	brne	SPGM_DataWrite1		;loop till zero

	POPY						;restore Y pointer
	ret
;
;******************************************************************************
; SPGM_PageErase:  Erase the reserved data space.
;	- Call with page # to erase in inbyteh (zero inclusive)
;	- Uses temp2, inbyteh, inbytel, r1, r0 and ZH:ZL.
;******************************************************************************
SPGM_PageErase:
	ldi		inbyteh,0			;point to 1st page
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*SPM_DATA)	;load RAMPZ
.endif
	SetZPtr	2*SPM_DATA			;pointer to data region (byte address)
	ldi		inbytel,PAGESIZE	;get FLASH page size (in bytes)
	lsl		inbyteh				;multiply page # by 2 for byte address
	mul		inbyteh,inbytel		;get 16-bit page number into r1:r0
	ADD16	ZH,ZL,r1,r0			;add offset to Z
;	ldi		inbyteh,RES_PAGE_CNT;load loop counter for # of reserved pages
SPGM_PageErase1:
	ldi		temp2, (1<<PGERS) | (1<<SPMEN);page erase command
	GoSub	Do_spm				;erase the page
;	ADDI16	ZH,ZL,2*PAGESIZE	;increment page pointer
;	dec		inbyteh				;decrement loop counter
;	brne	SPGM_PageErase1		;loop till zero
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,0				;clear RAMPZ
.endif
	ret							;return to caller
;
;******************************************************************************
; Do_spm:	- Saves SREG
;			- Disables IRQ's
;			- Waits for EEPROM if busy
;			- Executes the SPM instruction
;			- Re-enables FLASH page (if MCU supports it)
;	- Call with SPMCSR function in temp2 register
;	- Uses T
;******************************************************************************
Do_spm:		;Disable interrupts if enabled, store status
	LOAD	T,SREG				;save SREG
	cli
Wait_ee:	;Check that no EEPROM write in progress
	sbic	EECR,EEPE			;loop till EEPE bit clear
	rjmp	Wait_ee
	GoSub	Exec_SPM			;execute SPM instruction
	STORE	SREG,T				;restore SREG
	ret							;return to caller
;
