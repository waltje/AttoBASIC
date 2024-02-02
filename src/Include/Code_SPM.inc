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
.message "Enabled SPM support routines"

#if defined(__ATmega88__) || defined(__ATmega88PA__) || \
		defined(__ATmega168__) || defined(__ATmega168PA__) || \
		defined(__ATmega328__) || defined(__ATmega328P__)
	#define SPMEN	SELFPRGEN	;redefine SPMEN bit
#elif defined(__ATmega16__) || defined(__ATmega16L__) || \
		defined(__ATmega32__) || defined(__ATmega32A__)
	.ifndef SPMCSR
	  .equ	SPMCSR	= SPMCR		;redefine SPMCR for compatibility	
	.endif
#endif
;
;******************************************************************************
; RPMcommand: Reads data from the data space held in FLASH memory.
;	- Uses U, r1, inchar and Y and Z pointers.
;******************************************************************************
;#if TEST ;!!! For testing
;	PUSHSTAT
;	cli
;#endif
RPMcommand:
	PUSHZ						;save Z pointer
	PUSHY						;save Y pointer
	GoSub	SPM_test			;check for existnce of SPM exec routines
	GoSub	ChkCmdParms			;process line
	brne	RPMcommand1			;branch if data available
	Goto	Error_07			;no data available, error

RPMcommand1:	;command line parameter passed, see if the address exceeds
				; the number of elements stored.
	clr		bytecntr			;clear a register
	SetYPtr	DATASTK+DATASTKSZ	;point Y to top of data stack element
	GoSub	PopU				;fetch the desired address

.if (PGMmaxEl * RES_PAGE_CNT) < 256	;only reuired of <256 bytes of WPM memory
	cpi		U,(PGMmaxEl*RES_PAGE_CNT); parameter in range?
	brmi	RPMcommand1a		;branch if in range
	Goto	Error_1E			;out of range, error
.endif

RPMcommand1a:
	SetZPtr	2*SPM_DATA			;point Z to the data table (byte address)

RPMcommand2:	;command line parameter passed, see if the address exceeds
	ADD16	ZH,ZL,bytecntr,U	;adjust pointer to the offset
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*SPM_DATA)	;load RAMPZ
	elpm	U,Z					;fetch the element
.else
	lpm		U,Z					;fetch the element
.endif
	GoSub	PushU				;save it on the dat astack
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,0				;clear RAMPZ
.endif
	POPY						;restore Y pointer
	POPZ						;restore Z pointer
;#if TEST ;!!! For testing
;	PUSHSTAT
;#endif
	ret							;return to caller
;
;******************************************************************************
; WPMcommand: Writes data to the data space in FLASH.
; 				Reads the parameters from the command line and stores them
;				in the data space located in FLASH memory.  Always adds 
;				existing data if not starting on a PAGESIZE boundary.
;	- Uses U, temp, temp2, inbyteh, inbytel, DStkPtr, X, Y and Z pointers.
;******************************************************************************
WPMcommand:
;#if TEST ;!!! For testing
;	PUSHSTAT					;save SREG
;	cli
;#endif
	PUSHX						;save X pointer
	PUSHY						;save Y pointer
	PUSHZ						;save Z pointer

	GoSub	SPM_test			;check for existance of SPM exec routines

	GoSub	ChkCmdParms			;process line
	brne	WPMcommand1			;branch if data available
	Goto	Error_07			;no data available, error

WPMcommand1: 
;Check to see if there was an even address specified, If not, error.
	clr		bytecntr			;clear a register
	LOAD	temp,(DATASTK+DATASTKSZ-1);fetch start address (1st element)
	mov		H,temp				;save a copy in H
	mov		T,temp				;save a copy in T
	sbrc	H,bit0				;skip next if the address is an even number
	Goto	Error_1F			;not an even address, error code "1F"

.if ((PGMmaxEl * RES_PAGE_CNT) < 256)
	cpi		temp,2*(RES_PAGE_CNT*PAGESIZE);is the address within the range?
	brmi	WPMcommand1a		;continue if yes
	Goto	Error_1E			;out of range, error code "1E"
.endif

;If the PAGESIZE is less than 0x80 (256 bytes) then we need to 
; calculate the page to which we must write to.
WPMcommand1a:
.if PAGESIZE < 0x80
	GoSub	WPM_GetPage			;calculate the page we are writing to
								; returned in then inbyteh register
.else
	clr		inbyteh				;only page 0 for 256 byte PAGESIZE
.endif
;
;----------------------------------------------------------------------
;First, copy the contents of the page in FLASH to the temporary buffer.
; At this point, inbyteh holds the PAGE number in FLASH, so we need to
; point Y to the beginning of the resulting PAGE before copying the 
; data from the FLASH into the temporary buffer.
;----------------------------------------------------------------------
	GoSub	WPM_GetFLASHaddr	;target address in FLASH -> YH:YL
	CopyYtoX					;save a copy in X
	ANDI16	YH,YL,~(PAGESIZE-1)	;get start of the PAGE
;	ANDI16	YH,YL,(2*PAGESIZE)	;get start of the PAGE
.if RES_PAGE_CNT > 1	;for 256 byte data space
	or		YH,XH				;add back the high byte
.else
;	ORI16	YH,YL,2*(SPM_DATA)	;add back start address
.endif
	GoSub	WPM_StoreLPM		;fetch data from FLASH and store
;
;----------------------------------------------------------------------
;Second, merge the new data into the temporary buffer.
; H contains the target address of the new data, Subtract off the 
; PAGESIZE to calculate the actual address within the temporary buffer.
;----------------------------------------------------------------------
	SUBI16	XH,XL,(2*SPM_DATA)	;zero-adjust to within PAGE boundary.
	andi	XL,(2*PAGESIZE)-1	;get index into the PAGE

	CopyXtoZ					;copy X to Z
		
	mov		inbytel,DStkPtr		;fetch number of elements on DStk
	dec		inbytel				;adjust for address parameter
	SetYPtr	(DATASTK+DATASTKSZ-1);point Y to top of data stack element (less 1)
	GoSub	WPM_StoreData		;load the data from RAM into temp buffer
;
;----------------------------------------------------------------------
;Third, erase the target PAGE before writing the data into it.
; inbyteh holds page to erase, returns with ZH:ZL pointing to 
; the beginning of the proper page in FLASH
;----------------------------------------------------------------------
	GoSub	SPGM_PageErase
;
;----------------------------------------------------------------------
;Fourth, write the temporary buffer to the data space
; ZH:ZL already holds the start address of the page.
;----------------------------------------------------------------------
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*SPM_DATA)	;load RAMPZ
.endif
	ldi		temp2,(1<<PGWRT | 1<<SPMEN)
	GoSub	Do_spm				;write the data to FLASH

.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,0				;clear RAMPZ
.endif
	POPZ						;restore Z pointer
	POPY						;restore Y pointer
	POPX						;restore X pointer
;#if TEST ;!!! For testing
;	POPSTAT						;restore SREG
;#endif
	ret							;return to caller
;
;******************************************************************************
; EPMcommand: Erases the data in the reserved data space in FLASH.
;	- Uses outchar and inbyteh
;******************************************************************************
EPMcommand:
;#if TEST ;!!! For testing
;	PUSHSTAT
;	cli
;#endif
	GoSub	SPM_test			;check for existance of SPM exec routines

.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*SPM_DATA)	;load RAMPZ
.endif
	ldi		outchar,RES_PAGE_CNT;fetch the number of pages to erase
	clr		inbyteh				;start with page 0

EPMcommand1:
	GoSub	SPGM_PageErase		;erase the data space
	inc		inbyteh				;increment page
	dec		outchar				;decrement loop counter
	brne	EPMcommand1			;loop till zero

;#if TEST ;!!! For testing
;	POPSTAT
;#endif
	ret							;return to caller
;
;******************************************************************************
; WPM_StoreLPM: Copies the page contents held in FLASH to the temporary buffer.
;	- Call with:
;		YH:YL pointing to PAGE start address in FLASH
;	- Increments YH:YL before returning
;	- Uses inchar, r1. r0, YH and YL
;******************************************************************************
WPM_StoreLPM: 
	ldi 	inchar,PAGESIZE		;load page size counter
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*SPM_DATA)	;set extended address
.endif
	SetZPtr	0					;clear pointer into temporary buffer

;.ifdef CTPB	; if CTPB exists
;	ldi		temp2,(1<< CTPB | 1<<SPMEN)	;clear the temporary buffer
;	GoSub	Do_spm				;execute the SPM instructions
;.endif

;!!!!! test !!!!!
;	clr		r0
;	clr		r1

;WPM_StoreLPMa:
;	ldi		temp2,(1<<SPMEN)	;write to the temporary buffer
;	GoSub	Do_spm				;execute the SPM instructions
;	ADDI16	ZH,ZL,2				;increment temp buffer pointer
;	dec		inchar				;decrement PAGE counter
;	brne	WPM_StoreLPMa		;loop till entire PAGE xferred to buffer
;	ret

;!!!!! test !!!!!

WPM_StoreLPM1: 
	m_SwapYZ					;swap Y and Z
	lpm		r0,Z+				;fetch data into r0 (odd)
	lpm		r1,Z+				;fetch data into r1 (even)
	m_SwapYZ					;swap Y and Z
	ldi		temp2,(1<<SPMEN)	;write to the temporary buffer
	GoSub	Do_spm				;execute the SPM instructions
	ADDI16	ZH,ZL,2				;increment temp buffer pointer
	dec		inchar				;decrement PAGE counter
	brne	WPM_StoreLPM1		;loop till entire PAGE xferred to buffer
	ret							;return to caller
;
;******************************************************************************
; WPM_StoreData: Stuffs the temporary buffer with the data held in
;	RAM in the datastack.
;	- Call with:
;		inbytel with # of elements to store
;		YH:YL pointing to address in data stack
;		ZH:ZL pointing to address in temporary buffer
;	- Increments ZH:ZL before returning
;	- Uses r1 and r0
;******************************************************************************
WPM_StoreData:	;load the temporary buffer with the data elements
	clr		r1					;always clear r1
	ld		r0,-Y				;fetch data into r0 (odd)
	dec		inbytel				;decrement element pointer
	breq	WPM_StoreData2		;branch if no more elements (odd)
WPM_StoreData1:
	ld		r1,-Y				;fetch data into r1 (even)
	dec		inbytel				;decrement element pointer

WPM_StoreData2:	;load the temporary buffer with r1:r0
	ldi		temp2,(1<<SPMEN)	;write to the temporary buffer
	GoSub	Do_spm				;execute the SPM instructions
	adiw	ZH:ZL,2				;increment temp buffer pointer
	CPI16	ZH,ZL,2*PAGESIZE	;check for PAGESIZE rollover
	breq	WPM_StoreData3		;PAGESIZE rollover, exit
	tst		inbytel				;element pointer = 0?
	brne	WPM_StoreData		;loop till out of data elements
WPM_StoreData3:
	ret							;return to caller
;
;******************************************************************************
; WPM_GetFLASHaddr: Get the target address in FLASH and return it in YH:YL.
; - call with: H contains tartget address derived from command parameter.
; - uses bytecounter, which is assumed to be "0".
;******************************************************************************
WPM_GetFLASHaddr:
	SetYPtr	2*SPM_DATA			;point Y to the data table (byte address)
	ADD16	YH,YL,bytecntr,H	;get the desired address for new data
	ret							;return to caller
;
.if PAGESIZE < 0x80
;******************************************************************************
; WPM_GetPage: Calculates the page number in FLASH for parts with less than 
;	256 bytes per FLASH page.
;	- Call with: start address in temp register
;	- Returns with: page number in inbyteh register
;	- Uses temp, temp2
;******************************************************************************
WPM_GetPage:
	ldi		temp2,PAGESIZE		;load PAGESIZE in bytes

WPM_GetPage1:
	lsr		temp				;rotate bits right
	lsr		temp2				;rotate bits right
	brcc	WPM_GetPage1		;loop till we found the page
	mov		inbyteh,temp		;copy to page # tp inbyteh register
	ret							;return to caller
;
.endif
;
;******************************************************************************
; SPGM_PageErase:  Erase the reserved data space.
;	- Call with page # to erase in inbyteh (zero inclusive)
;	- Uses temp, temp2, inbyteh, inbytel, r1, r0 and ZH:ZL.
;******************************************************************************
SPGM_PageErase:
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*SPM_DATA)	;load RAMPZ
.endif
	SetZPtr	2*SPM_DATA			;pointer to data region (byte address)
.if AVRhasMUL
	ldi		inbytel,PAGESIZE	;get FLASH page size (in bytes)
	lsl		inbyteh				;multiply page # by 2 for byte address
	mul		inbyteh,inbytel		;get 16-bit page number into r1:r0
	ADD16	ZH,ZL,r1,r0			;add offset to Z
.endif
SPGM_PageErase1:
	ldi		temp2,(1<<PGERS | 1<<SPMEN);page erase command
	GoSub	Do_spm				;erase the page
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,0				;clear RAMPZ
.endif
	ret							;return to caller
;
;******************************************************************************
; SPM_test:	test for SPM instruction sequence being available in NRWW memory.
;			Errors if not byte match of SPM_EXEC = 0xbf 0x17 0x95 0xe8
;	- Uses inbyteh, inbytel, inchar, outchar, Y and Z pointers
;******************************************************************************
SPM_test:
	.warning "SPM_EXEC test disabled!"
	ret							;all good, return to caller

	SetZPtr	2*Exec_SPM			;set Z to beginning of SPM exec routine
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,BYTE3(2*Exec_SPM)	;set extended address
	elpm	inbytel,Z+			;fetch 1st byte
	elpm	inbyteh,Z+			;fetch 2nd byte
	elpm	outchar,Z+			;fetch 3rd byte
	elpm	inchar,Z			;fetch 4th byte
.else
	lpm		inbytel,Z+			;fetch 1st byte
	lpm		inbyteh,Z+			;fetch 2nd byte
	lpm		outchar,Z+			;fetch 3rd byte
	lpm		inchar,Z			;fetch 4th byte
.endif
	cpi		inbyteh,0xbf		;compare 1st byte to opcode
	brne	SPM_test1			;no match, error
	cpi		inbytel,0x17		;compare 2nd byte to opcode
	brne	SPM_test1			;no match, error
	cpi		inchar,0x95			;compare 3rd byte to opcode
	brne	SPM_test1			;no match, error
	cpi		outchar,0xe8		;compare 4th byte to opcode
	brne	SPM_test1			;no match, error
.ifdef RAMPZ					;for 64+KB of FLASH
	STOREI	RAMPZ,0				;clear RAMPZ
.endif
	ret							;all good, return to caller

SPM_test1:
	Goto	Error_1B			;error, SPM exec not found!
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
