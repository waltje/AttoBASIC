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
;/////////REGISTER ALLOCATION////////
	.def	romreg			= r0	;Used by LPM and MUL command
;	.def					= r1	;Used by MUL command
	.def	bytecntr		= r2	;used to count bytes (TWI too!)
	.def	currentcast		= r3	;Used in parsing text.
	.def	currlooptgt		= r4	;FOR-NEXT
	.def	currloopvar		= r5	;FOR-NEXT
	.def	loopreturnL		= r6	;FOR-NEXT
	.def	loopreturnH		= r7	;FOR-NEXT
;	.def					= r8	;for flags of all sorts
	.def	RTCcntr			= r9	;hold RTC interval countdown
	.def	LOOPStkPtr		= r10	;FOR_NEXT loop stack pointer
	.def	GoSubStkPtr		= r11	;GOSUB stack pointer
	.def	DStkPtr			= r12	;Data stack pointer.
	.def	ecode			= r13	;Cast returned here and error code stored here
	.def	accl			= r14	;DDS phase accumulator low-byte
	.def	accm			= r15	;DDS phase accumulator mid-byte
	.def	temp 			= r16	;General purpose working register #1
	.def	temp2	 		= r17	;General purpose working register #2
	.def	cmdparms		= r18	;relative number of parameters supplied
	.def	inchar 			= r19	;Char destined to go out the uart
	.def	outchar			= r20	;Char coming in from the uart
	.def	U				= r21	;Units
	.def	T				= r22	;Tens
	.def	H				= r23	;Hundreds
	.def	inbytel 		= r24	;registers used to pass int16_t variables
	.def	inbyteh 		= r25	;  to/from USB library routines

	;XL (r26) as flag register bit useage.
	.equ	VARSTK			= 0		;1 for variables to leave pointers on stack, =0 for values on stack
	.equ	VPTRSTK			= 1		;1 to indicate there is a variable pointer on the dstack
	.equ	SKPNXT			= 2		;1 to skip next line during RUN execution
	.equ	GSJAMR			= 3		;1 to tell runcommand to jamb GOSUB return address
	.equ	LPJAMN			= 4		;1 to tell runcommand to jam in loop return address next time
	.equ	LPCAPA			= 5		;1 to tell runcommand to capture loop return address
	.equ	GSCAPA			= 6		;1 to tell runcommand to capture GOSUB return address
	.equ	GSJAMD			= 7		;1 to tell runcommand to jamb destination address

	;XH (r27) as flag register bit useage.
	.equ	HALT			= 0		;1 to tell runcommand to halt
	.equ	SUSP			= 1		;1 to indicate that suspend had been requested
	.equ	RUNACT			= 2		;1 to indicate that the RUN command is executing
	.equ	GSACT			= 3		;1 to indicate that GOSUB is active
	.equ	FNACT			= 4		;1 to indicate that FOR-NEXT is active
	.equ	FNLEQ			= 5		;1 to switch = to be part of FOR-NEXT loop
	.equ	NOEXEC			= 6		;1 to indicate not to execute the keyword
	.equ	PRIACT			= 7		;1 to indicated a PRINT command is active
;	.equ	FNLNXT			= 3		;1 to activate Next as part of a loop (not used)

	.def	acch	=r30			;phase accumulator high-byte
;	.def	ZH		=r31			;Z-Reg High Byte

