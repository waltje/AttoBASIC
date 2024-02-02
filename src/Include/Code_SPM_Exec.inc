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
.message "SPM_EXEC enabled."
;******************************************************************************
; Exec_SPM:  This is the SPM instruction,  For parts with NRWW 
;  memory that does not start at address 0x0000, the SPM 
;  instruction MUST be executed from the protected memory located
;  in the NRWW partition.  Thus we must find a place to locate 
;  8 words (16 bytes) of instructions somewhere in that region, 
;  among a possible boot-loader.  Sigh ...
; - call with desired operation held in the temp2 register.
; - uses temp
;******************************************************************************
Exec_SPM:
	STORE	SPMCSR,temp2		;store the desired operation in SPMCSR
	spm							;execute the SPM instruction
Exec_SPM1:
	SKBC	SPMCSR,SPMEN,temp	;check SPMEN bit for clear
	Goto	Exec_SPM1			;loop till SPM bit clear
.ifdef RWWSB
Exec_SPM2:
	SKBS	SPMCSR,RWWSB,temp	;check RWWSB bit is clear
	ret							;return to caller
.endif
.ifdef RWWSRE	; Re-enable page
Exec_SPM3:
	sbrc	temp2,PGERS			;skip next if page-erase was not performed
	ret 						;return to caller as page-erase performed, setting 
								;  RWWSB bit will erase the temp-buffer
	ldi		temp2,(1<<RWWSRE) | (1<<SPMEN)
	STORE	SPMCSR,temp2		;store the desired operation in SPMCSR
	spm							;execute the SPM instruction
.endif
Exec_SPM9:
	ret							;return to caller
Exec_SPM_end:

