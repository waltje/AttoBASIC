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
.message "CLK (SysClkPsc) enabled"
 .ifdef CLKPR	; only if the CLKPR register exists
;*************************************************************************
; SCLK: read/write system clock prescaler.  When no command line
;		parameters are present, the current SysClkPsl is returned.  The
;		passed value is checked for validity.
;	Uses: U and T
;*************************************************************************
SCLKcommand:
	PUSHY						;save Y
	ldi		U,LOG2(FCLK_PS)		;load compile-time CLKPS divisor
	GoSub	pushU				;save on stack
	GoSub	Get1stParm			;get the last parameter
	tst		DStkPtr				;any values passed?
	breq	CLKcommand2			;branch if no values passed

	cpi		U,(8<<CLKPS0)		;check for maximum allowable
	brmi	CLKcommand1			;branch if within limits
	Goto	Error_1E			;error 1E, not within range

CLKcommand1:	;set the SysClkPsl to user value
	ldi		T,(1<<CLKPCE)		;load CLKPS enable bit
	STORE	CLKPR,T				;enable register change
	STORE	CLKPR,U				;set it
	rjmp	CLKcommand9			;exit routine

CLKcommand2:	;return the SysClkPsl value on the stack
	LOAD	U,CLKPR				;get the current value
	GoSub	PushU				;save it on the stack

CLKcommand9:
	Goto	PopYret					;restore Y and return
.endif
;
;*************************************************************************
; OSCcommand: Read/Write AVR's oscillator calibration byte
;	Uses: U
;*************************************************************************
OSCcommand:	;Set/read AVR's oscillator calibration byte
	PUSHY						;save Y
	LOAD	U,OSCCAL			;get the current value of OSCCAL
	GoSub	pushU				;save on stack
	GoSub	Get1stParm			;get the last parameter
	STORE	OSCCAL,U			;save the value to OSCCAL
	LOAD	U,OSCCAL			;get the current value of OSCCAL
	POPY						;restore Y
	Goto	PushURet			;exit
;
