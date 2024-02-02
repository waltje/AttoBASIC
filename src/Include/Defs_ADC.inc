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
;****************************************************************
; Analog Comparator Constants
;****************************************************************
#if defined(__ATtiny85__)
 .equ	ACSR_L	= ( (0<<ACD) | \
					(1<<ACBG) | \
					(0<<ACO) | \
					(1<<ACI) | \
					(0<<ACIE) | \
					(0<<ACIS1) | \
					(0<<ACIS0) )	; Use I-Ref on AIN0
#else
 .equ	ACSR_L	= ( (0<<ACD) | \
					(1<<ACBG) | \
					(0<<ACO) | \
					(1<<ACI) | \
					(0<<ACIE) | \
					(0<<ACIC) | \
					(0<<ACIS1) | \
					(0<<ACIS0) )	; Use I-Ref on AIN0
#endif
;
;****************************************************************
; ADC Constants
;****************************************************************
.equ	ADC_Avg		= 16		;number of reads for average (2^N)
.equ	ADC_Shft	= LOG2((1024*ADC_Avg)/256)	;number of right shifts

.equ	ADCK_0	= 0				; stop clock
.equ	ADCK_2	= 1				; Prescaler 2x
.equ	ADCK_4	= 2				; Prescaler 4x
.equ	ADCK_8	= 3				; Prescaler 8x
.equ	ADCK_16	= 4				; Prescaler 16x
.equ	ADCK_32	= 5				; Prescaler 32x
.equ	ADCK_64	= 6				; Prescaler 64x
.equ	ADCK_128= 7				; Prescaler 128x
;
.equ	ADCLK_MAX	= 200000
.if f_clk/EXP2(1) <= ADCLK_MAX	;calculate appropriate prescale value
	.equ		ADCK	= 1
.elif f_clk/EXP2(2) <= ADCLK_MAX
	.equ		ADCK	= 2
.elif f_clk/EXP2(3) <= ADCLK_MAX
	.equ		ADCK	= 3
.elif f_clk/EXP2(4) <= ADCLK_MAX
	.equ		ADCK	= 4
.elif f_clk/EXP2(5) <= ADCLK_MAX
	.equ		ADCK	= 5
.elif f_clk/EXP2(6) <= ADCLK_MAX
	.equ		ADCK	= 6
.elif f_clk/EXP2(7) <= ADCLK_MAX
	.equ		ADCK	= 7
.endif
;
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
; ADMUX Options: Ext. ref, Left Adjust Result, ADC0 selected
#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
  .equ	ADC_Ref			= 110				;110 = 1.1V or 256 = 2.56V
  .equ	ADC_Refm= ( (1<<REFS1) | \
					(1<<REFS0))		;reference mask
  .equ	ADC_Int	= ( (1<<REFS1) | \
					(1<<REFS0))		;Internal reference mask
  .equ	ADC_Ext	= ( (0<<REFS1) | \
					(0<<REFS0))		;External reference mask
  .equ	MUXMASK	= (EXP2(MUX3+1)-1)	;Mask for MUX bits
;
.equ	ADMUX_L	= ( (0<<REFS1) | \
					(0<<REFS0) | \
					(0<<ADLAR) | \
					(0<<MUX3) | \
					(0<<MUX2) | \
					(0<<MUX1) | \
					(0<<MUX0) )
;
; ADC Control Options A: Enable ADC in free-running mode,
;	prescaler configured based upon clock
.set	ADCSRA_L= ( (1<<ADEN) | \
					(1<<ADSC) | \
					(0<<ADATE) | \
					(1<<ADIF) | \
					(0<<ADIE) ) + ADCK
;
; ADC Control Options B: Free-running trigger source
.equ	ADCSRB_L= ( (0<<ACME) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )
;
; DIDR Options: no disable digital input
.equ	DIDR0_L	= ( (0<<ADC3D) | \
					(0<<ADC2D) | \
					(0<<ADC1D) | \
					(0<<ADC0D) )

;
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
  .equ	ADC_Ref			= 256		;110 = 1.1V or 256 = 2.56V
  .equ	ADC_Refm= ( (1<<REFS1) | \
					(1<<REFS0))		;reference mask
  .equ	ADC_Int	= ( (1<<REFS1) | \
					(1<<REFS0))		;Internal reference mask
  .equ	ADC_Ext	= ( (1<<REFS1) | \
					(0<<REFS0))		;External reference mask
;
  .equ	ADCSRB	= SFIOR				;redefine register name
;
  .equ	MUXMASK	= (EXP2(MUX4+1)-1)	;Mask for MUX bits
;
.equ	ADMUX_L	= ( (0<<REFS1) | \
					(0<<REFS0) | \
					(0<<ADLAR) | \
					(0<<MUX3) | \
					(0<<MUX2) | \
					(0<<MUX1) | \
					(0<<MUX0) )
;
; ADC Control Options A: Enable ADC in free-running mode,
;	prescaler configured based upon clock
.set	ADCSRA_L= ( (1<<ADEN) | \
					(1<<ADSC) | \
					(0<<ADATE) | \
					(1<<ADIF) | \
					(0<<ADIE) ) + ADCK
;
; ADC Control Options B: Free-running trigger source
.equ	ADCSRB_L= ( (0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )
;
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#elif defined(__ATtiny85__)
  .equ	ADC_Ref			= 256				;256 = 2.56V
  .equ	ADC_Refm= ( (1<<REFS2) | \
  					(1<<REFS1) | \
					(1<<REFS0))		;reference mask
  .equ	ADC_Int	= ( (1<<REFS2) | \
  					(1<<REFS1) | \
					(0<<REFS0))		;Internal reference mask
  .equ	ADC_Ext	= ( (0<<REFS2) | \
  					(0<<REFS1) | \
					(1<<REFS0))		;External reference mask

  .equ	MUXMASK	= (EXP2(MUX3+1)-1)	;Mask for MUX bits
;
; ADC Control Options A: Enable ADC in free-running mode,
;	prescaler configured based upon clock
  .set	ADCSRA_L= ( (1<<ADEN) | \
					(1<<ADSC) | \
					(0<<ADATE) | \
					(1<<ADIF) | \
					(0<<ADIE) ) + ADCK

; ADC Control Options B: Free-running trigger source
  .equ	ADCSRB_L= ( (0<<BIN) | \
					(0<<IPR) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )
;
  .equ	ADMUX_L	= ( (1<<REFS2) | \
  					(1<<REFS1) | \
					(0<<REFS0) | \
					(0<<ADLAR) | \
					(0<<MUX3) | \
					(0<<MUX2) | \
					(0<<MUX1) | \
					(0<<MUX0) )
;
; DIDR Options: no disable digital input
  .equ	DIDR0_L	= ( (0<<ADC3D) | \
					(0<<ADC2D) | \
					(0<<ADC1D) | \
					(0<<ADC0D) )

;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#elif defined(__ATtiny84__) || defined(__ATtiny84A__)
  .equ	ADC_Ref			= 110				;110 = 1.10V
  .equ	ADC_Refm= ( (1<<REFS1) | \
					(1<<REFS0))		;reference mask
  .equ	ADC_Int	= ( (1<<REFS1) | \
					(0<<REFS0))		;Internal reference mask
  .equ	ADC_Ext	= ( (0<<REFS1) | \
					(1<<REFS0))		;External reference mask

  .equ	MUXMASK	= (EXP2(MUX5+1)-1)	;Mask for MUX bits
;
; ADC Control Options A: Enable ADC in free-running mode,
;	prescaler configured based upon clock
  .set	ADCSRA_L= ( (1<<ADEN) | \
					(1<<ADSC) | \
					(0<<ADATE) | \
					(1<<ADIF) | \
					(0<<ADIE) ) + ADCK

; ADC Control Options B: Free-running trigger source
  .equ	ADCSRB_L= ( (0<<BIN) | \
					(0<<ADLAR) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )
;
  .equ	ADMUX_L	= ( (0<<REFS1) | \
  					(0<<REFS0) | \
					(0<<MUX5) | \
					(0<<MUX4) | \
					(0<<MUX3) | \
					(0<<MUX2) | \
					(0<<MUX1) | \
					(0<<MUX0) )
;
; DIDR Options: no disable digital input
  .equ	DIDR0_L	= ( (0<<ADC7D) | \
					(0<<ADC6D) | \
					(0<<ADC5D) | \
					(0<<ADC4D) | \
					(0<<ADC3D) | \
					(0<<ADC2D) | \
					(0<<ADC1D) | \
					(0<<ADC0D) )

;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#elif defined(__ATmega32U4__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__)  || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__)
  .equ	ADC_Ref			= 256		;256 = 2.56V
  .equ	ADC_Refm= ( (1<<REFS1) | \
					(1<<REFS0))		;reference mask
  .equ	ADC_Int	= ( (1<<REFS1) | \
					(1<<REFS0))		;Internal reference mask
  .equ	ADC_Ext	= ( (0<<REFS1) | \
					(0<<REFS0))		;External reference mask
;
  .equ	ADMUX_L	= ( (0<<REFS1) | \
					(0<<REFS0) | \
					(0<<ADLAR) | \
					(0<<MUX4) | \
					(0<<MUX3) | \
					(0<<MUX2) | \
					(0<<MUX1) | \
					(0<<MUX0) )
  ;
  ; ADC Control Options A: Enable ADC in free-running mode,
  ;	prescaler configured based upon clock
  .set	ADCSRA_L= ( (1<<ADEN) | \
					(1<<ADSC) | \
					(0<<ADATE) | \
					(1<<ADIF) | \
					(0<<ADIE) ) + ADCK
;
 #if defined(__ATmega32U4__)
  .equ	MUXMASK	= (EXP2(MUX5+1)-1)	;Mask for MUX bits

  ; ADC Control Options B: Free-running trigger source
  .equ	ADCSRB_L= ( (0<<ADHSM) | \
					(0<<ACME) | \
					(0<<MUX5) | \
					(0<<ADTS3) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )

 #elif defined(__ATmega644P__) || defined(__ATmega1284P__)
  .equ	MUXMASK	= (EXP2(MUX4+1)-1)	;Mask for MUX bits

  ; ADC Control Options B: Free-running trigger source
  .equ	ADCSRB_L= ( (0<<ACME) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )
 
 #elif defined(__AT90USB1286__)
  .equ	MUXMASK	= (EXP2(MUX4+1)-1)	;Mask for MUX bits
 
  ; ADC Control Options B: Free-running trigger source
  .equ	ADCSRB_L= ( (0<<ADHSM) | \
					(0<<ACME) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )

 #elif defined(__ATmega2560__)
  .equ	MUXMASK	= (EXP2(MUX5+1)-1)	;Mask for MUX bits

  ; ADC Control Options B: Free-running trigger source
  .equ	ADCSRB_L= ( (0<<MUX5) | \
					(0<<ACME) | \
					(0<<ADTS2) | \
					(0<<ADTS1) | \
					(0<<ADTS0) )
 #endif

; DIDR Options: no disable digital input
.equ	DIDR0_L	= ( (0<<ADC7D) | \
					(0<<ADC6D) | \
					(0<<ADC5D) | \
					(0<<ADC4D) | \
					(0<<ADC3D) | \
					(0<<ADC2D) | \
					(0<<ADC1D) | \
					(0<<ADC0D) )
;
#endif
