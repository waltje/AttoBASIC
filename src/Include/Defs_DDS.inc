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
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
.if DDS
  .equ	DDSOut	= PORTB2		;DDS Output pin
  .equ	DDSPort	= PORTB			;Port for DDS output pin
.endif
;==========================================================
#elif defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) 
.if (DDS && DATAREC)
  .equ	DDSOut	= PB0			;digital output pin
  .equ	DDSPort	= PORTB			;digital out port
.elif DDS
 .equ	DDSOut	= PD6			;DDS Output pin
 .equ	DDSPort	= PORTD			;Port for DDS output pin
.endif
;==========================================================
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) 

 .equ	DDSOut	= PB0			;DDS Output pin
 .equ	DDSPort	= PORTB			;Port for DDS output pin
;==========================================================
#elif defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) 
.if DDS
  .equ	DDSOut	= PORTD5		;DDS Output pin
  .equ	DDSPort	= PORTD			;Port for DDS output pin
.endif
;==========================================================
#elif defined(__ATmega2560__)
.if DDS
  .equ	DDSOut	= PORTE4		;DDS Output pin
  .equ	DDSPort	= PORTE			;Port for DDS output pin
.endif

#else
#error	"!! Invalid Target MCU !!"
#endif
;
;===============================================================================
; Use TIM1 for Tiny84/85, TIM2 for Mega88/168/328 and TIM4 for Mega32U4.  The 
;  sample rate for DDS is hard-coded to ( FCLK / 256 ), thus the maximum 
;  frequency attainable will be ( FCLK / 256 / 2 ).
;
	.equ	DDSRate		= f_clk/256			;DDS master sample rate based on FCLK
	.equ	DDSCnt		= ((f_clk/DDSRate/2)+5)	;# of TIMR ticks between DDS 
												; updates plus "fudge factor"
;
;cummulative DDS add/subtract values
.equ	_1Hz	= ((DDSCnt*EXP2(24)*1)/f_clk)
.equ	_10Hz	= ((DDSCnt*EXP2(24)*10)/f_clk)
.equ	_100Hz	= ((DDSCnt*EXP2(24)*100)/f_clk)
.equ	_1KHz	= ((DDSCnt*EXP2(24)*1000)/f_clk)
.equ	_10KHz	= ((DDSCnt*EXP2(24)*10000)/f_clk)
.equ	_100KHz	= ((DDSCnt*EXP2(24)*100000)/f_clk)

;==========================================================
#if defined(__ATtiny84__) || defined(__ATtiny84A__)
;use TIMER1 for DDS sample timer on Tiny84(A)
	.equ	DDSPre		= CK1_1		;DDS master clock Prescaler

	.equ	DDS_PRR		= PRR  
	.equ	DDS_PRTIM	= PRTIM1

	.equ	DDS_TCNTL	= TCNT1L
	.equ	DDS_TCNTH	= TCNT1H
	.equ 	DDS_OCRAL	= OCR1AL
	.equ 	DDS_OCRAH	= OCR1AH
	.equ	DDS_OCRBL	= OCR1BL
	.equ	DDS_OCRBH	= OCR1BH

	.equ	DDS_TIMSK	= TIMSK1
	;bit defs
	.equ	DDS_TOIE	= TOIE1
	.equ	DDS_OCIEA	= OCIE1A
	.equ	DDS_OCIEB	= OCIE1B
	.equ	DDS_ICIE	= ICIE1

	.equ	DDS_TIFR	= TIFR1
	;bit defs
	.equ	DDS_TOV		= TOV1
	.equ	DDS_OCFA	= OCF1A
	.equ	DDS_OCFB	= OCF1B
	.equ	DDS_ICF		= ICF1

	.equ	DDS_TCCRA	= TCCR1A
	;bit defs
	.equ	DDS_WGM0	= WGM10
	.equ	DDS_WGM1	= WGM11
	.equ	DDS_COMB0	= COM1B0
	.equ	DDS_COMB1	= COM1B1
	.equ	DDS_COMA0	= COM1A0
	.equ	DDS_COMA1	= COM1A1

	.equ	DDS_TCCRB	= TCCR1B
	;bit defs
	.equ	DDS_CS0		= CS10
	.equ	DDS_CS1		= CS11
	.equ	DDS_CS2		= CS12
	.equ	DDS_WGM2	= WGM12
	.equ	DDS_WGM3	= WGM13
	.equ	DDS_ICES	= ICES1
	.equ	DDS_ICNC	= ICNC1

	.equ	DDS_TCCRC	= TCCR1C
;
#elif defined(__ATtiny85__)
;use TIMER1 for DDS sample timer on Tiny85
	.equ	DDSPre		= CK1_1		;DDS master clock Prescaler

	.equ	DDS_PRR		= PRR
	.equ	DDS_PRTIM	= PRTIM1

	.equ	DDS_TCNT	= TCNT1
	.equ 	DDS_OCRA	= OCR1A
	.equ	DDS_OCRB	= OCR1B
	.equ 	DDS_OCRC	= OCR1C

	.equ	DDS_TIMSK	= TIMSK
	;bit defs
	.equ	DDS_TOIE	= TOIE1
	.equ	DDS_OCIEA	= OCIE1A
	.equ	DDS_OCIEB	= OCIE1B

	.equ	DDS_TIFR	= TIFR
	;bit defs
	.equ	DDS_OCFA	= OCF1A
	.equ	DDS_OCFB	= OCF1B
	.equ	DDS_TOV		= TOV1

	.equ	DDS_TCCR	= TCCR1
	;bit defs
	.equ	DDS_CTC		= CTC1
	.equ	DDS_PWMA	= PWM1A
	.equ	DDS_COMA0	= COM1A0
	.equ	DDS_COMA1	= COM1A1
	.equ	DDS_CS0		= CS10
	.equ	DDS_CS1		= CS11
	.equ	DDS_CS2		= CS12
	.equ	DDS_CS3		= CS13
;
;==========================================================
#elif defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__)

;use TIMER2 for DDS sample timer on Mega88/168/328 and USB1286
	.equ	DDSPre		= CK2_1		;DDS master clock Prescaler

;==========================================================
#if	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__)
	.equ	DDS_PRR		= PRR0
#else
	.equ	DDS_PRR		= PRR
#endif
	.equ	DDS_PRTIM	= PRTIM2

	.equ	DDS_TCNT	= TCNT2
	.equ 	DDS_OCRA	= OCR2A
	.equ	DDS_OCRB	= OCR2B 

	.equ	DDS_TIMSK	= TIMSK2
	;bit defs
	.equ	DDS_TOIE	= TOIE2
	.equ	DDS_TOIEA	= TOIE2A
	.equ	DDS_OCIEA	= OCIE2A
	.equ	DDS_OCIEB	= OCIE2B

	.equ	DDS_TIFR	= TIFR2
	;bit defs
	.equ	DDS_OCFA	= OCF2A
	.equ	DDS_OCFB	= OCF2B
	.equ	DDS_TOV		= TOV2

	.equ	DDS_TCCRA	= TCCR2A
	;bit defs
	.equ	DDS_WGM0	= WGM20
	.equ	DDS_WGM1	= WGM21
	.equ	DDS_COMB0	= COM2B0
	.equ	DDS_COMB1	= COM2B1
	.equ	DDS_COMA0	= COM2A0
	.equ	DDS_COMA1	= COM2A1

	.equ	DDS_TCCRB	= TCCR2B
	;bit defs
	.equ	DDS_CS0		= CS20
	.equ	DDS_CS1		= CS21
	.equ	DDS_CS2		= CS22
	.equ	DDS_WGM2	= WGM22
	.equ	DDS_FOCB	= FOC2B
	.equ	DDS_FOCA	= FOC2A
;==========================================================
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
;use TIMER2 for DDS sample timer on Mega16 and Mega32
	.equ	DDSPre		= CK2_1		;DDS master clock Prescaler

	.equ	DDS_TCNT	= TCNT2
	.equ 	DDS_OCRA	= OCR2

	.equ	DDS_TIMSK	= TIMSK
	;bit defs
	.equ	DDS_TOIE	= TOIE2
	.equ	DDS_OCIEA	= OCIE2

	.equ	DDS_TIFR	= TIFR
	;bit defs
	.equ	DDS_OCFA	= OCF2
	.equ	DDS_TOV		= TOV2

	.equ	DDS_TCCR	= TCCR2
	;bit defs
	.equ	DDS_CS0		= CS20
	.equ	DDS_CS1		= CS21
	.equ	DDS_CS2		= CS22
	.equ	DDS_WGM1	= WGM21
	.equ	DDS_COMA0	= COM20
	.equ	DDS_COMA1	= COM21
	.equ	DDS_WGM0	= WGM20
	.equ	DDS_FOCA	= FOC2

;==========================================================
#elif defined(__ATmega32U4__)
;use TIMER4 for DDS sample timer on Mega32U4
	.equ	DDSPre		= CK4_1		;DDS master clock Prescaler

	.equ	DDS_PRR		= PRR1  
	.equ	DDS_PRTIM	= PRTIM4

	.equ	DDS_TCNT	= TCNT4
	.equ 	DDS_OCRA	= OCR4A
	.equ	DDS_OCRB	= OCR4B
	.equ 	DDS_OCRC	= OCR4C
	.equ 	DDS_OCRD	= OCR4D

	.equ	DDS_TIMSK	= TIMSK4
	;bit defs
	.equ	DDS_TOIE	= TOIE4
	.equ	DDS_OCIEA	= OCIE4A
	.equ	DDS_OCIEB	= OCIE4B
	.equ	DDS_OCIED	= OCIE4D

	.equ	DDS_TIFR	= TIFR4
	;bit defs
	.equ	DDS_OCFA	= OCF4A
	.equ	DDS_OCFB	= OCF4B
	.equ	DDS_OCFC	= OCF4D
	.equ	DDS_TOV		= TOV4

	.equ	DDS_TCCRA	= TCCR4A
	;bit defs
	.equ	DDS_PWMB	= PWM4B
	.equ	DDS_PWMA	= PWM4A
	.equ	DDS_FOCB	= FOC4B
	.equ	DDS_FOCA	= FOC4A
	.equ	DDS_COMB0	= COM4B0
	.equ	DDS_COMB1	= COM4B1
	.equ	DDS_COMA0	= COM4A0
	.equ	DDS_COMA1	= COM4A1

	.equ	DDS_TCCRB	= TCCR4B
	;bit defs
	.equ	DDS_CS0		= CS40
	.equ	DDS_CS1		= CS41
	.equ	DDS_CS2		= CS42
	.equ	DDS_CS3		= CS43
	.equ	DDS_DTPS0	= DTPS40
	.equ	DDS_DTPS1	= DTPS41
	.equ	DDS_PSR		= PSR4
	.equ	DDS_PWMX	= PWM4X

	.equ	DDS_TCCRC	= TCCR4C

	.equ	DDS_TCCRD	= TCCR4D
	;bit defs
	.equ	DDS_WGM0	= WGM40
	.equ	DDS_WGM1	= WGM41
	.equ	DDS_FPF		= FPF4
	.equ	DDS_FPAC	= FPAC4
	.equ	DDS_FPES	= FPES4
	.equ	DDS_FPNC	= FPNC4
	.equ	DDS_FPEN	= FPEN4
	.equ	DDS_FPIE	= FPIE4

	.equ	DDS_TCCRE	= TCCR4E
;;
#endif
