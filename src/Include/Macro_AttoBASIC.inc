.macro INITDSTACK		;Initialize data stack pointer
	clr		DStkPtr
.endmacro
;
.macro INITGSTACK		;Initialize GOS-RET stack pointer
	clr		GoSubStkPtr
.endmacro
;
.macro INITLPTACK		;Initialize FOR-NEX stack pointer
	clr		LOOPStkPtr
.endmacro
;
.macro WAIT4ADCRDY		;wait for the ADC to be finished with a conversion
.if ADCSRA < 0x40
	in		R16,ADCSRA		;fetch status register
	sbrs	R16,ADIF		;Skip if bit in register set
	rjmp	PC-0x0002		;loop till ready
.else
	lds		R16,ADCSRA		;fetch status register
.	sbrs	R16,ADIF		;Skip if bit in register set
	rjmp	PC-0x0004		;loop till ready
.endif
.endmacro
;
#if	TEST
.macro	LEDON
	SETB	DDRD,PD6			;set TEENSY LED on
	SETB	PORTD,PD6
.endmacro
;
.macro	LEDOFF
	CLRB	PORTD,PD6			;set TEENSY LED on
	CLRB	DDRD,PD6
.endmacro
#endif

