	.equ	VER = 101	;version 1.01 (100x so its an integer)
;************************************************************************
;						Meg163 atto basic
;************************************************************************
;Copyright 2003 Richard Cappels projects@cappels.org  http://projects.cappels.org
;You may use this for personal use only. Commercial License is available.
;You may republsih this file provided this notice is kept intact.
;Plese let me know of any bugs or (especially) improvements. Thank you.
;
;************************************************************************
;REVISION HISTORY:
; by K. Scott Vitale, Florida, uSA ksv_prj@gmx.com
;=============== Version 1.01 ============================================
;	2016-0531	Bugfix: corrected coding error in "setbitinX" and
;				  "clrbitinX" where the output was always written to
;				  PORTD.
;	2016-0530	Merged code into main AttoBASIC code tree.  Supports
;				  ATmega16 and ATmega32 as well as ATmega163.
;	2002-1115	Original code by Richard Cappels
;
;************************************************************************
; Atto Basic for ATmega163 - 16KB FLASH, 1KB RAM, 512B EEPROM
;************************************************************************
;.nolist
.include "Include/Defs_MCU-Options.inc"	; include MCU defs
;.list

;************************************************************************
#ifndef FCLK	; Set the system clock frequency
  #define FCLK	8000000		;set Fclk (preprocessor variable)
#endif

;	.equ	 clock		= 1000000
;	.equ	 clock		= 3579545
;	.equ     clock 		= 3690000		;clock frequency for STK-500
;	.equ	 clock		= 4000000
;	.equ     clock 		= 8000000		;clock frequency
;	.equ	 clock		= 11053200

;******* UART SETUP ******************************************************
	.equ     baudrate 	=  38400		;choose a baud rate
	.equ     baudconstant 	= (FCLK/(16*baudrate))-1

;************************************************************************
; Define some constants
	.equ	NULL		= 0			;define a NULL
	.equ	SPACE		= ' '		;a space character
	.equ	BELL		= 0x07		;Bell character
	.equ	BS			= 0x08		;Backspace character
	.equ	LF			= 0x0a		;Line Feed character
	.equ	COMMA		= 0x0c		;Comma character
	.equ	CR			= 0x0d		;Carriage Return character
	.equ	PROMPT		= 0x3E		;Prompt character
;
;****** DS Interface delay **********************************************
	.equ	settletime	= $50	;Time for sampling, setting of data lines
	.equ	dataline= 4	;D4	Data line
	.equ	attline	= 3	;D3	Attention line

;/////////MEMORY ALLOCATION////////

;The highest memory location is designated RAMEND in the chip's definition include file.
;The use of ram, starting from RAMEND and going down is as follows:

;	uC RETURN STACK		$45F to $400
;	UNASSIGNED RAM		$3FF to $300	use this
;	DATA STACK		$2FF to $3F4 change to $3FF to $3F4
;	PROGRAM MEMORY		$3F3 to $200
;	LINE BUFFER		$0FF to $0C2
;	VARIABLE STORAGE	$0C1 to $09B
;	UNASSIGNED RAM		$09A to $060


;ALLOCATE BUFFERS


	.equ	stackbot 	= $08			;Point to check for stack underflow

	.equ	dstacktop	= $FE;**		;First byte (top) of data stack
	.equ	dstackbot 	= $F3			;Last byte in data stack ;on 2313 this was stackbot
	.equ	dstackpage	= $03			;Entire dstack must be on one page

	.equ	lbufftop	= $FF			;First byte (top) of line buffer
	.equ	lbuffbot 	= $C2			;Last byte in terminal input buffer
	.equ	lbuffpage	= $00

	.equ	vartop		= $C1			;First byte in variable space
	.equ	varbot		= $9B			;Last byte in varibale space
	.equ	varpage		= $00


	.equ	pmemtop		= $F3;*			;*First byte (top) of program memory
	.equ	pmemtopage	= $03			;Hight byte of top of program memory
	.equ	pmembot		= $00			;Last byte in program memory
	.equ	pmembotpage	= $02			;Hight byte of of address of bottom of memory
	.equ	pmempage	= $03			;For now, restrict to page 2
; *Note: pmemtop cannot be $FF or address comparison in storeline will fail.
;** Note dstack must be at the top of the same page of memory as pmemtopage
	.equ	eeprogstortop	= $FF	;First (top) byte in EEPROM
	.equ	eepromstrotopage= $01	;MSB of highest address in EEPROM

;/////////REGISTER ALLOCATION////////

	.def	romreg		= r0	;Used in LPM command

	.def	PClow		= r5	;Marks position in program counter for virtual machine
	.def	PChigh		= r6	;This pair points to next available space
	.def	bufferlimL	= r7	;Limit of buffer - used by parcer
	.def	bufferlimH	= r8	;Limit of buffer - used by parcer
	.def	currentcast	= r9	;Used in parsing text.
	.def	looptarget	= r10	;FOR-NEXT
	.def	loopvariable	= r11	;FOR-NEXT
	.def	loopreturnL	= r12	;FOR-NEXT
	.def	loopreturnH	= r13	;FOR-NEXT
	.def	gosubretL	= r14	;GOSUB return address Low
	.DEF	gosubretH	= r15	;GOSUB return address High
	.def	temp 		= r16	;General purpose variable
	.def	inchar 		= r17	;Char destined to go out the uart
	.def	outchar 	= r18	;Char coming in from the uart
	.def	inbytel 	= r19	;Lower byte for asci-hex conversion
	.def	inbyteh 	= r20	;Higher byte for asci-hex conversion
	.def	ecode		= r21	;Cast returned here and error code stored here
	.def	dstackpointer	= r22	;Data stack pointer.
	.def	U		= r23	;Units
	.def	T		= r24	;Tens
	.def	H		= r25	;Hundreds
	;note   XL		= r26	;Flags register
	;note	XH		= r26	;Flags register
;Notes on XL, flag register bit useage.
;	bit	0		=1 for variabls to leave pointers on stack, =0 for values on stack
;	bit 	1		=1 to skip next line during RUN execution
;	bit	2		=1 to switch = to be part of FOR-NEXT loop
;	bit	3		=1 to activate Next as part of a loop
;	bit	4		=1 to tell runcommand to jam in loop return address next time
;	bit	5		=1 to tell runcommand to capture loop return address
;	bit	6		=1 to tell runcommand to capture GOSUB return address
;	bit	7		=1 to tell runcommand to jamb destination address

;Notes on XH, flag register bit useage.
;	bit	0		=1 to tell runcommand to jamp GOSUB return address
;	bit	1		=1 to tell runcommand to halt
;	bit	2		=1 to indicate that gosub is active
;	bit	3		=1 to indicate line no higher than linebuff encountered
;	bit	4		=1 to indicate that first line number was thrown away
;	bit	5		=1 to indicated nonnumeric char found in line
;	bit	6		=1 to indicate that suspend had been requested
	.cseg
;/////////VECTORS////////////////
	.org     $00
	jmp     init		;reset handle
	jmp	source1		;The rest of these vectors point to dummy handlers
	jmp	source2
	jmp	source3
	jmp	source4
	jmp	source5
	jmp	source6
	jmp	source7
	jmp	source8
	jmp	source9
	jmp	source10
	jmp	source11
	jmp	source12
	jmp	source13
	jmp	source14
	jmp	source15
	jmp	source16
	jmp	source17

;////////NO INTERRUPTS USED//////

init:
	ldi	temp,$19
	out	PORTD,temp	;Set PORTD bit 0,4 and 5 weak pullup
	ldi	temp,$00	;Port B all inputs
	out 	DDRB,temp
	ldi	temp,$00
	out	DDRD,temp	;Port D all inputs (except UART)
	ldi	temp,high(ramend)
	out	sph,temp
	ldi     temp,low(ramend)
	out     spl,temp     	;Set the stack pointer

#if defined(__ATtmega163__)
	ldi     temp,baudconstant
	out     ubrr,temp     	;Load UART bard rate
#elif defined(__ATmega16__) || defined(__ATmega32__)
	ldi     temp,high(((FCLK/16)/baudrate)-1)
	out 	UBRRH,temp     	;Load UART baud rate
	ldi     temp,low(((FCLK/16)/baudrate)-1)
	out		UBRRL,temp     	;Load UART baud rate
#endif

	sbi		ucsrb,txen	;enable UART xmit
	sbi		ucsrb,rxen	;Enable UART receiver

	call	newprogram	;Initialize program memory

	sbic	PIND,0		;If PIND,0 is grounded then execute from EEPROM
	rjmp	readytorun
	rcall	loadcommand
	rcall   runcommand
readytorun:

	call	newprogramgreet	;Initialize program memory and print greeting

main:
	rcall	crlf
	ldi	dstackpointer,dstacktop	;Initialize data stack pointer
	rcall	getterminalline	;Gen line from terminal (60 chars + $0D)
	rcall	Interpretelinebuffer
	rjmp	main

fetchrombyteinc:
	inc	ZL
fetchrombyte:	;Read byte from rom, advance pointer.
		;Enter with ZH, ZL pointing to rom list.
	lpm			;Load byte from program memory into r0
	adiw	ZH:ZL,1		;Increment Z registers
    	tst	romreg		;Check if we've reached the end of the list of commands
    	breq	endoflist	;If so,return
	ret

endoflist:
	ldi	ecode,$00
	rjmp	error



getexecword:	;Find the address of the instruction that matches the three characters in H,T,and U.
		;Execute associated instruction then return.
	ldi	ZH,high(2*commandlist)	;Load high part of byte address into ZH
	ldi	ZL,low(2*commandlist)	;Load low part of byte address into ZL
tryanother:
	rcall	fetchrombyte		;See if the string matches contents of H and T
	cp	romreg,H
	brne	nomatchH
	rcall	fetchrombyte
	cp	romreg,T
	brne	nomatchT
	rcall	fetchrombyte
	cp	romreg,U
	brne	nomatchU
					;OK, A match has been found, get the address of the routine.
	adiw	ZH:ZL,1			;Increment Z registers -skip unused 4th instruction name byte
	lpm				;Get address low into r0 (romreg)
	mov	temp,romreg		;Stash low byte in temp
	adiw	ZH:ZL,1			;Increment Z registers to point to high byte
	lpm				;Get address high into r0 (romreg)
	mov	ZH,romreg
	mov	ZL,temp
     	icall				;Do an indirect jump
     	ret				;Then return when function is complete

nomatchH:
 	adiw	ZH:ZL,1			;Increment Z registers -skip T
nomatchT:
	adiw	ZH:ZL,1			;Increment Z registers -skip U
nomatchU:
	adiw	ZH:ZL,3			;Increment Z registers -skip address (3 bytes)
 	rjmp	tryanother


formword:	;Put up to three consecutive non-delimiters in registers H,T,and U, right-justified
		;Enter with YH, YL pointing to first char in buffer to be parsed
		;Output: non-delimiters in registers T,H,and U, right-justified, count of
		;chars in outchar. If three chars are collected, this routine advances
		;YL until the cast changes. Leaves with YH,HYL pointing to next char to be parsed.
		;Used also: temp and inbyteh.

	ldi	H,$F0	 	;U is always loaded, no need to ininitalize
	ldi	T,$F0		;Pre-load the others

	ldi	outchar,$01
	ld	temp,Y		;Get char from linebuffer into U


	rcall	Qcast		;See what type of char it is.
	cpi	ecode,$01	;If the char is a delimiter (type 1), skip
	brne	nottype1cast	; **** advancelbpointer only once, just after Qcast
	rcall	advanclbpointer
	rjmp	formword
nottype1cast:
	mov	U,temp 		;Store char and advance pointer
	rcall	advanclbpointer
	cpi	temp,CR	;If its a carriage return, stop and return.
	breq	exitfromword0D
	mov	currentcast,ecode;Save cast type for reference

Getanotherhcar:
	ld	temp,Y		;Get char from linebuffer
	rcall	Qcast
	cpi	temp,CR	;If carriage return, then return
	breq	exitfromword0D
	cp	currentcast,ecode ;If not same as cast of first char, then return.
	brne	exitfromword
	mov	H,T
	mov	T,U
	mov	U,temp  	;Store char and advance pointer
	rcall	advanclbpointer
	inc	outchar
	cpi	outchar,$03	;If third character, exit
	brne	Getanotherhcar	;Otherwise, get another
exitfromword:
				;The code below is to make sure ponter is not
				;left pointing to the end of a word that is longer
				;than three chars long. It is not
				;entered when the pointer is pointing to the
				;carriage return at the end of the buffer.
	ld	temp,Y		;Get char from linebuffer
	rcall	Qcast
	cp	currentcast,ecode
	breq	notclearedgroup
exitfromword0D:
	ret


notclearedgroup:
	rcall	advanclbpointer
	rjmp	exitfromword


advanclbpointer:
		;Decrements YL, Jumps to error routine if fetch is requester after
		;pointer is moved past the end of the buffer. Enter with buffer limits
		;in bufferlimH,bufferlimL
	cp	YH,bufferlimH
	brne	nohitend
	cp	YL,bufferlimL
	brne	nohitend
	ldi	ecode,$04
	rjmp	error
	ret
nohitend:
	sbiw	YH:YL,1
	ret


Interpretelinebuffer:		;Interpret line
	ldi	YH,lbuffpage		;Initialize line buffer pointer to lbufftop
	ldi	YL,lbufftop
	ldi	temp,lbuffbot-1	;Set buffer limits to those of line buffer.
	mov	bufferlimL,temp
	ldi	temp,lbuffpage
	mov	bufferlimH,temp
	ld	temp,Y		;Get first char from linebuffer
	rcall	Qcast
	cpi	ecode,$02
	brne	dotheline	;First char is not a numeral so interpret line
	rjmp	storeline	;If it's a number, then it's a line number -store it.
dotheline:
	rcall	crlf
	ori	XL,$01		;Flag variable ponters to be left on stack
	rjmp	interpretline


interpretlinev:
	andi	XL,$FE		;Flag to leave value of variables on stack
interpretline:	;Interpret line. Enter with YH, YL pointing to first char to be interpreted
	in	inbytel,spl
	cpi	inbytel,stackbot
	brsh	stacknotexceeded
;	brpl	stacknotexceeded
	ldi	ecode,$08
	rjmp	error
stacknotexceeded:

	rcall	formword
	cpi	U,CR
	breq	endofline

	mov	temp,currentcast ; Check to see if its a number
	cpi	temp,$02
	brne	dontmakenumber
	andi	H,$0F		;Its a number so make binary and push on stack
	andi	T,$0F
	andi	U,$0F
	rcall	decimaltobinary	;change from 3 BCD bytes to one binary vaue
	rcall	pushU
	rjmp	interpretline	;Keep going until CR is found
dontmakenumber:

	andi 	H,$5F		;Upper-case H,T,U
	andi	T,$5F
	andi	U,$5F

	cpi	outchar,$01	;If its a type 3 (letter) and 1 char long its a variable.
	brne	notavar		;(If outchar = 1 and ecode = 3 its a variable)
	mov	temp,currentcast
	cpi	temp,$03
	brne 	notavar
	rcall	processvariable
	rjmp	interpretline

notavar:
	rcall	getexecword	;Check for CR after coming back from routine
	cpi	U,CR
	breq	endofline
	ret

endofline:
	ori	XL,$01		;Flag variable ponters to be left on stack next line
	ret


processvariable:	;Make variable in A into a pointer and
			;if XL bit 0 is 1, put pointer on stack (assignment to variable), or
			;if XL bit 0 is 0 put the value of variable on stack.

	subi	U,$41		;Make into an offset
	cpi	U,(vartop-varbot);Test for range
	brmi	variableok
	ldi	ecode,$05
	rjmp	error
variableok:
	subi	U,-varbot	;Make into a pointer
	sbrs	XL,$00
	brne	notcodezero
	rcall	pushU		;Put pointer on stack (ecode = $00)
	ret
notcodezero:
	ldi	ZH,varpage
	mov	ZL,U		;Put contents of variable on stack
	ld	U,Z
	rcall	pushU
	ret


pushU:		;Push contents of U on datastack

	ldi	ZH,dstackpage
	mov	ZL,dstackpointer
	st	Z,U
	dec	dstackpointer
	cpi	dstackpointer,dstackbot-1 ;If no stack undrflow, return.
;	brpl	nopusherror
	brsh	nopusherror
	ldi	ecode,$06
	rjmp	error
nopusherror:
	ret


popU:		;Pop contents of U from datastack

	inc		dstackpointer	;
	cpi		dstackpointer,LOW(dstacktop+1)
	brmi	nopoperror
	ldi 	ecode,$07
	rjmp	error
nopoperror:
	ldi	ZH,dstackpage
	mov	ZL,dstackpointer
	ld	U,Z
	ret


;///////////////INSERT LINE ROUTINES//////////////////////////////



showhtu:
 ldi	outchar,$20
	   rcall emitchar
 ldi	outchar,$20
	   rcall emitchar
	    ldi	outchar,$20
	   rcall emitchar
 ldi	outchar,$20
	   rcall emitchar
 mov inbytel,H	;**** only for test
	   rcall sendbyte
	   ldi	outchar,$20
	   rcall emitchar
	    mov inbytel,T	;**** only for test
	   rcall sendbyte
	   ldi	outchar,$20
	   rcall emitchar
	    mov inbytel,U	;**** only for test
	   rcall sendbyte
	   ldi	outchar,$20
	   rcall emitchar
	   ret





breadcrumb:			; **** test only
  ldi	outchar,$20
	   rcall emitchar
 ldi	outchar,$20
	   rcall emitchar
 mov inbytel,ZL		;**** only for test
	   rcall sendbyte
	   ldi	outchar,$20
	   rcall emitchar

	   ret

linenotoolong:
	ldi	ecode,$0B	;Error: Line number has too many characters.
	rjmp	error



Countlinehcars:		;Count total chars and number of non-numeric, non-delimiter chars
				;If not empty, there will be more than zero non-numeric
				;and non-delimiter characters in the line before the
				;carriage return.Retruns count in temp. The number of numerals
				;in the line number is contained in U.

	ldi	YH,lbuffpage		;Initialize line buffer pointer to lbufftop
	ldi	YL,lbufftop
	clr	gosubretH
	clr	gosubretL
	clr	U
	andi	XH,$DF			;Clear nonnumeric char found flag
morchars:
	ld	temp,Y
	dec	YL
	inc	gosubretL
	cpi	temp,CR
	breq	donecounting
	rcall	Qcast
	cpi	ecode,$02
	breq	anothernum
	ori	XH,$20
	cpi	ecode,$01
	breq	morchars
	inc	gosubretH
	rjmp	morchars
donecounting:

	mov	temp,gosubretH  	;Return with non-numeric, non-delimer count in temp
	ret				;Return with char total count in gusubretL

anothernum:
	sbrs	XH,5
	inc	U
	rjmp	morchars


buffendhit:
	ldi	ecode,$04
	rjmp	error

memoryfull:
	ldi	ecode,$02
	rjmp	error



makeHTUdecimal:			;Make decimal digits in H,T,and U into a binary number in U.
	andi	H,$0F		;Its a number so make binary and push on stack
	andi	T,$0F
	andi	U,$0F
	rcall	decimaltobinary
	ret

storelbufatZ:
	ldi	YH,lbuffpage	;Initialize line buffer pointer to first char
	ldi	YL,lbufftop
anotherlinchar:
	cpi	ZL,pmembot
	breq	memoryfull
	ld	temp,Y		;Put contents of line buffer into temp
	st	Z,temp		;Put temp into the program memory
	sbiw	ZH:ZL,1
	sbiw	YH:YL,1
	cpi	temp,CR
	brne	anotherlinchar
	mov	PClow,ZL
	mov	PChigh,ZH
	ret


storeline:	;Store line in appropriate place.

	rcall	Countlinehcars	;Reject line if line number contains more than three numerals.
	cpi 	U,4
	brsh	linenotoolong

	mov	temp,PChigh
	cpi	temp,pmemtopage
	brne	pcnotattop
	mov	temp,PClow
	cpi	temp,pmemtop
	brne	pcnotattop
	rjmp	storelineatend
pcnotattop:


	ldi	YH,lbuffpage	;Initialize line buffer pointer to first char
	ldi	YL,lbufftop

				;Find out what line number in line buffer is and put in looptarget
	clr	U
	clr	T
	clr	H

fetchanothernum1:
	cp	YL,bufferlimL	;Error if past end of buffer
	brmi	buffendhit
	ld	temp,Y
	rcall	qcast		;Find out what kind of char this is
	cpi	ecode,$02	;Type $02 - a numeral
	brne	notanumeral1
	mov	H,T		;Ita a numeral so shift it in
	mov	T,U
	mov	U,temp
	sbiw	YH:YL,1
	rjmp	fetchanothernum1
notanumeral1:			;The pointer is not poiting to a numeral, so test the number.
	rcall	makeHTUdecimal
	mov	looptarget,U	;Put number of line in linebuffer into looptarget

		;//GOT LINE BUFFER LINE NUMBER///

		;////SCAN PROGRAM MEMORY////
scanpmem:		;Scan from lowest to top of program memory to find the place to
			;insert the input line.
	mov	ZH,PChigh	;Get current program memory location from PC high and PC low
	mov	ZL,PClow
	andi	XH,$F7		;Clear higherthan flag
	andi	XH,$EF		;Clear first line no thrown out flag
newnumber:
	clr	H		;Clear digits buffer
	clr	T
	clr	U
	clr	loopvariable	;Clear count of number of digits
nextchar:
	adiw	ZH:ZL,$01
	cpi	ZH,pmemtopage
	brne	notattop
	cpi	ZL,pmemtop+1
	breq 	justifynumber	;If Z exceeds top of buffer, its the end of the line number
notattop:
	ld	temp,Z
   	rcall	Qcast		;Find out what kind of character Z is pointing to
	cpi	ecode,$04	;If its a carriage return, then its the end of the line number
	breq 	justifynumber
	cpi	ecode,$02
	brne	newnumber	;If its not a numeral, then dump numeral registers and start over.
	inc	loopvariable	;Increment count of numerals shifted in


	mov	U,T		;Shift current char in
	mov	T,H
	mov	H,temp
	rjmp	nextchar	;Get next char

justifynumber:			;Right justify the number in HTU
	mov	temp,loopvariable
	cpi	temp,3		;There are three registers: H,T,and U.
	breq	nowjustified
	inc	loopvariable
	mov	U,T
	mov	T,H
	ldi	H,0
	rjmp	justifynumber
nowjustified:

	sbrc	XH,4		;If this is the first line number, skip it (only responding to last CR in pmem).
	rjmp	notfirstlineno
	ori	XH,$10
	rjmp	newnumber
notfirstlineno:


	rcall	makeHTUdecimal	;Put value of HTU into U.

	cp	looptarget,U	;DIVERT IF EQUAL
	breq	deleteandgo	;If current line no is equal to target, then delete and/or replace the line
	brsh	nothigher	;If current line no is higher than target, set flag XH,3
	ori	XH,$08
	mov	loopreturnH,ZH	;and save address of CR following higher line number
	mov	loopreturnL,ZL
nothigher:


	cpi	ZH,pmemtopage	;DIVERT IF BUFFER SCANNED AND NO HIGHER
	brne	notattop2	;Test for Z being past end of memory
	cpi	ZL,pmemtop+1
	brne	notattop2	;If not past end of memory, test to see if line higher than in line buffer,
	sbrs	XH,3		;and if no higher number found, store line at end of buffer
	rjmp	storelineatend 	;If at end and no higher found, store at end.
notattop2:





			;DIVERT IF HIGHER PRVIOUSLY FOUND AND (HIGHER OR END)

	sbrs	XH,3		;Was a higher line number found?
	rjmp	noisnertlowerline

	cp	U,looptarget	;Is current line number lower than target?
	brsh	notlowerline
	rjmp	insertinmiddle
notlowerline:


	cpi	ZH,pmemtopage	;Test for Z being past end of memory
	brne	notattop3
	cpi	ZL,pmemtop+1
	brne	notattop3 	;If not past end of memory, test to see if line higher than in line buffer,
	rjmp	insertinmiddle
notattop3:



noisnertlowerline:		;Don't insert a line in the midst of pmem -keep scanning

;

	;(DEFAULT LOOP BACK)


	cpi	ZH,pmemtopage
	brne	newnumber
	cpi	ZL,pmemtop+1
	brne	newnumber
	ret



storelineatend:	;Store line in next available locatoin in program memory from first char to $0D
		;Enter with YH, YL pointing to first char of line


	rcall	Countlinehcars		;If its a zero length line, don't store it.
	cpi	temp,$00
	brne	writelineatend
	ret
writelineatend:
	ldi	YH,lbuffpage	;Initialize line buffer pointer to first char
	ldi	YL,lbufftop


	mov	ZH,PChigh	;Get current program memory location from PC high and PC low
	mov	ZL,PClow
anotherlinchar2:		;Check to see if ponter is past end of program memory
	cpi	ZH,pmembotpage
	brne	pmemnotfull
	cpi	ZL,pmembot
	brne	pmemnotfull
	rjmp	memoryfull2
pmemnotfull:
	ld	temp,Y		;Put contents of line buffer into temp
	st	Z,temp		;Put temp into the program memory
	sbiw	ZH:ZL,$01
	sbiw	YH:YL,$01
	cpi	temp,CR
	brne	anotherlinchar2
	mov	PClow,ZL
	mov	PChigh,ZH
	ret

memoryfull2:
	ldi	temp,$02
	mov	ecode,temp   	; **** ecode is now a high register
	rjmp	error


;itsanemptylineRELAY: rjmp itsanemptyline 	;can this be displosed of? ****

deleteandgo:	;Delete a line in prorgam memory. Enter with ZL pointing to
		;first char in line to be deleated (first char of line number).
		;Moves everything from the first char of the line following the
		;line to be deleted to the  end of the program memory to
		;overwrite the deleted line. ZH must be set to the correct page

	sbiw	ZH:ZL,1
	mov	YH,ZH		;Copy Z into Y
	mov	YL,ZL

				;Scan Y down the pgrogam memory until either end ofmemory
continuescan:			;or CR is encountered. If end of memory, claim error.

	cpi	YH,pmembotpage
	brne	Nothitpmembot
	cpi	YL,pmembot
	brne	Nothitpmembot
	rjmp	buffendhit
Nothitpmembot:

	ld	temp,Y
	sbiw	YH:YL,1
	cpi	temp,CR
	brne	continuescan

	mov	temp,ZL		;The difference between ZL and YL is the size of the line being
	sub	temp,YL		;deleted. Adjust PClow accodingly.

	add	PClow,temp
	brcc	noinc
	inc	PChigh
noinc:


keepoverwriting:
	cpi	YH,pmembotpage
	brne	skip9
	cpi	YL,pmembot	;Copy memory(Y) to memory (Z) until end of memory is reached.
	breq	finishedoverwrite
skip9:
	ld	temp,Y
	st	Z,temp
	sbiw	YH:YL,1
	sbiw	ZH:ZL,1
	rjmp	keepoverwriting
finishedoverwrite:

	rcall	Countlinehcars
	cpi	temp,$00
	breq	dontwriteline

	rjmp	storeline
;	rjmp	scanpmem		;**** changed to storeline from scanpmem - seems to have fixed reluctance to try and re-write line after deleting it.
dontwriteline:
	ret





insertinmiddle:	;MAKE ROOM FOR LINE IN MIDST OF TEXT AND INSERT


	rcall	Countlinehcars
	cpi	temp,$00		;If its a zero length line, don't store it.
	breq	itsanemptyline
;	breq	itsanemptylineRELAY



	mov	temp,PChigh	 ;But first, check to see if there is enough memory
	cpi	temp,pmembotpage ;PC in same page as bottom of program memory?
	brne	cantbefull

	mov	temp,pclow
	subi	temp,pmembot	;Temp has remaining memory bytes (in last page of memory)
	cp	temp,gosubretL	;Compare bytes remaining with line size
	brlo	memoryfull2
cantbefull:

				;Make room to insert line

	mov	ZH,PChigh	;Point Z to bottom of program [the source]
	mov	ZL,PClow

	mov	YH,ZH		;Set Y = Z - size of line buffer content) [the destination]
	mov	YL,ZL
	sub	YL,gosubretL
	sbci	YH,$00

	mov	temp,YL		;Adjust PC (pointer to end of program)
;	subi	temp,2
	mov	PClow,temp	;**** clean this up by simplifying
	mov	temp,YH
;	sbci	temp,0
	mov	Pchigh,temp


movemore:
	cp	ZH,loopreturnH
	brne	notfinishedmoving
	cp	ZL,loopreturnL
	brne	notfinishedmoving
	rjmp	movingfinished
notfinishedmoving:


	ld	temp,Z
	st	Y,temp
	adiw	YH:YL,1
	adiw	ZH:ZL,1
	rjmp	movemore

movingfinished:			;The text has been moved to make room for line buffer contents


		;Insert contents of line buffer into program memory

	ldi	YH,lbuffpage	;Initialize line buffer pointer to first char
	ldi	YL,lbufftop
	mov	ZH,loopreturnH	;Get address of CR after higher line
	mov	ZL,loopreturnL
	sbiw	ZH:ZL,$01		;Move pointer to first char of line to be moved


fillmore:
	ld	temp,Y
	st	Z,temp
	cpi	temp,CR
	breq	finishedfilling
	sbiw	ZH:ZL,1
	sbiw	YH:YL,1
	rjmp	fillmore
finishedfilling:


itsanemptyline:
	   ret


;/////////////////END OF INSERT LINE ROUTINES///////////////

Qcast:		;Determine whether character is a letter, numeral, delimiter, or other (operator).
		;Input: temp. Output: ecode. Uses: inchar
		;Value in ecode = cast
		;0 = Operator (not one of the other casts
		;1 = Delimiter -space ($20) or comman ($2C)
		;2 = Numeral 0..9 ($30 through $39)
		;3 = Letter -A..Z uppercase ($41 throught $5A)
		;4 = Carriage return
		;Tests are made after anding the input byte with $5F

	clr	ecode		;Default is type 0
	mov	inchar,temp	;Copy to inchar for modification

	cpi	inchar,CR	;Is it a carriage return? -check this before uppercasing
	brne	not0D
	rjmp	makecast4
not0D:

	andi	inchar,$5F	;Make upper-case


	cpi	inchar,00	;Is it a space?
	brne	notspacecode
	rjmp	makecast1
notspacecode:
	cpi	inchar,$0C	;Is it a comma?
	brne	notaspacecode
makecast1:
	ldi	ecode,$01
	ret
notaspacecode:
	cpi	inchar,$10
	brpl	notunder10
	ret
notunder10:
	cpi	inchar,$5B
	brmi	notover5B
	ret
notover5B:
	cpi	inchar,$40
	brmi	notover40
	ldi	ecode,$03
	ret
notover40:
	cpi	inchar,$1A
	brpl	notunder1A
	ldi	ecode,$02
notunder1A:
	ret
makecast4:
	ldi	ecode,$04
	ret


deletekey:			;Backup up cursor -destructive backspace
	cpi 	YL,lbufftop
	brne	notexceededbuftop
	ldi	outchar,BELL	;If not CR at last position, ring the bell
	rcall	emitchar
	rjmp	anothertermchar
notexceededbuftop:
	mov	outchar,inchar
	rcall	emitchar0d
	inc 	YL
	ldi	inchar,CR
	st	Y,inchar	;Put in line buffer
	rjmp	anothertermchar




getterminalline: 	;Get characters from terminal into linebuffer. Stop accepting chars except
			;0D when end of buffer is reached.
	ldi	outchar,PROMPT		;load prompt character
	rcall	emitchar
	ldi	YH,lbuffpage		;Initialize line buffer pointer to lbufftop
	ldi	YL,lbufftop
anothertermchar:
	rcall	recvchar	;Get char from terminal
	cpi	inchar,BS	;Is it backspace/delet character?
	breq	deletekey
	st	Y,inchar	;Put in line buffer
	cpi	inchar,CR
	breq	CRreceived	;If char was CR,then return
	cpi	YL,lbuffbot	;If buffer is at last byte, don't store char but beep.
	brne	notlbuffend
	ldi	outchar,BELL	;If not CR at last position, ring the bell
	rcall	emitchar
	rjmp	anothertermchar
notlbuffend:
	dec	YL		;Not end of buffer and not CR, so emit and go another
echoandgo:
	mov	outchar,inchar
	rcall	emitchar0d
	rjmp	anothertermchar
CRreceived:			;Last char in line received
	ret

Error:
	rcall	crlf
	cpi	ecode,$00
	brne	notzero
	ldi	ZH,high(2*emessage0)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage0)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
notzero:


	cpi	ecode,$01
	brne	not1
	ldi	ZH,high(2*emessage1)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage1)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not1:


	cpi	ecode,$02
	brne	not2
	ldi	ZH,high(2*emessage2)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage2)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not2:


	cpi	ecode,$03
	brne	not3
	ldi	ZH,high(2*emessage3)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage3)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not3:


	cpi	ecode,$04
	brne	not4
	ldi	ZH,high(2*emessage4)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage4)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not4:


	cpi	ecode,$05
	brne	not5
	ldi	ZH,high(2*emessage5)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage5)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not5:


	cpi	ecode,$06
	brne	not6
	ldi	ZH,high(2*emessage6)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage6)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not6:


	cpi	ecode,$07
	brne	not7
	ldi	ZH,high(2*emessage7)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage7)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not7:


	cpi	ecode,$08
	brne	not8
	ldi	ZH,high(2*emessage8)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage8)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not8:


	cpi	ecode,$09
	brne	not9
	ldi	ZH,high(2*emessage9)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessage9)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
not9:


	cpi	ecode,$0A
	brne	notA
	ldi	ZH,high(2*emessageA)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessageA)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
notA:


	cpi	ecode,$0B
	brne	notB
	ldi	ZH,high(2*emessageB)	;Load high part of byte address into ZH
	ldi	ZL,low(2*emessageB)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rjmp	errorend
notB:

errorend:
	ldi     temp,low(ramend) 		;low(ramend)
	ldi		temp,high(ramend)
	out		sph,temp
	ldi     temp,low(ramend)
	out     spl,temp
	ldi		dstackpointer,dstacktop	;Initialize data stack pointer
	ldi		XL,$00			;Clear interpreter mode flags
	ldi		XH,$00
	rcall	crlf
	rjmp	main

sendbyte:      ;Send byte contained in inbytel to terminal
	rcall 	byte_to_asciihex
    mov     outchar,inbyteh
    rcall 	emitchar
    mov     outchar,inbytel
    rcall  	emitchar
	ret

emitchar0:			;emit char in r0
	mov	outchar,temp
emitchar:
	sbis	ucsra,udre	;wait until the register is cleared
	rjmp	emitchar
	rcall	Checkkeybd 	;Xsee if control-c or control-s is pressed

	sbrs	XH,6		;is XH,6 is clear, don't wait for key
	rjmp	waitdone
waitforkey:
	sbis	ucsra,rxc
	rjmp 	waitforkey	;wait for a key before continuing
	andi	XH,$BF		;clear the flag
waitdone:


	out	udr,outchar	;send the byte
	ret			;go back


crlf:   ldi	outchar,CR	;Send carriage return and line feed to terminal
	rcall	emitchar
	ldi	outchar,LF
	rcall 	emitchar
	ret


emitchar0D:			;Send outchar to terminal. Add line feed to carriage return.

	cpi	outchar,CR	;Its a carraiage return, send a linefeed also
	brne	notareturn
	rcall	crlf
	ret
notareturn:
	rjmp	emitchar


recvchar:     	;Receive a byte from the terminal
	sbis	ucsra,rxc		;Wait for byte to be received
	rjmp 	recvchar
	in		inchar,udr	;Read byte
	ret


sendromstring:			;call with location of string in Z
	lpm			;Load byte from program memory into r0
	tst		r0		;Check if we've reached the end of the message
	breq	finishsendstering;If so, return
	mov		outchar,r0
	rcall	emitchar
	adiw	ZH:ZL,1		;Increment Z registers
	rjmp	sendromstring
finishsendstering:
	ret


sendlromline:			;Send a string terminated in cariage return and line feed
				;Call with location of start of string in Z
	rcall	sendromstring
	rcall	crlf
	ret

;/////////////START DATA FORMAT CONVERSION ROTINES//////////////////

bitpositiontoormask:	;Convert bit position (0..7) to OR mask value
			;Enter with bit position on top of dstack, exit with
			;mask on top of stack

	rcall	popU		;Get bit position
	ldi	temp,$07
	cp	temp,U
	brmi	toomanybits
	ldi	temp,$01	;Put 0000 0001 pattern into temp for shifting
rotatesomemore:
	cpi	U,$00
	breq	finishedshifting
	lsl	temp
	dec	U
	rjmp	rotatesomemore
finishedshifting:
	mov	U,temp
	rcall	pushU
	ret

toomanybits:
	ldi	ecode,$0A
	rcall	error



sendHTUasdecimal:	;Send decimal number in H,T,U registers (hundreds,tens,units)
			;As numerals.
	cpi	H,0
	breq	dontsendh
	subi	H,-48
	mov	outchar,H
	rcall	emitchar
	cpi	t,0
	brne	dontsendh
	ldi	outchar,$30	;If U=0 then don't emit this zero
	rcall	emitchar
dontsendh:
	cpi	t,0
	breq	dontsendt
	subi	T,-48
	mov	outchar,T
	rcall	emitchar
dontsendt:
	subi	U,-48
	mov	outchar,U
	rcall	emitchar
	ret


binarytodecimal:	;Enter with 8 bit value in U, Exits with numerals in H,T,U
			;with Hundreds, Tens, and Units.
	clr	H
	clr	T
anotherh:
	subi 	U,100	;Find out how many hundreds in U
	brcs	hdone
	inc	H
	rjmp	anotherh
hdone:
	subi	U,-100	;Subtracted one too many -add back.
anothert:
	subi	U,10
	brcs	tdone
	inc	T
	rjmp	anotherT
tdone:
	subi	U,-10
	ret


byte_to_asciihex:     ;convert byte in inbytel to ascii in inbyteh,nbytel
	mov	inbyteh,inbytel
	lsr	inbyteh 	;convert the high nybble to ascii byte
	lsr	inbyteh
	lsr	inbyteh
	lsr	inbyteh
	subi	inbyteh,$D0	;add $30
	cpi	inbyteh,$3A
	brlo	PC+2 		;If less than 9 skip next instruction
	subi	inbyteh,$F9	;add 8 to ASCII (if data greater than 9)
     ; byte in inbyteh represents upper nybble that was in inbytel at start

	andi	inbytel,0b00001111;convert the lower nybble to ascii byte
	subi	inbytel,$D0	;add $30
	cpi	inbytel,$3A
	brlo	PC+2		;If less than 9 skip next instruction
	subi	inbytel,$F9 	;add 8 to ASCII (if data greater than 9)
     ; byte in inbyteh represents upper nybble that was in inbytel at start
	ret


asciihex_to_byte:     	;convert ascii in inbyteh,inbytel to byte in inbytel
	sbrc	inbyteh,6	;convert high byte
	subi	inbyteh,$f7	;add     inbyte,temp     ;if bit 6 is set, add $09
	andi	inbyteh,$0F

	sbrc	inbytel,6	;convert low byte
	subi	inbytel,$f7	;add     inbyte,temp     ;if bit 6 is set, add $09
	andi	inbytel,$0F

	lsl	inbyteh		;combine them
	lsl	inbyteh
	lsl	inbyteh
	lsl	inbyteh
	or	inbytel,inbyteh
	ret

decimaltobinary:	;Convert decimal number in H,T,U registers (hundreds,tens,units) to
			;binary value in U register.
			;But first check to see if nummber exceeds 255 decimal.


	cpi		H,3
	brpl	exceeded255
	cpi		H,2
	brmi	numisok
	cpi		T,6
	brpl	exceeded255
	cpi		T,5
	brmi	numisok
	cpi		U,6
	brpl	exceeded255
numisok:
	tst		T		;Accumulate the number of tens
	breq	moreh
	ldi		temp,10
	add		U,temp		;Add  10
	dec		T
	brne	decimaltobinary
moreh:
	tst		H		;Accumulate the number of hundreds
	brne	notdoneconv
	ret			;When hundreds = zero, return
notdoneconv:
	ldi		temp,100
	clc
	add		U,temp		;Add 100
	dec		H
	brcc	moreh		;If carry is not set, continue
exceeded255:
	ldi		ecode,$01	; **** ecode is now a high register
	rjmp	Error


;/////////////////END DATA FORMAT CONVERSION ROTINES/////////////////

TypeGreeting:				;Type greeting
	rcall	crlf			; **** consider combinng with sendromline if sendromline is not used elsewhere
	rcall	crlf
	ldi		ZH,high(2*hellomessage)	;Load high part of byte address into ZH
	ldi		ZL,low(2*hellomessage)	;Load low part of byte address into ZL
	rcall	sendlromline		;sent it.
	ret


;//////////BEGIN COMMAND TABLE //////////

;Note about command routines: Clear XL bit 0 to have variables leave their value
;on the stack. This flag is set again at the start of each line.

;Format for command list:
	;1. A .db line with three characters which are the first
	;three leters of the command plus a fourth to fill out the word.
	;Commands less than three bytes need to be righ-justfied, with
	;blank leading bytes having the value $50. Compare values are
	;anded with $5F.
	;2. A .dw line that is the label for the routine to be executed.

commandlist:
	.db	"LIS "
	.dw	listcommand
	.db	"SIZ "		;SIZE	bytes or ram remaining
	.dw	sizecommand

	.db	"NEW "		;NEW	clear progam space, start over
	.dw	newprogramgreet

	.db	$50,$50,$0B,$20	; +
	.dw	addcommand

	.db	$50,$50,$0D,$20	; -
	.dw	subtractcommand

	.db	"PRIN"		;PRINT
	.dw	printcommand

	.db	"PRX "		;PRX -Print in hexidecimal
	.dw	printhexcommand

	.db	"PRB "		;PRB -Print in binary
	.dw	prbcommand

	.db	"PWM8"		;PWM8  send value to 8 bit pwm channel, set DDRB,3 to 1
	.dw	pwm8command

	.db	"PWE "		;PWE  send value to 10 bit pwm channel, set DDRB,3 to 1
	.dw	pwecommand

	.db	"PWO "		;PWO   turn off pulse-width modulation. Do not change DDRB
	.dw	pwocommand

	.db	"EMIT"		;EMIT -send byte on stack as ascii to terminal.
	.dw	emitcommand

	.db	$50,$1A,$1D,$20	; :=
	.dw	setequalscommand

	.db	$50,$50,$1D,$20	;  =	eual sign. Evaluate expression
	.dw	evaluatecommand

	.db	"FOR "
	.dw	forcommand

	.db	"NEXT"
	.dw	nextcommand

	.db	$50,$1C,$1E,$20	; <>
	.dw	notequalcommand

	.db	$50,$50,$1E,$20	; >
	.dw	greaterthancommand

	.db	$50,$50,$1C,$20	; <
	.dw	lessthancommand

	.db	$50,$49,$46,$20	; IF
	.dw	ifcommand

	.db	$50,$54,$4F,$20	; TO
	.dw	tocommand

	.db	$50,$50,$04,$20	; $ -Hex command - interpret following as hexidecimal.
	.dw	hexcommand

	.db	"RUN "
	.dw	runcommand

	.db	"END "
	.dw	endcommand

	.db	"AND "
	.dw	andcommand

	.db	"LSR "
	.dw	lsrcommand

	.db	"LSL "
	.dw	lslcommand

	.db	$50,$4F,$52,$20	;OR -Logical OR
	.dw	orcommand

	.db	"XOR "
	.dw	eorcommand

	.db	"DUMP"
	.dw	dumppmembuffer

	.db	"GOTO"
	.dw	gotocommand

	.db	"GOSU"		;GOSUB
	.dw	gosubcommand

	.db	"RETU"		;RETURN
	.dw	returncommand

	.db	"THEN"		;THEN
	.dw	thencommand

	.db	"KEY "		;WAIT -pause execution until char received from terminal. Halt if control-C received
	.dw	keycommand

	.db	"ACO "		;ACO -Put analog comparitor output on the stack
	.dw	acocommand

	.db	"SAVE"		;SAVE	-Save contents of variable space and program memory to EEPROM
	.dw	savecommand

	.db	"LOAD"		;LOAD	-Load variable space and program memory from EEPROM
	.dw	loadcommand

	.db	"PEEK"		;PEEK -return data pointed to by parameter
	.dw	peekcommand

	.db	"POKE"		;POKE - "POKE data, address"
	.dw	pokecommand

	.db	"ODA "		;ODA -Out Data Direction A
	.dw	outdAcommand

	.db	"ODB "		;ODB -Out Data Direction A
	.dw	outdBcommand

	.db	"ODC "		;ODC -Out Data Direction C
	.dw	outdCcommand

	.db	"ODD "		;ODD -Out Data Direction D
	.dw	outdDcommand

	.db	"OPA "		;OPA -Out Port A
	.dw	outpAcommand

	.db	"OPB "		;OPB -Out Port B
	.dw	outpBcommand

	.db	"OPC "		;OPC -Out Port C
	.dw	outpCcommand

	.db	"OPD "		;OPD -Out Port D
	.dw	outpDcommand

	.db	"INA "		;INA -Input data from port A
	.dw	inAcommand

	.db	"INB "		;INB -Input data from port B
	.dw	inBcommand

	.db	"INC "		;INC -Input data from port C
	.dw	inCcommand

	.db	"IND "		;IND -Input data from port D
	.dw	inDcommand

	.db	"SBA "		;SBA -Set bit in A register
	.dw	setbitinA

	.db	"CBA "		;CBA -Clear bit in A register
	.dw	clearbitinA

	.db	"SBB "		;SBD -Set bit in B register
	.dw	setbitinB

	.db	"CBB "		;CBD -Clear bit in B register
	.dw	clearbitinB

	.db	"SBC "		;SBD -Set bit in C register
	.dw	setbitinC

	.db	"CBC "		;CBD -Clear bit in C register
	.dw	clearbitinC

	.db	"SBD "		;SBD -Set bit in D register
	.dw	setbitinD

	.db	"CBD "		;CBD -Clear bit in D register
	.dw	clearbitinD

	.db	"SDA "		;SDA -Set direction bit in A
	.dw	setdirectionA

	.db	"CDA "		;CDA -Clear direction bit in A
	.dw	cleardirectionA

	.db	"SDB "		;SDA -Set direction bit in B
	.dw	setdirectionB

	.db	"CDB "		;CDA -Clear direction bit in B
	.dw	cleardirectionB

	.db	"SDC "		;SDA -Set direction bit in C
	.dw	setdirectionC

	.db	"CDC "		;CDA -Clear direction bit in C
	.dw	cleardirectionC

	.db	"SDD "		;SDA -Set direction bit in D
	.dw	setdirectionD

	.db	"CDD "		;CDA -Clear direction bit in D
	.dw	cleardirectionD

	.db	"IBA "		;IBA -Test bit in A
	.dw	testbitinA

	.db	"IBB "		;IBA -Test bit in B
	.dw	testbitinB

	.db	"IBC "		;IBA -Test bit in C
	.dw	testbitinC

	.db	"IBD "		;IBA -Test bit in D
	.dw	testbitinD

	.db	"ADC "		;ADC -provide value from a to do converter
	.dw	ADCcommand

	.db	"DELA"		;DELAY -DELAY approx. 50 ms per count at 4 MHz
	.dw	DELAYcommand

	.db	"DSRE"		;DSRECIVE - recieve by to TOS from DS Interface
	.dw	dsrcommand

	.db	"DSCO"		;DSCOMMAND - Send TOS via DS Interface as command
	.dw	dsccommand

	.db	"DSDE"		;DSDATA - Send TOS via DS Interface as data
	.dw	dsdcommand

	.db	$00,$00


listcommand:	;lists program memory buffer to screen..
	rcall	crlf
	ldi	ZH,pmemtopage	;Initialize PC position inidcator
	ldi 	ZL,pmemtop
anotherchar:
	cp	ZH,PChigh
	brne	skip1
	cp	ZL,PClow	;Was cpi pmembot
	breq	endofpmem
skip1:
	ld	outchar,Z	;Put contents of line buffer into outchar;
	rcall	emitchar0D
	sbiw	ZH:ZL,1
	rjmp	anotherchar
endofpmem:
	rcall	crlf
	;(listcommand flows into sizecommand)

sizecommand:	;Find remaining program memory space. Uses U.
	rcall	crlf

	ldi	ZH,high(2*freeword)	;Load high part of byte address into ZH
	ldi	ZL,low(2*freeword)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.


	mov	U,PClow
	mov	H,PChigh
	subi	U,pmembot
	sbci	H,pmembotpage
	rcall	pushU
	mov	U,H
	rcall	binarytodecimal
	rcall	sendHTUasdecimal


	ldi	ZH,high(2*pageword)	;Load high part of byte address into ZH
	ldi	ZL,low(2*pageword)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
	rcall	popU
	rcall	binarytodecimal
	rcall	sendHTUasdecimal
	rcall	crlf
	ret




printcommand:			;Print TOS after call, to screen
	andi	XL,$FE		;Flag to leave value of variables on stack
	rcall	interpretline
	rcall	popU
	rcall	binarytodecimal
	rcall	sendHTUasdecimal
	rcall 	crlf
	ret


printhexcommand:			;Print TOS in hex, after call, to screen
	rcall	interpretlinev
	ldi	outchar,$24
	rcall	emitchar
	rcall	popU
	mov	inbytel,U
	rcall	sendbyte
	rcall 	crlf
	ret


setequalscommand:		;Get value at TOS after call to variable on stack when
				;equals command is called.
	andi	XL,$FE	;cbr	XL,0		;Flag to leave value of variables on stack
	rcall	interpretline	;**** can be made into single call to interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	ldi	ZH,varpage
	mov	ZL,U
	st	Z,temp
	ret



evaluatecommand:;Euqal sign	;If XL,2 = 0, perform subtract command.
				;If XL,2 = 1, set up FOR-NEXT loop
	sbrs	XL,$02
	rjmp	subtractcommand	;Its functionally identical to subtract.
				;Set up FOR-NEXT loop
	andi	XL,$FE		;Flag variable values to be left on stack next line
	rcall	interpretline	;Get the rest of the loop parameters on the stack

	rcall	popU		;Pop loop target
	mov	looptarget,U	;Store loop target

	rcall	popU		;Pop initial counter value
	mov	T,U		;Put in T for now

	rcall	popU		;Pop pointer to counter

	mov	loopvariable,U	;Copy variable pointer into loopvariable register

	ldi	ZH,varpage
	mov	ZL,U		;Copy  variable pointer into ZL as index
	st	Z,t		;Store initial counter value into counter

	ori	XL,$20		;Set flag asking runcommand to capture loop address


	ori	XL,$08		;Set flag activating NEXT
	andi	XL,$FA
	ret


nextcommand:			;The distant end of the FOR-NEXT loop
				;Tests the loop variable to see its equal target.
				;If its equal to target, go to next line
				;If not equal target, increment variable and
				;Signal RUNCOMMAND to jump to loop return

	ldi	ZH,varpage	;This is fetching the the loop variable from variable space
	mov	ZL,loopvariable
	ld	temp,Z		;Get loop variable value into temp
	cp	temp,looptarget
	breq	finishedfnloop	;If they are equal, the loop's done
	inc	temp
	st	Z,temp		;Increment loop variable and put it back into the register
	ori	XL,$10		;Set XL,4 to flag run command to jump to return address next time
finishedfnloop:
;	andi	XL,$F7		;Make sure that next flags is cleared
	ret


notequalcommand:		; <> command -Returns opposite result of subtractcommand
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	cp	U,temp
	breq	makeff
	clr	U
	rcall	pushU
	ret
makeff:	ser	U
	rcall	pushU
	ret


acocommand:			;Put inverse of analog comparitor output on stack (0 for high)
	rcall	interpretlinev
	ldi	U,0
	sbis	ACSR,ACO
	ldi	U,1
	rcall	pushU
	ret

peekcommand:			;Replace value on TOS (Top Of Stack) with contents of memory
				;at that location.
	rcall	interpretlinev
	rcall	popU
	mov	ZL,U
	ldi	ZH,0		;Peek is limited to page zero
	ld	U,Z
	rcall 	pushU
	ret
;
pokecommand:			;Store data at one down from TOS at location pointed to by TOS

	rcall	interpretlinev

	rcall	popU
	mov	temp,U		;Pop uses Z register, so initialize after all the popping is done.
	rcall	popU
	ldi	ZH,0		;Poke is limited to page zero
	mov	ZL,temp
	st	Z,U
	ret



addcommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	add	U,temp
	rcall	pushU
	ret


subtractcommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	sub	U,temp
	rcall	pushU
	ret


andcommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	and	U,temp
	rcall	pushU
	ret


lsrcommand:
	rcall	interpretlinev
	rcall	popU
	lsr	U
	rcall	pushU
	ret


lslcommand:
	rcall	interpretlinev
	rcall	popU
	lsl	U
	rcall	pushU
	ret


orcommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	or	U,temp
	rcall	pushU
	ret


eorcommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	eor	U,temp
	rcall	pushU
	ret


makeU1:
	ldi	U,1
	rcall	pushU
	ret

lessthancommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	cp	U,temp
	rjmp	evaldiff

pwm8command:
	rcall	interpretlinev
	ldi	temp,$81
	out	TCCR1A,temp 		; 8 bit PWM
	out	TCCR1B,temp 		; prescaler = 1
	sbi	DDRD,5
	rcall	popU
	ldi	temp,$00
	out 	OCR1AH,temp
	out	OCR1AL,U
	ret



pwecommand:
	rcall	interpretlinev
	ldi	temp,$83		;set timer1 PWM mode
	out	TCCR1A,temp 		; 10 bit PWM not reverse (Fck/510)
	ldi	temp,$01
	out	TCCR1B,temp 		; prescaler = 1
	sbi	DDRB,5
	rcall	popU
	mov	temp,U
	rcall	popU
	out 	OCR1AH,U
	out	OCR1AL,temp
	ret




pwocommand:				;Pulse Width Modulation OFF
	ldi	temp,$00		;set timer1 PWM mode
	out	TCCR1A,temp
	ret

greaterthancommand:
	rcall	interpretlinev
	rcall	popU
	mov	temp,U
	rcall	popU
	cp	temp,U
evaldiff:
	ldi	U,0
	brpl	makeU1		;**** suspicious brpl command
	breq	makeU1
	rcall	pushU
	ret


ifcommand:	;Set falg to skip next line if number on TOS returned from rest
		;of line is not zero
	rcall	interpretlinev
	rcall	popU	;Pop value off data stack
	tst	U		;
	breq	itszero
	ori	XL,$02
itszero:
	ret



thencommand:			;Return to previous calling routine using a
	in	temp,spl	;skanky trick with stack pointer. Get pointer
	subi	temp,$FE	;Add 02 to value (undo one return address)
	out	spl,temp	;Put value back in stack pointer
	ret			;Pop up to previous call. Don't try this at home.


forcommand:			;Initial statement in FOR-NEXT structure
				;Sets flag to switch "=" routine to set up the
				;FOR-NEXT loop
	ori	XL,$04		;Set bit 3
	ret


tocommand:			;To in FOR-NEXT structure. This is a dummy command
	rcall	interpretlinev
	ret


endcommand:			;Stop execution by setting flag
	ori	XH,$02		;Set flag telling runcommand to stop.
	ret




hexcommand:			;Nasty and dangerous - get two following chars as hex put on stack.
	ld	inbyteh,Y
	rcall	advanclbpointer
	ld	inbytel,Y
	rcall	asciihex_to_byte
	mov	U,inbytel
	rcall	pushU
	rcall	advanclbpointer
	rcall	interpretlinev
	ret



getlineno:			;Search buffer for line number, set up for runcommand to jump to it.
				;Can use Y because will will be modified by this routine
				;before runcommand uses it again anyway.
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	mov	inbyteh,U	;Put target line number into inbytel
	ldi	ZH,pmemtopage	;Could preserve ZH?
	ldi 	ZL,pmemtop	;Initialize PC position inidcator
	rjmp	pufistln	;Jump in the middle so first line no. can be found

searchCR:
	cp	ZH,PChigh
	brne	skip2
	cp	PClow,ZL	;Error if past end of buffer
	brsh	gotonotfound
skip2:
	ld	temp,Z
	cpi	temp,$0D
	breq	checkforlinumb	;Take the branch if CR is found
	sbiw	ZH:ZL,1
	rjmp	searchCR	;Else continue to look for CR

checkforlinumb:			;Found a CR now make line number
	sbiw	ZH:ZL,1		;Move pointer past CR
pufistln:
	clr	U
	clr	T
	clr	H

fetchanothernum:
	cp	ZH,PChigh
	brne	skip3
	cp	PClow,ZL	;Error if past end of buffer
;	brpl	gotonotfound
	brsh	gotonotfound
skip3:
	ld	temp,Z
	rcall	qcast		;Find out what kind of char this is
	cpi	ecode,$02	;$02 is a numeral
	brne	notanumeral
	mov	H,T		;Ita a numeral so shift it in
	mov	T,U
	mov	U,temp
	sbiw	ZH:ZL,1
	rjmp	fetchanothernum
notanumeral:			;The pointer is not poiting to a numeral, so test the number.

	andi	H,$0F		;Its a number so make binary and push on stack
	andi	T,$0F
	andi	U,$0F
	rcall	decimaltobinary	;Change from 3 BCD bytes to one binary vaue
	cp	inbyteh,U
	brne	searchCR	;No the current line number is in binary form
	ret


gotonotfound:
	ldi	ecode,$09	;Claim error - goto line not found
	rjmp	error


gotocommand:
	rcall	getlineno
	mov	YH,ZH
	mov	YL,ZL
	ret


gosubcommand:			;Do a gosub

	rcall	getlineno	;Get destination address in ZH and ZL
	ori	XL,$C0		;Set flags to capture return address and jam destination address
	ret


returncommand:			;Return from gosub
	sbrs	XH,2
	rjmp	returnnotactive
	ori	XH,$01		;Set flag calling for jaming of return address
	ret
returnnotactive:
	ori	XH,$02		;Set flag calling for program halt
	ret



Savecommand:		;Save contents of memory from dstacktop to pmembot to EEPROM eepromtop down.
			;Values of PChigh and PClow to be stored in top two bytes of dstack.
	ldi	dstackpointer,dstacktop	;Initialize data stack pointer
	mov	U,PClow
	rcall	pushU
	mov	U,PChigh
	rcall	pushU			;dstack in the same page as the top of pmem, so saving pemem from
	ldi	YH,pmemtopage		;the top down will also save dstack
	ldi	YL,dstacktop		;dstack top is at the top of the page so use this of LSByte
	ldi	ZL,eeprogstortop
	ldi	ZH,eepromstrotopage

storeanotherchar:
	ld	temp,Y			;Put contents of program memory into temp
wrat1:	sbic	eecr,eewe		;Wait for EEPROM write to not be busy
	rjmp	wrat1
	out	eearH,ZH
	out	eearL,ZL
	out	eedr,temp		;Set up the  write data
	sbi	eecr,eemwe
	sbi	eecr,eewe		;Trigger the write
	cpi	YH,pmembotpage
	brne	skip7
	cpi	YL,pmembot
	breq	finishedsaving
skip7:
	sbiw	ZH:ZL,1
	sbiw	YH:YL,1
	rjmp	storeanotherchar
finishedsaving:
	ret



Loadcommand:		;Save contents of memoru from dstacktop to pmembot to EEPROM eepromtop down.
			;Vales of PChight and PCLow to be read from top two bytes of dstack.
	ldi	YH,pmemtopage
	ldi	YL,dstacktop
	ldi	ZL,eeprogstortop
	ldi	ZH,eepromstrotopage
loadanotherchar:
wratz1:	sbic	eecr,eewe		;Wait for EEPROM write to not be busy
	rjmp	wratz1
	out	eearH,ZH
	out	eearl,ZL		;move to EEPROM address register
	sbi	eecr,eere		;Trigger the read
	in	temp,eedr		;Get the data into temp
	st	Y,temp
	cpi	YH,pmembotpage
	brne	skip8
	cpi	YL,pmembot
	breq	finishedloading
skip8:
	sbiw	ZH:ZL,1
	sbiw	YH:YL,1
	rjmp	loadanotherchar
finishedloading:

	ldi	dstackpointer,dstacktop-2 ;Initialize data stack pointer
	rcall	popU
	mov	PChigh,U
	rcall	popU
	mov	PClow,U
	ret


emitcommand:
	rcall	interpretlinev
	rcall	popU
	mov	outchar,U
	rcall	emitchar
	ret


keycommand:		;Receive a byte from the terminal
	sbis	ucsra,rxc		;Wait for byte to be received
	rjmp 	keycommand
	in	U,udr	;Read byte
	cpi	U,$03	;Is it control-C?
	brne	notermhalt1
	ori	XH,$02		;set flag to halt program
notermhalt1:
	rcall	pushU
	ret




prbcommand:	;print in binary format
	rcall	interpretlinev
	rcall	popU
	ldi	temp,$08
stillsendingbinary:
	ldi	outchar,$30
	rol	U
	brcc	dontsendone
	ldi	outchar,$31
dontsendone:
     	rcall  	emitchar
     	dec 	temp
     	brne	stillsendingbinary
     	rcall	crlf
	ret




setbitinA:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PORTA
	or	temp,U
	out	PORTA,temp
	ret



clearbitinA:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,PORTA
	and	temp,U
	out	PORTA,temp
	ret


setbitinB:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PORTB
	or	temp,U
	out	PORTB,temp
	ret



clearbitinB:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,PORTB
	and	temp,U
	out	PORTB,temp
	ret


setbitinC:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PORTC
	or	temp,U
	out	PORTC,temp
	ret



clearbitinC:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,PORTC
	and	temp,U
	out	PORTC,temp
	ret




setbitinD:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PORTD
	or	temp,U
	out	PORTD,temp
	ret



clearbitinD:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,PORTD
	and	temp,U
	out	PORTD,temp
	ret


setdirectionA:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,DDRA
	or	temp,U
	out	DDRA,temp
	ret



cleardirectionA:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,DDRA
	and	temp,U
	out	DDRA,temp
	ret


setdirectionB:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,DDRB
	or	temp,U
	out	DDRB,temp
	ret



cleardirectionB:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,DDRB
	and	temp,U
	out	DDRB,temp
	ret

setdirectionC:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,DDRC
	or	temp,U
	out	DDRC,temp
	ret



cleardirectionC:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,DDRC
	and	temp,U
	out	DDRC,temp
	ret


setdirectionD:				;Set bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,DDRD
	or	temp,U
	out	DDRD,temp
	ret



cleardirectionD:				;Clear bit in position inidicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	com	U
	in	temp,DDRD
	and	temp,U
	out	DDRA,temp
	ret


testbitinA:				;Test bit in specified position indicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PINA
	and	temp,U
	ldi	U,0
	tst	temp
	brne	JA
	ldi	U,1
JA:	rcall	pushU
	ret


testbitinB:				;Test bit in specified position indicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PINB
	and	temp,U
	ldi	U,0
	tst	temp
	brne	JB
	ldi	U,1
JB:	rcall	pushU
	ret


testbitinC:				;Test bit in specified position indicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PINC
	and	temp,U
	ldi	U,0
	tst	temp
	brne	JC
	ldi	U,1
JC:	rcall	pushU
	ret


testbitinD:				;Test bit in specified position indicated at top of stack.
	rcall	interpretlinev
	rcall	bitpositiontoormask
	rcall	popU
	in	temp,PIND
	and	temp,U
	ldi	U,0
	tst	temp
	brne	JD
	ldi	U,1
JD:	rcall	pushU
	ret



outpAcommand:			;Output to PORTA
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	PORTA,U
	ret

outpBcommand:			;Output to PORTB
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	PORTB,U
	ret

outpCcommand:			;Output to PORTC
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	PORTC,U
	ret


outpDcommand:			;Output to PORTD
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	PORTD,U
	ret



outdAcommand:			;Output to DDRA
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	DDRA,U
	ret



outdBcommand:			;Output to DDRB
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	DDRB,U
	ret




outdCcommand:			;Output to DDRC
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	DDRC,U
	ret


outdDcommand:			;Output to DDRD
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	out	DDRD,U
	ret

inAcommand:
	in	U,PINA
	rcall	pushU
	ret

inBcommand:
	in	U,PINB
	andi	U,$FC
	rcall	pushU
	ret

inCcommand:
	in	U,PINC
	rcall	pushU
	ret

inDcommand:
	in	U,PIND
	andi	U,$FC
	rcall	pushU
	ret


ADCcommand:	;Perform 8 bit A to D conversion for channel specified and put result on stack
		;If no channel is specified, convert channel zero
	ldi	U,$00
	rcall	pushU		;Put default channel on stack
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	andi	U,$1F		;Mask off any extra bits
	ori	U,$E0		;Create ADMUX walue
	out	ADMUX,U		;Set up for left-justified 2.56 volt FS measurement
	ldi	temp,$D6
	out	ADCSR,temp	;Start conversion with 125 khz clock (assumes 8 MHz xtal)
waitforAtoD:
	sbic	ADCSR,6		;Wait for conversion to complete
	rjmp	waitforAtoD
	in	U,ADCH		;Get the 8 bit result
	rcall	pushU		;Push the data onto the stack
	ret


breakmsgtxt:
	.db	"Break "
	.db	$0A,$0D
	.db     $00,$00



;//////////RUN COMMAND/////////////////

runcommand:
	ldi	YH,pmemtopage		;Initialize PC position inidcator
	ldi 	YL,pmemtop
checkandrunline:
	ldi	dstackpointer,dstacktop	;Initialize data stack pointer
					;Parameters may be passed on the stack within a line
					;but not between lines.

	ori	XL,$01		;Flag variable ponters to be left on stack

	rcall	Checkkeybd	;See if Control-C key was received


	sbrs	XL,$05		;FOR-NEXT management. Capture F-N return adddress
	rjmp	nocaploopad
	mov	loopreturnL,YL	;Copy return address (in interpreter p. space) to registers
	mov	loopreturnH,YH
	andi	XL,$DF		;Turn off flag -address captured.
nocaploopad:

	sbrs	XL,$04		;FOR-NEXT management. If XL,4 is set, jam in new address
	rjmp	noaddressjam
	mov	YH,loopreturnH
	mov	YL,loopreturnL
	andi	XL,$EF		;Flip jaming flag off
noaddressjam:

	sbrs	XL,$06		;GOSUB-RETURN management: Capture GOSUB return address
	rjmp	nocapgosubret
	mov	gosubretL,YL
	mov	gosubretH,YH
	andi	XL,$BF		;Flip flag off
nocapgosubret:


	sbrs	XL,$07		;GOSUB-RETURN management:jamb destination address
	rjmp	nodestaddjam
	mov	YH,ZH
	mov	YL,ZL
	ori	XH,$04		;Set flag indicating that gosub is active
	andi	XL,$7F		;Flip flag off
nodestaddjam:


	sbrs	XH,$00		;GOSUB-RETURN management: Jamb return address
	rjmp	noretjam
	mov	YH,gosubretH
	mov	YL,gosubretL
	andi	XH,$FA		;Flip flags off (including gosub active flag)
noretjam:


	sbrs	XH,$01		;Halt flag -if set, halt
	rjmp	nostop
	rcall	breakmessage
	mov	YH,PChigh
	mov	YL,PClow
	andi	XH,$FD		;Clear the halt flag
nostop:
	cp	YH,PChigh
	brne	skip4
	cp	PClow,YL	;See if the end of the buffer has been reached
;	brpl	endofprogram	;**** changed to BRSH now works past about 127 bytes
	brsh	endofprogram
skip4:
	ld	temp,Y		;Get char from buffer into temp
	rcall	Qcast		;See what type of char it is.

	sbrs	XL,$01		;If XL bit 1 is clear, skip this line- move cursor
	rjmp	noskipline	;to next carriage return
	cpi	ecode,$04
	breq	noskipline
	sbiw	YH:YL,1
	rjmp	checkandrunline


noskipline:
	andi	XL,$FD		;Make sure that skip line flag is cleared for next time around
	; **** does clearing this flag now actuall do anyting?
;	Code below uses Qcast call several lines above.
	cpi	ecode,$02	;See if its a numeral
	brne	notonlinenumber
	sbiw	YH:YL,1		;If numeral, move pointer past it
	rjmp	checkandrunline
notonlinenumber:

	rcall	interpretline
	rjmp 	checkandrunline
endofprogram:
	ret



newprogramgreet:
	rcall	TypeGreeting
newprogram:		;Ready program space and interpreter for new program
	ldi	temp,pmemtopage
	mov	PChigh,temp		;Initialize PC position inidcator
	ldi 	temp,pmemtop
	mov	PClow,temp
	mov	ZH,PChigh		;Initialize program memory pointer to top of program memory
	mov	ZL,PClow
	ldi	dstackpointer,dstacktop	;Initialize data stack pointer

	ldi	XL,$00			;Clear interpreter mode flags
	ldi	XH,$00

	ldi	temp,$00
doanotherone:
	cpi	ZH,pmembotpage
	brne	skip6
	cpi	ZL,pmembot
	breq	endofpmem1
skip6:
	st	Z,temp			;fill memory with nulls ($00)
	sbiw	ZH:ZL,1
	rjmp	doanotherone
endofpmem1:
	ret



dumppmembuffer:	;dumps pmem buffer for debug only.

	ldi	ZH,pmemtopage		;Initialize PC position inidcator
	ldi 	ZL,pmemtop
anotherchar2:
	cpi	ZH,pmembotpage
	brne	skip5
	cpi	ZL,pmembot
	breq	endofpmem2
skip5:
	ld	inbytel,Z	;Put contents of line buffer into outchar;
	rcall	sendbyte
	ldi	outchar,$20
	rcall	emitchar
	ldi	outchar,$20
	rcall	emitchar
	sbiw	ZH:ZL,1
	rjmp	anotherchar2
endofpmem2:
	rcall	crlf		;dump XL, looptarget,loopvariable,loopreturnL
	rcall	crlf


	mov	inbytel,PChigh
	rcall	sendbyte

	ldi	outchar,$20
	rcall 	emitchar
	mov	inbytel,PClow
	rcall	sendbyte


	rcall	crlf
	rcall 	crlf
	ret


breakmessage:
	ldi	ZH,high(2*breakmsgtxt)	;Load high part of byte address into ZH
	ldi	ZL,low(2*breakmsgtxt)	;Load low part of byte address into ZL
	rcall	sendromstring		;sent it.
ret




	;Dummy interrupt handlers

	source1:
	source2:
	source3:
	source4:
	source5:
	source6:
	source7:
	source8:
	source9:
	source10:
	source11:
	source12:
	source13:
	source14:
	source15:
	source16:
	source17:
reti



DELAYcommand:		;wait for (tos) times 2^16 times around the loop.
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	clr	H
	clr	T
	clr	ecode
	clr	temp
decu:
	tst	U
	breq	delaydone
dech:
	dec	H
dect:
	dec	T
dececode:
	dec	ecode
dectemp:
decinbyteh:
	dec	inbyteh
	brne	decinbyteh
	dec	temp
	brne	dectemp
	brne	dececode
	brne	dect
	brne	dech
	dec	U
	rjmp	decu

delaydone:
	ret


dsdcommand:		;Send byte (top of stack) via DS port
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	mov	inchar,U
	clc
	rcall	SendDSByte
	ret


dsccommand:		;Send byte (top of stack) via DS port
	rcall	interpretlinev	;Get target line number to TOS (Top Of Stack)
	rcall	popU
	mov	inchar,U
	sec
	rcall	SendDSByte
	ret



dsrcommand:		;Receive a byte vai DS port
	rcall	ReceiveDSByte
	mov	U,inchar
	rcall	pushU
	ret

;/////////////////// DS INTERFACE BYTE SEND AND RECEIVE ROUTINES////////////////

;	Registers used:
;	inchar 	;byte to be sent
;	temp 	;scratch
;	inbytel,inbyteh	;counter used for timing delay
;	H ;counter for number of bits transfered


;//////////SEND A BYTE
; Send carry bit as first bit, then send inchar bit-by-bit. inchar destroyed.
SendDSByte:

	in	temp,DDRD	;assure that PORT D is set up to be compatible with DS Interface
	andi	temp,$E7
	out	DDRD,temp
	in	temp,PORTD	;set pullups
	ori	temp,$18
	out	PORTD,temp

	ldi	H,$09

Sendbyte1:

	sbis		PIND,attline		;Wait for Attention line to go high =--SEND A BIT
	rjmp		Sendbyte


	sbi		PORTD,dataline		;Put data on data line
	cbi		DDRD,dataline
	brcs		Recvbyte1
	cbi		PORTD,dataline
	sbi		DDRD,dataline
Recvbyte1:
	cbi		PORTD,attline		;Set Attention line low
	sbi		DDRD, attline

	rcall		shortdelay		;Wait a short time so other chip can see Attnetion line is low
	cbi		DDRD,attline		;Release Attention line for a peak
	sbi		PORTD,attline
	rcall		shortdelay		;Short delay to allow settling of lines
	sbic		PIND,attline		;If Attention line isn't low, go back and put it low again, else continue
	rjmp		Recvbyte1

	brcs		szero			;Invert data line
	sbi		PORTD,dataline
	cbi		DDRD,dataline
	rjmp		Recvbyte3
szero:
	cbi		PORTD,dataline
	sbi		DDRD,dataline
Recvbyte3:

	sbis		PIND,attline		;Wait for Attention line to go high
	rjmp		Recvbyte3

	cbi		DDRD,dataline		;Let data line float
	sbi		PORTD,dataline
					;-finished sending a bit

	rol		inchar			;Shift inchar through carry

	dec		H
	brne		Sendbyte1			;Continue until all bits sent
	ret


ReceiveDSByte: ;Get a byte into inchar, and "start bit" into carry bit.

	in	temp,DDRD	;assure that PORT D is set up to be compatible with DS Interface
	andi	temp,$E7
	out	DDRD,temp
	in	temp,PORTD
	ori	temp,$18
	out	PORTD,temp
	ldi	H,$09

W3:					;Get a bit from the input into the carry
	sbic	PIND,attline		;Wait for Attention line to go low
	rjmp	w3

	clc				;Latch dataline into carry bit
	sbis	PIND,dataline
	rjmp	NotaOne
	sec

	cbi	PORTD,attline		;ACK by pulling attention line low
	sbi	DDRD, attline

W1:					;Wait for data line to go low
	sbic	PIND,dataline
	rjmp	W1
	rjmp	RelesaseAttLineAndGo


NotaOne:

	cbi	PORTD,attline		;ACK by pulling attention line low
	sbi	DDRD, attline


W2:					;Wait for data line to go high
	sbis	PIND,dataline
	rjmp	W2

RelesaseAttLineAndGo:			;Release attention line
	cbi	DDRD, attline
 	sbi	PORTD,attline
					;DONE RECEIVING BIT
	rol	inchar		;Shift inchar through carry

	dec	H
	brne	W3			;Continue until all bits received
ret



shortdelay:
	ldi	inbytel,settletime
outerloop:
	ldi	inbyteh,settletime
innerloop:
	dec	inbyteh
	brne	innerloop
	dec	inbytel
	brne	outerloop
	ret


Checkkeybd:		;See if there is any reason to stop output or stop program execution
			;If control-s is found in keyboard buffer and if XH bit 6 is set, wait until
			;another key is pressed. This routine assures that XH,6 is cleared at exit.
			;If control-C then stop program execution before executing
			;the next line.

	sbis	ucsra,rxc		;if byte received, from termnial, get it
	rjmp	nokeydown
	in	temp,udr	;Read byte
	cpi	temp,$13	;Is it control-S?
	brne	noterstop
	ori	XH,$40		;Set the flag indicating that suspend was requested
	rjmp	notermhalt

noterstop:

	in	temp,udr	;Read byte
	cpi	temp,$03	;Is it control-C?
	brne	notermhalt
	ori	XH,$02		;set flag to halt program.

notermhalt:
nokeydown:
	ret

;** Strings ********************************************************
#if ( (FCLK/10000000) == 0 )
  #define CLOCKRATE	$20,(FCLK/1000000)+$30
#else
  #define CLOCKRATE	(FCLK/10000000)+$30,((FCLK/1000000)%10)+$30
#endif

hellomessage:
#if defined(__ATmega16__)
	.db	"AttoBASIC for ATmega16 @",CLOCKRATE,"MHz V:", \
		(VER/100)+$30,'.',\
		((VER%100)/10)+$30, \
		((VER%100)%10)+$30,CR,LF
#elif defined(__ATmega163__)
	.db	"AttoBASIC for ATmega163 @",CLOCKRATE,"MHz V:", \
		(VER/100)+$30,'.',\
		((VER%100)/10)+$30, \
		((VER%100)%10)+$30," ",CR,LF
#elif defined(__ATmega32__)
	.db	"AttoBASIC for ATmega32 @",CLOCKRATE,"MHz V:", \
		(VER/100)+$30,'.',\
		((VER%100)/10)+$30, \
		((VER%100)%10)+$30,CR,LF
#endif
	.db	"(C)2002 Dick Cappels projects@cappels.org ",CR,LF
	.db	"Mods (C)2016 Scott Vitale ksv_prj@gmx.com ",CR,LF
	.db "All rights reserved. Not for commercial use. ",CR,LF,NULL
	.dw	0

;	.db     "      ATTOBASIC version 163-021115C. Not for commercial use.",$0A,$0D
;	.db     "      Copyright 2002 by Richard Cappels all rights reserved.",$0A,$0D
;	.db     "      http://projects.cappels.org       projects@cappels.org".$0A,$0D,0

freeword:
	.db	"Free  pages:"
	.dw	0

pageword:
	.db	"  chars:"
	.dw	0

emessage0:
	.db	"Error 0: Unknown command or operator. "
	.dw	0

emessage1:
	.db	"Error 1: Encountered a value that is not between 0 and 255. "
	.dw	0

emessage2:
	.db	"Error 2: Program memory is full."
	.dw	0

emessage3:
	.db	"Error 3: Encountered a character that is not a recognized type. "
	.dw	0

emessage4:
	.db	"Error 4:(internal error -buffer limit exceeded) "
	.dw	0

emessage5:
	.db	"Error 5: Inappropriate variable name encountered. "
	.dw	0

emessage6:
	.db	"Error 6: Data stack push error -too many arguments on one line. "
	.dw	0

emessage7:
	.db	"Error 7: Data stack pop error.",$0A,$0D
	.db	"         Function tried to use a value that was not available."
	.dw	0

emessage8:
	.db	"Error 8: Machine stack error", $0A,$0D
	.db	"         Functions and/or calls nested too deeply or too complex"
	.dw	0

emessage9:
	.db	"Error 9: GOTO or GOSUB destination line was not found."
	.dw	0

emessageA:
	.db	"Error A: Specified bit position range of 0..7 exceeded"
	.dw	0

emessageB:
	.db	"Error B: Line number has more than three numerals "
	.dw	0

;end of program


