	.equ	VER = 120	;version 1.20 (100x so its an integer)
;************************************************************************
;						2313 atto basic
;************************************************************************
;Original code Copyright 2002 Richard Cappels projects@cappels.org
;       http://projects.cappels.org
;You may use this for personal use only. Commercial License is available.
;You may republsih this file provided this notice is kept intact.
;Plese let me know of any bugs or (especially) improvements. Thank you.
;
;************************************************************************
;REVISION HISTORY:
; by K. Scott Vitale, Florida, uSA ksv_prj@gmx.com
;=============== Version 1.2 ============================================
;	2016-0503	Added DELay and RST commands.
;				Modified: changed 16-bit pointer to be even-register 
;				  aligned for use with the movw, sbiw and adiw 
;				  instructions.
;	2013-1010	Code compacting to save flash - total of 136 bytes 
;				  (or 6.5%) on AT90S2313, 132 bytes on ATtiny2313[A].
;
;=============== Version 1.1 ============================================
;	2013-0615	Minor code compacting to save flash.
;				Added "SStst" definition to enable/disable self-start
;				  feature.
;
;=============== Version 1.0 ============================================
;	2011-0528	Compacted code to allow UART Rx handler.
;				Added feature enabling support so that a user can 
;				 enable/disable certain command features if not 
;				 needed.
;				Fixed input line parsing routine; ignores linefeed
;				 characters ($0A) when they are seen.  This allows for
;				 uploading programs using a terminal emulation program.
;				Changed UART receive handler; MPU now enters sleep 
;				 mode until a character is received.  Note: any interrupt
;				 wakes from sleep but the 'recvchar' routine checks for
;				 data in RX buffer.  If no data is ready, it goes back
;				 to sleep.  This allows for other interrupt-driven 
;				 routines (such as DDS output) to function as well and keeps
;				 the MPU running in a low-power state when waiting for
;				 user input. 
;				Added support for ATtiny2313 and ATtiny2313A.  Simply select
;				 the appropriate AVR DEF file to include and conditional
;				 assembly takes care of the rest.
;
;************************************************************************
;ERROR CODES
; 0: Unknown command or operator.
; 1: Encountered a value that is not between 0 and 255.
; 2: Program memory is full.
; 3: Encountered a character that is not a recognized type.
; 4:(internal error -buffer limit exceeded)
; 5: Inappropriate variable name encountered (A to D only).
; 6: Push data stack error -too many arguments on one line.
; 7: Pop data stack error. Function tried to use a value that was not available.
; 8: Machine stack error Functions and/or calls nested too deeply or too complex
; 9: GOTO or GOSUB destination line was not found.
;************************************************************************
;.nolist
.include "Include/Defs_MCU-Options.inc"	; include MCU defs
;.list

;************************************************************************
; Code enabling feature.  Select "1" to enable code generation, 
;	"0" to disable.
	.equ	PWM		= 1		;"1" to enable PWM routines (+68 bytes)
	.equ	AComp	= 1		;"1" to enable Analog Comp routines (+16 bytes)
	.set	DELAY	= 1		;"1" to enable delay routines (+54 bytes)
	.set	RESET	= 1		;"1" to enable reset routine (+28 bytes)
	.set	SStst	= 1		;"1" to enable self-start test at RESET (+8 bytes)
;
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
;************************************************************************
#ifndef FCLK_PS	; Set the system clock prescaler (ATtiny2313[A]) only
  #define FCLK_PS  1		;System clock prescaler (1,2,4,8,16,32,64,128 or 256)
#endif

#ifndef FCLK	; Set the system clock frequency
  #define FCLK	(12000000/FCLK_PS)       //set Fclk (preprocessor variable)
#endif

;//////////UART SETUP//////////////
	.equ     baudrate 		= 38400		;choose a baud rate
	.equ     baudconstant 	= (FCLK/(16*baudrate))-1
	
;****************************************************************
; Watchdog Timer Constants
;****************************************************************
.equ	WDMAX	= 9					;maximum prescaler value
;
.equ WDT_OFF    = 0x00
.equ WDT_16MS   = (1<<WDE)
.equ WDT_32MS   = (1<<WDP0)
.equ WDT_64MS   = (1<<WDP1)
.equ WDT_125MS  = (1<<WDP1) | (1<<WDP0)
.equ WDT_250MS  = (1<<WDP2)
.equ WDT_500MS  = (1<<WDP2) | (1<<WDP0)
.equ WDT_1S     = (1<<WDP2) | (1<<WDP1)
.equ WDT_2S     = (1<<WDP2) | (1<<WDP1) | (1<<WDP0)
#if defined(__ATtiny2313__) || defined(__ATtiny2313A__)
.equ WDT_4S     = (1<<WDP3)
.equ WDT_8S     = (1<<WDP3) | (1<<WDP0)
#endif
;

;/////////MEMORY ALLOCATION////////

;The highest memory location is designated RAMEND in the chip's definition include file.
;The use of ram, starting from RAMEND and going down is as follows:

;	uC RETURN STACK		set with stacksize
;	DATA STACK		set with dstacksize
;	LINE BUFFER		set with lbuffsize
;	VARIABLE STORAGE	set with varsize 
;	PROGRAM MEMORY		set with pmemsize

	;ALLOCATE BUFFERS
	.equ	stacksize	= 28	;Bytes allocated to uC return stack
	.equ	dstacksize	=  4	;Bytes allocated to interpreter data stack
	.equ	lbuffsize	= 20	;Bytes allocated to line buffer
	.equ	varsize		=  4	;Bytes allocated to variables (A,B,C,D)
	.equ	pmemsize	= SRAM_SIZE-(stacksize+dstacksize+lbuffsize+varsize)	;Bytes allocated to program memory
	
	.equ	stackbot 	= ramend + 1 - stacksize;Last byte in uC stack
	.equ	dstacktop	= stackbot-1		;First byte (top) of data stack
	.equ	dstackbot 	= stackbot-dstacksize	;Last byte in data stack
	.equ	lbufftop	= dstackbot-1		;First byte (top) of line buffer
	.equ	lbuffbot 	= dstackbot-lbuffsize	;Last byte in terminal input buffer
	.equ	vartop		= lbuffbot-1		;First byte in variable space
	.equ	varbot		= lbuffbot-varsize	;Last byte in varibale space
	.equ	pmemtop		= varbot-1		;First byte (top) of program memory
	.equ	pmembot		= varbot-pmemsize	;Last byte in program memory

	.equ	eeprogstortop	= $7F			;First (top) byte in EEPROM

;/////////REGISTER ALLOCATION////////

	.def	romreg			= r0	;Used in LPM command

	.def	currentcast		= r5	;Used in parsing text.
	.def	PClow			= r6	;Marks position in program counter for virtual machine
	.def	PChigh			= r7	;This pair points to next available space
	.def	bufferlimL		= r8	;Limit of buffer - used by parcer
	.def	bufferlimH		= r9	;Limit of buffer - used by parcer
	.def	looptarget		= r10	;FOR-NEXT
	.def	loopvariable	= r11	;FOR-NEXT
	.def	loopreturnL		= r12	;FOR-NEXT
	.def	loopreturnH		= r13	;FOR-NEXT
	.def	gosubretL		= r14	;GOSUB return address Low
	.def	gosubretH		= r15	;GOSUB return address High
	.def	temp 			= r16	;General purpose variable
	.def	inchar 			= r17	;Char destined to go out the uart
	.def	outchar 		= r18	;Char coming in from the uart
	.def	inbytel 		= r19	;Lower byte for asci-hex conversion
	.def	inbyteh 		= r20	;Higher byte for asci-hex conversion
	.def	ecode			= r21	;Cast returned here and error code stored here
	.def	dstackpointer	= r22	;Data stack pointer.
	.def	U				= r23	;Units 
	.def	T				= r24	;Tens
	.def	H				= r25	;Hundreds
	;note   XL				= r26	;Flags register
	;note	XH				= r27	;Flags register
;Notes on XL, flag register bit useage.
;	bit	0		=1 for variabls to leave pointers on stack, =0 for values on stack
;	bit 1		=1 to skip next line during RUN execution
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
;*********************************************************************
.include "Macro_Misc.inc"
.include "Macro_IO.inc"
.if DELAY
  .include "Macro_Delays.inc"
.endif
;
	.cseg
;/////////VECTORS//////////////// 
	.org	$00
	ldi		temp,ramend ;low(ramend)		
	out		spl,temp     ;Set the stack pointer

	clr		r1			;Zero a register
	out 	DDRB,r1		;Port B all inputs
	out		DDRD,r1		;Port D all inputs (except UART)
#if defined(__AT90S2313__)
	ldi     temp,low(((FCLK/16)/baudrate)-1)
;
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	ldi     temp,high(((FCLK/16)/baudrate)-1)     
;
#else
#error	"Target MCU unknown!"
#endif
	Goto	init		;continue reset setup

;////////ONLY URXCaddr INTERRUPT USED//////
.org URXCaddr			; UART, Rx Complete
	reti

init:     
; USART: Set baudrate
#if defined(__AT90S2313__)
	out		UBRR,temp     	;Load UART baud rate

; USART: RX Interupts, enable TX and RX
	ldi		temp,(1<<RXCIE|0<<TXCIE|0<<UDRIE| \
					1<<RXEN|1<<TXEN)
	out		UCR,temp
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	out 	UBRRH,temp     	;Load UART baud rate
	ldi     temp,low(((FCLK/16)/baudrate)-1)
	out		UBRRL,temp     	;Load UART baud rate
; USART: RX Interupts, enable TX and RX
	ldi		temp,(1<<RXCIE|0<<TXCIE|0<<UDRIE| \
					1<<RXEN|1<<TXEN|0<<UCSZ2)
	out		UCSRB,temp
#else
#error	"Target MCU unknown!"
#endif

.if RESET
;	Disable watchdog timer in case we arrived here via WDT
  #if defined(__AT90S2313__)
	ldi		U,(1<<WDTOE|1<<WDE)	;1st write to WDT
	ldi		T,(0<<WDE)			;2313 always reset

	STORE	WDTCR,U				;write WDTCR register
	STORE	WDTCR,T				;and again to disable WDT
  #elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	CLRB	MCUSR,WDRF,temp		;clear WDRF flag
	ldi		U,(1<<WDCE|1<<WDE)	;1st write to WDT
	ldi		T,(1<<WDCE|0<<WDE)	;2313 always reset

	STORE	WDTCSR,U			;write WDTCSR register
	STORE	WDTCSR,T			;and again to disable WDT
  #endif
.endif
;
	ldi		temp,(0<<SM|1<<SE)
	out		MCUCR,temp		;enable sleep mode, default "idle"

	GoSub	newprogram		;Initialize program memory	
		
.if SStst
	sbi		PORTD,PD0		;Set PORTD bit 0 weak pullup
	sbic	PIND,PD0		;If PIND,0 is grounded then execute from EEPROM
	Goto	readytorun
	GoSub	loadcommand
	GoSub   runcommand
.endif

;=================================================================
readytorun:
	GoSub	newprogramgreet	;Initialize program memory and print greeting	
	
;=================================================================
main:	
	sei						;insure unterrupts enabled
	GoSub	crlf
	ldi		dstackpointer,dstacktop	;Initialize data stack pointer 
	GoSub	getterminalline	;Gen line from terminal (19 chars + $0D)
	GoSub	Interpretelinebuffer
	Goto	main

;=================================================================
fetchrombyteinc:
	inc	ZL	
fetchrombyte:	;Read byte from rom, advance pointer.
				;Enter with ZH, ZL pointing to rom list.
	lpm			;Load byte from program memory into r0
	adiw	ZL,1		;Increment Z registers
	tst		romreg		;Check if we've reached the end of the list of commands
	breq	endoflist	;If so,return
	ret

endoflist:
	ldi		ecode,$00
	Goto	error

;endoflist:			;Return to previous calling routine using a	
;	in	temp,spl	;skanky trick with stack pointer. Get pointer
;	subi	temp,$FE	;Add 02 to value (undo one return address)
;	out	spl,temp	;Put value back in stack pointer
;	ret			;Pop up to previous call. Don't try this at home.


;=================================================================
getexecword:	;Find the address of the instruction that matches the three characters in H,T,and U.
		;Execute associated instruction then return.
	SetZPtr	(2*commandlist)	;Load high part of byte address into ZH
tryanother:
	GoSub	fetchrombyte		;See if the string matches contents of H and T
	cpse	romreg,H
	Goto	nomatchH
	GoSub	fetchrombyte
	cpse	romreg,T
	Goto	nomatchT
	GoSub	fetchrombyte
	cpse	romreg,U
	Goto	nomatchU
					;OK, A match has been found, get the address of the routine.
	adiw	ZL,1			;Increment Z registers -skip unused 4th instruction name byte
	lpm				;Get address low into r0 (romreg)
	mov		temp,romreg		;Stash low byte in temp
	adiw	ZL,1			;Increment Z registers to point to high byte
	lpm				;Get address high into r0 (romreg)
	mov		ZH,romreg		
	mov		ZL,temp	
	icall				;Do an indirect jump
	ret				;Then return when function is complete
     		
nomatchH:
 	adiw	ZL,1			;Increment Z registers -skip T
nomatchT:
	adiw	ZL,1			;Increment Z registers -skip U
nomatchU:
	adiw	ZL,3			;Increment Z registers -skip address (3 bytes)
 	Goto	tryanother
	
;=================================================================
formword:	;Put up to three consecutive non-delimiters in registers H,T,and U, right-justified
		;Enter with YH, YL pointing to first char in buffer to be parsed
		;Output: non-delimiters in registers T,H,and U, right-justified, count of 
		;chars in outchar. If three chars are collected, this routine advances
		;YL until the cast changes. Leaves with YH,HYL pointing to next char to be parsed.
		;Used also: temp and inbyteh.
		
	ldi		H,$F0	 	;U is always loaded, no need to ininitalize
	ldi		T,$F0		;Pre-load the others
	
	ldi		outchar,$01
	ld		temp,Y		;Get char from linebuffer into U


	GoSub	Qcast		;See what type of char it is.
	cpi		ecode,$01	;If the char is a delimiter (type 1), skip
	brne	nottype1cast	; **** advancelbpointer only once, just after Qcast
	GoSub	advanclbpointer	
	Goto	formword	
nottype1cast:
	mov		U,temp 		;Store char and advance pointer
	GoSub	advanclbpointer
	cpi		temp,$0D	;If its a carriage return, stop and return.
	breq	exitfromword0D
	mov		currentcast,ecode;Save cast type for reference

Getanotherhcar:
	ld		temp,Y		;Get char from linebuffer
	GoSub	Qcast
	cpi		temp,$0D	;If carriage return, then return
	breq	exitfromword0D
	cp		currentcast,ecode ;If not same as cast of first char, then return.
	brne	exitfromword
	mov		H,T					;Store char and advance pointer
	mov		T,U
	mov		U,temp  
	GoSub	advanclbpointer
	inc		outchar
	cpi		outchar,$03	;If third character, exit
	brne	Getanotherhcar	;Otherwise, get another
exitfromword:
				;The code below is to make sure ponter is not 
				;left pointing to the end of a word that is longer
				;than three chars long. It is not 
				;entered when the pointer is pointing to the
				;carriage return at the end of the buffer.
	ld		temp,Y		;Get char from linebuffer
	GoSub	Qcast
	cp		currentcast,ecode
	breq	notclearedgroup
exitfromword0D:
	ret
	
notclearedgroup:
	GoSub	advanclbpointer
	Goto	exitfromword
	
;=================================================================
advanclbpointer:
		;Decrements YL, Jumps to error routine if fetch is requester after 
		;pointer is moved past the end of the buffer. Enter with buffer limits 
		;in bufferlimH,bufferlimL
	cp		YH,bufferlimH
	brne	nohitend
	cp		YL,bufferlimL
	brne	nohitend
	ldi		ecode,$04
	Goto	error
	ret
nohitend:
	dec		YL
	ret

;=================================================================
Interpretelinebuffer:		;Interpret line
	SetYPtr	lbufftop		;Initialize line buffer pointer to lbufftop 
	ldi		temp,lbuffbot-1	;Set buffer limits to those of line buffer.
	mov		bufferlimL,temp
	clr		bufferlimH
	ld		temp,Y			;Get first char from linebuffer
	GoSub	Qcast
	cpi		ecode,$02
	brne	dotheline		;First char is not a numeral so interpret line
	Goto	storeline		;If it's a number, then it's a line number -store it.
dotheline:
	GoSub	crlf
	ori		XL,$01		;Flag variable ponters to be left on stack
	Goto	interpretline	
	
;=================================================================
interpretlinev:
	andi	XL,$FE		;Flag to leave value of variables on stack
interpretline:	;Interpret line. Enter with YH, YL pointing to char to be interpreted
	in		inbytel,spl	
	cpi		inbytel,$CB
	brpl	stacknotexceeded
	ldi		ecode,$08
	Goto	error
stacknotexceeded:

	GoSub	formword 
	cpi		U,$0D
	breq	endofline

	mov		temp,currentcast ; Check to see if its a number	
	cpi		temp,$02
	brne	dontmakenumber	
	andi	H,$0F		;Its a number so make binary and push on stack
	andi	T,$0F
	andi	U,$0F
	GoSub	decimaltobinary	;change from 3 BCD bytes to one binary vaue
	GoSub	pushU
	Goto	interpretline	;Keep going until CR is found
	
dontmakenumber:
	andi 	H,$5F		;Upper-case H,T,U
	andi	T,$5F
	andi	U,$5F	

	cpi		outchar,$01	;If its a type 3 (letter) and 1 char long its a variable.
	brne	notavar		;(If outchar = 1 and ecode = 3 its a variable)
	mov		temp,currentcast
	cpi		temp,$03
	brne 	notavar
	GoSub	processvariable
	Goto	interpretline

notavar:
	GoSub	getexecword	;Check for CR after coming back from routine
	cpi		U,$0d
	breq	endofline
	ret
					
endofline:
	ori		XL,$01		;Flag variable ponters to be left on stack next line
	ret
	
	
;=================================================================
processvariable:	;Make variable in A into a pointer and
			;if XL bit 0 is 1, put pointer on stack (assignment to variable), or
			;if XL bit 0 is 0 put the value of variable on stack.
	
	subi	U,$41		;Make into an offset		
	cpi		U,varsize	;Test for range
	brmi	variableok
	ldi		ecode,$05
	Goto	error
variableok:
	subi	U,-varbot	;Make into a pointer
	sbrs	XL,$00	
	brne	notcodezero	
	Goto	PushUret
notcodezero:
	ldi		ZH,0		;Put contents of variable on stack
	mov		ZL,U		
	ld		U,Z	
	Goto	PushUret
;
;=================================================================
pushU:		;Push contents of U on datastack
	clr		ZH
	mov		ZL,dstackpointer
	st		Z,U
	dec		dstackpointer
	cpi		dstackpointer,dstackbot-1 ;If no stack undrflow, return.
	brpl	nopusherror
	ldi		ecode,$06
	Goto	error
nopusherror:
	ret
	
	
;=================================================================
popU:		;Pop contents of U from datastack
	inc		dstackpointer	;
	cpi		dstackpointer,dstacktop+1
	brmi	nopoperror
	ldi 	ecode,$07
	Goto	error
nopoperror:
	clr		ZH
	mov		ZL,dstackpointer
	ld		U,Z
	ret
	
;=================================================================
storeline:	;Store line in next available location in program memory from first char to $0D
		;Enter with YH, YL pointing to first char of line
	Copy16	ZH,ZL,PChigh,PClow	;Get current program memory 
								; location from PC high and PC low
anotherlinchar:
	cpi		ZL,pmembot
	breq	memoryfull
	ld		temp,Y		;Put contents of line buffer into temp
	st		Z,temp		;Put temp into the program memory
	dec		ZL
	dec		YL
	cpi		temp,$0D
	brne	anotherlinchar
	mov		PClow,ZL
	ret

memoryfull:
	ldi		ecode,$02
	Goto	error

		
;=================================================================
Qcast:		;Determiin whether character is a letter, numeral, delimiter, or other (operator).
		;Input: temp. Output: ecode. Uses: inchar
		;Value in ecode = cast
		;0 = Operator (not one of the other casts
		;1 = Delimiter -space ($20) or comman ($2C)
		;2 = Numeral 0..9 ($30 through $39)
		;3 = Letter -A..Z uppercase ($41 throught $5A)
		;4 = Carriage return
		;Tests are made after anding the input byte with $5F
		
	mov		inchar,temp		;Copy to inchar for modification
	andi	inchar,$5F		;Make upper-case
	clr		ecode			;Default is type 0
	cpi		inchar,CR		;Is it a carriage return?
	brne	not0D
	Goto	makecast4
not0D:
	cpi		inchar,(SPACE & 0x5F)	;Is it a space?
	brne	notspacecode
	Goto	makecast1
notspacecode:
	cpi		inchar,COMMA	;Is it a commna?
	brne	notaspacecode	
makecast1:
	ldi		ecode,$01
	ret	
notaspacecode:
	cpi		inchar,$10
	brpl	notunder10
	ret
notunder10:
	cpi		inchar,$5B
	brmi	notover5B	
	ret	
notover5B:
	cpi		inchar,$40
	brmi	notover40
	ldi		ecode,$03
	ret
notover40:	
	cpi		inchar,$1A	
	brpl	notunder1A
	ldi		ecode,$02
notunder1A:
	ret
makecast4:
	ldi		ecode,$04
	ret
	
deletekey:			;Backup up cursor -destructive backspace
	cpi 	YL,lbufftop
	brne	notexceededbuftop
	ldi		outchar,BELL	;If not CR at last position, ring the bell
	GoSub	emitchar
	Goto	anothertermchar
notexceededbuftop:
	mov		outchar,inchar
	GoSub	emitchar0d
	inc 	YL
	ldi		inchar,CR
	st		Y,inchar	;Put in line buffer
	Goto	anothertermchar
	
;=================================================================
getterminalline: 	;Get characters from terminal into linebuffer.
;					; Stop accepting chars except CR when end of
					; buffer is reached.
	ldi		outchar,PROMPT
	GoSub	emitchar
	SetYPtr	lbufftop			;Initialize line buffer pointer to lbufftop 
anothertermchar:
	GoSub	recvchar			;Get char from terminal
	cpi		inchar,BS			;Is it backspace/delet character?
	breq	deletekey
	cpi		inchar,LF			;Is it linefeed character?
	brne	anothertermchar1	;if not, continue
	Goto	anothertermchar		;ignore linefeeds and get another character
anothertermchar1:
	st		Y,inchar			;Put in line buffer
	cpi		inchar,CR
	breq	CRreceived			;If char was CR,then return
	cpi		YL,lbuffbot			;If buffer is at last byte, don't store char but beep.
	brne	notlbuffend	
	ldi		outchar,BELL		;If not CR at last position, ring the bell
	GoSub	emitchar
	Goto	anothertermchar
notlbuffend:
	dec		YL					;Not end of buffer and not CR, so emit and go another
echoandgo:
	mov		outchar,inchar
	GoSub	emitchar0d
	Goto	anothertermchar
CRreceived:						;Last char in line received
	ret
	
;=================================================================
Error:
	SetZPtr	(2*errormessage)	;Load high part of byte address into ZH

	GoSub	sendromstring		;sent it.
	mov		inbytel,ecode
	GoSub	sendbyte1

	ldi     temp,ramend 		;low(ramend)
	out     spl,temp     		;Set the stack pointer

	GoSub	newprogram1			;reset dstackpointer and flags

;	ldi		dstackpointer,dstacktop	;Initialize data stack pointer
;	clr		XL				;Clear interpreter mode flags
;	clr		XH

	Goto	main
	
;=================================================================
sendbyte:      ;Send byte contained in inbyteh & inbytel to terminal
	GoSub 	byte_to_asciihex
	mov     outchar,inbyteh
	GoSub 	emitchar
sendbyte1:
	mov     outchar,inbytel
	GoSub  	emitchar
	ret	
	
;=================================================================
emitchar0:			;emit char in r0
	mov	outchar,r0	
emitchar:
;	sbi		ucr,txen	;enable xmit
#if defined(__AT90S2313__)
	sbis	USR,UDRE	;wait until the register is cleared
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	sbis	UCSRA,UDRE	;wait until the register is cleared
#endif
	Goto	emitchar     
	out		udr,outchar	;send the byte
;	cbi		ucr,txen	;clear xmit enable
	ret		;go back
	
;=================================================================
crlf:   
	ldi		outchar,$0D	;Send carriage return and line feed to terminal
	GoSub	emitchar
	ldi		outchar,$0A
	GoSub 	emitchar
	ret

;=================================================================
emitchar0D:			;Send outchar to terminal. Add line feed to carriage return.
	cpi		outchar,$0D	;Its a carraiage return, send a linefeed also
	brne	notareturn
	GoSub	crlf
	ret
notareturn:   
	Goto	emitchar

;=================================================================
recvchar:     	;Receive a byte from the terminal         
	sleep						;go to to sleep
#if defined(__AT90S2313__)
	sbis	USR,RXC		;Wait for byte to be received
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	sbis	UCSRA,RXC		;Wait for byte to be received
#endif
	Goto 	recvchar
	in		inchar,udr	;Read byte
	ret			

;=================================================================
sendromstring:			;call with location of string in Z
   	lpm			;Load byte from program memory into r0
   	tst		r0		;Check if we've reached the end of the message
   	breq	finishsendstring;If so, return
   	GoSub	emitchar0
   	adiw	ZL,1		;Increment Z registers
   	Goto	sendromstring
finishsendstring:
   	ret

;=================================================================
sendlromline:			;Send a string terminated in cariage return and line feed
				;Call with location of start of string in Z               
	GoSub	sendromstring
	GoSub	crlf
	ret

;/////////////START DATA FORMAT CONVERSION ROTINES//////////////////

;=================================================================
bitpositiontoormask:	;Convert bit position (0..7) to or mask value
			;Enter with bit position on top of dstack, exit with
			;mask on top of orstack
			
	GoSub	popU		;Get bit position
	ldi		temp,$07
	cp		temp,U
	brmi	toomanybits			
	ldi		temp,$01	;Put 0000 0001 pattern into temp for shifting
rotatesomemore:
	cpi		U,$00
	breq	finishedshifting
	lsl		temp
	dec		U
	Goto	rotatesomemore
finishedshifting:
	mov		U,temp
	Goto	PushUret

toomanybits:
	ldi		ecode,$0A
	GoSub	error
;
sendHTUasdecimal:	;Send decimal number in H,T,U registers (hundreds,tens,units)
			;As numerals.
	tst		H
	breq	dontsendh
	mov		outchar,H
	GoSub	sendit
dontsendh:	
	tst		T
	breq	dontsendt
	mov		outchar,T
	GoSub	sendit
dontsendt:
	mov		outchar,U
sendit:
	subi	outchar,-48	
	GoSub	emitchar
	ret
;	
;=================================================================
binarytodecimal:	;Enter with 8 bit value in U, Exits with numerals in H,T,U
			;with Hundreds, Tens, and Units.
	clr		H
	clr		T
anotherh:
	subi 	U,100	;Find out how many hundreds in U
	brcs	hdone
	inc		H
	Goto	anotherh
hdone:
	subi	U,-100	;Subtracted one too many -add back.
anothert:
	subi	U,10
	brcs	tdone
	inc		T
	Goto	anotherT
tdone:
	subi	U,-10
	ret
	
;=================================================================
byte_to_asciihex:     ;convert byte in inbytel to ascii in inbyteh,inbytel
	mov		inbyteh,inbytel
	swap	inbyteh				;swap nibbles
	andi	inbyteh,$0F			;strip off low nibble
	subi	inbyteh,-$30		;add $30
	cpi		inbyteh,$3A    
	brlo	byte_to_asciihex1	;If less than 9 skip next instruction
	subi	inbyteh,-7 			;add 7 to ASCII (if data greater than 9)
     ; byte in inbyteh represents upper nybble that was in inbytel at start
byte_to_asciihex1:
	andi	inbytel,$0F			;strip off low nibble
	subi	inbytel,-$30		;add $30
	cpi		inbytel,$3A     
	brlo	byte_to_asciihex2	;If less than 9 skip next instruction
	subi	inbytel,-7 			;add 7 to ASCII (if data greater than 9)
     ; byte in inbyteh represents upper nybble that was in inbytel at start
byte_to_asciihex2:
	ret

;=================================================================
asciihex_to_byte:     	;convert ascii in inbyteh,inbytel to byte in inbytel
	sbrc	inbyteh,6	;convert high byte
	subi	inbyteh,-9	;add     inbyte,temp     ;if bit 6 is set, add 9
	andi	inbyteh,$0F

	sbrc	inbytel,6	;convert low byte
	subi	inbytel,-9	;add     inbyte,temp     ;if bit 6 is set, add 9
	andi	inbytel,$0F

	swap	inbyteh     ;swap nibbles
	or		inbytel,inbyteh
	ret
	
	
;=================================================================
decimaltobinary:	;Convert decimal number in H,T,U registers (hundreds,tens,units) to 
			;binary value in U register.		
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
	ldi		ecode,$01
	Goto	Error	


;/////////////////END DATA FORMAT CONVERSION ROTINES/////////////////
;=================================================================
TypeGreeting:				;Type greeting
	GoSub	crlf			; **** consider combinng with sendromline if sendromline is not used elsewhere
	SETZPTR	(2*signonmessage)	;Load high part of byte address into ZH
	GoSub	sendlromline		;sent it.
	ret
;
;=================================================================
listcommand:	;lists program memory buffer to screen..
	GoSub	crlf
	clr		ZH		;Initialize PC position inidcator
	ldi 	ZL,pmemtop
anotherchar:
;	cpi	ZL,pmembot
	cp		ZL,PClow
	breq	endofpmem
	ld		outchar,Z	;Put contents of line buffer into outchar;	
	GoSub	emitchar0D
	dec		ZL
	Goto	anotherchar
endofpmem:
	GoSub	crlf
	;(list flows into sizecommand)

;=================================================================
sizecommand:	;Find remaining program memory space. Uses U.
	GoSub	crlf
	mov		U,PClow		
	subi	U,pmembot
	GoSub	binarytodecimal
	GoSub	sendHTUasdecimal
	SetZPtr	(2*freeword)	;Load high part of byte address into ZH
	GoSub	sendlromline		;sent it.
	ret

;=================================================================
printcommand:			;Print TOS after call to screen
	GoSub	Get1stParm
	GoSub	binarytodecimal
	GoSub	sendHTUasdecimal	
	GoSub 	crlf
	ret

;=================================================================
printhexcommand:			;Print TOS in hex, after call, to screen
	GoSub	interpretlinev
	ldi		outchar,$24
	GoSub	emitchar
	GoSub	popU
	mov		inbytel,U
	GoSub	sendbyte	
	GoSub 	crlf
	ret

;=================================================================
prbcommand:	;print in binary format
	GoSub	Get1stParm
	ldi		temp,$08
stillsendingbinary:	
	ldi		outchar,$30
	rol		U
	brcc	dontsendone
	ldi		outchar,$31
dontsendone:
	GoSub  	emitchar
	dec 	temp
	brne	stillsendingbinary 
	GoSub	crlf   	
	ret		

;=================================================================
setequalscommand:		;Get value at TOS after call to variable on stack when 
				;equals command is called.
	GoSub	Get1stParm
	mov		temp,U
	GoSub	popU
	ldi		ZH,$00
	mov		ZL,U
	st		Z,temp
	ret

;=================================================================
evaluatecommand:		;If XL,2 = 0, perform subtract command. 
				;If XL,2 = 1, set up FOR-NEXT loop
	sbrs	XL,$02
	Goto	subtractcommand	;Its functionally identical to subtract.
				;Set up FOR-NEXT loop
	GoSub	Get1stParm
	mov		looptarget,U	;Store loop target
	
	GoSub	popU		;Pop initial counter value
	mov		T,U		;Put in T for now

	GoSub	popU		;Pop pointer to counter
	
	mov		loopvariable,U	;Copy variable pointer into loopvariable register

	ldi		ZH,$00
	mov		ZL,U		;Copy  variable pointer into ZL as index
	st		Z,T		;Store initial counter value into counter
	
	ori		XL,$20		;Set flag asking runcommand to capture loop address
	
	ori		XL,$08		;Set flag activating NEXT
	andi	XL,$FA
	ret
	
;=================================================================
nextcommand:			;The distant end of the FOR-NEXT loop
				;Tests the loop variable to see its equal target.
				;If its equal to target, go to next line
				;If not equal target, increment variable and 
				;Signal RUNCOMMAND to jump to loop return
						
	clr		ZH		;As if ZH was anything but zero on this chip
	mov		ZL,loopvariable
	ld		temp,Z		;Get loop variable value into temp
	cp		temp,looptarget
	breq	finishedfnloop	;If they are equal, the loop's done
	inc		temp
	st		Z,temp		;Increment loop variable and put it back into the register			
	ori		XL,$10		;Set XL,4 to flag run command to jump to return address next time
finishedfnloop:
;	andi	XL,$F7		;Make sure that next flags is cleared 
	ret

;=================================================================
notequalcommand:		; <> command -Returns opposite result of subtractcommand
	GoSub	interpretline
	GoSub	popU
	mov		temp,U
	GoSub	popU
	cp		U,temp
	breq	makeff
make00:	
	clr		U
	Goto	PushUret
makeff:	
	ser	U
	Goto	PushUret
	
.if AComp
acocommand:			;Put inverse of analog comparitor output on stack (0 for high)
	clr		U			;U = 0
	sbis	ACSR,ACO
	inc		U			;U = 1
	Goto	PushUret
.endif
	
;=================================================================
peekcommand:			;Replace value on TOS (Top Of Stack) with contents of memory
				;at that location.
	GoSub	Get1stParm
	mov		ZL,U
	clr		ZH
	ld		U,Z
	Goto	PushUret
;
;=================================================================
pokecommand:			;Store data at one down from TOS at location pointed to by TOS
	GoSub	GetTwoParm
	clr		ZH
	mov		ZL,temp
	st		Z,U
	ret
	
;=================================================================
addcommand:
	GoSub	GetTwoParm
	add		U,temp
	Goto	PushUret


;=================================================================
subtractcommand:
	GoSub	GetTwoParm
	sub		U,temp
	Goto	PushUret

;=================================================================
andcommand:
	GoSub	GetTwoParm
	and		U,temp
	Goto	PushUret

;=================================================================
orcommand:
	GoSub	GetTwoParm
	or		U,temp
	Goto	PushUret

;=================================================================
eorcommand:
	GoSub	GetTwoParm
	eor		U,temp
	Goto	PushUret

makeU1:
	ldi		U,1
	Goto	PushUret

;=================================================================
lessthancommand:
	GoSub	GetTwoParm
	cp		U,temp
	Goto	evaldiff

.if PWM	
;=================================================================
pwm8command:
	GoSub	interpretlinev
	ldi		temp,(1<<PWM10)+(1<<COM1A1);set timer1 PWM mode
	out		TCCR1A,temp 		; 8 bit PWM not reverse (Fck/510)
	out		TCCR1B,temp 		; prescaler = 1
	sbi		DDRB,3
	GoSub	popU
	out 	OCR1AH,r1			;r1 is always "0"
	out		OCR1AL,U
	ret

;=================================================================
pwecommand:
	GoSub	interpretlinev
	ldi		temp,$83		;set timer1 PWM mode
	out		TCCR1A,temp 		; 10 bit PWM not reverse (Fck/510)
	ldi		temp,$01
	out		TCCR1B,temp 		; prescaler = 1
	sbi		DDRB,3
	GoSub	popU
	mov		temp,U
	GoSub	popU
	out 	OCR1AH,U
	out		OCR1AL,temp
	ret

;=================================================================
pwocommand:				;Pulse Width Modulation OFF	
	out		TCCR1A,r1	;set timer1 PWM mode
	ret
.endif

;=================================================================
greaterthancommand:
	GoSub	GetTwoParm
	cp		temp,U
evaldiff:
	ldi		U,0
	brpl	makeU1
	breq	makeU1
	Goto	PushUret

;=================================================================
ifcommand:	;Set falg to skip next line if number on TOS returned from rest 
		;of line is not zero
	GoSub	Get1stParm
	tst		U		;
	breq	itszero
	ori		XL,$02
itszero:
	ret
	
;=================================================================
thencommand:			;Return to previous calling routine using a	
	pop		temp		;skanky trick with stack pointer. pop
	pop		temp		; two bytes off the stack
	ret	

;=================================================================
forcommand:			;Initial statement in FOR-NEXT structure
				;Sets flag to switch "=" routine to set up the
				;FOR-NEXT loop
	ori		XL,$04		;Set bit 3
	ret

;=================================================================
tocommand:			;To in FOR-NEXT structure. This is a dummy command
	GoSub	interpretlinev
	ret

;=================================================================
endcommand:			;Stop execution by setting flag
	ori		XH,$02		;Set flag telling runcommand to stop.
	ret
		
;=================================================================
hexcommand:			;Nasty and dangerous - get two following chars as hex put on stack.
	ld		inbyteh,Y
	GoSub	advanclbpointer
	ld		inbytel,Y
	GoSub	asciihex_to_byte
	mov		U,inbytel
	GoSub	pushU
	GoSub	advanclbpointer
	GoSub	interpretlinev
	ret 	
	
;=================================================================
getlineno:			;Search buffer for line number, set up for runcommand to jump to it.
				;Can use Y because will will be modified by this routine
				;before runcommand uses it again anyway.
	GoSub	Get1stParm

	mov		inbyteh,U	;Put target line number into inbytel
	clr		ZH		
	ldi 	ZL,pmemtop	;Initialize PC position inidcator
	Goto	pufistln	;Jump in the middle so first line no. can be found
	
;=================================================================
searchCR:
	cp		PClow,ZL	;Error if past end of buffer
	brpl	gotonotfound
	ld		temp,Z
	cpi		temp,$0D
	breq	checkforlinumb	;Take the branch if CR is found
	dec		ZL
	Goto	searchCR		;Else continue to look for CR

checkforlinumb:				;Found a CR now make line number
	dec		ZL				;Move pointer past CR
pufistln:
	clr		U
	clr		T
	clr		H
		
fetchanothernum:	
	cp		PClow,ZL		;Error if past end of buffer
	brpl	gotonotfound
	ld		temp,Z
	GoSub	qcast			;Find out what kind of char this is
	cpi		ecode,$02		;$02 is a numeral
	brne	notanumeral
	mov		H,T				;It's a numeral so shift it in
	mov		T,U
	mov		U,temp
	dec		ZL	
	Goto	fetchanothernum
notanumeral:				;The pointer is not poiting to a numeral, so test the number.
	
	andi	H,$0F			;It's a number so make binary and push on stack
	andi	T,$0F
	andi	U,$0F
	GoSub	decimaltobinary	;Change from 3 BCD bytes to one binary vaue	
	cp		inbyteh,U
	brne	searchCR	;No the current line number is in binary form
	ret
	
gotonotfound:
	ldi		ecode,$09	;Claim error - goto line not found
	Goto	error
	
;=================================================================
gotocommand:
	GoSub	getlineno
	mov		YH,ZH
	mov		YL,ZL
	ret

;=================================================================
gosubcommand:			;Do a gosub
	GoSub	getlineno	;Get destination address in ZH and ZL	
	ori		XL,$C0		;Set flags to capture return address and jam destination address
	ret
	
;=================================================================
returncommand:			;Return from gosub
	sbrs	XH,2
	Goto	returnnotactive
	ori		XH,$01		;Set flag calling for jaming of return address	
	ret
returnnotactive:
	ori		XH,$02		;Set flag calling for program halt
	ret	
;
;=================================================================
Savecommand:		;Save contents of memoru from dstacktop to pmembot to EEPROM eepromtop down.
			;Values of PChigh and PClow to be stored in top two bytes of dstack.
	ldi		dstackpointer,dstacktop	;Initialize data stack pointer
	mov		U,PClow
	GoSub	pushU
	mov		U,PChigh
	GoSub	pushU
	clr 	YH	
	ldi		YL,dstacktop
	ldi		ZL,eeprogstortop
storeanotherchar:
	ld		temp,Y			;Put contents of program memory into temp
wrat1:
	sbic	eecr,eewe		;Wait for EEPROM write to not be busy
	Goto	wrat1
	out		eear,ZL				
	out		eedr,temp		;Set up the  write data
	sbi		eecr,eemwe
	sbi		eecr,eewe		;Trigger the write
	cpi		YL,pmembot
	breq	finishedsaving
	dec		ZL
	dec		YL
	Goto	storeanotherchar
finishedsaving:
	ret

;=================================================================
Loadcommand:		;Save contents of memoru from dstacktop to pmembot to EEPROM eepromtop down.
			;Vales of PChight and PCLow to be read from top two bytes of dstack.
	SetYPtr	dstacktop		;point Y to top of datastack
	ldi		ZL,eeprogstortop
loadanotherchar:
wratz1:	
	sbic	eecr,eewe		;Wait for EEPROM write to not be busy
	Goto	wratz1
	out		eear,ZL			;move to EEPROM address register
	sbi		eecr,eere		;Trigger the read
	in		temp,eedr		;Get the data into temp
	st		Y,temp
	cpi		YL,pmembot
	breq	finishedloading
	dec		ZL
	dec		YL
	Goto	loadanotherchar
finishedloading:
	ldi		dstackpointer,dstacktop-2 ;Initialize data stack pointer
	GoSub	popU
	mov		PChigh,U
	GoSub	popU
	mov		PClow,U
	ret

;=================================================================
emitcommand:
	GoSub	Get1stParm
	mov		outchar,U
	GoSub	emitchar
	ret

;=================================================================
keycommand:		;Receive a byte from the terminal     
	sleep						;go to to sleep
#if defined(__AT90S2313__)
	sbis	USR,RXC		;Wait for byte to be received
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	sbis	UCSRA,RXC		;Wait for byte to be received
#endif
	Goto 	keycommand
	in		U,udr	;Read byte
	cpi		U,$03	;Is it control-C?
	brne	notermhalt1
	ori		XH,$02		;set flag to halt program
notermhalt1:
	Goto	PushUret
;
.if RESET
;*********************************************************************
;RSTcommand: causes a system reset using the watchdog timer
;	set to "System Reset" mode.  The time delay defaults to 32mS.
;*********************************************************************
RSTcommand:
	cli							;disable global IRQ's
#if defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	CLRB	MCUSR,WDRF,temp		;clear WDRF flag
#endif

#if defined(__AT90S2313__)
	ldi		U,(1<<WDTOE|1<<WDE)
	ldi		temp,(1<<WDTOE|1<<WDE)	;2313 always reset
	ldi		temp,(0<<WDTOE|1<<WDE|5<<WDP0)	;~1S

	STORE	WDTCR,U				;write WDTCR register
	STORE	WDTCR,temp			;and again to enable WDT
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	ldi		U,(1<<WDCE|1<<WDE)
	ldi		temp,(0<<WDIE|1<<WDE|WDT_1S)	; tiny2313(A) to
											; "system reset" mode
	STORE	WDTCSR,U			;write WDTCSR register
	STORE	WDTCSR,temp			;and again to enable WDT
#endif

RSTcommand1:
	sleep						;let it happen
	nop
	ret
.endif
;
.if DELAY
;****************************************************************************
;delaycommand:  Delays "N" x 10mS (10 to 2550 mS) based on the value passsed
;	using a simple software driven loop
;	USES: U, temp and Y-ptr.
;****************************************************************************
DELAYcommand:
	PUSHY						;save Y reg
	GoSub	Get1stParm			;get the last parameter

delaycommand1:
	_WAIT_mS	FCLK,10			;wait 10mS
;	_WAIT_mS	FCLK,9			;wait 9mS
;	_WAIT_10us	FCLK,97			;wait 970uS
	dec		U					;decrement variable
	brne	delaycommand1		;loop till zero
	POPY						;restore Y reg
	ret
.endif
;
.ifdef PORTB
;=================================================================
inbcommand:				;input from PINB
	in		U,PINB
	Goto	PushUret

;=================================================================
outpbcommand:			;Output to PORTB
	GoSub	Get1stParm
	out		PORTB,U
	ret
;
;=================================================================
outdbcommand:			;Output to DDRB
	GoSub	Get1stParm
	out		DDRB,U
	ret
;
;=================================================================
setbitinB:				;Set bit in position inidicated at top of stack.
	GoSub	Get1stParmOred
	in		temp,PORTB
	or		temp,U
	rjmp	OutPortBret

;=================================================================
clearbitinB:				;Clear bit in position inidicated at top of stack.
	GoSub	Get1stParmOred
	com		U
	in		temp,PORTB
	and		temp,U
	rjmp	OutPortBret

;=================================================================
; OutPortBret: code compaction for the above routines.
;	Called with value in temp register
;=================================================================
OutPortBret:
	out		PORTB,temp
	ret
;
.endif

.ifdef PORTD
;=================================================================
indcommand:				;input from PIND
	in		U,PIND
	andi	U,$FC
	Goto	PushUret
;
;=================================================================
outpdcommand:			;Output to PORTD
	GoSub	Get1stParm
	andi	U,$FE		;D0 is not an output -its the UART xmit pin
	out		PORTD,U
	ret

;=================================================================
outddcommand:			;Output to DDRD
	GoSub	Get1stParm
	out		DDRD,U
	ret

;=================================================================
setbitinD:				;Set bit in position inidicated at top of stack.
	GoSub	Get1stParmOred
	in		temp,PORTD
	or		temp,U
	rjmp	OutPortDret

;=================================================================
clearbitinD:				;Clear bit in position inidicated at top of stack.
	GoSub	Get1stParmOred
	com		U
	in		temp,PORTD
	and		temp,U
	rjmp	OutPortDret

;=================================================================
; OutPortDret: code compaction for the above routines
;=================================================================
OutPortDret:
	out		PORTD,temp
	ret
;
.endif
;
;=================================================================
; Calls line interpreter, calculates the bit-OR mask, fetches 1st parameter 
;	on the and returns with it in U. 
Get1stParmOred:
	GoSub	interpretlinev
	GoSub	bitpositiontoormask
	GoSub	popU
	ret
;
;=================================================================
; Calls line interpreter, fetches 1st parameter on the and returns with 
;	it in U. 
Get1stParm:
	GoSub	interpretlinev	;Get target line number to TOS (Top Of Stack)
	GoSub	popU
	ret
;
;=================================================================
; Calls line interpreter, fetches 1st parameter, places it in temp, fetches the 
;  2nd parameter and returns with it in U. 
GetTwoParm:
	GoSub	Get1stParm		;fetch 1st parameter
	mov		temp,U			;save in temp
	GoSub	popU			;fetch 2nd parameter
	ret
;
;//////////RUN COMMAND/////////////////
runcommand:			
	clr		YH		;Initialize PC position inidcator
	ldi 	YL,pmemtop	
checkandrunline:
	ldi		dstackpointer,dstacktop	;Initialize data stack pointer
					;Parameters may be passed on the stack within a line
					;but not between lines.
checkterminalhalt:		;Halt if control-C terminal received 
#if defined(__AT90S2313__)
	sbis	USR,RXC		;if byte received, from termnial, get it
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	sbis	UCSRA,RXC		;if byte received, from termnial, get it
#endif
	Goto	notermhalt
	in		temp,udr	;Read byte
	cpi		temp,$03	;Is it control-C?
	brne	notermhalt
	ori		XH,$02		;set flag to halt program.
notermhalt:
	sbrs	XL,$05		;FOR-NEXT management. Capture F-N return adddress
	Goto	nocaploopad
	mov		loopreturnL,YL	;Copy return address (in interpreter p. space) to registers
	mov		loopreturnH,YH
	andi	XL,$DF		;Turn off flag -address captured.
nocaploopad:
	sbrs	XL,$04		;FOR-NEXT management. If XL,4 is set, jam in new address
	Goto	noaddressjam
	mov		YH,loopreturnH 
	mov		YL,loopreturnL
	andi	XL,$EF		;Flip jaming flag off
noaddressjam:
	sbrs	XL,$06		;GOSUB-RETURN management: Capture GOSUB return address
	Goto	nocapgosubret
	mov		gosubretL,YL
	mov		gosubretH,YH
	andi	XL,$BF		;Flip flag off
nocapgosubret:
	sbrs	XL,$07		;GOSUB-RETURN management:jam destination address
	Goto	nodestaddjam
	mov		YH,ZH 
	mov		YL,ZL
	ori		XH,$04		;Set flag indicating that gosub is active
	andi	XL,$7F		;Flip flag off
nodestaddjam:
	sbrs	XH,$00		;GOSUB-RETURN management: Jamp return address
	Goto	noretjam
	mov		YH,gosubretH 
	mov		YL,gosubretL
	andi	XH,$FA		;Flip flags off (including gosub active flag)

noretjam:
	sbrs	XH,$01		;Halt flag -if set, halt
	Goto	nostop
	Copy16	YH,YL,PChigh,PClow
;	movw	YH:YL,PChigh:PClow
	andi	XH,$FD		;Clear the halt flag

nostop:
	cp		PClow,YL	;See if the end of the buffer has been reached
	brpl	endofprogram

	ld		temp,Y		;Get char from buffer into temp
	GoSub	Qcast		;See what type of char it is.
	
	sbrs	XL,$01		;If XL bit 1 is clear, skip this line- move cursor
	Goto	noskipline	;to next carriage return
	cpi		ecode,$04
	breq	noskipline
	dec		YL	
	Goto	checkandrunline
	
noskipline:
	andi	XL,$FD		;Make sure that skip line flag is cleared for next time around
	; **** does clearing this flag now actuall do anyting?
;	Code below uses Qcast call several lines above.
	cpi		ecode,$02	;See if its a numeral
	brne	notonlinenumber	
	dec		YL		;If numeral, move pointer past it
	Goto	checkandrunline
notonlinenumber:

;	cp	PClow,YL	;See if the end of the buffer has been reached
; moved	brpl	endofprogram
	GoSub	interpretline			
	Goto 	checkandrunline
endofprogram:
;	cbi	ucr,rxen	;Clear UART receive enable
	ret

;=================================================================
newprogramgreet:
	GoSub	TypeGreeting		
newprogram:	;Ready program space and interpreter for new program
	clr		PChigh		;Initialize PC position inidcator
	ldi 	temp,pmemtop
	mov		PClow,temp
	Copy16	ZH,ZL,PChigh,PClow	;Initialize program memory pointer 
								; to top of program memory
newprogram1:
	ldi		dstackpointer,dstacktop	;Initialize data stack pointer
	clr		XL			;Clear interpreter mode flags
	clr		XH	
	
	clr		temp
doanotherone:
;	cpi	ZL,pmembot	; **** any real reason to do this?
;	breq	endofpmem1
;	st	Z,temp		;fill memory with nulls ($00)
;	dec	ZL
;	Goto	doanotherone
endofpmem1:
	ret
;
;*****************************************************************
; Pushes the contents of U onto datastack and returns.
;	DOES NOT CLOBBER Y-Ptr
;*****************************************************************
PushURet:
	GoSub	pushU				;fetch data from dstack
	ret
;
;//////////BEGIN COMMANDS//////////

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
		
	.db	"PRI "		;PRINT
	.dw	printcommand

	.db	"PRX "		;PRX -Print in hexidecimal
	.dw	printhexcommand

	.db	"PRB "		;PRB -Print in binary
	.dw	prbcommand

.if PWM	
	.db	"PWM8"		;PWM8  send value to 8 bit pwm channel, set DDRB,3 to 1
	.dw	pwm8command


	.db	"PWE "		;PWE  send value to 10 bit pwm channel, set DDRB,3 to 1
	.dw	pwecommand


	.db	"PWO "		;PWO   turn off pulse-width modulation. Do not change DDRB
	.dw	pwocommand
.endif

	.db	"EMIT"		;EMIT -send byte on stack as ascii to terminal. 
	.dw	emitcommand	

	.db	$50,$1A,$1D,$20	; :=
	.dw	setequalscommand
	
	.db	$50,$50,$1D,$20	;  =
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
	
	.db	$50,$4F,$52,$20	;OR -Logical OR
	.dw	orcommand
	
	.db	"EOR "
	.dw	eorcommand
	
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
		
.if RESET
	.db "RST "          ;Reset via watchdog timer
	.dw	RSTcommand
.endif

.if DELAY
	.db	"DELA"          ;DELAY N
	.dw	delaycommand
.endif

.if AComp
	.db	"ACO "		;ACO -Put analog comparitor output on the stack
	.dw	acocommand
.endif
		
	.db	"SAVE"		;SAVE	-Save contents of variable space and program memory to EEPROM
	.dw	savecommand

	.db	"LOAD"		;LOAD	-Load variable space and program memory from EEPROM
	.dw	loadcommand
		
	.db	"PEEK"		;PEEK -return data pointed to by parameter
	.dw	peekcommand
	
	.db	"POKE"		;POKE - "POKE data, address"
	.dw	pokecommand
		
.ifdef PORTB
	.db	"OPB "		;OPB -Out Port B
	.dw	outpbcommand
		
	.db	"ODB "		;ODB -Out Data Direction B
	.dw	outdbcommand
	
	.db	"INB "		;INB -Input data from port B
	.dw	inbcommand	
		
	.db	"SBB "		;SBD -Set bit in B register
	.dw	setbitinB
		
	.db	"CBB "		;CBD -Clear bit in B register
	.dw	clearbitinB
.endif
				
.ifdef PORTD
	.db	"OPD "		;OPD -Out Port B
	.dw	outpdcommand
		
	.db	"ODD "		;ODB -Out Data Direction BD
	.dw	outddcommand	
	
	.db	"IND "		;IND -Input data from port D
	.dw	indcommand	

	.db	"SBD "		;SBD -Set bit in D register
	.dw	setbitinD
		
	.db	"CBD "		;CBD -Clear bit in D register
	.dw	clearbitinD
.endif					
	.dw	0			;end of command list

;*************************************************************************************
#if ( (FCLK/10000000) == 0 )
  #define CLOCKRATE	$20,(FCLK/1000000)+$30
#else
  #define CLOCKRATE	(FCLK/10000000)+$30,((FCLK/1000000)%10)+$30
#endif

signonmessage:
#if defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	.db	"AttoBASIC for ATtiny2313 @",CLOCKRATE,"MHz V:", \
		(VER/100)+$30,'.',\
		((VER%100)/10)+$30, \
		((VER%100)%10)+$30,CR,LF
	.db	"(C)2002 R. Cappels projects@cappels.org ",CR,LF
	.db	"Mods (C)2016 S. Vitale ",CR,LF,NULL
#elif defined(__AT90S2313__)
	.db	"AttoBASIC for AT90S2313 @",CLOCKRATE,"MHz V:", \
		(VER/100)+$30,'.',\
		((VER%100)/10)+$30, \
		((VER%100)%10)+$30," ",CR,LF
	.db	"(C)2002 R. Cappels projects@cappels.org ",CR,LF
	.db	"Mods (C)2016 S. Vitale ksv_prj@gmx.com ",CR,LF,NULL
#endif

errormessage:
	.db	"Error ",NULL,NULL

freeword:
	.db	" Free",NULL

;*** end of Program *********************
