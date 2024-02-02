;sketchbook/hardware/tiny/bootloaders/optiboot/optiboot_attiny85.hex: file format ihex
; Optiboot 4.4: SOFT_UART,LED_DATA_FLASH=3,VIRTUAL_BOOT_PARTITION,LED=2(PB2),TX=4(PB4),RX=3(PB3),DBAUD_RATE=9600,F_CPU=1MHz
;------1------2-------3-------4-------5-------6-------7-------8-------9-------A-------B-------C-------D-------E-------F-------0
.nolist
#define SMALLUART 0
#define __SFR_OFFSET 0x0
#include <avr/io.h>
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
 #error Unachievable baud rate (too slow) BAUD_RATE
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
 #error Unachievable baud rate (too fast) BAUD_RATE
#endif // baud rate fast check

.list
.section .version
optiboot_version: .word 0x0406 ; version 4.6 due to bugfixes by Scott Vitale
;
;--------------------------------------------------------------------
;.section .reset,"ax",@progbits ;.orig 0x0000
;	rjmp	Optiboot
;
;--------------------------------------------------------------------
.section .text ;.orig 0x1E00
Optiboot:
; Adaboot nowait mod
	clr	r1		;Set R1 to 0x00: XOR R1: R1 <- 0x00. Just zero register, could be handy
	in	r24, MCUSR	;Store Control Register to R24: R24 <- MCUCR
	out	MCUSR, r1	;Reset Control Register to 0x0: R01 -> MCUCR
	sbrs	r24, 1		;Check EXTRF bit of MCUSR was set
	rjmp	appStart	; If ISC is off - we exit: appStart()
;Adaboot nowait end
;
#ifndef UART_DDR
 #define UART_DDR	DDRB
#endif
#ifndef UART_Port
 #define UART_Port	PORTB
#endif

;Set GPIO pins
#ifndef LED
 #define LED		2
#endif
#ifndef UART_Tx
 #define UART_Tx	1
#endif
#ifndef UART_Rx
 #define UART_Rx	0
#endif

	sbi	UART_DDR, UART_Tx	;Set IO Bit: Set DDBx in DDRB: Pin PBx to Output  - TX pin
	sbi	UART_Port, UART_Tx	;Set IO Bit: Set PORTBx in PORTB: Pin PBx to High - TX pi
;
;Make friendly start blink - prepare timer for blink rate and wd for start timeout
#if LED_START_FLASHES > 0
flashLed:
	sbi	UART_DDR, LED	;Set IO Bit: Set DDBx in DDRB: Pin PBx to Output - BLINK pin
 #if F_CPU > 4000000L
  #define PSK1V 0x0C
  #define T1PSK 2048
 #else
  #define PSK1V 0x0B
  #define T1PSK 1024
 #endif // F_CPU
	ldi	r24, PSK1V	;Load prescaler bits to R24 - T1 Prescale CK/T1PSK
	out	TCCR1, r24	;Set TCCR1 - T1 prescaler from R24
 #define BLINKS LED_START_FLASHES*2
	ldi	r24, BLINKS	;Load number of blinks to R24 : Load number of start blinks to param
;flashLed:
 #if (F_CPU/T1PSK/4) < 128
  #define TMRMND -(F_CPU/T1PSK/4)
	ldi	r18, TMRMND	;Load timer remainder to R18 : -(F_CPU/(1024*16))
 #endif
	ldi	r25, 0x04	;Load 0x04 to R25 (00000100)
_lT1Blink:
	out	TIFR, r25	;Clear timer flags and set TOV1 Timer1 Overflow : Sync/reset clocks
 #ifdef TMRMND
	out	TCNT1, r18	;Set TCNT1 to R18
 #else
	nop			;We need to skip one cycle anyway for timer sync or we're screwd
 #endif // TMRMND
_lT1Wait:
 #if BLINKS > 6 // If we're going to greet for a long time - do rx checks so we won't skip some data
	sbis	PINB, UART_Rx	; if RX Pin is Low
	rjmp	initVars	;  data is pouring, go handle it
 #endif // BLINKS > 6
	in	r0, TIFR	; Read TIFR to R00
	sbrs	r0, TOV1	; Skip if TOV1 is set
	rjmp	_lT1Wait	;Loop while timer overflows
	wdr			;Reset Watchdog
	sbi	PINB, LED	;Set IO Bit: Set PINB2 of PINB: Toggle PB2 level - blink
	dec	r24		;Decrement R24 loop counter
	brne	_lT1Blink	;Until(Z) lT1Blink
	ret
#endif // LED_START_FLASHES > 0
;
#define ADD_L	r14
#define ADD_H	r15
initVars:
	eor	ADD_L, ADD_L	;XOR R14 : LoadAddrH
	eor	ADD_H, ADD_H	;XOR R15 : LoadAddrL
	ldi	r21, 0x03	;Load 0x3 to R21 : SPMCSR bits PGERS & SPMEN
	ldi	r20, 0x01	;Load 0x1 to R20 : SPMCSR bit SPMEN

	ldi	r24, 0x0F	;Set WD Params: Load 00001111 to R24 : Set WDE(WDEnab), Prescale to 256K (2s)
	rcall	watchdogConfig	; Call watchdogConfig

#ifndef BOOT_START
 #error BOOT_START must be defined to section start
 #define BOOT_START 0x1E00
#endif
#define PC_END ((( FLASHEND + 1 )/2) - 1) // Convert flashend from bytes to words - Pgm-space for PC
#define BOOT_SHIFT ( PC_END - ( FLASHEND - BOOT_START )/2 - 1) // Get rjmp shift from RST to optiboot
#define RJMP_OPH ( BOOT_SHIFT / 0x100 + 0xC0 )	// Build MSB - rjmp opcode and shift MSB
#define RJMP_OPL ( BOOT_SHIFT % 0x100 )		// Opcode remainder - LSB of the shift
	ldi	r19, RJMP_OPL	;Prepare reset vector opcode to jump to optiboot. That is 0xC (1100) << 4
	mov	r5, r19		;Move R19 -> R5  : and address shift. We jump from 0 so since address wraps
	ldi	r19, RJMP_OPH	;Load MSB to R19 : the address shift will be FFF-<size of section in words+
	mov	r4, r19		;Move R19 -> R4  : +1> - here +1 is to count actual rjmp instruction itself
LoopMain:
	rcall	GetChar		; r24=GetChar() : Read Command Byte
_chkStkGetParam:
	cpi	r24, 0x41	;Compare R24 with 0x41 (65, 'A' STK_GET_PARAM)
	brne	_chkStkSetDev	;Goto next check if notZ (NotEqual)
	rcall	GetChar		; r24=GetChar(): Read Parameter Byte
	mov	r17, r24	;Store param to R17
	rcall	verifySpace	; verifySpace
	cpi	r17, 0x82	;Compare param with 0x82(130) - MINVER
	breq	__sendMinVer	;If Z(Eq) GoTo sendMinVer
	cpi	r17, 0x81	;Compare param with 0x81(129) - MAJVER
	brne	__sendGenVer	;If notZ(Neq) Goto sendGenVer
__sendMinVer:
	ldi	r24, 0x04	;Load MinVer(4) to R24 for putch. Well, and MajVer as well
	rjmp	__sendParam	;Goto sendParam
__sendGenVer:
	ldi	r24, 0x03	;Load GenVer(3) to R24 for putch - generic version reply
__sendParam:
	rcall	PutChar		; PutChar(r24) Send R24 over Tx
	rjmp	endLoop		;Goto endLoop
_chkStkSetDev:
	cpi	r24, 0x42	;Compare R24 with 0x42(66, 'B', STK_SET_DEVICE)
	brne	_chkStkSetDevExt;Goto next check if notZ (Neq)
	ldi	r24, 0x14	;Load param to R24 (20) to skip next 20 chars
	rjmp	__stkSkipInput	;Goto stkSkipInput // Ignore the data
_chkStkSetDevExt:
	cpi	r24, 0x45	;Compare R24 with 0x45 (69, STK_SET_DEVICE_EXT)
	brne	_chkStkLoadAddr	;Goto next check if NotZ(Neq)
	ldi	r24, 0x05	;Again load param to R24 (5) to skip next 5 chars
__stkSkipInput:
	rcall	getNch		; getNchar skip xR24 chars
	rjmp	endLoop		;Goto EndLoop
_chkStkLoadAddr:
	cpi	r24, 0x55	;Compare R24 with 0x55 (85, U, STK_LOAD_ADDRESS)
	brne	_chkStkUnivrsl	;Goto next check if notZ(Neq)
	rcall	GetChar		; r24=GetChar()
	mov	r10, r24	;Load ADDR_LOW to R10
	rcall	GetChar		; r24=GetChar
	mov	r11, r24	;Load ADDR_HIGH to R11
	add	r10, r10	;Convert from word address to byte address (eg. *2)
	adc	r11, r11	;same for high byte but with carry bit from previous operation
	movw	ADD_L, r10	;Store resulting address bytes to R15:R14 from R11:R10
	rjmp	_chkStkElse	;Goto chkStkElse for verifySpace
_chkStkUnivrsl:
	cpi	r24, 0x56	;Compare R24 with 0x56 (86, V, STK_UNIVERSAL)
	brne	_chkStkProgPage	;Skip to next check if notZ(Neq)
	ldi	r24, 0x04	;Load param to R24 (4) to skip next 4 chars
	rcall	getNch		; getNch(r24) skip 4 bytes
	ldi	r24, 0x00	;Load param to R24 (0) to reply 0 over Tx
	rjmp	__sendParam	;Goto sendParam above to send the data
_chkStkProgPage:
	cpi	r24, 0x64	;Compare R24 with 0x64 (100, d, STK_PROG_PAGE)
	breq	.+2		;Skip line if Z(Eq) to handle the input
	rjmp	_chkStkReadPage	;Goto next check otherwise
	rcall	GetChar		; GetChar() skip length MSB
	rcall	GetChar		; r24=GetChar()
	mov	r17, r24	;Store length LSB to R17
	rcall	GetChar		; GetChar() skip MEMTYPE

#ifndef RAMSTART
 #error RAMSTART Must be defined
 #define RAMSTART    0x0060
#endif
#define RAMSTH	(RAMSTART / 0x100)
#define RAMSTL	(RAMSTART % 0x100)
	ldi	r28, RAMSTL	;Initialize indirect memory pointer: YL
	ldi	r29, RAMSTH	;And YH. Resulting buffer pointer is at RAMSTART
__lReadPage:
	rcall	GetChar		; r24=GetChar
	st	Y+, r24		;Store received byte to (Y), increment pointer
	dec	r17		;decrease length
	brne	__lReadPage	;Until(Z) lReadPage
;__EndLoop
	rcall	verifySpace	; verifySpace
__boot_page_erase_short:
	movw	r30, ADD_L	;Initialize Pointer Z: R31:R30 from R15:R14 - should keep LoadAddr
	out	SPMCSR, r21	;Set SPMCSR - (Store Program Memory Control/Status Register) to 00000011
	spm			;Erase Program Memory at (Z) - (R31:R30) - LoadAddr
__lSPMBusyWait:
	in	r0, SPMCSR	;Load SPMCSR to R0
	sbrc	r0, 0		;Skip if bit 0 (SPMEN) is 0
	rjmp	__lSPMBusyWait	;Loop lSPMBusyWait Until SPM completes
;__EndLoop

#define PBENDH	(RAMSTART + SPM_PAGESIZE) / 0x100
#define PBENDL	(RAMSTART + SPM_PAGESIZE) % 0x100

#ifdef VIRTUAL_BOOT_PARTITION
	cp	ADD_L, r1	;Compare R14 (LoadAddrL) and R1 (zero)
	cpc	ADD_H, r1	;Compare with zero and Carry with R15 (LoadAddrH) and if notZ(Neq) -
	brne	__LoadPage	;Goto LoadPage. Otherwise (0x0000) patch first page saving reset vector
__virtual_boot_partition:
 #ifndef APP_VECT
  #error APP_VECT Must be defined
  #define APP_VECT    0x0C	;default to WDT vector
 #endif
 #define APP_SHFT    RAMSTART+APP_VECT*2
 #define NEW_RSTV    RAMSTART+SPM_PAGESIZE*2+4
 #define NEW_WDVT    RAMSTART+SPM_PAGESIZE*2+6
	lds	r24, RAMSTART	;Load *(RAMSTART) byte to R24 : Store first command (A reset vector opcode)
	lds	r25, RAMSTART+1	; and next byte to R25 : to R25:R24. This is original ResetVector.
	sts	NEW_RSTV+1, r25	;Store it to *(NEW_RSTV) Addresses should be divided by two (bytes to words)
	sts	NEW_RSTV, r24	; and previous byte.
	lds	r8, APP_SHFT	;Load *(WDT) - existing WatchdogVector to R8
	lds	r9, APP_SHFT+1	;  and MSB to R9 : R9:R8 <- (APP_SHFT)
	sts	NEW_WDVT+1, r9	; Store them to New WatchdogVector which is NewResetVector+3
	sts	NEW_WDVT, r8	;  and +2
	sbiw	r24, APP_VECT	; Just decrease the rjmp opcode word by WDT - it shouldn't be above 0x40
	sts	APP_SHFT, r24	;  and store results at Watchdog reset vector - low
	sts	APP_SHFT+1, r25	;  and high bytes
	sts	RAMSTART, r5	; Replace first command (i.e. 0x00 - reset vector)
	sts	RAMSTART+1, r4	; with rjmp optiboot opcode
#endif // VIRTUAL_BOOT_PARTITION
__LoadPage:
	movw	r24, ADD_L	;Phew, Store R15:R14 (LoadAddr) to R25:R24
	ldi	r26, RAMSTL	;Load RAMSTART LSB to R26 (XL)
	ldi	r27, RAMSTH	;Load RAMSTART MSB to R27 (XH)
__lFillPgPage:
	ld	r6, X+		;Load (R26:R27) - *X - to R6 and increment X
	ld	r7, X+		;Load *(R26:R27) - *(X) - to R7 and increment X
	movw	r30, r24	;Store R25:R24 (LoadAddr) to R31:R30 (Z)
	movw	r0, r6		;Store R7:R6 (*Addr) to R1:R0 - instruction word
	out	SPMCSR, r20	;Set SPMCSR to R2(00000001) - clear all but set SPMEN
	spm			;Start Program Memory (R1:R0 --> (Z))
	adiw	r24, 0x02	;R25:R24 += 2 - Increment LoadAddr by word
	ldi	r31, PBENDH	;Load buffer end msb to R31
	cpi	r26, PBENDL	;Compare buffer end lsb to R26 (XL)
	cpc	r27, r31	; and R27 (XH) with R31 and Carry (1+C)
	brne	__lFillPgPage	;Until(Z) lFillPgPage
;__EndLoop
	eor	r1, r1		;CLR R1 : Return back our zero register to zero state
	ldi	r18, 0x05	;Load 00000101 to R18
	movw	r30, ADD_L	;Move R15:R14 to R31:R30 Reset Z to LoadAddr
	out	SPMCSR, r18	;Set SPMCSR to R18 - Clear all but set PGWRT,SPMEN
	spm			;Write filled page from pg buffer to flash
__lSpmBusyWait2:
	in	r0, SPMCSR	;Read SPMCSR
	sbrc	r0, 0		;Skip if SPMEN is clear
	rjmp	__lSpmBusyWait2	;Until(Z) lSpmBusyWait2
	rjmp	endLoop		;Goto EndLoop
_chkStkReadPage:
	cpi	r24, 0x74	;Compare R24 with 0x74 (116, t, STK_READ_PAGE)
	brne	_chkStkReadSign	;else if notZ(Neq) Skip to next check
	rcall	GetChar		; GetChar Skip byte - High (never sent)
	rcall	GetChar		; r24=GetChar() Read length (must be <=FF)
	mov	r17, r24	;Store length to R17
	rcall	GetChar		; GetChar Skip byte - MemType
	rcall	verifySpace	; verifySpace - sync
	movw	r30, ADD_L	;Load Z from R15:R14 (LoadAddr)
__lReadAndSend:
#ifdef VIRTUAL_BOOT_PARTITION
	sbiw	r30, 0x00	;Compare Word Z with 0x0000 (Z==0) - Original ResetVectorL
	brne	.+6		;Skip 3 lines if notZ(Neq) - next address condition
	lds	r24, NEW_RSTV	; Load NewResetVectorL *(0x0184) to R24 - param for send.
	rjmp	__StepAndSend	;Goto StepAndSend
	cpi	r30, 0x01	;Compare R30 (ZL) and 1
	cpc	r31, r1		; and R31 (ZH) with zero and carry - Original ResetVectorH
	brne	.+6		;Skip 3 lines if notZ(Neq) - next address condition
	lds	r24, NEW_RSTV+1	; Load NewResetVectorH to R24 - param for send
	rjmp	__StepAndSend	;Goto StepAndSend
	cpi	r30, APP_VECT*2	;Compare R30 (ZL) with 2x OriginalWdVectorL
	cpc	r31, r1		; and R31 (ZH) with R1 (zero)
	brne	.+6		;Skip 3 lines if notZ(Neq) - next address condition
	lds	r24, NEW_WDVT	; Load NewWdVectorL to R24 - param for send
	rjmp	__StepAndSend	;Goto StepAndSend
	cpi	r30, APP_VECT*2+1;Compare R30 (ZL) original WatchdogVectorH
	cpc	r31, r1		; and R31 (ZH) with R1 (zero)
	brne	.+8		;Skip 4 lines if notZ(Neq) - default case
	lds	r24, NEW_WDVT+1	; Load NewWdVectorH to R24 - param for send
__StepAndSend:
	adiw	r30, 0x01	; Step
	rjmp	__SendAndLoop	;Goto StepAndSend
#endif // VIRTUAL_BOOT_PARTITION
	lpm	r24, Z+		;LoadProgMemory from Z++ to R24 - param for send
__SendAndLoop:
	rcall	PutChar		; PutChar(r24) Send R24 over Tx
	dec	r17		;Decrement R17 (length)
	brne	__lReadAndSend	;Until(Z) lReadAndSend
; STK_LOAD_ADDRESS must always be used prior to Cmnd_STK_PROG_PAGE or Cmnd_STK_READ_PAGE (2525B–AVR–04/03)
	rjmp	endLoop		;Goto EndLoop
_chkStkReadSign:
	cpi	r24, 0x75	;Compare R24 with 0x74 (117, u, STK_READ_SIGN)
	brne	_chkStkLvPgMode	;Skip to next check if notZ(Neq)
	rcall	verifySpace	; verifySpace
	ldi	r24, SIGNATURE_0;Load first signature octet: 0x1E(30)
	rcall	PutChar		; PutChar(r24) and send it
	ldi	r24, SIGNATURE_1;Load second signature octet: 0x93(147)
	rcall	PutChar		; PutChar(r24) and send it
	ldi	r24, SIGNATURE_2;Load third signature octet: 0x0B(11)
	rjmp	__sendParam	; send param from above jumping to endLoop
_chkStkLvPgMode:
	cpi	r24, 0x51	;Compare R24 with 0x51 (81, Q, STK_LEAVE_PROG_MODE)
	brne	_chkStkElse	;bail to default case if notZ(Neq)
	ldi	r24, 0x08	;Load 00001000 to R24 param to reset WD prescaler
	rcall	watchdogConfig	; watchdogConfig (r24)
_chkStkElse:
	rcall	verifySpace	; verifySpace
endLoop:
	ldi	r24, 0x10	;Load 0x10(16, STK_OK) to R24
	rcall	PutChar		; PutChar(r24) Send param over Tx
	rjmp	LoopMain	;Goto LoopMain

;-----------------------------------------------------------------------
; AVR305 - http://www.atmel.com/Images/doc0952.pdf
#ifdef HIGH_PRECISION // I'd counter my _precision_ statement though, no one counted call/ret overhead here
 #define UART_B_VAL (((F_CPU/BAUD_RATE/2)-20)/6)+1	// Old version delay - shorter, more precise, longer code
#else
 #define UART_B_VAL (((F_CPU/BAUD_RATE)-20)/6)		// New version delay - longer, less precise, shorter code
#endif
#if UART_B_VAL > 255
 #error Baud rate too slow for soft UART
#endif
#define delayArg r25
;
uartDelay:
	ldi	delayArg, UART_B_VAL	;Calibrated Delay Loop: 67 or 135? times for ATt85/8MHz/IOSC
_lUartDelay:
	dec	delayArg		; Decrement R25
	brne	_lUartDelay		;Until(Z) lUartDelay
	ret
;
;-----------------------------------------------------------------------
verifySpace:
	rcall	GetChar			; r24=GetChar()
	cpi	r24, 0x20		;is SPACE (0x20 ' ')?
	breq	.+2			;Skip line if Z(equal)
	rjmp	.-2			; make somersault - endless loop: MCU will be reset by WD (in 2s)
	ldi	r24, 0x14		;Load 0x14 to R24 : char to put (STK_INSYNC)
	;rjmp	PutChar			;continue at PutChar(R24) and return with its ret back to main loop
;
;-----------------------------------------------------------------------
#define delayArgTxVal 18
#define bitcnt r18
#define rSOURCE r24
#if SMALLUART
; transmit byte contained in rSOURCE - 12 instructions
PutChar:
TxByte:
	sbi UART_DDR, UART_Tx			; set Tx line to output
	ldi bitcnt, 10					; 1 start + 8 bit + 1 stop
	com rSOURCE						; invert and set carry
TxLoop:
	; 9 cycle loop
	brcc tx1
	cbi UART_Port, UART_Tx 			; transmit a 0
tx1:
	brcs TxDone
	sbi UART_Port, UART_Tx 			; transmit a 1
TxDone:
	ldi delayArg, delayArgTxVal		; 3 * 18 - 1 = 53 cycles
	; ldi delayArg, 41 for 16Mhz
	rcall Delay3Cycle
	lsr rSOURCE
	dec bitcnt
	brne TxLoop
	; fall into delay for stop bit = 1 = idle state

; delay (3 cycle * delayArg) -1 + 4 cycles (ret instruction)
; also clears carry (subi instead of dec) to save 1 instr in RxBit
Delay3Cycle:
	subi delayArg, 1
	brne Delay3Cycle
	ret
;
#else
/*====== Original Code ========================================================*/
PutChar:
	ldi	bitcnt, 0x0A		;Load bitcount to R18: 10
	com	rSOURCE			;Invert rSOURCE
	sec				;Set Carry C bit - start bit
_lTx:
	brcc	_raiseTx		;If Carry is 0 then Goto RaiseTx
	cbi	UART_Port, UART_Tx	;Clear IO Bit: Clear UART_Tx of PORTB: Lower Tx
	rjmp	_keepTx			;GoTo KeepTx
_raiseTx:
	sbi	UART_Port, UART_Tx	;Set IO Bit: Set PORTB4 of PORTB: Raise Tx
	nop				;Align Low/High timing
_keepTx:
	rcall	uartDelay		; uartDelay
	rcall	uartDelay		; uartDelay
#ifdef HIGH_PRECISION
	rcall	uartDelay		; uartDelay
	rcall	uartDelay		; uartDelay
#endif
	lsr	rSOURCE			;Shift right R24 one bit, LSB to Carry
	dec	bitcnt			;Decrement R18 bitcounter
	brne	_lTx			;Until(Z) LoopTx
	ret
#endif
;
;-----------------------------------------------------------------------
#define delayArgRxVal 19
#define rDEST r24
#if SMALLUART
; receive byte into rDEST - 11 instructions
GetChar:
RxByte:
	ldi rDEST, 0x80					; bit shift counter
WaitStart:
	sbic UART_Port-2, UART_Rx		; wait for start edge
	rjmp WaitStart
;	ldi delayArg, delayArgRxVal		; 1.5 bit delay
	ldi delayArg, 32			; 1.5 bit delay
	;ldi delayArg, 67 for 16Mhz
RxBit:
	rcall Delay3Cycle				; delay and clear carry
	; 6 cycle loop
	; adiw delayArg, 19 clears carry
	ldi delayArg, 19				; 3 * 19 - 1 = 56 cycles
	;ldi delayArg, 40 for 16Mhz
	sbic UART_Port-2, UART_Rx
	sec
	ror rDEST
	brcc RxBit
	; fall into UartInit
;
#else
/*====== Original Code ========================================================*/
#define rDEST r24
;
GetChar:
#if LED_START_FLASHES > 0
	sbi	PINB, LED		;Set IO Bit: PINled of PINB: Toggle LED - turn on
#endif
	ldi	bitcnt, 0x09		;Load bit counter to R19 : 8+1stop
	eor	rDEST, rDEST		;initialize return data
_lWaitRx:
	sbic	UART_Port-2, UART_Rx	;Skip if PINrx of PINB is Clear: break if RX is low
	rjmp	_lWaitRx	 	; Until Rx Start Bit lWaitRx
	rcall	uartDelay		; uartDelay // Eat start bit
_lRx:
#ifdef HIGH_PRECISION
	rcall	uartDelay		; uartDelay // Position
	rcall	uartDelay		; uartDelay // to the bit
#endif
	rcall	uartDelay		; uartDelay // far edge
	rcall	uartDelay		; uartDelay // and feel the beat
	clc				;Clear Carry C Bit
	sbic	UART_Port-2, UART_Rx	;Skip if PINrx of PINB is Clear: Skip if data is 0
	sec				;Set Carry C Bit otherwise
	dec	bitcnt			;Decrement bit counter
	breq	_endlRx			;Break if Zero : Should be on 9th stop bit to break
	ror	rDEST			;Shift Right R24 through Carry: Fill in incoming bits
	rjmp	_lRx			;Loop for next Rx Bit
_endlRx:
	wdr				;Reset Watchdog
#if LED_START_FLASHES > 0
	sbi	PINB, LED		;Set IO Bit: Set PINled of PINB: Toggle LED - turn off
#endif
	ret				;return with result in R24
#endif
;
;-----------------------------------------------------------------------
getNch:
	mov	r16, r24	;move R24 to R16
_lGetNch:
	rcall	GetChar		; r24=GetChar
	subi	r16, 0x01	;Decrement R16
	brne	_lGetNch	;Until(Z) lGetNch
	rcall	verifySpace	; verifySpace
	ret
;
;-----------------------------------------------------------------------
watchdogConfig:
	ldi	r25, 0x18	;Reset Watchdog
	out	WDTCR, r25	;WDTCR <- 00011000
	out	WDTCR, r24	;WDTCR <- R24
	ret
;
;-----------------------------------------------------------------------
appStart:
#message Using vector APP_VECT for application start
	clr	r24		; disabe WD
	rcall	watchdogConfig	; Reset WD
;	ldi	r28,0x07	;"magic key" in YH:L
;	ldi	r29,0xb0
	ldi	r30, APP_VECT	; Set ZL R30 <- ijmp address in Vector table
	clr	r31		; Set ZH R31 <- 0x00 (XOR R31)
	ijmp			; Jump to RESET vector (to jump further to code)

; vim: ts=8 noet ft=aasm:
