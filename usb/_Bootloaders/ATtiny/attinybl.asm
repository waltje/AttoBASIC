;ATTiny Bootloader
; - This is my port of the t2313bl v0.8 bootloader by Arne Rossius to the attiny25/45/85
; - It uses a modified version of the AVR305 Software UART
; - I've added support for the AVR910 blockmode commands. This makes the bootloader bigger but faster.
;
;Features:
; - automatic switching to main program if nothing is received within 1 second
; - transparent to the application
; - located at the end of the flash memory
; - automatic modification of the application's Reset Vector while uploading
; - AVR910 compatible protocol (see below for supported commands)
;
;Requirements:
; - ATTiny25/45/85 running at any frequency which can provide a valid BAUD rate
; - TTL RS232 interface connected to PB2/RxD and PB3/TxD in the target application
; - The main (user) program MUST have a rjmp command at 0x000
; - To make the bootloader as small as possible it it coded to use with avrdude.
; - The default avrdude.conf file does not include unique avr910_devcode parameter for the
;   attiny25/45/85. If you update avrdude.conf with the DevCode values from avr910_2313_v38c.asm
;   I'm using in this file you will be able to use the avrdude "-t" option for the attiny24/45/85.
;
;Version:
; v1.0 Initial release
; v1.1 Disable the WDT in the initialization section. This needs to be done before jumping to the appln.
;
;
;Programming commands (subset of avr910.asm from Atmel):
;*                                     +-------------+-------------+------+
;*  Commands                           | Host writes |  Host reads |      |
;*  --------                           +-----+-------+-------+-----+      |
;*                                     | ID  | data  |  data |     | Note |
;* +-----------------------------------+-----+-------+-------+-----+------+
;* | Enter programming mode            | 'P' |       |       | 13d |   1  |
;* | Report autoincrement address      | 'a' |       |       | 'Y' |      |
;* | Set address                       | 'A' | ah al |       | 13d |   2  |
;* | Write program memory, low byte    | 'c' |    dd |       | 13d |   3  |
;* | Write program memory, high byte   | 'C' |    dd |       | 13d |   3  |
;* | Issue Page Write                  | 'm' |       |       | 13d |      |
;* | Read program memory               | 'R' |       | dd(dd)|     |   4  |
;* | Write data memory                 | 'D' |    dd |       | 13d |      |
;* | Read data memory                  | 'd' |       |    dd |     |      |
;* | Chip erase (excluding bootloader) | 'e' |       |       | 13d |      |
;* | Leave programming mode            | 'L' |       |       | 13d |   5  |
;* | Select device type (0x1a only)    | 'T' |    dd |       | 13d |   6  |
;* | Return signature bytes (1E9108)   | 's' |       |  3*dd |     |      |
;* | Return supported device code (1a) | 't' |       |  n*dd | 00d |   7  |
;* | Return software ident. "utsrqp"   | 'S' |       |  s[7] |     |   8  |
;* | Return sofware version            | 'V' |       | dd dd |     |   9  |
;* | Return hardware version (0.0)     | 'v' |       | dd dd |     |   9  |
;* | Return programmer type ("S")      | 'p' |       |    dd |     |  10  |
;* +-----------------------------------+-----+-------+-------+-----+------+
;* | New Commands since Version 3.5    |     |       |       |     |      |
;* | Implemented Atmel Bootloader commands (Atmel Appl. Note 109)  |      |
;* | Report Block write Mode           | 'b' |       |'Y'2*nn| 13d |  16  |
;* | Block Write                       | 'B' |2*nn'M'|  n*dd | 13d |  16  |
;* | Block Read                        | 'g' |2*nn'M'|  n*dd | 13d |  16  |
;* +-----------------------------------+-----+-------+-------+-----+------+




;**************************************************************************
;*********************                                *********************
;********************* Start of Configuration Section *********************
;*********************                                *********************
;**************************************************************************


;**************************************************************************
;*********************      Choose which attiny       *********************
;**************************************************************************

;.include "tn25def.inc"
;.equ	DevCode = 0x1a ;AVR910 device code for ATTiny25 (from avr910_2313_v38c.asm)

;.include "tn45def.inc"
;.equ	DevCode = 0x1b ;AVR910 device code for ATTiny45 (from avr910_2313_v38c.asm)

.include "tn85def.inc"
.equ	DevCode = 0x1c ;AVR910 device code for ATTiny85 (from avr910_2313_v38c.asm)


;**************************************************************************
;******************* Configure the AVR305 Software UART *******************
;**************************************************************************

;Pin definitions
.equ	RxD	=0	;Receive pin is PB0
.equ	TxD	=1	;Transmit pin is PB1

;Enter the Freq you are running the processor at, the Baud rate and number of stop bits you want
;.equ	Freq = 1000000	;clock frequency
.equ	Freq = 8000000	;clock frequency
;.equ	Baud = 28800	;baud rate
.equ	Baud = 19200	;baud rate
.equ	sb   = 1		;Number of stop bits (1, 2, ...)

; Valid Baud rates for the Internal Oscillator (See AVR305 for more frequencies and their Baud rates)
;     Clock:  1 MHz  2 MHz  4 MHz  8MHz
;   2400 bps  b=66   b=135
;   4800 bps  b=31   b=66   b=135
;   9600 bps  b=14   b=31   b=66   b=135
;  14400 bps  b=8    b=19   b=42   b=89
;  19200 bps  b=5    b=14   b=31   b=66
;  28800 bps  b=2    b=8    b=19   b=42
;  38400 bps         b=5    b=14   b=31
;  57600 bps         b=2    b=8    b=19
; 115200 bps                b=2    b=8
; 230400 bps                       b=2

;The AVR305 "b" value is calculated for you based on the Freq & Baud you choose.
;Freq & Baud must be valid values from AVR305 so that the value for b will fit into an 8-bit register:
;- Baud rates which are too low will error "Operand(s) out of range in 'ld1 r17,0xNNN" and NNN will be > 256
;- Baud rates which are too high will not error as they load a small negative value into b, which won't work
.equ	b	= ((Freq/(Baud/2))-39)/12


;**************************************************************************
;********************* Choose your bootloader features ********************
;**************************************************************************

.equ	BLKMODE = 1
;       BLKMODE = 0      - Only include bytemode cmds for both flash and eeprom -> Smaller
;       BLKMODE = 1      - Only include blockmode commands for both flash and eeprom -> Faster

.equ	EEPROM  = 0
;       EEPROM  = 0      - Don't include the EEPROM cmds -> Smaller again
;       EEPROM  = 1      - Include the EEPROM cmds

.equ	AVRDUDE_Y = 0
;       AVRDUDE_Y = 0    - Avrdude -y & -Y not supported
;       AVRDUDE_Y = 1    - Avrdude -y & -Y supported. Bytemode eeprom cmds included, even when EEPROM=0


;**************************************************************************
;*********************                                *********************
;*********************  End of Configuration Section  *********************
;*********************                                *********************
;**************************************************************************



.equ	VerMaj = 1	;major version number
.equ	VerMin = 0	;minor version number

;You only need to change this if you have changed the bootloader code.
.if   ( PAGESIZE == 16 )			; ATTiny25 (Code size/Bootloader size in 16 word pages)
	.if ( BLKMODE )
		.if ( EEPROM )
			.if ( AVRDUDE_Y )
				.equ	BLSIZE = 17	;(528/544) Blockmode FLASH and Block and bytemode EEPROM cmds
			.else
				.equ	BLSIZE = 16	;(502/512) Blockmode FLASH and EEPROM cmds
			.endif
		.else
			.if ( AVRDUDE_Y )
				.equ	BLSIZE = 16	;(492/512) Blockmode FLASH and bytemode EEPROM cmds
			.else
				.equ	BLSIZE = 15	;(454/480) Blockmode FLASH cmds
			.endif
		.endif
	.else
		.if ( EEPROM ) || ( AVRDUDE_Y )
			.equ	BLSIZE = 13		;(414/416) Bytemode FLASH and EEPROM cmds
		.else
			.equ	BLSIZE = 12		;(376/384) Bytemode FLASH cmds
		.endif
	.endif
.else								; ATTiny45/85 (Code size/Bootloader size in 32 word pages)
	.if ( BLKMODE )
		.if ( EEPROM )
			.if ( AVRDUDE_Y )
				.equ	BLSIZE = 9	;(532/576) Blockmode FLASH and Block and bytemode EEPROM cmds
			.else
				.equ	BLSIZE = 8	;(506/512) Blockmode FLASH and EEPROM cmds
			.endif
		.else
			.if ( AVRDUDE_Y )
				.equ	BLSIZE = 8	;(496/512) Blockmode FLASH and bytemode EEPROM cmds
			.else
				.equ	BLSIZE = 8	;(454/512) Blockmode FLASH cmds
			.endif
		.endif
	.else
		.if ( EEPROM ) || ( AVRDUDE_Y )
			.equ	BLSIZE = 7		;(418/448) Bytemode FLASH and EEPROM cmds
		.else
			.equ	BLSIZE = 6		;(376/384) Bytemode FLASH cmds
		.endif
	.endif
.endif

;Size of the buffer to use as the receive buffer for block write commands
;- Must be an integer multple of the pagesize
.equ BUFSIZE  = SRAM_SIZE - 0x40	; Use all but last 0xNN bytes of internal RAM

.if ( BUFSIZE%PAGESIZE > 0 )
	.error "The block mode buffer must be an integer multiple of the pagesize"
.endif


.equ WATCHDOG_OFF    = 0x00
.equ WATCHDOG_16MS   = (1<<WDE)
.equ WATCHDOG_32MS   = (1<<WDP0) | (1<<WDE)
.equ WATCHDOG_64MS   = (1<<WDP1) | (1<<WDE)
.equ WATCHDOG_125MS  = (1<<WDP1) | (1<<WDP0) | (1<<WDE)
.equ WATCHDOG_250MS  = (1<<WDP2) | (1<<WDE)
.equ WATCHDOG_500MS  = (1<<WDP2) | (1<<WDP0) | (1<<WDE)
.equ WATCHDOG_1S     = (1<<WDP2) | (1<<WDP1) | (1<<WDE)
.equ WATCHDOG_2S     = (1<<WDP2) | (1<<WDP1) | (1<<WDP0) | (1<<WDE)
.equ WATCHDOG_4S     = (1<<WDP3) | (1<<WDE)
.equ WATCHDOG_8S     = (1<<WDP3) | (1<<WDP0) | (1<<WDE)

.def	DataL  = R0				; Data to write to the page buffer (must be R0/R1)
.def	DataH  = R1

.def	Ctr2L  = R2				; Block mode buffer counter backup
.def	Ctr2H  = R3

.def	temp   = R16			; Temporary registers
.def	temp2  = R17			; (Do not use these to save values across a subroutine call)

.def	delay  = R18			; Delay counter for UART_delay & byte counter for writing flash
.def	bitcnt = R19			; Bit counter for UART

.def	Hex00L = R22			; Can save code space with registers setup as a 0x0000
.def	Hex00H = R23			; constant for use in "out" & "movw" commands

.def	InstrL = R24			; Copy of an instruction
.def	InstrH = R25

.def	CtrL   = R26			; 16-bit Counter
.def	CtrH   = R27			; (Used for nbr of bytes in the block cmds)

.def	Addr2L = R28			; Address backup
.def	Addr2H = R29			; (Also used as the Y register for indexed access to RAM)

.def	AddrL  = R30			; flash/EEPROM address
.def	AddrH  = R31			; (Must be the Z register for indexed access to flash ROM)


.cseg
.org 0x000
	rjmp	bl_start
.org 0x001
 progstart:
	; ***************************************************
	; ******* MAIN PROGRAM WILL BE INSERTED HERE! *******
	; ***************************************************

.org (FLASHEND+1)-(BLSIZE*PAGESIZE)
bl_start:
	;Initialization Notes
	;We only get to here after a power on reset so we use the reset defaults wherever possible to save code space
	;- The interrupts are all disabled, so we don't have to worry about them.
	;- We don't care what the EESAVE fuse is set to.
	;- EEARH is undefined on reset but we can save code by not refering to it for the attiny25/45
	;- SPH & SPL are set to RAMEND on reset for ATTiny25/45/85
	;- We use the default value of the clock prescaler. If you alter this then you will need to alter the value
	;  of "Freq" to match your fuse setting.

	ldi	Hex00L, 0x00			; Initialize our register constant 0x0000
	ldi	Hex00H, 0x00

	in temp2,	MCUSR			; Save the reset flags
	out MCUSR,	Hex00L			; and immediately set them all to zero

	ldi temp, 	(1<<WDCE) | (1<<WDE)
	out WDTCR, 	temp			; Must set MCUSR:WDRF to zero before disabling the WDT
	;ldi temp, 	WATCHDOG_OFF
	out WDTCR, 	Hex00L			; Need to disable the watchdog timer before we jump to the app

	sbrc temp2,	WDRF			; If this was a watchdog reset
	rjmp addr0					; then jump straight to the app
	ldi temp, 	WATCHDOG_1S		; else enable the watchdog timer and/or set delay for the bootloader
	out WDTCR, 	temp			; (Don't need to use any special procedure to enable the WDT)

	ldi	temp,0xFF				; PORTB will be initialized as all inputs after a reset
	out	PORTB,temp				; Enable pullups on all input pins and TXD high when it is set as an output

	ldi	temp,(1<<TXD)			; Make TXD an ouput. Leave all other pins as inputs
	out	DDRB,temp

.if ( EEPROM ) || ( AVRDUDE_Y )
	out	EECR, Hex00L			;EEPROM: atomic write (erase + write)
.endif

	; An extra 4 cmds (8 bytes) to change the clock frequency in software for faster downloads
	; NOTE: You may need to increase the BLSIZE to cater for this
	;ldi temp, (1<<CLKPCE)		; Enable clock prescaler change
	;out CLKPR, temp
	;ldi temp, 0x00				; Set CLKPS3:0 = 0000 for division factor =1 (8MHz) for testing
	;ldi temp, 0x01				; Set CLKPS3:0 = 0001 for division factor =2 (4MHz) for testing
	;ldi temp, 0x02				; Set CLKPS3:0 = 0010 for division factor =4 (2MHz) for testing
	;out CLKPR, temp


bl_loop:
	rcall	getc

	cpi	temp, 'A'				;'A' set address
	brne	bl_AA
	rcall	getc				;MSB first
	mov	AddrH, temp
	rcall	getc
	mov	AddrL, temp
	lsl	AddrL					;convert word address to byte address
	rol	AddrH
	rjmp	ret_ok
bl_AA:


.if	( BLKMODE )
	cpi		temp, 'B'			;'B' Block Write (AddrH:AddrL is already set)
	;brne	bl_GG				; block write section is too long for this relative jump
	breq	bl_G0
	rjmp	bl_GG
bl_G0:
	rcall	getc				;Number of bytes being sent: High byte
	mov		CtrH, temp
	rcall	getc				;Number of bytes being sent: Low byte
	mov		CtrL, temp
	rcall	getc				;Type of memory('F'lash or 'E'eprom)
	mov 	temp2, temp

	;Read the block of bytes into the RAM buffer
	ldi		Addr2L, LOW(SRAM_START)	;Addr2H:Addr2L used as Y reg pointing to the RAM buffer
	ldi		Addr2H, HIGH(SRAM_START)
	movw	Ctr2H:Ctr2L, CtrH:CtrL	;Backup CtrH:CtrL (nbr of bytes in this block cmd)
bl_GG1b:
	rcall	getc				;Get a byte
	st 		Y+, temp			;and save it into the RAM buffer at this address
	sbiw 	CtrH:CtrL, 1		;until we've fetched CtrH:CtrH nbr of bytes
   	brne	bl_GG1b

	;Setup Addr2H:Addr2L and CtrH:CtrH for writing the memory (either Flash or EEPROM)
	ldi		Addr2L, LOW(SRAM_START)	;Addr2H:Addr2L used as Y reg pointing to the RAM buffer
	ldi		Addr2H, HIGH(SRAM_START)
	movw	CtrH:CtrL, Ctr2H:Ctr2L	;Restore CtrH:CtrL from backup

	cpi 	temp2,'F'
.if ( EEPROM )
	brne	bl_GG4
.else
	brne	ret_err1
.endif	; ( EEPROM )

	;We are writing the program memory (Flash)
bl_GG2a:
	;Fill the flash page buffer
	ldi		delay, PAGESIZE		;Counts the number of words in the flash temporary buffer
bl_GG2b:
	ld 		DataL,Y+			;Copy the byte in the RAM buffer at Addr2H:Addr2L into DataL
	ld 		DataH,Y+			;Copy the byte in the RAM buffer at Addr2H:Addr2L into DataH

	cpi		AddrL, 0x00			; Test if Addr is 0x0000
	cpc		AddrH, AddrL
	brne	write_flash_1b
	; Addr is 0x0000
	; This is the first cmd for page 0x0000 so we update the page at addr0 here
	; Read in page and exchange the nop at addr0 with the converted user program rjmp
	movw	InstrH:InstrL, DataH:DataL	 ;backup instruction
	andi	InstrH, 0x0F		;convert app's POR rjmp cmd in InstrH:InstrL to rjmp from addr0 instead
	ldi		temp, HIGH(addr0)
	subi	InstrL, LOW(addr0)
	sbc		InstrH, temp
	andi	InstrH, 0x0F
	ori		InstrH, 0xC0
	rcall	firstinstr			;and rewrite the page at addr0 with the cmd in DataH:DataL at addr0
	; Firstinstr() sets up new command for 0x0000: "rjmp bl_start" in DataH:DataL
	; for when we fall through to next section
write_flash_1b:
	; Addr > 0x0000
	ldi		temp, 0x01			;store word in DataH:DataL to the flash page buffer
	rcall	do_spm
	adiw	AddrH:AddrL, 2		;increment address
	sbiw 	CtrH:CtrL, 2		;Decrement our RAM buffer counter for this block by two bytes
   	breq	bl_GG2c				;If we get to the end of this then write the flash page
	dec		delay				;else decrement our flash page buffer counter by 1 word
	brne	bl_GG2b				;looping until we fill the page buffer
bl_GG2c:
	;Write the flash page
	ldi		temp, HIGH(bl_start*2)		;Check if the address is the first page of the bootloader
	cpi		AddrL, LOW(bl_start*2)
	cpc		AddrH, temp
	brsh	ret_err1			;and exit because the app is too large to fit in the available flash
	sbiw	AddrH:AddrL, 2		;Point AddrH:AddrL back to the page we just wrote for call to writepage
	rcall	writepage
	adiw	AddrH:AddrL, 2		;Restore AddrH:AddrL ready for the next call to write_flash

	cpi		CtrL, 0x00			;Test if CtrH:CtrL is 0
	cpc		CtrH, CtrL
	brne	bl_GG2a				;If it isn't then loop back to write another page
	rjmp	ret_ok
bl_GG4:
.if ( EEPROM )
	cpi 	temp2,'E'
	brne	ret_err1
	;We are writing the data memory (EEPROM)
bl_GG5:
	ld 		temp,Y+				;Copy the byte in the RAM buffer at Addr2H:Addr2L into temp
.if ( AVRDUDE_Y )				;Include the EEPROM read/write code as a subroutine
	rcall	EEPROM_write
.else	; ( AVRDUDE_Y )			;Include the EEPROM read/write inline
	out		EEARL, AddrL
.if   ( PAGESIZE == 32 )
	out		EEARH, AddrH		;Don't need to use the extra code to do this for ATtiny25/45
.endif	; ( PAGESIZE == 32 )
	out		EEDR, temp
	sbi		EECR, EEMPE
	sbi		EECR, EEPE
bl_GG6:
	sbic	EECR, EEPE			;wait until EEPROM is ready (i.e. after previous write)
	rjmp	bl_GG6
	adiw	AddrH:AddrL, 1		;increment Addr
.endif	; ( AVRDUDE_Y )
	sbiw 	CtrH:CtrL, 1		;until we've written CtrH:CtrH nbr of bytes
	brne	bl_GG5
	rjmp	ret_ok
.endif	; ( EEPROM )
bl_GG:

.else	; ( BLKMODE )
	cpi	temp, 'c'				;'c' write program memory (buffer), low byte
	brne	bl_c
	rcall	getc
	mov	DataL, temp
	rjmp	ret_ok
bl_c:

	cpi	temp, 'C'				;'C' write program memory (buffer), high byte
	brne	bl_CC
	rcall	getc
	mov		DataH, temp

	cpi		AddrL, 0x00			; Test if Addr is 0x0000
	cpc		AddrH, AddrL
	brne	bl_CC1
	; Addr is 0x0000
	;This is the cmd at Addr 0x0000 so we update the page at addr0
	;read in page, exchange the nop at addr0 with the converted user program rjmp
	movw	InstrH:InstrL, DataH:DataL ;backup instruction
	andi	InstrH, 0x0F		;convert app's POR rjmp cmd in InstrH:InstrL to rjmp from addr0 instead
	ldi		temp, HIGH(addr0)
	subi	InstrL, LOW(addr0)
	sbc		InstrH, temp
	andi	InstrH, 0x0F
	ori		InstrH, 0xC0
	rcall	firstinstr			;and rewrite the page at addr0 with the cmd in InstrH:InstrL at addr0
	; Firstinstr() sets up new command for 0x0000: "rjmp bl_start" in DataH:DataL
	; for when we fall through to next section
bl_CC1:
	; Addr > 0x0000
	ldi		temp, 0x01			;store DataH:DataL in buffer
	rcall	do_spm
	adiw	AddrH:AddrL, 2		;increment address
	rjmp	ret_ok
bl_CC:


	cpi	temp, 'R'				;'R' read program memory
	brne	bl_RR

	cpi		AddrL, 0x00			; Test if Addr is 0x0000
	cpc		AddrH, AddrL
	brne	bl_RR1
	; Address is 0x0000.
	; Convert saved rjmp cmd at address addr0 to run from address 0x0000
	; so that it will match the cmd at 0x0000 in the original hex file
	ldi		AddrL, LOW(addr0*2)
	ldi		AddrH, HIGH(addr0*2)
	lpm		InstrL, Z+
	lpm		temp, Z+
	andi	temp, 0x0F 			;convert rjmp target
	ldi		temp2, HIGH(-addr0)
	subi	InstrL, LOW(-addr0)
	sbc		temp, temp2
	andi	temp, 0x0F
	ori		temp, 0xC0			;need to have high byte in temp at this stage

	ldi		AddrH, 0x00			;Set the next address to 0x0002
	ldi		AddrL, 0x02
	rjmp	bl_RR2
bl_RR1:
	;Address is > 0x0000. Return the actual cmd at this address
	lpm		InstrL, Z+
	lpm		temp, Z+			;return high byte
bl_RR2:
	rcall	putc
	mov		temp, InstrL		;return low byte
	rjmp	ret_put
bl_RR:
.endif	; ( BLKMODE )


	cpi	temp, 'T'				;'T' select device type
	brne	bl_TT				;Seems redundant in a bootloader when we also check the signature bytes
	rcall	getc				;Save some code by fetching the device id
	rjmp	ret_ok				;and returning OK without checking
ret_err1:						;Need to support the blockmode relative jumps to ret_err1, even though not used by 'T' cmd.
	rjmp	ret_err
bl_TT:



.if ( BLKMODE )
	cpi	temp, 'g'				;'g' Block Read (AddrH:AddrL is already set)
	brne	bl_g
	rcall	getc				;Number of bytes being sent: High byte
	mov	CtrH, temp
	rcall	getc				;Number of bytes being sent: Low byte
	mov	CtrL, temp
	rcall	getc				;Type of memory('F'lash or 'E'eprom)

	cpi temp,'F'
.if ( EEPROM )
	brne	bl_g4
.else	; ( EEPROM )
    brne	ret_err1
.endif	; ( EEPROM )
	;We are reading the program memory (Flash)
bl_g1:

	cpi		AddrL, 0x00			; Test if Addr is 0x0000
	cpc		AddrH, AddrL
	brne	bl_g2
	; Address is 0x0000.
	; Convert saved rjmp cmd at address addr0 to run from address 0x0000
	; so that it will match the cmd at 0x0000 in the original hex file
	ldi		AddrL, LOW(addr0*2)
	ldi		AddrH, HIGH(addr0*2)
	lpm		temp, Z+
	lpm		InstrH, Z+
	andi	InstrH, 0x0F 		;convert rjmp target
	ldi		temp2, HIGH(-addr0)
	subi	temp, LOW(-addr0)
	sbc		InstrH, temp2
	andi	InstrH, 0x0F
	ori		InstrH, 0xC0		;need to have low byte in temp at this stage

	ldi		AddrH, 0x00			; Set the next address to 0x0002
	ldi		AddrL, 0x02
	rjmp	bl_g3
bl_g2:
	;Address is > 0x0000. Return the actual cmd at this address
	lpm		temp, Z+
	lpm		InstrH, Z+
bl_g3:
	rcall	putc				;return low byte
	mov		temp, InstrH
	rcall	putc				;return high byte
	sbiw CtrH:CtrL, 2			;Decrement buffer counter by one word
   	brne	bl_g1
	rjmp	bl_loop
bl_g4:
.if ( EEPROM )
	cpi temp,'E'
	brne	ret_err1
	;We are reading the data memory (EEPROM)
bl_g5:
.if ( AVRDUDE_Y )				; Include the EEPROM read/write code as a subroutine
	rcall	EEPROM_read
.else	; ( AVRDUDE_Y )			; Include the EEPROM read/write inline
	out		EEARL, AddrL
.if   ( PAGESIZE == 32 )
	out		EEARH, AddrH		; Don't need to use the extra code to do this for ATtiny25/45
.endif	; ( PAGESIZE )
	sbi		EECR, EERE
	in		temp, EEDR
	adiw	AddrH:AddrL, 1 		;increment Addr
.endif	; ( AVRDUDE_Y )
	rcall	putc
	sbiw CtrH:CtrL, 1			;Decrement buffer counter by one byte
   	brne	bl_g5
	rjmp	bl_loop
.endif	; ( EEPROM )
bl_g:

	cpi	temp, 'b'				;'b' support block mode reads & writes
	brne	bl_b
	ldi	temp, 'Y'				; Y = yes
	rcall	putc
	ldi	temp, HIGH(BUFSIZE)
	rcall	putc
	ldi	temp, LOW(BUFSIZE)
	rjmp	ret_put
bl_b:
.endif	; ( BLKMODE )


.if ( BLKMODE && EEPROM && AVRDUDE_Y )	;Include the EEPROM read/write code as a subroutine
	cpi	temp, 'd'				;'d' read data memory
	brne	bl_d
	rcall 	EEPROM_read
	rjmp	ret_put
bl_d:


	cpi	temp, 'D'				;'D' write data memory
	brne	bl_DD
	rcall	getc				;Get data value
	rcall	EEPROM_write
	rjmp	ret_ok
bl_DD:
.endif	; ( BLKMODE && EEPROM && AVRDUDE_Y )


.if ( EEPROM && !BLKMODE ) || ( AVRDUDE_Y && !EEPROM )	;Include the EEPROM read/write code inline
	cpi	temp, 'd'				;'d' read data memory
	brne	bl_d
	out	EEARL, AddrL
.if   ( PAGESIZE == 32 )
	out		EEARH, AddrH		;Don't need to use the extra code to do this for ATtiny25/45
.endif	; ( PAGESIZE )
	sbi	EECR, EERE
	in	temp, EEDR
	adiw	AddrH:AddrL, 1 		;increment Addr
	rjmp	ret_put
bl_d:


	cpi	temp, 'D'				;'D' write data memory
	brne	bl_DD
	rcall	getc				; Get data value
	out		EEARL, AddrL
.if   ( PAGESIZE == 32 )
	out		EEARH, AddrH		; Don't need to use the extra code to do this for ATtiny25/45
.endif	; ( PAGESIZE )
	out		EEDR, temp
	sbi		EECR, EEMPE
	sbi		EECR, EEPE
bl_DD1:
	sbic	EECR, EEPE			; Wait until EEPROM is ready (i.e. after previous write)
	rjmp	bl_DD1
	adiw	AddrH:AddrL, 1		; Increment Addr
	rjmp	ret_ok
bl_DD:
.endif	; ( EEPROM && !BLKMODE ) || ( AVRDUDE_Y && !EEPROM )


	cpi	temp, 'a'				;'a' report autoincrement address
	brne	bl_a
	ldi	temp, 'Y'
	rjmp	ret_put
bl_a:


	cpi	temp, 's'				;'s' read signature bytes
	brne	bl_s
	ldi	temp, SIGNATURE_002
	rcall	putc
	ldi	temp, SIGNATURE_001
	rcall	putc
	ldi	temp, SIGNATURE_000
	rjmp	ret_put
bl_s:


.if ( !BLKMODE )
	cpi	temp, 'm'				;'m' issue page write
	brne	bl_m
	ldi	temp, HIGH(bl_start*2)	;do not accept adresses where the bootloader resides
	cpi	AddrL, LOW(bl_start*2)
	cpc	AddrH, temp
	brsh	ret_err
	rcall	writepage
	rjmp	ret_ok
bl_m:
.endif	; ( !BLKMODE )


	cpi	temp, 'v'				;'v' return hardware version (redundant for bootloader)
	breq	bl_SS2				; Save code by returning device id, 0x00 as the hardware version


	cpi	temp, 'V'				;'V' return software version
	breq	bl_SS2				; Save code by returning device id, 0x00 as the software version


	cpi	temp, 't'				;'t' read supported device codes (redundant for bootloader)
	breq	bl_SS2				; Required  by avrdude. Save code space by making it the last 2 bytes of software identifier


	cpi	temp, 'S'				;'S' return software identifier
	brne	bl_SS				; Save code by embedding the device list as the last 2 characters in the software identifier
	ldi		temp2,	5			; Write 'S' five times (temp2 is safe to use across a call to getc)
bl_SS1:
	ldi		temp,'S'
	rcall	putc
	dec		temp2
	brne	bl_SS1
bl_SS2:
	ldi	temp, DevCode			; then put our list of supported device codes as the last two characters
	rcall	putc
	ldi	temp, 0x00
	rjmp	ret_put
bl_SS:


	cpi	temp, 'e'				;'E' chip erase
	breq	ret_ok				; writepage() erases every page before writing so chip erase is functionally redundant


	cpi	temp, 'P'				;'P' enter programming mode
	breq	ret_ok				; Redundant for bootloader


	cpi	temp, 'L'				;'L' leave programming mode
	breq	ret_ok				;Gracefully end session with avrdude

	; Save some code size by allowing ret_err to return '?' for the programmer type. Avrdude doesn't mind.


;****************************** end of commands ******************************
; bootloader main loop falls through to ret_err for unknown commands

ret_err:
	ldi	temp, '?'				;unknown command or error, return '?'
	rjmp	ret_put

ret_ok:
	ldi	temp, 13				;everything went right, return CR

ret_put:
	rcall	putc				;return the char in temp
	rjmp	bl_loop

;************************ code to jump to application ************************

addr0:
	;*************************************************************************
				;JUMP TO USER PROGRAM (BOOTLOADER EXITS HERE)
	rjmp addr0	;gets replaced by the rjmp instr. at 0x000 in the user program
				;THIS COMMAND MUST NOT BE IN THE SAME PAGE AS THE SPM COMMAND!
	;*************************************************************************


;***************************************************************************

getc:
	ldi 	bitcnt,8			;8 data bits (we don't use the stop bit, so no point sampling it)

getc_1a:
	sbis 	PINB,RxD			;Wait for idle
	rjmp 	getc_1a

getc_1b:						;Wait for start bit
	sbic 	PINB,RxD
	rjmp 	getc_1b

	rcall UART_delay			;0.5 bit delay

getc_2:
	rcall UART_delay			;1 bit delay
	rcall UART_delay

	clc							;clear carry
	sbic 	PINB,RxD			;if RX pin high
	sec					;
	ror 	temp				;shift bit into temp (need to do this here if bitcnt=8)

	wdr							;Make the cycle count up to 9 for each bit

	dec 	bitcnt				;If that was the 8th bit
	breq 	getc_3				;   then we have finished
								;else
	rjmp 	getc_2				;   go get next bit

getc_3:
	ret


;***************************************************************************

putc:
	ldi	bitcnt,9+sb				;1+8+sb (sb is nbr of stop bits)
	com	temp					;Invert everything
	sec							;Start bit

putc_0:
	brcc	putc_1				;If carry set
	cbi	PORTB,TxD				;    send a '0'
	rjmp	putc_2				;else

putc_1:
	sbi	PORTB,TxD				;    send a '1'
	wdr							;Make the cycle count up to 9 for '1' bits

putc_2:
	rcall UART_delay			;One bit delay
	rcall UART_delay

	lsr	temp					;Get next bit
	dec	bitcnt					;If not all bit sent
	brne	putc_0				;   send next
								;else
	ret							;   return


;***************************************************************************

UART_delay:
	ldi		delay,b
UART_delay1:
	dec		delay
	brne	UART_delay1

	ret



.if ( BLKMODE && EEPROM && AVRDUDE_Y )	;Include the EEPROM read/write code as a subroutine
;***************************************************************************

EEPROM_read:
	out	EEARL, AddrL
.if   ( PAGESIZE == 32 )
	out		EEARH, AddrH		; Don't need to use the extra code to do this for ATtiny25/45
.endif	; ( PAGESIZE )
	sbi	EECR, EERE
	in	temp, EEDR
	adiw	AddrH:AddrL, 1 		;increment Addr
	ret


;***************************************************************************

EEPROM_write:
	out		EEARL, AddrL
.if   ( PAGESIZE == 32 )
	out		EEARH, AddrH		; Don't need to use the extra code to do this for ATtiny25/45
.endif	; ( PAGESIZE )
	out		EEDR, temp
	sbi		EECR, EEMPE
	sbi		EECR, EEPE
EEPROM_write_1:
	sbic	EECR, EEPE			; Wait until EEPROM is ready (i.e. after previous write)
	rjmp	EEPROM_write_1
	adiw	AddrH:AddrL, 1		; Increment Addr
	ret


.endif	; ( BLKMODE && EEPROM && AVRDUDE_Y )
;***************************************************************************

firstinstr:
	ldi	AddrL, LOW((addr0+PAGESIZE)*2) & ~(PAGESIZE*2-1) ;Get address of the first byte of the page
	ldi	AddrH, HIGH((addr0+PAGESIZE)*2)					 ;after the page with "addr0" in it
	ldi	temp2, PAGESIZE
firstinstr_loop:
	sbiw AddrH:AddrL, 1			;Fetch words from the page with "addr0" (in reverse order)
	lpm	DataH, Z				;Copying the page in reverse order saves some code size because
	sbiw AddrH:AddrL, 1			;it leaves AddrH:AddrL correctly set for the call to writepage
	lpm	DataL, Z
	cpi	AddrL, LOW(addr0*2)
	brne	firstinstr_write	;until we get to "addr0"
	movw	DataH:DataL, InstrH:InstrL	;when we use the contents of InstrH:InstrL instead
firstinstr_write:
	ldi	temp, 0x01				;and copy them into the flash buffer
	rcall	do_spm
	dec	temp2
	brne	firstinstr_loop		;Until we fill the buffer

	rcall	writepage			;Then write the flash page back (with the 1st pgm cmd saved@addr0)

	;Reset AddrH:AddrL with 0x0000 and replace the cmd for address 0x0000 with "rjmp bl_start"
	movw	AddrH:AddrL, Hex00H:Hex00L
	ldi	temp2, HIGH(bl_start-1) | 0xC0	;New command for 0x000: "rjmp bl_start"
	ldi	temp, LOW(bl_start-1)
	movw	DataH:DataL, temp2:temp		;Return "rjmp bl_start" in DataH:DataL
	ret


;***************************************************************************

writepage:
	;erases the page and writes the buffer to it
	ldi	temp, 0x03 				;erase page
	rcall	do_spm
	ldi	temp, 0x05 				;write page
do_spm:
	;execute the spm action specified in temp
	out	SPMCSR, temp
	spm
	ret
