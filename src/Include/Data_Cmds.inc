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
;//////////BEGIN COMMANDS//////////

;Note about command routines: Clear XL bit 0 to have variables leave their value
;on the stack. This flag is set again at the start of each line.

;Format for command list:
	;1. A .db line with three characters which are the first
	;three letters of the command plus a fourth to fill out the word.
	;Commands less than three bytes need to be righ-justfied, with
	;blank leading bytes having the value $50. Compare values are
	;anded with $5F.
	;2. A .dw line that is the label for the routine to be executed.

commandlist:
	.db	"LIS "
commandlist1:
	.dw	listcommand
commandlist2:

.if HELP
	.db	"HELP"
	.dw	HELPcommand
.endif

	.db	"RUN "
	.dw	runcommand

	.db	"SAVE"		;SAVE	-Save contents of variable space and program memory to EEPROM
	.dw	savecommand

	.db	"LOAD"		;LOAD	-Load variable space and program memory from EEPROM
	.dw	loadcommand

	.db	"PRI "		;PRINT
	.dw	printcommand

	.db	"PRX "		;PRX -Print in hexidecimal
	.dw	printhexcommand

	.db	"PRB "		;PRB -Print in binary
	.dw	prbcommand

	.db	"EMIT"		;EMIT -send byte on stack as ascii to terminal.
	.dw	emitcommand

	.db	"FOR "
	.dw	forcommand

	.db	$50,"TO "		; TO
	.dw	tocommand

	.db	"NEXT"
	.dw	nextcommand

	.db	"FRE "			;FREE bytes or ram remaining
	.dw	FREEcommand

	.db	"NEW "			;NEW	clear progam space, start over
	.dw	newprogram

	.db	$50,"IF "		; IF
	.dw	ifcommand

	.db	"THE "			;THEN
	.dw	thencommand

	.db	"END "
	.dw	endcommand

	.db	"RST "		;RST - causes a system reset
	.dw	RSTcommand

.if CFG
	.db	"CFG "		;CFG - reads and writes non-volatile memory
	.dw	CFGcommand
.endif

.if RENUMBR
	.db	"RENU"		; RENUM - renumbers the current program in memory
	.dw	RENUmcommand
.endif

	.db	$50,(':' & $5F),('=' & $5F),$20	; :=
	.dw	setequalscommand

	.db	$50,$50,('>' & $5F),$20	; > greater-than operator
;	.dw	greaterthancommand
	.dw	diffcommand

	.db	$50,$50,('<' & $5F),$20	; < less-than operator
;	.dw	lessthancommand
	.dw	diffcommand

	.db	$50,('<'& $5F),('>'& $5F),$20	; "<>" - not equal to
	.dw	notequalcommand

;	.db	$50,$1C,$1E,$20			; <>
	.db	$50,('!'& $5F),('='& $5F),$20	; "!=" - not equal to
	.dw	notequalcommand

	.db	$50,$50,('=' & $5F),$20	;  =	equal sign. Evaluate expression
	.dw	evaluatecommand

	.db	$50,$50,('+' & $5F),$20	; +
	.dw	addcommand

	.db	$50,$50,('-' & $5F),$20	; -
	.dw	subtractcommand

	.db	$50,$50,('*' & $5F),$20	; *
	.dw	multiplycommand

	.db	$50,$50,('/' & $5F),$20	; /
	.dw	DIVIDEcommand

	.db	"MOD "					; modulus
	.dw	MODULUScommand

	.db	$50,$50,('%' & $5F),$20	; % modulus
	.dw	MODULUScommand

	.db	$50,$50,('$' & $5F),$20	; $ Hex command - interpret following as hexidecimal.
	.dw	HEXcommand

	.db	$50,$50,(''' & $5F),$20	; $ binary command - interpret following as 8-bit binary
	.dw	BINcommand

	.db	$50,$50,('"' & $5F),$20	; $ String command - print following as a string
	.dw	STRINGcommand

	.db	"DATA"		; DATA - saves DATA for later program use
	.dw	DATAcommand

	.db	"READ"		; READ - read next data byte from DATA statement
	.dw	READcommand

	.db	"REST"		; RESTore - resetes the pointer to the DATA array
	.dw	RESTcommand

.if SPGM
	.db	"WPM "		; Write Program memory - write data to FLASH
	.dw	WPMcommand
;	.dw SPGM_DataWrite

	.db	"RPM "		; Read Program memory - read data from FLASH
	.dw	RPMcommand

	.db	"EPM "		; Erase Program memory - clear all data from FLASH
	.dw	EPMcommand
;	.dw SPGM_PageErase
.endif

.if	( INTREG && (DHT || RTC || DFR) )
	.db	$50,$50,('@' & $5F),$20	;next is a constant
	.dw	FETCHcommand
.endif

	.db	"LSR "
	.dw	lsrcommand

	.db	"LSL "
	.dw	lslcommand

	.db	$50,$50,('&' & $5F),$20	; & - alternate Logical AND
	.dw	andcommand

	.db	"AND "			;AND - Logical AND
	.dw	andcommand

	.db	$50,$50,('|' & $5F),$20	; | - alternate Logical OR
	.dw	orcommand

	.db	$50,"OR "		;OR -Logical OR
	.dw	orcommand

	.db	$50,$50,('^' & $5F),$20	; ^ - alternate Logical XOR
	.dw	XORcommand

	.db	"XOR "			;XOR - Logical XOR
	.dw	XORcommand

	.db	"COM "		;Logical COMpliment (1's compliment)
	.dw	comcommand

	.db	$50,$50,('!' & $5F),$20	; alternate Logical NOT
	.dw	COMcommand

	.db	"NEG "		;NEG - Logical NEGate (2's compliment
	.dw	negcommand

	.db	"SWAP"		;SWAP - SWAP the high and low nibbles
	.dw	SWAPcommand

;********************************************************************
.if XTRAOPS	;support for byte-size additional operators
	.db	"NBH "		;NBH - Return the high nibble
	.dw	NBHcommand

	.db	"NBL "		;NBL - Return the low nibble
	.dw	NBLcommand

	.db	"REV "		;REV - reverse the bit value
	.dw	REVcommand

	.db "RND "		;generate 8-bit random # (w/ optional value passed)
	.dw	RNDcommand
.endif
;********************************************************************

	.db	"AOV "		;AOV - enable/disable arithmetic over/underflow
	.dw	aovcommand	; detection.  Enabled by default.

	.db "CRC "		;generate 8-bit CRC from command parameters
	.dw	CRCcommand

.if	CRC16R
	.db "CRW "		;generate 16-bit CRC from command parameters
	.dw	CRWcommand
.endif

	.db	"DELA"		;DELAY
	.dw	delaycommand

	.db	"GOT "
	.dw	gotocommand

	.db	"GOS "		;GOSUB
	.dw	gosubcommand

	.db	"RET "		;RETURN
	.dw	returncommand

	.db	"KEY "		;WAIT -pause execution until char received from terminal. Halt if control-C received
	.dw	keycommand

 .if SCLK
  .ifdef CLKPR
	.db	"CLK "		;CLK - read/write system clock prescaler
	.dw	SCLKcommand
  .endif
	.db "OSC "		;Set/read AVR's oscillator calibration byte
	.dw	OSCcommand
.endif

.if AComp
	.db	"ACO "		;ACO - Put analog comparator output on the stack
	.dw	acocommand

	.db	"ACS "		;ACS - selects the ADC channel for the analog 
	.dw	acscommand	;      comparator's "AIN1(-)" interrupt

  .if DATAREC
	.db	"ACI "		;ACI - enable or disable the analog comparator's interrupt
	.dw	acicommand

	.db	"ACR "		;ACR - set 8 bit pwm channel as analog comparator reference (same as "PWM8")
	.dw	ACRcommand

	.db	"DIG "		;DIG - Digital OUT for AVR Data Recorder
	.dw	DIGcommand

	.db	"LDD "		;LDD - Load default capture program for the AVR Data Recorder
	.dw	LDDcommand
  .endif
.endif
;
;*******************************************************
.if EFS		;file system support
	.db	"ERA "		;ERA - erase a saved program
	.dw	ERAcommand

	.db	"INIT"		;INIT - Initialize file system
	.dw	INITcommand

	.db	"CAT "		;CAT - CATalog a list of saved programs
	.dw	CATcommand

	.db	"REM "		;REM - REMark used to identify a program
	.dw	REMcommand
.endif
;
;*******************************************************

	.db	"PEEK"		;PEEK -return data pointed to by parameter
	.dw	peekcommand

	.db	"POKE"		;POKE - "POKE data, address"
	.dw	pokecommand

	.db	"EER "		;EER -read a byte from EEP
	.dw	EERcommand

	.db	"EEW "		;EEW -write a byte to EEP
	.dw	EEWcommand
;
.if	( INTREG && (DHT || RTC || DFR) )
	.db	"VPG "		;VPG - return the page in RAM internal variable are stored in
	.dw	VPGcommand
.endif

.if LPMD
	.db	"SLP "		;SLP - Enter Sleep Mode
	.dw	slpcommand
.endif
;
.if DFR			;Data file using Serial EEPROM
	.db	"DFW "		;DFW - write data to random address
	.dw	DFWcommand

	.db	"DFR "		;DFR - read data from random address
	.dw	DFRcommand

	.db	"DFL "		;DFL - log data to the address held in the internal
	.dw	DFLcommand	;  address counter

	.db	"DFX "		;DFX - reset the internal address counter to "0"
	.dw	DFXcommand

	.db	"DFF "		;DFF - set data dump format to "n" bytes
	.dw	DFFcommand

	.db	"DFD "		;DFD - Dump sequential data from address
	.dw	DFDcommand

	.db	"DFV "		;DFV - set the initialization value for the DFI command
	.dw	DFVcommand

	.db	"DFI "		;DFI - Initialize Data File
	.dw	DFIcommand

  .if INTREG		;only if internal variables
	.db	"DFA "		;DFA - Returns the low-byte of the address in RAM of
	.dw	DFAcommand	; the logging command
  .endif
.endif
;
.if (NRF && RTC)	;NRF and RTC commands
	.db	"RFI "		;RFI - initialize the nRF24L01
	.dw	RFIcommand

	.db	"RFA "		;RFA - set the nRF24L01 addresses
	.dw	RFAcommand

	.db	"RFC "		;RFP - set the nRF24L01's TX/RX channel
	.dw	RFCcommand

	.db	"RFP "		;RFP - set the nRF24L01's TX power level
	.dw	RFPcommand

	.db	"RFB "		;RFB - set the nRF24L01's RX buffer sizes
	.dw	RFBcommand

	.db	"RFE "		;RFE - enable/disable the nRF24L01's TX PA
	.dw	RFEcommand

	.db	"RFT "		;RFT - transmit data
	.dw	RFTcommand

	.db	"RFX "		;RFX - transfer the nRF24L01's RX buffer to
	.dw	RFXcommand	; the DATA staement buffer

	.db	"RFF "		;RFF - flush the nRF24L01's TX/RX buffers
	.dw	RFFcommand

	.db	"RFD "		;RFD - set or return the nRF24L01's data rate
	.dw	RFDcommand

	.db	"RFW "		;RFW - write a value to a register in the nRF24L01
	.dw	RFWcommand

	.db	"RFR "		;RFR - return the value of a nRF24L01 register or STATUS
	.dw	RFRcommand
.endif
;
.if (DHT && RTC)	;Acquire temperature and humidity from DHTxx device
	.db	"DHT "		;DHT - read DHTxx temperature
	.dw	DHTcommand

	.db	"DHH "		;DHH - read DHTxx humidity
	.dw	DHHcommand

	.db	"DHR "		;DHR - return sensor status
	.dw	DHRcommand

	.db	"DHU "		;DHU - set units flag (deg C/deg F)
	.dw	DHUcommand

	.db	"DHS "		;DHS - set ignore sign flag
	.dw	DHScommand

  .if INTREG		;only if internal variables
	.db	"DHI "		;DHI - returns the value of the low byte of RH_I@RAM
	.dw	DHIcommand

	.db	"DHD "		;DHD - returns the value of the low byte of RH_D@RAM
	.dw	DHDcommand

	.db	"DTI "		;DTI - returns the value of the low byte of RT_I@RAM
	.dw	DTIcommand

	.db	"DTD "		;DTD - returns the value of the low byte of RT_D@RAM
	.dw	DTDcommand
  .endif
.endif

.if RTC
	.db	"RTI "		;RTI - Set real-time counter interval
	.dw	RTIcommand

	.db	"RTR "		;RTR - Resets the real-time counter to zero
	.dw	RTRcommand

	.db	"RTP "		;RTP - Print real-time counter value or assign to variable
	.dw	RTPcommand

  .if INTREG		;only if internal variables
	.db	"RTC "		;RTC[3:0] - returns the value of the BYTE[n] of RT3@RAMRTCReg
	.dw	RTCcommand
  .endif
.endif

.if OWR	;1-wire interface
;	.db	"OWA "		;OWA - One-Wire Addess
;	.dw	OWAcommand

	.db	"OWI "		;OWI - One-Wire Initialize bus
	.dw	OWIcommand

	.db	"OWW "		;OWW - One-Wire Write data
	.dw	OWWcommand

	.db	"OWR "		;OWA - One-Wire Read data
	.dw	OWRcommand
.endif

#if defined(__ATtiny85__) || \
	defined(__ATtiny84__) || defined(__ATtiny84A__)

  .if PIO && defined(PORTA)
	;direction register
	.db	"ODA "		;ODA -Out Data Direction A
	.dw	outdAcommand

	.db	"IDA "		;IDA -Input direction direction A
	.dw	inDirectionA

	.db	"SDA "		;SDA -Set direction bit in B
	.dw	setdirectionA

	.db	"CDA "		;CDA -Clear direction bit in B
	.dw	cleardirectionA

	;output
	.db	"OPA "		;OPA -Out Port B
	.dw	outpAcommand

	.db	"SBA "		;SBA -Set bit in A register
	.dw	setbitinA

	.db	"CBA "		;CBA -Clear bit in A register
	.dw	clearbitinA

	.db	"XBA "		;XBA -XOR bit in A register
	.dw	xorbitinA

	.db	"PBA "		;PBA -Pulse bit in A register
	.dw	plsbitinA

	;input
	.db	"INA "		;INB -Input data from port B
	.dw	inAcommand

	.db	"IBA "		;IBB -Test bit in B
	.dw	testbitinA
  .endif

  .if PIO && defined(PORTB)
	;direction register
	.db	"ODB "		;ODB -Out Data Direction B
	.dw	outdbcommand

	.db	"IDB "		;IDB -Input direction direction B
	.dw	inDirectionB

	.db	"SDB "		;SDB -Set direction bit in B
	.dw	setdirectionB

	.db	"CDB "		;CDB -Clear direction bit in B
	.dw	cleardirectionB

	;output
	.db	"OPB "		;OPB -Out Port B
	.dw	outpbcommand

	.db	"SBB "		;SBD -Set bit in B register
	.dw	setbitinB

	.db	"CBB "		;CBB -Clear bit in B register
	.dw	clearbitinB

	.db	"XBB "		;XBB -XOR bit in B register
	.dw	xorbitinB

	.db	"PBB "		;PBB -Pulse bit in A register
	.dw	plsbitinB

	;input
	.db	"INB "		;INB -Input data from port B
	.dw	inbcommand

	.db	"IBB "		;IBB -Test bit in B
	.dw	testbitinB
  .endif
#else
.if PIO && defined(PORTA)
	;direction register
	.db	"ODA "		;ODA -Out Data Direction A
	.dw	outdXcommand

	.db	"IDA "		;IDA -Input direction direction A
	.dw	inDirectionX

	.db	"SDA "		;SDA -Set direction bit in A
	.dw	setdirectionX

	.db	"CDA "		;CDA -Clear direction bit in A
	.dw	cleardirectionX

	;output
	.db	"OPA "		;OPA -Out Port A
	.dw	outpXcommand

	.db	"SBA "		;SBA -Set bit in A register
	.dw	setbitinX

	.db	"CBA "		;CBA -Clear bit in A register
	.dw	clearbitinX

	.db	"XBA "		;XBA -XOR bit in A register
	.dw	xorbitinX

	.db	"PBA "		;PBA -Pulse bit in A register
	.dw	plsbitinX

	;input
	.db	"INA "		;INA -Input data from port A
	.dw	inXcommand

	.db	"IBA "		;IBA -Test bit in A
	.dw	testbitinX
.endif
;
 .if PIO && defined(PORTB)
	;direction register
	.db	"ODB "		;ODB -Out Data Direction A
	.dw	outdXcommand

	.db	"IDB "		;IDB -Input direction direction B
	.dw	inDirectionX

	.db	"SDB "		;SDB -Set direction bit in B
	.dw	setdirectionX

	.db	"CDB "		;CDB -Clear direction bit in B
	.dw	cleardirectionX

	;output
	.db	"OPB "		;OPB -Out Port B
	.dw	outpXcommand

	.db	"SBB "		;SBD -Set bit in B register
	.dw	setbitinX

	.db	"CBB "		;CBD -Clear bit in B register
	.dw	clearbitinX

	.db	"XBB "		;XBB -XOR bit in B register
	.dw	xorbitinX

	.db	"PBB "		;PBB -Pulse bit in A register
	.dw	plsbitinX

	;input
	.db	"INB "		;INB -Input data from port B
	.dw	inXcommand

	.db	"IBB "		;IBB -Test bit in B
	.dw	testbitinX
  .endif
#endif
;
;*******************************************************
 .if PIO && defined(PORTC)
	;direction register
	.db	"ODC "		;ODC -Out Data Direction C
	.dw	outdXcommand

	.db	"IDC "		;IDC -Input direction direction C
	.dw	inDirectionX

	.db	"SDC "		;SDC -Set direction bit in C
	.dw	setdirectionX

	.db	"CDC "		;CDC -Clear direction bit in C
	.dw	cleardirectionX

	;output
	.db	"OPC "		;OPC -Out Port C
	.dw	outpXcommand

	.db	"SBC "		;SBD -Set bit in C register
	.dw	setbitinX

	.db	"CBC "		;CBD -Clear bit in C register
	.dw	clearbitinX

	.db	"XBC "		;XBC -XOR bit in C register
	.dw	xorbitinX

	.db	"PBC "		;PBC -Pulse bit in C register
	.dw	plsbitinX

	;input
	.db	"INC "		;INC -Input data from port C
	.dw	inXcommand

	.db	"IBC "		;IBB -Test bit in C
	.dw	testbitinX
 .endif
;
;*******************************************************
.if PIO && defined(PORTD)
	;direction register
	.db	"ODD "		;ODD -Out Data Direction D
	.dw	outdXcommand

	.db	"IDD "		;IDD -Input direction direction D
	.dw	inDirectionX

	.db	"SDD "		;SDD -Set direction bit in D
	.dw	setdirectionX

	.db	"CDD "		;CDD -Clear direction bit in D
	.dw	cleardirectionX

	;output
	.db	"OPD "		;OPD -Out Port D
	.dw	outpXcommand

	.db	"SBD "		;SBD -Set bit in D register
	.dw	setbitinX

	.db	"CBD "		;CBD -Clear bit in D register
	.dw	clearbitinX

	.db	"XBD "		;XBD -XOR bit in D register
	.dw	xorbitinX

	.db	"PBD "		;PBD -Pulse bit in D register
	.dw	plsbitinX

	;input
	.db	"IND "		;IND -Input data from port D
	.dw	inXcommand

	.db	"IBD "		;IBD -Test bit in D
	.dw	testbitinX
  .endif

;**********************
.if PIO && defined(PORTE)
	;direction register
	.db	"ODE "		;ODB -Out Data Direction E
	.dw	outdXcommand

	.db	"IDE "		;IDE -Input direction direction E
	.dw	inDirectionX

	.db	"SDE "		;SDE -Set direction bit in E
	.dw	setdirectionX

	.db	"CDE "		;CDE -Clear direction bit in E
	.dw	cleardirectionX

	;output
	.db	"OPE "		;OPE -Out Port E
	.dw	outpXcommand

	.db	"SBE "		;SBE -Set bit in E register
	.dw	setbitinX

	.db	"CBE "		;CBE -Clear bit in E register
	.dw	clearbitinX

	.db	"XBE "		;XBE -XOR bit in E register
	.dw	xorbitinX

	.db	"PBE "		;PBE -Pulse bit in E register
	.dw	plsbitinX

	;input
	.db	"INE "		;INE -Input data from port E
	.dw	inXcommand

	.db	"IBE "		;IBE -Test bit in E
	.dw	testbitinX
  .endif

;*******************************************************
.if PIO && defined(PORTF)
	;direction register
	.db	"ODF "		;ODF -Out Data Direction F
	.dw	outdXcommand

	.db	"IDF "		;IDF -Input direction direction F
	.dw	inDirectionX

	.db	"SDF "		;SDF -Set direction bit in F
	.dw	setdirectionX

	.db	"CDF "		;CDF -Clear direction bit in F
	.dw	cleardirectionX

	;output
	.db	"OPF "		;OPF -Out Port F
	.dw	outpXcommand

	.db	"SBF "		;SBF -Set bit in F register
	.dw	setbitinX

	.db	"CBF "		;CBF -Clear bit in F register
	.dw	clearbitinX

	.db	"XBF "		;XBF -XOR bit in F register
	.dw	xorbitinX

	.db	"PBF "		;PBF -Pulse bit in F register
	.dw	plsbitinX

	;input
	.db	"INF "		;INF -Input data from port F
	.dw	inXcommand

	.db	"IBF "		;IBF -Test bit in F
	.dw	testbitinX
  .endif

;*******************************************************
.if PIO && defined(PORTG)
	;direction register
	.db	"ODG "		;ODG -Out Data Direction G
	.dw	outdXcommand

	.db	"IDG "		;IDG -Input direction direction G
	.dw	inDirectionX

	.db	"SDG "		;SDG -Set direction bit in G
	.dw	setdirectionX

	.db	"CDG "		;CDG -Clear direction bit in G
	.dw	cleardirectionX

	;output
	.db	"OPG "		;OPG -Out Port G
	.dw	outpXcommand

	.db	"SBG "		;SBG -Set bit in G register
	.dw	setbitinX

	.db	"CBG "		;CBG -Clear bit in G register
	.dw	clearbitinX

	.db	"XBG "		;XBG -XOR bit in G register
	.dw	xorbitinX

	.db	"PBG "		;PBG -Pulse bit in G register
	.dw	plsbitinX

	;input
	.db	"ING "		;ING -Input data from port G
	.dw	inXcommand

	.db	"IBG "		;IBG -Test bit in G
	.dw	testbitinX
  .endif

;*******************************************************
.if PIO && defined(PORTH)
	;direction register
	.db	"ODH "		;ODH -Out Data Direction H
	.dw	outdXcommand

	.db	"IDH "		;IDH -Input direction direction H
	.dw	inDirectionX

	.db	"SDH "		;SDH -Set direction bit in H
	.dw	setdirectionX

	.db	"CDH "		;CDH -Clear direction bit in H
	.dw	cleardirectionX

	;output
	.db	"OPH "		;OPH -Out Port H
	.dw	outpXcommand

	.db	"SBH "		;SBH -Set bit in H register
	.dw	setbitinX

	.db	"CBH "		;CBH -Clear bit in H register
	.dw	clearbitinX

	.db	"XBH "		;XBH -XOR bit in H register
	.dw	xorbitinX

	.db	"PBH "		;PBH -Pulse bit in H register
	.dw	plsbitinX

	;input
	.db	"INH "		;INH -Input data from port H
	.dw	inXcommand

	.db	"IBH "		;IBH -Test bit in H
	.dw	testbitinX
  .endif

;*******************************************************
.if PIO && defined(PORTJ)
	;direction register
	.db	"ODJ "		;ODJ -Out Data Direction J
	.dw	outdXcommand

	.db	"IDJ "		;IDJ -Input direction direction J
	.dw	inDirectionX

	.db	"SDJ "		;SDJ -Set direction bit in J
	.dw	setdirectionX

	.db	"CDJ "		;CDJ -Clear direction bit in J
	.dw	cleardirectionX

	;output
	.db	"OPJ "		;OPJ -Out Port J
	.dw	outpXcommand

	.db	"SBJ "		;SBJ -Set bit in J register
	.dw	setbitinX

	.db	"CBJ "		;CBJ -Clear bit in J register
	.dw	clearbitinX

	.db	"XBJ "		;XBJ -XOR bit in J register
	.dw	xorbitinX

	.db	"PBJ "		;PBJ -Pulse bit in J register
	.dw	plsbitinX

	;input
	.db	"INJ "		;INJ -Input data from port J
	.dw	inXcommand

	.db	"IBJ "		;IBJ -Test bit in J
	.dw	testbitinX
  .endif

;*******************************************************
.if PIO && defined(PORTK)
	;direction register
	.db	"ODK "		;ODK -Out Data Direction K
	.dw	outdXcommand

	.db	"IDK "		;IDK -Input direction direction K
	.dw	inDirectionX

	.db	"SDK "		;SDK -Set direction bit in K
	.dw	setdirectionX

	.db	"CDK "		;CDK -Clear direction bit in K
	.dw	cleardirectionX

	;output
	.db	"OPK "		;OPK -Out Port K
	.dw	outpXcommand

	.db	"SBK "		;SBK -Set bit in K register
	.dw	setbitinX

	.db	"CBK "		;CBK -Clear bit in K register
	.dw	clearbitinX

	.db	"XBK "		;XBK -XOR bit in K register
	.dw	xorbitinX

	.db	"PBK "		;PBK -Pulse bit in K register
	.dw	plsbitinX

	;input
	.db	"INK "		;INK -Input data from port K
	.dw	inXcommand

	.db	"IBK "		;IBK -Test bit in K
	.dw	testbitinX
  .endif

;*******************************************************
.if PIO && defined(PORTL)
	;direction register
	.db	"ODL "		;ODL -Out Data Direction L
	.dw	outdXcommand

	.db	"IDL "		;IDL -Input direction direction L
	.dw	inDirectionX

	.db	"SDL "		;SDL -Set direction bit in L
	.dw	setdirectionX

	.db	"CDL "		;CDL -Clear direction bit in L
	.dw	cleardirectionX

	;output
	.db	"OPL "		;OPL -Out Port L
	.dw	outpXcommand

	.db	"SBL "		;SBL -Set bit in L register
	.dw	setbitinX

	.db	"CBL "		;CBL -Clear bit in L register
	.dw	clearbitinX

	.db	"XBL "		;XBL -XOR bit in L register
	.dw	xorbitinX

	.db	"PBL "		;PBL -Pulse bit in L register
	.dw	plsbitinX

	;input
	.db	"INL "		;INL -Input data from port L
	.dw	inXcommand

	.db	"IBL "		;IBL -Test bit in L
	.dw	testbitinX
.endif
;
;*********************
.if	DSI
	.db	"DSRE"		;DSRECIVE - recieve by to TOS from DS Interface
	.dw	dsrcommand

	.db	"DSCO"		;DSCOMMAND - Send TOS via DS Interface as command
	.dw	dsccommand

	.db	"DSDE"		;DSDATA - Send TOS via DS Interface as data
	.dw	dsdcommand

.endif
;
;************************
.if ICP
  .ifdef TCNT1H	;enabled if 16-bit timer available
	.db	"ICG "		;ICG - set input capture gate time to value [0..7]
	.dw	ICGcommand	; NOTE this command must be issued before issuing
					; a 'ICP' command

	.db	"ICE "		;ICG [x] - set input capture edge select to [0..1]
	.dw	ICEcommand	; where 0 = falling and 1 = rising edge.
					; [x] is optional and defaults rising edge (1).

	.db	"ICP "		;ICP - start capture with 'ICG' gatetime and return
	.dw	ICPcommand	;  low-byte of ICP register, store high-byte in 'Z'
  .endif
.endif

;************************
#if !defined(__ATtiny84__) && !defined(__ATtiny84A__) && \
	!defined(__ATtiny85__)
 .if PWM
	.db	"PWE "		;PWE  send value to 10 bit pwm channel, set DDRB,3 to 1
	.dw	PWEcommand
 .endif
#endif

.if PWM
	.db	"PWM "		;PWM  send value to 8 bit pwm channel, set DDRB,3 to 1
	.dw	PWMcommand

	.db	"PWO "		;PWO   turn off pulse-width modulation. Do not change DDRB
	.dw	PWOcommand
.endif

;************************
.if DDS
	.db	"DDS "		;DDS - Enables DDS and sets frequency
	.dw	DDScommand
.endif

;**********************
.if DATAREC
	.set	ADConv	= 1		;insure ADConv routines are enabled

	.db	"ADS "		;ADS - select CH0 or CH1 to AD536
	.dw	ADScommand

	.db	"ADU "		;ADU - select TRMS or dBV from AD536
	.dw	ADUcommand

	.db	"ACR "		;ACR  send value to 8 bit pwm channel, same as "PWM8"
	.dw	ACRcommand

	.db	"ADG "		;ADG - select CH0 or CH1 gains
	.dw	ADGcommand
.endif
;
.if ADConv
	.db	"ADC "		;ADC -provide value from a to do converter
	.dw	ADCcommand

	.db	"ADR "		;ADR -select between external and internal ref
	.dw	ADRcommand
.endif
;
;**********************
.ifdef SPDR
  .if SPI
; SPM [0..2] SPM must be called first to init the SPR
;	Init defaults to Master, Mode 0, F_CLK/16, MSB first
	.db	"SPM "		;SPR -set SPI mode [0..2]
	.dw	SPMcommand

; SPO [0..1] is optional  (see data sheet)
	.db	"SPO "		;SPR -set data order to U where U = [0..1]
	.dw	SPOcommand

; SPC [0..15] is optional (see data sheet)
	.db	"SPC "		;SPC -set the SPI clock rate
	.dw	SPCKcommand

; SPW [0..255] set the SS pin low and transmits U
;	Execute this command to write data to SPI
;	Note that user must set the SPI_SS on SPI_PORT high after last byte
	.db	"SPW "		;SPX -write a value to the SPI port
	.dw	SPWcommand

; SPR recieves a byte into U
;	Execute this command to recieve data from the SPI port
;	Note that user must set the SPI_SS on SPI_PORT high after last byte
	.db	"SPR "		;SPR -read a value from the SPI port
	.dw	SPRcommand

; SPS [0..1] is optional  (see data sheet)
	.db	"SPS "		;SPS -set SPI_SS pin to level to U where U = [0..1]
	.dw	SPScommand
  .endif
.endif

;**********************
.if TWI
; TWI [0..2] TWI must be called first to init the TWI interface.
;	Defaults to Master and TWI_CLK = 400K.  Where x =
;	  0 to disable TWI interface.
;	  1 to enable TWI interface with 400K clock
;	  2 to enable TWI interface with 100K clock
;	  3 to enable TWI interface with  50K clock (really slow?)
	.db	"TWI "		;TWI -Init the TWI mode [0..2]
	.dw	TWIcommand

; TWS Set START condition
	.db	"TWS "
	.dw	TWScommand

; TWP set STOP condition
	.db	"TWP "
	.dw	TWPcommand

; TWA [x] - TWA sends the slave address to the bus.
;	.db	"TWA "
;	.dw	TWAcommand

; TWW [0 1 2 3 .. DATASTKSZ]
;	Execute this command writes a series of data bytes to a TWI slave
	.db	"TWW "
	.dw	TWWcommand

 ;TWX - TWX receives data from a slave device into the DATA buffer
;	.db	"TWX "
;	.dw	TWXcommand

; TWR receives a byte into U
;	Execute this command to recieve data from the TWI bus onto stack
	.db	"TWR "
	.dw	TWRcommand

; TWB receives the current bus status (lsr'ed by 3 bits) into U
	.db	"TWB "
	.dw	TWBcommand
.endif
;
;*******************************************************
.if	DEBUG
	.db	"DEB "		;DEBug - enable or disable program line display
	.dw	debugcommand

	.db	"DUMP"
	.dw	dumppmembuffer

	.db	"VDUM"		;VDUM[P] - dump contents of variables
	.dw	dumpvmembuffer

	.db	"IDUM"		;IDUM[P] - dump contents of I/O
	.dw	dumpio

	.db	"EDUM"		;EDUM[P] - dump contents of EEP
	.dw	dumpemembuffer

	.db	"FILL"		;FILL - Fill contents of RAM
	.dw	FILLmembuffer
.endif
;
;*******************************************************
#if ( defined(__ATmega328__) || \
	  defined(__ATmega328P__) || \
	  defined(__ATmega168PA__) || \
	  defined(__AT90USB1286__)  || \
	  defined(__ATmega2560__) || \
	  defined(__ATmega32U4__) )

	.db	"WTF "
	.dw	WTFcommand
#endif
;
;************************
#if BTLDR
  #if ( defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) )
	.db	"BLDR"					;BLDR - jump to bootloader
	.dw	BLDRcommand
  #endif
#endif

	.dw	0x000000				;end of command list

