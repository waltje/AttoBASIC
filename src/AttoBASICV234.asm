	.equ	Version = 234	;version 2.34 (100x so its an integer)
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
; NOTE: USB is supported on Mega32U4, USB1286 and other AVR USB variants.
;		Mega88 support is inherant but some routines must be disabled
;		in order to fit into the 8K Program Space.
;		See the file "Include\Defs_Constants.inc" for details.
;
;			USB Serial Code Copyright (c) 2008,2010 PJRC.COM, LLC
;			http://www.pjrc.com/teensy/
;
;			LUFA DFU and CDC BootLoader for M32U4 and USB1286
;				written by Dean Camera
;			dean@fourwalledcubicle.com - http://www.lufa-lib.org
;
; **************************************************************************
; Internal Timer usage:
;	TINY84:
;		SIO uses TMR0 (8-bit)
;		PWM uses TMR1 (16-bit)
;		DDS uses TMR1 (16-bit) as sample clock.
;	TINY85:
;		SIO uses TMR0 (8-bit)
;		PWM uses TMR1 (8-bit)
;		DDS uses TMR1 (8-bit) as sample clock.
;	MEGA88/168/328:
;		RTC uses TMR0 (8-bit).
;		PWM uses TMR1 (16-bit)
;		ICP uses TMR1 (16-bit)
;		DHT uses TMR1 (temporarily as sample clock).
;		DDS uses TMR2 (8-bit) as sample clock.
;	MEGA16/32:
;		RTC uses TMR0 (8-bit).
;		PWM uses TMR1 (16-bit)
;		ICP uses TMR1 (16-bit)
;		DHT uses TMR1 (temporarily as sample clock).
;		DDS uses TMR2 (8-bit) as sample clock.
;	MEGA32U4:
;		RTC uses TMR0 (8-bit).
;		PWM uses TMR1 (16-bit)
;		ICP uses TMR1 (16-bit)
;		DHT uses TMR3 (temporarily as sample clock).
;		DDS uses TMR4 (8-bit) as sample clock.
;	MEGA644/1284:
;		RTC uses TMR0 (8-bit).
;		PWM uses TMR1 (16-bit)
;		ICP uses TMR1 (16-bit)
;		DHT uses TMR1 (temporarily as sample clock).
;		DDS uses TMR2 (8-bit) as sample clock.
;	USB1286:
;		RTC uses TMR0 (8-bit).
;		PWM uses TMR1 (16-bit)
;		DDS uses TMR2 (8-bit) as sample clock.
;		ICP uses TMR3 (16-bit)
;		DHT uses TMR3 (temporarily as sample clock).
;	Mega2560:
;		RTC uses TMR0 (8-bit).
;		PWM uses TMR1 (16-bit).
;		DDS uses TMR2 (8-bit) as sample clock.
;		ICP uses TMR3 (16-bit)
;		DHT uses TMR3 (temporarily as sample clock).
;		N/A uses TMR4 (16-bit)
;		N/A uses TMR5 (16-bit)

;************************************************************************
;REVISION HISTORY:
; All revisions listed below by K. Scott Vitale, Florida, uSA
;	email: ksv_prj@gmx.com
;========================================================================
; TO-DO:	* Add "TOP" command to set top of program memory.
;			* Support 16 bit integers?
;			* Add support for SPM; WPM/RPM/EPM commands to support 
;			  storing up to 256 bytes of data lookup tables in FLASH 
;			  memory.  ATtiny85 would be limited to 32 bytes.
;========================================================================
; Version 2.34
; Stats: Files 178
; Total Lines (including headers and comments):
;******************************************************************************
;	2017-0207	Bugfix: The DATA statement would hang if more than 12 entries 
;				  were on the same line.  This was due to saving the Y-Ptr too
;				  soon.  Corrected by moving the PUSHY statement afer the test
;				  for zero parameters.
;				Bugfix: In the "outdXcommand" routine, AttoBASIC always disabled 
;				  the JTAG interface, which would disconnect the hardware debugger
;				  from the target while debugging.  Routine now checks for the 
;				  OCDEN fuse being enabled, signifying that a debug session is 
;				  likely in process.  If OCDEN is enabled, the JTAG interface
;				  is not disabled.  Also added support for the mega16/32 and 
;				  mega644/1284 parts that use PORTC for JTAG instead of PORTF.
;				Added: pin definitions for JTAG parts to be used with the JTAG
;				  pin disabling fix above.
;	2016-1203	Added: For the SPI interface routines, added the ability to 
;				  disable AttoBASIC's control of the SS pin.  Use the define
;				  under the "SPI port pins" section of the "Defs_Constants.inc"
;				  file.  By default, AttoBASIC no longer controls the SS pin. 
;	2016-0720	Bugfix: When SelfStart is enabled and the target program 
;				  contains a multiple of 30 bytes, which is the file record 
;				  payload size, the RUN-time interpreter interprets garbage
;				  characters. 
;				  Fix1: In the "LOADcommand1a" routine; between the data load 
;				   from buffer and store to program memory, test for NULL 
;				   character in the loaded data signifying the end of the
;				   record.  Increment the program pointer (ZH:ZL) upon exit.
;				  Fix2: In the "RUNcommand" routine; check for the
;				   "SSTEn" flag set in GPIOR2.  If set, skip file loading as 
;				   the file has already been loaded.  Then clear the "SSTEn"
;				   flag.
;	2016-0715	Modified: Code reduction in the "WriteEEP" routine by removing
;				  interrupt support.  The idea being, just check for busy, write
;				  and return. Let the main loop take care of the sleep handling.
;	2016-0713	Modified: Code compaction in the "BitPositiontoORmask" routine.
;				Modified: Code compaction in the "DDScommand" routine.
;				Modified: Code compaction in all routines by creating common
;				  exit subroutines named "PopYret" and "PushUret".
;				Modified: Code compaction by creating routine named "ShiftHTU".
;				Modified: "ADCcommand" routine to make use of the DIDR registers
;				  for disenabling of the digital input buffers on the chosen
;				  ADC pin.  The DIDR register contents are not reset after an
;				  ADC read as the assumption is that if the pin is being read
;				  as an analog voltage then it's intended purpose is to be an
;				  analog input.
;	2016-0711	Added: REV command to reverse the order of bits in a value.
;	2016-0708	Added: Support for ATmega644P and ATmega1284P.
;				Added: Optiboot support for ATmega644P/1284P.
;				Modified: Analog comparator's "AIN-" select input with the
;				  "ACS" command.  Five choices available; ADC[3:0] or AIN1(-).
;	2016-0621	Added: Support for ATtiny84(A).  Same support as that of the
;				  ATtiny85.
;				Added: Conditional assembly in the "SLPcommand" routine to
;				  disable code generation if ADC and AC are not enabled.
;				Added: Optiboot85 support for ATtint84(A).
;				Added: "SYSCLK" routine to read and write the system clock
;				  prescaler.  Intended to be used for very low power 
;				  operation.
;				Added OSC comand to read and write the internal oscillator's
;				 calibration register.
;				Renamed: "SYSCLKSET" routine to "SYSCLKINIT".
;	2016-0615	Added: Support for the older ATmega16(L) and ATmega32(A).  
;				  Due to unenhanced WDT, not all peripheral modules are 
;				  functional.
;				Bugfix: DDS and ICP functions interfere with each other. ICG
;				  command disable the DDS sampling clock's IRQ's.
;				Bugfix: DELay routine does not function properly if the
;				  DDS command is used.  This is because the if the RTC
;				  routines are enabled, the RTC interrupts are used with a
;				  SLEEP instruction.  Once the DDS command is used, the
;				  DDS sampling timer is then generating interrupts, which
;				  pull the DELAY routine out of SLEEP.  Corrected by polling
;				  the RTCIRQ flag in GPIOR0.
;	2016-0530	Added: build support for ATmega163. Note that the code
;				  from the original AttoBASIC "mega163" is not merged
;				  into the current AttoBASIC code and is only supported
;				  at the HEX file level.  Thus NO NEW FEATURES!
;				Added: build support for AT90S8515/8515XR.  Note
;				  that the code from the original AttotBASIC "8515" and
;				  and "8515XR" is not merged into the current AttoBASIC code
;				  and is only supported at the HEX file level.  Thus NO
;				  NEW FEATURES!
;				Modified: "Init3" routine to only erase the 8 file handle
;				  locations if EFS is enabled.
;	2016-0510	Added: "DStkRev" routine to reverse data on the data stack.
;				  Currently unused.
;				Modified: Input line buffer now supports up to 104 characters
;				  on MCU's with 1024+ bytes of RAM, otherwise limited to 54
;				  characters (Tiny85).
;	2016-0503	Added: build support for AT90S2313/ATtiny2313[A].  Note
;				  that the code from the original AttotBASIC "2313" is
;				  not merged into the current AttoBASIC code and is only
;				  supported at the HEX file level.  Thus NO NEW FEATURES!
;	2016-0430	Modified: On USB-SIO builds, modifed the RUNcommand routine
;				  so that a program run in "self-start" mode, upon
;				  completion will wait for a host to connect indefinately.
;				Modified: Initialization routines for USB-SIO builds so
;				  that if self-start is enabled, there is a 5 second wait
;				  for a USB connection to be made before executing the
;				  program.  Also changed the USB connection to 2 minutes
;				  if self-start has not been enabled.
;	2016-0427	Modified: Optimized code to use new routines named as
;				  "CheckUisByteVal", "CheckUisNiblVal" and "CheckUisBitVal"
;				  for various checks for bit, nibble and byte value ranges
;				  dpecified in U.  Error if not within range.
;				Modified: Abreviated some of the string messages to gain
;				  program memory for ATtiny85.
;	2016-0426	Modified: EFS; moved file index pointers to use 0x18 to
;				  0x1F in E2P so that locations 0x00 to 0x07 can be used
;				  for AttoBASIC internal use and 0x08 to 0x17 for application
;				  use.
;				Modified: SAVe and LOAd routines to start saving at E2P
;				  location 0x10.= when EFS is not enabled.
;				Added: CFG command for "non-volatile configuration" to
;				  hold such options as "self-start" [dis][en]able,
;				  "Arithmetic Overflow" [dis][en]able, and
;				  "Debug" [dis][en]able.  The reserved locations are
;				   0x00 and 0x01 in EEPROM.  ATtiny85/ATmega88 support
;				   a limited subset.
;				Added:  16 bytes of reserve EEPROM for application use at
;				  addresses 0x08 to 0x17.
;				Modified: EEW routine so as not to erase the reserved
;				  section of E2P.
;				Bugfix: INIT1 routine would jump to INIT2 routine upon
;				  self-start detection, which is correct.  However, the
;				  Init2 routine completely bypasses the I/O port setup
;				  routines.
;	2015-1219	Modified GoTo and GoSub macro definitions, which now
;				  properly support detection of jmp/rjmp and call/rcall
;				  based on FLASH memory size.
;	2015-0401	BUGFIX: Corrected bug in USB SIO builds where saving too
;				  large a file to the EFS does not emit a message because
;				  USB SIO was not re-enabled before sending the error
;				  message.
;				Increased input line buffer size to 72 characters.
;				BUGFIX: added conditional support for the M168PA and M88PA
;				  to the "Code_Vectors.inc" and "Code_IRQvectors.inc" files.
;				  Thanks to Elmar Muller of Germany.;
;	2015-0212	BUGFIX: Corrected bug in "CRC16" routine where a the 2nd
;				  CRC byte was not accessible and caused a data stack error.
;				ADDED: Various CRC8 routines from different authors.  Picked
;				  lowest byte count with fastest execution speed.  Using
;				  DALLAS 1-WIRE CRC algorithm of x^8 + x^5 + x^4 + 1.
;	2015-0202	BUGFIX:  Corrected bug in "DeleteKey" routine where a BS
;				  would overwrite the prompt character and an infinite
;				  loop would occur.
;	2014-1006	ADDED: DEBug command to enable or disable line number
;				  display to aid in debugging.  Enabled for all builds.
;				MODIFIED: "prbcommand" routine to use H as bit counter
;				  instead of TEMP as VUART destroys TEMP on tiny85 builds.
;	2014-0908	ADDED: Enabled SPW command to use strings, which
;				  is useful for writing strings to SPI devices.
;	2014-0908	ADDED: Enabled TWW command to use strings, which is useful
;				  for writing strings to I2C-based displays.
;	2014-0905	MODIFIED: If the TWR command has no command line par-
;				  ameters, the value of a single TWI read is placed on the
;				  data stack.  This differs in that the TWR command was
;				  modified to accept a number as its parameter, which
;				  indicated the number of data bytes to store in the DATA
;				  buffer.  Now single-byte transfers can be assigned to
;				  a varible or multi-byte transfers can be buffered by
;				  the DATA statement.
;	2014-0902	BUGFIX:  SAVE command locks up if program is same size as
;				  available EEP.  Replaced "brpl" to "brsh".
;				BUGFIX: added 3mS delay in "WriteEEP" routine to allow for
;				  EEPROM write duration.  Sometimes saving a file would
;				  corrupt the EEPROM contents for the blocks being written
;				  to during the saving.
;
;========================================================================
; Version 2.33 (unreleased)
;	2014-0510	Check for proper operation in self-start mode on USB
;				  flavors.
;	2014-0509	Added: register "temp2" and re-arranged other registers.
;				  This is transparent to the user.
;				Further reduced string sizes by removing unnecessary
;				  NULL terminators.
;				Added: ^U to line editing to erase line.
;				Modify: On USB flavors, if self-start disabled OR the
;				  self-start pin is floating, continually poll for a USB
;				  connection.
;	2014-0505	Added: SIO using interrupt-driven software UART.  The
;				  Soft-UART can be used with INT0 or PCINT pins, which
;				  allows the TX and RX functions to be assigned to any
;				  pins.  Code size is about 360 bytes.
;				Consolodated FLASH memory by creating routines for often-
;				  used error messages; $06, $07, $08, $0A, $0C, $0D,
;				  $11, $12, $16 and 1D.
;				Moved the "KEYcommand" routine into each SIO module's
;				  include file since they are each target/build specific.
;	2014-0503	Conserved FLASH space by tokenizing common strings used
;				  within error messages and such by adding TOK_NOT,
;				  TOK_RNG and TOK_SUP tokens.
;				Modified: "sendromstring" and "calcromstrlen" routines
;				  to be aware of additional string substitution
;				  tokens; TOK_NOT, TOK_RNG and TOK_SUP.
;	2014-0430	Added: support for OptiBoot85 for the ATtiny85.
;	2014-0424	Added: support for ATtiny85.  Due to the size of the
;				  USI UART handler, there is little functionality BUT
;				  it is a novelty ... and kewl as well! :)
;				ATtiny85 hardware tested: 1-wire, ADC, PWM, Low-power and
;				  DDS.
;				Configured USI for ATMEL AVR307 Application note with
;				  clock and baud timing conditionals BUT this code takes
;				  over 600 bytes of code space so it is unused, which
;				  frees the USI for use as SPI or TWI (when I write the
;				  code for it, that is!).
;				BUGFIX: when EFS disabled and EEP is erased, the LOAD
;				  command would hang when trying to load a program
;				  because a null character, signifying the end of
;				  the program in EEP, was never found.
;				BUGFIX: In "ORandSTORE", "ANDandSTORE" and "XORandSTORE"
;				  routines, added 0x20 offset to port addresses due to
;				  use of he "ld" and "st" instructions.
;	2014-0423	Modified: "ADRcommand" routine for use with parts having
;				  REF2 bit (ATtiny85).  The reference select bits are
;				  now cleared then the target's REF bits are OR'ed into
;				  the MUX register.
;				BUGFIX: Insured that USB interrupts are disabled in the
;				  "LOAD" routine when the EFS is not enabled.
;				BUGFIX: Made "WriteEEP" routine more robust by insuring
;				  that a prior EEW is not in progress before writing or
;				  erasing a byte in EEPROM.  Also insured that the EERIE
;				  flag is set when the T-Flag is set to insure that the
;				  sleep instruction can wake up to an EEW complete IRQ.
;	2014-0419	BUGFIX: VDUMP command failed to display variables because
;				  only ZL was tested.  When the variables crossed a page
;				  boundary, there were issues.  Used CPI16 macro instead
;				  of "cpi" instruction.
;				BUGFIX: FETCHROMBYTE routine would error with a "command
;				  not found" if the low byte of the address of a command
;				  happened to be 0x00.  Since 2011, this dormant bug
;				  has never showed - until now!  Fix:  check both low
;				  and high bytes of routine's address to be sure neither
;				  is 0x00.  Yes! This was thought to be fixed in V2.32
;				  but was missing a "tst r1" command.
;	2014-0417	Modify: moved self-start pin-check routine closer to
;				  beggining of MCU RESET vector.  The self-start pin
;				  is checked promptly after port/pin settings.
;				Modify: Isolated the serial I/O routines by placing them
;				  in separate files for each type: UART, USB and USI.
;				  This simplifies debugging code and requires fewer MCU-
;				  specific conditional statements to be mixed amongst the
;				  various serial I/O handling routines.  Also makes it
;				  MUCH easier to debug.
;				BUGFIX: Corrected conditional statement bug that pertains
;				  to the DataRecorder project, wherein certain features
;				  were not enabled.
;========================================================================
; Version 2.32
;	2013-1024	Bugfix:  Bug introduced into OPTIBOOT on 2013-1003, where
;				  the MCUSR was not cleared to 0.  This caused the
;				  bootloader to constantly cycle upon a timeout rather
;				  than to transfer control to the application.  Fix:  the
;				  flags are saved by the bootloader.  Reloading MCUSR with
;				  the saved flags before jumping to the application seems
;				  to have fixed the problem.
;	2013-1022	Added PB[p] command to toggle the state of an I/O pin for
;				  10 mS.
;				Added support for "ODp" command to disable the JTAG
;				  interface before enable the port I/O pins.  Only
;				  valid on devices with JTAG on port F.
;				Modified SPR command to transfer "N" bytes of data from
;				  an SPI device to the DATA buffer.
;				Modified SPW command to transfer multiple bytes of data
;				  submitted as command line paramters to an SPI device.
;	2013-1021	Bugfix: FETCHROMBYTE routine would error with a "command
;				  not found" if the low byte of the address of a command
;				  happened to be 0x00.  Since 2011, this dormant bug
;				  has never showed - until now!  Fix:  check both low
;				  and high bytes of routine's address to be sure neither
;				  is 0x00.
;				Bugfix: EFS; attempt to load from an empty file loads the
;				  contents of file block 0, which is the file record imdex
;				  block, tus corrupting program memory.  Now checks for
;				  an empty file and emits error message if the requested
;				  block is empty.
;	2013-1019	Modified "getterminalline" rotuine to ignore TAB char-
;				  acters.  If mistakenly embedded in a program, their
;				  presence caused an unknown operator error.
;	2013-1018	Modified LUFA CDC Bootloader code to invoke bootloader
;				  if PORTF4 is grounded.
;				Added LUFA CDC Bootloader support to m32u4 and usb1286
;				  builds. CDC bootloader supports the AVR109 protocol,
;				  which is supported by AVRDUDE.
;	2013-1017	Modified TWR command to transfer "N" bytes of data from
;				  a TWI slave to the DATA buffer.
;				Modified TWW command to transfer multiple bytes of data
;				  submitted as command line paramters to a TWI slave.
;				Removed TWA command as the TWW command now recognizes
;				  TWI slave address TX/RX and DATA ACK conditions.  ACK
;				  always placed on the bus, even if the last data byte
;				  has been sent.  Use TWP to signal end of bus writes.
;	2013-1016	Revamped TWI support.  Most commands now respond with
;				  the bus status after a command is executed.  This
;				  allows for an AttoBASIC program to do its own error
;				  handling without aborting on a failure.
;	2013-1015	Consolodated conditional testing by moving tests to
;				  each individual software module, which keeps the
;				  main code file cleaner.  Trying to keep a standard
;				  method of (dis-)enabling the various software modules
;				  and automatically activating support code from other
;				  modules, when they are needed.
;				Cleaned up conditional statements in DHT support routines.
;				Corrected long-time bug in TWI routine where activating
;				  port-pin PUP's was hard-coded to PORTC and not to the
;				  target MCU's specific SCL/SDA pins.  This bug never
;				  showed because the test platform always had external
;				  PUP's.
;				Consolodated code by creating "PushURet" routine as the
;				  same instruction sequence is used by multiple commands.
;	2013-1014	Changed "!" operator to the COM (NOT) operator.  Was NEG.
;				Modified the BIN routine to accept from 1 to 8 characters.
;
;------------------------------------------------------------------------
; Version 2.31
;	2013-1013	Compacted code wherein recurring calls to retrieve two
;				  paremeters from the stack are now processed as a
;				  separate call to "GetTwoParms".  Much like "Get1stParm".
;				Disabled USB IRQ's in the 1-Wire support routines.
;				Added 1mS delay to OWI routine to allow port pin to
;				  settle upon 1st time bus initialization.
;				Corrected THEN commands conditional assembly to pop the 3rd
;				  byte of the return address from the stack for parts with
;				  more than 64KW or FLASH.  Originally, the test was for
;				  32KW of FLASH, which was incorrect and specifically
;				  affected the AT90USB1286 only.
;				Simplified THEN command by simply popping return addresses
;				  of the stack.
;	2013-1007	Added support for 1-Wire devices.  See command reference
;				  for details.
;				Added the PRIACT flag to XH run-time flags so that some
;				  routines have a method to know if they are following
;				  a PRINT, PRX or PRB command.
;	2013-1003	Modified stk500v2.c code to push 2 or 3 0x00's onto the
;				  stack so that a "RET" can be used instead of an "EICALL"
;				  when transferring control to the application program.
;				Corrected stk500v2.c code to leave all flags within the
;				  MCUSR register to remain untouched so the application
;				  can determine a course of action based upon them.
;				Updated OptiBoot Bootloader to version 5.0a.
;				Corrected optiboot.c code to leave all flags within the
;				  MCUSR register to remain untouched so the application
;				  can determine a course of action based upon them.
;	2013-0928	Added new "MkRndSeed" routine, which scans the entire RAM
;				  upon startup to create a new polynomial seed for the
;				  RND command.  MkRndSeed is part of the initialization.
;	2013-0924	Modified VPG routine to use a LUT to determine the page
;				  addresses of the various constants.  This method was
;				  needed because MCU builds of differing RAM sizes caused
;				  page cross-over issues.
;				Added WTF command, which 42 is the answer to.
;	2013-0923	Added support for the nRF24L01 series of 2.4GHz trans-
;				  ceivers.  See Command List for details.
;				Modified the default size of the data stack so that when
;				  the nRF24L01 support is enabled, the data stack increases
;				  to a depth of 32 bytes.
;				Modified RESTore routines to return the size of the data
;				  held in the DATA buffer.  This makes it easier to
;				  determine a payload size when using the nRF24L01
;				  routines since each payload received could be between
;				  1 and 32 bytes in length.
;	2013-0922	Updated host development system, which now uses GCC 4.5.3.
;				  GCC 4.5.3 introduces certain changes that caused the
;				  "optiboot" loader so expand byond 512 bytes.  Added
;				  conditional compiler-time option of "OS_main" instead
;				  of "naked" for main's attributes.
;	2013-0921	Added register "cmdparms", which is updated by the "pushU"
;				  and "popU" routines.  It is meant to be used as a rel-
;				  ative counter to determine if new data was added to the
;				  data stack after a call to the "InterpretLine" or
;				  "InterpretLineV" routines.
;------------------------------------------------------------------------
; Version 2.30
;	2013-0811	Whew!!  Lots of stuff!! Enjoy and please report the bugs!
;	2013-0809	Bugfix: TWI, TWR routine improperly interpreted a slave's
;				  DACK to the last byte of a write as a bus error.
;				  Fix: testing for proper status code depending on if
;				  the last byte or "Nth" byte.
;				Added code to insure register r1 is always "0" when
;				  the usb_serial_io routines are enabled and called.
;				Simplified RAM allocation for each routine if enabled by
;				  adding specific EQUATES for each routine's size.
;	2013-0808	Disabled DDS interrupts in ICP capture routines as the
;				  DDS interrupts caused severe inaccuracies in the timing
;				  routines.
;				Changed the manner in which variables are internally
;				  addressed; variables are now addressed as offset pointers
;				  rather than as the low-byte of a pointer.  Also corrected
;				  all routines that process variables.
;				Added BIN (') routine; same as HEX ($) routine but takes
;				  an 8-char binary number and converts to a single 8-bit
;				  number. Ex: '01010101 is $55.
;	2013-0806	Corrected line editing, program load/save, "new program"
;				  and LIST routines to use and/or check for the correct
;				  starting or ending location of program memory.  Previous
;				  versions inadvertently checked for top of PROGMEM + 2.
;				  Normally this wasn't a problem.as the data stack was
;				  positioned just above PROGMEM and it is unlikely there
;				  will ever be 11 bytes of data on the data stack. However,
;				  this version moves the variable space right above
;				  PROGMEM and thus variable "A" gets clobbered by the 1st
;				  line of a program.  Or if variable "A" is assigned a
;				  value then the 1st character of the 1st line gets
;				  clobbered and the RUN command aborts with an unknow
;				  command.
;				Corrected bug in "SPI_Init" routine where SPI_SS line was
;				  not being set high after the SPI port was initialized.
;				  This caused issues with SPI devices wherein the device
;				  was already selected when it should not have been.
;	2013-0803	Corrected THEN routine to compensate for 24-bit call/ret
;				  pointers on MCU's with more than 64KB of FLASH.
;	2013-0802	Added port pin inits for DSI routines; if enabled, set pins
;				  to inputs.
;				Added WDT timeout in DSI routines while monitoring
;				  pin-change events to prevent software lock-up in the
;				  event that the DS slave stops responding or isn't
;				  available.
;				Tested and verified proper operation of the DS routines.
;	2013-0730	On USB builds, added a timeout to the INIT routine, where
;				  if there is no host to initialize the USB interface
;				  within the timeout period the the USB interface is left
;				  uninitialized and AttoBASIC continues in a stand-alone
;				  mode just like its USART counterparts.  This is mainly
;				  so that the self-start feature may be used without
;				  needing a host USB to talk to.  To further support the
;				  stand-alone USB mode, also modified all serial I/O
;				  routines to check for the USB intereface being enabled
;				  and available before attempting to fetch or send
;				  characters through a potentially non-existent USB
;				  interface.
;	2013-0728	Cleaned up ADC and ADR funkyness.  ADR now enables the
;				  ADC and initiates the 1st conversion.  ADC now handles
;				  channels > 31 on MCU's that support it. Results of the
;				  ADC command can now be assigned to a variable.
;	2013-0727	Added command "IDUMP" to dump the contents of the MCU's
;				  I/O.
;				Bugfix: The STK500V2 bootloader does not handle watchdog
;				  timer resets and thus gets stuck in the bootloader.
;				  A newer, non-functional version has a fix for this.
;				  Modified the STK500V2 source to recognize the WDT as a
;				  valid reset source and jump to the application code.
;				  This issue affects the RST command on the ARDUINO
;				  Mega 2560 boards.
;				Added nesting of FOR-NEXT function.  Now supports
;				  nesting to 4 levels deep.  More generates a machine
;				  stack error.
;				Modified the "ADRcommand" routine to enable the ADC,
;				  which will set the reference and start the 1st con-
;				  version.
;	2013-0726	Renmamed "DStackPointer" to "DStkPtr".
;				Modified usage of "DStkPtr" so that it is an index that
;				  indicates the number of elements on the data stack
;				  instead of the low-byte address of the actual element
;				  in RAM.  Fixes page-crossing issues and reduced code
;				  size.
;				Modified usage of "GuSuStkPtr" so that it is an index that
;				  indicates the number of elements on the GOSUB stack
;				  instead of the low-byte address of the actual element
;				  in RAM.  Fixes page-crossing issues and reduced code
;				  size.
;				Moved "LNBFNH", "LN1TOSS" and "LNNONNC" flags out of
;				  XH registeras they are only used during line editing.
;				  "Code_Editing.inc" now use "local flags" held in the
;				  "outchar" register temporarily.
;				Moved "GSJAMR" from XH to XL as FNLNXT flag in XL is not
;				  used.
;				Moved "FNLEQ" flag from XL to XH.
;				Added "VPTRSTK" flag in XL to flag whether or not there
;				  is a variable with a pointer on the dstack or not.
;				  Set/cleared by the "ProcessVariable" routine.
;	2013-0722	Added nesting of GOSUB-RETURN function.  Now supports
;				  nesting to 4 levels deep.  More generates a machine
;				  stack error.
;	2013-0719	Bugfix: on Mega2560, which has more than 128KB of FLASH,
;				  all "(r)call" and "irq" address pointers are three (3)
;				  bytes instead of two (2).  Thus more stack space is
;				  needed.
;				Bugfix: in "OC0A_int", the frequency of updating the
;				  DHT busy timer was dependent on the value of the RTI
;				  command.  Fix: check the DHT busy timer every OC0A_int
;				  independent of RTI's setting.
;				Added DATA, READ and RESTore commands to support storing
;				  and retrieving data in a program.  There is no check
;				  for the statement being used in a running program so
;				  these statements can be used in interactive mode.
;				Changed register definitions for H, T, U and outchar so
;				 That T and U are aligned for 16-bit register operations.
;				Modified "getterminalline", "Countlinehchars",
;				  "storeline", "insertinmiddle" routines to use immediate
;				  pointer comparisons when working within the line buffer.
;				  This also removed the need to use registers "bufferlimL"
;				  and "bufferlimH", freeing them up for future use.
;	2013-0718	Bugfix: In some cases, after using the watchdog as an
;				  interrupt source, the WDTIRQ flag in GPIOR0 was left
;				  set causing the SLP command to prematurely end.  Fix:
;				  Cleared all interrupt source flags in GPIOR0 before
;				  entering sleep mode in the "SLPcommand" routine.
;				Added 8 second watchdog timeout in "Init" routine for
;				  USB builds.
;				Added the "VPG" command, which returns the page in RAM
;				  where the internal variables are stored.  It is
;				  intended to be used with as the page number when
;				  accessing the internal variables with the PEEK and
;				  POKE statements.
;				Added "@" command.  The "FETCHcommand" routine is used
;				  to access system internal variables.  Technically, it
;				  is a dummy command that simply passes control to the
;				  interpreter, which processes the next command in the
;				  buffer/program, which should be a variable name.
;				Added DHI, DHD, DTI and DTD commands, which return the
;				  low byte of the address of that internal variable in
;				  RAM.  They are intended to be used with the "VPG"
;				  command for accessing the internal variables with the
;				  PEEK and POKE commands.
;				Added the "RTC" command, which takes as its argument a
;				  number from 0 to 3, designating which of the four
;				  Real-time Counter bytes one wishes to access.  It is
;				  intended to be used with the "VPG" command for
;				  accessing the internal variables with the PEEK and
;				  POKE commands.
;				Added the "DFA" command, which takes as its argument a
;				  number from 0 to 1, designating which of the two
;				  DataFile sequential address counter bytes one wishes
;				  to access.  It is intended to be used with the "VPG"
;				  command for accessing the internal variables with
;				  the PEEK and POKE commands.
;				Added "FILL" command as part of the DEBUG routines.
;				  FILL is for filling RAM with a preset value.
;	2013-0716	Modified SPW, SPR and SPS commands to check that the SPI
;				  hardware was enabled before continuing, error if not.
;				  This prevents software hang when using those commands
;				  before issuing the SPM command.
;				Moved operator commands, LSR, LSL, AND, OR, XOR, COM and
;				  NEG commands into "Code_Operators.inc" to make it easier
;				  to find them.
;				Modified LSR and LSL operators to accept a 2nd parameter,
;				  which is the number of bits to shift.
;				Added SWAP command to swap the nibbles of the parameter.
;				Added the NBH and NBL commands to return the high and
;				  low nibbles of a paramter.
;				Compacted code by adding the "Get1stParm" routine since
;				  most commands always call the line interpreter and
;				  then pop the value off the datastack.  This routine
;				  only saves code space if the commands that use it are
;				  within the 2047 byte address range of it.
;	2013-0715	Modified sign-on message to include flavor-specific
;				  information, which cuts down on the amount of FLASH
;				  used.
;				Modified SStst (in "Code_Init.inc") to load program "0"
;				  from EFS, if support is enabled.
;				Bugfix: Disabled RXCIE in USART during "DHTread" routine
;				  as the CP2102 UART to USB converter would spew nulls
;				  into the receiver, which caused a flurry of IRQ's.
;				  The "DHTread" routine is only expecting pin-change
;				  IRQ's during reading of the sensor.
;	2013-0713	Added support for the ATmega2560 MCU, which includes
;				  support for ARDUINO MEGA2560 product (using USART0).
;				Bugfix: DHT routine will abort program execution upon a
;				  checksum error.  Changed "DHTRead" routine to zero both
;				  temperature and humidity readings as a sign that there
;				  was a checksum error detected from the DHT sensor.
;				Bugfix: "deletekey" routine did not properly handle a
;				  backspace character.
;				Modified RUN command to accept a file number as a parameter.
;				  If no parameter, run the current program.  If a
;				  parameter is supplied and it is a valid file number
;				  then the program is loaded then run.
;	2013-0712	Bugfix: in "Checkkeybd" routine, clearing the HALT flag
;				  then looking for ^C to be pressed twice was nearly
;				  impossible when using USB serial since the RUN commnand
;				  and the "emitchar" routine both called "Checkkeydb".
;				  This resulted in infinite loop programs being unable
;				  to be broken out of.  Fix: in "Checkkeybd" routine,
;				  removed clearing the HALT flag before checking for a
;				  key in case a prior call to "Checkkeybd" had already
;				  set the HALT flag before the RUN command could break
;				  program execution.  Also, ^C only needs to be pressed
;				  once to BREAK, not twice.
;				Added a second channel of PWM for both 8-bit and 10-bit
;				  resolutions.  This is valid for M32u4 and USB1286 parts.
;				  Only valid on M88/168/328 if the SPI and Data File
;				  rotines have been disabled.  Ex: PWM [x] [c].
;				Removed support for PWM on the OC0A pin.
;
;	2013-0711	Implemented limited file system to be able to SAVe and
;				  LOAd programs to/from EEPROM by program number.  The
;				  number of files available is determined by the EEPROM
;				  size of the device to a maximum of 8.
;				In supportof file system: added routine during INIT to
;				  look for blank EEP and set file handles and size to
;				  zero.
;				Renamed "Code_EEP.in" to "Code_EFS.inc".
;				Corrected compile-time issue with string generation
;				  where M88 and M168 caused the assembler to insert 0x00
;				  into the header strings thereby causing some strings
;				  to misprint.
;	2013-0709	Added the "EEW" command to write a byte to the EEPROM.
;				  If the address given is E2END+1 then then entire EEP
;				  is erased. Ex: "EEW 0 4 0" is 0x400 and will erase
;				  the entire EEP because E2END is 0x3FF on a Mega32U4.
;				Added the "EER" command to read a byte from the EEPROM.
;				Added "SetByteRegs" routine to consolodate address pointer
;				  retrieval from datastack for the PEEK, POKE, EER and EEW
;				  routines   This reduced code size.
;	2013-0706	Bugfix: DUMp and EDUmp routines; corrected last byte of
;				  sequence problem in EDUmp by moving check for last address.
;				Corrected printing of "0x0FFF" in DUMp and added byte
;				  index header.
;				Changed order of EDUmp and DUMp routines to dump from
;				  start of memory to end instead of end to start.
;				Added ^S checking in the DUMp and EDUmp routines to
;				  suspend printing the output.
;	2013-0705	Modified SAVe routine to use SLEEP mode while waiting for
;				  completion of atomic writes.  This was done as a means
;				  to save power as each write-byte takes about 4ms.
;				Modified INIT routine to check for the last byte of EEPROM
;				  being 0xFF, which indicates that the EEPROM is erased.
;				  If the last byte is erased, it is replaced with a 0x00,
;				  which signifies end-of-program.
;				Cleaned up coding on the DEBUG routines by consolodating
;				  printing of the value of Z register pair into one
;				  routine and adding header to show the byte index.
;	2013-0704	Modified SAVe routine to also check PCHI/PCLO when saving
;				  programs to EEPROM.  Once PCHI/PCLO have been reached,
;				  a null character is written too EEPROM to signify the
;				  end of the program.  This speeds execution when saving
;				  to devices with large amounts of EEPROM since only the
;				  program is saved.
;				Modified LOAd routine to check for null character in the
;				  data when loading programs from EEPROM, which signifies
;				  the end of a program.  This speeds execution when
;				  loading from devices with large amounts of EEPROM.
;				Insured that the port pin is set to input and reduced the
;				  code size in the ICP routines.
;				Bugfix: The FILLCNTR in the RAMFILL macro in "NEWPROGRAM"
;				  did not fill the last byte of PROGMEM.  Fix: add one to
;				  the FILLCNTR.
;	2013-0703	Modified USB Serial I/O library to acccess PROG_DATA
;				  residing in PROGMEM above 64K region using "morepgmspace.h"
;				  macros, which makes use of the RAMPZ register and ELPM
;				  instruction.
;				Modified "BLDRcommand" to make use of the RAMPZ register
;				  and ELPM instruction.
;	2013-0701	Updated USB Serial I/O library to V1.7 and modified calls
;				  to match accordingly.  Since RAM usage changed, modified
;				  order of variables held in RAM.
;				Modified USB serial I/O init routine to await for DTR to
;				  be asserted before continuing with sign-on message.
;				  This allows for one's terminal program to notify
;				  AttoBASIC that the user is ready and awaiting a prompt.
;				Changed INIT routine so that the system clock pre-scaler
;				  is always set to the value of FCLK_PS (see
;				  Defs_Constants.inc), this is partly due to Teensy++ 2.0
;				  having the DIV8 fuse programmed, yielding a CPU clock
;				  of 2MHz.
;				Added call to "usb_serial_shutdown" in RST command to insure
;				  proper disconnect from USB host when using the USB
;				  serial I/O builds.
;	2013-0629	Added support for TEENSY++ 2.0 HALFKAY bootloader by
;				  relocating USB serial I/O routines and DDS data below
;				  the 512 word BOOTLOADER address of the USB1286.  Use
;				  the TEENSY definition to make this custom build.
;	2013-0627	Enabled LUFA's BootloaderDFU.c to check for PORTF4
;				  (JTAG TCK) pin shorted to GND, which will
;				  invoke the DFU bootloader.  This is for platforms
;				  running with a Mega32U4 or USB1286 where a JTAG
;				  port may available on the target PC board.
;	2013-0626	Added support for AT90USB1286 (PJRC's TEENSY++ 2.0).
;				Added "USED" bytes to "FREE" command.  Also changed
;				  message content, which also reduced code size.
;				Bugfix: May not have affected MCU's with smaller RAM
;				  sizes; RAMFILL macro's pointers in "NEWPROGRAM"
;				  routine get destroyed due to ISR's overwriting them.
;				  Fix: disable interrupts during RAM fill.
;------------------------------------------------------------------------
; Version 2.22
;	2013-0619	Bugfix:  "DHTread" routine did not re-enable RTC IRQ's
;				  when a checksum or non-response is detected.
;	2013-0613	Corrected HELP command formatting error when using
;				  Mega32U4 with USB serial.  inbyte[h:l] were being used
;				  as character and line length counters, which on a
;				  Mega88/168/328 and Mega32U4 without USB build is
;				  acceptable.  However, those two registers are reserved
;				  on the Mega32U4 when USB is enabled as they are used
;				  to pass data to the usb_serial library routines.
;	2013-0611	Corrected TWI bit-rate generation conditional statements
;				  to correctly calculate the maximum available bit clock
;				  value based on the system clock frequency.  Added
;				  conditional assembly to the "TWIcommand" routine to
;				  support 400K and 100K selection if the system clock
;				  allows it, otherwise, only the maximum rate detected
;				  is allowed.
;	2013-0610	Added support for system clock prescaler.  Set this in
;				  the DEFS_CONSTANTS.INC file. This is mainly for use in
;				  testing AttoBASIC at different clock speeds but can
;				  be used normally if so desired.
;	2013-0607	Added support for the DHT11 and DHT22 temperature and
;				  humidity sensors.  Use DHT [x] (temperature), DHH
;				  (humidity), DHR and DHS commands,  Rounding is performed
;				  as well as sign for the DHT22 in Celsius readings.  See
;				  notes in the command list.
;	2013-0605	Bugfix: Moved the set flag (RTCIRQ in GPIOR0) in the
;				  "OC0A_int" routine out of the conditional loop to the
;				  end of the routine where it should be.
;				Updated the DDS register value to provide compensation
;				  for lost cycles due to interrupt processing.  Accuracy
;				  measured to less than 0.6% error.
;				Created "WDDisable" routine to allow global calls for
;				  commands that make use of the watchdog timer.
; ******		Updated DDS definitions to be device-specific.  There is
;				  no actual code change, just additional conditional
;				  assembly statements and definitions.
;
;
;------------------------------------------------------------------------
; Version 2.21
;	2012-0828	Corrected location of DDS routine's wave table.  The
;				 conditional assembly moved the wave table to the end of
;				 PROGMEM if the build did not contain a boot-loader, which
;				 was fine if one was going to use ISP to prgram the HEX file.
;				 However, if one is going to use a boot-loader then the
;				 wave table pointer is pointing to the boot-loader code,
;				 which causes eroneous waveform data.  The conditionals now
;				 always assume there is a boot-loader and the wave table is
;				 located at the beginning of the page below it.
;				Now setting DDS sample rate to FCLK/256 to accomodate for
;				 various clock speeds.  With a constant rate, the RTC ISR
;				 would interfere with the DDS ISR routine.
;				Corrected FCLK definition for AVR Data Recorder, whereas
;				 FCLK was always assigned a value of "8000000" reguardless
;				 of its real value.
;
;========================================================================
; Version 2.20
;	2012-0815	Added "LDDcommand", which loads the default capture
;				 and record program for the Data Recorder.
;				Bugfix: Corrected bug in "diffcommand" routine, which
;				 is actually the "<" and ">" operators, where a half-
;				 carry produced an incorrect result.
;	2012-0808	Added "ACI" command to enable/disable analog comparator
;				 interrupts.  This is mainly for use with the "SLP 0"
;				 command.  ACI only supports inputs on AIN+ and AIN-.
;				Added "ACS" command to select source to the analog
;				 comparator's "-" input.  Either AIN-, ADC0 or ADC1.
;				Modified "ACOcommand" routine to select AIN- source
;				 based on the value selected by the "ACS" command.
;	2012-0807	Bugfix: Corrected bug in "peekcommand" routine where
;				 there was an extra call to "interpretlinev" at the
;				 end of the routine, which added erroneous data to the
;				 stack.  Removed the extra call.
;				Bugfix: Corrected bug in the "DDScommand" routine that
;				 falsely detected whether the command line parameter was
;				 a "0" or a "1".
;	2012-0805	Added a WORDS command to list the available commands.
;				Modified operators; "=", "!=", "<" and "<" to return a
;				 "1" for a "true" condition or a "0" for a false condition.
;				Inverted the detection logic of the "IFcommand" routine
;				 to accomodate the modified operators.
;				Changed the "ACO" command to return the actual state of
;				 the ACO bit.  Was inverted in prior versions.
;				Combined the "lessthancommand" and the "greaterthancommand"
;				 routines to become the "diffcommand", which determines
;				 which comparision to perform then performs it.
;	2012-0803	Added "DIG" command to directly control the N-MOSFET on
;				 the AVR Data Recorder.
;	2012-0730	Shortened some error messages.
;				Added support to the "PRINT" command to print strings
;				 embedded in quotes. Supports sending CR/LF (the "~"
;				 character) embedded within the string.
;				Modified "getterminalline" routine to allow use of the
;				 "#" character embedded in stings.  Normally, any
;				 characters following a "#" will be stripped.
;				Modified certain operator to single-character commands;
;				 AND = "&", OR = "|", XOR = "^", and NEG to "!".  This
;				 change reduces the amount of program memory needed for
;				 the command while also conforming to "C conventions".
;				Changed the "<>" (not equal) operator to "!=".  Done
;				 to conform to "C conventions".
;				Modified "RUNcommand" routine to store the currently
;				 executing line #, which is stored in RAM@CurrLine.  This
;				 aids in tracing program statement errors, which are now
;				 printed along with the error message.
;				Added routine "GETPROGLINENO" to support the aforementioned
;				 error message.
;				Added "RUNACT" flag for the use of the "GETPROGLINENO"
;				 routine.  The "RUN" command sets it then resets it
;				 after program ends.
;	2012-0729	Bugfix: When "BIGRAM" is disabled, the "LOADcommand"
;				 incorrrectly calculates the low-pointer based on the
;				 assembly-time constant of "dstacktop".  Cleaned up
;				 "SAVEcommand" and "LOADcommand" routines to use
;				 16-bit macros for loading and comparing pointers.
;				Re-ordered "LOADcommand" routine; execute NEWprogram,
;				 print "loading from EEPROM" then execute "FREE" command.
;	2012-0728	Due to sampling inaccuracies with the "ADC" command,
;				 modified the routine to average 10-bit samples then
;				 convert to an 8-bit result before saving on the data stack.
;				Added modulus operator, "%". Why?  The remainder is already
;				 available from the "DIVIDEcommand" routine, so why not?
;	2012-0726	Shortened some of the error messages to conserve string
;				 space.
;				Disabled BTL command for the Data Recorder as it is
;				 very messy to implement. :)  However, the bootloader code
;				 is still generated in specific builds and will be invoked
;				 upon a RESET if the BOOTRST is set.
;	2012-0723	Added specific commands for the "AVR Data Recorder".  If
;				 enabled, conditional assembly is performed to enable
;				 selecting channel #'s, setting gain and the AD536's output
;				 type (TRMS or dBV).
;	2012-0714	Bugfix: "subtract" routine issued an "arithmetic underflow"
;				 when a operation resulted in a half-carry.  The "brvs"
;				 instruction should have been a "brcs" instruction.  This was
;				 the result of adding the "AOV" command from V2.00.
;	2012-0711	Modified "hexcommand" routine to detect 1 or 2 characters.
;				 Therefore, when entering only 1 character no longer returns
;				 erroneous results.
;				Bugfix: Corrected Z-ptr overflow problem in the "Error" routine
;				 where adding the message offset to ZL may result in a carry
;				 to ZH that was not accounted for.  Changed to a 16-bit add.
;				Bugfix: When the "SELFSTRT feature is activated, any console
;				 I/O would be block because global interrupts were not enabled
;				 after the serial I/O hardware was initialized.  Added "sei"
;				 after serial I/O initialization in the "Code_Init.inc" routine.
;				Bugfix: For Mega168/328 using the Optiboot loader; modified
;				 the BASH script to correctly fill unused bytes between addresses
;				 in the HEX image.  There are two bytes required to be placed
;				 as the last two bytes in the HEX file but were placed right
;				 after the code.
;	2012-0709	Modified DUMP command to dump all of RAM contents instead
;				 of just PROGRAM memory.
;				Added CPU Clock frequency in the sign-on and "NEW" messages.
;				 Works for integer frequencies of 1MHZ+, no fractionals.
;	2012-0629	Bugfix: Added conditional compile directives in "usb_serial.h"
;				 to properly set the PLL for using 8MHz or 16MHz CPU clock.
;				 V2.11 did not set this up properly and using the USB_Serial
;				 I/O rouines at 8MHz yielded an unrecognized USB device..
;				On Mega32u4 with USB_Serial I/O; modified variable locations
;				 in RAM so as to allow more room for programs with the
;				 BIGRAM option set.
;	2012-0624	Added "XB[p] [n]" command to toggle bit "n" on port "p".
;				Pre-release bugfix: "OCR0A" ISR caused an intermittant
;				 error, which resulted in halting program execution with
;				 various single errors displayed.  The fix was to save the
;				 state of SREG and restore it upon exit of the ISR. Duh!
;				Pre-release bugfix: When RTC routines are enabled, the
;				 SLP command would randomly "hang" waiting for a WDT
;				 interrupt to occur.  Any other interrupt would awaken
;				 the AVR's "sleep" instruction and program execution
;				 would continue.  This seemed to be attributed to the
;				 SLP command routine not being keenly aware that it
;				 should only respond to a WDT interrupt when required.
;				 Resolved this by adding a set of flags in GPIOR0, which
;				 are set by their correspondng routines and cleared by
;				 the SLP routine, thus effectively ignoring the DDS and
;				 RTC interrupts.
;	2012-0621	Added "RST" command, which invokes the watchdog timer
;				 in "System Reset" mode to cause a hardware reset
;				 of AttoBASIC.
;				Added "Real-time Counter" interface routines.
;				Pre-release bugfix: moved "inc dstakpointer" to proper
;				 location.  Was inadvertently moved during register
;				 re-assignment, which caused intermittant data stack
;				 errors.
;	2012-0531	Bugfix: The ADC command had a "default" value, which is
;				 PUSHed onto the stack before acquiring a user-supplied
;				 value. Variable assignment of the value returned from
;				 the ADC command would cause a misalignment in the datastack
;				 pointer and the default value would be returned instead
;				 of tha actual value provided by the user.  Removed the
;				 ability to use a "default" channel with the ADC command.
;				Bugfix: Corrected ADC bug whereas the Mega328 was not
;				 actually supported in conditional assembly.
;				Added "Data File" interface routines.  Presently, the
;				 routines support the MICROCHIP 25AAxxx series of Serial
;				 EEPROM devices using the SPI interface.  Address range is
;				 16 bits or up to 65,536 storage locations.  The intent
;				 behind this added functionality is for use as a data-
;				 recorder when used with the "SLP" instruction.  Eventually,
;				 these routines could be used with SD memory cards as they
;				 also use the SPI interface.
;				Added "#" sign, which tells the line interpreter to ignore
;				 all characters after the "#" until a CR is found.  This
;				 aids in commenting programs when one is editing and
;				 uploading them via a serial terminal.
;	2012-0530	Bugfix:  Line parsing routine improperly inserted and
;				 deleted lines.  This was caused in V211 while optimizing
;				 relative branches.  Specifically, in "Code_Editing.inc",
;				 the routing named "noinsertlowerline" was a fall-through
;				 from the "notattop3" label.  When it was moved, a jump
;				 was not inserted to compensate ("rjmp noinsertlowerline).
;				Modified DSI routines to reduce the number of instructions
;				 by implimenting the existing "Delay1mS" routine in place
;				 of the "shortdelay" routine, which is a software loop
;				 delay dependant on the CPU clock frequency.  The "Delay1mS"
;				 routine is independant of CPU clock frequency.  Code
;				 reduction is 16 bytes (helpful on Mega88).
;				Added "SLP N" instruction.  When executed, the CPU enters
;				 the power-down mode. The routine is exited once the
;				 "waking event" has ocurred.
;	2012-0311	Added "BIGRAM" conditional assembly to allow larger
;				 program storage.  Caveat: Programs larger than EEPROM
;				 cannot be saved and will be truncated!  This option is
;				 good for when one wishes to save and load programs via
;				 a terminal emulator.
;				Changed SIZE command to print in bytes instead of pages
;				 and characters.  Easier for we humans to read. :)
;========================================================================
; Version 2.11
;	2012-0307	Bugfix: storage of programs does not exceed 1st page.
;				 This was due to a bug introduced when relative branches
;				 in the line edting routines overflowed and reverse com-
;				 pares were used in the "noinsertlowerline" routine.  The
;				 routine was corrected and relocated to remove the branch
;				 errors.
;========================================================================
; Version 2.1
;	2012-0315	Bugfix: Corrected bug in "IBx" command.  Logic was reversed
;				 whereas if the bit tested was high, a "0" was returned on
;				 the data stack and versa-vise.
;	2012-0221	Added additional routines to BLDR command; the command is
;				 always enabled on the appropriate devices that support a
;				 a boot-loader.  However, the BOOTSZ1:0 fuse bits are
;				 used to determine the location of a boot-loader then a
;				 check for the existence of a valid boot-loader jump table
;				 is checked before invoking the boot-loader.  If a boot-loader
;				 is not found, then an error message is displayed.
;	2012-0220	Added conditional assembly to support PEEKs and POKEs
;				 within entire 64K of data space by passing the page #
;				 and index into the page.  Was limited to page zero only.
;				 Calling format is "page,index".  If assembled for page
;				 zero only, the address byte will always point within page
;				 zero and any additional values will be ignored.
;	2012-0218	Changed "Init" routine to enable port pull-up, check pin
;				 then disable port pull-up if "Self-Start" feature is
;				 enabled.  This allows for an ARDUINO user to use AttoBASIC
;				 without needing to be concerned about the "SS" feature
;				 loading a program from E2P when not desired.
;	2012-0213	Added conditional assembly to support USB Serial I/O and
;				 USB LUFA DFU Boot-loader for Mega32U4 running at 8MHz
;				 and 16MHz.
;				Wrote Linux BASH shell script to support automagic creation
;				 of all processors, clock speeds and if available, USB and
;				 boot-loader code inclusion.
;	2012-0209	Added software version code to the sign-on message.
;				Changed sign-on string to reflect version with build CPU
;				 on separate line.
;	2012-0208	Added enhanced printing support fo internal messages.
;				 Specifically for printing centered text and repeated
;				 characters.  This decreases FLASH spaced used to hold
;				 space and repeating characters.
;				 If the first character of a string contained in PROGMEM is:
;				  - 0xF8 then the line length is calculated and centered
;				    by prefixing spaces before the string.
;				  - 0xF9 then the next sequential number is the number of
;				    times to repeat the following character.
;				  - 0xFA emits a CR/LF combination.
;				  The constant defined as "LNLNGTH" contains the system-wide
;				  max length of system message strings.
;				Cleaned up system message string usage conserving ~400 bytes
;				 of PROGMEM space.
;				Cleaned up use of conditional statements in "Data_Msgs,inc".
;				Corrected message order of E2PROM "loadcommand".
;				Added USB buffer overflow check.
;	2012-0207	Added new command "BLDR" to directly call the bootloader.
;				 Currently, this is only functional on the Mega32U4 (with
;				 USB serial I/O enabled) as the LUFA DFU bootloader can be
;				 enabled via a DEFINE directive.
;				Added support for Mega328, which many ARDUINO's use.
;				Added code to disable the watchdog timer at INIT time.  If
;				 using the ATMEL FLIP DFU application to "start the app"
;				 and the "RESET" is checked, the watchdog timer is enabled
;				 by the bootloader and a watchdog timeout is used to reset
;				 the MCU.
;				Bugfix: Regarding "PopU" and "PushU" routines, which
;				 occured when "dstackpointer" was moved to a low register.
;				 "temp" register is now saved and restored in each of those
;				 routines.
;	2012-0205	Cleaned up code; compacted some operations and verified
;				 code functionality since committing PC and GOSUBRETL to RAM.
;	2012-0203	Added support for USB via built-in hardware controller.
;				 The CDC protocol (virtual serial port) is used.
;				 USB library adds 3022 bytes.  Only available on M32U4.
;				Moved registers inbyteh:inbytel to R25:R24 for inter-
;				 facing to USB library routines.
;	2012-0130	Modified registers; moved dstackpointer to R12, ecode to
;				 R13, PCHI/PCLO and GOSUBRETH/GOSUBRETL to RAM.  Adjusted
;				 code accordingly.
;
;========================================================================
; Version 2.0
;	2011-1020	Bugfix: Corrected bug in SB[x] and CB[x] commands.  Wrong
;				 port offset was used.
;	2011-0606	Compacted PORT I/O commands to use one routine for each
;				 command variation.  I.e. 'OPB' and 'OPD' use the same
;				 routine as the desired port is contained within the
;				 3rd letter of the command being executed (trimed 574B).
;				Added assemble-time directive to enable/disable "PIND0
;				 test".  See DEFS_CONSTANTS.INC file.
;	2011-0525	Bugfix: Corrected ADC command to dummy read ADC before
;				 returning the correct result.
;				Enabled support for ADC command to take up to 64 channels
;				 as Mega32U4 supports as such.
;	2011-0524	Added support for ATmega32U4.  Change the DEF file to
;				 select between Mega88/168 and Mega32U4.  USB functionality
;				 not yet provided.  Serial I/O only.  Note: the XML and DEF
;				 files included with AVRStudio 4.18 SP3 are not complete
;				 for the Mega32U4.  The TWI module is missing.
;	2011-0518	Added command separator for program lines.  The command
;				 separator is the semicolon (";").  Thus, muliple commands
;				 can be on the same program line.  This is helpful for using
;				 the TWI commands for example.
;	2011-0517	Added feature enabling support so that a user can
;				 enable/disable certain command features if not
;				 needed.  See DEFS_CONSTANSTS.INC file.
;				Changed UART receive handler; MPU now enters sleep
;				 mode until a character is received.  Note: any interrupt
;				 wakes from sleep but the 'recvchar' routine checks for
;				 data in RX buffer.  If no data is ready, it goes back
;				 to sleep.  This allows for other interrupt-driven
;				 routines (such as DDS output) to function as well and keeps
;				 the MPU running in a low-power state when waiting for
;				 user input.
;				Added D[irect] D[igital] S[ynthesis] feature as follows:
;				'DDS [x] - Outputs a frequency on the defined port pin
;				 at the 6-BCD-digit frequency held in the X/Y/Z variables.
;				 The DDS sample frequency is set to twice the Interrupt
;				 service routine's duration, which is 5uS.  Therefore, the
;				 output frequency range will be 0 to 25KHz in 1Hz steps.
;				 X = 0 to disable DDS and x = 1 to enable [X/Y/Z set first].
;				 Without X same as 0 [disable].
;				Added version coding to the signon prompt.
;	2011-0515	Added routines to support TWI/I2C interface as follows:
;				TWI [x] - must be rcalled first to initialize the TWI
;				 interface. X = 0 for 400Kbps and x = 1 for 100Kbps clock.
;				 Without [x] is same as x = 0. Defaults to Master
;				 @ 400Kbps with PORT pull-ups enabled.
;				TWS - Assert a START condition on the bus.
;				TWP - Assert a STOP condition on the bus.
;				TWA [x] - TWA sends the slave address to the bus.
;				 Returns with the bus status on the stack.
;				TWW [x] - TWW sends a byte to the bus.  Returns with
;				 the bus status on the stack.
;				TWR [x] - Receives a byte from the TWI bus and places
;				 it onto the stack.  x = 0 to signal to the slave that
;				 this is the last byte to receive, x = 1 to signal to
;				the slave there is more data to receive. Without [x]
;				is same as x = 1.
;				TWB - Queries the TWI status register for the last
;				 detected condition of the bus.  [Note: the byte returned
;				 is right-shifted 3 bit positions. If a STOP condition
;				 has been detected, $80 is returned to indicate so.
;				Restructured error code lookup table.  Saved 54 bytes for
;				 17 error messages.
;				Restructured error code routine.  Register ecode conflicts
;				 with any error codes over $12.  Implimenting some of the
;				 additional commands required additional error codes .  The
;				 error code is now kept in RAM @ERRCode separate from
;				 the input parsing routine.
;				Added '*' and '/' commands for 8-bit multiply and divide
;				Added underflow and overflow detection to arithmetic
;				 commands with error messages.
;				Added 'AOV [x]' command to enable arithmetic overflow
;				 and underflow detection where x = 1 enabled error and
;				 x = 0 disables detection.  Defaults to x = 1.  Note that
;				 when detection is disabled, the result from an arithmetic
;				 operation will return the 8-bit result.  Expect errors
;				 if not careful!
;				Added 'ID[p]" command to return the value of the DDR[p]
;				 register.
;	2011-0514	Fixed input line parsing routine; ^H and BACKSPACE
;				 now actually erases the character.
;				Fixed input line parsing routine; ignores linefeed
;				 characters ($0A) when they are seen.  This allows for
;				 uploading programs using a terminal emulation program.
;	2011-0513	Added new commands to support input capture on ICP1.
;				'ICG [x]' - Initializes ICP mode and sets Input Capture
;				 gate time to x[0..7] where x is optional (default 0):
;				  = 0 is disable ICP.
;				  = 1 is   10mS gate time
;				  = 2 is   25mS gate time
;				  = 3 is   50mS gate time
;				  = 4 is  100mS gate time
;				  = 5 is  250mS gate time
;				  = 6 is  500mS gate time
;				  = 7 is 1000mS gate time
;				'ICE [x]' - optionally sets the edge select.  Where
;				 x = 0 for falling and 1 for rising.
;				'ICP' - returns the low byte value and stores the
;				 high byte in variable 'Z'.  'Z' is clobbered.
;				 Returns an error if there is a 16-bit overflow
;				 (and clears 'Z').
;				Added ability to select PWM channel between OC0A and
;				 OC1A during program assembly. If OC0A is used, the
;				 10-bit 'PWE [x]' command is disabled.
;				 Code can be easily mod'ed to switch back and forth.
;	2011-0512	Added size check to EEP save routine. Will only save
;				 if the current program size + datastack fits into EEP.
;				 Stops saving when EEP bottom is reached because
;				 wrap-around sux!
;				Added/modified ADC commands as follows:
;				'ADC [x]' - supports channels [0..15].
;				'ADR [x]' - optionally set INT/EXT ref.
;				 where x = 0 for INT and x = 1 for EXT.  Without [x] is
;				 same as x = 0, int. ref
;				Added RND command to generate 8-bit random #.
;	2011-0511	Added routines to support the SPI interface.  New CMD's:
;				'SPM [x]' - MUST be rcalled first.  Initializes the SPI
;				 to operate in Mode [0..3] (see data sheet).
;				 Defaults to Master, F_CLK/16, MSB first.
;				'SPO [x]' - optionally set MSB/LSB mode
;				 where x = 0 for MSB and x = 1 for LSB.
;				'SPC [x]' - optionally set SPI_clk to [0.15].  See
;				 Pg 168 of data sheet for actual code meanings.
;				'SPW [x]' - write byte to SPI.  Note that SPI_SS pin is
;				 set low when this cmd is executed and not restored so
;				 user must toggle pin high with the SPS cmd.
;				'SPR'	  - read byte from SPI. Note that SPI_SS pin is
;				 set low when this cmd is executed and not restored so
;				 user must toggle pin high with the SPS cmd.
;				'SPS [x]' - set SPI_SS pin to [x]. Defaults to '1'
;				Added EDUMP command to dump the contents of EEP.
;				Added VDUMP command to dump the contents of the
;				 26 variables [A..Z].
;	2011-0510	Ported from M163 to M88/168.
;				No PORTA on M168 so code.
;				Split out Register defs and constants to separate files
;				 to ease in reading of source code.
;				Modified greeting to include build date and Scott's
;				 copyleft info.
;				Changed DELAY routine to be F_CLK aware and thus
;				 transparent to the end user.
;
;************************************************************************
;.nolist
.include "Include/Defs_MCU-Options.inc"	;!!!Start Here to define your flavor
;
#if defined(__AT90S2313__)
	#message AT90S2313 selected.
#elif defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	#message ATtiny2313[A] selected.
#elif defined(__AT90S8515__)
	#message AT90S8515 selected.
#elif defined(__ATmega16__)
	#message ATmega16 selected.
#elif defined(__ATmega16L__)
	#message ATmega16L selected.
#elif defined(__ATmega163__)
	#message ATmega163 selected.
#elif defined(__ATmega32__)
	#message ATmega32 selected.
#elif defined(__ATmega32A__)
	#message ATmega32A selected.
#elif defined(__ATtiny84__) || defined(__ATtiny84A__)
	#message ATtiny84(A) selected.
#elif defined(__ATtiny85__)
	#message ATtiny85 selected.
#elif defined(__ATmega88__) || defined(__ATmega88PA__)
	#message ATmega88[P] selected.
#elif defined(__ATmega168__) || defined(__ATmega168PA__)
	#message ATmega168[P] selected.
#elif defined(__ATmega328__) || defined(__ATmega328P__)
	#message ATmega328[P] selected.
#elif defined(__ATmega32A__)
	#message ATmega32A selected.
#elif defined(__ATmega32U4__)
	#message ATmega32U4 selected.
#elif defined(__ATmega644P__)
	#message ATmega644P selected.
#elif defined(__ATmega1284P__)
	#message ATmega1284P selected.
#elif defined(__AT90USB1286__)
	#message AT90USB1286 selected.
#elif defined(__ATmega2560__)
	#message ATmega2560 selected.
#else
	#error Invalid MCU or none defined!
#endif
;
; If 2313 is selected, disable all other code generation
#if defined(__AT90S2313__) || defined(__ATtiny2313__) || defined(__ATtiny2313A__)
  .include "Include/Code_at90s2313.inc"	;
; If AT90S8515 is selected, disable all other code generation
#elif defined(__AT90S8515__)
  .include "Include/Code_at90s8515.inc"	;
; Mega16 or Mega32 support now included in AttoBASIC V2.34
; If mega16 or mega32 is selected, disable all other code generation
;#elif defined(__ATmega16__) || defined(__ATmega16L__) || \
;	defined(__ATmega32__) || defined(__ATmega32A__)
; If mega163 is selected, disable all other code generation
#elif defined(__ATmega163__)
  .include "Include/Code_mega163.inc"	;
#else
.include "Include/Defs_Constants.inc"	;
.include "Include/Defs_USART.inc"		;Hardware USART support for all flavors

;**** Add ATtiny84/85 API definitions ****************************************
#if BTLDR && ( defined(__ATtiny84__) || defined(__ATtiny84A__) )
; #if USI
; 	.include "Include/Defs_usi_uart-attiny84.inc"	;API defs for USI support
; #else
; 	.include "Include/Defs_vuart-attiny84.inc"		;API defs for SoftUART support
; #endif
	#message "Using BootLoader API's for UART"		;use API defs from bootloader
	.include "Include/Defs_optiboot85-attiny84.inc"
#elif BTLDR && defined(__ATtiny85__)
; #if USI
; 	.include "Include/Defs_usi_uart-attiny85.inc"	;API defs for USI support
; #else
; 	.include "Include/Defs_vuart-attiny85.inc"		;API defs for SoftUART support
; #endif
	#message "Using BootLoader API's for UART"		;use API defs from bootloader
	.include "Include/Defs_optiboot85-attiny84.inc"
#endif
;
;**** Add ATmega32U4 and AT90USB1286 API definitions ****************************
#if USB && defined(__ATmega32U4__)
  .include "Include/Defs_usb_serial-atmega32u4.inc"	;API defs for USB support
#elif USB && defined(__AT90USB1286__)
  #if TEENSY
   .include "Include/Defs_usb_serial-TEENSYPP20.inc"	;API defs for TEENSY USB support
  #else
   .include "Include/Defs_usb_serial-at90usb1286.inc"	;API defs for USB support
  #endif
#endif
;
;**** Macro definitions **********************************************************
.include "Include/Macro_IO.inc"
.include "Include/Macro_Misc.inc"
.include "Include/Macro_Delays.inc"
.include "Include/Macro_AttoBASIC.inc"	;AttoBASIC specific macros
;
.list
;
;******************************************************************************
;Define RAM/EEPROM Storage Variables
;******************************************************************************
.include	"Include/Data_RAM.inc"
.include	"Include/Data_EEP.inc"
;
;************************************************************************
;Start of code space here...
;************************************************************************
;set up interupt vectors
.include "Include/Code_Vectors.inc"
;***************************************************************
;
#if !defined(__ATtiny84__) && !defined(__ATtiny84A__) && \
	!defined(__ATtiny85__) && \
	!defined(__ATmega88__) && !defined(__ATmega88PA__)
;***************************************************************
; Since the reserved data space must be page aligned, it must
;  be strategically placed.
;***************************************************************
 .if SPGM
  .include "Include/Data_SPM.inc"	;Reserve data space in FLASH
 .endif
;***************************************************************
#endif
;
;***************************************************************
; RESET: This is the reset vector. Every thing starts here!!
;***************************************************************
RESET:
	wdr							;reset the watchdog in case it is enabled
#if BTLDR && (defined(__ATtiny84__) || defined(__ATtiny84A__) || \
			  defined(__ATtiny85__))	;special case for Tiny84/85 bootloader
;	CPI16	YH,YL,0xB007		;is YH:L = 0xB007?
;	breq	RESET1				;yes, skip bootloader
	LOAD	temp,MCUSR			;fetch RESET source
	sbrc	temp,EXTRF			;skip next if EXTRF bit clear
	Goto	Optiboot			;no, invoke bootloader leaving MCUSR alone!
#endif
RESET1:
	InitSP						;init stack pointer to RAMEND

;.if SPGM
;	GoSub	SPM_test
;.endif

RESET2:
	GoSub	Init				;initialize machine

#if !TEST
	GoSub	TypeGreeting		;print signon greeting
#endif
;
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || \
	defined(__ATmega88__) || defined(__ATmega88PA__)
;***************************************************************
; Since the reserved data space must be page aligned, it must
;  be strategically placed.
;***************************************************************
.if SPGM
.error "Help!"
  .include "Include/Data_SPM.inc"	;Reserve data space in FLASH
.endif
;***************************************************************
#endif
;
;***************************************************************
main:
	sei							;insure unterrupts enabled
	GoSub	crlf				;send a CR/LF combo to the console
	INITDSTACK					;Initialize data stack pointer
	GoSub	getterminalline		;Get line from terminal (60 chars + $0D)
	GoSub	Interpretelinebuffer;Interpret the line in the buffer
	Goto	main				;loop for more
;
;=============================================================
; getterminalline: 	;Get characters from terminal into
;	linebuffer. Stop accepting chars except 0x0D when end of
;	buffer is reached.
;=============================================================
getterminalline:
#if !defined(__ATtiny84__) && !defined(__ATtiny84A__) && \
	!defined(__ATtiny85__)
 #if!(USB || USI)	;if using USART insure RX Interupts, enable TX and RX
 	ldi		outchar,(1<<RXCIE|0<<TXCIE|0<<UDRIE| \
					1<<RXEN|1<<TXEN|0<<UCSZ2)
	STORE	UCSRB,outchar
 #endif
#endif
	ldi		outchar,PROMPT		;display prompt
	GoSub	emitchar

	SetYPtr	LNBUFF+LNBUFFSZ-1	;Initialize line buffer pointer to first char
anothertermchar:
	GoSub	recvchar			;Get char from terminal
	cpi		inchar,$0D			;is it a CR?
	brne	anothertermchar1	;branch if no CR
	CLRB	GPIOR0,IGNCHAR,temp2;clear the IGNORE CHARACTERS flag
	CLRB	GPIOR0,STRGDET,temp2;clear the STRING DETECTED flag
	Goto	anothertermchar3	;CR, skip ahead
anothertermchar1:
	cpi		inchar,'"'			;Is it '"' character?
	brne	anothertermchar2
	SKBC	GPIOR0,STRGDET,temp2;skip next if STRING DETECTED flag clear
	rjmp	anothertermchar1a	;flag set, clear the STRING DETECTED flag
	SETB	GPIOR0,STRGDET,temp2;set the STRING DETECTED flag
	rjmp	anothertermchar2	;continue
anothertermchar1a:
	CLRB	GPIOR0,STRGDET,temp2;set the string detected flag

anothertermchar2:
	cpi		inchar,'#'			;Is it '#' character?
	brne	anothertermchar3
	SKBS	GPIOR0,STRGDET,temp2;skip next if STRING DETECTED flag set
	SETB	GPIOR0,IGNCHAR,temp2;set the IGNORE CHARACTERS flag

anothertermchar3:
	CPI16	YH,YL,LNBUFF+LNBUFFSZ;end of buffer?
	brne	anothertermchar4	;no, continue
	ldi		outchar,BELL		;If not CR at last position, ring the bell
	GoSub	emitchar
	Goto	anothertermchar
;
anothertermchar4:
	cpi		inchar,CTRL_U		;Is it ^U character?
	breq	deleteline			;yes, process it
anothertermchar4a:
	cpi		inchar,CTRL_H		;Is it backspace (^H) character?
	breq	deletekey			;yes, process it
	cpi		inchar,DELETE		;Is it backspace character?
	breq	deletekey			;yes, process it
	cpi		inchar,LF			;Is it linefeed character?
	breq	anothertermchar		;ignore linefeeds and get another character
	cpi		inchar,CTRL_I		;Is it TAB character?
	breq	anothertermchar		;ignore TABS (corrupts our programs!) and get another character
	SKBS	GPIOR0,IGNCHAR,temp2;skip next if IGNORE CHARACTERS flag set
	st		Y,inchar			;Put in line buffer
	cpi		inchar,CR			;check for CR again
	brne	anothertermchar5	;ranch if not CR
	ret							;Last char in line received
anothertermchar5:
	CPI16	YH,YL,LNBUFF		;end of buffer?
	brne	notlbuffend			;yes, don't store char but beep.
	ldi		outchar,BELL		;If not CR at last position, ring the bell
	GoSub	emitchar			;send a BELL character
	Goto	anothertermchar		;loop for more characters
notlbuffend:
	SKBS	GPIOR0,IGNCHAR,temp2;skip next if IGNORE CHARACTERS flag set
	SUBI16	YH,YL,1				;Not end of buffer and not CR, so emit and go another
echoandgo:
	mov		outchar,inchar
	GoSub	emitchar0d
	Goto	anothertermchar
;
;========================================================================
; DeleteLine: Backup up cursor and erase line with a destructive
;	backspaces.
;	- Enter with Y pointing to the last address of the characters in the
;		line buffer.
;========================================================================
DeleteLine:
	PUSHZ						;save Z-Ptr
	SetZPtr	LNBUFF+LNBUFFSZ-1	;point to end of buffer less 1
	SUB16	ZH,ZL,YH,YL			;subtract for byte count
	mov		bytecntr,ZL			;seed byte counter
DeleteLine1:
	SETZPTR (2*clrchar)			;Load Z-ptr w/ ^H,space,^H string
	GoSub	sendromstring		;send the string
	dec		bytecntr				;decrement counter
	brne	DeleteLine1			;loop till zero
	SetYPtr	LNBUFF+LNBUFFSZ-1	;Re-init the line buffer pointer
	POPZ						;restore Z-Ptr
	Goto	anothertermchar
;
;========================================================================
; DeleteKey: Backup up cursor and erase character with a destructive
;	backspace.
;	- Enter with Y pointing to the address of the character to be deleted
;		plus 1.
;========================================================================
DeleteKey:
	CPI16	YH,YL,LNBUFF+LNBUFFSZ-1;end of buffer?
	breq	DeleteKey1			;no, within limits of the line buffer
	clr		inchar
	st		Y+,inchar			;Put in line buffer
	SETZPTR (2*clrchar)			;Load Z-ptr w/ ^H,space,^H string
	GoSub	sendromstring		;send the string
DeleteKey1:
	Goto	anothertermchar
;
;***********************************************************************
;Interpretelinebuffer:	This is the main entry point for the interpreter
;	executing in immediate mode.  The command line is interpreted from
;	then line buffer starting at "LNBUFF" in RAM and groeing down.
;***********************************************************************
Interpretelinebuffer:		;Interpret line
	SetYPtr	LNBUFF+LNBUFFSZ-1;Initialize line buffer pointer to lbufftop

	ld		inchar,Y		;Get first char from linebuffer
	GoSub	qcast			;the the cast of the character
	cpi		temp,QC_NUM		;is it a numeral (ecode in temp)
	brne	dotheline		;First char is not a numeral so interpret line
	Goto	storeline		;If it's a number, then it's a line number -store it.
dotheline:
	GoSub	crlf
	ori		XL,(1<<VARSTK)	;Flag variable pointers to be left on stack
	rjmp	interpretline
;
;***********************************************************************
;InterpretLineV:	This is the main entry point for the interpreter
;	during "RUN" mode.  This routine is usually called by the AttoBASIC
;	commands to parse and interpret the remaining characters of the
;	program line.
;	- It is used to place data on the data stack or execute another
;		command to place its return data on the stack.
;	- This code is rentrant to the depth of the AVR's hardware stack and
;		the interpreter's data stack.
;***********************************************************************
InterpretLineV:
	andi	XL,~(1<<VARSTK) 	;clear VARSTK flag
	andi	XH,~(1<<NOEXEC) 	;clear NOEXEC flag

Interpretline:
	LOAD	inbytel,spl			;get Stack-Ptr low byte
	LOAD	inbyteh,sph			;get Stack-Ptr high byte
;	cpi		inbytel,LOW(RAMEND-MCUSTKSZ);check for stack pointer RAM exceeeded
	CPI16	inbyteh,inbytel,(RAMEND-MCUSTKSZ);check for stack pointer RAM exceeeded
	brsh	stacknotexceeded	;AVR hardware stack not exceeded, continue
	Goto	Error_08			;stack error, inform user

stacknotexceeded:
	GoSub	formword			;get the command word.
	cpi		U,$0D				;CR?
	brne	stacknotexceeded1	;not, branch

	sbrc	XH,RUNACT			;skip next if we're executing interactive
	GoSub	GETPROGLINENO		;fetch and store the 1st line #
	rjmp	endofline			;stop processing

stacknotexceeded1:
	cpi		U,';'				;command separator
	breq	endofline			;yes, stop processing
	mov		temp,currentcast 	;Check to see if its a number
	cpi		temp,QC_NUM			;is it a number?
	brne	dontmakenumber		;no, don't make it a number
	rcall	makeHTUdecimal		;Its a number so make binary and push on stack
;!!added
;	sbrs	XH,NOEXEC			;skip next if NOEXEC flag set
;!!added
	GoSub	pushU
	rjmp	interpretline		;Keep going until CR is found

dontmakenumber:
	andi 	H,$5F				;Upper-case H,T,U
	andi	T,$5F
	andi	U,$5F

	cpi		temp,QC_QUOT		;is it a quote (string)?
	breq	notavar				;if so, its not a variable
	cpi		outchar,1			;If its a type 3 (letter) and 1 char long its a variable.
	brne	notavar				;(If outchar = 1 and ecode = 3 its a variable)
	mov		temp,currentcast	;get the last cast
	cpi		temp,QC_LTR			;is it a letter?
	brne 	notavar				;if not a letter then its not a variable
	GoSub	processvariable		;
	rjmp	interpretline

notavar:
	GoSub	getexecword			;lookup the KEYWORD and execute it
;	ld		U,Z					;fetch the next character in the program
	cpi		U,CR				;is last character a CR
	breq	endofline			;if yes, its the end of the line
	ret							;not the end of the line

endofline:
	ori		XL,(1<<VARSTK)	;Flag variable pointers to be left on stack next line
	ret
;
;********************************************************************************
GetExecWord:	;Find the address of the instruction that matches the three characters in H,T,and U.
				;Execute associated instruction then return.
	SETZPTR (2*commandlist)	;Load high part of byte address into ZH
tryanother:
	GoSub	fetchrombyte	;See if the string matches contents of H and T
	cp		romreg,H
	brne	nomatchH
	GoSub	fetchrombyte
	cp		romreg,T
	brne	nomatchT
	GoSub	fetchrombyte
	cp		romreg,U
	brne	nomatchU		;OK, A match has been found, get the address of the routine.
	adiw	ZH:ZL,1			;Increment Z registers -skip unused 4th instruction name byte
	lpm		temp,Z+			;Get address low into temp
	lpm		ZH,Z			;Get address high into ZH
	mov		ZL,temp			;move address low in ZL
;!!added
;	sbrs	XH,NOEXEC		;skip next if NOEXEC flag set
;!!added
    icall					;Do an indirect jump
    ret						;Then return when function is complete

nomatchH:					;Increment Z registers -skip T
 	adiw	ZH:ZL,1
nomatchT:					;Increment Z registers -skip U
	adiw	ZH:ZL,1
nomatchU:					;Increment Z registers -skip address (3 bytes)
	adiw	ZH:ZL,(commandlist2-commandlist1-1)
 	Goto	tryanother
;
;********************************************************************************
fetchrombyte:	;Read byte from rom, advance pointer. Enter with ZH, ZL
				; pointing to PROGROM list.
	lpm		r0,Z+			;Load byte from program memory into r0 and incr Z
 	tst		romreg			;Check if we've reached the end of the list of commands
	brne	fetchrombyte1	;not zero so finish
	lpm		r1,Z			;Load next byte from program memory into r1
 	tst		r1				;Check if we've reached the end of the list of commands
	brne	fetchrombyte1	;not zero so finish
	STOREI	ERRCode,$00
	Goto	error

fetchrombyte1:
	ret
;
;**************************************************************************************************
formword:	;Put up to three consecutive non-delimiters in registers H,T,and U, right-justified
			;Enter with YH, YL pointing to first char in buffer to be parsed
			;Output: non-delimiters in registers T,H,and U, right-justified, count of
			;chars in outchar. If three chars are collected, this routine advances
			;YL until the cast changes. Leaves with YH,YL pointing to next char to be parsed.
			;Used also: temp and inbyteh.

	ldi	H,$F0	 				;U is always loaded, no need to ininitalize
	ldi	T,$F0					;Pre-load the others

	ldi		outchar,$01
	ld		inchar,Y			;Get char from linebuffer into inchar
	GoSub	Qcast				;See what type of char it is.
	cpi		temp,QC_DELIM		;is it a delimiter? (ecode in temp)
	brne	nottype1cast		; **** advancelbpointer only once, just after Qcast
	GoSub	advanclbpointer
	Goto	formword

nottype1cast:
	mov		U,inchar 			;Store char and advance pointer
	GoSub	advanclbpointer
	cpi		inchar,$0D			;If its a carriage return, stop and return.
	breq	exitfromword0D

	mov		currentcast,ecode	;Save cast type for reference

Getanotherhcar:
	ld		inchar,Y			;Get char from linebuffer
	cpi		inchar,$0D			;If carriage return, then return
	breq	exitfromword0D
	GoSub	Qcast				;get the cast of the character
	cp		currentcast,ecode 	;If not same as cast of first char, then return.
	brne	exitfromword
	GoSub	ShiftHTU			;Store char and advance pointer
	GoSub	advanclbpointer
	inc		outchar
	cpi		outchar,$03			;If third character, exit
	brne	Getanotherhcar		;Otherwise, get another

;********************************************************************************
; The code below is to make sure pointer is not left pointing to the end of
;	a word that is longer than three chars. It is not entered when the pointer
;	is pointing to the carriage return at the end of the buffer.
;********************************************************************************
ExitFromWord:
	ld		inchar,Y			;Get char from linebuffer
	GoSub	Qcast				;get the cast of the character
	cp		currentcast,ecode	;is it the same?
	breq	notclearedgroup		;yes, continue scanning
exitfromword0D:
	ret							;return to caller
;
notclearedgroup:
	GoSub	advanclbpointer
	Goto	exitfromword

;********************************************************************************
advanclbpointer:
	;Decrements YL, Jumps to error routine if fetch is requester after
	;pointer is moved past the end of the buffer.
	CPI16	YH,YL,LNBUFF-1		;end of buffer?
	brne	nohitend

	STOREI	ERRCode,$04
	Goto	error
	ret

nohitend:
	SUBI16	YH,YL,1				;decrement buffer pointer
	ret
;
;*************************************************************************
;///////////////INSERT LINE ROUTINES//////////////////////////////
;*************************************************************************
; Qcast:	Determine whether a character is a letter, numeral, delimiter,
;	or other (operator).
;	- Enter with character in inchar.
;	- Returns with cast code in ecode.
;	- Uses: inchar
;	Value in ecode = cast
;		0 = Operator (not one of the other casts
;		1 = Delimiter -space ($20) or comma ($2C)
;		2 = Numeral 0..9 ($30 through $39)
;		3 = Letter -A..Z uppercase ($41 throught $5A)
;		4 = Carriage return
;		5 = quote character (string support) or "@" for a constant
;	Tests are made after anding the input byte with $5F
;*************************************************************************
Qcast:
	push	inchar		;save original character for later
	clr		temp		;Default is type 0

Qcast1:
	cpi		inchar,$0D	;Is it a carriage return? -check this before uppercasing
	brne	not0D
	Goto	makecast4
not0D:
	andi	inchar,$5F	;Make upper-case
	cpi		inchar,(' ' & $5F)	;Is it a space?
	brne	notaspacecode
	Goto	makecast1
notaspacecode:
	cpi		inchar,(',' & $5F)	;Is it a comma?
	brne	notacommacode
makecast1:
	ldi		temp,QC_DELIM
	rjmp	Qcast_out
notacommacode:
	cpi		inchar,('"' & $5F)	;Is it a quote?
	brne	notaquotecode
	ldi		temp,QC_QUOT		;its a quote
	rjmp	Qcast_out
notaquotecode:
	cpi		inchar,('@' & $5F)	;Is it an fetch?
	brne	notafetchcode
	ldi		temp,QC_QUOT		;its a "@", consider a string
	rjmp	Qcast_out
notafetchcode:
	cpi		inchar,$10
	brpl	notunder10
	rjmp	Qcast_out
notunder10:
	cpi		inchar,$5B
	brmi	notover5B
	rjmp	Qcast_out
notover5B:
	cpi		inchar,$40
	brmi	notover40
	ldi		temp,QC_LTR			;its a letter
	rjmp	Qcast_out
notover40:
	cpi		inchar,$1A
	brpl	notunder1A
	brpl	Qcast_out
	ldi		temp,QC_NUM			;its a number
notunder1A:
	rjmp	Qcast_out
;
makecast4:
	ldi		temp,QC_CR			;its a CR
;
Qcast_out:
	mov		ecode,temp
	pop		inchar
	ret
;
;=============================================================
sendbyte:      ;Send byte contained in inbytel to terminal
	GoSub	byte_to_asciihex
	push	inbytel				;save low digit 'cause USB kills inbyteh:l
	mov     outchar,inbyteh
	GoSub	emitchar
	pop		outchar				;restore low digit
	GoSub	emitchar
	ret
;
;=============================================================
crlf:
	ldi		outchar,$0D	;Send carriage return and line feed to terminal
	GoSub	emitchar
	ldi		outchar,$0A
	GoSub	emitchar
	ret

;=============================================================
emitchar0D:	;Send outchar to terminal. Add line feed to carriage return.
	cpi		outchar,$0D			;Its a carraiage return, send a linefeed also
	brne	notareturn
	GoSub	crlf
	ret
notareturn:
	Goto	emitchar
;
;***********************************************************************
;read the 6-digit packed BCD in registers H, T and U
;and convert to 6,4 or 2 ASCII characters sending to the serial
;link (char stored in 'outchar')
;***********************************************************************
;
SEND_6:
	mov		outchar,H			;get digits 6 & 5
	GoSub	SEND_2BCD			;send them
SEND_4:
	mov		outchar,T			;get digits 4 & 3
	GoSub	SEND_2BCD			;send them
SEND_2:
	mov		outchar,U			;get digits 2 & 1
	GoSub	SEND_2BCD			;send them
	ret
;
;***********************************************************************
;convert "digits" stored in 'outchar' to two ASCII BCD digits and send
;  them out the serial link
;***********************************************************************
SEND_2BCD:
	push	outchar				;save a copy
	andi	outchar,0xf0		;strip off low-nibble
	swap	outchar				;put into low-nibble
	subi	outchar,-(0x30)		;make it ASCII
	GoSub	emitchar			;send 10's digit
	pop		outchar				;retrieve the copy
	andi	outchar,0x0f		;strip off low-nibble
	subi	outchar,-(0x30)		;make it ASCII
	GoSub	emitchar			;send 1's digit
	ret
;
;=============================================================
; Send a string terminated in cariage return and line feed
; Called with location of start of string in Z
;=============================================================
sendlromline:
	GoSub	sendromstring
	GoSub	crlf
	ret
;
;=============================================================
;sendromstring: parses and prints a string from PROGMEM.
;	Called with location of string in Z
;	Parses the escape tokens for "repeat" and "center" as well
;	 as word substitution tokens.
;=============================================================
sendromstring:
	lpm		outchar,Z+			;get first character
	tst		outchar				;test for NULL, is end of message
	brne	sendromstring1		;branch if not a NULL
	ret							;return to caller
;
sendromstring1:
	cpi		outchar,TOK_CTR		;is it a "center" token?
	brne	sendromstring2		;branch if not
	GoSub	calcromstrlen		;get length of string
	Goto	sendromstring		;finish processing string

sendromstring2:
	cpi		outchar,TOK_REP		;is it a repeat token?
	brne	sendromstring3		;branch if not
	lpm		r1,Z+				;get character counter into r1
	lpm		outchar,Z+			;get character to repeat
	GoSub	repeatchar			;print it
	Goto	sendromstring

sendromstring3:
	cpi		outchar,TOK_CRLF	;is it a CR/LF token?
	brne	sendromstring4		;branch if not
	GoSub	crlf				;send CR/LF
	Goto	sendromstring

sendromstring4:
	cpi		outchar,TOK_ERR		;is it a "error" string token?
	brne	sendromstring5		;branch if not
	PUSHZ						;save Z-ptr
	SETZPTR (2*emessage)		;Load Z-ptr w/ new message address
	GoSub	sendromstring		;send the string
	POPZ						;restore previous Z-ptr
	Goto	sendromstring

sendromstring5:
	cpi		outchar,TOK_RNG		;is it a "value exceeded range" string token?
	brne	sendromstring6		;branch if not
	PUSHZ						;save Z-ptr
	SETZPTR (2*valexcdrng)		;Load Z-ptr w/ new message address
	GoSub	sendromstring		;send the string
	POPZ						;restore previous Z-ptr
	Goto	sendromstring

sendromstring6:
	cpi		outchar,TOK_NOT		;is it a " not " string token?
	brne	sendromstring7		;branch if not
	PUSHZ						;save Z-ptr
	SETZPTR (2*notstring)		;Load Z-ptr w/ new message address
	GoSub	sendromstring		;send the string
	POPZ						;restore previous Z-ptr
	Goto	sendromstring

sendromstring7:
	cpi		outchar,TOK_SUP		;is it a " support (C)" string token?
	brne	sendromstring8		;branch if not
	PUSHZ						;save Z-ptr
	SETZPTR (2*supportstrg)		;Load Z-ptr w/ new message address
	GoSub	sendromstring		;send the string
	POPZ						;restore previous Z-ptr
	Goto	sendromstring

sendromstring8:
	GoSub	emitchar			;send the character
	Goto	sendromstring		;loop for next character
;
;=============================================================================
; Calculates the number of printable characters in the string pointed to by Z.
;  Exits with Z pointing to the 1st printable character.
;
calcromstrlen:
	push	temp					;save temp register
	PUSHZ							;save Z-Ptr
	clr		r1						;zero character counter
calcromstrlen1:
	lpm		outchar,Z+				;get next character
	cpi		outchar,LF				;is it a LF?
	breq	calcromstrlen5			;is a LF, branch
calcromstrlen1a:
	cpi		outchar,CR				;is it a CR?
	breq	calcromstrlen5			;is a CR, branch
calcromstrlen1b:
	cpi		outchar,TOK_CRLF		;is it a CR/LF token?
	breq	calcromstrlen5			;is a TOK_CRLF token, branch
calcromstrlen1c:
	cpi		outchar,TOK_REP			;is it a repeat token?
	breq	calcromstrlen5			;is a TOK_REP token, branch
calcromstrlen1d:
	cpi		outchar,TOK_RNG			;is it a "range exceeded" token?
	brne	calcromstrlen1e			;not a TOK_RNG token, branch
	ldi		temp,STRLEN("Value exceeded range")
	add		r1,temp					;add the extra characters to count
calcromstrlen1e:
	cpi		outchar,TOK_NOT			;is it a "not" token?
	brne	calcromstrlen1f			;not a TOK_NOT token, branch
	ldi		temp,STRLEN(" not ")
	add		r1,temp					;add the extra characters to count
calcromstrlen1f:
	cpi		outchar,TOK_SUP			;is it a "support" token?
	brne	calcromstrlen1g			;not a TOK_SUP token, branch
	ldi		temp,STRLEN(" support (C)")
	add		r1,temp					;add the extra characters to count
calcromstrlen1g:
	inc		r1						;increment character counter
	Goto	calcromstrlen1			;loop for more

calcromstrlen5:	;r1 contains string length
	dec		r1						;decr character counter by 1
	ldi		outchar,LNLEN			;get max line length
	sub		r1,outchar				;subtract for difference
	neg		r1						;adjust for negative offset
	lsr		r1						;divide by two for offset

	ldi		outchar,' '				;load "space" character
	GoSub	repeatchar				;print the characters

	POPZ
	pop		temp					;restore temp register
	ret
;
;***********************************************************************
; Repeatedly prints the character held in the "outchar" register by the number
;	of times held in the r1 register
;***********************************************************************
repeatchar:
	GoSub	emitchar			;send the character
	dec		r1					;decrement counter
	brne	repeatchar			;loop till zero
	ret
;
;***********************************************************************
; ProcessVariable:	;Make variable in U into a pointer and
;	- if XL bit 0 is 1, put pointer on stack (assignment to variable), or
;	- if XL bit 0 is 0, put the value of variable on stack.
;***********************************************************************
ProcessVariable:
	subi	U,'A'				;subtract ASCII 'A' to make into an offset
	cpi		U,(VARSTORSZ)		;Test for range
	brmi	variableok
	STOREI	ERRCode,$05			;not a good variable, inform user
	Goto	error
variableok:	;at this point, the variable is an offset into the variable space
	sbrs	XL,VARSTK			;skip next if we're to leave the value
	brne	notcodezero			;
	GoSub	pushU				;Put pointer offsett onn stack (ecode = $00)
	ori		XL,(1<<VPTRSTK) 	;pointer pushed the stack, set VPTRSTK flag
	ret
notcodezero:
	SetZPtr	VARSTOR				;Z = pointer to variable space
	clr		T					;clear high register
	ADD16	ZH,ZL,T,U			;add offset to variable address
	ld		U,Z
	GoSub	pushU
	andi	XL,~(1<<VPTRSTK) 	;value on stack, clear VPTRSTK flag
	ret
;
;**********************************************************
pushU:	;Push contents of U on datastack, increment
		; offset pointer
	push	temp				;save register
	ldi		temp,DATASTKSZ		;stack depth
	cp		temp,DStkPtr		;stack overflow?
	brne	nopusherror			;no, continue
	Goto	Error_06			;stack overflow

nopusherror:
	push	r1					;save r1
	clr		r1					;zero register
	SetZPtr	(DATASTK+DATASTKSZ)-1;Z = TOS - 1
	SUB16	ZH,ZL,r1,DStkPtr	;subtract offset
	st		Z,U					;save U on data stack
	inc		DStkPtr				;increment offset pointer
	inc		cmdparms			;increment relative pointer
	pop		r1					;restore r1
	pop		temp				;restore register
	ret
;
;**********************************************************
popU:	;Pop contents of U from datastack, decrement
		; offset pointer
	tst		DStkPtr				;is DStkPtr = 0 ?
	brne	nopoperror			;no, continue
	Goto	Error_07

nopoperror:
	push	r1					;save r1
	clr		r1					;zero register
	SetZPtr	(DATASTK+DATASTKSZ)	;Z = TOS
	SUB16	ZH,ZL,r1,DStkPtr	;subtract offset
	ld		U,Z					;save U on data stack
	dec		DStkPtr				;deccrement offset pointer
	dec		cmdparms			;decrement relative pointer
	pop		r1					;restore r1
	ret
;
;**********************************************************
; pushGS: Push the address held in YH:YL onto the
;	GOSUB-RET stack, decrement pointer
;	- Next available address is left in GoSubStkPtr
;	- Preserves Z
;**********************************************************
pushGS:
	PUSHZ						;save Z
	ldi		temp,GOSUDEPTH		;stack depth
	cp		temp,GoSubStkPtr	;stack overflow?
	brne	pushGS1				;no, continue
	Goto	Error_08			;stack overflow!

pushGS1:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
	ldi		temp,GOSUCHUNK		;set GOSUCHUNK size
	mov		U,GoSubStkPtr		;copy DStkPtr in U
	GoSub	MPY8u				;8x8 unsigned software multiply
#else
	ldi		inchar,GOSUCHUNK	;set GOSUCHUNK size
	mov		temp,GoSubStkPtr	;copy DStkPtr in temp
	mul		temp,inchar			;multiply for pointer
#endif
	SetZPtr	(GOSUSTK+GOSUSTKSZ);Z = TOS
	SUB16	ZH,ZL,r1,r0			;subtract offset
	st		-Z,YH				;save it
	st		-Z,YL
	inc		GoSubStkPtr			;increment offset pointer
	POPZ						;restore Z
	ret
;
;***********************************************************
; popGS: Pop contents of GOSUB-RET address from the Gstack
;	and return its address in YH:YL.  Increment the pointer.
;	- "GoSubStkPtr" register holds the address of the high
;		byte of the next return address.
;	- Preserves Z
;***********************************************************
popGS:
	PUSHZ						;save Z
	tst		GoSubStkPtr			;GoSubStkPtr = 0?
	brne	popGS1				;no, continue
	Goto	Error_07			;stack under flow!

popGS1:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
	ldi		temp,GOSUCHUNK		;set GOSUCHUNK size
	mov		U,GoSubStkPtr		;copy DStkPtr in U
	GoSub	MPY8u				;8x8 unsigned software multiply
#else
	ldi		inchar,GOSUCHUNK	;set GOSUCHUNK size
	mov		temp,GoSubStkPtr	;copy DStkPtr in temp
	mul		temp,inchar			;multiply for pointer
#endif
	SetZPtr	(GOSUSTK+GOSUSTKSZ)	;Z = TOS
	SUB16	ZH,ZL,r1,r0			;subtract offset
	ld		YL,Z+				;load it
	ld		YH,Z
	dec		GoSubStkPtr			;decrement offset pointer
	POPZ						;restore Z
	ret
;
;**********************************************************
; pushLP: Push the address held in YH:YL onto the
;	LOOP stack as well as the contents of the "currlooptgt"
;	and "currloopvar" registers.  Decrement pointer.
;	- "LOOPStkPtr" register holds the address of the high
;		byte of the return address.
;	- Preserves Z
;**********************************************************
pushLP:
	PUSHZ						;save Z
	ldi		temp,LOOPDEPTH		;stack depth
	cp		temp,LOOPStkPtr		;stack overflow?
	brne	pushLP1				;no, continue
	Goto	Error_08

pushLP1:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
	ldi		temp,LOOPCHUNK		;set LOOPCHUNK
	mov		U,LOOPStkPtr		;copy LOOPStkPtr in r0
	GoSub	MPY8u				;8x8 unsigned software multiply
#else
	ldi		inchar,LOOPCHUNK	;set LOOPCHUNK
	mov		temp,LOOPStkPtr		;copy LOOPStkPtr in temp
	mul		temp,inchar			;multiply for pointer
#endif
	SetZPtr	(LOOPSTK+LOOPSTKSZ);Z = TOS
	SUB16	ZH,ZL,r1,r0			;subtract offset
	st		-Z,loopreturnL		;save return address
	st		-Z,loopreturnH
	st		-Z,currlooptgt		;store loop target value
	st		-Z,currloopvar		;store loop variable
	inc		LOOPStkPtr			;increment LOOP index pointer
	POPZ						;restore Z
	ret
;
;**********************************************************
; popLP: Pop contents of LOOP address from the LOOP stack
;	and return its address in YH:YL.  Restore contents of
;	the "currlooptgt" and "currloopvar" registers and
;	increment the pointer.
;	- "LOOPStkPtr" register holds the address of the high
;		byte of the next return address.
;	- Preserves Z
;**********************************************************
popLP:
	PUSHZ						;save Z
	tst		LOOPStkPtr			;LOOPStkPtr = 0?
	brne	popLP1				;no, continue
	Goto	Error_07

popLP1:
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
	ldi		temp,LOOPCHUNK		;set LOOPCHUNK
	mov		U,LOOPStkPtr		;copy LOOPStkPtr in U
	dec		U					;LOOPStkPtr less 1
	GoSub	MPY8u				;8x8 unsigned software multiply
#else
	ldi		inchar,LOOPCHUNK	;set LOOPCHUNK
	mov		temp,LOOPStkPtr		;copy LOOPStkPtr in temp
	dec		temp				;LOOPStkPtr less 1
	mul		temp,inchar			;multiply for pointer
#endif
	SetZPtr	(LOOPSTK+LOOPSTKSZ)	;Z = TOS
	SUB16	ZH,ZL,r1,r0			;subtract offset
	ld		loopreturnL,-Z		;fetch high address
	ld		loopreturnH,-Z		;fetch low address
	ld		currlooptgt,-Z		;fetch loop target
	ld		currloopvar,-Z		;fetch loop variable
;	dec		LOOPStkPtr			;decrement offset pointer
	POPZ						;restore Z
	ret
;
;**********************************************************************
; BitPositiontoORmask:	;Convert bit position (0..7) to OR mask value
;	Enter with bit position on top of dstack, exit with	mask on top
;	of stack
;**********************************************************************
BitPositiontoORmask:
	GoSub	popU				;Get bit position from stack
	GoSub	CheckUisByteVal		; 0 < U < 7 ?

BitPositiontoORmask1:	;an additional entry point
	ldi		temp,1				;Put 0000 0001 pattern into temp for shifting
	tst		U					;U=0?
	breq	BitPositiontoORmask3;no need to bit shift

BitPositiontoORmask2:
	lsl		temp				;rotate bit position left
	dec		U					;decrement bit counter
	brne	BitPositiontoORmask2;loop till all bits shifted

BitPositiontoORmask3:
	mov		U,temp
	Goto	PushURet			;exit
;
;****************************************************************
;SendHTUasdecimal:	Send decimal number in H,T,U
;	registers (hundreds,tens,units)
;****************************************************************
SendHTUasdecimal:
			;As numerals.
	cpi		H,0
	breq	dontsendh
	subi	H,-48
	mov		outchar,H
	GoSub	emitchar
	cpi		T,0
	brne	dontsendh
	ldi		outchar,$30	;If U=0 then don't emit this zero
	GoSub	emitchar
dontsendh:
	cpi		T,0
	breq	dontsendt
	subi	T,-48
	mov		outchar,T
	GoSub	emitchar
dontsendt:
	subi	U,-48
	mov		outchar,U
	GoSub	emitchar
	ret
;
;*************************************
TypeGreeting:				;Type greeting
	SETZPTR	(2*hellomessage)	;Load high part of byte address into ZH
	GoSub	sendlromline		;sent it.
	ret
;
;*************************************
LISTcommand:	;lists program memory buffer to screen..
	GoSub	crlf
	SetZPtr	(PROGMEM+PROGMEMSZ)-1;Initialize PC position inidcator

	LOAD16	YH,YL,PCLO		;get PC into Y

anotherchar:
	CP16	ZH,ZL,YH,YL
	breq	endofpmem
skip1:
	ld		outchar,Z	;Put contents of line buffer into outchar;
	GoSub	emitchar0D
	SUBI16	ZH,ZL,1
	Goto	anotherchar
endofpmem:
	GoSub	crlf
	;(listcommand flows into sizecommand)

;*************************************
FREEcommand:	;Find remaining program memory space. Uses U.
	GoSub	crlf				;send CR/LF

;print USED bytes first
	LOADi16	inbyteh,inbytel,PROGMEM+PROGMEMSZ-1;Initialize PC position inidcator
	LOAD16	T,H,PCLO			;get PC
	SUB16	inbyteh,inbytel,T,H	;get used program memory

	GoSub	BIN2BCD16			;convert to 6-digit BCD
	GoSub	SEND_6				;send the digits

	SETZPTR (2*usedmem)			;Load ROMSTRING to Z-pointer
	GoSub	sendromstring		;sent it.

;print bytes free next
	LOAD16	inbyteh,inbytel,PCLO;get PC
	SUBI16	inbyteh,inbytel,PROGMEM;calculate free bytes

	GoSub	BIN2BCD16			;convert to 6-digit BCD
	GoSub	SEND_6				;send the digits

	SETZPTR (2*freemem)			;Load ROMSTRING to Z-pointer
	GoSub	sendromstring		;sent it.

	ret
;
;*************************************
PRINTcommand:			;Print TOS after rcall, to screen
	ori		XH,(1<<PRIACT)		;set the "print active" flag
	GoSub	ChkCmdParms			;interpret line, return # of parms
	ldi		temp,5				;cast code for a quote character
	cp		currentcast,temp	;was the last character a quote?
	breq	PRINTcommand9		;yes, don't proccess any numbers

PRINTcommand1:	;this is the print loop
	GoSub	popU
	GoSub	binarytodecimal
	GoSub	sendHTUasdecimal
	ldi		outchar,' '			;send a space as a separator
	GoSub	emitchar
	tst		cmdparms			;parameter count = 0?
	brne	PRINTcommand1		;loop till zero
	GoSub	crlf
PRINTcommand9:
	andi	XH,~(1<<PRIACT)		;clear the "print active" flag
	ret
;
;******************************************************
printhexcommand:			;Print TOS in hex, after rcall, to screen
	ori		XH,(1<<PRIACT)		;set the "print active" flag
	GoSub	ChkCmdParms			;interpret line, return # of parms

printhexcommand1:	;this is the print loop
	ldi		outchar,$24
	GoSub	emitchar
	GoSub	popU
	mov		inbytel,U
	GoSub	sendbyte
;	GoSub	crlf
	ldi		outchar,' '			;send a space as a separator
	GoSub	emitchar
	tst		cmdparms			;parameter count = 0?
	brne	printhexcommand1	;loop till zero
	GoSub	crlf
	andi	XH,~(1<<PRIACT)		;clear the "print active" flag
	ret
;
;***********************************************
PRBcommand:	;print in binary format
	ori		XH,(1<<PRIACT)		;set the "print active" flag
	GoSub	ChkCmdParms			;interpret line, return # of parms

PRBcommand1:	;print in binary format
	GoSub	popU
	ldi		H,8					;load bit counter
stillsendingbinary:
	ldi		outchar,'0'			;bit is a "0"
	rol		U					;rotate bit in CARRY flag
	brcc	dontsendone			;if CARRY clear, keep the "0"
	ldi		outchar,'1'			;bit is a "1"
dontsendone:
	GoSub	emitchar
	dec 	H					;decrement bit counter
	brne	stillsendingbinary 	;loop till 0
;	GoSub	crlf   				;send a CR/LF combo
	ldi		outchar,' '			;send a space as a separator
	GoSub	emitchar
	tst		cmdparms			;parameter count = 0?
	brne	PRBcommand1			;loop till zero
	GoSub	crlf   				;send a CR/LF combo
	andi	XH,~(1<<PRIACT)		;clear the "print active" flag
	ret							;return to caller
;
;******************************************************
STRINGcommand:			;Print sting enclosed in quotes
	ld		outchar,Y			;get character
	GoSub	advanclbpointer		;advance past character
	cpi		outchar,'"'			;is it the end quote mark?
	breq	STRINGcommand9		;yes, exit
	cpi		outchar,CRLFCHAR	;is it the CR/LF indicator?
	breq	STRINGcommand1		;yes, print CR/LF combo
	GoSub	emitchar			;print the character
	rjmp	STRINGcommand

STRINGcommand1:
	GoSub	crlf				;send a CR?LF combo
	rjmp	STRINGcommand		;loop for more

STRINGcommand9:
	ret							;return to caller
;
;******************************************************
setequalscommand:		;Get value at TOS after rcall to variable on stack when
				;equals command is rcalled.
	GoSub	GetTwoParm			;get two parameters from dstack
	SetZPtr	VARSTOR				;Z = start of variable space
	clr		T					;clear high register
	ADD16	ZH,ZL,T,U			;add offset to pointer
	st		Z,temp
	ret
;
;******************************************************
evaluatecommand:;Equal sign	;If XL,2 = 0, perform EQUAL command.
	sbrs	XH,FNLEQ		;Skip next if flag set up FOR-NEXT loop
	Goto	EQUALcommand	;Its functionally identical to subtract.
				;Set up FOR-NEXT loop

	GoSub	Get1stParm			;get the last parameter
	mov		currlooptgt,U		;Store loop target

	GoSub	popU				;Pop initial counter value
	mov		temp,U				;Put in temp for now

	GoSub	popU				;Pop pointer to counter

	mov		currloopvar,U		;Copy variable pointer into currloopvar register

	clr		T					;clear high register
	SetZPtr	VARSTOR				;point to variable sapce
	ADD16	ZH,ZL,T,U			;add to obtain variable's position
	st		Z,temp				;Store initial counter value into counter

	ori		XL,(1<<LPCAPA)		;Set flag asking runcommand to capture loop address
	andi	XL,~(1<<VARSTK) 	;Clear the VARSTK flag
	andi	XH,~(1<<FNLEQ)		;Clear the FNLEQ flag
	ret
;
;*************************************
IFcommand:	;Set flag to skip next line if number on TOS returned from rest
			; of line is not zero
	GoSub	Get1stParm			;get the last parameter
	tst		U
	brne	itsnotzero
	ori		XL,(1<<SKPNXT)		;set skip next line # flag
itsnotzero:
	ret

;**************************************************************
;THENcommand: Return to previous rcalling routine using by
;	popping the previous commands return address off the stack.
;**************************************************************
THENcommand:
;	GoSub	interpretlinev
;test ^^^^^
;.if USBSUPP	;USB interrupts will screw us up!
;	GoSub	D_USBINT			;disable USB ints
;.endif
	pop		temp				;pop prior 16-bit return address off stack
	pop		temp
.if (FLASHEND > 0xFFFF)			;for 64KW+ parts
	pop		temp				;pop 3rd byte of return address
.endif
;.if USBSUPP
;	GoSub	E_USBINT			;re-enable USB ints
;.endif
	ret							;Pop up to previous rcall. Don't try this at home.
;
;*************************************
FORcommand:						;Initial statement in FOR-NEXT structure
								;Sets flag to switch "=" routine to set up the
								;FOR-NEXT loop
	ori		XH,(1<<FNLEQ)		;Set XL:FNLEQ flag to process variable
	ret
;
;***********************************************************
;NEXTcommand: This is the distant end of the FOR-NEXT loop.
;	Tests the loop variable to see its equal target.
;	- If its equal to target, go to next line
;	- If not equal target, increment variable and
;		Signal RUNCOMMAND to jump to loop return
;***********************************************************
NEXTcommand:
	SetZPtr	VARSTOR				;Fetch the the loop variable
	mov		U,currloopvar		;from variable space
	clr		T					;clear high register
	ADD16	ZH,ZL,T,U			;add for offset into variable space
	ld		temp,Z				;Get loop variable value into temp
	cp		temp,currlooptgt
	breq	NEXTcommand1		;If they are equal, the loop's done
	inc		temp
	st		Z,temp				;Increment loop variable and
								;  put it back into the register
	ori		XL,1<<LPJAMN		;Set XL:LPJAMN to flag run command
								; to jump to return address next time
	ret

NEXTcommand1:	;finished loop, check for more levels on stack
	dec		LOOPStkPtr			;decrement offset pointer
	breq	NEXTcommand3		;no more loops to process		;
	rcall	popLP				;restore prior LOOP info

NEXTcommand3:
	ret
;
;************************************************************************
; FETCHcommand:   This is a dummy command for human ease of identifying
;	an internal variable name.  Once called, control is passed again to
;	then line interpreter, which will invoke the variable name, another
;	AttoBASIC command, to return the symbol's low-byte addess.
;************************************************************************
.if	INTREG
  FETCHcommand:					;fall through to the "TO" command
.endif
;
;************************************************************************
; TOcommand: To in FOR-NEXT structure. This is a dummy command
;************************************************************************
TOcommand:
	GoSub	interpretlinev
	ret
;
;*************************************
ENDcommand:			;Stop execution by setting flag
	ori		XH,(1<<HALT)		;Set flag telling runcommand to stop.
	ret
;
;*************************************
HEXcommand:	;Nasty and dangerous - get two following chars as hex put on stack.
	ld		inbyteh,Y			;get 1st character
	GoSub	advanclbpointer		;advance past 1st character
	ld		inbytel,Y			;get next character
	mov		inchar,inbytel		;copy for qcast
	GoSub	qcast				;get cast
	cpi		temp,QC_DELIM		;is it a delimiter?
	breq	hexcommand1			;yes, only 1 character
	cpi		temp,QC_CR			;is it a CR?
	breq	hexcommand1			;yes, only 1 character
	GoSub	advanclbpointer		;advance past 2nd character
	Goto	hexcommand2			;convert and push on stack
hexcommand1:
	mov		inbytel,inbyteh		;only one char, set up for conversion
	clr		inbyteh
hexcommand2:
	GoSub	asciihex_to_byte
	mov		U,inbytel
	GoSub	pushU
	GoSub	interpretlinev
	ret
;
;*******************************************************************
; BINcommand:	Returns the 8 character string as a
;	binary value and places it on the data stack.
;	- Hack: Because the command parser interprets the '`' and '@'
;		characters as the same, we need to determine which one
;		caused a call to this routine and branch of necessary.
;*******************************************************************
BINcommand:
.if INTREG	;only if internal registers enabled
	ldd		temp,Y+1			;get the character @Y+1 in temp
	cpi		temp,$27			;is it the BIN command?
	breq	BINcommand1			;yes, continue
	Goto	VPGcommand			;transfer control to the VPG command
.endif

BINcommand1:
	clr		U					;all bits = 0
	ldi		temp,8				;load bit counter
	mov		bytecntr,temp

BINcommand2:
	ld		inbytel,Y			;get 1st character
	GoSub	advanclbpointer		;advance past character
	mov		inchar,inbytel		;copy for qcast
	GoSub	qcast				;get cast
	mov		currentcast,temp	;copy to currentcast
	cpi		temp,QC_NUM			;is it a number?
	breq	BINcommand2a		;yes, continue
	ADDI16	YH,YL,1				;Y = Y+1
	rjmp	BINcommand3			;exit

BINcommand2a:
	clc							;clear carry bit
	andi	inchar,0x01			;convert to a number
	sbrc	inchar,bit0			;skip next if bit0 = 0
	sec							;set carry bit
	rol		U					;rotate bit into U
	dec		bytecntr			;decrement bit counter
	brne	BINcommand2			;loop till zero

BINcommand3:
	GoSub	pushU
	GoSub	interpretlinev
	ret

BINcommand9:
	STOREI	ERRCode,$1C			;not a 0/1 error, inform user
	Goto	error
;
;****************************************************************************
;delaycommand:  Delays "N" x 10mS (10 to 2550 mS) based on the value passsed.
;	If the RTC routines are enabled, sleep mode is entered and the RTC
;	  is used as the interrupt source.
;	If the RTC routines are not enabled, a simple software driven loop
;	  is used instead.
;	USES: U, temp, temp2 and Y-ptr.
;****************************************************************************
DELAYcommand:			;Delay "N" x 10mS
	PUSHY						;save Y reg
	GoSub	Get1stParm			;get the last parameter

delaycommand1:
.if RTC
	ldi		temp,10/(1000/RTC_IPS);"n" x RTC interrupts
delaycommand2:
	CLRB	GPIOR0,RTCIRQ,temp2	;clear the RTCIRQ flag
	sleep						;sleep for now
	nop							;for benefit of SLEEP instr.
	SKBS	GPIOR0,RTCIRQ,temp2	;check the RTCIRQ flag
	rjmp	delaycommand2
	dec		temp
	brne	delaycommand2		;loop till 10 interrupts passed
.else
	_WAIT_mS	f_clk,9			;wait 9mS
	_WAIT_10us	f_clk,97		;wait 970uS
.endif
	dec		U					;decrement variable
	brne	delaycommand1		;loop till zero
	Goto	PopYret					;restore Y and return
;
;***********************************************
AOVcommand:
	clr		U					;disabled without a parameter
	GoSub	pushU				;save on stack
	GoSub	Get1stParm			;get the last parameter
	ldi		temp,$1				;only values of [0..1] allowed
	cp		temp,U
	brmi	AOVcommand9			;branch if allowable exceeded
	CLRB	GPIOR1,AOVErr,temp	;enable error detection
	sbrs	U,0					;skip next if bit 0 set
	ret							;return to caller
	SETB	GPIOR1,AOVErr,temp	;disble error detection
AOVcommand1:
	ret							;return to caller

AOVcommand9:
	Goto	Error_0C			;error code "C"
;
;*********************************************************************
;RSTcommand: causes a system reset using the watchdog timer
;	set to "System Reset" mode.  The time delay defaults to 32mS.
;*********************************************************************
RSTcommand:
	CLRB	MCUSR,WDRF,temp		;clear WDRF flag

.if USBSUPP
	GoSub	usb_shutdown		;insure USB interface has been terminated
								; before restart.
	GoSub	Delay1S				;wait 1 Second before restart
.endif

	;set the WD to "System Reset Mode" (1<<WDE | 0<<WDIE)
#if defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	STOREI	WDTCSR,(1<<WDE|WDT_1S),temp	;set it to 1.0S
#else
	STOREI	WDTCSR,(1<<WDCE|1<<WDE),temp	;enable access
	STOREI	WDTCSR,(0<<WDIE|1<<WDE|WDT_1S),temp	;set it to 1.0S
#endif
	cli							;disable global IRQ's

RSTcommand1:
	sleep						;let it happen
	nop
	ret
;
;******************************************************************
; makeHTUdecimal: Make decimal digits in H,T,and U into a binary
;	number in U.
;******************************************************************
makeHTUdecimal:
	andi	H,$0F
	andi	T,$0F
	andi	U,$0F
	GoSub	decimaltobinary
	ret
;
;***********************************************
getlineno:	;Search buffer for line number, set up for runcommand to jump to it.
			;Can use Y because will will be modified by this routine
			;before runcommand uses it again anyway.
	GoSub	Get1stParm			;get the last parameter
	mov		outchar,U			;temporarily put target line number into outchar

	SetZPtr	(PROGMEM+PROGMEMSZ)-1;Initialize PC position inidcator

	LOAD16	inbyteh,inbytel,PCLO;save current PC in inbyteh:inbytel
	Goto	pufistln			;Jump in the middle so first line no. can be found

searchCR:
	cp		ZH,inbyteh
	brne	skip2
	cp		inbytel,ZL			;Error if past end of buffer
	brsh	gotonotfound
skip2:
	ld		temp,Z
	cpi		temp,$0D
	breq	checkforlinumb		;Take the branch if CR is found
	SUBI16	ZH,ZL,1
	Goto	searchCR			;Else continue to look for CR

checkforlinumb:					;Found a CR now make line number
		SUBI16	ZH,ZL,1			;Move pointer past CR
pufistln:
	clr	U
	clr	T
	clr	H

fetchanothernum:
	cp		ZH,inbyteh
	brne	skip3
	cp		inbytel,ZL			;Error if past end of buffer
;	brpl	gotonotfound
	brsh	gotonotfound
skip3:
	ld		inchar,Z			;put char pointed to by Z into inchar
	GoSub	qcast				;Find out what kind of char this is
	cpi		temp,QC_NUM			;is it a numeral? (ecode in temp)
	brne	notanumeral
	GoSub	ShiftHTU			;It's a numeral so shift it in
	SUBI16	ZH,ZL,1
	Goto	fetchanothernum
notanumeral:	;The pointer is not pointing to a numeral, so test the number.

	rcall	makeHTUdecimal		;Its a number so make binary and push on stack
;	GoSub	decimaltobinary		;Change from 3 BCD bytes to one binary vaue
	cp		outchar,U
	brne	searchCR			;No the current line number is in binary form
	ret

;***********************************************
GOTonotfound:
	STOREI	ERRCode,$09	;Claim error - goto line not found
	Goto	error

;***********************************************
GOTocommand:
	GoSub	getlineno
	COPY16	YH,YL,ZH,ZL	;copy to Y
	ret
;
;***********************************************
GOSubcommand:			;Do a gosub
	GoSub	getlineno	;Get destination address in ZH and ZL
	ori		XL,(1<<GSJAMD | \
				1<<GSCAPA)	;Set flags to capture return address
							; and jam destination address
	ret
;
;***********************************************
RETurncommand:			;Return from gosub
	sbrs	XH,GSACT	;skip next if GOSUB active
	Goto	RETurnnotactive
	ori		XL,(1<<GSJAMR);Set flag rcalling for jaming of return address
	ret

RETurnnotactive:
	ori		XH,(1<<HALT)	;Set flag rcalling for program halt
	ret

.if EFS
;*********************************************************************
;REMcommand: REM is a dummy command used to support catalogging
;*********************************************************************
REMcommand:
	ld		outchar,Y			;get character @PC[HI:LO]
	GoSub	advanclbpointer		;advance past 1st character
	cpi		outchar,CR			;is it the CR character?
	brne	REMcommand			;no, continue looping
	ret
.endif
;
;***********************************************
emitcommand:
	GoSub	Get1stParm			;get the last parameter
	mov		outchar,U
	GoSub	emitchar
	ret
;
;*************************************************************
NEWprogram:		;Ready program space and interpreter for new program
	SetZPtr	(PROGMEM+PROGMEMSZ)-1;Initialize program memory pointer to top of program memory
	STORE16	PCLO,ZH,ZL		;Initialize PC position indicator

	INITDSTACK					;Initialize data stack pointer

	clr		XL					;Clear interpreter mode flags
	clr		XH

.if UARTSUPP
;	GoSub	D_UARTINT			;disable UART ints
.endif

	PushStat					;save SREG
	cli							;disable interrupts during
								; memory fill
	GoSub	ClrProgMem			;clear PROGMEM

	GoSub	ClrVarMem			;clear variable

	PopStat						;restore SREG

.if UARTSUPP
;	GoSub	E_UARTINT			;enable UART ints
.endif

endofpmem1:
	GoSub	crlf
	SETZPTR (2*signonmessage)	;Load high part of byte address into ZH
	GoSub	sendlromline		;sent it.
	ret
;
;************************************************************
; CheckUisByteVal: Test that U is in the range of [0..7]
;  Transfers to error process if not.
;************************************************************
CheckUisByteVal:
	cpi		U,8					;8 bit positions per byte
	brsh	CheckUisByteVala	;branch if allowable exceeded
	ret							;return to caller
CheckUisByteVala:
	Goto	Error_0A			; error!
;
;************************************************************
; CheckUisNiblVal: Test that U is in the range of [0..3]
;  Transfers to error process if not.
;************************************************************
CheckUisNiblVal:
	cpi		U,4					;4 bit positions per nibble
	brsh	CheckUisNiblVala	;branch if allowable exceeded
	ret							;return to caller
CheckUisNiblVala:
	Goto	Error_0D			; error!
;
;************************************************************
; CheckUisBitVal: test that U is in the range of [0..1].
;  Transfers to error process if not.
;************************************************************
CheckUisBitVal:
	cpi		U,2					;2 states per bit
	brsh	CheckUisBitVala		;branch if allowable exceeded
	ret							;return to caller
CheckUisBitVala:
	Goto	Error_0C			; error!
;
.if CFG
;******************************************************************************
; CFGcommand: Reads and writes non-volatile memory.
;  - If no parameters are passed, the value of the configuration byte is
;	 printed in hexadecimal and binary.
;  - If one (1) parameter is passed then the value of that bit position is
;    placed on the data stack.
;  - If two (2) parameters are passed then the value in the 2nd parameter is
;    written to the bit position specified in the 1st parameter.  In a
;    special case, if the 1st parameter is "8" then the 2nd parameter is
;    assumed to be a byte value to be directly written to the configuration
;    byte in EEPROM.
;******************************************************************************
CFGcommand:
	PUSHY						;save Y-Ptr
	GoSub	ChkCmdParms			;interpret line, tst for zero parms
	SetYPtr	AB_CFG0				;point Y to AB_CFG0 byte in EEPROM
	GoSub	ReadEEP				;get AB_CFG0 into temp register
	tst		cmdparms			;is there 0 parameter on dstack?
	breq	CFGcommand1			;0 parameters on dstack
	cpi		cmdparms,1			;is there 1 parameter on dstack?
	breq	CFGcommand2			;yes, branch
#if !defined(__ATtiny84__) && \
	!defined(__ATtiny84A__) && \
	!defined(__ATtiny85__) && \
	!defined(__ATmega88__) && \
	!defined(__ATmega88PA__)	; disable CFG [B] [V] parameters
	cpi		cmdparms,2			;is there 2 parameter on dstack?
	breq	CFGcommand3			;yes, branch
#endif
	Goto	Error_16			;error code "16", "Too many arguments"

;read and return the configuration byte on dstack
CFGcommand1:
	mov		U,temp				;move temp to U register
CFGcommand1a:
	GoSub	PUSHU				;save the value on the dstack
	rjmp	CFGcommand4b			;exit routine
;
;read config bit position and return data on data stack
#if defined(__ATtiny84__) || \
	defined(__ATtiny84A__) || \
	defined(__ATtiny85__) || \
	defined(__ATmega88PA__)	 || \
	defined(__ATmega88__)			; On Tiny85 and Mega88, only
;								  support byte-write to
								;  configuration byte
CFGcommand2:
	GoSub	POPU				;fetch the value from dstack
	rjmp	CFGcommand4		;and write it to the EEPROM
;
#else	; For all other MCU's,
CFGcommand2:
	GoSub	POPU				;fetch 1st parameter
	GoSub	CheckUisByteVal		;check for proper range
	tst		U					;is U=0? If so, no shift
	breq	CFGcommand2b		;yes, skip shift
CFGcommand2a:
	lsr		temp				;shift config byte value right
	dec		U					;decrement bit counter
	brne	CFGcommand2a		;loop till zero
CFGcommand2b:
	ldi		U,1					;preload U with a "1"
	sbrs	temp,bit0			;skip next of bit 0 is set
	clr		U					;clear U
CFGcommand2c:
	rjmp	CFGcommand1a		;save U and exit routine
#endif
;
;write data to bit position in the configuration byte
#if !defined(__ATtiny84__) && !defined(__ATtiny84A__) && \
	!defined(__ATtiny85__) && \
	!defined(__ATmega88__) && !defined(__ATmega88PA__) ; disable for Tiny84/85/Mega88
CFGcommand3:
	mov		H,temp				;copy temp to H register
	GoSub	POPU				;fetch 2nd parameter

	SetYPtr	DATASTK+DATASTKSZ-1	;point Y to top of data stack less 1
	ld		temp,Y				;fetch the 1st parameter
	cpi		temp,8				;does temp=8? If so, direct write
	breq	CFGcommand4			;yes, direct write.

	GoSub	CheckUisBitVal		;check for proper range [0..1]
	mov		T,U					;save value in T register

	GoSub	POPU				;fetch 1st parameter
	GoSub	CheckUisByteVal		;check for proper range [0..7]

	GoSub	PUSHU				;save bit position on dstack
	GoSub	bitpositiontoormask	;calculate the mask value
	clr		DStkPtr				;reset data stack pointer (we have
								;  mask value in U already)
	sbrc	T,bit0				;skip next if clear bit
	rjmp	CFGcommand3b		;set bit

; clear bit by AND'ing mask with current configuration byte
CFGcommand3a:
	com		U					;invert mask
	and		U,H					;AND with current configuration byte
	rjmp	CFGcommand4			;jump to write EEP

; set bit by OR'ing mask with current configuration byte
CFGcommand3b:
	or		U,H					;OR with current configuration byte
#endif
;
;direct bytewide write to the configuration byte
CFGcommand4:
	mov		temp,U				;copy to the temp register
	SetYPtr	AB_CFG0				;point Y to AB_CFG0 byte in EEPROM
	GoSub	WriteEEP			;write the byte and fall through
;
CFGcommand4b:	;exit routine
	Goto	PopYret					;restore Y and return
;
.endif
;
#if DREC	; only for the AVR Data Recorder
;******************************************************************************
; LDDcommand: Load default capture program into program RAM.
;	This command is only for the the AVR Data Recorder
;******************************************************************************
LDDcommand:
	rcall	NEWprogram			;clear program memory

	SetYPtr	(PROGMEM+PROGMEMSZ)	;set Y pointer to program memory in RAM
	SetZPtr	(2*Default_Prog)	;point Z to program text in PROGMEM

LDDcommand1:
	lpm		temp,Z+				;get character
	tst		temp				;is it a null character (EOS)?
	breq	LDDcommand2			;exit if EOS
	st		-Y,temp				;store character in RAM
	rjmp	LDDcommand1			;loop

LDDcommand2:
	STORE16	PCLO,YH,YL			;Save PC position indicator
	ret							;return to caller
;
#endif
;
;****************************************************************************
; WTFcommand: Random goofiness
;****************************************************************************
WTFcommand:
#if ( defined(__ATmega328__) || \
	  defined(__ATmega328P__) || \
	  defined(__ATmega168PA__) || \
	  defined(__AT90USB1286__)  || \
	  defined(__ATmega2560__) || \
	  defined(__ATmega32U4__) )

	sbrc	XH,PRIACT			;PRIACT flag clear, print to console
	rjmp	WTFcommand1			;just return a value on dstack
	sbrc	XL,VPTRSTK			;VPTRSTK flag clear, print to console
	rjmp	WTFcommand1			;just return a value on dstack

	GoSub	RNDcommand			;fetch a random number
	GoSub	popU				;fetch in U
	tst		U					;test and set some flags
	brmi	WTFcommand2			;branch if N flag set

	SETZPTR (2*WTFmsg2)			;Z = string address
	rjmp	WTFcommand3			;send and exit
WTFcommand2:
	SETZPTR (2*WTFmsg1)			;Z = string address

WTFcommand3:
	GoSub	sendromstring		;sent it.
#endif
WTFcommand1:
	ldi		U,42				;"the answer"
	Goto	PushURet			;exit
;
;.if SPGM
;******************************************************************************
; DStkRev: Reverses the data on the data stack
;	Enter with:
;		-  DStkPtr = # of elements on the datastack
;	- Uses inbyteh, inbytel, bytecntr, Y and Z pointers.
;******************************************************************************
;DStkRev:
;	PUSHZ						;save Z pointer
;	PUSHY						;save Y pointer
;	clr		inbyteh				;clear a register
;	mov		bytecntr,DStkPtr	;load counter
;	SetZPtr	DATASTK+DATASTKSZ	;point Z to 1st element of data stack
;	SUB16	ZH,ZL,inbyteh,DStkPtr;adjust to last element of data stack
;	SetYPtr	DATASTK+DATASTKSZ-1	;point Y to top of data stack element
;	lsr		bytecntr			;divide # of elements by 2
;DStkRev1:
;	ld		inbytel,Z			;fetch 1st element
;	ld		inbyteh,Y			;fetch last element
;	st		Z+,inbyteh			;store as last element
;	st		Y,inbytel			;store as first element
;	SUBI16	YH,YL,1				;decrement Y pointer
;	dec		bytecntr			;decrement element counter
;	brne	DStkRev1			;loop till match
;	POPY						;restore Y pointer
;	POPZ						;restore Z pointer
;	ret							;return to caller
;
;.endif
;
.if HELP
;***********************************************
HELPcommand:
	PushY						;save Y pointer
	GoSub	Dump_Perf			;print beginning "perforation"
	ldi		outchar,' '			;toss in space for beginning of new line
	GoSub	emitchar			;send the character

	SetZPtr	(2*commandlist)		;Set Z-Ptr to beginning of command list

HELPcommand1:
	ldi		YH,LNLEN-4			;line length counter

HELPcommand2:
	ldi		YL,2*(commandlist1-commandlist);character counter

HELPcommand3:
	lpm		H,Z+				;get 1st character into H
	tst		H					;end of list?
	brne	HELPcommand3a		;no, continue
	rjmp	HELPcommand9		;yes, exit

HELPcommand3a:
	lpm		T,Z+				;get 2nd character into T
	lpm		U,Z+				;get 3rd character into U

;check for "TO" command
	cpi		T,'T'				;test for "TO" command
	brne	HELPcommand3b		;not a "T"
	cpi		U,'O'
	brne	HELPcommand3d		;not a "O"
	ldi		H,' '				;1st character is a space
	rjmp	HELPcommand4		;continue to print

HELPcommand3b:	;check for "IF" command
	cpi		T,'I'				;test for "IF" command
	brne	HELPcommand3c		;not a "I"
	cpi		U,'F'
	brne	HELPcommand3d		;not a "O"
	ldi		H,' '				;1st character is a space
	rjmp	HELPcommand4		;continue to print

HELPcommand3c:
;check for "OR" command
	cpi		T,'O'				;test for "OR" command
	brne	HELPcommand3d		;not a "O"
	cpi		U,'R'
	brne	HELPcommand3d		;not a "R"
;check for "-OR" command
	cpi		H,'W'
	breq	HELPcommand4		;is a "W" for "WORD" command
	cpi		H,'X'
	breq	HELPcommand4		;is a "X" for "XOR" command
	cpi		H,'F'
	breq	HELPcommand4		;is a "F" for "FOR" command
	ldi		H,' '				;1st character is a space
	rjmp	HELPcommand4		;continue to print

HELPcommand3d:
;check for "  @" command
	cpi		U,'@'				;test for '@' or higher
	breq	HELPcommand3f		;yes, modify H and T registers

;check for "  ^" command
	cpi		U,('^' & $5F)		;test for '^' or higher
	brge	HELPcommand3f		;branch if same or higher

;check for "  |" command
	cpi		U,('|' & $5F)		;test for '|' or higher
	brge	HELPcommand3e		;branch if same or higher

	cpi		U,$30				;test for an operator command
	brge	HELPcommand4		;branch if higher

HELPcommand3e:
	subi	U,(-$20)			;add to character

HELPcommand3f:	;determine if it is a plain text command or not
	ldi		H,' '				;1st character is a space
	cpi		T,$30				;test for an operator command
	brmi	HELPcommand3g		;branch if higher
	clr		T

HELPcommand3g:
	subi	T,(-$20)			;add to character

HELPcommand4:
	mov		outchar,H			;send the string
	GoSub	emitchar			;send the character
	mov		outchar,T			;send the string
	GoSub	emitchar			;send the character
	mov		outchar,U			;send the string
	GoSub	emitchar			;send the character
	ldi		outchar,' '			;toss in space for separator
	GoSub	emitchar			;send the character

	adiw	ZH:ZL,(commandlist2-commandlist1+2);skip address to command

	subi	YH,2*(commandlist1-commandlist);decrement line length counter
	brpl	HELPcommand2		;loop till negative

	GoSub	crlf				;end line with a CR/LF

#if USB && ( defined(__ATmega32U4__) || defined(__AT90USB1286__) )
;This code is not really neccessary but is prudent to implement
; so the USB buffers do not overflow.
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	GoSub	usb_serial_flush_output	;flush the line buffer
	pop		r1					;restore r1
	GoSub	Delay10mS			;give USB host time to retrieve
HELPcommand5:
#endif

	ldi		outchar,' '			;toss in space for beginning of new line
	GoSub	emitchar

	rjmp	HELPcommand1		;continue looping till end if list

HELPcommand9:
	GoSub	crlf				;end line with a CR/LF
	GoSub	Dump_Perf			;print beginning "perforation"
	Goto	PopYret					;restore Y and return
.endif
;
;*******************************************************************************
; RUNcommand: runs the program currently in program memory.  If a value is on
;	the datastack when the RUN command is executed, it is assumed to be a
;	file number, which is loaded before being executed.
;*******************************************************************************
RUNcommand:
.if EFS	;only if EFS is enabled
	SKBC	GPIOR2,SSTEn,temp	;skip next if not SelfStart
	rjmp	RUNcommand2			;SelfStart, skip loading file
	Gosub	interpretlinev
	tst		DStkPtr
	breq	RUNcommand2			;nothing on the data stack
	GoSub	popU				;fetch the data on dstack

	ldi		temp,HANDLES_MAX-1	;only valid file handles allowed
	cp		temp,U				;check for valid file handle
	brge	RUNcommand1a		;branch if good file handle

	Goto	Error_16			;error code "16"

RUNcommand1a:
;	sbr		XH,(1<<RUNACT)		;set flag to indicate we are executing a command
	ori		XH,(1<<RUNACT)		;set flag to indicate we are executing a command
	GoSub	ClrProgMem			;clear PROGMEM before loading
	GoSub	LOADcommand			;load the file number
.endif
;
RUNcommand2:
	INITGSTACK					;Initialize GOS-RET stack pointer
	INITLPTACK					;Initialize FOR-NEX loop stack pointer

	CLRB	GPIOR2,SSTEn,temp	;clear SelfStart flag
;	sbr		XH,(1<<RUNACT)		;set flag to indicate we are executing a command
	ori		XH,(1<<RUNACT)		;set flag to indicate we are executing a command
	SetYPtr	(PROGMEM+PROGMEMSZ)-1;Initialize PC position inidcator

	GoSub	GETPROGLINENO		;fetch and store the 1st line #

checkandrunline:	;This is the main RUN loop, executed once per program line
	INITDSTACK					;Initialize data stack pointer

	ori		XL,(1<<VARSTK)		;Flag variable pointers to be left on stack
	GoSub	Checkkeybd			;See if Control-C key was received
;*********************************************************
noaddressjam:				;GOSUB-RETURN management
	sbrs	XL,GSCAPA			;skip next if XL:GSCAPA flag set
	rjmp	nocapgosubret
	rcall	pushGS				;push return address
	andi	XL,~(1<<GSCAPA)		;clear flag

nocapgosubret:				;GOSUB-RETURN management
	sbrs	XL,GSJAMD			;skip next if XL:GSJAMD flag set
	rjmp	nodestaddjam
;	mov		YH,ZH 				;jamb destination address
;	mov		YL,ZL
	CopyZtoY					;jamb destination address
	ori		XH,(1<<GSACT)		;Set flag indicating that gosub is active
	andi	XL,~(1<<GSJAMD)		;Flip GOSUB jam address flag off

nodestaddjam:				;GOSUB-RETURN management
	sbrs	XL,GSJAMR			;skip next if XL:GSJAMR flag set
	rjmp	loopflagcheck
	rcall	popGS				;restore GOSUB-RETURN address
	tst		GoSubStkPtr			;GoSubStkPtr = 0?
	brne	nodestaddjam1		;no, more GOSUBS active
	andi	XH,~(1<<GSACT)		;clear GSACT flag
nodestaddjam1:
	andi	XL,~(1<<GSJAMR)		;clear GSJAMR flag

loopflagcheck:				;FOR-NEXT management.
	sbrs	XL,LPCAPA			;FOR-NEXT management. Capture F-N return adddress
	rjmp	nocaploopad
;	mov		loopreturnL,YL		;Copy return address (in interpreter p. space) to registers
;	mov		loopreturnH,YH
	;Copy return address (in interpreter p. space) to registers
	Copy16	loopreturnH,loopreturnL,YH,YL
	rcall	pushLP				;push FOR_NEXT address on stack
	andi	XL,~(1<<LPCAPA)		;Turn off flag -address captured.

nocaploopad:				;FOR-NEXT management.
	sbrs	XL,LPJAMN			;;skip next if XL:LPJAMN flag is set
	rjmp	noretjam			; LPJAMN flag not set, continue
;	mov		YH,loopreturnH 		; Jam in new address
;	mov		YL,loopreturnL
	; Jam in new address
	Copy16	YH,YL,loopreturnH,loopreturnL
	andi	XL,~(1<<LPJAMN)		;clear jaming flag

noretjam:					;HALT via "END" or control-C
	sbrs	XH,HALT				;Skip next if XH,HALT flag is set
	rjmp	nostop
	GoSub	breakmessage		;HALT, so send break message to user
	LOAD16	YH,YL,PCLO			;get PC into Y
	andi	XH,~(1<<HALT)		;Clear the halt flag

nostop:
	LOAD16	inbyteh,inbytel,PCLO;get end of program counter
	SUB16	inbyteh,inbytel,YH,YL;less than PCHI/LO?
	brsh	endofprogram
	;not end of program so keep processing program lines
	ld		inchar,Y			;Get char from buffer into temp
	GoSub	Qcast				;See what type of char it is.
	sbrs	XL,SKPNXT			;skip this line if XL:SKPNXT is clear
	rjmp	noskipline			;to next carriage return
	cpi		temp,QC_CR			;is it a CR? (ecode in temp)
	breq	noskipline
	SUBI16	YH,YL,1				;If numeral, move pointer past it
	rjmp	checkandrunline		;loop and continue processing lines

noskipline:
	andi	XL,~(1<<SKPNXT)		;Make sure that skip line flag is cleared for next time around
	; **** does clearing this flag now actuall do anyting?
;	Code below uses Qcast rcall several lines above.
	cpi		temp,QC_NUM			;is it a number? (ecode in temp)
	brne	notonlinenumber		;no, not a the begginning of a program line
	SUBI16	YH,YL,1				;If numeral, move pointer past it
	rjmp 	checkandrunline		;loop for next program line

notonlinenumber:
	GoSub	interpretline		;interpret program line, Y pointer is at the
								;  1st char of program line
	rjmp 	checkandrunline		;loop for next program line

endofprogram:	;we hit an END or ^C caused a BREAK condition
	STOREI	CurrLine,0			;line "0" as default
	andi	XH,~(1<<RUNACT)		;clear flag to indicate we are
								; in interpreter mode

#if defined(__ATmega32U4__) || defined(__AT90USB1286__)
 .if CFG && USB	; if USB SIO and configuration register is available
;	fetch the configuration register to see if we just ended the
;	self-start program
	SetYPtr	AB_CFG0				;point Y to AB_CFG0 byte in EEPROM
	GoSub	ReadEEP				;get AB_CFG0 into temp register
	sbrs	temp,CFG_SSTRT		;check for self-start enabled
	ret							;not ending self-start program

	LOAD	inbytel,usb_configuration;load USB config status
	tst		inbytel				;if zero, we have no connection
	brne	endofprogramA		;we are already connected to a USB host
	GoSub	InitSIOa			;attempt to (re)initialize serial I/O
endofprogramA:
 .endif
#endif
	ret
;
;*********************************************************
;GETPROGLINENO:	fetches the line number of the next line to
;	execute.
;	- Enter with current program pointer in Y
;	- Return with line number in U
;*********************************************************
GETPROGLINENO:
	LOAD	temp,PCHI			;check for end of program
	cp		YH,temp
	brne	GETPROGLINENO1
	LOAD	temp,PCLO
	cp		YL,temp
	brne	GETPROGLINENO1
	ret

GETPROGLINENO1:
	PUSHY						;save Y-pointer
	push	ecode				;save registers
	push	inchar

	clr		T					;clear T and U registers
	clr		U

GETPROGLINENO2:
	ld		inchar,Y			;Get char from buffer, increment pointer
	GoSub	Qcast				;See what type of char it is.
	cpi		temp,QC_NUM			;is it a number? (check for change of cast)
	breq	GETPROGLINENO3		;yes, continue
	GoSub	decimaltobinary		;convert HTU to binary (contains current line#)
	STORE	CurrLine,U			;save current line # for later
	rjmp	GETPROGLINENO8		;exit

GETPROGLINENO3:
	GoSub	ShiftHTU			;shift number into HTU
	andi	U,$0F				;convert to BCD
	SUBI16	YH,YL,1				;If numeral, move pointer past it
	rjmp	GETPROGLINENO2
;
GETPROGLINENO8:
	SKBS	GPIOR2,DebugEn,temp	;skip next if debug enabled
	rjmp	GETPROGLINENO9		;no debug display
	rcall	PrnLnNo				;print the line #
	rcall	crlf				;send a CR/LF combo to the console

GETPROGLINENO9:
	pop		inchar				;restore registers
	pop		ecode
	Goto	PopYret					;restore Y and return
;
;*******************************************************************
breakmessage:
#if ( USB && ( defined(__ATmega32U4__) || defined(__AT90USB1286__) ) )
	push	r1					;save r1
	clr		r1					;insure r1 = 0
	GoSub	usb_serial_flush_input 	;flush any junk input
	pop		r1					;restore r1
#endif
	SETZPTR (2*breakmsgtxt)	;Load high part of byte address into ZH
	GoSub	sendromstring		;sent it.
	ret

;/////////////////END DATA FORMAT CONVERSION ROUTINES/////////////////
;
;****************************************************************************
; MkRndSeed: scan RAM and generate a random seed from the unitialized RAM
;	at powerup.  Saves the seed in RAM for later use with the RND command.
;****************************************************************************
MkRndSeed:
	ldi		U,RNDPoly			;get poly-seed into U
	SetZPtr	SRAM_START			;Z = start of RAM

MkRndSeed1:
	ld		temp,Z+				;fetch a byte
	eor		U,temp				;xor it
	CPI16	ZH,ZL,RAMEND		;end of RAM?
	brne	MkRndSeed1			;loop till end of RAM
	STORE	RNDSeed,U			;save it in RAM
	ret
;

;****************************************************************************
; ShiftHTU: Shifts registers inchar->U->T->H
;****************************************************************************
ShiftHTU:
	mov		H,T					;shift number into HTU
	mov		T,U
	mov		U,inchar			;move last digit into U
	ret							;return to caller
;
;****************************************************************************
; ClearHTU: Clears registers H, T and U
;****************************************************************************
ClearHTU:
	clr		H
	clr		T
	clr		U
	ret							;return to caller
;
;****************************************************************************
; DELAY1US: Delays 1uS.
;****************************************************************************
Delay1uS:
	push		r29					;save register
	_WAIT_uS	f_clk,1				;delay 1uS
	pop			r29					;restore register
	ret
;
;****************************************************************************
; DELAY5US: Delays 5uS.
;****************************************************************************
Delay5uS:
	push		r29					;save register
	ldi			r29,5				;2 x 5uS = 10uS
	rcall		Delay1uS			;delay 1uS
	dec			r29					;decrement counter
	brne		PC-2				;loop till zero
	pop			r29					;restore register
	ret
;
;****************************************************************************
; DELAY10US: Delays 10uS.
;****************************************************************************
Delay10uS:
	push		r29					;save register
	ldi			r29,10				;10 x 1uS = 10uS
	rcall		Delay1uS			;delay 1uS
	dec			r29					;decrement counter
	brne		PC-2				;loop till zero
	pop			r29					;restore register
	ret
;
.if AComp	;only used for the AComp
;****************************************************************************
; DELAY100US: Delays 100uS.
;****************************************************************************
Delay100uS:
	push		r29					;save register
	ldi			r29,100				;100 x 1uS = 100uS
	rcall		Delay1uS			;delay 1uS
	dec			r29					;decrement counter
	brne		PC-2				;loop till zero
	pop			r29					;restore register
	ret
;
.endif
;
;****************************************************************************
; DELAY1MS: Delays 1mS.
;	Destroys YH and YL
;****************************************************************************
Delay1mS:
	PUSHY							;save Y
	_WAIT_10uS	f_clk,99			;delay 990uS (fudge)
	POPY							;restore Y
	ret
;
;****************************************************************************
; DELAY10MS: Delays 10mS.
;	Destroys r16
;****************************************************************************
Delay10mS:
	push		r16					;save register
	ldi			r16,10				;load loop counter
Delay10mSa:
	rcall		Delay1mS			;call 1ms delay
	dec			r16
	brne		Delay10mSa
	pop			r16
	ret
;;
;****************************************************************************
; DELAY100MS: Delays 100mS.
;	Destroys r16
;****************************************************************************
Delay100mS:
	push		r16					;save register
	ldi			r16,10				;load loop counter
Delay100mSa:
	rcall		Delay10mS			;call 10ms delay
	dec			r16
	brne		Delay100mSa
	pop			r16
	ret
;
#if USB
;****************************************************************************
; DELAY1S: Delays 1 Second.
;****************************************************************************
Delay1S:
	push	r16					;save registers
;	push	r28
;	push	r29
	PUSHY
	ldi		r16,100				;load loop count
Delay1Sa:
	rcall	Delay10mS			;delay 10mS
;	_WAIT_mS	f_clk,10
	dec		r16					;decrement
	brne	Delay1Sa			;loop till zero
	POPY
;	pop		r29					;restore register
;	pop		r28
	pop		r16
	ret
#endif
;
;****************************************************************************
; PrnLnNo: Print program line number held in RAM@CurrLine
;****************************************************************************
PrnLnNo:
	SetZPtr	(2*errlinenum)		;point to string table
	GoSub	sendromstring		;sent it.
	PUSHY						;save Y pointer
	clr		YH					;copy line # to YH:YL
	LOAD	YL,CurrLine
	GoSub	D3ASC				;print the line #
	POPY						;restore Y pointer
	ret							;return to caller
;
;*************************************************************
; ClrProgMem:	clear PROGMEM in RAM.  Sets up registers then
;	Calls "FillMem" routine
;*************************************************************
ClrProgMem:	;clear PROGMEM in RAM
	SetZPtr	PROGMEM				;start of PROGMEM in RAM
	LOADI16	inbyteh,inbytel,PROGMEMSZ	;length
	clr		temp				;set fill value
	rjmp	FillMem
;
;*************************************************************
; ClrVarMem: clear variable space in RAM.  Sets up registers then
;	Calls "FillMem" routine
;*************************************************************
ClrVarMem:	;clear PROGMEM in RAM
	SetZPtr	VARSTOR				;start of variable storage
	LOADI16	inbyteh,inbytel,VARSTORSZ	;length
	clr		temp				;set fill value
;	rjmp	FillMem
;	fall into "FillMem" routine
;
;*************************************************************
; FillMem: Clear contents of RAM
;	- Enter with start pointer in Z
;	- Enter with number of bytes in inbyteh:inbytel
;	- Enter with fill value in temp
;	- Uses U, Z, inbyteh:inbytel
;*************************************************************
FillMem:
	st		Z+,temp				;out in SRAM and decrement X-pointer
	SUBI16	inbyteh,inbytel,1	;decrement byte counter
	brne	FillMem				;loop till zero
	ret							;return to caller
;
;*************************************************************
; Dump_Perf:	Print a perferation to screen
;*************************************************************
Dump_Perf:
	SETZPTR	(2*perferate)		;print beginning "perforation"
	GoSub	sendromstring		;sent it.
	ret
;
;*************************************************************
; URXC1_int:	USART RX Complete ISR
;*************************************************************
;.if !USBSUPP
;URXC_int:
;URXC1_int:					; USART1 Rx Complete
;	push	temp
;	CLRB	UCSRA,RXC,temp	;clear USART RX complete flag
;	pop		temp
;	reti
;.endif
;
#if USI && \
	( defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) )
;*************************************************************
; D_USIINT:	Disable USI hardware interrupts, save state to RAM
;*************************************************************
D_USIINT:
	LOAD	temp,UART_status	;get UART status
	tst		temp				;any activity?
	brne	D_USIINT			;loop till no activity
	cli							;disable IRQs
	clr		r1					;zero a register
	GoSub	USI_UART_Close
	sei							;restore IRQs
	ret
;
;*************************************************************
; E_USIINT:	Restore USI hardware interrupts state from RAM
;*************************************************************
E_USIINT:
	cli
	GoSub	USI_UART_Initialise_Transmitter
	ret							;return to caller
;
#elif !USI && \
	( defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__) )
;*************************************************************
; D_UARTINT:	Disable USI hardware interrupts, save state to RAM
;*************************************************************
D_UARTINT:
	LOAD	inbytel,vuart_state		;get vuart_state
;	cpi		inbytel,(1<<TX_COMPLETE | \
;					 0<<RX_COMPLETE | \
;					 1<<DATA_EMPTY)
;	brne	D_UARTINT				;loop till no activity
	sbrs	inbytel,DATA_EMPTY		;skip next if TX ready
	rjmp	D_UARTINT				;loop till no activity

	;disable RX and TX timer IRQ's
	LOAD	inbytel,TIMSK			;fetch TIMSK register
    andi	inbytel,~(1<<OCIE0A | \
					  1<<OCIE0B)	;disable TIMER0 IRQ's
    STORE	TIMSK,inbytel
	ret								;return to caller
;
;*************************************************************
; E_UARTINT:	Restore USI hardware interrupts state from RAM
;*************************************************************
E_UARTINT:
	cli
	;disable RX and TX timer IRQ's
	LOAD	inbytel,TIMSK			;fetch TIMSK register
    ori		inbytel,(1<<OCIE0A | \
					 1<<OCIE0B)		;enable TIMER0 IRQ's
    STORE	TIMSK,inbytel
	ret								;return to caller
;
#endif
;
#if USB && (defined(__ATmega32U4__) || defined(__AT90USB1286__))
;*************************************************************
; D_USBINT:	Disable USB hardware interrupts, save state to RAM
;*************************************************************
D_USBINT:
	cli							;disable IRQs
	LOAD	temp,UDIEN			;fetch USB IRQ register
	tst		temp				;is register cleared?
	breq	D_USBINT1			;yes, already saved USB IRQ
	STORE	usb_irqstate,temp	;save in RAM
	STOREI	UDIEN,0				;disable USB interrupts
D_USBINT1:
	sei							;restore IRQs
	ret
;
;*************************************************************
; E_USBINT:	Restore USB hardware interrupts state from RAM
;*************************************************************
E_USBINT:
	cli							;disable IRQs
	LOAD	temp,usb_irqstate	;fetch from RAM
	STORE	UDIEN,temp			;fetch USB IRQ register
	sei							;restore IRQs
	ret							;return to caller
;
#endif
;
.if LPMD || DHT		; if Low-Power Mode OR DHT routines enabled
;****************************************************************************
; WDT_int: Watchdog Time-out interrupt used in conjuction with the
;	low-power sleep mode.  Disables watchdog before exit.
;****************************************************************************
WDT_int:
	STOREI	WDTCSR,(1<<WDCE|1<<WDE),temp
	STOREI	WDTCSR,0,temp			;disable watchdog
	SETB	GPIOR0,WDTIRQ,temp		;set WDTIRQ flag
	reti							;return to caller
;
.endif
;
;****************************************************************************
; Include code snippets
;****************************************************************************
.include "Include/Code_Editing.inc"		;Line editing routines
.include "Include/Code_EFS.inc"			;EEPROM/File System routines
.include "include/Code_Math.inc"		;include 16 and 32 bit math routines
.include "Include/Code_ERRPROC.inc"		;Error processing routine
.include "Include/Code_Init.inc"		;Machine initialization
.include "Include/Code_DataBytes.inc"	;RAM and EEP routines

.if SCLK
 .include "Include/Code_SysClock.inc"	;System Clock Prescaler routines
.endif
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
 .if TWI
; .include "Include/Code_TWI-usi.inc"	;USI TWI routines
 .endif
#else
 .if TWI
 .include "Include/Code_TWI.inc"		;TWI routines
 .endif
#endif

.if (SPI || DFR || NRF)
 .include "Include/Code_SPI.inc"		;SPI routines
.endif
.if PIO
 .include "Include/Code_PORTIO.inc"		;PORT I/O routines
.endif
.if PWM
 .include "Include/Code_PWM.inc"		;PWM routines
.endif
.include "Include/Code_Operators.inc"	;test operators
.if ADconv
 .include "Include/Code_Analog.inc"		;ADC routines
.endif
.if AComp
 .include "Include/Code_AComp.inc"		;Analog comp routines
.endif
.if ICP
 .include "Include/Code_ICP.inc"		;ICP routines
.endif
.if DSI
 .include "Include/Code_DS.inc"			;DS interface routines
.endif
.if DFR
 .include "Include/Code_DataFile.inc"	;Data File routines
.endif
.if LPMD
 .include "Include/Code_LOPWR.inc"		;Low-power routines
.endif
.if DDS
 .include "Include/Code_DDS.inc"		;DDS routines
.endif
.if RTC
 .include "Include/Code_RTC.inc"		;Real-time Counter routines
.endif
.if DHT
 .include "Include/Code_DHT.inc"		;DHTxx sensor routines
.endif
.if NRF
 .include "Include/Code_NRF.inc"		;nRF24L01(+) routines
.endif
.if OWR
 .include "Include/Code_OWR.inc"		;One-Wire routines
.endif
.if SPGM
 .include "Include/Code_SPM.inc"		;SPM routines
.endif
.if DEBUG
 .include "Include/Code_Debug.inc"		;Debug routines
.endif
.if RENUMBR
 .include "Include/Code_Renumber.inc"	;renumber routines
.endif
;.include "Include/Code_DeCompress.inc"
;
;************* Add in SIO Support *****************
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
 #if USI
 .include "Include/Code_sio_tinyusi.inc"	;USI as UART for ATtiny85
 #else
 .include "Include/Code_sio_tinyusart.inc"	;Soft-UART for ATtiny85
 #endif
;
;USART (not USB) support for ATmeag32U4 and AT90USB1286 parts
#elif !USB && ( defined(__ATmega32U4__) || defined(__AT90USB1286__) )
	.include "Include/Code_sio_usart.inc"
;
;USART support for Mega2560, Mega88/168/328, Mega16/32 and Mega644/1284 parts
#elif defined(__ATmega2560__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__) || \
	defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__)
	.include "Include/Code_sio_usart.inc"
#endif
;
;************* Add in Bootloader Support Command *****************
#if defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__) || \
	defined(__ATmega644P__) || \
	defined(__ATmega1284P__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
  .include "Include/Code_BLDR.inc"		;BootLoader access
#endif
;
;*****************************************************************
;	include data
.include "Include/Data_Cmds.inc"		;Command Table
.include "Include/Data_Msgs.inc"		;User messages
#if DREC	;only for Data Recorder
.include "Include/Data_DR_Prog.inc"		;default capture program
#endif
;
.include "Include/Code_IRQvectors.inc"	;default IRQ Vectors
;
;******************************************************************************
; Exec_SPM:  This is the SPM instruction,  For parts with NRWW
;  memory that does not start at address 0x0000, the SPM
;  instruction MUST be executed from the protected memory located
;  in the NRWW partition.  Thus we must find a place to locate
;  3 words (6 bytes) of instructions somewhere in that region,
;  among a possible boot-loader.  Sigh ...
;  CODE is enabled in the INCLUDE file
; - call with desired operation held in the temp2 register.
;******************************************************************************
#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
	defined(__ATtiny85__)
	.include "Include/Code_SPM_Exec.inc"
#endif
;
;******************************************************************************
;USB Serial I/O Support
;******************************************************************************
;	USB SIO support for Mega32U4 parts
#if USB && defined(__ATmega32U4__)
	.include "Include/Code_sio_usb.inc"
	.cseg
 	.org	__ctors_start		;start of ROM code + data
 #if FCLK == 8000000
	#message "Notice: USB support for ATmega32U4 @ 8MHz enabled"
	.include	"Include/Code_usb_serial_atmega32u4-8M.inc"
 #elif FCLK == 16000000
	#message "Notice: USB support for ATmega32U4 @ 16MHz enabled"
	.include	"Include/Code_usb_serial_atmega32u4-16M.inc"
 #else
	#message "Warning: Invalid clock specified for USB on Mega32U4!"
 #endif
;******************************************************************************
;	USB SIO support for USB1286 parts
#elif USB && defined(__AT90USB1286__)
	.include "Include/Code_sio_usb.inc"
	.cseg
 	.org	__ctors_start		;start of ROM code + data
 #if FCLK == 8000000
    #message "Notice: USB support for AT90USB1286 @ 8MHz enabled"
    .include	"Include/Code_usb_serial_at90usb1286-8M.inc"
 #elif FCLK == 16000000
  #if TEENSY
	#message "Notice: USB support for TEENSY++ 2.0 enabled"
	.include	"Include/Code_usb_serial_TEENSYPP20.inc"
  #else
	#message "Notice: USB support for AT90USB1286 @ 16MHz enabled"
	.include	"Include/Code_usb_serial_at90usb1286-16M.inc"
  #endif
 #else
	#message "Warning: Invalid clock specified for USB on AT90USB1286!"
 #endif
#endif
;
;******************************************************************************
;USI Serial I/O Support for ATtiny84/85
;******************************************************************************
;*********** Attiny84 USI Support ***************************
#if USI && (defined(__ATtiny84__) || defined(__ATtiny84A__));if using USI as UART
 .cseg
 .org __ctors_start		;start of ROM code + data
  #if FCLK == 4000000
	#message "Notice: USI support for ATtiny84 @ 4MHz enabled"
	.include "Include/Code_usi_uart_attiny84-4M.inc"
  #elif FCLK == 8000000
	#message "Notice: USI support for ATtiny84 @ 8MHz enabled"
	.include "Include/Code_usi_uart_attiny84-8M.inc"
  #elif FCLK == 10000000
	#message "Notice: USI support for ATtiny84 @ 10MHz enabled"
	.include "Include/Code_usi_uart_attiny84-10M.inc"
  #elif FCLK == 16000000
	#message "Notice: USI support for ATtiny84 @ 16MHz enabled"
	.include "Include/Code_usi_uart_attiny84-16M.inc"
  #elif FCLK == 20000000
	#message "Notice: USI support for ATtiny84 @ 20MHz enabled"
	.include "Include/Code_usi_uart_attiny84-20M.inc"
  #else
	#message "Warning: Unsupported clock specified for USI on ATtiny85!"
  #endif
;*********** Attiny85 USI Support ***************************
#elif USI && defined(__ATtiny85__)	;if using USI as UART
 .cseg
 .org __ctors_start		;start of ROM code + data
  #if FCLK == 4000000
	#message "Notice: USI support for ATtiny85 @ 4MHz enabled"
	.include "Include/Code_usi_uart_attiny85-4M.inc"
  #elif FCLK == 8000000
	#message "Notice: USI support for ATtiny85 @ 8MHz enabled"
	.include "Include/Code_usi_uart_attiny85-8M.inc"
  #elif FCLK == 10000000
	#message "Notice: USI support for ATtiny85 @ 10MHz enabled"
	.include "Include/Code_usi_uart_attiny85-10M.inc"
  #elif FCLK == 16000000
	#message "Notice: USI support for ATtiny85 @ 16MHz enabled"
	.include "Include/Code_usi_uart_attiny85-16M.inc"
  #elif FCLK == 20000000
	#message "Notice: USI support for ATtiny85 @ 20MHz enabled"
	.include "Include/Code_usi_uart_attiny85-20M.inc"
  #else
	#message "Warning: Unsupported clock specified for USI on ATtiny85!"
  #endif
;#else
; #if defined(__ATtiny85__) ;if using SoftUART
; .cseg
; .org SWUART		;start of ROM code + data
;  #if FCLK == 4000000
;	#message "Notice: Soft-UART support for ATtiny85 @ 4MHz enabled"
;	.include "Include/Code_vuart-attiny85-4M.inc"
;  #elif FCLK == 8000000
;	#message "Notice: Soft-UART support for ATtiny85 @ 8MHz enabled"
;	.include "Include/Code_vuart-attiny85-8M.inc"
;  #elif FCLK == 10000000
;	#message "Notice: Soft-UART support for ATtiny85 @ 10MHz enabled"
;	.include "Include/Code_vuart-attiny85-10M.inc"
;  #elif FCLK == 16000000
;	#message "Notice: Soft-UART support for ATtiny85 @ 16MHz enabled"
;	.include "Include/Code_vuart-attiny85-16M.inc"
;  #elif FCLK == 20000000
;	#message "Notice: Soft-UART support for ATtiny85 @ 20MHz enabled"
;	.include "Include/Code_vuart-attiny85-20M.inc"
;  #else
;	#message "Warning: Invalid clock specified for USI on ATtiny85!"
;  #endif
; #endif
#endif
;
;******************************************************************************
; DDS waveform lookup table MUST go before Boot-Loader
;******************************************************************************
.if DDS
	.include "Include/Data_SqrWav.inc"	;DDS waveform (8x compressed)
.endif
;
;******************************************************************************
; BootLoader add-in support
;******************************************************************************
#if BTLDR
	.include "Include/Code_Bootloader.inc"	;Bootloader determinations
#endif
;
;******************************************************************************
; Exec_SPM:  This is the SPM instruction,  For parts with NRWW
;  memory that does not start at address 0x0000, the SPM
;  instruction MUST be executed from the protected memory located
;  in the NRWW partition.  Thus we must find a place to locate
;  8 words (16 bytes) of instructions somewhere in that region,
;  among a possible boot-loader.  Sigh ...  As long as we control
;  the bootloader and the user programs the MCU with our bootloader
;  then there isn't a problem.
;  CODE is enabled within the INCLUDE file
; - call with desired operation held in the temp2 register.
;******************************************************************************
#if (!defined(__ATtiny84__) && !defined(__ATtiny84A__) && \
	!defined(__ATtiny85__))
 .if SPGM
	.org	FLASHEND - 13				;(Exec_SPM_end - Exec_SPM)
	.include "Include/Code_SPM_Exec.inc"
 .endif
#endif
;
#endif	;end conditional from inclusion of 2313/8515 & m163 code (at beginning of this file)
;******************************************************************************
;end of file
;******************************************************************************
;
