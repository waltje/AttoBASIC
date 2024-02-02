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
;****************************************************************************
; INTERUPT VECTORS - place holders, which can be placed at end of program
;	space.
;****************************************************************************
#if defined(__ATtiny84__) || defined(__ATtiny84A__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef PCINT0_int
  PCINT0_int:					; Pin Change Interrupt Request 0
.endif
.ifndef PCINT1_int
  PCINT1_int:					; Pin Change Interrupt Request 1
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef ICP1_int
  ICP1_int:						;Timer/Counter1 Capture Event
.endif
.ifndef DDS_int
   DDS_int:						;Timer/Counter1 Compare Match A
 .endif
.ifndef TIMER1_COMPA_int
   TIMER1_COMPA_int:				;Timer/Counter1 Compare Match A
.endif
.ifndef TIMER1_COMPB_int
  TIMER1_COMPB_int:				;Timer/Counter1 Compare Match B
.endif
.ifndef TIMER1_OVF_int
  TIMER1_OVF_int:				;Timer/Counter1 Overflow
.endif
.ifndef TIMER0_COMPA_int
  TIMER0_COMPA_int:				;Timer/Counter0 Compare Match A
.endif
.ifndef TIMER0_COMPB_int
  TIMER0_COMPB_int:				;Timer/Counter0 Compare Match B
.endif
.ifndef TIMER0_OVF_int
  TIMER0_OVF_int:				;Timer/Counter0 Overflow
.endif
.ifndef ANA_COMP_int
  ANA_COMP_int:					;Analog Comparator
.endif
.ifndef ADCC_int
  ADCC_int:						;ADC Conversion Complete
.endif
.ifndef EE_RDY_int
  EE_RDY_int:					;EEPROM Ready
.endif
.ifndef USI_START_int
  USI_START_int:				;USI START
.endif
.ifndef USI_OVF_int
  USI_OVF_int:					;USI Overflow
.endif
;
#elif defined(__ATtiny85__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef PCINT0_int
  PCINT0_int:					; Pin Change Interrupt Request 0
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef TIMER1_COMPA_int
  TIMER1_COMPA_int:				;Timer/Counter1 Compare Match A
.endif
.ifndef TIMER1_OVF_int
  TIMER1_OVF_int:				;Timer/Counter1 Overflow
.endif
.ifndef TIMER0_OVF_int
  TIMER0_OVF_int:				;Timer/Counter0 Overflow
.endif
.ifndef EE_RDY_int
  EE_RDY_int:					;EEPROM Ready
.endif
.ifndef ANA_COMP_int
  ANA_COMP_int:					;Analog Comparator
.endif
.ifndef ADCC_int
  ADCC_int:						;ADC Conversion Complete
.endif
.ifndef TIMER1_COMPB_int
  TIMER1_COMPB_int:				;Timer/Counter1 Compare Match B
.endif
.ifndef TIMER0_COMPA_int
  TIMER0_COMPA_int:				;Timer/Counter0 Compare Match A
.endif
.ifndef TIMER0_COMPB_int
  TIMER0_COMPB_int:				;Timer/Counter0 Compare Match B
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef USI_START_int
  USI_START_int:				;USI START
.endif
.ifndef USI_OVF_int
  USI_OVF_int:					;USI Overflow
.endif
;
#elif defined(__ATmega88__) || defined(__ATmega88PA__) || \
	defined(__ATmega168__) || defined(__ATmega168PA__) || \
	defined(__ATmega328__) || defined(__ATmega328P__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef PCI0_int
  PCI0_int:						; Pin Change Interrupt Request 0
.endif
.ifndef PCI1_int
  PCI1_int:						; Pin Change Interrupt Request 1
.endif
.ifndef PCI2_int
  PCI2_int:						; Pin Change Interrupt Request 2
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2A_int
  OC2A_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2B_int
  OC2B_int:						; Timer/Counter2 Compare Match B
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0A_int
  OC0A_int:						; Timer/Counter0 Compare Match A
.endif
.ifndef OC0B_int
  OC0B_int:						; Timer/Counter0 Compare Match B
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Counter0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC_int
  URXC_int:						; USART1 Rx Complete
.endif
.ifndef UDRE_int
  UDRE_int:						; USART, Data Register Empty
.endif
.ifndef UTXC_int
  UTXC_int:						; USART Tx Complete
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; Self-Program Memory Read
.endif
;
#elif defined(__ATmega16__) || defined(__ATmega16L__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Counter0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC_int
  URXC_int:						; USART1 Rx Complete
.endif
.ifndef UDRE_int
  UDRE_int:						; USART, Data Register Empty
.endif
.ifndef UTXC_int
  UTXC_int:						; USART Tx Complete
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef OC0_int
  OC0_int:						; Timer/Counter0 Compare Match
.endif
.ifndef SPMR_int
  SPMR_int:						; Self-Program Memory Read
.endif
;
#elif defined(__ATmega32__) || defined(__ATmega32A__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0_int
  OC0_int:						; Timer/Counter0 Compare Match
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Counter0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC_int
  URXC_int:						; USART1 Rx Complete
.endif
.ifndef UDRE_int
  UDRE_int:						; USART, Data Register Empty
.endif
.ifndef UTXC_int
  UTXC_int:						; USART Tx Complete
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; Self-Program Memory Read
.endif
;
#elif defined(__ATmega32U4__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef INT3_int
  INT3_int:						; External Interrupt Request 3
.endif
.ifndef Res1addr_int
  Res1addr_int:					; Reserved1
.endif
.ifndef Res2addr_int
  Res2addr_int:					; Reserved2
.endif
.ifndef INT6_int
  INT6_int:						; External Interrupt Request 6
.endif
.ifndef Res3addr_int
  Res3addr_int:					; Reserved3
.endif
.ifndef PCI0_int
  PCI0_int:						; Pin Change Interrupt Request 0
.endif
.ifndef USB_GEN_int
  USB_GEN_int:					; USB General Interrupt Request
.endif
.ifndef USB_COM_int
  USB_COM_int:					; USB Endpoint/Pipe Interrupt Communication Request
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef Res4addr_int
  Res4addr_int:					; Reserved4
.endif
.ifndef Res5addr_int
  Res5addr_int:					; Reserved5
.endif
.ifndef Res6addr_int
  Res6addr_int:					; Reserved6
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OC1C_int
  OC1C_int:						; Timer/Counter1 Compare Match C
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0A_int
  OC0A_int:						; TimerCounter0 Compare Match A
.endif
.ifndef OC0B_int
  OC0B_int:						; TimerCounter0 Compare Match B
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Couner0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC1_int
  URXC1_int:						; USART1 Rx Complete
.endif
.ifndef UDRE1_int
  UDRE1_int:						; USART1, Data Register Empty
.endif
.ifndef UTXC1_int
  UTXC1_int:						; USART1 Tx Complete
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef ICP3_int
  ICP3_int:						; Timer/Counter3 Capture Event
.endif
.ifndef OC3A_int
  OC3A_int:						; Timer/Counter3 Compare Match A
.endif
.ifndef OC3B_int
  OC3B_int:						; Timer/Counter3 Compare Match B
.endif
.ifndef OC3C_int
  OC3C_int:						; Timer/Counter3 Compare Match C
.endif
.ifndef OVF3_int
  OVF3_int:						; Timer/Couner3 Overflow
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; out Program Memory Read
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter4 Compare Match A
.endif
.ifndef OC4A_int
  OC4A_int:						; Timer/Counter4 Compare Match A
.endif
.ifndef OC4B_int
  OC4B_int:						; Timer/Counter4 Compare Match B
.endif
.ifndef OC4D_int
  OC4D_int:						; Timer/Counter4 Compare Match C
.endif
.ifndef OVF4_int
  OVF4_int:						; Timer/Couner4 Overflow
.endif
.ifndef FPF4_int
  FPF4_int:						; Timer/Counter4 Fault Protection Interrupt
.endif
#elif defined(__AT90USB1286__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef INT3_int
  INT3_int:						; External Interrupt Request 3
.endif
.ifndef INT4_int
  INT4_int:						; External Interrupt Request 4
.endif
.ifndef INT5_int
  INT5_int:						; External Interrupt Request 5
.endif
.ifndef INT6_int
  INT6_int:						; External Interrupt Request 6
.endif
.ifndef INT7_int
  INT7_int:						; External Interrupt Request 7
.endif
.ifndef PCI0_int
  PCI0_int:						; Pin Change Interrupt Request 0
.endif
.ifndef USB_GEN_int
  USB_GEN_int:					; USB General Interrupt Request
.endif
.ifndef USB_COM_int
  USB_COM_int:					; USB Endpoint/Pipe Interrupt Communication Request
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2A_int
  OC2A_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2B_int
  OC2B_int:						; Timer/Counter2 Compare Match B
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OC1C_int
  OC1C_int:						; Timer/Counter1 Compare Match C
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0A_int
  OC0A_int:						; TimerCounter0 Compare Match A
.endif
.ifndef OC0B_int
  OC0B_int:						; TimerCounter0 Compare Match B
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Couner0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC1_int
  URXC1_int:						; USART1 Rx Complete
.endif
.ifndef UDRE1_int
  UDRE1_int:						; USART1, Data Register Empty
.endif
.ifndef UTXC1_int
  UTXC1_int:						; USART1 Tx Complete
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef ICP3_int
  ICP3_int:						; Timer/Counter3 Capture Event
.endif
.ifndef OC3A_int
  OC3A_int:						; Timer/Counter3 Compare Match A
.endif
.ifndef OC3B_int
  OC3B_int:						; Timer/Counter3 Compare Match B
.endif
.ifndef OC3C_int
  OC3C_int:						; Timer/Counter3 Compare Match C
.endif
.ifndef OVF3_int
  OVF3_int:						; Timer/Couner3 Overflow
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; out Program Memory Read
.endif
#elif defined(__ATmega644P__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef PCI0_int
  PCI0_int:						; Pin Change Interrupt Request 0
.endif
.ifndef PCI1_int
  PCI1_int:						; Pin Change Interrupt Request 1
.endif
.ifndef PCI2_int
  PCI2_int:						; Pin Change Interrupt Request 2
.endif
.ifndef PCI3_int
  PCI3_int:						; Pin Change Interrupt Request 3
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2A_int
  OC2A_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2B_int
  OC2B_int:						; Timer/Counter2 Compare Match B
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OC1C_int
  OC1C_int:						; Timer/Counter1 Compare Match C
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0A_int
  OC0A_int:						; TimerCounter0 Compare Match A
.endif
.ifndef OC0B_int
  OC0B_int:						; TimerCounter0 Compare Match B
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Couner0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC0_int
  URXC0_int:						; USART0 Rx Complete
.endif
.ifndef UDRE0_int
  UDRE0_int:						; USART0, Data Register Empty
.endif
.ifndef UTXC0_int
  UTXC0_int:						; USART0 Tx Complete
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; out Program Memory Read
.endif
#elif defined(__ATmega1284P__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef PCI0_int
  PCI0_int:						; Pin Change Interrupt Request 0
.endif
.ifndef PCI1_int
  PCI1_int:						; Pin Change Interrupt Request 1
.endif
.ifndef PCI2_int
  PCI2_int:						; Pin Change Interrupt Request 2
.endif
.ifndef PCI3_int
  PCI3_int:						; Pin Change Interrupt Request 3
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2A_int
  OC2A_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2B_int
  OC2B_int:						; Timer/Counter2 Compare Match B
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OC1C_int
  OC1C_int:						; Timer/Counter1 Compare Match C
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0A_int
  OC0A_int:						; TimerCounter0 Compare Match A
.endif
.ifndef OC0B_int
  OC0B_int:						; TimerCounter0 Compare Match B
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Couner0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC0_int
  URXC0_int:						; USART0 Rx Complete
.endif
.ifndef UDRE0_int
  UDRE0_int:						; USART0, Data Register Empty
.endif
.ifndef UTXC0_int
  UTXC0_int:						; USART0 Tx Complete
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; out Program Memory Read
.endif
.ifndef URXC1_int
  URXC1_int:						; USART1 Rx Complete
.endif
.ifndef UDRE1_int
  UDRE1_int:						; USART1, Data Register Empty
.endif
.ifndef UTXC1_int
  UTXC1_int:						; USART1 Tx Complete
.endif
#elif defined(__ATmega2560__)
.ifndef INT0_int
  INT0_int:						; External Interrupt Request 0
.endif
.ifndef INT1_int
  INT1_int:						; External Interrupt Request 1
.endif
.ifndef INT2_int
  INT2_int:						; External Interrupt Request 2
.endif
.ifndef INT3_int
  INT3_int:						; External Interrupt Request 3
.endif
.ifndef INT4_int
  INT4_int:						; External Interrupt Request 4
.endif
.ifndef INT5_int
  INT5_int:						; External Interrupt Request 5
.endif
.ifndef INT6_int
  INT6_int:						; External Interrupt Request 6
.endif
.ifndef INT7_int
  INT7_int:						; External Interrupt Request 7
.endif
.ifndef PCI0_int
  PCI0_int:						; Pin Change Interrupt Request 0
.endif
.ifndef PCI1_int
  PCI1_int:						; Pin Change Interrupt Request 1
.endif
.ifndef PCI2_int
  PCI2_int:						; Pin Change Interrupt Request 2
.endif
.ifndef WDT_int
  WDT_int:						; Watchdog Time-out Interrupt
.endif
.ifndef DDS_int
  DDS_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2A_int
  OC2A_int:						; Timer/Counter2 Compare Match A
.endif
.ifndef OC2B_int
  OC2B_int:						; Timer/Counter2 Compare Match B
.endif
.ifndef OVF2_int
  OVF2_int:						; Timer/Counter2 Overflow
.endif
.ifndef ICP1_int
  ICP1_int:						; Timer/Counter1 Capture Event
.endif
.ifndef OC1A_int
  OC1A_int:						; Timer/Counter1 Compare Match A
.endif
.ifndef OC1B_int
  OC1B_int:						; Timer/Counter1 Compare Match B
.endif
.ifndef OC1C_int
  OC1C_int:						; Timer/Counter1 Compare Match C
.endif
.ifndef OVF1_int
  OVF1_int:						; Timer/Counter1 Overflow
.endif
.ifndef OC0A_int
  OC0A_int:						; TimerCounter0 Compare Match A
.endif
.ifndef OC0B_int
  OC0B_int:						; TimerCounter0 Compare Match B
.endif
.ifndef OVF0_int
  OVF0_int:						; Timer/Couner0 Overflow
.endif
.ifndef SPI_int
  SPI_int:						; SPI Serial Transfer Complete
.endif
.ifndef URXC0_int
  URXC0_int:						; USART0 Rx Complete
.endif
.ifndef UDRE0_int
  UDRE0_int:						; USART0, Data Register Empty
.endif
.ifndef UTXC0_int
  UTXC0_int:						; USART0 Tx Complete
.endif
.ifndef ACI_int
  ACI_int:						; Analog Comparator
.endif
.ifndef ADCC_int
  ADCC_int:						; ADC Conversion Complete
.endif
.ifndef ERDY_int
  ERDY_int:						; EEPROM Ready
.endif
.ifndef ICP3_int
  ICP3_int:						; Timer/Counter3 Capture Event
.endif
.ifndef OC3A_int
  OC3A_int:						; Timer/Counter3 Compare Match A
.endif
.ifndef OC3B_int
  OC3B_int:						; Timer/Counter3 Compare Match B
.endif
.ifndef OC3C_int
  OC3C_int:						; Timer/Counter3 Compare Match C
.endif
.ifndef OVF3_int
  OVF3_int:						; Timer/Couner3 Overflow
.endif
.ifndef URXC1_int
  URXC1_int:						; USART1 Rx Complete
.endif
.ifndef UDRE1_int
  UDRE1_int:						; USART1, Data Register Empty
.endif
.ifndef UTXC1_int
  UTXC1_int:						; USART1 Tx Complete
.endif
.ifndef TWI_int
  TWI_int:						; Two-wire Serial Interface
.endif
.ifndef SPMR_int
  SPMR_int:						; out Program Memory Read
.endif
.ifndef ICP4_int
  ICP4_int:						; Timer/Counter4 Capture Event
.endif
.ifndef OC4A_int
  OC4A_int:						; Timer/Counter4 Compare Match A
.endif
.ifndef OC4B_int
  OC4B_int:						; Timer/Counter4 Compare Match B
.endif
.ifndef OC4C_int
  OC4C_int:						; Timer/Counter4 Compare Match C
.endif
.ifndef OVF4_int
  OVF4_int:						; Timer/Couner4 Overflow
.endif
.ifndef ICP5_int
  ICP5_int:						; Timer/Counter5 Capture Event
.endif
.ifndef OC5A_int
  OC5A_int:						; Timer/Counter5 Compare Match A
.endif
.ifndef OC5B_int
  OC5B_int:						; Timer/Counter5 Compare Match B
.endif
.ifndef OC5C_int
  OC5C_int:						; Timer/Counter5 Compare Match C
.endif
.ifndef OVF5_int
  OVF5_int:						; Timer/Couner5 Overflow
.endif
.ifndef URXC2_int
  URXC2_int:						; USART2 Rx Complete
.endif
.ifndef UDRE2_int
  UDRE2_int:						; USART2, Data Register Empty
.endif
.ifndef UTXC2_int
  UTXC2_int:						; USART2 Tx Complete
.endif
.ifndef URXC3_int
  URXC3_int:						; USART3 Rx Complete
.endif
.ifndef UDRE3_int
  UDRE3_int:						; USART3, Data Register Empty
.endif
.ifndef UTXC3_int
  UTXC3_int:						; USART3 Tx Complete
.endif
#endif
NO_VEC:
	reti							;return to caller
;#if defined(__ATtiny84__) || defined(__ATtiny84A__) || \
;	defined(__ATtiny85__) || defined(__ATmega88__) || defined(__ATmega88PA__)
;	reti							;return to caller
;#else
;	SetZPtr	(2*emessage20)			;set pointer to message
;	GoSub	sendromstring			;sent it.
;	Goto	RSTcommand				;reset the system
;#endif
;
