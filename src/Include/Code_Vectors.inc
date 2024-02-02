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
;*********************************************************************
; Interrupt vectors placed at start of program 0x0000
;*********************************************************************
	.cseg
	.org	0x0000
#if defined(__ATtiny84__)
	Goto	RESET				;Reset vector
	Goto	INT0_int			;External Interrupt Request 0
	Goto	PCINT0_int			;Pin Change Interrupt Request 0
	Goto	PCINT1_int			;Pin Change Interrupt Request 1
	Goto	WDT_int				;Watchdog Time-out
	Goto	ICP1_int			;Timer/Counter1 Capture Event
	Goto	DDS_int				;Timer/Counter1 Compare Match A
	Goto	TIMER1_COMPB_int	;Timer/Counter1 Compare Match B
 	Goto	TIMER1_OVF_int		;Timer/Counter1 Overflow
	Goto	TIMER0_COMPA_int	;Timer/Counter0 Compare Match A
	Goto	TIMER0_COMPB_int	;Timer/Counter0 Compare Match B
	Goto	TIMER0_OVF_int		;Timer/Counter0 Overflow
#if BTLDR
	Goto	RESET				;for OptiBoot85 AppStart vector support
#else
	Goto	ANA_COMP_int		;Analog Comparator
#endif
	Goto	ADCC_int			;ADC Conversion Complete
	Goto	EE_RDY_int			;EEPROM Ready
	Goto	USI_START_int		;USI START
	Goto	USI_OVF_int			;USI Overflow
;
#elif defined(__ATtiny85__)
	Goto	RESET				;Reset vector
	Goto	INT0_int			;External Interrupt Request 0
	Goto	PCINT0_int			;Pin Change Interrupt Request 0
	Goto	DDS_int				;Timer/Counter1 Compare Match A
 	Goto	TIMER1_OVF_int		;Timer/Counter1 Overflow
	Goto	TIMER0_OVF_int		;Timer/Counter0 Overflow
	Goto	EE_RDY_int			;EEPROM Ready
#if BTLDR
	Goto	RESET				;for OptiBoot85 AppStart vector support
#else
	Goto	ANA_COMP_int		;Analog Comparator
#endif
	Goto	ADCC_int			;ADC Conversion Complete
	Goto	TIMER1_COMPB_int	;Timer/Counter1 Compare Match B
	Goto	TIMER0_COMPA_int	;Timer/Counter0 Compare Match A
	Goto	TIMER0_COMPB_int	;Timer/Counter0 Compare Match B
	Goto	WDT_int				;Watchdog Time-out
	Goto	USI_START_int		;USI START
	Goto	USI_OVF_int			;USI Overflow
;
#elif defined(__ATmega48__) || defined(__ATmega88__) || defined(__ATmega88PA__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	PCI1_int			; Pin Change Interrupt Request 1
	Goto	PCI2_int			; Pin Change Interrupt Request 2
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	DDS_int				;Timer/Counter2 Compare Match A
	Goto	OC2B_int			; Timer/Counter2 Compare Match B
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC_int			; USART Rx Complete
	Goto	UDRE_int			; USART, Data Register Empty
	Goto	UTXC_int			; USART Tx Complete
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ACI_int				; Analog Comparator
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
#elif defined(__ATmega168__) || defined(__ATmega168PA__) || \
	defined(__ATmega328__) || defined(__ATmega328P__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	PCI1_int			; Pin Change Interrupt Request 1
	Goto	PCI2_int			; Pin Change Interrupt Request 2
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	DDS_int				;Timer/Counter2 Compare Match A
	Goto	OC2B_int			; Timer/Counter2 Compare Match B
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC_int			; USART Rx Complete
	Goto	UDRE_int			; USART, Data Register Empty
	Goto	UTXC_int			; USART Tx Complete`
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ACI_int				; Analog Comparator
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
#elif defined(__ATmega16__) || defined(__ATmega16L__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	DDS_int				; Timer/Counter2 Compare Match
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC_int			; USART Rx Complete
	Goto	UDRE_int			; USART, Data Register Empty
	Goto	UTXC_int			; USART Tx Complete`
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ACI_int				; Analog Comparator
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	INT2_int			; External Interrupt Request 1
	Goto	OC0_int				; TimerCounter0 Compare Match
	Goto	SPMR_int			; Store Program Memory Read
#elif	defined(__ATmega32__) || defined(__ATmega32A__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	INT2_int			; External Interrupt Request 1
	Goto	DDS_int				; Timer/Counter2 Compare Match
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0_int				; TimerCounter0 Compare Match
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC_int			; USART Rx Complete
	Goto	UDRE_int			; USART, Data Register Empty
	Goto	UTXC_int			; USART Tx Complete`
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ACI_int				; Analog Comparator
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
#elif defined(__ATmega32U4__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	INT2_int			; External Interrupt Request 2
	Goto	INT3_int			; External Interrupt Request 3
	Goto	Res1addr_int		; Reserved1
	Goto	Res2addr_int		; Reserved2
	Goto	INT6_int			; External Interrupt Request 6
	Goto	Res3addr_int		; Reserved3
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	USB_GEN_int			; USB General Interrupt Request
	Goto	USB_COM_int			; USB Endpoint/Pipe Interrupt Communication Request
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	Res4addr_int		; Reserved4
	Goto	Res5addr_int		; Reserved5
	Goto	Res6addr_int		; Reserved6
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OC1C_int			; Timer/Counter1 Compare Match C
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC1_int			; USART1 Rx Complete
	Goto	UDRE1_int			; USART1, Data Register Empty
	Goto	UTXC1_int			; USART1 Tx Complete
	Goto	ACI_int				; Analog Comparator
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ICP3_int			; Timer/Counter3 Capture Event
	Goto	OC3A_int			; Timer/Counter3 Compare Match A
	Goto	OC3B_int			; Timer/Counter3 Compare Match B
	Goto	OC3C_int			; Timer/Counter3 Compare Match C
	Goto	OVF3_int			; Timer/Couner3 Overflow
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
	Goto	DDS_int				; Timer/Counter4 Compare Match A
	Goto	OC4B_int			; Timer/Counter4 Compare Match B
	Goto	OC4D_int			; Timer/Counter4 Compare Match C
	Goto	OVF4_int			; Timer/Counter4 Overflow
	Goto	FPF4_int			; Timer/Counter4 Fault Protection Interrupt
#elif defined(__AT90USB1286__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	INT2_int			; External Interrupt Request 2
	Goto	INT3_int			; External Interrupt Request 3
	Goto	INT4_int			; External Interrupt Request 4
	Goto	INT5_int			; External Interrupt Request 5
	Goto	INT6_int			; External Interrupt Request 6
	Goto	INT7_int			; External Interrupt Request 7
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	USB_GEN_int			; USB General Interrupt Request
	Goto	USB_COM_int			; USB Endpoint/Pipe Interrupt Communication Request
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	DDS_int				; Timer/Counter2 Compare Match A
	Goto	OC2B_int			; Timer/Counter2 Compare Match B
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OC1C_int			; Timer/Counter1 Compare Match C
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC1_int			; USART1 Rx Complete
	Goto	UDRE1_int			; USART1, Data Register Empty
	Goto	UTXC1_int			; USART1 Tx Complete
	Goto	ACI_int				; Analog Comparator
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ICP3_int			; Timer/Counter3 Capture Event
	Goto	OC3A_int			; Timer/Counter3 Compare Match A
	Goto	OC3B_int			; Timer/Counter3 Compare Match B
	Goto	OC3C_int			; Timer/Counter3 Compare Match C
	Goto	OVF3_int			; Timer/Couner3 Overflow
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
#elif defined(__ATmega644P__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	INT2_int			; External Interrupt Request 2
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	PCI1_int			; Pin Change Interrupt Request 1
	Goto	PCI2_int			; Pin Change Interrupt Request 2
	Goto	PCI3_int			; Pin Change Interrupt Request 3
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	DDS_int				; Timer/Counter2 Compare Match A
	Goto	OC2B_int			; Timer/Counter2 Compare Match B
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC0_int			; USART0 Rx Complete
	Goto	UDRE0_int			; USART0, Data Register Empty
	Goto	UTXC0_int			; USART0 Tx Complete
	Goto	ACI_int				; Analog Comparator
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
#elif defined(__ATmega1284P__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	INT2_int			; External Interrupt Request 2
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	PCI1_int			; Pin Change Interrupt Request 1
	Goto	PCI2_int			; Pin Change Interrupt Request 2
	Goto	PCI3_int			; Pin Change Interrupt Request 3
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	DDS_int				; Timer/Counter2 Compare Match A
	Goto	OC2B_int			; Timer/Counter2 Compare Match B
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC0_int			; USART0 Rx Complete
	Goto	UDRE0_int			; USART0, Data Register Empty
	Goto	UTXC0_int			; USART0 Tx Complete
	Goto	ACI_int				; Analog Comparator
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
	Goto	URXC1_int			; USART1 Rx Complete
	Goto	UDRE1_int			; USART1, Data Register Empty
	Goto	UTXC1_int			; USART1 Tx Complete
#elif defined(__ATmega2560__)
	Goto	RESET				; Reset vector
	Goto	INT0_int			; External Interrupt Request 0
	Goto	INT1_int			; External Interrupt Request 1
	Goto	INT2_int			; External Interrupt Request 2
	Goto	INT3_int			; External Interrupt Request 3
	Goto	INT4_int			; External Interrupt Request 4
	Goto	INT5_int			; External Interrupt Request 5
	Goto	INT6_int			; External Interrupt Request 6
	Goto	INT7_int			; External Interrupt Request 7
	Goto	PCI0_int			; Pin Change Interrupt Request 0
	Goto	PCI1_int			; Pin Change Interrupt Request 1
	Goto	PCI2_int			; Pin Change Interrupt Request 2
	Goto	WDT_int				; Watchdog Time-out Interrupt
	Goto	DDS_int				; Timer/Counter2 Compare Match A
	Goto	OC2B_int			; Timer/Counter2 Compare Match B
	Goto	OVF2_int			; Timer/Counter2 Overflow
	Goto	ICP1_int			; Timer/Counter1 Capture Event
	Goto	OC1A_int			; Timer/Counter1 Compare Match A
	Goto	OC1B_int			; Timer/Counter1 Compare Match B
	Goto	OC1C_int			; Timer/Counter1 Compare Match C
	Goto	OVF1_int			; Timer/Counter1 Overflow
	Goto	OC0A_int			; TimerCounter0 Compare Match A
	Goto	OC0B_int			; TimerCounter0 Compare Match B
	Goto	OVF0_int			; Timer/Couner0 Overflow
	Goto	SPI_int				; SPI Serial Transfer Complete
	Goto	URXC0_int			; USART0 Rx Complete
	Goto	UDRE0_int			; USART0, Data Register Empty
	Goto	UTXC0_int			; USART0 Tx Complete
	Goto	ACI_int				; Analog Comparator
	Goto	ADCC_int			; ADC Conversion Complete
	Goto	ERDY_int			; EEPROM Ready
	Goto	ICP3_int			; Timer/Counter3 Capture Event
	Goto	OC3A_int			; Timer/Counter3 Compare Match A
	Goto	OC3B_int			; Timer/Counter3 Compare Match B
	Goto	OC3C_int			; Timer/Counter3 Compare Match C
	Goto	OVF3_int			; Timer/Couner3 Overflow
	Goto	URXC1_int			; USART1 Rx Complete
	Goto	UDRE1_int			; USART1, Data Register Empty
	Goto	UTXC1_int			; USART1 Tx Complete
	Goto	TWI_int				; Two-wire Serial Interface
	Goto	SPMR_int			; Store Program Memory Read
	Goto	ICP4_int			; Timer/Counter4 Capture Event
	Goto	OC4A_int			; Timer/Counter4 Compare Match A
	Goto	OC4B_int			; Timer/Counter4 Compare Match B
	Goto	OC4C_int			; Timer/Counter4 Compare Match C
	Goto	OVF4_int			; Timer/Couner4 Overflow
	Goto	ICP5_int			; Timer/Counter5 Capture Event
	Goto	OC5A_int			; Timer/Counter5 Compare Match A
	Goto	OC5B_int			; Timer/Counter5 Compare Match B
	Goto	OC5C_int			; Timer/Counter5 Compare Match C
	Goto	OVF5_int			; Timer/Couner5 Overflow
	Goto	URXC2_int			; USART2 Rx Complete
	Goto	UDRE2_int			; USART2, Data Register Empty
	Goto	UTXC2_int			; USART2 Tx Complete
	Goto	URXC3_int			; USART3 Rx Complete
	Goto	UDRE3_int			; USART3, Data Register Empty
	Goto	UTXC3_int			; USART3 Tx Complete
#endif

