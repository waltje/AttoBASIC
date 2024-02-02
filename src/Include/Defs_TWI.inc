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
/*#if defined(__ATtiny25__) | defined(__ATtiny45__) | defined(__ATtiny85__) | defined(__AT90Tiny26__) | defined(__ATtiny26__)*/
    #define DDR_USI             DDRB
    #define PORT_USI            PORTB
    #define PIN_USI             PINB
    #define PORT_USI_SDA        PORTB0
    #define PORT_USI_SCL        PORTB2
    #define PIN_USI_SDA         PINB0
    #define PIN_USI_SCL         PINB2
/*#endif*/



.ifdef TWDR
  .if TWI
;****************************************************************************
;	TWI definitions:
;****************************************************************************
;	Bit and byte definitions
;****************************************************************************/
.equ	TWI_TIMEOUT	= 20				; device response timeout in mS
;
;Select TWI clock frequency
.equ	TWI_CLK400	= 400000			; 400 kHz
.equ	TWI_CLK100	= 100000			; 100 kHz
.equ	TWI_CLK075	= 75000				;  75 kHz
.equ	TWI_CLK050	= 50000				;  50 kHz
.equ	TWI_CLK025	= 25000				;  25 kHz
;
;SCL frequency = F_CPU / ((16 + (2 * TWBR)) * TWSR)
.set	TWBCLK400K 	= ((f_clk/TWI_CLK400)-16)/2	;400K register value
.set	TWBCLK100K 	= ((f_clk/TWI_CLK100)-16)/2	;100K register value
.set	TWBCLK075K 	= ((f_clk/TWI_CLK075)-16)/2	; 75K register value
.set	TWBCLK050K 	= ((f_clk/TWI_CLK050)-16)/2	; 50K register value
.set	TWBCLK025K 	= ((f_clk/TWI_CLK025)-16)/2	; 25K register value
;
.if (TWBCLK400K > 9 && TWBCLK400K <255)
  .set	TWBCLK		= TWBCLK400K
  .message "TWI @ 400K available"
.elif (TWBCLK100K > 9 && TWBCLK100K <255)
  .set	TWBCLK			= TWBCLK100K
  .message "Only TWI @ 100K available"
.elif (TWBCLK075K > 9 && TWBCLK075K <255)
  .set	TWBCLK			= TWBCLK075K
  .message "Only TWI @ 075K  available"
.elif (TWBCLK050K > 9 && TWBCLK050K <255)
  .set	TWBCLK			= TWBCLK050K
  .message "Only TWI @ 050K available"
.elif (TWBCLK025K > 9 && TWBCLK025K <255)
  .set	TWBCLK			= TWBCLK025K
  .message "Only TWI @ 025K  available"
.else
  .error "!! System Clock is too low for TWI operation !!"
.endif
;
;****************************************************************************
;TWI Port definitions
;****************************************************************************
#if defined(__ATmega88__) || \
	defined(__ATmega88PA__) || \
	defined(__ATmega168__) || \
	defined(__ATmega168PA__) || \
	defined(__ATmega328__) || \
	defined(__ATmega328P__)
.equ	TWI_PORT	= PORTC			;TWI Port
.equ	TWI_SCL		= PC5			;Pin for TWI SCL
.equ	TWI_SDA		= PC4			;Pin for TWI SDA
.equ	PRRTWI		= PRR			;PRR register for TWI interface
;-----------------------------------------------------------
#elif defined(__ATmega16__) || \
	defined(__ATmega16L__) || \
	defined(__ATmega32__) || \
	defined(__ATmega32A__) 
.equ	TWI_PORT	= PORTC			;TWI Port
.equ	TWI_SCL		= PC0			;Pin for TWI SCL
.equ	TWI_SDA		= PC1			;Pin for TWI SDA
;-----------------------------------------------------------
#elif defined(__ATmega644P__) || \
	  defined(__ATmega1284P__) 
.equ	TWI_PORT	= PORTC			;TWI Port
.equ	TWI_SCL		= PC0			;Pin for TWI SCL
.equ	TWI_SDA		= PC1			;Pin for TWI SDA
.equ	PRRTWI		= PRR0			;PRR register for TWI interface
;-----------------------------------------------------------
#elif defined(__ATmega32U4__) || \
	defined(__AT90USB1286__) || \
	defined(__ATmega2560__)
.equ	TWI_PORT	= PORTD			;TWI Port
.equ	TWI_SCL		= PORTD0		;Pin for TWI SCL
.equ	TWI_SDA		= PORTD1		;Pin for TWI SDA
.equ	PRRTWI		= PRR0			;PRR register for TWI interface
;-----------------------------------------------------------
#elif defined(__ATtiny85__)
.equ	TWI_PORT	= PORTB			;TWI port
.equ	TWI_SCL		= PORTB2		;Pin for TWI SCL
.equ	TWI_SDA		= PORTB0		;Pin for TWI SDA
.equ	PRRTWI		= PRR			;PRR register for USI interface
;
;.equ	DDR_USI			= DDRB
;.equ	PORT_USI		= PORTB
;.equ	PIN_USI			= PINB
;.equ	PORT_USI_SDA	= PORTB0
;.equ	PORT_USI_SCL	= PORTB2
;.equ	PIN_USI_SDA		= PINB0
;.equ	PIN_USI_SCL		= PINB2
#endif
;
;****************************************************************************
;TWI State machine definitions (these are not used)
;****************************************************************************
.equ	TW_Idle		= 0						;TWI in idle condition
.equ	TW_SndStart	= 1						;TWI to send START condition
.equ	TW_SndAddr	= 2						;TWI to send SLAVE address
.equ	TW_SndData	= 3						;TWI to send data from buffer
.equ	TW_GetDatAck= 4						;TWI to get data and store in
;											; buffer, expecting ACK
.equ	TW_GetDatNak= 5						;TWI to get data and store in
;											; buffer, expecting NACK
.equ	TW_SndStop	= 6						;TWI to send STOP condition
.equ	TW_Error	= 7						;TWI has detected an error
;
;****************************************************************************
;TWI Status register definitions
;****************************************************************************
.equ	TW_BUSY_STATE	= 0xFF				;TWI unit is busy
.equ	TW_NO_STATE		= 0xF8				;No relevant state information
											; available; TWINT = "0"
.equ	TW_BUS_ERROR	= 0x00				;Bus error due to an illegal
											; START or STOP condition
.equ	TW_ARB_LOST		= 0x38				;Arbitration lost
;
;General Master staus codes
.equ	TW_START		= 0x08				;START has been transmitted
.equ	TW_REP_START	= 0x10				;Repeated START has been
											;transmitted
;Master Transmitter staus codes
.equ	TW_MTX_ADR_ACK	= 0x18				;SLA+W has been tramsmitted
											; and ACK received
.equ	TW_MTX_ADR_NACK	= 0x20				;SLA+W has been tramsmitted
											; and NACK received
.equ	TW_MTX_DATA_ACK	= 0x28				;Data byte has been tramsmitted
											; and ACK received
.equ	TW_MTX_DATA_NACK= 0x30				;Data byte has been tramsmitted
											; and NACK received
.equ	TW_MTX_ARB_LOST	= 0x38				;Arbitration lost in SLA+W or
											; data bytes
;Master Receiver staus codes
.equ	TW_MRX_ARB_LOST	= 0x38				;Arbitration lost in SLA+R or
											; NACK bit
.equ	TW_MRX_ADR_ACK	= 0x40				;SLA+R has been tramsmitted
											; and ACK received
.equ	TW_MRX_ADR_NACK	= 0x48				;SLA+R has been tramsmitted
											; and NACK received
.equ	TW_MRX_DATA_ACK	= 0x50				;Data byte has been received
											; and ACK tramsmitted
.equ	TW_MRX_DATA_NACK= 0x58				;Data byte has been received
											; and NACK tramsmitted
;
;************************************************************************
.include "Include/Macro_TWI.inc"
;****************************************************************************
  .endif
.endif
