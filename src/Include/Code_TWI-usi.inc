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
.if TWI
  .ifdef USIDR
  .include "Include/Defs_TWI.inc"			;include TWI definitions
;**************************************************************************
; TWI_Master_Init: USI TWI single master initialization function
;**************************************************************************
TWI_Master_Init:
	SETB	TWI_PORT,TWI_SDA		;set SDA pin high
	SETB	TWI_PORT,TWI_SCL		;set SCL pin high

	SETB	TWI_PORT-1,TWI_SCL		;set SCL to output
	SETB	TWI_PORT-1,TWI_SDA		;set SDA to output

	STOREI	USIDR, 0xFF				;Preload data register with "released level" data.

	ldi		temp,(  (0<<USISIE) | \
					(0<<USIOIE) | \
					(1<<USIWM1) | \
					(0<<USIWM0) | \
					(1<<USICS1) | \
					(0<<USICS0) | \
					(1<<USICLK) | \
					(0<<USITC) ) 	;Disable IRQ's, USI to 2-wire mode, Software 
									;  stobe as counter clock source.
	STORE	USICR,temp

	ldi		temp,(  (1<<USISIF) | \
					(1<<USIOIF) | \
					(1<<USIPF) | \
					(1<<USIDC) | \
					(0<<USICNT0) )	;Clear flags and reset counter.
	STORE	USISR,temp
	ret
;
;**************************************************************************
; TWI_Get_State_Info: Use this function to get hold of the error message 
;	from the last transmission
;**************************************************************************
TWI_Get_State_Info:
	LOAD	temp,TWI_state
	ret								;return to caller
;

;**************************************************************************
; TWI_Master_Transfer: Data to be sent has to be placed into the USIDR 
;	prior to calling this function. Data read, will be return'ed from 
;	the function.
;	Enter with data in U
;	Returns with data in U
;**************************************************************************
TWI_Master_Transfer:
_0x001c:
	STORE	USIDR,U					;save data in USIDR
	
	ldi		temp,(  (0<<USISIE) | \
					(0<<USIOIE) | \
					(1<<USIWM1) | \
					(0<<USIWM0) | \
					(1<<USICS1) | \
					(0<<USICS0) | \
					(1<<USICLK) | \
					(1<<USITC) ) 	;Disable IRQ's, USI to 2-wire mode, Software 
									;  stobe as counter clock source, Toggle Clock Port.
;	STORE	USICR,temp
 	ldi	r20, 0x13	; 19
 	ldi	r21, 0x00	; 0
  ;		USICR = temp;                          // Generate positve SCL edge. MC: why the same data has inverted effect when applied two times to USICR?
	ldi	r22, 0x2B	; 43
	ldi	r18, 0x11	; 17
	ldi	r19, 0x00	; 0
_0x0028:
	movw	r24, r20

_0x002a:
	sbiw	r24, 0x01	; 1
	brne	_0x002a     	; 0x2a <USI_TWI_Master_Transfer+0xe>
	out	0x0d, r22	; 13
;	while( !(IS_SCL_HIGH) );
_0x0030:
	sbis	0x16, 2	; 22
	rjmp	_0x0030     	; 0x30 <USI_TWI_Master_Transfer+0x14>
	movw	r24, r18
_0x0036:
	sbiw	r24, 0x01	; 1
	brne	_0x0036    	; 0x36 <USI_TWI_Master_Transfer+0x1a>
;		USICR = temp;                          // Generate negative SCL edge.
	out	0x0d, r22	; 13
;	}while( !(USISR & (1<<USIOIF)) );        // Check for transfer complete.
	sbis	0x0e, 6	; 14
	rjmp	_0x0028	; 0x28 <USI_TWI_Master_Transfer+0xc>
	ldi	r24, 0x13	; 19
	ldi	r25, 0x00	; 0
_0x0044:
	sbiw	r24, 0x01	; 1
	brne	_0x0044      	; 0x44 <USI_TWI_Master_Transfer+0x28>
;	temp  = USIDR;                           // Read out data.
	in	r24, 0x0f	; 15
;	USIDR = 0xFF;                            // Release SDA.
	ldi	r25, 0xFF	; 255
	out	0x0f, r25	; 15
;	ENABLE_SDA_AS_OUTPUT;
	sbi	0x17, 0	; 23
	ret
;
;**************************************************************************
; TWI_Master_Stop: Assert a STOP condition on the bus
;**************************************************************************
TWI_Master_Stop:
_0x0052:
;	PULL_SDA_LOW;
	CLRB	TWI_PORT,TWI_SDA		;set SDA pin low
;	PULL_SCL_HIGH;
	SETB	TWI_PORT,TWI_SCL		;set SCL pin high
;	while( !(IS_SCL_HIGH) ); //MC: TODO: timeout? what can cause a missing SCL low? in stop condition it is possibly a slave stretching (pulling SCL low to ask for some time)?
_0x0056:
	sbis	0x16, 2	; 22
	rjmp	_0x0056 ; 0x56 <USI_TWI_Master_Stop+0x4>
	ldi	r24, 0x11	; 17
	ldi	r25, 0x00	; 0
_0x005e:
	sbiw	r24, 0x01	; 1
	brne	_0x005e      	; 0x5e <USI_TWI_Master_Stop+0xc>
;	_delay_loop_2( T4_TWI );
;	PULL_SDA_HIGH;
	sbi	0x18, 0	; 24
	ldi	r24, 0x13	; 19
	ldi	r25, 0x00	; 0
_0x0068:
	sbiw	r24, 0x01	; 1
	brne	_0x0068 ; 0x68 <USI_TWI_Master_Stop+0x16>
	ldi	r24, 0x01	; 1
	ret
;
;**************************************************************************
;TWI_Start_Transceiver_With_Data: Success or error code is returned. 
;	Error codes are defined in USI_TWI_Master.h
;**************************************************************************
TWI_Start_Transceiver_With_Data:
	push	r16
	push	r17
	push	r28
	push	r29
	movw	r28, r24
	mov		r17, r22
;	USI_TWI_state.errorState = 0;
;	USI_TWI_state.addressMode = TRUE;
	ldi		r24, 0x01	; 1
	sts	0x0060, r24
;	USI_TWI_state.errorState = USI_TWI_UE_DATA_COL;
;	if ( !(*msg & (1<<TWI_READ_BIT)) )                // The LSB in the address byte determines if is a masterRead or masterWrite operation.
	ld		r24, Y
	sbrc	r24, 0
	rjmp	_0x008e ; 0x8e <USI_TWI_Start_Transceiver_With_Data+0x1e>
;	USI_TWI_state.masterWriteDataMode = TRUE;
	ldi		r24, 0x03	; 3
	sts		0x0060, r24
;	Release SCL to ensure that (repeated) Start can be performed */
;	PULL_SCL_HIGH;
_0x008e:
	SETB	TWI_PORT,TWI_SCL		;set SCL pin high
	sbi		0x18, 2	; 24
;	while( !(IS_SCL_HIGH) );
_0x0090:
	sbis	0x16, 2	; 22
	rjmp	_0X0090		 ; 0x90 <USI_TWI_Start_Transceiver_With_Data+0x20>
	ldi		r24, 0x13	; 19
	ldi		r25, 0x00	; 0
_0x0098:
	sbiw	r24, 0x01	; 1
	brne	_0x0098  ; 0x98 <USI_TWI_Start_Transceiver_With_Data+0x28>
;	_delay_loop_2( T2_TWI );                         // Delay for T2TWI if TWI_STANDARD_MODE
;	Generate Start Condition  TODO: encapsulate start_condition() */
;	PULL_SDA_LOW;
	cbi	0x18, 0	; 24
	ldi	r24, 0x11	; 17
	ldi	r25, 0x00	; 0
_0x00a2:
	sbiw	r24, 0x01	; 1
	brne	_0x00a2      	; 0xa2 <USI_TWI_Start_Transceiver_With_Data+0x32>
;	_delay_loop_2( T4_TWI );
;	PULL_SCL_LOW;
	cbi		0x18, 2	; 24
;	PULL_SDA_HIGH;
	sbi		0x18, 0	; 24
;	*(msg++)  = USI_TWI_Master_Transfer( tempUSISR_8bit );
;	Prepare to generate ACK (or NACK in case of End Of Transmission) */
;	if( msgSize == 1)                            // If transmission of last byte was performed.
;	uSIDR = 0xFF;                              // Load NACK to confirm End Of Transmission.
	ldi		r16, 0xFF	; 255
	/*MC TODO: do/while sucks. Anyway, this loop is executed once for every byte that needs to be sent or received */
	/*Write address and Read/Write data */
		/* If masterWrite cycle (or inital address tranmission)*/
;		if (USI_TWI_state.addressMode || USI_TWI_state.masterWriteDataMode)
_0x00ac:
	lds		r24, 0x0060
	andi	r24, 0x03	; 3
	breq	_0x00e6     	; 0xe6 <USI_TWI_Start_Transceiver_With_Data+0x76>
	;Write a byte */
;			PULL_SCL_LOW;
	cbi		0x18, 2	; 24
;	USIDR     = *(msg++);                        // Setup data. // MC: put next byte on the wire?
	ld		r24, Y
	out		0x0f, r24	; 15
;	USI_TWI_Master_Transfer( tempUSISR_8bit );    // Send 8 bits on bus.
	ldi		r24, 0xF0	; 240
	rcall	_0x001c		; 0x1c <USI_TWI_Master_Transfer>

;	Clock and verify (N)ACK from slave */
;	DDR_USI  &= ~(1<<PIN_USI_SDA);                // Enable SDA as input.
	cbi		0x17, 0	; 23
;			if( USI_TWI_Master_Transfer( tempUSISR_1bit ) & (1<<TWI_NACK_BIT) )
	ldi		r24, 0xFE	; 254
	rcall	_0x001c    	; 0x1c <USI_TWI_Master_Transfer>
	lds		r25, 0x0060
	sbrs	r24, 0
	rjmp	_0x00de ; 0xde <USI_TWI_Start_Transceiver_With_Data+0x6e>
;	if ( USI_TWI_state.addressMode )
	sbrs	r25, 0
	rjmp	_0x00d4  ; 0xd4 <USI_TWI_Start_Transceiver_With_Data+0x64>
;	USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_ADDRESS;
	ldi		r24, 0x06	; 6
	rjmp	_0x00d6  ; 0xd6 <USI_TWI_Start_Transceiver_With_Data+0x66>
;	else
;	USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_DATA;
_0x00d4:
	ldi		r24, 0x05	; 5
_0x00d6:
	sts		0x0060, r24
;				return (FALSE);
	ldi		r24, 0x00	; 0
	rjmp	_0x0108  ; 0x108 <USI_TWI_Start_Transceiver_With_Data+0x98>
;			USI_TWI_state.addressMode = FALSE;            // Only perform address transmission once.
_0x00de:
	andi	r25, 0xFE	; 254
	sts		0x0060, r25
	rjmp	_0x00fc	; 0xfc <USI_TWI_Start_Transceiver_With_Data+0x8c>
;		/* Else masterRead cycle*/
;		/* Read a data byte */
;			ENABLE_SDA_AS_INPUT;
_0x00e6:
	cbi		0x17, 0	; 23
;			*(msg++)  = USI_TWI_Master_Transfer( tempUSISR_8bit );
	ldi		r24, 0xF0		; 240
	rcall	_0x001c    	; 0x1c <USI_TWI_Master_Transfer>
	st		Y, r24

;		/* Prepare to generate ACK (or NACK in case of End Of Transmission) */
;			if( msgSize == 1)                            // If transmission of last byte was performed.
	cpi		r17, 0x01	; 1
brne	_0x00f6  ; 0xf6 <USI_TWI_Start_Transceiver_With_Data+0x86>
;				USIDR = 0xFF;                              // Load NACK to confirm End Of Transmission.
	out		0x0f, r16	; 15
	rjmp	_0x00f8  ; 0xf8 <USI_TWI_Start_Transceiver_With_Data+0x88>
;			else
;				USIDR = 0x00;                              // Load ACK. Set data register bit 7 (output for SDA) low.
_0x00f6:
	out		0x0f, r1	; 15
;			USI_TWI_Master_Transfer( tempUSISR_1bit );   // Generate ACK/NACK.
_0x00f8:
	ldi		r24, 0xFE	; 254
	rcall	_0x001c		; 0x1c <USI_TWI_Master_Transfer>
;	while( --msgSize) ;                             // Until all data sent/received.
_0X00fc:
	subi	r17, 0x01	; 1
	adiw	r28, 0x01	; 1
	and		r17, r17
	brne	_0x00ac     	; 0xac <USI_TWI_Start_Transceiver_With_Data+0x3c>

;	USI_TWI_Master_Stop();                           // Send a STOP condition on the TWI bus.
 	rcall	_0x0052    	; 0x52 <USI_TWI_Master_Stop>

	/* Transmission successfully completed*/
	ldi	r24, 0x01	; 1
_0x0108:
	pop	r29
	pop	r28
	pop	r17
	pop	r16
 	ret
;
  .endif
.endif
