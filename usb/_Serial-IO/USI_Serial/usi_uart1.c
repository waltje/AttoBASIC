/*****************************************************************************
 *
 * Copyright (C) 2003 Atmel Corporation
 *
 * File              : USI_UART.c
 * Compiler          : IAR EWAAVR 2.28a
 * Created           : 18.07.2002 by JLL
 * Modified          : 02-10-2003 by LTA
 *
 * Support mail      : avr@atmel.com
 *
 * Supported devices : ATtiny26
 *
 * Application Note  : AVR307 - Half duplex UART using the USI Interface
 *
 * Description       : Functions for USI_UART_receiver and USI_UART_transmitter.
 *                     Uses Pin Change Interrupt to detect incomming signals.
 *
 *
 ****************************************************************************/

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usi_uart_config.h"

#ifndef F_CPU
  #define F_CPU 8000000
//  #define F_CPU 16000000
#endif

#define PRESCL0X                0;              // Timer0 stop.
#define PRESCL1X                1;              // Timer0 1x prescaler.
#define PRESCL8X                2;              // Timer0 8x prescaler.
#define PRESCL64X               3;              // Timer0 64x prescaler.

#define TOPCNT1X                200UL
#define TOPCNT8X                (TOPCNT1X * 8)

#ifndef TIMER_PRESCALER
  #define TIMER_PRESCALER       1UL
  #define PRSCLR                PRESCL1X
#endif

// Check for initial counter overflow.  Change pre-scaler if overflow
#if ( (F_CPU / BAUDRATE) / TIMER_PRESCALER ) > TOPCNT8X
  #undef TIMER_PRESCALER
  #define TIMER_PRESCALER       64UL
  #undef PRSCLR
  #define PRSCLR                PRESCL64X
#endif
// Check for initial counter overflow.  Change pre-scaler if overflow
#if ( (F_CPU / BAUDRATE) / TIMER_PRESCALER ) > TOPCNT1X
  #undef TIMER_PRESCALER
  #define TIMER_PRESCALER       8UL
  #undef PRSCLR
  #define PRSCLR                 PRESCL8X
#endif

//  #undef TIMER_PRESCALER
//  #define TIMER_PRESCALER       8

//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER)
#define TIMER0_SEED               (256 - ( (F_CPU / BAUDRATE) / TIMER_PRESCALER ))

#if ( ( ( (F_CPU / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
    #define INITIAL_TIMER0_SEED       ( 256 - ( ( F_CPU / ( BAUDRATE / TIMER_PRESCALER ) ) * 1/2) )
    #define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
    #define INITIAL_TIMER0_SEED       ( 256 - ( ( F_CPU / ( BAUDRATE / TIMER_PRESCALER ) ) * 3/2) )
    #define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
    #error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0


//********** Static Variables **********//
unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];           // UART buffers. Size is definable in the header file.
volatile unsigned char UART_RxHead;
volatile unsigned char UART_RxTail;
unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
volatile unsigned char UART_TxHead;
volatile unsigned char UART_TxTail;

//register static unsigned char UART_TxData asm("r2");   // Tells the compiler to store the byte to be transmitted in registry.
//static unsigned char UART_TxData;   // Tells the compiler to store the byte to be transmitted in registry.
static volatile uint8_t UART_TxData;                                // Tells the compiler to use Register 15 instead of SRAM

volatile union UART_status                           // Status byte holding flags.
{
	unsigned char status;
	struct
	{
		unsigned char ongoing_Transmission_From_Buffer : 1;
		unsigned char ongoing_Transmission_Of_Package : 1;
		unsigned char ongoing_Reception_Of_Package : 1;
		unsigned char reception_Buffer_Overflow : 1;
		unsigned char flag4 : 1;
		unsigned char flag5 : 1;
		unsigned char flag6 : 1;
		unsigned char flag7 : 1;
	};
} UART_status = {0};

//********** USI_UART functions **********//
// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x )
{
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}

// Flush the UART buffers.
void USI_UART_Flush_Buffers( void )
{
	UART_RxTail = 0;
	UART_RxHead = 0;
	UART_TxTail = 0;
	UART_TxHead = 0;
}

// Close the hardware associated with the USI
void USI_UART_Close( void )
{
	USI_UART_Flush_Buffers();
	TCNT0  = 0x00;                                  //clear TCNT register
	TCCR0A  = 0x00;                                         // "Normal mode" for Timer0
	TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                // stop Timer0.
	TIFR |= (1<<TOV0);
	TIMSK &= ~(0<<TOIE0);
	USICR = 0;
	USISR = 0;
	GIFR |= (1<<PCIF);
	GIMSK &= ~(1<<PCIE);
//	PCMSK &= ~((1<<PCINT1) | (1<<PCINT0));
	PCMSK &= ~((1<<PCINT0));
	DDRB &= ~((1<<PB1) | (1<<PB0));
	PORTB &= ~((1<<PB1) | (1<<PB0));
//	PORTB &= ~((1<<PB3) | (1<<PB2) | (1<<PB1) | (1<<PB0));
}

// Initialise USI for UART transmission.
void USI_UART_Initialise_Transmitter( void )
{
	cli();
	TIFR = (1<<TOV0);                               //clear  Timer0 OVF flag
	TIMSK = (1<<TOIE0);                             //Enable Timer0 OVF flag
	GTCCR  |= (1<<PSR0);                                    // Reset the prescaler
	TCNT0  = 0x00;                                  //clear TCNT register
	TCCR0A  = 0x00;                                 // "Normal mode" for Timer0
#if ( TIMER_PRESCALER == 1 )
	TCCR0B  = (0<<CS02)|(0<<CS01)|(1<<CS00);                // start Timer0 w/ 1x prescaler.
#elif ( TIMER_PRESCALER == 8 )
	TCCR0B  = (0<<CS02)|(1<<CS01)|(0<<CS00);                // start Timer0 w/ 8x prescaler
#elif ( TIMER_PRESCALER == 64 )
	TCCR0B  = (0<<CS02)|(1<<CS01)|(1<<CS00);                // start Timer0 w/ 64x prescaler
#endif
	USICR  = (0<<USISIE)|(1<<USIOIE)|                     // Enable USI Counter OVF interrupt.
	         (0<<USIWM1)|(1<<USIWM0)|                     // Select Three Wire mode.
	         (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|         // Select Timer0 OVER as USI Clock source.
	         (0<<USITC);

	USIDR  = 0xFF;                                        // Make sure MSB is '1' before enabling USI_DO.
	USISR  = 0xF0 |                                       // Clear all USI interrupt flags.
	         0x0F;                                        // Preload the USI counter to generate interrupt at first USI clock.
	DDRB  |= (1<<PB1);                                    // Configure USI_DO as output.

	UART_status.ongoing_Transmission_From_Buffer = TRUE;

	sei();
}

// Initialise USI for UART reception.
// Note that this function only enables pinchange interrupt on the USI Data Input pin.
// The USI is configured to read data within the pinchange interrupt.
void USI_UART_Initialise_Receiver( void )
{
//	PORTB |=   (1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);         // Enable pull up on USI DO, DI and SCK pins. (And PB3 because of pin change interrupt)
//	DDRB  &= ~((1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0));      // Set USI DI, DO and SCK pins as inputs.
	PORTB |=  (1<<PB1)|(1<<PB0);                     // Enable pull up on USI DO and DI pin.
	DDRB  &= ~(1<<PB1 | 1<<PB0);                             // Set USI DI and DO pins as inputs.
	USICR  =  0;                                        // Disable USI.
	GIFR  |=  (1<<PCIF);                                // Clear pin change interrupt flag.
	GIMSK |=  (1<<PCIE);                               // Enable pin change interrupt for PB3:0.
	PCMSK |=  (1<<PB0);
}

// Puts data in the transmission buffer, after reverseing the bits in the byte.
// Initiates the transmission rutines if not already started.
void USI_UART_Transmit_Byte( unsigned char data )
{
	unsigned char tmphead;

	tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;    // Calculate buffer index.
	while ( tmphead == UART_TxTail ) ;                      // Wait for free space in buffer.
	UART_TxBuf[tmphead] = Bit_Reverse(data);                // Reverse the order of the bits in the data byte and store data in buffer.
	UART_TxHead = tmphead;                                  // Store new index.

	if ( !UART_status.ongoing_Transmission_From_Buffer ) // Start transmission from buffer (if not already started).
	{
		while ( UART_status.ongoing_Reception_Of_Package ) ;  // Wait for USI to finsh reading incoming data.
		USI_UART_Initialise_Transmitter();
	}
}

// Returns a byte from the receive buffer. Waits if buffer is empty.
unsigned char USI_UART_Receive_Byte( void )
{
	unsigned char tmptail;

	while ( UART_RxHead == UART_RxTail ) ;            // Wait for incomming data
	tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK; // Calculate buffer index
	UART_RxTail = tmptail;                            // Store new index
	return Bit_Reverse(UART_RxBuf[tmptail]);          // Reverse the order of the bits in the data byte before it returns data from the buffer.
}

// Check if there is data in the receive buffer.
unsigned char USI_UART_Data_In_Receive_Buffer( void )
{
	return ( UART_RxHead != UART_RxTail );            // Return 0 (FALSE) if the receive buffer is empty.
}


// ********** Interrupt Handlers ********** //

// The pin change interrupt is used to detect USI_UART reseption.
// It is here the USI is configured to sample the UART signal.
ISR(PCINT0_vect)
{
	if (!( PINB & (1<<PB0) ))                                 // If the USI DI pin is low, then it is likely that it
	{                                                         //  was this pin that generated the pin change interrupt.
//		GTCCR  |= (1<<PSR0);                            // Reset the prescaler
		TIFR   = (1<<TOV0);                               // Clear Timer0 OVF interrupt flag.
		TIMSK |= (1<<TOIE0);                              // Enable Timer0 OVF interrupt.
		TCNT0  = INTERRUPT_STARTUP_DELAY + INITIAL_TIMER0_SEED; // Plant TIMER0 seed to match baudrate (incl interrupt start up time.).
		TCCR0A  = 0x00;                                 // "Normal mode" for Timer0
#if ( TIMER_PRESCALER == 1 )
		TCCR0B  = (0<<CS02)|(0<<CS01)|(1<<CS00);        // start Timer0 w/ 1x prescaler.
#elif ( TIMER_PRESCALER == 8 )
		TCCR0B  = (0<<CS02)|(1<<CS01)|(0<<CS00);        // start Timer0 w/ 8x prescaler
#elif ( TIMER_PRESCALER == 64 )
		TCCR0B  = (0<<CS02)|(1<<CS01)|(1<<CS00);        // start Timer0 w/ 64x prescaler
#endif
		USICR  = (0<<USISIE)|(1<<USIOIE)|                 // Enable USI Counter OVF interrupt.
		         (0<<USIWM1)|(1<<USIWM0)|                 // Select Three Wire mode.
		         (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|     // Select Timer0 OVER as USI Clock source.
		         (0<<USITC);
		// Note that enabling the USI will also disable the pin change interrupt.
		USISR  = 0xF0 |                                   // Clear all USI interrupt flags.
		         USI_COUNTER_SEED_RECEIVE;                // Preload the USI counter to generate interrupt.

		GIMSK &=  ~(1<<PCIE);                                    // Disable pin change interrupt for PB3:0.

		UART_status.ongoing_Reception_Of_Package = TRUE;
	}
}

// The USI Counter Overflow interrupt is used for moving data between memmory and the USI data register.
// The interrupt is used for both transmission and reception.
ISR(USI_OVF_vect)
{
	unsigned char tmphead,tmptail;

	// Check if we are running in Transmit mode.
	if( UART_status.ongoing_Transmission_From_Buffer )
	{
		// If ongoing transmission, then send second half of transmit data.
		if( UART_status.ongoing_Transmission_Of_Package )
		{
			UART_status.ongoing_Transmission_Of_Package = FALSE; // Clear on-going package transmission flag.

			USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);     // Load USI Counter seed and clear all USI flags.
			USIDR = (UART_TxData << 3) | 0x07;          // Reload the USIDR with the rest of the data and a stop-bit.
		}
		// Else start sendinbg more data or leave transmit mode.
		else
		{
			// If there is data in the transmit buffer, then send first half of data.
			if ( UART_TxHead != UART_TxTail )
			{
				UART_status.ongoing_Transmission_Of_Package = TRUE; // Set on-going package transmission flag.

				tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK; // Calculate buffer index.
				UART_TxTail = tmptail;                  // Store new index.
				UART_TxData = UART_TxBuf[tmptail];  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
				// The bit reversing is moved to the application section to save time within the interrupt.
				USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT); // Load USI Counter seed and clear all USI flags.
				USIDR  = (UART_TxData >> 2) | 0x80; // Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
				//  of bit of bit reversed data).
			}
			// Else enter receive mode.
			else
			{
				UART_status.ongoing_Transmission_From_Buffer = FALSE;

				TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00); // Stop Timer0.
//				PORTB |=   (1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0); // Enable pull up on USI DO, DI and SCK pins. (And PB3 because of pin change interrupt)
//				DDRB  &= ~((1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0)); // Set USI DI, DO and SCK pins as inputs.
				PORTB |=  (1<<PB1)|(1<<PB0); // Enable pull up on USI DO and DI pin.
				DDRB  &= ~(1<<PB1 | 1<<PB0);     // Set USI DI and DO pin as inputs.
				USICR  =  0;                            // Disable USI.
				GIFR   =  (1<<PCIF);                    // Clear pin change interrupt flag.
				GIMSK |=  (1<<PCIE);                   // Enable pin change interrupt for PB3:0.
			}
		}
	}

	// Else running in receive mode.
	else
	{
		UART_status.ongoing_Reception_Of_Package = FALSE;

		tmphead     = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK; // Calculate buffer index.

		if ( tmphead == UART_RxTail )                           // If buffer is full trash data and set buffer full flag.
		{
			UART_status.reception_Buffer_Overflow = TRUE; // Store status to take actions elsewhere in the application code
		}
		else                                                    // If there is space in the buffer then store the data.
		{
			UART_RxHead = tmphead;                          // Store new index.
			UART_RxBuf[tmphead] = USIDR;                    // Store received data in buffer. Note that the data must be bit reversed before used.
		}                                                       // The bit reversing is moved to the application section to save time within the interrupt.

		TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);         // Stop Timer0.
//		PORTB |=   (1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0); // Enable pull up on USI DO, DI and SCK pins. (And PB3 because of pin change interrupt)
//		DDRB  &= ~((1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0)); // Set USI DI, DO and SCK pins as inputs.
		PORTB |=  (1<<PB1)|(1<<PB0);             // Enable pull up on USI DO and DI pin.
		DDRB  &= ~(1<<PB1 | 1<<PB0);                     // Set USI DI and DO pin as inputs.
		USICR  =  0;                                    // Disable USI.
		GIFR   =  (1<<PCIF);                            // Clear pin change interrupt flag.
		GIMSK |=  (1<<PCIE);                           // Enable pin change interrupt for PB3:0.
	}

}

// Timer0 Overflow interrupt is used to trigger the sampling of signals on the USI ports.
ISR(TIMER0_OVF_vect)
{
	TCNT0 += TIMER0_SEED;               // Reload the timer,
	                                    // current count is added for timing correction.
}

