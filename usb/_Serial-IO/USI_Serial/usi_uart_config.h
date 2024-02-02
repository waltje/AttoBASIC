/*****************************************************************************
 *
 * Copyright (C) 2003 Atmel Corporation
 *
 * File          : USI_UART_config.h
 * Compiler      : IAR EWAAVR 2.28a
 * Created       : 18.07.2002 by JLL
 * Modified      : 02-10-2003 by LTA
 *
 * Support mail  : avr@atmel.com
 *
 * AppNote       : AVR307 - Half duplex UART using the USI Interface
 *
 * Description   : Header file for USI_UART driver
 *
 *
 ****************************************************************************/

#ifndef BAUDRATE
//  #define BAUDRATE                    38400
#endif

//********** USI UART Defines **********//
#define UART_RX_BUFFER_SIZE        4     /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART_TX_BUFFER_SIZE        4

//********** USI_UART Prototypes **********//
unsigned char Bit_Reverse( unsigned char );
void USI_UART_Flush_Buffers( void );
void USI_UART_Initialise_Receiver( void );
void USI_UART_Initialise_Transmitter( void );
void USI_UART_Close( void );
void USI_UART_PutChar( unsigned char );
unsigned char USI_UART_GetChar( void );
unsigned char USI_UART_Data_In_Receive_Buffer( void );

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01
#define CPU_4MHz        0x02
#define CPU_2MHz        0x03
#define CPU_1MHz        0x04
#define CPU_500kHz      0x05
#define CPU_250kHz      0x06
#define CPU_125kHz      0x07
#define CPU_62kHz       0x08
