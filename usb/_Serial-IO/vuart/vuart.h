/**
 * vuart.h is a header for full-duplex software UART
 *
 * Copyright (C) 2010 Masood Behabadi <masoodbeh@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VUART_H_
#define VUART_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/*
 * Controls whether code for transmitting data be compiled
 */
#define VUART_TRANSMIT_SUPPORT 1
/*
 * Controls whether code for receiving data be compiled
 */
#define VUART_RECEIVE_SUPPORT 1

/*
 * Sets baud rate for UART
 */
#define VUART_BAUDRATE 38400
/*
 * Sets the size of data in each UART frame
 */
#define VUART_CHAR_SIZE 8
/*
 * Sets number of stop bits
 */
#define VUART_STOP_BITS 1

/*
 * Bit positions definition in state variable
 */
#define VUART_DATA_EMPTY 0
#define VUART_RECEIVE_COMPLETE 1
#define VUART_TRANSMIT_COMPLETE 2

/*
 * A non-zero reading means new data is received and ready to be read
 */
#define vuart_is_receive_complete() vuart_state & (1 << VUART_RECEIVE_COMPLETE)
/*
 * A non-zero reading means previous data in transmit buffer has completely shifted out
 */
#define vuart_is_transmit_complete() vuart_state & (1 << VUART_TRANSMIT_COMPLETE)
/*
 * A non-zero reading means transmit buffer is empty and new data can be written to it
 */
#define vuart_is_data_empty() vuart_state & (1 << VUART_DATA_EMPTY)

/*
 * Holds current receive and transmit state
 */
volatile uint8_t vuart_state;

#if VUART_RECEIVE_SUPPORT

uint8_t vuart_read_data();

#endif

#if VUART_TRANSMIT_SUPPORT

void vuart_write_data(uint8_t data);

#endif

void vuart_init();

#endif /* VUART_H_ */
