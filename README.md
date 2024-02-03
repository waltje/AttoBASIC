Welcome to *AttoBASIC*, a BASIC interpreter for AVR-based microcontrollers!

# Overview
*AttoBASIC* is a very small Basic interpreter for a very small chip.  The interpreter uses on-chip RAM only.  Originally targeted and developed for limited debugging, monitor and control use on an ATMEL AT90S2313, it has expanded with the ATMEL line of AVR8 microcontrollers. 

Most AVR-based boards from and for the Arduino, Adafruit, Olimex and Teensy(2.0) series (and their clones) are supported.

The current version (**2.34**) supports the following micro-controllers:

- ATtiny84/85
- ATmega16/32
- ATmega88/168/328
- ATmega32U4
- ATmega644P/1284P
- ATmega2560
- AT90USB128

Older versions support AT90S2313, ATtiny2313(A), AT90S8515 & ATmega163 and are now included in the HEX file builds as is the full source code.

# Author
Copyright 2001-2016 Richard Cappels, <projects@appels.org>
Copyright 2012-2023 Scott Vitale, <ksv_prj@gmx.com>

# Updates
Version 2.0 and above incorporates many enhancements by Scott Vitale; including TWI (I2C®), SPI, Dallas 1-wire® and DS serial interfaces, a high resolution Direct Digital Synthesizer (DDS) function, a real-time counter, data file management, Nordic nRF24L01(+), DHTxx Humidity and Temperature Sensor and support for UART and USB serial I/O.  Support for most Arduino, Adafruit, Olimex and many other products running a supported MCU is possible.  Various bootloaders are included with AttoBASIC.
