#!/bin/bash

. colordefs.sh				# terminal color definitions

if [ ! -e images ] ; then mkdir images ; fi
#
for MCU in m32 m328 m644 m1284 m2560 usb1286 ; do
  for CLK in 4 8 10 16 20 ; do
    echo "  ${YEL}Processing Monitor for ${MCU} @${CLK}MHz ...${NORMAL}"
    make clean > /dev/null 2>&1 ; make ${MCU}-${CLK} > /dev/null 2>&1
  done
done

make clean > /dev/null 2>&1
