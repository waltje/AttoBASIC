#! /bin/bash

if [ $# == 0 ] ; then
   echo
   echo "Usage: `basename $)` [FileName] [size in bytes]"
   echo
else
   SRC=$1
   SIZE=$2
   DST=${SRC%.*}_tnunc.hex

   echo "Processing ${SRC}.  Truncating to ${SIZE} bytes."
   srec_cat ${SRC} -intel -o - -binary | \
   dd bs=1 count=${SIZE} 2>/dev/null  | \
   srec_cat - -binary -o ${DST} -intel
fi
