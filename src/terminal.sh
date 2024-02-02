# Invokes picocom using a selectable menu of possible USB
# character devices along with their USB descriptions
#
#! /bin/bash

# ANSI COLORS
CRE="$(echo -e '\r\033[K')"
NML="$(echo -e '\033[0;39m')"
RED="$(echo -e '\033[1;31m')"
GRN="$(echo -e '\033[1;32m')"
YEL="$(echo -e '\033[1;33m')"
BLU="$(echo -e '\033[1;34m')"
MAG="$(echo -e '\033[1;35m')"
CYN="$(echo -e '\033[1;36m')"
WHT="$(echo -e '\033[1;37m')"
#
#set some defaults
MENUNAME=menu.lst
#BAUD=19200
BAUD=38400
#SENDCMD=(--send-cmd \"ascii-xfr -v -s -l 50 -c 15\")
SENDCMD=(--send-cmd \"ascii-xfr -v -s -l 20 -c 2\")
RECVCMD=(--receive-cmd \"ascii-xfr -v -r -d\")
PORT=/dev/ttyUSB0

# Use UDEV to gather identifying info about each USB device of interest
udevinfo () {
   local UDEVINFO=`udevadm info -a -p $(udevadm info -q path -n $1) |  \
   egrep "ATTRS{product}==" | \
   head -n 1  | \
   sed 's/ATTRS{product}==//g' | \
   sed 's/"//g' | \
   sed 's/ /_/g' | \
   sed 's/____/-/g'`
   echo ${UDEVINFO} ;
}

# Use UDEV to check the USB device of interest.
# Returns "1" if good device, "0" otherwise
udevchk () {
   udevadm info -q path -n $1 2>&1 |  \
   egrep "device node not found" > /dev/null
   if [ $? == 0 ] ; then
      echo "0" ;
   else
      echo "1" ;
   fi
}

strindex() {
   x="${1%%$2*}"
   [[ $x = $1 ]] && echo -1 || echo ${#x}
}

#look for the USB device; either ttyACMx or ttyUSBx
if [ -e ${MENUNAME} ] ; then rm -f menu.lst ; fi
find /dev/ -maxdepth 1 -type c -name "ttyACM*" -or -name "ttyUSB*" | while read DEV ; do
   if [ ! -h $DEV ] ; then
      #echo ${YEL}${DEV}${WHT}$(udevinfo $DEV)${NML} >> ${MENUNAME} ;
      if [ `udevchk $DEV` == 1 ] ; then
         echo ${DEV}$(udevinfo $DEV) >> ${MENUNAME} ;
      fi ;
   fi ;
done

if [ -e ${MENUNAME} ] ; then
   echo
   echo "${BLU}Select the device (VCP):${NML}"
   echo "${YEL}Quit${NML}" >> menu.lst
   select DEVN in `cat ${MENUNAME}` ; do
      if [ "${DEVN}" == "Quit" ] ; then
         echo ${NML}
         exit ;
      else
         IDX=`echo ${DEVN} | grep -m 1 -b -o "-" | awk 'BEGIN {FS=":"}{print $1}'`
         DEVN=${DEVN:0:${IDX:0:2}} ;
         echo ${NML}
         break ;
      fi ;
   done
   if [ -e ${MENUNAME} ] ; then rm -f menu.lst ; fi
else
   echo
   echo "${RED}Error: Communications port not found!${NML}"
   echo
   exit
fi


#picocom -b ${BAUD} ${DEVN} --send-cmd "ascii-xfr -s -l 50 -c 15" --receive-cmd "ascii-xfr -v -r -d"
picocom -b ${BAUD} ${DEVN} --send-cmd "ascii-xfr -s -l 30 -c 3" --receive-cmd "ascii-xfr -v -r -d"
