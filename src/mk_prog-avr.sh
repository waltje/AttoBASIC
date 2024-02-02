# mk_prog-duino.sh - allows one to upload AttoBASIC to an ARDUINO compatible
#  platform using the bootloader.
#  Support added for DFU bootloaders on USB capable parts.
#  Also allows one to upload any HEX file.
#  File size is checked to insure that the file contents do not overwrite
#  the bootloader.  If so, the file is truncated then uploaded.
#
#  Modifications:
#	2016-0114	Added support for MINIPRO TL-866(A/CS)
#
#! /bin/bash

#set some defaults
DEBUG=0					# some debugging assistance

# how to request fuse data, "h"ex or "b"inary
FUSE_DISP=b
if [ ${FUSE_DISP} == h ] ; then
  FUSE_GREP="0x"
  elif [ ${FUSE_DISP} == b ] ; then
  FUSE_GREP="0b"
fi
FUSE_SUG=$PWD/fuses.txt				# name of the suggested fuses file
FUSE_SUG_LN=3					# 3 lines of fuses, lfuse, hfuse and efuse

PGMR_OPTS=""					#if avrdude needs additional options

MENUNAME=menu.lst
HEXDIR=AVR_Specific_Builds		#base location of HEX files
BASEDIR=${PWD}				#base directory of AttoBASIC project
MCUTYPE=(`ls -1v ${HEXDIR}`)		#create a list of available MCU types
#DFUSUPPRESS="--suppress-validation"	#if validation errors with dfu-programmer

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
###### Function definitions ###############################
#
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

# Wait for a USB device to come back on-line.  This is
#  necessary for the AVR DRAGON.
# Call with $1=USB_ID (mmmm:pppp)
function usb_device_wait() {
  N=20		# loop for 5 second timeout
  echo -n "${BLUE}Wait for USB device recovery " ;
  while [ "$(lsusb | egrep "$USB_ID")" == "" ] ; do
    sleep 0.25		# delay 1/4 second
    echo -n "."
    (( N-- ))	# decrement loop counter
    if [ $N == 0 ] ; then
      echo "${RED}The programmer stopped responding.${NML}"
      break
    fi
  done
  sleep 0.25		# delay 1/4 second
  echo ${NML}
}

# Fetch the proper mcu device name froo avrdude
# Call with $1=MCU long name
function get_avrdude_pn() {
  ${AVRDUDE} -p ? 2>&1 | egrep -m 1 "$1" | awk '{print $1}'
}

# Fetch the flash size for the selected part
# Call with $1=avr-gcc part name
function get_flash_size() {
  ROM_SIZE=$( cat ${HDR_PATH}/$1 | egrep "#define FLASHEND" | awk '{print $3}' | sed 's/0x//' )
  echo $(( $( hexToDec ${ROM_SIZE} ) + 1 ))
}

# Convert hexadecimal to decimal
function hexToDec() {
  in=${1^^}
  echo "ibase=16;$in" | bc
}

# Convert hexadecimal to binary
function hexToBin() {
  in=${1^^} ;
  echo "ibase=16;obase=2;$in" | BC_LINE_LENGTH=0 bc
}
# Convert binary to hexadecimal
function binToHex() {
  echo "ibase=2;obase=10000;$1" | bc
}

# Convert binary to decimal
function binToDec() {
  echo "ibase=2;$1" | bc
}

# Convert decimal to hexadecimal
function decToHex() {
  echo "obase=16; $1" | bc
}

# Convert decimal to binary
function decToBin() {
  echo "obase=2;$1" | bc
}

# Read fuses and print them
# call with $1=${PN}, $2=${PGMR}, $3=${DEVN}
function read_fuse {
  if [ ${#3} -gt 1 ] ; then DEVICE="-P $3" ; fi
  #  if [[ $2 == *dragon* ]] ; then echo ; echo "Wait for USB device recovery..." ; sleep 2 ; fi  # we need to let the AVR Dragon recover in between USB accesses

  # we need to let the AVR Dragon recover in between USB accesses
  if [[ $USB_ID != *NONE* ]] ; then
    echo ;
    usb_device_wait ${USB_ID}
  fi
  echo

  if [ $DEBUG == 1 ] ; then
    echo "Using: sudo $(basename ${AVRDUDE}) -u -p ${1} -c ${2} ${DEVICE} -U lock:r:-:${FUSE_DISP} -U lfuse:r:-:${FUSE_DISP} -U hfuse:r:-:${FUSE_DISP} -U efuse:r:-:${FUSE_DISP}"
  fi

  REPLY="$(sudo ${AVRDUDE} -u -p ${1} -c ${2} ${DEVICE} -U lock:r:-:${FUSE_DISP} -U lfuse:r:-:${FUSE_DISP} -U hfuse:r:-:${FUSE_DISP} -U efuse:r:-:${FUSE_DISP} 2>/dev/null | egrep "${FUSE_EGREP}")"		# filter hexadecimal or binary display
  #echo "REPLY = ${#REPLY} ${REPLY}"

  if [ ${#REPLY} == 15 ] ; then		# catch inccorrect response from programmer
    echo "${RED}This programmer does not support reading or writing of fuse bits.${NML}"
    elif [ ${#REPLY} -ge 14 ] ; then
    N=1
    echo "${BLUE}$(echo "$1" | tr [:lower:] [:upper:]) Fuses${NML}"
    echo "-----------------------"
    for F_TYPE in  "lock " "lfuse" "hfuse" "efuse" ; do
      PRINT=$( echo $REPLY | awk -v n=$N '{print $n}' | tr 'a-z' 'A-Z' )  # all to upper case
      #echo "PRINT = ${#PRINT} ${PRINT}"
      if [ ${#PRINT} -gt 0 ] ; then
        if [ ${FUSE_DISP} == b ] ; then
          echo "$F_TYPE = 0x$( binToDec ${PRINT:2} | awk '{ printf "%02X", $0 }' ) 0b$( echo ${PRINT:2} | awk '{ printf "%08d", $0 }' )"		# display fuse in binary and hexadecimal
          elif [ ${FUSE_DISP} == h ] ; then
          echo "$F_TYPE = 0x$( echo ${PRINT:2} ) 0b$( hexToBin ${PRINT:2} | awk '{ printf "%08d", $0 }' )"		# display fuse in hexadecimal and binary
        fi
        (( N++ ))
      fi
    done
  fi
  echo
}

# print middle lines from a file
# Call with $1=start_line $2=count $3=file_name
print_middle() {
  local S=$1 C=$2;
  shift 2;
  sed -n "$S,$(($S + $C -1))p; $(($S + $C))q" "$@";
}

# Find the local avr-gcc IO header file location
#  The name of the hidden file to store the location in is
#  passed in $1.
find_avr_headers() {
  echo -n "Please wait, locating AVR headers ."
  for HDR_PATH in $(find /usr/lib /usr/local/lib -type d -type d -name "avr" 2> /dev/null | grep "avr/include/avr" ) ; do
    echo -n "."
    if [ -e "$HDR_PATH/io.h" ] ; then
      echo "HDR_PATH=$HDR_PATH" > $1
      export HDR_PATH=$HDR_PATH
      if [ ${DEBUG} == 1 ] ; then echo "Header path = $HDR_PATH" ; fi
      break
    fi
  done
  echo
}

# ----- End of functions -----------------------------------------------
#
#                        \\\\ ////
#                       \\  - -  //
#                           @ @
#                   ---oOOo-( )-oOOo---
#
#=== Main Code ==========================================================

# Check for dependancies
export SREC=`which srec_cat`			#find srec_cat in case we need it
if [ -z ${SREC} ] ; then
  echo
  echo "${RED}srec_cat executable not found!${NML}"
  echo
  exit
fi
#
export AVRDUDE=`which avrdude`			#find AVRDUDE
if [ -z ${AVRDUDE} ] ; then
  echo
  echo "${RED}avrdude executable not found!${NML}"
  echo
  exit
else
  #  BAUD=(230400 115200 57600 38400)	#AVRDude's available baud rates (as an array)
  BAUD=(115200 57600 38400 19200)	#AVRDude's available baud rates (as an array)
fi
#
export DFUPROG=`which dfu-programmer`		#find DFU-PROGRAMMER
if [ -z ${DFUPROG} ] ; then
  echo
  echo "${RED}dfu-programmer not found!${NML}"
  echo
  exit
fi
#
export BC=`which bc`		#find BC calculator
if [ -z ${BC} ] ; then
  echo
  echo "${RED}bc not found!${NML}"
  echo
  exit
fi
#
export USBRST=`which usbreset`		#find USBRESET calculator
if [ -z ${USBRST} ] ; then
  echo
  echo "${RED}usbreset not found!${NML}"
  echo
  exit
fi

# Check for suggested fuses file
if [ ! -e ${FUSE_SUG} ] ; then
  echo
  echo "${RED}fuses.txt file not found!  Was it accidently deleted?${NML}"
  echo
  exit
fi

# =============================================================================
# Find the avr-gcc headers
# =============================================================================
# find the location of the part descriptor files
HDR_LASTPATH=.avr_hdrloc.txt
HDR_PATH=avr/include/avr			# partial location of part definition files
if [ -e ${HDR_LASTPATH} ] ; then
  source ${HDR_LASTPATH}
  if [ ! -e "$HDR_PATH/io.h" ] ; then
    echo "1 - Invalid header location determined."
    find_avr_headers ${HDR_LASTPATH}
  fi
else
  find_avr_headers ${HDR_LASTPATH}
fi

# =============================================================================
# Locate and erase any temporary files left in the HEX file directory
# =============================================================================
find ${HEXDIR} -type f -name "_*.hex" -exec rm -f {} \;

# =============================================================================
# Always select the MCU type
# =============================================================================
echo
echo "${MAG}Select the MCU:${YEL}"
select PN in ${MCUTYPE[@]} Quit ; do
  if [ "${PN}" == "Quit" ] ; then
    echo ${NML}
    exit ;
  else
    echo ${NML}
    AVRDUDE_PN=$PN		# save the part name for avrdude
    break ;
  fi ;
done
#
# Move to the MCU's directory
if [ -e ${HEXDIR}/${PN} ] ; then
  pushd ${HEXDIR}/${PN} 2>&1> /dev/null;
else
  echo
  echo "${RED}This error should not occur!${NML}"
  echo
fi
#
# =============================================================================
# set the AVR part number for avrdude and FLASH size for the part.
# =============================================================================
case ${PN} in
  2313)
    PN=AT90S2313
    AVRDUDE_PN=$(get_avrdude_pn ${PN})	# fetch the part name for avrdude
    HEADER=io2313.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=0				#size of the bootloader in bytes (OptiBoot85)
    PGMR=dragon_isp			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  tn2313)
    PN=ATtiny2313
    AVRDUDE_PN=$(get_avrdude_pn ${PN})	# fetch the part name for avrdude
    HEADER=iotn2313.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=0				#size of the bootloader in bytes (OptiBoot85)
    PGMR=dragon_isp			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  tn84)
    PN=ATtiny84
    AVRDUDE_PN=$(get_avrdude_pn ${PN})	# fetch the part name for avrdude
    HEADER=iotn84.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    #BTLD_SIZE=480			#size of the bootloader in bytes (OptiBoot85)
    BTLD_SIZE=512			#size of the bootloader in bytes (OptiBoot85)
    PGMR=dragon_isp			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  tn85)
    PN=ATtiny85
    AVRDUDE_PN=$(get_avrdude_pn ${PN})	# fetch the part name for avrdude
    HEADER=iotn85.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    #BTLD_SIZE=480			#size of the bootloader in bytes (OptiBoot85)
    BTLD_SIZE=512			#size of the bootloader in bytes (OptiBoot85)
    PGMR=dragon_isp			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  m88pa)
    PN=ATmega88
    AVRDUDE_PN=$(get_avrdude_pn ${PN})	# fetch the part name for avrdude
    HEADER=iom88.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=0				#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y				#supported by AVRDUDE
  ;;
  m16)
    PN=ATmega16
    AVRDUDE_PN=m16			# save the part name for avrdude
    HEADER=iom16.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=1048			#size of the bootloader in bytes
    #PGMR=dfu				#which programming algorithm to use
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  m163)
    PN=ATmega163
    AVRDUDE_PN=m163			# save the part name for avrdude
    HEADER=iom163.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=512			#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    SUPPORT=Y				#supported by AVRDUDE
  ;;
  m168)
    PN=ATmega168
    AVRDUDE_PN=m168			# save the part name for avrdude
    HEADER=iom168.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=512			#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y				#supported by AVRDUDE
  ;;
  m328)
    PN=ATmega328P
    AVRDUDE_PN=m328p		# save the part name for avrdude
    HEADER=iom328p.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=512			#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y				#supported by AVRDUDE
  ;;
  m32)
    PN=ATmega32
    AVRDUDE_PN=m32			# save the part name for avrdude
    HEADER=iom32.h			#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=2048			#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  m644p)
    PN=ATmega644P
    AVRDUDE_PN=m644p		# save the part name for avrdude
    HEADER=iom644p.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=512			#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  m1284p)
    PN=ATmega1284P
    AVRDUDE_PN=m1284p		# save the part name for avrdude
    HEADER=iom1284p.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=512			#size of the bootloader in bytes
    PGMR=arduino			#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="Y"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  m32u4)
    PN=ATmega32U4
    AVRDUDE_PN=m32u4		# save the part name for avrdude
    HEADER=iom32u4.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=2048			#size of the bootloader in bytes
    #PGMR=dfu				#which programming algorithm to use
    PGMR=avr109				#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    SUPPORT=Y
  ;;
  usb1286)
    PN=AT90USB1286
    AVRDUDE_PN=usb1286		# save the part name for avrdude
    HEADER=iousb1286.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=2048			#size of the bootloader in bytes
    #PGMR=dfu				#which programming algorithm to use
    PGMR=avr109				#which programming algorithm to use
    PGMR_OPTS=""			#any additional avrdude options
    SUPPORT=Y
  ;;
  m2560)
    PN=ATmega2560
    AVRDUDE_PN=m2560		# save the part name for avrdude
    HEADER=iom2560.h		#name of the avr-gcc header file
    ROM_SIZE=$( get_flash_size ${HEADER} )	# Fetch the ROM size
    BTLD_SIZE=4096			#size of the bootloader in bytes
    PGMR=stk500v2			#which programming algorithm to use
    PGMR_OPTS=""			#stk500v2 bootloader does not support
    MP_ICSP="N"				#minipro TL-866 ICSP support?
    #  the "chip erase" command and will cause a failure
    SUPPORT=Y				#supported by AVRDUDE
  ;;
esac

# =============================================================================
# Prompt user for part, build and programmer
# =============================================================================
if [ ${SUPPORT} == N ] ; then	# error check in case programming the part is not supported
  echo "${RED}Sorry! ${PN} not supported in Serial Port bootloader mode!"
  echo "   ${PN} uses the ${PGMR} bootloader mode.${NORMAL}"
  echo "   ${YEL}Please download ATMEL's FLIP programmer.${NORMAL}"
  echo
  exit
else
  #select the file name
  echo
  echo "${MAG}Select build type:${YEL}"
  select FN in "${RED}$(echo ${PN} | tr 'a-z' 'A-Z') Fuses${YEL}" `ls -1v *.hex` Quit ; do
    if [ "${FN}" == "Quit" ] ; then
      echo ${NML}
      exit ;
    else
      echo ${NML}
      break ;
    fi ;
  done ;

  # ==================================================================================
  # Define the various types of programmers supported for the part number selected
  # ==================================================================================
  if [ ${PN} == AT90USB1286 ] || [ ${PN} == ATmega32U4 ] ; then
    PGMR_SEL="DFU AVR109 USBASP USBTINY DRAGON_ISP DRAGON_JTAG"
    elif [ ${PN} == ATmega2560 ] ; then
    PGMR_SEL="STK500V2 USBASP USBTINY DRAGON_ISP DRAGON_JTAG"
    elif [ ${PN} == AT90S2313 ] || [ ${PN} == ATtiny2313 ]  || [ ${PN} == 8515 ] ; then
    PGMR_SEL="USBASP USBTINY DRAGON_ISP DRAGON_PP MINIPRO-ICSP MINIPRO-ZIF"
    elif [ ${PN} == ATtiny84 ] || [ ${PN} == ATtiny85 ] ; then
    PGMR_SEL="ARDUINO USBASP USBTINY DRAGON_ISP DRAGON_HVSP MINIPRO-ZIF"
    elif [ ${PN} == ATmega88P ] || [ ${PN} == ATmega168 ] || [ ${PN} == ATmega328P ] ; then
    PGMR_SEL="ARDUINO USBASP USBTINY DRAGON_ISP DRAGON_PP MINIPRO-ICSP MINIPRO-ZIF"
    elif [ ${PN} == ATmega163 ] ; then
    PGMR_SEL="ARDUINO USBASP USBTINY DRAGON_ISP DRAGON_PP "
  elif [ ${PN} == ATmega16 ] || [ ${PN} == ATmega32 ] || \
  [ ${PN} == ATmega644P ] ||  [ ${PN} == ATmega1284P ] ; then
    PGMR_SEL="ARDUINO USBASP USBTINY DRAGON_ISP DRAGON_JTAG DRAGON_PP MINIPRO-ICSP MINIPRO-ZIF"
  else
    PGMR_SEL=""
    echo "${RED}Error!  Should not see this error!${NML}"	#error condition
    echo
    exit
  fi
  if [ ${#PGMR_SEL} -gt 0 ] ; then
    echo "${MAG}Select programmer protocol:${YEL}"	# multiple programming protocols, ask which one
    select PGMR in ${PGMR_SEL} Quit ; do
      if [ "${PGMR}" == "Quit" ] ; then
        echo ${NML}
        exit ;
      else
        echo ${NML}
        PGMR=`echo ${PGMR} | tr '[:upper:]' '[:lower:]'`
        if [[ ${PGMR} == *dragon* ]] ; then
          export USB_ID=03eb:2107
          elif [[ ${PGMR} == *usbtiny* ]] ; then
          export USB_ID=1781:0c9f
          elif [[ ${PGMR} == *usbasp* ]] ; then
          export USB_ID=16c0:05dc
          elif [[ ${PGMR} == *minipro* ]] ; then
          export USB_ID=04d8:e11c
        else
          export USB_ID="NONE"
        fi
        break ;
      fi ;
    done
  fi

  # ==================================================================================
  # look for the USB device to use; either ttyACMx, or ttyUSBx for ARDUINO, AVR-109
  #  or STK500V2 protocols
  # ==================================================================================
  if [ ${PGMR} == arduino ] || [ ${PGMR} == stk500v2 ] || [ ${PGMR} == avr109 ]; then
    #look for the USB device; either ttyACMx or ttyUSBx
    if [ -e ${MENUNAME} ] ; then rm -f menu.lst ; fi
    find /dev/ -maxdepth 1 -name "ttyACM*" -or -name "ttyUSB*" | \
    while read DEV ; do
      if [ ! -h $DEV ] ; then
        #echo ${YEL}${DEV}${WHT}$(udevinfo $DEV)${NML} >> ${MENUNAME} ;
        if [ `udevchk $DEV` == 1 ] ; then
          echo ${DEV}$(udevinfo $DEV) >> ${MENUNAME} ;
        fi ;
      fi ;
    done
    #
    if [ -e ${MENUNAME} ] ; then
      echo
      echo "${BLU}Select the device (VCP):${NML}"
      select DEVN in `cat ${MENUNAME}` ; do
        if [ "${DEVN}" == "Quit" ] ; then
          echo ${NML}
          exit ;
        else
          IDX=$(echo ${DEVN} | grep -b -o "-" | awk 'BEGIN {FS=":"};{print $1}')
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
  fi

  # =============================================================================
  # Are we programming fuses?
  # =============================================================================
  if [[ $(echo $FN | tr [:lower:] [:upper:] ) == *FUSES* ]] ;then

    # Linux Minipro does not support fuse operations for some AVR's
    if ( [ ${PGMR} == minipro-icsp ] || [ ${PGMR} == minipro-zif ] ) && \
    ( [ ${PN} == ATtiny84 ] || [ ${PN} == ATtiny85 ] ) ; then
      echo
      echo "${RED}Error: fuse programming for ${YEL}${PN}${RED} is not "
      echo "   supported in this version of minipro.${NML}"
      echo
      exit
    fi

    # Programming fuses so no file checks required
    FN=FUSES		# to let other routines know what we are doing
    IO_FILE=${HDR_PATH}/${HEADER}		# build the part definition file name
    if [ $DEBUG == 1 ] ; then echo "Debug: ${PN} defs: $IO_FILE" ; fi
    echo "${BLU}Fuse bit definitions for ${WHT}${PN} ${BLU}from $( basename ${IO_FILE} )."
    # find the starting and ending line numbers for LOW fuse description
    # and print the descriptions
    echo -n ${CYN}	# set the color
    L_FUSE_SLN=$( cat ${IO_FILE} | egrep -n "* Low Fuse Byte *" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    L_FUSE_ELN=$( cat ${IO_FILE} | egrep -n "LFUSE_DEFAULT" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    if [ ${#L_FUSE_ELN} -gt 0 ] ; then
      print_middle $L_FUSE_SLN $(( $L_FUSE_ELN - $L_FUSE_SLN )) ${IO_FILE}| sed 's/#define FUSE_//g' | sed 's/(unsigned char)~_BV/bit/g'
    fi
    echo

    # find the starting and ending line numbers for HIGH fuse description
    # and print the descriptions
    H_FUSE_SLN=$( cat ${IO_FILE} | egrep -n "* High Fuse Byte *" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    H_FUSE_ELN=$( cat ${IO_FILE} | egrep -n "HFUSE_DEFAULT" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    if [ ${#H_FUSE_ELN} -gt 0 ] ; then
      print_middle $H_FUSE_SLN $(( $H_FUSE_ELN - $H_FUSE_SLN )) ${IO_FILE}| sed 's/#define FUSE_//g' | sed 's/(unsigned char)~_BV/bit/g'
    fi
    echo

    # find the starting and ending line numbers for EXTENDED fuse description
    # and print the descriptions
    E_FUSE_SLN=$( cat ${IO_FILE} | egrep -n "* Extended Fuse Byte *" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    E_FUSE_ELN=$( cat ${IO_FILE} | egrep -n "EFUSE_DEFAULT" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    if [ ${#E_FUSE_ELN} -gt 0 ] ; then
      print_middle $E_FUSE_SLN $(( $E_FUSE_ELN - $E_FUSE_SLN )) ${IO_FILE}| sed 's/#define FUSE_//g' | sed 's/(unsigned char)~_BV/bit/g'
    fi
    echo

    #    # find the starting and ending line numbers for LOCK fuse description
    #    # and print the descriptions
    #    K_FUSE_SLN=$( cat ${IO_FILE} | egrep -n "* Lock Bits *" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    #    K_FUSE_ELN=$( cat ${IO_FILE} | egrep -n "KFUSE_DEFAULT" | awk 'BEGIN {FS=":"}{print $1}' 2>&1 )
    #    if [ ${#K_FUSE_ELN} -gt 0 ] ; then
    #      print_middle $K_FUSE_SLN $(( $K_FUSE_ELN - $K_FUSE_SLN )) ${IO_FILE}| sed 's/#define __BOOT_LOCK_//g' | sed 's/(unsigned char)~_BV/bit/g'
    #    fi
    #    echo

    echo "${NML}Try ${WHT}http://www.engbedded.com/fusecalc ${NML}or${WHT}"
    echo "    http://eleccelerator.com/fusecalc/fusecalc.php ${NML}to determine fuses."
    echo

    FUSE_LN_START=$(cat ${FUSE_SUG} | egrep -nw "^${PN}" | awk -F ':' '{print $1}')	# find the line with the PN +1
    FUSE_LSUG=$( sed -n "$(( ${FUSE_LN_START} + 1 ))p" ${FUSE_SUG} | awk -F "0x" '{print $2}' 2>&1 )	# Lfuse
    FUSE_HSUG=$( sed -n "$(( ${FUSE_LN_START} + 2 ))p" ${FUSE_SUG} | awk -F "0x" '{print $2}' 2>&1 )	# Hfuse
    FUSE_ESUG=$( sed -n "$(( ${FUSE_LN_START} + 3 ))p" ${FUSE_SUG} | awk -F "0x" '{print $2}' 2>&1 )	# Efuse
    FUSE_KSUG=$( sed -n "$(( ${FUSE_LN_START} + 4 ))p" ${FUSE_SUG} | awk -F "0x" '{print $2}' 2>&1 )	# Efuse
    if [ $DEBUG == 1 ] ; then
      echo "FUSE_KSUG = $FUSE_KSUG"
      echo "FUSE_LSUG = $FUSE_LSUG"
      echo "FUSE_HSUG = $FUSE_HSUG"
      echo "FUSE_ESUG = $FUSE_ESUG"
    fi
    echo "${BLU}The sugested fuse setting for ${WHT}${PN}${BLU} are:"
    echo "  ${BLU}lock = ${YEL}0x${FUSE_KSUG}  ${BLU}lfuse = ${YEL}0x${FUSE_LSUG}  ${BLU}hfuse = ${YEL}0x${FUSE_HSUG}  ${BLU}efuse = ${YEL}0x${FUSE_ESUG}${NML}"
    echo "  ${CYN}Enter ${YEL}'FF'${CYN} for 'lock' alone to erase the lock bits.${NML}"
    read -p "${MAG}Enter fuse values [lock] [low] [high] [ext] in hex (Default [ENTER]: ${YEL}${FUSE_KSUG} ${FUSE_LSUG} ${FUSE_HSUG} ${FUSE_ESUG}) : ${WHT}" K_FUSE L_FUSE H_FUSE E_FUSE
    if [ ${#K_FUSE} == 0 ] ; then K_FUSE=${FUSE_KSUG} ; fi		# take default efuse
    if [ ${#L_FUSE} == 0 ] ; then L_FUSE=${FUSE_LSUG} ; fi		# take default lfuse
    if [ ${#H_FUSE} == 0 ] ; then H_FUSE=${FUSE_HSUG} ; fi		# take default hfuse
    if [ ${#E_FUSE} == 0 ] ; then E_FUSE=${FUSE_ESUG} ; fi		# take default efuse
    # add "0x" and convert to upper case
    K_FUSE="0x$(echo $K_FUSE | tr 'a-z' 'A-Z')"
    L_FUSE="0x$(echo $L_FUSE | tr 'a-z' 'A-Z')"
    H_FUSE="0x$(echo $H_FUSE | tr 'a-z' 'A-Z')"
    E_FUSE="0x$(echo $E_FUSE | tr 'a-z' 'A-Z')"
    if [ "${K_FUSE}" == "0xFF" ] ; then
      PGMR_OPTS="-e"		#chip erase needed to reset memory lock bits
    fi
    if [ $DEBUG == 1 ] ; then
      echo "L_FUSE = $L_FUSE"
      echo "H_FUSE = $H_FUSE"
      echo "E_FUSE = $E_FUSE"
      echo "K_FUSE = $K_FUSE"
    fi
  else

    # =============================================================================
    # Programming a HEX file so calculate the size of the file specified to be
    #  sure it fits
    # =============================================================================
    FN_SIZE=`srec_cat ${FN} -intel -o - -binary | wc -c`
    AVAIL_SIZE=`echo "${ROM_SIZE} - ${BTLD_SIZE}" | bc`
    if [ ${DEBUG} == 1 ] ; then
      echo "PARTNAME   : $PN"
      echo "ROM_SIZE   : $ROM_SIZE"
      echo "FN_SIZE    : $FN_SIZE"
      echo "AVAIL_SIZE : $AVAIL_SIZE"
      echo "PROGRAMMER : ${PGMR}"
      echo "FILE NAME  : ${FN}"
    fi

    # =============================================================================
    # Perform file size check.  Strip the bootloader if one exists.
    # =============================================================================
    if [ ${PGMR} == dragon_isp ] || \
    [ ${PGMR} == dragon_jtag ] || \
    [ ${PGMR} == dragon_hvsp ] || \
    [ ${PGMR} == minipro-icsp ] || \
    [ ${PGMR} == minipro-zif ] || \
    [ ${PGMR} == usbasp ] || \
    [ ${PGMR} == usbtiny ]; then
      echo
      echo "${YEL}ISP or JTAG programmer selected."
      echo "  The HEX image will not be checked for size.${NML}"
      echo
    else
      if [ ${FN_SIZE} -gt ${AVAIL_SIZE} ] ; then
        echo
        echo "${YEL}The HEX file specified will not fit in FLASH!"
        if [ "$PN" == "attiny85" ] ; then
          echo "  ${RED}Warning: Will not overwrite the bootloader on."
          echo "           a ${BLU}${PN}${RED}. Use an ISP programmer instead"
          echo
        fi
        echo " Truncating the file to ${AVAIL_SIZE} bytes.${NML}"
        echo
        TMPFILE=_${RANDOM}.hex ;		#create a temporary file
        # use srec_cat to truncate the file to the correct size
        ${SREC} ${FN} -intel -fill 0xFF 0x0000 0x`echo "obase=16; ${ROM_SIZE}" | bc` \
        -exclude 0x`echo "obase=16; ${AVAIL_SIZE}" | bc` 0x`echo "obase=16; ${ROM_SIZE}" | bc` \
        -o ${TMPFILE} -intel -obs=16
        #
        FN=${TMPFILE} ;
        echo "${WHT}New file is ${BLU}`srec_cat ${FN} -intel -o - -binary | wc -c`${WHT} bytes${NML}"
        echo
      fi
    fi
  fi

  # =============================================================================
  # test for a connection
  # =============================================================================
  if [ ${PGMR} == arduino ] ; then		#only "arduino" programmer gets a baud
    N=`echo "${#BAUD[@]} - 1" | bc`		#get index pointer to baud rate array
    echo "${BLU}Attempting to communicate with the ARDUINO on ${DEVN} ..."
    for BAUD_N in `seq 0 ${N}` ; do
      ERROR=1 ;		#set a flag
      #            echo "Using: avrdude -p ${PN} -c ${PGMR} -P ${DEVN} -b ${BAUD[${BAUD_N}]}"
      echo -n "${BLU}Testing at ${BAUD[${BAUD_N}]} baud ...${NML} "
      ${AVRDUDE} -u -p ${PN} -c ${PGMR} -P ${DEVN} -b ${BAUD[${BAUD_N}]} > /dev/null 2>&1
      if [ $? ==  0 ] ; then
        ERROR=0 ;		#set a flag
        echo "${YEL}Success!  Using baud rate of ${BAUD[$BAUD_N]}${NML}"
        break ;				#successful
      else
        echo "${RED}error communicating at ${BAUD[${BAUD_N}]} baud!${NML}"
      fi
    done
    if [ ${ERROR} -gt 0 ] ; then			#cannot communicate with ARDUINO so exit
      echo
      echo "${RED}There was an error communicating with the ARDUINO!${NML}"
      echo
      if [ -f ${TMPFILE} ] ; then rm -f ${TMPFILE} 2>&1>/dev/null ; fi	#delete the temp file if it exists
      exit
    fi
  fi
  #printf "%s\n" "${BAUD[@]}" ## print array
  #
  # =============================================================================
  # Determine the proper ISP bitrate
  # =============================================================================
  # if [ ${FN} == FUSES ] || [[ "${FN}" == *1MHZ* ]] ; then
  if [[ "${FN}" == *4MHZ* ]] ; then
    BITRATE="-B 500kHz"
    elif [[ "${FN}" == *8MHZ* ]] ; then
    BITRATE="-B 1MHz"
    elif [[ "${FN}" == *16MHZ* ]] || [[ "${FN}" == *20MHZ* ]] ; then
    BITRATE="-B 2MHz"
  else
    BITRATE="-B 200kHz"	# In case we are using a stock AVR @ 8MHz / div 8 clock
  fi

  # =============================================================================
  # Connection achieved, programm the device with the selected HEX file
  # =============================================================================
  echo
  echo "${BLU}Programming the AVR with ${FN} ...${NML}"
  # ++++++++++++++++++++++++++++++++++++++++++++++++++++
  # ARDUINO programmer
  # ++++++++++++++++++++++++++++++++++++++++++++++++++++
  if [ ${PGMR} == arduino ] ; then		#only "arduino" programmer gets a baud
    if [ $FN == "FUSES" ] ; then
      CMDLINE="${AVRDUDE} -u  -p ${PN} -c ${PGMR} -P ${DEVN} -b ${BAUD[${BAUD_N}]} -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m -U lock:w:${K_FUSE}:m"
    else
      CMDLINE="${AVRDUDE} -u  -p ${PN} -c ${PGMR} -P ${DEVN} -b ${BAUD[${BAUD_N}]} -U flash:w:${FN}"
    fi
    echo "Using: ${CYN}${CMDLINE}${NML}" ;
    echo
    ${CMDLINE} 2>&1 | egrep "Writing|Reading" ;
    echo -n ${BLU}
    read_fuse ${PN} ${PGMR} ${DEVN}
    echo -n ${NML}

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    # USBASP or USBTINY programmer
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    elif [ ${PGMR} == usbasp ] || [ ${PGMR} == usbtiny ] ; then
    DEVN=usb
    if [ $FN == "FUSES" ] ; then
      CMDLINE="sudo ${AVRDUDE} ${PGMR_OPTS} -u -p ${AVRDUDE_PN} -c ${PGMR} ${BITRATE} -P ${DEVN} -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m -U lock:w:${K_FUSE}:m"
    else
      CMDLINE="sudo ${AVRDUDE} ${PGMR_OPTS} -u -p ${AVRDUDE_PN} -c ${PGMR} ${BITRATE} -P ${DEVN} -U flash:w:${FN}"
    fi

    echo "Using: ${CYN}${CMDLINE} ${NML}" ;
    echo
    ${CMDLINE} 2>&1 | egrep "Writing|Reading" ;
    echo -n ${BLU}
    read_fuse ${PN} ${PGMR} ${DEVN}
    echo -n ${NML}

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    # STK500V2, avr-109
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    elif [ ${PGMR} == stk500v2 ] || [ ${PGMR} == avr109 ] ; then

    if [ $FN == "FUSES" ] ; then
      CMDLINE="${AVRDUDE} ${PGMR_OPTS} -u  -p ${AVRDUDE_PN} -c ${PGMR} -P ${DEVN} -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m -U lock:w:${K_FUSE}:m"
    else
      CMDLINE="${AVRDUDE} ${PGMR_OPTS} -u  -p ${AVRDUDE_PN} -c ${PGMR} -P ${DEVN} -U flash:w:${FN}"
    fi

    if [ ${PN} == ATmega2560 ] ; then
      PGMR_OPTS="-D"			#stk500v2 bootloader does not support CHIP ERASE command
    fi

    echo "Using: ${CYN}${CMDLINE} ${NML}" ;
    echo
    ${CMDLINE} 2>&1 | egrep "Writing|Reading" ;
    echo -n ${BLU}
    read_fuse ${PN} ${PGMR} ${DEVN}
    echo -n ${NML}

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    # DFU programmer
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    elif [ ${PGMR} == dfu ] ; then
    #echo "Using: ${CYN}${DFUPROG} ${PN} flash --suppress-validation ${FN} ${NML}" ;
    echo "Using: ${CYN}${DFUPROG} ${PN} flash ${FN} ${NML}" ;
    echo
    echo "  ${WHT}Erasing FLASH memory${NML}" ;
    sudo ${DFUPROG} ${PN} erase --force >/dev/null 2>&1
    if [ $? == 0 ] ; then
      echo "  ${WHT}Programming FLASH memory${NML}" ;
      sudo ${DFUPROG} ${PN} flash ${FN} ${DFUSUPPRESS} >/dev/null 2>&1
      if [ $? == 0 ] ; then sudo ${DFUPROG} ${PN} launch ; fi	#launch the user application
    else
      echo
      echo "${RED}There was failure to erase the ${PN}!${NML}"
      echo
    fi

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    # DRAGON_ISP, DRAGON_JTAG, DRAGON_PP or DRAGON_HVSP programmer
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    elif [ ${PGMR} == dragon_isp ] || [ ${PGMR} == dragon_jtag ] || [ ${PGMR} == dragon_hvsp ] || [ ${PGMR} == dragon_pp ] ; then
    if [ $FN == "FUSES" ] ; then
      CMDLINE="${AVRDUDE} ${PGMR_OPTS} -u -p ${AVRDUDE_PN} -c ${PGMR} ${BITRATE} -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m -U lock:w:${K_FUSE}:m"
    else
      CMDLINE="${AVRDUDE} ${PGMR_OPTS} -u -p ${AVRDUDE_PN} -c ${PGMR} ${BITRATE} -U flash:w:${FN}"
    fi

    echo "Using: ${CYN}${CMDLINE} ${NML}" ;
    echo
    sudo ${CMDLINE} 2>&1 | egrep "Writing|Reading" ;
    AVRDUDE_ERR=$?			# get the error code from AVRDUDE
    echo "${YEL}AvrDude returned ${AVRDUDE_ERR}.${NML}"
    if [ ${AVRDUDE_ERR} -gt 0 ] ; then
      echo
      echo "${RED}There was failure.  Err_Code = ${AVRDUDE_ERR} ${PN}!${NML}"
      echo
    fi

    echo -n ${BLU}
    read_fuse ${PN} ${PGMR} ${DEVN}
    echo -n ${NML}

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    # MiniPro TL-866(A/CS) programmer
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
    elif [ ${PGMR} == minipro-icsp ] || [ ${PGMR} == minipro-zif ] ; then
    #find minipro since we need it
    export MINIPRO=`which minipro`
    if [ -z ${MINIPRO} ] ; then
      echo
      echo "${RED}minipro executable not found!${NML}"
      echo
      exit
    fi
    #find miniprohex since we need it
    export MINIPROHEX=`which miniprohex`
    if [ -z ${MINIPROHEX} ] ; then
      echo
      echo "${RED}miniprohex executable not found!${NML}"
      echo
      exit
    fi

    # insure that the MINIPRO TL-866(A/CS) supports this part!
    if [ $(minipro -l | egrep -x --ignore-case "$PN") != $(echo $PN | tr [a-z] [A-Z]) ] ; then
      echo
      echo "${RED}Sorry, the TL-866(A/CS) does not support this part!" ;
      echo
      exit
    fi

    # What Minipro mode?
    if [ ${MP_ICSP} == "Y" ] && [ ${PGMR} == minipro-icsp ] ; then			# is ISP supported?
      echo "${YEL}Using MiniPro TL-866(A/CS) ${RED}ICSP${YEL} mode.${NML}"
      MP_OPTS="-i" ;
    else
      echo "${YEL}Using MiniPro TL-866(A/CS) ${RED}ZIF${YEL} mode.${NML}"
      MP_OPTS="-S"
    fi

    # create the fuse file for the minipro executable
    FUSEFILE=$(tempfile)

    # process fuses for the Minipro
    if [ $FN == "FUSES" ] ; then
      echo "fuses_lo = $L_FUSE" > ${FUSEFILE}
      echo "fuses_hi = $H_FUSE" >> ${FUSEFILE}
      echo "fuses_ext = $E_FUSE" >> ${FUSEFILE}
      echo "lock_byte = $K_FUSE" >> ${FUSEFILE}
      MP_OPTS="${MP_OPTS} -c config" ;
      CMDLINE="${MINIPRO} ${MP_OPTS} -p ${PN} -w ${FUSEFILE}"
    else
      MP_OPTS="${MP_OPTS} -c code" ;
      CMDLINE="${MINIPROHEX} ${MP_OPTS} -p ${PN} -w ${FN}"
    fi

    # execute the programming
    echo "Using: ${CYN}${CMDLINE} ${NML}" ;
    echo
    ${CMDLINE} ;			# execute the programming
    AVRDUDE_ERR=$?			# get the error code from AVRDUDE
    echo "${YEL}minipro returned ${AVRDUDE_ERR}.${NML}"
    if [ ${AVRDUDE_ERR} -gt 0 ] ; then
      echo
      echo "${RED}There was failure.  Err_Code = ${AVRDUDE_ERR} ${PN}!${NML}"
      echo
    fi

    # read back the fuses
    if [ ${PN} != ATtiny84 ] && [ ${PN} != ATtiny85 ] ; then	# no fuse readback for some AVR's
      echo ${BLU}
      CMDLINE="${MINIPRO} -c config -p ${PN} -r ${FUSEFILE}"
      ${CMDLINE}
      echo -n ${CYN}; cat ${FUSEFILE} | sed 's/fuses/  fuses/g' | sed 's/lock/  lock/g' ; echo ${NML}
    fi
    rm ${FUSEFILE}
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++
  fi
fi
