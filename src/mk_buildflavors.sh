# This script automates the build process for AttoBASIC.
# Specifically, it functions in three (3) parts; 1) Update (or add)
#  headers to each source code file, 2) Build all support modules,
#  which includes bootloaders, USB and Serial I/O drivers. 3) builds
#  HEX files for all MCU, clock speed and bootloader configurations
#  for all supported flavors.
#
# Version: 1.1 - 04/26/2016; Added source code header processing.
# Version: 1.0 - Writen ??/??/??.  Work in progress.
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

NOBUILD=0					#"1" to bypass builds
DEBUG=0						#"1" to enable debug output
MKLST=N						#"Y" to copy LST files
#

SRC_LIST="AttoBASIC*.asm Include/Code_*.inc Include/D[ae][tf][as]*.inc"
PARTS="2313 tn2313 tn84 tn85 m88 8515 m16 m163 m168 m32 m328 m32u4 m644p m1284p usb1286 m2560"	# MCU's to build for
DEFTMPL=Include/Defs_Constants.tmpl		#definition template file
#DEFS="Include/Defs_Constants.inc"		#target definition file
DEFS="Include/Defs_MCU-Options.inc"		#target definition file
FLAVDIR=AVR_Specific_Builds				#flavors build directory
BLDRDIR="_Bootloaders"					#location of USART bootloaders
USBSIODIR="_Serial-IO/USB_Serial"		#location of USB Serial I/O library
USISIODIR="_Serial-IO/USI_Serial"		#location of USI Serial I/O library
SWUARTSIODIR="_Serial-IO/vuart"			#location of Soft-UART Serial I/O library
TNSIODIR="_Serial-IO/ATtiny"			#location of ATtiny Serial I/O library
MINDEFSSIZE=1500						#Defs_Constants.inc must be at least this size
ERRFN=`tempfile`						#place to store erroneous builds

logerror() {	# function to display error message and log it to a file
  echo "${RED}Error producing HEX file for ${WHT}${TARGET}${RED} @ ${WHT}${MCLK}MHZ${RED} with ${YEL}${BUILD}${RED} option ...${NORMAL}" | \
  tee -a ${ERRFN} ;
}

howmanyfiles() {			#returns the number of HEX files built within the directory passed to it
  find $1 -type f -name "*.hex" | wc -l
}

# Start a timer
START=$(date +%s.%N)

#create the file name prefix and revision level
export ROOTDIR=$PWD					# root directory of this build
pushd ../../ > /dev/null ;			# up two levels
FNBASE=$(basename $PWD | sed 's/-//g' |  tr '[:lower:]' '[:upper:]')
popd > /dev/null
export REV=$(basename $PWD |  tr '[:lower:]' '[:upper:]') ;
FNPFX=$(echo ${FNBASE}${REV} | tr '[:lower:]' '[:upper:]') ;

#-----------------------------------------------------------------------------------
# Ask to update source code files or not
#  Then update all source code header files to reflect version
#  and current copyright year.
#-----------------------------------------------------------------------------------
echo
read -t 15 -N 1 -p "${YEL}Update source code file headers? (y/n) :${NORMAL} " UPDATEHDR
if [ $? -ge 128 ] ; then UPDATEHDR=n ; fi
echo
if [ ${UPDATEHDR} == y ] ; then
  echo "${BLU}Updating source code file headers.${NORMAL}"
  # remove all backup files ending with "~"
  find . -name "*.*~" -exec rm {} \;
  VERSION=${REV:1:1}.${REV:2}
  YEAR=$(date +%Y)
  HDRMASTER=_Utils/header.txt	# location of the master verbiage
  # replace version and date and calculate number of lines
  HDR_LINES=$( cat ${HDRMASTER} | sed "s/{VERSION}/$VERSION/g" | \
  sed "s/{DATE}/$YEAR/g" | tee ${HDRMASTER}_ | wc -l )

  # Scan source code files.  A header is assumed to be contained within
  for FN in ${SRC_LIST} ; do
    # Extract the last line number of the header.
    #  Returns empty if it does not exist.
    LINE=$(cat ${FN} | egrep -n "~~~ End of Header ~~~"); LINE=${LINE:0:2}
    if [ ${#LINE} -gt 0 ] ; then
      # Process main source code and add EQU with version as the 1st line.
      if [[ ${FN} == AttoBASIC* ]] ; then
        echo -e "\t.equ\tVersion = ${REV:1}\t;version ${REV:1:1}.${REV:2} (100x so its an integer)" > _$(basename $FN)
        cat $FN | sed "1,${LINE}d" | cat _$(basename $FN) ${HDRMASTER}_ - > __$FN
        mv __$FN $FN
      else
        # Process only remaining INC files
        echo "Processing $FN ..."
        cat $FN | sed "1,${LINE}d" | cat ${HDRMASTER}_ - > _$(basename $FN)
        mv _$(basename $FN) $FN
      fi
    fi
  done
  rm ${HDRMASTER}_
fi

#-----------------------------------------------------------------------------------
# build libraries
#-----------------------------------------------------------------------------------
echo
read -t 15 -N 1 -p "${YEL}Build support libraries? (y/n) :${NORMAL} " BUILDLIBS
if [ $? -ge 128 ] ; then BUILDLIBS=n ; fi
echo
if [ ${BUILDLIBS} == y ] ; then
  echo "${BLU}Processing ${WHT}Bootloaders ${BLU}file(s)${NORMAL}"
  pushd ${BLDRDIR} > /dev/null 2>&1
  source mk_attobasic_bldr.sh
  popd > /dev/null 2>&1 			#return to project directory

  echo "${BLU}Processing ${WHT}USB Serial I/O ${BLU}file(s)${NORMAL}"
  pushd ${USBSIODIR} > /dev/null 2>&1
  source makeall.sh > /dev/null 2>&1
  popd > /dev/null 2>&1 			#return to project directory

  echo "${BLU}Processing ${WHT}USI Serial I/O ${BLU}file(s)${NORMAL}"
  pushd ${USISIODIR} > /dev/null 2>&1
  source makeall.sh > /dev/null 2>&1
  popd > /dev/null 2>&1 			#return to project directory

  #  echo "${BLU}Processing ${WHT}SWUART Serial I/O ${BLU}file(s)${NORMAL}"
  #  pushd ${SWUARTSIODIR} > /dev/null 2>&1
  #  source makeall.sh > /dev/null 2>&1
  #  popd > /dev/null 2>&1 			#return to project directory

  echo "${BLU}Processing ${WHT}Data Recorder ${BLU}file(s)${NORMAL}"
  ./mk_dr_def_prg.sh > /dev/null 2>&1
fi

#-----------------------------------------------------------------------------------
# Build HEX files
#-----------------------------------------------------------------------------------
echo
read -t 15 -N 1 -p "${YEL}Build HEX  files? (y/n) :${NORMAL} " BUILDHEX
if [ $? -ge 128 ] ; then BUILDHEX=y ; fi	# timeout assumes to build HEX files
echo
if [ ${BUILDHEX} == y ] ; then
  if [ -e ${DEFS} ] ; then
    DEFSSIZE=`stat -c %s ${DEFS}`		# get current DEFS file size
    if [ ${DEFSSIZE} -ge ${MINDEFSSIZE} ] ; then
      cp ${DEFS} ${DEFS}.bak				#make a copy of current Defs_Constants.inc
    else
      echo ;
      echo "${RED}The file ${DEFS} is not complete!${NORMAL}" ;
      echo "  Restore from a backup and restart." ;
      echo ;
      exit ;
    fi
  else
    echo "Error: ${DEFS} does not exist!"
    exit
  fi

  if [ ${MKLST} == Y ] ; then echo "Assembly Listing is On." ; fi

  #PSCLR=$(cat ${DEFS}.bak | egrep  "#define	FCLK_PS") #save the prescaler value
  LINE=$(cat ${DEFS}.bak | egrep -n "+++ Break Here +++")
  LINE=${LINE:0:2} ;					# extract the line number
  #cat ${DEFS}.bak | sed '1,'${LINE}'d' > ${DEFS}

  #get the actual file name
  FN=$(ls AttoBASIC*.asm | tr '[:lower:]' '[:upper:]') ;
  if [ ${FN} == $(basename ${FNPFX}.ASM) ] ; then
    FN=$(ls AttoBASIC*.asm) ;
  else echo "No project file found!" ;
  fi

  rm -fr ${FLAVDIR}/*						#clear out target HEX file directory

  if [ ${NOBUILD} == 0 ] ; then
    for TARGET in $PARTS ; do		#target MCUs
      # Pull the MCU signature bytes from the DEF file.
      SIG000="$(cat Include/${TARGET}def.inc | egrep "SIGNATURE_000" | cut -d ' ' -f2)" ; SIG000=${SIG000:0:4} ;
      SIG001="$(cat Include/${TARGET}def.inc | egrep "SIGNATURE_001" | cut -d ' ' -f2)" ; SIG001=${SIG001:0:4} ;
      SIG002="$(cat Include/${TARGET}def.inc | egrep "SIGNATURE_002" | cut -d ' ' -f2)" ; SIG002=${SIG002:0:4} ;
      mkdir -p ${FLAVDIR}/${TARGET}/					#make the target directory
      for MCLK in 4 8 16 20 ; do						#target MCLK
        for BUILD in usb_btlddfu usb_btldcdc usb_nobtldr uart_btldr uart_nobtldr datarecdr_btldr datarecdr_nobtldr teensypp20 ; do		#target build options
          case "${BUILD}" in
            #=====================================================
            "usb_btlddfu" )
              if ( [ ${TARGET} == m32u4 ] || [ ${TARGET} == usb1286 ] ) \
              && ( [ ${MCLK} == 8 ] || [ ${MCLK} == 16 ] ) ; then #USB builds only for m32u4 and usb1286 @8/16MHz
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;	#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;			# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;		# next line
                echo "#define BTLDR 1" >> ${DEFS} ;			# next line
                echo "#define BTDFU 1" >> ${DEFS} ;			# next line
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 1" >> ${DEFS} ;				# next line
                echo "#define TEENSY 0" >> ${DEFS} ;			# next line
                echo "#define DREC 0" >> ${DEFS} ;				# next line
                echo "#define TEST 0" >> ${DEFS} ;				# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS}
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
            #=====================================================
            "usb_btldcdc" )
              if ( [ ${TARGET} == m32u4 ] || [ ${TARGET} == usb1286 ] ) \
              && ( [ ${MCLK} == 8 ] || [ ${MCLK} == 16 ] ) ; then #USB builds only for m32u4 and usb1286 @8/16MHz
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;	#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;			# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;		# next line
                echo "#define BTLDR 1" >> ${DEFS} ;			# next line
                echo "#define BTDFU 0" >> ${DEFS} ;			# next line
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 1" >> ${DEFS} ;				# next line
                echo "#define TEENSY 0" >> ${DEFS} ;			# next line
                echo "#define DREC 0" >> ${DEFS} ;				# next line
                echo "#define TEST 0" >> ${DEFS} ;				# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS}
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
            #=====================================================
            "usb_nobtldr" )
              if ( [ ${TARGET} == m32u4 ] || [ ${TARGET} == usb1286 ] ) \
              && ( [ ${MCLK} == 8 ] || [ ${MCLK} == 16 ] ) ; then #USB builds only for m32u4 and usb1286 @8/16MHz
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;	#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;				# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;		#next line
                echo "#define BTLDR 0" >> ${DEFS} ;				# next line
                echo "#define BTDFU	0" >> ${DEFS} ;
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 1" >> ${DEFS} ;				# next line
                echo "#define TEENSY 0" >> ${DEFS} ;				# next line
                echo "#define DREC 0" >> ${DEFS} ;				# next line
                echo "#define TEST 0" >> ${DEFS} ;				# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS}
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
            #=====================================================
            "uart_btldr" )
              if [ ${TARGET} != m88pa ] && [ ${TARGET} != 2313 ]  && [ ${TARGET} != tn2313 ] && \
              [ ${TARGET} != 8515 ] && [ ${TARGET} != m163 ] ; then
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;	#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;				# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;		#next line
                echo "#define BTLDR 1" >> ${DEFS} ;				# next line
                echo "#define BTDFU	0" >> ${DEFS} ;
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 0" >> ${DEFS} ;				# next line
                echo "#define TEENSY 0" >> ${DEFS} ;				# next line
                echo "#define DREC 0" >> ${DEFS} ;				# next line
                echo "#define TEST 0" >> ${DEFS} ;				# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS}
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
            #=====================================================
            "uart_nobtldr" )
              if [ ${TARGET} == m88pa ] && [ ${TARGET} == 2313 ] && [ ${TARGET} == tn2313 ] && \
              [ ${TARGET} == 8515 ] && [ ${TARGET} == m163 ] && [ ${TARGET} == m16 ] && [ ${TARGET} == m32 ] && \
              [ ${TARGET} == m644p ] && [ ${TARGET} == m1284p ] ; then
                BUILD=uart
              fi
              echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
              HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
              MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
              LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
              LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
              echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;		#1st line
              echo "#define FCLK_PS 1" >> ${DEFS} ;				# next line
              echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;			#next line
              echo "#define BTLDR 0" >> ${DEFS} ;				# next line
              echo "#define BTDFU	0" >> ${DEFS} ;
              echo "#define USI 0" >> ${DEFS} ;			# next line
              echo "#define USB 0" >> ${DEFS} ;				# next line
              echo "#define TEENSY 0" >> ${DEFS} ;				# next line
              echo "#define DREC 0" >> ${DEFS} ;				# next line
              echo "#define TEST 0" >> ${DEFS} ;				# next line
              if [ ${DEBUG} == 0 ] ; then
                ./WineBuild.sh >/dev/null 2>&1
              else
                ./WineBuild.sh
              fi
              cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS} ;
              if [ -e ${FN%*.*}.hex ] ; then
                echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                echo
                mv ${FN%*.*}.hex ${HEXFN} ;
                mv ${FN%*.*}.map ${MAPFN} ;
                mv ${FN%*.*}.log ${LOGFN} ;
                if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
              else
                echo
                logerror ${TARGET} ${MCLK} ${BUILD}
                mv ${FN%*.*}.log ${LOGFN} ;
                echo
              fi
            ;;
            "teensypp20" )		# Specific build for Teensy++ 2.0
              if [ ${TARGET} == usb1286 ] && [ ${MCLK} == 16 ] ; then
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;		#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;					# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;			#next line
                echo "#define BTLDR 0" >> ${DEFS} ;					# next line
                echo "#define BTDFU	0" >> ${DEFS} ;
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 1" >> ${DEFS} ;					# next line
                echo "#define TEENSY 1" >> ${DEFS} ;					# next line
                echo "#define DREC 0" >> ${DEFS} ;						# next line
                echo "#define TEST 0" >> ${DEFS} ;						# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS} ;
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
                elif [ ${TARGET} == usb1286 ] && [ ${MCLK} == 8 ] ; then
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;		#1st line
                echo "#define FCLK_PS 2" >> ${DEFS} ;					# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;			#next line
                echo "#define BTLDR 0" >> ${DEFS} ;					# next line
                echo "#define BTDFU	0" >> ${DEFS} ;
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 1" >> ${DEFS} ;					# next line
                echo "#define TEENSY 1" >> ${DEFS} ;					# next line
                echo "#define DREC 0" >> ${DEFS} ;						# next line
                echo "#define TEST 0" >> ${DEFS} ;						# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS} ;
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
            #=====================================================
            "datarecdr_btldr" )		# Specific build for AVR Data Recorder
              if [ ${TARGET} == m328 ] ; then #Data Recorder builds only for M328
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;		#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;					# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;		#next line
                echo "#define BTLDR 1" >> ${DEFS} ;					# next line
                echo "#define BTDFU	0" >> ${DEFS} ;
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 0" >> ${DEFS} ;					# next line
                echo "#define TEENSY 0" >> ${DEFS} ;					# next line
                echo "#define DREC 1" >> ${DEFS} ;						# next line
                echo "#define TEST 0" >> ${DEFS} ;						# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS} ;
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
            #=====================================================
            "datarecdr_nobtldr" )		# Specific build for AVR Data Recorder
              if [ ${TARGET} == m328 ] ; then #Data Recorder builds only for M328
                echo "${BLU}Processing ${WHT}${TARGET} ${BLU}@ ${WHT}${MCLK}MHZ${BLU} with ${YEL}${BUILD}${BLU} option ..."
                HEXFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.hex ;
                MAPFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.map ;
                LOGFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.log ;
                LSTFN=${FLAVDIR}/${TARGET}/${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}.lst ;
                echo ".include \"Include/${TARGET}def.inc\"" > ${DEFS} ;		#1st line
                echo "#define FCLK_PS 1" >> ${DEFS} ;					# next line
                echo "#define FCLK ${MCLK}000000" >> ${DEFS} ;		#next line
                echo "#define BTLDR 0" >> ${DEFS} ;					# next line
                echo "#define BTDFU	0" >> ${DEFS} ;
                echo "#define USI 0" >> ${DEFS} ;			# next line
                echo "#define USB 0" >> ${DEFS} ;					# next line
                echo "#define TEENSY 0" >> ${DEFS} ;					# next line
                echo "#define DREC 1" >> ${DEFS} ;						# next line
                echo "#define TEST 0" >> ${DEFS} ;						# next line
                if [ ${DEBUG} == 0 ] ; then
                  ./WineBuild.sh >/dev/null 2>&1
                else
                  ./WineBuild.sh
                fi
                cat ${DEFS}.bak | sed '1,'${LINE}'d' >> ${DEFS} ;
                if [ -e ${FN%*.*}.hex ] ; then
                  echo "${BLU}Moving ${YEL}${FNPFX}_${TARGET}-${MCLK}MHZ-${BUILD}${BLU} files..."
                  echo
                  mv ${FN%*.*}.hex ${HEXFN} ;
                  mv ${FN%*.*}.map ${MAPFN} ;
                  mv ${FN%*.*}.log ${LOGFN} ;
                  if [ ${MKLST} == Y ] ; then mv ${FN%*.*}.lst ${LSTFN} ; fi
                else
                  echo
                  logerror ${TARGET} ${MCLK} ${BUILD}
                  mv ${FN%*.*}.log ${LOGFN} ;
                  echo
                fi
              fi
            ;;
          esac
        done
      done
    done
  fi
  #
  #update the timer and calculate the difference
  END=$(date +%s.%N) ;
  DIFF=$(echo "$END - $START" | bc)
  #
  if [ -s ${ERRFN} ] ; then
    echo "${BLU}============================ ${YEL}Error List${BLU} ====================================${NORMAL}"
    cat ${ERRFN} ;						#print the error log
    echo "${BLU}============================================================================${NORMAL}"
  fi
  echo
  echo "${CYA}`howmanyfiles ${FLAVDIR}` files were built."
  echo "${YEL}$(date -u -d @${DIFF} +"%M:%S")${CYA} (mm:ss) to build.${NORMAL}"
  echo
  #
  rm -f ${ERRFN} 2>&1 >/dev/null				#remove error log file
  cp -f ${DEFS}.bak ${DEFS} 2>&1 >/dev/null		#restore original Defs_Constants.inc file

fi
