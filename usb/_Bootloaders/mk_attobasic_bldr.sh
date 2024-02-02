#! /bin/bash

. colordefs.sh					# terminal color definitions

DEBUG=0							# "1" for debugging

LUFADIR="LUFA-110528"			#location of LUFA bootloaders

ORG="ORG"						#remove the "ORG" statement line

BOOTSTRTTN85=0x1E00				#start of Bootloader for Tiny85
BOOTENDTN85=0x1FFF				#end of Bootloader for Tiny85

BOOTSTRTM168=0x3E00				#start of Bootloader for Mega168
BOOTENDM168=0x3FFF				#end of Bootloader for Mega168

BOOTSTRTM328=0x7E00				#start of Bootloader for Mega328
BOOTENDM328=0x7FFF				#end of Bootloader for Mega328

BOOTSTRTM16=0x3E00				#start of Bootloader for Mega16/L
BOOTENDM16=0x3FFF				#end of Bootloader for Mega16/L

BOOTSTRTM32=0x7E00				#start of Bootloader for Mega32/A
BOOTENDM32=0x7FFF				#end of Bootloader for Mega32/A

BOOTSTRTM644=0xFC00				#start of Bootloader for Mega644
BOOTENDM644=0xFCFF				#end of Bootloader for Mega644

BOOTSTRTM1284=0x1FC00			#start of Bootloader for Mega1284
BOOTENDM1284=0x1FCFF			#end of Bootloader for Mega1284

BOOTSTRTM2560=0x3E000			#start of Bootloader for Mega2560
BOOTENDM2560=0x3FFFF			#end of Bootloader for Mega2560

# build for each bootloader type
for DIR in optiboot85 optiboot stk500v2 ; do	#our source directories
  BOOTSRCDIR=images			#location of our HEX files
  BOOTSRCPFX=${DIR}_			#prefix of each HEX file
  BOOTSRCSFX=mhz.hex			#suffix of each HEX file

  # define target MCU's based on the bootloader type
  if [ ${DIR} == stk500v2 ] ; then
    BTLD=STK					#set bootloader type
    TARGET="m2560"			#define target MCU for this bootloader
  elif [ ${DIR} == optiboot ] ; then
    BTLD=AVR					#set bootloader type
    TARGET="m168 m328 m16 m32 m644p m1284p"	#define target MCU for this bootloader
  elif [ ${DIR} == optiboot85 ]; then
    BTLD=AVR					#set bootloader type
    #      TARGET="tn85"			#define target MCU for this bootloader
    TARGET=""					#define target MCU for this bootloader
  fi

  #location of target files
  BOOTDESTDIR=$(find ${ROOTDIR} -maxdepth 2 -type d -name "Include")

  pushd ${DIR}	2>&1 >/dev/null
  source makeall.sh				#make all bootloader flavors

  for MCU in ${TARGET}; do
    BOOTDESTSFX=mhz.inc			#suffix of each target file
    BOOTDESTPFX=Code_Bootloader${BTLD}-${MCU}	#prefix of each target file

    if [ $MCU == tn85 ] ; then
      BOOTSTRT=${BOOTSTRTTN85} ;
      BOOTEND=${BOOTENDTN85} ;
    elif [ $MCU == m168 ] ; then
      BOOTSTRT=${BOOTSTRTM168} ;
      BOOTEND=${BOOTENDM168} ;
    elif [ $MCU == m328 ] ; then
      BOOTSTRT=${BOOTSTRTM328} ;
      BOOTEND=${BOOTENDM328} ;
    elif [ $MCU == m16 ] ; then
      BOOTSTRT=${BOOTSTRTM16} ;
      BOOTEND=${BOOTENDM16} ;
    elif [ $MCU == m32 ] ; then
      BOOTSTRT=${BOOTSTRTM32} ;
      BOOTEND=${BOOTENDM32} ;
    elif [ $MCU == m2560 ] ; then
      BOOTSTRT=${BOOTSTRTM2560} ;
      BOOTEND=${BOOTENDM2560} ;
    fi

    for MCLK in 4 8 10 16 20 ; do
      if [ ${MCU} == drec ] ; then
        MCU=m328 ;
      fi
      if [ ${DEBUG} == 1 ] ; then
        echo "${MAG}====================================="
        echo "PWD        = ${PWD}"
        echo "MCU        = ${MCU}"
        echo "MCLK       = ${MCLK}"
        echo "DIR        = ${DIR}"
        echo "BootSrcDir = ${BOOTSRCDIR}"
        echo "Src Pfx    = ${BOOTSRCPFX}"
        echo "Src Sfx    = ${BOOTSRCSFX}"
		echo "Src File   = ${BOOTSRCPFX}${MCU}-${MCLK}${BOOTSRCSFX}"
        echo "Dest Pfx   = ${BOOTDESTPFX}"
        echo "Dest Sfx   = ${BOOTDESTSFX}"
        echo "Dest Dir   = ${BOOTDESTDIR}"
        echo "Dest File  = ${BOOTDESTPFX}-${MCLK}${BOOTDESTSFX}"
        echo "BootStart  = ${BOOTSTRT}"
        echo "BootEnd    = ${BOOTEND}"
        echo "Conditional= ${BOOTSRCDIR}/${BOOTSRCPFX}${MCU}-${MCLK}${BOOTSRCSFX}"
        echo ${NML}
      fi
      if [ -e ${BOOTSRCDIR}/${BOOTSRCPFX}${MCU}-${MCLK}${BOOTSRCSFX} ] ; then
        rm -f ${BOOTDESTDIR}/${BOOTDESTPFX}${MCU}-${MCLK}${BOOTDESTSFX}	#delete existing file
        echo "${BLU}    Processing: ${WHT}${BOOTSRCDIR}/${BOOTSRCPFX}${MCU}-${MCLK}${BOOTSRCSFX}${NORMAL}"
#        srec_cat ${BOOTSRCDIR}/${BOOTSRCPFX}${MCU}-${MCLK}${BOOTSRCSFX} -intel \
#        -fill 0xFF ${BOOTSTRT} ${BOOTEND} \
#        -o - -asm -HEXadecimal_STyle | \
        srec_cat ${BOOTSRCDIR}/${BOOTSRCPFX}${MCU}-${MCLK}${BOOTSRCSFX} -intel \
        -o - -asm -HEXadecimal_STyle | \
        sed '/'"$ORG"'/ d' |
        sed 's/ DB/ .db/' > ${BOOTDESTDIR}/${BOOTDESTPFX}-${MCLK}${BOOTDESTSFX}
      fi
    done
  done
  popd 2>&1 > /dev/null
  echo
done
#-------------------------------------------------------------------------------
# Process the DFU Bootloader
echo
echo "${BLU}Processing ${WHT}DFU Bootloader ${BLU}file(s)${NORMAL}"
pushd ${LUFADIR}/Bootloaders/DFU/ 2>&1 > /dev/null
source makeall.sh > /dev/null 2>&1
popd 2>&1 > /dev/null				#return to project directory

# Process the CDC Bootloader
echo "${BLU}Processing ${WHT}CDC Bootloader ${BLU}file(s)${NORMAL}"
pushd ${LUFADIR}/Bootloaders/CDC/ 2>&1 > /dev/null
source makeall.sh > /dev/null 2>&1
popd 2>&1 > /dev/null				#return to project directory

