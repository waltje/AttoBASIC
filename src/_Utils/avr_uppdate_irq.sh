# update_irq.sh - Place the AVR IRQ vector table in the "SOURCE" file
#  Execute this script and the resulting output is placed in the
#  "DESTINATION" file.
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

if [ $# == 0 ] ; then
   echo
   echo "${WHT}Usage: ${YEL}`basename $0` [SOURCE] [DESTINATION]${NML}"
   echo "${BLU}  where: SOURCE is a file named \"${YEL}Source.txt${BLU}\" containing"
   echo "  the vector table in the format:"
   echo -e "${CYN}\trjmp\tRESET\t\t\t;Reset vector"
   echo -e "\trjmp\tINT0_int\t\t;External Interrupt Request 0"
   echo -e "\trjmp\tPCINT0_int\t\t;Pin Change Interrupt Request 0${NML}"
   echo
else
   SOURCE=Source.txt
   DESTINATION=VectorTable.txt

   if [ -f ${DESTINATION} ] ; then rm -f ${DESTINATION} ; fi

   cat ${SOURCE} | while read "IRQ" "DESC" ; do
      LBL="`echo ${IRQ} | sed 's/://g'`"
      echo ".ifndef ${LBL}" >> ${DESTINATION}
      echo -en "  ${IRQ}" >> ${DESTINATION}
      if [ ${#IRQ} -le 10 ] ; then
         echo -en "\t\t\t\t\t\t" >> ${DESTINATION}
         elif [ ${#IRQ} -le 13 ] ; then
         echo -en "\t\t\t\t\t" >> ${DESTINATION}
         elif [ ${#IRQ} -le 21 ] ; then
         echo -en "\t\t\t\t" >> ${DESTINATION}
      else
         echo -en "\t\t\t" >> ${DESTINATION}
      fi
      echo "${DESC}" >> ${DESTINATION}
      echo ".endif" >> ${DESTINATION}
   done
fi
