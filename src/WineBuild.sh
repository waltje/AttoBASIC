#! /bin/bash

DEBUG=0		# set to "1" to enable debug info

#figure out the project file name
if [ $# == 0 ] ; then
  pushd ../../ > /dev/null ;			# up two levels
  #FNBASE=$(basename $PWD | sed 's/-//g' |  tr '[:lower:]' '[:upper:]')
  FNBASE=$(basename $PWD)
  popd > /dev/null
  REV=$(basename $PWD |  tr '[:lower:]' '[:upper:]') ;
  #FNPFX=$(echo ${FNBASE}${REV} | tr '[:lower:]' '[:upper:]') ;
  FNPFX=${FNBASE}${REV} ;
else
  #   FNPFX=$(echo $1 |  tr '[:lower:]' '[:upper:]') ;
  FNPFX=${1} ;
  shift
fi

if [ $DEBUG == 1 ] ; then
  echo "FNPFX=${FNPFX}"
fi

if [ -e $FNPFX.asm ] ; then
  #   (wineconsole "avrasm2.exe" -S "labels.tmp" -I Include/ -fI -W+ie -C V3 -o "$FNPFX.hex" \
  #   -d "$FNPFX.obj" -e "$FNPFX.eep" -m "$FNPFX.map" -l "$FNPFX.lst" "$FNPFX.asm" 2>&1 >/dev/null) &
  
  (wine "avrasm2.exe" -V -v2 -S "labels.tmp" -I Include/ -fI -W+ie $1 $2 $3 $4 $5 \
  -o "$FNPFX.hex" -d "$FNPFX.obj" -e "$FNPFX.eep" -m "$FNPFX.map" -l "$FNPFX.lst" "$FNPFX.asm" > $FNPFX.log 2>&1 ) &
  #   | sed -r 's/\x07//g;s/\x08//g;s/\x00//g;s/\x1b[[()=][;?0-9]*[0-9A-Za-z]?//g;s/\r//g;s/\t/   /g' > $FNPFX.log 2>&1 ) &
  # sed -r 's/\x1B\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g'
  # sed -r s/\x07//g;s/\x08//g;s/\x00//g;s/\x1b[[()=][;?0-9]*[0-9A-Za-z]?//g;s/\r//g
  #   (wine "avrasm2.exe" -S "labels.tmp" -I Include/ -fI -W+ie -C V3 -o "$FNPFX.hex" -d "$FNPFX.obj" -e "$FNPFX.eep" \
  #	-m "$FNPFX.map" -l "$FNPFX.lst" "$FNPFX.asm" > $FNPFX.log 2>&1) &
  
  while [ -d /proc/$! ] ; do sleep 0.5 ; done
else
  echo
  echo "There is no project file named ${FNPFX}!"
  echo
fi
