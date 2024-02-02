#! /bin/bash

DEBUG=1							# set to "1" for debug info

PROJDIR=$PWD					# where are we?
FNPFX="$(basename $PWD)"		# file name prefix is the current directory name
FNSFX=zip						# file name suffix is always "zip"
FLIST="files.lst"				# list of files to archive
ZIPOPTS="-9vr"					# list of options to pass to zip

if [ $DEBUG == 1 ] ; then
  echo "PROJDIR = $PROJDIR"
  echo "FNPFX   = $FNPFX"
  echo "FNSFX   = $FNSFX"
  echo "FLIST   = $FLIST"
  echo "ZIPOPTS = $ZIPOPTS"
fi

for N in a b c d ; do
  if [ -e ${FLIST} ] ; then rm -f ${FLIST} >/dev/null 2>&1 ; fi	# delete the previous file list
  FN="../${FNPFX}${N}.${FNSFX}"	# set the target file name
  echo
  echo "+++++++++++++++++++++++++++++++++++++++++++++++"
  case $N in
    a)
      #Ex: *.pdf, *.html, assets/* and 2016-1204_ATTOBASICV234_APP.ZIP
      find . -name "*.pdf" -or -name "*.htm*" -or -name "*_APP.ZIP" -or -name "assets" >> ${FLIST}	# create file list for file "a"
#      find assets >> ${FLIST}
    ;;

    b)
      # Ex: 2016-1204_ATTOBASICV234_HEX.ZIP and 2016-1204_ATTOBASICV234_SRC.ZIP
      find . -name "*_HEX.ZIP" -or -name "*_SRC.ZIP" >> ${FLIST}	# create file list for file "b"
    ;;

    c)
      #Ex: 2016-1204_ATTOBASICV234_USB.ZIP
      find . -name "*_USB.ZIP" >> ${FLIST}	# create file list for file "c"
    ;;

    d)
      # Ex: 2016-1204_ATTOBASICV234_CODE.ZIP
      find . -name "*_CODE.ZIP" >> ${FLIST}	# create file list for file "d"
    ;;
  esac

  if [ $DEBUG == 1 ] ; then
    echo "Files to archive in ${FN} are:"
    cat ${FLIST}
    echo "-----------------------------------------------"
    echo
  fi

  echo "Processing ${FN} ..."
  if [ -e ${FN} ] ; then rm -f ${FN} >/dev/null 2>&1 ; fi	# delete the previous file list
  cat ${FLIST} | zip ${ZIPOPTS} ${FN} -@
done
if [ -e ${FLIST} ] ; then rm -f ${FLIST} >/dev/null 2>&1 ; fi	# delete the previous file list
