#! /bin/bash
# Updates and adds headers to source code files.

export REV=$(basename $PWD |  tr '[:lower:]' '[:upper:]') ;
SRC_LIST="AttoBASIC*.asm Include/Code*.inc Include/D[ae][tf][as]*.inc"

echo "${BLU}Updating source code file headers.${NORMAL}"

# remove all backup files ending with "~"
find . -name "*.*~" -exec rm {} \;
VERSION=${REV:1:1}.${REV:2}
YEAR=$(date +%Y)
HDRMASTER=_Utils/header.txt	# location of the master verbiage

# replace version and date and calculate number of lines
HDR_LINES=$( cat ${HDRMASTER} | sed "s/{VERSION}/$VERSION/g" | \
sed "s/{DATE}/$YEAR/g" | tee ${HDRMASTER}_ | wc -l )
for FN in ${SRC_LIST}  ; do
  # Extract the last line number of the header.
  #  Returns empty if it does not exist.
  LINE=$(cat ${FN} | egrep -n "~~~ End of Header ~~~"); LINE=${LINE:0:2}

echo "Line = $LINE"

  if [ ${#LINE} == 0 ] ; then
    # Add EQU with version to main source code file
    if [[ ${FN} == AttoBASIC* ]] ; then
      echo -e "\t.equ\tVersion = ${REV:1}\t\t;version ${REV:1:1}.${REV:2} (100x so its an integer)" > _$(basename $FN)
    fi

    # Process only remaining INC files
    echo "Processing $FN ..."
#    head -n $(( ${LINE} )) _${HDRMASTER} >> _$(basename $FN)
    cat ${HDRMASTER}_ $FN  > _$(basename $FN)
    mv _$(basename $FN) $FN
  fi
done

rm ${HDRMASTER}_
