#!/bin/bash
#
#
. colordefs.sh				# terminal color definitions

pushd Ver4v5 >/dev/null 2>&1	# Using Version 4.5 code

echo "  ${YEL}Processing Tiny85 Optiboot85 flavors ...${NORMAL}"
# Added by Scott Vitale for AttoBASIC support
#make clean > /dev/null 2>&1 ; make TN85-1M > /dev/null 2>&1
#make clean > /dev/null 2>&1 ; make TN85-2M > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make TN85-4 > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make TN85-8 > /dev/null 2>&1
#make clean > /dev/null 2>&1 ; make TN85-10 > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make TN85-16 > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make TN85-20 > /dev/null 2>&1
make clean > /dev/null 2>&1

popd >/dev/null 2>&1
