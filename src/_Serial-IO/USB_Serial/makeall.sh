#!/bin/bash
#
#
# Added by Scott Vitale for SFG200 support
# create a config file that contains the USB serial strings
REV="${REV:0:2}.${REV:2:2}"

USB_CONFIG=usb_config.h			# file holding USB identifier strings
LINE_NO=$( grep -n '#define STR_PRODUCT' < ${USB_CONFIG} ) ; LINE_NO=${LINE_NO:0:2}
sed -e "${LINE_NO}d" ${USB_CONFIG} > _${USB_CONFIG}
sed -e "${LINE_NO}i#define STR_PRODUCT\t\t\L\"AttoBASIC ${REV}\"" _${USB_CONFIG} > ${USB_CONFIG}

LINE_NO=$( grep -n '#define STR_SERIAL_NUM' < ${USB_CONFIG} ) ; LINE_NO=${LINE_NO:0:2}
sed -e "${LINE_NO}d" ${USB_CONFIG} > _${USB_CONFIG}
sed -e "${LINE_NO}i#define STR_SERIAL_NUM\t\t\L\"AttoBASIC ${REV}\"" _${USB_CONFIG} > ${USB_CONFIG}

rm _${USB_CONFIG}

make clean > /dev/null 2>&1 ; make M32U4-8M > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make M32U4-16M > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make USB1286-8M > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make USB1286-16M > /dev/null 2>&1
make clean > /dev/null 2>&1 ; make TEENSYPP20 > /dev/null 2>&1
make clean > /dev/null 2>&1
