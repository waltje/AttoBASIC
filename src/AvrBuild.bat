@ECHO OFF
SETLOCAL
SET FN=AttoBASIC
SET FN=foo

avrasm2.exe -Slabels.tmp -fI -W+ie -IInclude -CV3 -o%FN%.hex -d%FN%.obj -e%FN%.eep -m%FN%.map -l%FN%.lst AttoBASICV234.asm
