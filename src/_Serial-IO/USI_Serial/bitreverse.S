#include <avr/io.h>
// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
.global Bit_Reverse
.func Bit_Reverse
Bit_Reverse:
	ldi r25, 0x80
	rotate_bit:
	rol r24
	ror r25
	brcc rotate_bit
	mov r24, r25
	ret
.endfunc

