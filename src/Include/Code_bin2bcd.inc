; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;                       AttoBASIC V2.34
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Copyright (C)2011-2017 Kenneth Scott Vitale <ksv_prj@gmx.com>
; Note: All code is maintained by Kenneth Scott Vitale since 2011
;
; All Versions of AttoBASIC from V2.00 forward, including ATtiny84,
;	ATtiny85, ATmega16(A)/32(A), ATmega32U4, ATmega88/168/328,
;	AT90USB1286, ATmega644P/1284P and Mega2560 ports and merging of
;   original AttoBASIC code for AT90S2313, AT90S8515 and ATmega163 are
;	Copyright (C)2011-2017 by: K. Scott Vitale, Florida, uSA
;			email: ksv_prj@gmx.com
;	Plese let me know of any bugs or (especially) improvements.
;		Thank you - KSV
; Original AttoBASIC code for AT90S2313, AT90S8515 and ATmega163
;	Copyright 2003 Richard Cappels projects@cappels.org
;	http://projects.cappels.org
;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;  NOTICE:
;	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;   "AS IS"AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
;   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
;   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;	POSSIBILITY OF SUCH DAMAGE.
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;	This source code may be used for personal use only.
;	Commercial License is available.
;	This source code may be republished provided this notice and all
;	  support files are kept intact.
; ~~~~~~~~~~~~~~~~~~~~~~~~~ End of Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;****************** 
; 16 bit binary to 5 digit ASCII conversion 
; the 16 bit value is in bytes ZH & ZL 
; each of the 5 ascii digits is stored (sent) to a separate memory located (UART xmt buffer) 
; 10000 base 10=$2710, 1000 base 10 =$03E8, 100 base 10=0064, 10 base 10= $000A 
; D4/3/2/1ASCII can also be called to convert a 4/3/2/1 digit number to ASCII 
bin2ascii: ldi temp2, $2F ;Init ASCII conversion ('0'-1) 
loop10k: inc temp2      ;count up 10k "counts" 
         subi ZL, $10 
         sbci ZH, $27 
         brcc loop10k  ;IF >=10000 THEN subtract again 
         subi ZL, $F0  ;subtract -10000 (add 10000) 
         sbci ZH, $D8 
       rcall send_char   ;send 10k ASCII char 

d4ascii: ldi temp2, $2F ;Init ASCII conversion ('0'-1) 
loop1k:  inc temp2      ;count up 1k "counts" 
         subi ZL, $E8 
         sbci ZH, $03 
         brcc loop1k   ;IF >=1000 THEN subtract again 
         subi ZL, $18  ;subtract -1000 (add 1000) 
         sbci ZH, $FC 
       rcall send_char 

d3ascii: ldi temp2, $2F ;Init ASCII conversion ('0'-1) 
loop100: inc temp2      ;count up 100 "counts" 
         subi ZL, $64 
         sbci ZH, $00 
         brcc loop100  ;IF >=100 THEN subtract again 
         subi ZL, $9C  ;subtract -100 (add 100) 
         sbci ZH, $FF 
       rcall send_char 
            ;(ZH==0) so ignore ZH from here 

d2ascii: ldi temp2, $2F ;Init ASCII conversion ('0'-1) 
loop10:  inc temp2      ;count up 10 "counts" 
         subi ZL, $0A 
         brcc loop10   ;IF >=10 THEN subtract again 
         subi ZL, $F6  ;subtract -10 (add 10) 
       rcall send_char 
       mov temp2, ZL 
d1ascii: subi temp2, $D0  ;convert 1's digit to ASCII 
         rcall send_char 
       ldi temp2, ' ' 
       rcall send_char  ;send out a space 
       ret

