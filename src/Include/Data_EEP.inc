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
;******************************************************************************
; EEPROM Storage
;******************************************************************************
	.eseg
;******************************************************************************
.org	0
;   0x00 to 0x07 are reserved for AttoBASIC internal use.
RESV_AB:	.byte	8
;
;	0x08 to 0x0F are reserved for USER use.
RESV_USER:	.byte	16
;
.if !EFS	;support for file system (if enabled)
;==============================================================
; SAVe and LOAd program storage (if EFS not enabled)
;	0x10 to the device's E2PEND are reserved for program storage.
RESV_PROG:	.byte	EEPROMEND - E2P_RESV
;
.else 		;support for file system (if enabled)
;==============================================================
;STRUCTURE: The entire EEP is divided into allocation 
;	units of "EFSBLKSZ" bytes.
;
;FILE INDEX:
; The file index records (BPHNDL * HANDLES_MAX) contain
;	the address of the file's 1st allocation unit
;	The number of available file index records is determined 
;	by the "HANDLES_MAX" equate.
; Byte:
;    0x18: File "0"; 1st block of program or  "0x00" for empty 
;    0x19: File "1"; 1st block of program or  "0x00" for empty 
;    0x1A: File "2"; 1st block of program or  "0x00" for empty 
;    0x1B: File "3"; 1st block of program or  "0x00" for empty 
;    0x1C: File "4"; 1st block of program or  "0x00" for empty 
;    0x1D: File "5"; 1st block of program or  "0x00" for empty 
;    0x1E: File "6"; 1st block of program or  "0x00" for empty 
;    0x1F: File "7"; 1st block of program or  "0x00" for empty 
;
;ALLOCATION UNIT: Each allocation unit (32 bytes) consists of:
; Byte:	
;	 0:	Record Type
;	 1:	Start of data payload
;	...
;	30:	End of data payload
;	31:	Next Block
;
; Where "Record Type" is:
;		0xFF = Unused Record (erased EEP byte)
;		0x01 = Data Record
;
;==============================================================
.endif
;
	.org	E2END	;top of EEPROM
 E2PPROGTOP:		;always defined
;
.if EFS	;support for file system
;	.org	0		;bottom of EEPROM
	.org	0x18	;bottom of EEPROM
;	.org	RESV_PROG + RESV_USER + RESV_AB	;bottom of EEPROM
;
#if TEST
  EFSIDX0:	.db		EFSFILE0/EFSBLKSZ		;file 0
  EFSIDX1:	.db		EFSFILE1/EFSBLKSZ		;file 1
  EFSIDX2:	.db		EFSFILE2/EFSBLKSZ		;file 2
  EFSIDX3:	.db		EFSFILE3/EFSBLKSZ		;file 3
#else
  EFSIDX0:	.db		0						;file 0
  EFSIDX1:	.db		0						;file 1
  EFSIDX2:	.db		0						;file 2
  EFSIDX3:	.db		0						;file 3
#endif
  EFSIDX4:	.db		0						;file 4
  EFSIDX5:	.db		0						;file 5
  EFSIDX6:	.db		0						;file 6
  EFSIDX7:	.db		0						;file 7
;
;			.dd		0		;reserve 16 more bytes of EEP
;			.dd		0
;			.dd		0
;			.dd		0
;	.org	(E2END-(BPHNDL*HANDLES_MAX))	;preserved 4 bytes per handle
.endif

#if TEST
.if EFS 
.equ	EFS_RECORD = 1		;"1" to generate dummy EEPROM record for testing.
;
  .if EFS_RECORD
EFSFILE0:
			.db		1
			.db		"10 REM This is program 0",CR
			.db		"20 PR"
			.db		(EFSFILE0a/EFSBLKSZ)
EFSFILE0a:
			.db		1
			.db		"I 10",CR
			.db		"30 PRI 100",CR
			.db		"40 PRI 200",CR
			.db		"50 "
			.db		(EFSFILE0b/EFSBLKSZ)
EFSFILE0b:
			.db		1
			.db		"END",CR
			.db		0,0,0,0,0,0,0,0
			.db		0,0,0,0,0,0,0,0
			.db		0,0,0,0,0,0,0,0
			.db		0,0
			.db		0
;-------
EFSFILE1:
			.db		1
			.db		"10 REM This is program 1",CR
			.db		"20 FO"
			.db		(EFSFILE1a/EFSBLKSZ)
EFSFILE1a:
			.db		1
			.db		"R N=0 10",CR
			.db		"30 PRX N",CR
			.db		"40 NEX",CR
			.db		"50 EN"
			.db		(EFSFILE1b/EFSBLKSZ)
EFSFILE1b:
			.db		1
			.db		"D",CR
			.db		0,0,0,0,0,0,0,0
			.db		0,0,0,0,0,0,0,0
			.db		0,0,0,0,0,0,0,0
			.db		0,0,0,0,0
EFSFILE2:
			.db		0xFF,0x00
			.dw		0
			.dd		0
			.dd		0
			.dd		0
			.dd		0
			.dd		0
			.dd		0
			.dd		0
EFSFILE3:
			.db		1
			.db		"10 REM This is program 3",CR
			.db		"20 RT"
			.db		(EFSFILE3a/EFSBLKSZ)
EFSFILE3a:
			.db		1
			.db		"P ; SLP 5 ; RTP",CR
			.db		"30 SLP 6 ; RTP"
			.db		(EFSFILE3b/EFSBLKSZ)
EFSFILE3b:
			.db		1
			.db		CR
			.db		"30 FOR I=0 100",CR
			.db		"40 PRI ",'"',"The va"
			.db		(EFSFILE3c/EFSBLKSZ)
EFSFILE3c:
			.db		1
			.db		"lue of I is ",'"',' ',';'," PRI I",CR
			.db		"50 NEX",CR
			.db		"6"
			.db		(EFSFILE3d/EFSBLKSZ)
EFSFILE3d:
			.db		1
			.db		"0 PRI ",'"',"Hello",'"',CR
			.db		"70 RTP",CR
			.db		"80 END",CR
			.db		0,0,0
  .endif
.endif
#endif
