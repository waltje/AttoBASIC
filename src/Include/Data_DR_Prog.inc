;************************************************
; This is the default program that is loaded upon
;  execution of the LDD command.  It is copied 
;  directly to program RAM.  User must then save 
;  to EEPROM                                     
;************************************************
Default_Prog:                                    
	.db "5 DFV 0; DFI 0 255; DFX",0x0d
	.db "10 RTI 1 ",0x0d
	.db "15 ADG 0 1; ADG 1 1",0x0d
	.db "20 RTI 2 ; RTR ",0x0d
	.db "25 A:=PEEK VPG@DFA1",0x0d
	.db "30 B:=PEEK VPG@DFA0",0x0d
	.db "35 M:= A= $FF; N:= B> $FE",0x0d
	.db "40 IF M & N THEN GOTO 110",0x0d
	.db "45 DFL PEEK VPG@RTC2 ",0x0d
	.db "50 DFL PEEK VPG@RTC1 ",0x0d
	.db "55 DFL PEEK VPG@RTC0 ",0x0d
	.db "60 DFL ADC 0 ",0x0d
	.db "65 DFL ADC 1 ",0x0d
	.db "70 PRI ",'"',"RTP: ",'"',"; RTP",0x0d
	.db "75 PRI ",'"',"ADC0: ",'"',"; PRI ADC 0 ",0x0d
	.db "80 PRI ",'"',"ADC1: ",'"',"; PRI ADC 1 ",0x0d
	.db "85 PRI ",'"',"~",'"'," ",0x0d
	.db "90 FOR I= 1 60 ",0x0d
	.db "95 SLP 6 ",0x0d
	.db "100 NEXT ",0x0d
	.db "105 GOTO 25",0x0d
	.db "110 DFF 5",0x0d
	.db "115 END",0x0d
	.dw 0			;Terminate string
