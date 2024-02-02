# vim: set ts=4 noet:
PROGRAM  = optiboot85
TARGET  = attiny85
MCU_TARGET = $(TARGET)

# Port and pin definitions for RX/TX pins
UART_Port	= PORTB
UART_DDR	= DDRB
PRX		= 0
PTX		= 1
LED		= 2

#for creating file names based on F_CPU
FRQ		= $(shell echo ${AVR_FREQ} / 1000000 | bc)

# set specifics about program location, etc.
BAUD_RATE	= 38400			# desired baud rate
#APP_VECT	= 0x0C
APP_VECT	= 0x07
VER_START	= 0x1ffe		# where to store the Optiboot version (2 bytes)
RESET_START	= 0x0000		# start of application program, usually the RESET vector
RAM_START	= 0x0060		# start of RAM
FLASHES		= 0
ifeq ($(FLASHES),0)			# if we don't need the LED then we save a few bytes of code
 BOOT_START	= 0x1E26
else
 BOOT_START	= 0x1E00
endif

CODE_FILE	= Code_$(PROGRAM)-$(FRQ)M.inc	# the name of the target AttoBASIC include file
DEF_FILE	= Defs_$(PROGRAM).inc			# the name of the target AttoBASIC entry point definitions

# gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE    = avr-size
CC      = avr-gcc
AS      = avr-as
NM	= avr-nm
#commandline utils
SHELL	= /bin/bash
COPY	= cp
MOVE	= mv
SREC	= srec_cat
CAT	= cat
SED	= sed
CUT	= cut
BC	= bc
REMOVE	= rm -f
REMOVEDIR = rm -rf

IMGDIR	 = images
OBJ      = $(PROGRAM).o
DEFS	 = -DBOOT_START=$(BOOT_START) -DRAMSTART=$(RAM_START) -DAPP_VECT=$(APP_VECT) -DF_CPU=$(AVR_FREQ) -DBAUD_RATE=$(BAUD_RATE)
DEFS	+= '-DLED_START_FLASHES=$(FLASHES)' '-DSOFT_UART' '-DLED=$(LED)' '-DUART_Tx=$(PTX)' '-DUART_Rx=$(PRX)' '-DVIRTUAL_BOOT_PARTITION'
OPTIMIZE = -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls

#override CFLAGS  = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
#override LDFLAGS = -m$(TARGET) $(LDSECTIONS) -Wl,-Map=$(PRORAM).map,--cref -nostartfiles -nostdlib -Wl,-bihex
#CFLAGS	+= '-DLED_START_FLASHES=3' '-DSOFT_UART' '-DLED=$(LED)' '-DUART_Tx=$(PTX)' '-DUART_Rx=$(PRX)' '-DVIRTUAL_BOOT_PARTITION'

CFLAGS  = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
CFLAGS		+= -Wa,--gdwarf-2 -Wa,-alcms=$(PROGRAM).lst -Wa,-D -Wa,--warn
LDSECTIONS	 = -Wl,--print-gc-sections \
		   -Wl,--section-start=.reset=$(RESET_START) \
		   -Wl,--section-start=.text=$(BOOT_START) \
		   -Wl,--section-start=.version=$(VER_START)
LDFLAGS = -m$(TARGET) $(LDSECTIONS) -Wl,-Map=$(PROGRAM).map,--cref -nostartfiles -nostdlib -Wl,-bihex -Wl,--retain-symbols-file $(PROGRAM)-api.lst

# Directory where out "include file goes"
INC_DIR = ../../../Include
#INC_DIR = .

# Define Messages in English
MSG_ERRORS_NONE = Errors: none
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_ELF = Creating ELF file:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_CREATING_INCLUDE = Creating include file:
MSG_CREATING_DEFINES = Creating defines file:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_REMOVING = Removing

#========================================================================
.PHONY: TN85-4
TN85-4: TARGET	= attiny85
TN85-4: FUSES		= -e -Uefuse:w:0xFE:m -Uhfuse:w:0xD7:m -Ulfuse:w:0xE2:m
TN85-4: CODE_FILE	= Code_$(PROGRAM)-4M.inc
TN85-4: DEF_FILE	= Defs_$(PROGRAM).inc
TN85-4: AVR_FREQ	= 4000000
TN85-4: $(PROGRAM).hex
TN85-4: $(PROGRAM).lst
TN85-4: $(PROGRAM).lss
TN85-4: $(PROGRAM).sym
TN85-4: $(PROGRAM).inc
TN85-4: finish

#========================================================================
.PHONY: TN85-8
TN85-8: TARGET	= attiny85
TN85-8: FUSES		= -e -Uefuse:w:0xFE:m -Uhfuse:w:0xD7:m -Ulfuse:w:0xE2:m
TN85-8: AVR_FREQ	= 8000000
TN85-8: $(PROGRAM).hex
TN85-8: $(PROGRAM).lst
TN85-8: $(PROGRAM).lss
TN85-8: $(PROGRAM).sym
TN85-8: $(PROGRAM).inc
TN85-8: finish

#========================================================================
.PHONY: TN85-16
TN85-16: TARGET	= attiny85
TN85-16: FUSES		= -e -Uefuse:w:0xFE:m -Uhfuse:w:0xD7:m -Ulfuse:w:0xE2:m
TN85-16: AVR_FREQ	= 16000000
TN85-16: $(PROGRAM).hex
TN85-16: $(PROGRAM).lst
TN85-16: $(PROGRAM).lss
TN85-16: $(PROGRAM).sym
TN85-16: $(PROGRAM).inc
TN85-16: finish

#========================================================================
.PHONY: TN85-20
TN85-20: TARGET	= attiny85
TN85-20: FUSES		= -e -Uefuse:w:0xFE:m -Uhfuse:w:0xD7:m -Ulfuse:w:0xE2:m
TN85-20: AVR_FREQ	= 20000000
TN85-20: $(PROGRAM).hex
TN85-20: $(PROGRAM).lst
TN85-20: $(PROGRAM).lss
TN85-20: $(PROGRAM).sym
TN85-20: $(PROGRAM).inc
TN85-20: finish

#========================================================================
%.o: %.S
	@echo
	@echo $(MSG_ASSEMBLING) $(PROGRAM).S
	$(CC) $(CFLAGS) -c -o $@ $<

%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $(PROGRAM).elf
	$(CC) $(LDFLAGS) -o $@ $^
	$(SIZE) $@

%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $(PROGRAM).hex
	@$(OBJCOPY) --gap-fill 0xFF -j .text -j .optiboot -j .data -j .version --set-section-flags .version=alloc,load -O ihex $< $@

# Create extended listing file from ELF output file.
%.lss : %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $(PROGRAM).lss
	@$(OBJDUMP) -h -S -z $< > $@

# Create a symbol table from ELF output file.
%.sym : %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $(PROGRAM).sym
	@$(NM) -n $< > $@

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Create the include hex listing and subroutine pointers for use with the AttoBASIC project.
%.inc:
	@echo
	@echo $(MSG_CREATING_INCLUDE) $(CODE_FILE) at $(BOOT_START)
	@$(REMOVE) $(INC_DIR)/$(CODE_FILE)
	@$(SREC) $(PROGRAM).hex -intel -offset -$(BOOT_START) -o - -asm -HEXadecimal_STyle | \
		$(SED) 's/ DB / .db/g' > $(INC_DIR)/$(CODE_FILE)
	@echo
	@echo $(MSG_CREATING_DEFINES) $(DEF_FILE)
	@$(CAT) $(PROGRAM).sym | \
		$(SED) 's/ T / FLASH /g' | \
		$(SED) 's/ t / FLASH /g' | \
		$(SED) 's/0000/0x/g' | \
		$(SED) 's/0001/0x1/g' | \
		$(SED) 's/ d / RAM /g' | \
		$(SED) 's/ b / RAM /g' | \
		$(SED) 's/ D / RAM /g' | \
		$(SED) 's/ B / RAM /g' | \
		$(SED) 's/0080/0x/g' > temp.sym
#		@echo "${UART_Port} RAM UART_Port" >> temp.sym
#		@echo "${UART_DDR} RAM UART_DDR" >> temp.sym
#		@echo "${PTX} RAM UART_Tx" >> temp.sym
#		@echo "${PRX} RAM UART_Rx" >> temp.sym
	@$(REMOVE) $(INC_DIR)/$(DEF_FILE)
	@echo $(MSG_REMOVING) $(INC_DIR)/$(DEF_FILE)
	@$(CAT) temp.sym | while read ADDR TYPE SYMBOL ; do \
		if [ $$TYPE == FLASH ] ; then echo ".equ $$SYMBOL = $$ADDR/2" >> $(INC_DIR)/$(DEF_FILE) ; fi ;\
		if [ $$TYPE == RAM ] ; then echo ".equ $$SYMBOL = $$ADDR" >> $(INC_DIR)/$(DEF_FILE) ; fi ;\
		done
	@$(REMOVE) temp.sym

finish:
#settings for HEX, SYM and LST files
	@$(MOVE) $(PROGRAM).hex $(IMGDIR)/$(PROGRAM)_tn85-$(FRQ)mhz.hex
	@$(MOVE) $(PROGRAM).sym $(IMGDIR)/$(PROGRAM)_tn85-$(FRQ)mhz.sym
	@$(MOVE) $(PROGRAM).lss $(IMGDIR)/$(PROGRAM)_tn85-$(FRQ)mhz.lst

clean:
	@echo $(MSG_CLEANING) $(PROGRAM)
	@$(REMOVE) -rf $(PROGRAM).o \
		$(PROGRAM).elf \
		$(PROGRAM).lst \
		$(PROGRAM).map \
		$(PROGRAM).sym \
		$(PROGRAM).lss \
		$(PROGRAM).eep \
		$(PROGRAM).srec \
		$(PROGRAM).bin \
		$(PROGRAM).hex
	@echo
