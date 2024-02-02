###############################################################################
# Makefile for the project usi_uart
###############################################################################

## General Flags
PROJECT = vuart
TARGET = $(PROJECT)

# Set the desired baud rate
BAUDRATE=38400

# Added by KSV
# Add extra command line define for different targets
PROG_START = 0x01C88
#PROGDATA_START = 0x01FFE
DATA_START = 0x80021A

# Port and pin definitions
UART_Port	= PORTB
UART_DDR	= DDRB
PRX		= 2
PTX		= 1

#Pass the I/O ports to the compiler
DEFS	+= '-DUART_Port=$(UART_Port)' '-DUART_DDR=$(UART_DDR)' '-DUART_Tx=$(PTX)' '-DUART_Rx=$(PRX)'

#for creating file names based on F_CPU
FRQ	= $(shell echo ${F_CPU} / 1000000 | bc)

# For AttoBASIC use
DEF_FILE	= Defs_$(PROJECT)-$(MCU).inc
CODE_FILE	= Code_$(PROJECT)-$(MCU)-$(FRQ)M.inc

# Directory where out "include file goes"
INC_DIR = ../../Include
#INC_DIR = .

### Tiny84 variants
ifeq ($(MAKECMDGOALS),TN84-1M)
  MCU = attiny84
  F_CPU = 1000000
  BAUDRATE=9600
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN84-2M)
  MCU = attiny84
  F_CPU = 2000000
  BAUDRATE=9600
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN84-4M)
  MCU = attiny84
  F_CPU = 4000000
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN84-8M)
  MCU = attiny84
  F_CPU = 8000000
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
  LFOPT = -U lfuse:w:0xd2:m
endif

ifeq ($(MAKECMDGOALS),TN84-10M)
  MCU = attiny84
  F_CPU = 10000000
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN84-16M)
  MCU = attiny84
  F_CPU = 16000000
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
  LFOPT = -U lfuse:w:0xe1:m
#  CODE_FILE = Code_$(TARGET)_$(MCU)-16M.inc
endif

ifeq ($(MAKECMDGOALS),TN84-20M)
  MCU = attiny84
  F_CPU = 20000000
  FUSEOPT = $(FUSEOPT_t84)
  LOCKOPT = -U lock:w:0x2f:m
endif

### Tiny85 variants
ifeq ($(MAKECMDGOALS),TN85-1M)
  MCU = attiny85
  F_CPU = 1000000
  BAUDRATE=9600
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN85-2M)
  MCU = attiny85
  F_CPU = 2000000
  BAUDRATE=9600
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN85-4M)
  MCU = attiny85
  F_CPU = 4000000
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN85-8M)
  MCU = attiny85
  F_CPU = 8000000
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
  LFOPT = -U lfuse:w:0xd2:m
endif

ifeq ($(MAKECMDGOALS),TN85-10M)
  MCU = attiny85
  F_CPU = 10000000
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN85-16M)
  MCU = attiny85
  F_CPU = 16000000
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
  LFOPT = -U lfuse:w:0xe1:m
#  CODE_FILE = Code_$(TARGET)_$(MCU)-16M.inc
endif

ifeq ($(MAKECMDGOALS),TN85-20M)
  MCU = attiny85
  F_CPU = 20000000
  FUSEOPT = $(FUSEOPT_t85)
  LOCKOPT = -U lock:w:0x2f:m
endif

### Tiny13 variants
ifeq ($(MAKECMDGOALS),TN13-1M)
  MCU = attiny13
  F_CPU = 1000000
  BAUDRATE=9600
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN13-2M)
  MCU = attiny13
  F_CPU = 2000000
  BAUDRATE=9600
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN13-4M)
  MCU = attiny13
  F_CPU = 4000000
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN13-8M)
  MCU = attiny13
  F_CPU = 8000000
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
  LFOPT = -U lfuse:w:0xd2:m
endif

ifeq ($(MAKECMDGOALS),TN13-10M)
  MCU = attiny13
  F_CPU = 10000000
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
endif

ifeq ($(MAKECMDGOALS),TN13-16M)
  MCU = attiny13
  F_CPU = 16000000
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
  LFOPT = -U lfuse:w:0xe1:m
#  CODE_FILE = Code_$(TARGET)_$(MCU)-16M.inc
endif

ifeq ($(MAKECMDGOALS),TN13-20M)
  MCU = attiny13
  F_CPU = 20000000
  FUSEOPT = $(FUSEOPT_t13)
  LOCKOPT = -U lock:w:0x2f:m
endif

# Default MCU name
ifndef MCU
  MCU = attiny85
endif

# Default Processor frequency.
ifndef F_CPU
  F_CPU = 16000000
endif

ifndef BAUDRATE
  BAUDRATE=38400
endif

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c

# List C++ source files here. (C dependencies are automatically generated.)
CPPSRC =

## Include Directories
INCLUDES = .

# Object files directory
#     To put object files in current directory, use a dot (.), do NOT make
#     this an empty or blank macro!
OBJDIR = .

## Objects that must be built in order to link
# Define all object files.
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o)

# Create object files directory
$(shell if [ ! -d $(OBJDIR) ] ; then mkdir $(OBJDIR) 2>/dev/null ; fi)

# Include the dependency files.
-include $(wildcard .dep/*)

# Define all listing files.
LST = $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst)

# Define programs and commands.
SHELL = sh
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AR = avr-ar rcs
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp
SREC = srec_cat
CAT = cat
SED = sed
CUT = cut
WINSHELL = cmd

# Define Messages in English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_ELF = Creating ELF file:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_CREATING_INCLUDE = Creating include file:
MSG_CREATING_DEFINES = Creating defines file:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_CPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU) -DF_CPU=$(F_CPU)
#Pass the baudrate to the compiler
COMMON += -DBAUDRATE=$(BAUDRATE)

## UART_INVERT reverses the polarity of TXD and RXD to
## connect to RS-232C directly.
#COMMON += -DUART_INVERT

# List Assembler source files here.
#     Make them always end in a capital .S.  Files ending in a lowercase .s
#     will not be considered source files but generated files (assembler
#     output from the compiler), and will be deleted upon "make clean"!
#     Even though the DOS/Win* filesystem matches both .s and .S the same,
#     it will preserve the spelling of the filenames, and gcc itself does
#     care about how the name is spelled on its command-line.
ASRC =

# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s
#OPT = 0

# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2
#DEBUG = stabs

# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
#CSTANDARD = -std=c99
CSTANDARD = -std=gnu99

# Place -D or -U options here for C sources
CDEFS  = -DF_CPU=$(F_CPU)UL
CDEFS += -DF_USB=$(F_USB)UL
#CDEFS += -DPROG_START_ADDR=$(PROG_START)UL
#CDEFS += -DPROG_START_ADDR=$(PROG_START)UL

## Compile options common for all C compilation units.
CFLAGS = $(COMMON) $(DEFS)
CFLAGS += $(CSTANDARD)
CFLAGS += -Wall -gdwarf-2 -Os -fsigned-char
#CFLAGS += -MD -MP -MT $(*F).o -MF .dep/$(@F).d
CFLAGS += -MD -MP -MT $(*F).o -MF $(OBJDIR)/$(@F).d
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -ffunction-sections
CFLAGS += -fno-inline-small-functions
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -fno-strict-aliasing

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS  = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--retain-symbols-file $(TARGET)-api.lst
LDFLAGS += -nostartfiles -nodefaultlibs -nostdlib
LDFLAGS += -Wl,--relax

#--------------- linker Options for placing code at specific location -------
ifdef PROG_START
  LDFLAGS += -Wl,--section-start=.text=$(PROG_START)
endif
#LDFLAGS += -Wl,-Ttext,$(PROG_START)	# same as line above
ifdef $PROGDATA_START
  LDFLAGS += -Wl,--section-start=.progdata=$(PROGDATA_START)
endif

#--------------- This code's specific RAM addreses as linker Options -------
ifdef DATA_START
  LDFLAGS += -Wl,--section-start=.data=$(DATA_START)
  LDFLAGS += -Wl,-Tdata,$(DATA_START)	# same as line above
endif
LDFLAGS +=

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_CPPFLAGS = -mmcu=$(MCU) -I. -x c++ $(CPPFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)

## Intel Hex file production flags
# Output format. (can be srec, ihex, binary)
FORMAT = ihex
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature
HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) $(MCU_FLAG) $(FORMAT_FLAG) $(TARGET).elf
MCU_FLAG = $(shell $(SIZE) --help | grep -- --mcu > /dev/null && echo --mcu=$(MCU) )
FORMAT_FLAG = $(shell $(SIZE) --help | grep -- --format=.*avr > /dev/null && echo --format=avr )

## Objects explicitly added by the user
LINKONLYOBJECTS =

# Generate Code for each Target
# Tiny84 variants
TN84-1M : all
TN84-2M : all
TN84-4M : all
TN84-8M : all
TN84-10M : all
TN84-16M : all
TN84-20M : all

# Tiny85 variants
TN85-1M : all
TN85-2M : all
TN85-4M : all
TN85-8M : all
TN85-10M : all
TN85-16M : all
TN85-20M : all

# Tiny13 variants
TN13-1M : all
TN13-2M : all
TN13-4M : all
TN13-8M : all
TN13-10M : all
TN13-16M : all
TN13-20M : all

## Build
all : begin gccversion sizebefore build sizeafter end
#all: $(TARGET) hex eep lss size

# Change the build target to build a HEX file or a library.
.PHONY : build
build: elf hex eep lss sym inc

elf : $(TARGET).elf
hex : $(TARGET).hex
eep : $(TARGET).eep
lss : $(TARGET).lss
sym : $(TARGET).sym
inc : $(TARGET).inc

# Eye candy.
# Display compiler version information.
.PHONY: gccversion
gccversion :
	@$(CC) --version

# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin :
	@echo
	@echo $(MSG_BEGIN)

end :
	@echo $(MSG_END)
	@echo

sizebefore :
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter :
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	2>/dev/null; echo; fi

# Always compile version.c to get correct compilation date and time
.PHONY: version.c
version.c :
	@echo '#include <avr/pgmspace.h>' > $@
	@echo '#include "version.h"' >> $@
	@echo -n 'const char fwVersionStr[] PROGMEM = "Build: "' >> $@
	@date +'"%Y%m%d.%H%M\r\n";' >> $@;

# Create ELF output file.
%.eep : %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) $< $@ || exit 0

# Create final output files (.hex, .eep) from ELF output file.
%.hex : %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) --gap-fill 0xFF $(HEX_FLASH_FLAGS) $< $@

# Create extended listing file from ELF output file.
%.lss : %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -z $< > $@

# Create a symbol table from ELF output file.
%.sym : %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Create library from object files.
.SECONDARY : $(TARGET).a
.PRECIOUS : $(OBJ)
%.a : $(OBJ)
	@echo
	@echo $(MSG_CREATING_LIBRARY) $@
	$(AR) $@ $(OBJ)

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf : $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $^ --output $@ $(LDFLAGS)

# Compile: create object files from C source files.
$(OBJDIR)/%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@

# Compile: create object files from C++ source files.
$(OBJDIR)/%.o : %.cpp
	@echo
	@echo $(MSG_COMPILING_CPP) $<
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@

# Compile: create assembler files from C++ source files.
%.s : %.cpp
	$(CC) -S $(ALL_CPPFLAGS) $< -o $@

# Assemble: create object files from assembler source files.
$(OBJDIR)/%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@

# Create the include hex listing and subroutine pointers for use with the AttoBASIC project.
%.inc:
	@echo
	@echo $(MSG_CREATING_INCLUDE) $(INC_DIR)/$(CODE_FILE) at $(PROG_START)
	@$(REMOVE) $(INC_DIR)/$(CODE_FILE)
	@$(SREC) $(PROJECT).hex -intel -offset -$(PROG_START) -o - -asm -HEXadecimal_STyle | \
		$(SED) 's/ DB / .db/g' > $(INC_DIR)/$(CODE_FILE)
	@echo
	@echo $(MSG_CREATING_DEFINES) $(INC_DIR)/$(DEF_FILE)
	@$(CAT) $(PROJECT).sym | \
		$(SED) 's/ T / FLASH /g' | \
		$(SED) 's/ t / FLASH /g' | \
		$(SED) 's/0000/0x/g' | \
		$(SED) 's/0001/0x1/g' | \
		$(SED) 's/ d / RAM /g' | \
		$(SED) 's/ b / RAM /g' | \
		$(SED) 's/ D / RAM /g' | \
		$(SED) 's/ B / RAM /g' | \
		$(SED) 's/0080/0x/g' | \
		$(SED) 's/__ctors_start/SWUART/g' | \
		$(SED) 's/__vector_10/TIMER0_COMPA_int/g' | \
		$(SED) 's/__vector_11/TIMER0_COMPB_int/g' | \
		$(SED) 's/__vector_1/INT0_int/g' >> temp.sym
		@echo "${UART_Port} RAM UART_Port" >> temp.sym
		@echo "${UART_DDR} RAM UART_DDR" >> temp.sym
		@echo "${PTX} RAM UART_Tx" >> temp.sym
		@echo "${PRX} RAM UART_Rx" >> temp.sym
	@$(REMOVE) $(INC_DIR)/$(DEF_FILE)
	@echo $(MSG_REMOVING) $(INC_DIR)/$(DEF_FILE)
	@$(CAT) temp.sym | while read ADDR TYPE SYMBOL ; do \
		if [ $$TYPE == FLASH ] ; then echo ".equ $$SYMBOL = $$ADDR/2" >> $(INC_DIR)/$(DEF_FILE) ; fi ;\
		if [ $$TYPE == RAM ] ; then echo ".equ $$SYMBOL = $$ADDR" >> $(INC_DIR)/$(DEF_FILE) ; fi ;\
		done
	@$(REMOVE) temp.sym

# Target: clean project.
.PHONY: clean clean_list
clean: begin clean_list end

clean_list :
	@echo $(MSG_CLEANING)
	@$(REMOVE) $(TARGET).hex
	@$(REMOVE) $(TARGET).eep
	@$(REMOVE) $(TARGET).cof
	@$(REMOVE) $(TARGET).elf
	@$(REMOVE) $(TARGET).map
	@$(REMOVE) $(TARGET).sym
	@$(REMOVE) $(TARGET).lss
	@$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.o*) $(CPPSRC:%.cpp=$(OBJDIR)/%.o*) $(ASRC:%.S=$(OBJDIR)/%.o*)
	@$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst)
	@$(REMOVE) $(SRC:.c=.s)
	@$(REMOVE) $(SRC:.c=.d)
	@$(REMOVE) $(SRC:.c=.i)
	@$(REMOVEDIR) .dep
	@$(REMOVE) $(LST) $(GDBINITFILE)
