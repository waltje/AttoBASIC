# ----------------------------------------------------------------------------
# Makefile to compile and link stk500boot bootloader
# Author: Peter Fleury
# based on WinAVR Makefile Template written by Eric B. Weddington, Jörg Wunsch, et al.
#
# Adjust F_CPU below to the clock frequency in Mhz of your AVR target
# Adjust BOOTLOADER_ADDRESS to your AVR target
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging,
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------
#	<MLS> = Mark Sproul msproul-at-skychariot.com

# Target file name (without extension).
TARGET = monitor

# MCU name
#MCU = atmega128


# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the
#     processor frequency. You can then use this symbol in your source code to
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
#F_CPU = 16000000


# Bootloader
# Please adjust if using a different AVR
# 0x0e00*2=0x1C00 for ATmega8  512 words Boot Size
# 0xFC00*2=0x1F800 for ATmega128  1024 words Boot Size
# 0xF800*2=0x1F000 for ATmega1280
# 0xF000*2=0x1E000 for ATmega1280
#BOOTLOADER_ADDRESS = 1E000


# Output format. (can be srec, ihex, binary)
FORMAT = ihex


# List C source files here. (C dependencies are automatically generated.)
SRC = monitor.c
IMGDIR = images

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
#OPT = -Os
OPT = -Os


# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS =


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99


# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)UL


# Place -I options here
CINCS =

#---------------- Compiler Options ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += $(OPT)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-jump-tables
CFLAGS += -Wall -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:.c=.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs

#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
PRINTF_LIB =
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB =
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)

MATH_LIB = -lm

#---------------- External Memory Options ----------------

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =

#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)
#LDFLAGS += -Wl,--retain-symbols-file $(TARGET)-api.lst

#--------------- linker Options -------
# START_ADDRESS (=Start of section in bytes - not words) is defined above.
LDFLAGS += -Wl,--section-start=.text=$(START_ADDRESS) -nostartfiles
#LDFLAGS += -Wl,--section-start=.text=$(START_ADDRESS)
#LDFLAGS += -Wl,--section-start=.text=$(START_ADDRESS) -nostartfiles -nodefaultlibs


#--------------- bootloader linker Options -------
# BOOTLOADER_ADDRESS (=Start of Boot Loader section
# in bytes - not words) is defined above.
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nostartfiles -nodefaultlibs
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nostartfiles
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS)

#---------------- Programming Options (avrdude) ----------------

# Programming hardware: alf avr910 avrisp bascom bsd
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = stk500v2

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = com1    # programmer connected to serial device

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude>
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)

#---------------- Debugging Options ----------------

# For simulavr only - target MCU frequency.
DEBUG_MFREQ = $(F_CPU)

# Set the DEBUG_UI to either gdb or insight.
# DEBUG_UI = gdb
DEBUG_UI = insight

# Set the debugging back-end to either avarice, simulavr.
DEBUG_BACKEND = avarice
#DEBUG_BACKEND = simulavr

# GDB Init Filename.
GDBINIT_FILE = __avr_gdbinit

# When using avarice settings for the JTAG
JTAG_DEV = /dev/com1

# Debugging port used to communicate between GDB / avarice / simulavr.
DEBUG_PORT = 4242

# Debugging host used to communicate between GDB / avarice / simulavr, normally
#     just set to localhost unless doing some sort of crazy debugging when
#     avarice is running on a different computer.
DEBUG_HOST = localhost



#============================================================================


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
MOVE = mv
SREC = srec_cat
CAT = cat
SED = sed
CUT = cut
WINSHELL = cmd

INC_DIR = ../../Include

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_INCLUDE = Creating Include Files:

# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o)

# Define all listing files.
LST = $(SRC:.c=.lst) $(ASRC:.S=.lst)

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m32-20:	MCU = atmega32
m32-20:	F_CPU = 20000000
m32-20:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m32-20:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m32-20:	START_ADDRESS = 0
m32-20:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m32-20:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m32-20:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m32-16:	MCU = atmega32
m32-16:	F_CPU = 16000000
m32-16:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m32-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m32-16:	START_ADDRESS = 0
m32-16:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m32-16:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m32-16:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m32-10:	MCU = atmega32
m32-10:	F_CPU = 10000000
m32-10:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m32-10:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m32-10:	START_ADDRESS = 0
m32-10:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m32-10:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m32-10:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m32-8:	MCU = atmega32
m32-8:	F_CPU = 8000000
m32-8:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m32-8:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m32-8:	START_ADDRESS = 0
m32-8:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m32-8:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m32-8:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m32-4:	MCU = atmega32
m32-4:	F_CPU = 4000000
m32-4:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m32-4:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m32-4:	START_ADDRESS = 0
m32-4:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m32-4:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m32-4:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m328-20:	MCU = atmega328
m328-20:	F_CPU = 20000000
m328-20:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m328-20:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m328-20:	START_ADDRESS = 0
m328-20:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m328-20:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m328-20:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m328-16:	MCU = atmega328
m328-16:	F_CPU = 16000000
m328-16:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m328-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m328-16:	START_ADDRESS = 0
m328-16:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m328-16:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m328-16:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m328-10:	MCU = atmega328
m328-10:	F_CPU = 10000000
m328-10:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m328-10:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m328-10:	START_ADDRESS = 0
m328-10:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m328-10:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m328-10:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m328-8:	MCU = atmega328
m328-8:	F_CPU = 8000000
m328-8:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m328-8:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m328-8:	START_ADDRESS = 0
m328-8:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m328-8:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m328-8:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m328-4:	MCU = atmega328
m328-4:	F_CPU = 4000000
m328-4:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m328-4:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m328-4:	START_ADDRESS = 0
m328-4:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m328-4:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m328-4:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym


############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
usb1286-20:	MCU = at90usb1286
usb1286-20:	F_CPU = 20000000
usb1286-20:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
usb1286-20:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
usb1286-20:	START_ADDRESS = 0
usb1286-20:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
usb1286-20:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
usb1286-20:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
usb1286-16:	MCU = at90usb1286
usb1286-16:	F_CPU = 16000000
usb1286-16:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
usb1286-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
usb1286-16:	START_ADDRESS = 0
usb1286-16:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
usb1286-16:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
usb1286-16:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
usb1286-10:	MCU = at90usb1286
usb1286-10:	F_CPU = 10000000
usb1286-10:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
usb1286-10:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
usb1286-10:	START_ADDRESS = 0
usb1286-10:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
usb1286-10:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
usb1286-10:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
usb1286-8:	MCU = at90usb1286
usb1286-8:	F_CPU = 8000000
usb1286-8:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
usb1286-8:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
usb1286-8:	START_ADDRESS = 0
usb1286-8:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
usb1286-8:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
usb1286-8:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
usb1286-4:	MCU = at90usb1286
usb1286-4:	F_CPU = 4000000
usb1286-4:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
usb1286-4:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
usb1286-4:	START_ADDRESS = 0
usb1286-4:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
usb1286-4:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
usb1286-4:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m644-20:	MCU = atmega644
m644-20:	F_CPU = 20000000
m644-20:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m644-20:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m644-20:	START_ADDRESS = 0
m644-20:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m644-20:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m644-20:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m644-16:	MCU = atmega644
m644-16:	F_CPU = 16000000
m644-16:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m644-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m644-16:	START_ADDRESS = 0
m644-16:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m644-16:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m644-16:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m644-10:	MCU = atmega644
m644-10:	F_CPU = 10000000
m644-10:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m644-10:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m644-10:	START_ADDRESS = 0
m644-10:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m644-10:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m644-10:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m644-8:	MCU = atmega644
m644-8:	F_CPU = 8000000
m644-8:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m644-8:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m644-8:	START_ADDRESS = 0
m644-8:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m644-8:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m644-8:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m644-4:	MCU = atmega644
m644-4:	F_CPU = 4000000
m644-4:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m644-4:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m644-4:	START_ADDRESS = 0
m644-4:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m644-4:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m644-4:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m1284-20:	MCU = atmega1284
m1284-20:	F_CPU = 20000000
m1284-20:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-20:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m1284-20:	START_ADDRESS = 0
m1284-20:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m1284-20:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m1284-20:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m1284-16:	MCU = atmega1284
m1284-16:	F_CPU = 16000000
m1284-16:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m1284-16:	START_ADDRESS = 0
m1284-16:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m1284-16:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m1284-16:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m1284-10:	MCU = atmega1284
m1284-10:	F_CPU = 10000000
m1284-10:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-10:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m1284-10:	START_ADDRESS = 0
m1284-10:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m1284-10:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m1284-10:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m1284-8:	MCU = atmega1284
m1284-8:	F_CPU = 8000000
m1284-8:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-8:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m1284-8:	START_ADDRESS = 0
m1284-8:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m1284-8:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m1284-8:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	Jul 19,	2016	<KSV> Adding various MCU and clock support
m1284-4:	MCU = atmega1284
m1284-4:	F_CPU = 4000000
m1284-4:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-4:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m1284-4:	START_ADDRESS = 0
m1284-4:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m1284-4:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=19200
m1284-4:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
#	May 25,	2010	<MLS> Adding 1280 support
mega1280: MCU = atmega1280
mega1280: F_CPU = 16000000
mega1280: START_ADDRESS = 1E000
mega1280: CFLAGS += -D_MEGA_BOARD_
mega1280: begin gccversion sizebefore build sizeafter end
			mv $(TARGET).hex stk500boot_v2_mega1280.hex


############################################################
#	Jul 6,	2010	<MLS> Adding 2560 support
mega2560:	MCU = atmega2560
mega2560:	F_CPU = 16000000
mega2560:	START_ADDRESS = 0
mega2560:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
mega2560:	begin gccversion sizebefore build sizeafter end
			mv $(TARGET).hex $(TARGET)_mega2560.hex

############################################################
#	Jul 13,	2013	<KSV> Adding various clock support
m2560-20:	MCU = atmega2560
m2560-20:	F_CPU = 20000000
m2560-20:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m2560-20:	START_ADDRESS = 0
m2560-20:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m2560-20:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m2560-20:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
m2560-16:	MCU = atmega2560
m2560-16:	F_CPU = 16000000
m2560-16:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m2560-16:	START_ADDRESS = 0
m2560-16:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m2560-16:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m2560-16:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
m2560-10:	MCU = atmega2560
m2560-10:	F_CPU = 10000000
m2560-10:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m2560-10:	START_ADDRESS = 0
m2560-10:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m2560-10:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
m2560-8:	MCU = atmega2560
m2560-8:	F_CPU = 8000000
m2560-8:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m1284-16:	PROGRAM = $(TARGET)_$(MCU)-$(CLKRATE)mhz
m2560-8:	START_ADDRESS = 0
m2560-8:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m2560-8:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m2560-8:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

############################################################
m2560-4:	MCU = atmega2560
m2560-4:	F_CPU = 4000000
m2560-4:	CLKRATE = $(shell echo $(F_CPU)/1000000 | bc)
m2560-4:	PROGRAM = $(TARGET)_m2560-$(CLKRATE)mhz
m2560-4:	START_ADDRESS = 0
m2560-4:	DEF_FILE = Defs_$(TARGET)-$(MCU).inc
m2560-4:	CFLAGS += -D_MEGA_BOARD_ -DBAUDRATE=38400
m2560-4:	begin gccversion sizebefore build sizeafter end
			$(MOVE) $(TARGET).hex $(IMGDIR)/$(PROGRAM).hex
			$(MOVE) $(TARGET).lss $(IMGDIR)/$(PROGRAM).lst
			$(MOVE) $(TARGET).sym $(IMGDIR)/$(PROGRAM).sym

#########################################################################
#Initial config on Amber128 board
#	avrdude: Device signature = 0x1e9702
#	avrdude: safemode: lfuse reads as 8F
#	avrdude: safemode: hfuse reads as CB
#	avrdude: safemode: efuse reads as FF
#	Jul 17,	2010	<MLS> Adding 128 support
############################################################
amber128: MCU = atmega128
#amber128: F_CPU = 16000000
amber128: F_CPU = 14745600
amber128: START_ADDRESS = 1E000
amber128: CFLAGS += -D_BOARD_AMBER128_
amber128: begin gccversion sizebefore build sizeafter end
			mv $(TARGET).hex stk500boot_v2_amber128.hex

############################################################
#	Aug 23, 2010 	<MLS> Adding atmega2561 support
m2561: MCU = atmega2561
m2561: F_CPU = 8000000
m2561: START_ADDRESS = 3E000
m2561: CFLAGS += -D_ANDROID_2561_ -DBAUDRATE=57600
m2561: begin gccversion sizebefore build sizeafter end
			mv $(TARGET).hex stk500boot_v2_android2561.hex

############################################################
#	avrdude: Device signature = 0x1e9801
#	avrdude: safemode: lfuse reads as EC
#	avrdude: safemode: hfuse reads as 18
#	avrdude: safemode: efuse reads as FD
#	Aug 23,	2010	<MLS> Adding cerebot 2560 @ 8mhz
#avrdude -P usb -c usbtiny -p m2560 -v -U flash:w:/Arduino/WiringBootV2_upd1/stk500boot_v2_cerebotplus.hex
############################################################
cerebot:	MCU = atmega2560
cerebot:	F_CPU = 8000000
cerebot:	START_ADDRESS = 3E000
cerebot:	CFLAGS += -D_CEREBOTPLUS_BOARD_ -DBAUDRATE=38400 -DUART_BAUDRATE_DOUBLE_SPEED=1
cerebot:	begin gccversion sizebefore build sizeafter end
			mv $(TARGET).hex stk500boot_v2_cerebotplus.hex

############################################################
#	Aug 23, 2010 	<MLS> Adding atmega2561 support
penguino: MCU = atmega32
penguino: F_CPU = 16000000
penguino: START_ADDRESS = 7800
penguino: CFLAGS += -D_PENGUINO_ -DBAUDRATE=57600
penguino: begin gccversion sizebefore build sizeafter end
			mv $(TARGET).hex stk500boot_v2_penguino.hex

############################################################
# Default target.
all: begin gccversion sizebefore build sizeafter end

build: elf hex eep lss sym
#build:  hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
#eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym
inc: $(TARGET).inc

############################################################
# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo


############################################################
# Display size of file.
HEXSIZE = $(SIZE) $(TARGET).hex
ELFSIZE = $(SIZE) $(TARGET).elf

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	2>/dev/null; echo; fi

############################################################
# Display compiler version information.
gccversion :
	@$(CC) --version

############################################################
# Program the device.
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)

############################################################
# Generate avr-gdb config/init file which does the following:
#     define the reset signal, load the target file, connect to target, and set
#     a breakpoint at main().
gdb-config:
	@$(REMOVE) $(GDBINIT_FILE)
	@echo define reset >> $(GDBINIT_FILE)
	@echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	@echo end >> $(GDBINIT_FILE)
	@echo file $(TARGET).elf >> $(GDBINIT_FILE)
	@echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	@echo load  >> $(GDBINIT_FILE)
endif
	@echo break main >> $(GDBINIT_FILE)

debug: gdb-config $(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	@echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
	@$(WINSHELL) /c start avarice --jtag $(JTAG_DEV) --erase --program --file \
	$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT)
	@$(WINSHELL) /c pause

else
	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
	@$(WINSHELL) /c start avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)


############################################################
# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000

coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof

extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof

############################################################
# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

############################################################
# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

############################################################
# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

############################################################
# Create the include hex listing and subroutine pointers for use with the AttoBASIC project.
%.inc:
	@echo $(MSG_CREATING_INCLUDE) $< $@ at $(PROG_START)
	@$(SREC) $(TARGET).hex -intel -offset -$(PROG_START) -fill 0xFF -within $(TARGET).hex -o  $(INC_DIR)/$(CODE_FILE) -asm -HEXadecimal_STyle
	@$(SREC) $(TARGET).hex -intel -offset -$(PROG_START) -o $(INC_DIR)/$(CODE_FILE) -asm -HEXadecimal_STyle
	@$(SREC) $(TARGET).hex -intel -offset -$(PROG_START) -o - -asm -HEXadecimal_STyle | \
		$(SED) 's/ DB / .db/g' > $(INC_DIR)/$(CODE_FILE)
	@$(CAT) $(TARGET).sym | $(SED) 's/ T / FLASH /g' | \
		$(SED) 's/0000/0x/g' | \
		$(SED) 's/0001/0x1/g' | \
		$(SED) 's/ d / RAM /g' | \
		$(SED) 's/ b / RAM /g' | \
		$(SED) 's/0080/0x/g' | \
		$(SED) 's/__vector_10/USB_GEN_int/g' | \
		$(SED) 's/__vector_11/USB_COM_int/g' > temp.sym
	@$(REMOVE) $(INC_DIR)/$(DEF_FILE)
	@$(CAT) temp.sym | while read ADDR TYPE SYMBOL ; do \
		if [ $$TYPE == FLASH ] ; then echo ".equ $$SYMBOL = $$ADDR/2" >> $(INC_DIR)/$(DEF_FILE) ; fi ;\
		if [ $$TYPE == RAM ] ; then echo ".equ $$SYMBOL = $$ADDR" >> $(INC_DIR)/$(DEF_FILE) ; fi ;\
		done
	@$(REMOVE) temp.sym

############################################################
# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $^ --output $@ $(LDFLAGS)

############################################################
# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@

############################################################
# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@

############################################################
# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

############################################################
# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@

############################################################
# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) *.hex *.eep *.cof *.elf *.map *.sym *.lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) .dep/*
	$(REMOVE) $(OBJ) \
		$(LST) \
		$(SRC:.c=.s) \
		$(SRC:.c=.d) \
		.dep/* \
		$(IMGDIR)/*.o \
		$(IMGDIR)/*.elf \
		$(IMGDIR)/*.map \
		$(IMGDIR)/*.eep \
		$(IMGDIR)/*.srec \
		$(IMGDIR)/*.bin

############################################################
# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

############################################################
# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program debug gdb-config

