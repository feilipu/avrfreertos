# ----------------------------------------------------------------------------
# Makefile to compile and link stk500boot bootloader
# Author: Peter Fleury
# File:   $Id: Makefile,v 1.3 2006/03/04 19:26:17 peter Exp $
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


# Target file name (without extension).
TARGET = stk500boot_v2_mega2560


# List C source files here. (C dependencies are automatically generated.)
SRC = stk500boot.c 


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
#OPT = s 
OPT = 2  # XXX Opt 2 seems to make flash loading much faster - ymmv


# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = stabs


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
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
		 -fno-jump-tables -frename-registers -mrelax -fweb
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


#--------------- bootloader linker Options -------
# BOOTLOADER_ADDRESS (=Start of Boot Loader section
# in bytes - not words) is defined above.
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nostartfiles -nodefaultlibs
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nostartfiles
LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS)

#---------------- Programming Options (avrdude) ----------------

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500 stk500v2 usbtiny
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = avrisp2

# com1 = serial port. Use lpt1 to connect to parallel port.
#AVRDUDE_PORT = com1    # programmer connected to serial device
#AVRDUDE_PORT = /dev/ttyACM0
AVRDUDE_PORT = usb

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep

# avrdude needs a part number, that is not the same as the mcu.
#Valid parts are:
#  t10  = ATtiny10        [/etc/avrdude.conf:15443]
#  t8   = ATtiny9         [/etc/avrdude.conf:15403]
#  t5   = ATtiny5         [/etc/avrdude.conf:15363]
#  t4   = ATtiny4         [/etc/avrdude.conf:15323]
#  ucr2 = 32UC3A0512      [/etc/avrdude.conf:15302]
#  x128a4 = ATXMEGA128A4    [/etc/avrdude.conf:15204]
#  x64a4 = ATXMEGA64A4     [/etc/avrdude.conf:15107]
#  x32a4 = ATXMEGA32A4     [/etc/avrdude.conf:15010]
#  x16a4 = ATXMEGA16A4     [/etc/avrdude.conf:14913]
#  x256a3b = ATXMEGA256A3B   [/etc/avrdude.conf:14816]
#  x256a3 = ATXMEGA256A3    [/etc/avrdude.conf:14719]
#  x192a3 = ATXMEGA192A3    [/etc/avrdude.conf:14622]
#  x128a3 = ATXMEGA128A3    [/etc/avrdude.conf:14525]
#  x64a3 = ATXMEGA64A3     [/etc/avrdude.conf:14428]
#  x256a1 = ATXMEGA256A1    [/etc/avrdude.conf:14331]
#  x192a1 = ATXMEGA192A1    [/etc/avrdude.conf:14234]
#  x128a1d = ATXMEGA128A1REVD [/etc/avrdude.conf:14137]
#  x128a1 = ATXMEGA128A1    [/etc/avrdude.conf:14040]
#  x64a1 = ATXMEGA64A1     [/etc/avrdude.conf:13943]
#  m6450 = ATMEGA6450      [/etc/avrdude.conf:13754]
#  m3250 = ATMEGA3250      [/etc/avrdude.conf:13565]
#  m645 = ATMEGA645       [/etc/avrdude.conf:13376]
#  m325 = ATMEGA325       [/etc/avrdude.conf:13187]
#  usb82 = AT90USB82       [/etc/avrdude.conf:13007]
#  usb162 = AT90USB162      [/etc/avrdude.conf:12823]
#  usb1287 = AT90USB1287     [/etc/avrdude.conf:12634]
#  usb1286 = AT90USB1286     [/etc/avrdude.conf:12445]
#  usb647 = AT90USB647      [/etc/avrdude.conf:12257]
#  usb646 = AT90USB646      [/etc/avrdude.conf:12068]
#  m32u4 = ATmega32U4      [/etc/avrdude.conf:11880]
#  t84  = ATtiny84        [/etc/avrdude.conf:11697]
#  t44  = ATtiny44        [/etc/avrdude.conf:11515]
#  t24  = ATtiny24        [/etc/avrdude.conf:11333]
#  m128rfa1 = ATMEGA128RFA1   [/etc/avrdude.conf:11145]
#  m2561 = ATMEGA2561      [/etc/avrdude.conf:10951]
#  m2560 = ATMEGA2560      [/etc/avrdude.conf:10758]
#  m1281 = ATMEGA1281      [/etc/avrdude.conf:10570]
#  m1280 = ATMEGA1280      [/etc/avrdude.conf:10381]
#  m640 = ATMEGA640       [/etc/avrdude.conf:10193]
#  t85  = ATtiny85        [/etc/avrdude.conf:10012]
#  t45  = ATtiny45        [/etc/avrdude.conf:9833]
#  t25  = ATtiny25        [/etc/avrdude.conf:9653]
#  pwm3b = AT90PWM3B       [/etc/avrdude.conf:9472]
#  pwm2b = AT90PWM2B       [/etc/avrdude.conf:9289]
#  pwm3 = AT90PWM3        [/etc/avrdude.conf:9106]
#  pwm2 = AT90PWM2        [/etc/avrdude.conf:8922]
#  t2313 = ATtiny2313      [/etc/avrdude.conf:8735]
#  m328p = ATMEGA328P      [/etc/avrdude.conf:8547]
#  t88  = attiny88        [/etc/avrdude.conf:8361]
#  m168 = ATMEGA168       [/etc/avrdude.conf:8173]
#  m88  = ATMEGA88        [/etc/avrdude.conf:7987]
#  m48  = ATMEGA48        [/etc/avrdude.conf:7800]
#  t861 = ATTINY861       [/etc/avrdude.conf:7612]
#  t461 = ATTINY461       [/etc/avrdude.conf:7423]
#  t261 = ATTINY261       [/etc/avrdude.conf:7234]
#  t26  = ATTINY26        [/etc/avrdude.conf:7077]
#  m8535 = ATMEGA8535      [/etc/avrdude.conf:6919]
#  m8515 = ATMEGA8515      [/etc/avrdude.conf:6761]
#  m8   = ATMEGA8         [/etc/avrdude.conf:6600]
#  m161 = ATMEGA161       [/etc/avrdude.conf:6460]
#  m32  = ATMEGA32        [/etc/avrdude.conf:6285]
#  m6490 = ATMEGA6490      [/etc/avrdude.conf:6092]
#  m649 = ATMEGA649       [/etc/avrdude.conf:5907]
#  m3290p = ATMEGA3290P     [/etc/avrdude.conf:5724]
#  m3290 = ATMEGA3290      [/etc/avrdude.conf:5539]
#  m329p = ATMEGA329P      [/etc/avrdude.conf:5354]
#  m329 = ATMEGA329       [/etc/avrdude.conf:5170]
#  m169 = ATMEGA169       [/etc/avrdude.conf:4990]
#  m163 = ATMEGA163       [/etc/avrdude.conf:4832]
#  m162 = ATMEGA162       [/etc/avrdude.conf:4636]
#  m1284p = ATMEGA1284P     [/etc/avrdude.conf:4446]
#  m644p = ATMEGA644P      [/etc/avrdude.conf:4254]
#  m644 = ATMEGA644       [/etc/avrdude.conf:4064]
#  m324p = ATMEGA324P      [/etc/avrdude.conf:3873]
#  m164p = ATMEGA164P      [/etc/avrdude.conf:3682]
#  m16  = ATMEGA16        [/etc/avrdude.conf:3504]
#  c32  = AT90CAN32       [/etc/avrdude.conf:3313]
#  c64  = AT90CAN64       [/etc/avrdude.conf:3123]
#  c128 = AT90CAN128      [/etc/avrdude.conf:2933]
#  m128 = ATMEGA128       [/etc/avrdude.conf:2755]
#  m64  = ATMEGA64        [/etc/avrdude.conf:2574]
#  m103 = ATMEGA103       [/etc/avrdude.conf:2434]
#  8535 = AT90S8535       [/etc/avrdude.conf:2313]
#  8515 = AT90S8515       [/etc/avrdude.conf:2199]
#  4434 = AT90S4434       [/etc/avrdude.conf:2116]
#  4433 = AT90S4433       [/etc/avrdude.conf:1992]
#  2343 = AT90S2343       [/etc/avrdude.conf:1868]
#  2333 = AT90S2333       [/etc/avrdude.conf:1740]
#  2313 = AT90S2313       [/etc/avrdude.conf:1627]
#  4414 = AT90S4414       [/etc/avrdude.conf:1514]
#  1200 = AT90S1200       [/etc/avrdude.conf:1399]
#  t15  = ATtiny15        [/etc/avrdude.conf:1266]
#  t13  = ATtiny13        [/etc/avrdude.conf:1093]
#  t12  = ATtiny12        [/etc/avrdude.conf:960]
#  t11  = ATtiny11        [/etc/avrdude.conf:896]
  
AVRDUDE_PART = m1284p


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

AVRDUDE_FLAGS = -p $(AVRDUDE_PART) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
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
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
COPY = cp
WINSHELL = cmd


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
#	May 25,	2010	<MLS> Adding 1280 support
mega1280: MCU = atmega1280
mega1280: F_CPU = 16000000
mega1280: BOOTLOADER_ADDRESS = 1E000
mega1280: CFLAGS += -D_MEGA_BOARD_
mega1280: begin gccversion sizebefore build sizeafter end 
			mv $(TARGET).hex stk500boot_v2_mega1280.hex


############################################################
#	Jul 6,	2010	<MLS> Adding 2560 support
mega2560:	MCU = atmega2560
mega2560:	F_CPU = 16000000
mega2560:	BOOTLOADER_ADDRESS = 3E000
mega2560:	CFLAGS += -D_MEGA_BOARD_
mega2560:	begin gccversion sizebefore build sizeafter end 
#			mv $(TARGET).hex stk500boot_v2_mega2560.hex
			

############################################################
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
amber128: BOOTLOADER_ADDRESS = 1E000
amber128: CFLAGS += -D_BOARD_AMBER128_
amber128: begin gccversion sizebefore build sizeafter end 
			mv $(TARGET).hex stk500boot_v2_amber128.hex

############################################################
#	Aug 23, 2010 	<MLS> Adding atmega2561 support
m2561: MCU = atmega2561
m2561: F_CPU = 8000000
m2561: BOOTLOADER_ADDRESS = 3E000
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
cerebot:	BOOTLOADER_ADDRESS = 3E000
cerebot:	CFLAGS += -D_CEREBOTPLUS_BOARD_ -DBAUDRATE=38400 -DUART_BAUDRATE_DOUBLE_SPEED=1
cerebot:	begin gccversion sizebefore build sizeafter end 
			mv $(TARGET).hex stk500boot_v2_cerebotplus.hex


############################################################
#	Aug 23, 2010 	<MLS> Adding atmega2561 support
penguino: MCU = atmega32
penguino: F_CPU = 16000000
penguino: BOOTLOADER_ADDRESS = 7800
penguino: CFLAGS += -D_PENGUINO_ -DBAUDRATE=57600
penguino: begin gccversion sizebefore build sizeafter end 
			mv $(TARGET).hex stk500boot_v2_penguino.hex


# Default target.
all: begin gccversion sizebefore build sizeafter end

build: elf hex eep lss sym
#build:  hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym



# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	2>/dev/null; echo; fi



# Display compiler version information.
gccversion : 
	@$(CC) --version



# Program the device.  
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)


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

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $^ --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@ 


# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) *.hex
	$(REMOVE) *.eep
	$(REMOVE) *.cof
	$(REMOVE) *.elf
	$(REMOVE) *.map
	$(REMOVE) *.sym
	$(REMOVE) *.lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) .dep/*



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program debug gdb-config

