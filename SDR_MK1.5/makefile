# Hey Emacs, this is a -*- makefile -*-
#----------------------------------------------------------------------------
# WinAVR Makefile Template written by Eric B. Weddington, J�rg Wunsch, et al.
#  >> Modified for use with the LUFA project. <<
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau
# Carlos Lamas
# Dean Camera
# Opendous Inc.
# Denver Gingerich
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make dfu = Download the hex file to the device, using dfu-programmer (must
#            have dfu-programmer installed).
#
# make flip = Download the hex file to the device, using Atmel FLIP (must
#             have Atmel FLIP installed).
#
# make doxygen = Generate DoxyGen documentation for the project (must have
#                DoxyGen installed)
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------


# MCU name
MCU = uc3b0512


# Target board (see library "Board Types" documentation, NONE for projects not requiring
# LUFA board drivers). If USER is selected, put custom board drivers in a directory called
# "Board" inside the application directory.
BOARD = NONE


# Targeted chip architecture (see library "Architectures" documentation)
ARCH = UC3

# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the
#     processor frequency in Hz. You can then use this symbol in your source code to
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
#
#     This should be the frequency the system core runs at, after the system clock
#     has been set up correctly and started.
F_CPU = 60000000
#F_CPU = 66000000

#
# DRC clock value to keep the maximum serial bus bitrate under 30MHz and dividing with 48kHz multiples from 192kHz and below
#
#F_ADC = 59904000

#
# DRC clock value closest to 64MHz dividing with 48kHz multiples from 192kHz and below
#
#F_ADC = 63936000



# USB controller master clock frequency.
#     This will define a symbol, F_USB, in all source code files equal to the
#     input clock frequency (before any prescaling is performed) in Hz. This value may
#     differ from F_CPU, as the USB clock is often sourced from the same oscillator as
#     the CPU clock, but run through multipliers and dividers to achieve the desired
#     clock rate.
#
#     For the UC3 chips, this should be equal to 48MHz or 96MHz.
F_USB = 48000000

# PBA bus frequency
F_PBA_SPEED = $(F_CPU)

# CPU Input Clock
FOSC0 = 12000000


# Output format. (can be srec, ihex, binary)
FORMAT = ihex


# Target file name (without extension).
TARGET = SDR_MK1.5


# Object files directory
#     To put object files in current directory, use a dot (.), do NOT make
#     this an empty or blank macro!
OBJDIR = .


# Path to the LUFA library
#LUFA_PATH = ../../../../../engineering/Software/LUFA111203TRUNK
LUFA_PATH = ../../../../../engineering/Software/LUFA120219


# Paths of the ASF. Defining these here will permit using the AVR Software Framework folders directly without modifications
ASF_PATHS = asf/avr32/drivers/flashc     \
            asf/avr32/drivers/gpio       \
            asf/avr32/drivers/intc       \
            asf/avr32/drivers/spi        \
            asf/avr32/drivers/pm         \
            asf/avr32/drivers/cpu        \
			asf/avr32/drivers/tc         \
            asf/avr32/drivers/ssc/i2s    \
			asf/avr32/drivers/pdca       \
            asf/avr32/utils              \
            asf/avr32/utils/preprocessor \
            asf/avr32/boards             \
            asf/common/services/clock    \
            asf/common/services/spi      \
            asf/common/utils             \
            asf/common/boards            \
            config

# uIP Paths
UIP_PATHS = uIP                          \
            uIP/uip                      \
			uIP/drivers                  \
			uIP/drivers/ksz8851          \
			uIP/drivers/interfaces       \
			uIP/libs/memb                \
			uIP/apps/dhcpc


# redefine stuff associated with AVR8 pgmspace
UIP_OPTS = -D PSTR="(uint8_t*)"
UIP_OPTS += -D PGM_VOID_P="void *"
UIP_OPTS += -D PGM_P="uint8_t *"
UIP_OPTS += -D strlen_P="strlen"
UIP_OPTS += -D strncpy_P="strncpy"
#UIP_OPTS += -D memcpy_P="memcpy"




# LUFA library compile-time options and predefined tokens
LUFA_OPTS  = -D USB_DEVICE_ONLY
LUFA_OPTS += -D FIXED_CONTROL_ENDPOINT_SIZE=32
LUFA_OPTS += -D FIXED_NUM_CONFIGURATIONS=1
LUFA_OPTS += -D USE_STATIC_OPTIONS="(USB_DEVICE_OPT_FULLSPEED | USB_OPT_GCLK_SRC_PLL | USB_OPT_GCLK_CHANNEL_1)"
#LUFA_OPTS += -D INTERRUPT_CONTROL_ENDPOINT
LUFA_OPTS += -D NO_SOF_EVENTS
LUFA_OPTS += -D ORDERED_EP_CONFIG
LUFA_OPTS += -D USE_RAM_DESCRIPTORS

# Define _BOARD_H_ so ASF will not nag on us for not having a board defined (we do not use board_init() to be sure nothing surplus gets initialized)
# The board init code is in sdr_mk1.5.c SetupHardware()
LUFA_OPTS += -D _BOARD_H_


# Create the LUFA source path variables by including the LUFA root makefile
include $(LUFA_PATH)/LUFA/makefile


# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c                                                 \
      Descriptors.c                                               \
	  NetSDR.c                                                    \
	  SDR-IQ.c                                                    \
	  terminal.c                                                  \
	  OmniRig.c                                                   \
	  LM97593.c                                                   \
	  twi.c                                                       \
	  tx8m.c                                                      \
	  cdce913_coeff.c                                             \
      asf/common/services/spi/uc3_spi/spi_master.c                \
      asf/common/services/clock/uc3b0_b1/sysclk.c                 \
      asf/avr32/drivers/spi/spi.c                                 \
      asf/avr32/drivers/intc/intc.c                               \
      asf/avr32/drivers/gpio/gpio.c                               \
      asf/avr32/drivers/ssc/i2s/ssc_i2s.c                         \
      asf/avr32/drivers/pm/power_clocks_lib.c                     \
      asf/avr32/drivers/pm/pm_conf_clocks.c                       \
      asf/avr32/drivers/pm/pm.c                                   \
	  asf/avr32/drivers/tc/tc.c                                   \
      asf/avr32/drivers/flashc/flashc.c                           \
	  asf/avr32/drivers/pdca/pdca.c                               \
	  uIP/clock-arch.c                                            \
      uIP/uip/uip.c                                               \
      uIP/uip/uip_arp.c                                           \
      uIP/uip/uiplib.c                                            \
      uIP/uip/timer.c                                             \
      uIP/drivers/ksz8851/network.c                               \
	  uIP/drivers/ksz8851/ksz8851.c                               \
	  uIP/libs/memb/memb.c                                        \
	  uIP/apps/dhcpc/dhcpc.c                                      \
      $(LUFA_PATH)/LUFA/Platform/UC3/InterruptManagement.c        \
      $(LUFA_SRC_USB)                                             \
      $(LUFA_SRC_USBCLASS)

      ##uIP/uip/uip-neighbor.c                                      \ needed only if UIP_CONF_IP6 is used
	  ##uIP/apps/udpds/udpds.c                                      \
	  ##uIP/uip/psock.c                                             \
	  ##uIP/apps/telnetd/shell.c                                    \
	  ##uIP/apps/telnetd/telnetd.c                                  \
	  ##adOS/adOS.c                                                 \
	  ##uIP/apps/hello-world/hello-world.c                          \
      ##enc28j60/ip_arp_udp_tcp.c
      ##enc28j60/enc28j60.c
      ##enc28j60/websrv_help_functions.c


#Comment out the line below if Xilinx bitstream and programming will not be included (disables programming routines etc.)
TX8MXILINX = PRESENT
#Xilinx Bitstream
XSRC = ../../../../../engineering/designs/tx8m/Xilinx/tx8m.bit

# List C++ source files here. (C dependencies are automatically generated.)
CPPSRC =


# List Assembler source files here.
#     Make them always end in a capital .S.  Files ending in a lowercase .s
#     will not be considered source files but generated files (assembler
#     output from the compiler), and will be deleted upon "make clean"!
#     Even though the DOS/Win* filesystem matches both .s and .S the same,
#     it will preserve the spelling of the filenames, and gcc itself does
#     care about how the name is spelled on its command-line.
ASRC = $(LUFA_PATH)/LUFA/Platform/UC3/Exception.S
#ASRC = asf/avr32/drivers/intc/exception.S



# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)

# note, that no functions having inline definition, but no ATTR_ALWAYS_INLINE set for them, will be inlined unless optimization is enabled.
# See: http://gcc.gnu.org/ml/gcc-help/2007-01/msg00051.html
# Alternative is to specify -funit-at-a-time compiler option.
OPT = 1


# Debugging level.
DEBUG = 3


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS = $(LUFA_PATH)/  \
               $(ASF_PATHS)/  \
               $(UIP_PATHS)/  \


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99


# Place -D or -U options here for C sources
CDEFS  = -DF_CPU=$(F_CPU)UL
CDEFS += -DF_USB=$(F_USB)UL
CDEFS += -DF_ADC=$(F_ADC)UL
CDEFS += -DF_PBA_SPEED=$(F_PBA_SPEED)UL
CDEFS += -DFOSC0=$(FOSC0)UL
CDEFS += -DBOARD=BOARD_$(BOARD)
CDEFS += -DARCH=ARCH_$(ARCH)
CDEFS += $(LUFA_OPTS)
CDEFS += $(UIP_OPTS)


# Place -D or -U options here for ASM sources
ADEFS  = -DF_CPU=$(F_CPU)UL
ADEFS += -DF_USB=$(F_USB)UL
ADEFS += -DF_ADC=$(F_ADC)UL
ADEFS += -DF_PBA_SPEED=$(F_PBA_SPEED)UL
ADEFS += -DFOSC0=$(FOSC0)UL
ADEFS += -DBOARD=BOARD_$(BOARD)
ADEFS += -DARCH=ARCH_$(ARCH)
ADEFS += $(LUFA_OPTS)
ADEFS += $(UIP_OPTS)


# Place -D or -U options here for C++ sources
CPPDEFS  = -DF_CPU=$(F_CPU)UL
CPPDEFS += -DF_USB=$(F_USB)UL
CPPDEFS += -DF_ADC=$(F_ADC)UL
CPPDEFS += -DF_PBA_SPEED=$(F_PBA_SPEED)UL
CPPDEFS += -DFOSC0=$(FOSC0)UL
CPPDEFS += -DBOARD=BOARD_$(BOARD)
CPPDEFS += -DARCH=ARCH_$(ARCH)
CPPDEFS += $(LUFA_OPTS)
CPPDEFS += $(UIP_OPTS)
#CPPDEFS += -D__STDC_LIMIT_MACROS
#CPPDEFS += -D__STDC_CONSTANT_MACROS



#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -ffunction-sections
CFLAGS += -fno-strict-aliasing
CFLAGS += -masm-addr-pseudos
CFLAGS += -Wall
CFLAGS += -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:%.c=$(OBJDIR)/%.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)
#CFLAGS += -fgnu89-inline
#-funit-at-a-time enables inlining by inline keyword only even when no optimization is enabled
CFLAGS += -funit-at-a-time
#CFLAGS += -fdata-sections


#---------------- Compiler Options C++ ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CPPFLAGS = -g$(DEBUG)
CPPFLAGS += $(CPPDEFS)
CPPFLAGS += -O$(OPT)
CPPFLAGS += -funsigned-char
CPPFLAGS += -funsigned-bitfields
CPPFLAGS += -ffunction-sections
CPPFLAGS += -fno-strict-aliasing
CPPFLAGS += -fno-exceptions
CPPFLAGS += -masm-addr-pseudos
CPPFLAGS += -Wall
CPPFLAGS += -Wundef
CPPFLAGS += -Wa,-adhlns=$(<:%.cpp=$(OBJDIR)/%.lst)
CPPFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
#CPPFLAGS += $(CSTANDARD)
#CPPFLAGS += -fgnu89-inline


#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adhlns:   create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex
#       dump that will be displayed for a given single line of source input.
ASFLAGS = -g3 $(ADEFS)


#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS  = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections --rodata-writable
LDFLAGS += -Wl,--direct-data
LDFLAGS += -T avr32elf_uc3b0512.x


#============================================================================


# Define programs and commands.
SHELL = sh
CC = avr32-gcc
OBJCOPY = avr32-objcopy
OBJDUMP = avr32-objdump
SIZE = avr32-size
AR = avr32-ar rcs
NM = avr32-nm
REMOVE = rm -f
REMOVEDIR = rm -rf
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
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_CPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:




# Define all object files.
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o) $(XSRC:%.bit=$(OBJDIR)/%.o)

# Define all listing files.
LST = $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst)


# Compiler flags to generate dependency files.
GENDEPFLAGS = -MMD -MP -MF .dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mpart=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_CPPFLAGS = -mpart=$(MCU) -I. -x c++ $(CPPFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mpart=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)





# Default target.
all: begin gccversion build end

# Change the build target to build a HEX file or a library.
build: elf hex lss sym
#build: lib


elf: $(TARGET).elf
hex: $(TARGET).hex
lss: $(TARGET).lss
sym: $(TARGET).sym
LIBNAME=lib$(TARGET).a
lib: $(LIBNAME)



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
ELFSIZE = $(SIZE) $(MCU_FLAG) $(FORMAT_FLAG) $(TARGET).elf
MCU_FLAG = $(shell $(SIZE) --help | grep -- --mcu > nul && echo --mcu=$(MCU) )
FORMAT_FLAG = $(shell $(SIZE) --help | grep -- --format=.*avr > nul && echo --format=avr )


#sizebefore:
#	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
#	2>nul; echo; fi

#sizeafter:
#	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
#	2>nul; echo; fi



# Display compiler version information.
gccversion :
	@$(CC) --version


# Program the device.
flip: $(TARGET).hex
	batchisp -hardware usb -device at32$(MCU) -operation erase f
	batchisp -hardware usb -device at32$(MCU) -operation loadbuffer $(TARGET).hex program
	batchisp -hardware usb -device at32$(MCU) -operation start reset 0

dfu: $(TARGET).hex
	dfu-programmer $(MCU) erase
	dfu-programmer $(MCU) flash $(TARGET).hex
	dfu-programmer $(MCU) reset


# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse -R .lock -R .signature $< $@

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



# Create library from object files.
.SECONDARY : $(TARGET).a
.PRECIOUS : $(OBJ)
%.a: $(OBJ)
	@echo
	@echo $(MSG_CREATING_LIBRARY) $@
	$(AR) $@ $(OBJ)


# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
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

# Convert Xilinx .bit file to object code
$(OBJDIR)/%.o : %.bit
	@echo Converting Xilinx Bitstream $<
	$(OBJCOPY) -v -w -I binary -O elf32-avr32 -B avr --rename-section .data=.text \
	--redefine-sym _binary________________engineering_designs_tx8m_Xilinx_tx8m_bit_start=fpga_bitstream_start \
    --redefine-sym _binary________________engineering_designs_tx8m_Xilinx_tx8m_bit_end=fpga_bitstream_end \
    --redefine-sym _binary________________engineering_designs_tx8m_Xilinx_tx8m_bit_size=fpga_bitstream_size_sym \
	$< $@
	$(NM) $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@


# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).cof
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o)
	$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRC:.c=.i)
	$(REMOVEDIR) .dep

doxygen:
	@echo Generating Project Documentation...
	@doxygen Doxygen.conf
	@echo Documentation Generation Complete.

clean_doxygen:
	rm -rf Documentation

checksource:
	@for f in $(SRC) $(CPPSRC) $(ASRC); do \
	    if [ -f $$f ]; then \
	        echo "Found Source File: $$f" ; \
	    else \
	        echo "Source File Not Found: $$f" ; \
	    fi; done


# Create object files directory
$(shell mkdir $(OBJDIR) 2>nul)


# Include the dependency files.
-include $(shell mkdir .dep 2>nul) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex lss sym doxygen clean clean_list clean_doxygen  \
dfu flip checksource

