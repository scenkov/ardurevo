# Try "make help" first

.DEFAULT_GOAL := all

##
## Target-specific configuration.  This determines some compiler and
## linker options/flags.
##

# Try "make help" for more information on BOARD and MEMORY_TARGET;
# these default to a Maple Flash build.
BOARD            ?= laserlab_MP32V1F4
MEMORY_TARGET    ?= flash

##
## Useful paths, constants, etc.
##

SRCROOT := .
BUILD_PATH       := $(SRCROOT)/build

HARDWARE_PATH    := $(SRCROOT)/hardware
HAL_PATH         := $(SRCROOT)/hardware/hal
STM32_PATH       := $(HARDWARE_PATH)/STM32F4xx_DSP_StdPeriph_Lib_V1.0.0
cpu_flags        := -mcpu=cortex-m4
WIRISH_PATH      := $(SRCROOT)/wirish
LIBRARIES_PATH   := $(SRCROOT)/Libraries
FIRMWARE_PATH    := $(SRCROOT)/Firmware/Acopter32

SKETCH          := ACopter32
SKETCHPDESRCS   := $(wildcard $(FIRMWARE_PATH)/*.pde)
SKETCHSRCS      := $(wildcard $(addprefix $(SRCROOT)/,$(SRCSUFFIXES)))
SKETCHPDE       := $(wildcard $(FIRMWARE_PATH)/$(SKETCH).pde)
SKETCHCPP       := $(FIRMWARE_PATH)/$(SKETCH).cpp

# List of input files to the sketch.cpp file in the order they should
# be appended to create it
SKETCHCPP_SRC   := $(SKETCHPDE) $(sort $(filter-out $(SKETCHPDE),$(SKETCHPDESRCS)))


LIBRARY_INCLUDES := 

SUPPORT_PATH     := $(SRCROOT)/support
# Support files for linker
LDDIR            := $(SUPPORT_PATH)/ld
# Support files for this Makefile
MAKEDIR          := $(SUPPORT_PATH)/make

# $(BOARD)- and $(MEMORY_TARGET)-specific configuration
include $(MAKEDIR)/target-config.mk

##
## Compilation flags
##

GLOBAL_FLAGS    := -D$(VECT_BASE_ADDR)
GLOBAL_FLAGS    += -DBOARD_$(BOARD)
GLOBAL_FLAGS    += -DMCU_$(MCU)
GLOBAL_FLAGS    += -DSTM32F4XX
GLOBAL_FLAGS    += -DUSE_STDPERIPH_DRIVER
GLOBAL_FLAGS    += -DHSE_VALUE=8000000
GLOBAL_FLAGS    += -DARM_MATH_CM4
GLOBAL_FLAGS    += -D__FPU_PRESENT
GLOBAL_FLAGS    += -D__FPU_USED=1
# GLOBAL_CFLAGS -----------------------------------------------------------------------------------
GLOBAL_CFLAGS   := $(cpu_flags)
GLOBAL_CFLAGS   += -mthumb             #Generate code for the Thumb instruction set
GLOBAL_CFLAGS   += -ggdb                  #Produce debugging information in the operating system’s native format
#GLOBAL_CFLAGS   += -O0                 #Reduce compilation time and make debugging produce the expected results. This is the default
#GLOBAL_CFLAGS   += -nostdlib           #Do not use the standard system startup files or libraries when linking
GLOBAL_CFLAGS   += -Wall               #This enables all the warnings about constructions that some users consider questionable, and that are easy to avoid (or modify to prevent the warning), even in conjunction with macros
GLOBAL_CFLAGS   += -ffunction-sections
GLOBAL_CFLAGS   += -fdata-sections
GLOBAL_CFLAGS   += -mfpu=fpv4-sp-d16
GLOBAL_CFLAGS   += -mfloat-abi=softfp
GLOBAL_CFLAGS   += $(GLOBAL_FLAGS)
# GLOBAL_CXXFLAGS ---------------------------------------------------------------------------------
#GLOBAL_CXXFLAGS := -fno-rtti       #Disable generation of information about every class with virtual functions for use by the C++ runtime type identification features
#GLOBAL_CXXFLAGS += -fno-exceptions
GLOBAL_CXXFLAGS += -Wall
GLOBAL_CXXFLAGS += -fpermissive    #Downgrade some diagnostics about nonconformant code from errors to warnings. Thus, using ‘-fpermissive’ will allow some nonconforming code to compile.
GLOBAL_CXXFLAGS += -Wno-psabi
GLOBAL_CXXFLAGS += $(GLOBAL_FLAGS)
# GLOBAL_ASFLAGS ----------------------------------------------------------------------------------
GLOBAL_ASFLAGS  := $(cpu_flags)
GLOBAL_ASFLAGS  += -mthumb
GLOBAL_CFLAGS   += -ggdb                 #Produce debugging information in the operating system’s native format
#GLOBAL_CFLAGS   += -O0                #Reduce compilation time and make debugging produce the expected results. This is the default
GLOBAL_ASFLAGS  += -x assembler-with-cpp
GLOBAL_ASFLAGS  += $(GLOBAL_FLAGS)
# GLOBAL_LDFLAGS ----------------------------------------------------------------------------------
LDFLAGS         := $(cpu_flags)
LDFLAGS         += -mthumb
LDFLAGS         += -static
LDFLAGS         += -Wall
LDFLAGS         += -Wl,--gc-sections
LDFLAGS         += -Wl,--cref
LDFLAGS         += -Wl,-Map,$(BUILD_PATH)/$(BOARD).map
#LDFLAGS         += -Xlinker --gc-sections --print-gc-sections
LDFLAGS         += -T$(LDDIR)/$(LDSCRIPT)
LDFLAGS         += -L$(LDDIR)

##
## Build rules and useful templates
##

include $(SUPPORT_PATH)/make/build-rules.mk
include $(SUPPORT_PATH)/make/build-templates.mk

##
## Set all submodules here
##

# Try to keep LIBMAPLE_MODULES a simply-expanded variable
# Official libraries:
LIBRARY_MODULES := $(STM32_PATH)
LIBRARY_MODULES += $(HAL_PATH)
LIBRARY_MODULES += $(WIRISH_PATH)

LIBRARY_MODULES += $(LIBRARIES_PATH)/AC_PID
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_ADC
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_AHRS
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_AnalogSource
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Airspeed
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Baro
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Buffer
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Common
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Compass
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Declination
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_GPS
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Camera
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Limits
#LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_GSM
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_InertialSensor
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_InertialNav
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Math
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Menu
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Motors
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Mount
#LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Navigation
#LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_OpticalFlow
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_PeriodicProcess
#LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_PID
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_RangeFinder
#LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_RC_Channel
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_Relay
LIBRARY_MODULES += $(LIBRARIES_PATH)/APM_PerfMon
LIBRARY_MODULES += $(LIBRARIES_PATH)/APM_PI
LIBRARY_MODULES += $(LIBRARIES_PATH)/APM_RC
LIBRARY_MODULES += $(LIBRARIES_PATH)/Arduino_Mega_ISR_Registry
LIBRARY_MODULES += $(LIBRARIES_PATH)/DataFlash
LIBRARY_MODULES += $(LIBRARIES_PATH)/EEPROM
LIBRARY_MODULES += $(LIBRARIES_PATH)/Filter
LIBRARY_MODULES += $(LIBRARIES_PATH)/GCS_Mavlink
LIBRARY_MODULES += $(LIBRARIES_PATH)/AP_LeadFilter
#LIBRARY_MODULES += $(LIBRARIES_PATH)/GPS_IMU
#LIBRARY_MODULES += $(LIBRARIES_PATH)/mapleSDfat
#LIBRARY_MODULES += $(LIBRARIES_PATH)/ME_Armed
LIBRARY_MODULES += $(LIBRARIES_PATH)/memcheck
#LIBRARY_MODULES += $(LIBRARIES_PATH)/MP32_WNK
#LIBRARY_MODULES += $(LIBRARIES_PATH)/PID
LIBRARY_MODULES += $(LIBRARIES_PATH)/RC_Channel
#LIBRARY_MODULES += $(LIBRARIES_PATH)/Trig_LUT
#LIBRARY_MODULES += $(LIBRARIES_PATH)/Waypoints

# Firmware libraries:
LIBRARY_MODULES += $(FIRMWARE_PATH)

# Call each module's rules.mk:
$(foreach m,$(LIBRARY_MODULES),$(eval $(call LIBRARY_MODULE_template,$(m))))

##
## Targets
##

# main target
include $(SRCROOT)/build-targets.mk

.PHONY: all clean

# Force a rebuild if the target changed
PREV_BUILD_TYPE = $(shell cat $(BUILD_PATH)/build-type 2>/dev/null)
build-check:
ifneq ($(PREV_BUILD_TYPE), $(MEMORY_TARGET))
    $(shell rm -rf $(BUILD_PATH))
endif

#all: build-check MSG_INFO $(BUILD_PATH)/$(BOARD).bin
all: build-check MSG_INFO cleansketch $(SKETCHCPP) $(BUILD_PATH)/$(BOARD).bin

cleansketch:
	rm -f $(SKETCHCPP)
	rm -f main.cpp

clean:	cleansketch
	rm -rf build

AWK := awk
CAT := cat

$(SKETCHCPP):	$(SKETCHCPP_SRC)
	$(AWK) -v mode=header -f SKETCH_SPLITTER     $(SKETCHCPP_SRC) >  $@
#	echo "#line 1 \"autogenerated\""                              >> $@
#	echo "#if defined(ARDUINO) && ARDUINO >= 100"                 >> $@
#	echo "#include \"Arduino.h\""                                 >> $@
#	echo "#else"                                                  >> $@
#	echo "#include \"WProgram.h\""                                >> $@
#	echo "#endif"                                                 >> $@
	$(AWK)                -f SKETCH_PROTOTYPER $(SKETCHCPP_SRC)   >> $@
	$(AWK) -v mode=body   -f SKETCH_SPLITTER   $(SKETCHCPP_SRC)   >> $@
	$(CAT) $@  $(WIRISH_PATH)/main.cxx > main.cpp
	#rm -f $(SKETCHCPP)
	
#
# The prototype scanner is an awk script used to generate function
# prototypes from the concantenated .pde files.
#
# Function definitions are expected to follow the form
#
#   <newline><type>[<qualifier>...]<name>([<arguments>]){
#
# with whitespace permitted between the various elements.  The pattern
# is assembled from separate subpatterns to make adjustments easier.
#
# Note that $ requires special treatment here to avoid upsetting make,
# and backslashes are doubled in the partial patterns to satisfy
# escaping rules.
#
# This script requires BWK or GNU awk.
#
define SKETCH_PROTOTYPER
  BEGIN {                                                               \
    RS="{";                                                             \
    type       = "((\\n)|(^))[[:space:]]*[[:alnum:]_]+[[:space:]]+";    \
    qualifiers = "([[:alnum:]_\\*&]+[[:space:]]*)*";                    \
    name       = "[[:alnum:]_]+[[:space:]]*";                           \
    args       = "\\([[:space:][:alnum:]_,&\\*\\[\\]]*\\)";             \
    bodycuddle = "[[:space:]]*$$";                                      \
    pattern    = type qualifiers name args bodycuddle;                  \
  }                                                                     \
  match($$0, pattern) {                                                 \
    proto = substr($$0, RSTART, RLENGTH);                               \
    gsub("\n", " ", proto);                                             \
    printf "%s;\n", proto;                                              \
  }
endef
