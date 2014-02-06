BOARD_TYPE     := 9
BOARD_REVISION := 3

# Useful tools
CC       := arm-none-eabi-gcc
CXX      := arm-none-eabi-g++
LD       := arm-none-eabi-ld -v
AR       := arm-none-eabi-ar
AS       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
DISAS    := arm-none-eabi-objdump
OBJDUMP  := arm-none-eabi-objdump
SIZE     := arm-none-eabi-size
DFU      := dfu-util
AT       := @
ECHO	 := echo
CAT      := cat
OPENOCD_WRAPPER  := support/scripts/openocd-wrapper.sh

# Suppress annoying output unless V is set
ifndef V
   SILENT_CC       = @echo '  [CC]       ' $(@:$(BUILD_PATH)/%.o=%.c);
   SILENT_AS       = @echo '  [AS]       ' $(@:$(BUILD_PATH)/%.o=%.S);
   SILENT_CXX      = @echo '  [CXX]      ' $(@:$(BUILD_PATH)/%.o=%.cpp);
   SILENT_LD       = @echo '  [LD]       ' $(@F);
   SILENT_AR       = @echo '  [AR]       '
   SILENT_OBJCOPY  = @echo '  [OBJCOPY]  ' $(@F);
   SILENT_DISAS    = @echo '  [DISAS]    ' $(@:$(BUILD_PATH)/%.bin=%).disas;
   SILENT_OBJDUMP  = @echo '  [OBJDUMP]  ' $(OBJDUMP);
   MSG_FWINFO      = @echo '  [FWINFO]   ' 
   MSG_OPFIRMWARE  = @echo '  [OPFW]     '
   MSG_LOAD_FILE   = @echo '  [BIN/HEX]  '
   MSG_COMPILING   = @echo '  [CC]       '
   V0              =
   V1              = $(AT)
endif

BUILDDIRS :=
TGT_BIN   :=

CFLAGS   = $(GLOBAL_CFLAGS) $(TGT_CFLAGS)
CXXFLAGS = $(GLOBAL_CXXFLAGS) $(TGT_CXXFLAGS)
ASFLAGS  = $(GLOBAL_ASFLAGS) $(TGT_ASFLAGS)
	
THUMB   = -mthumb
CSTANDARD = -std=gnu99
CONLYFLAGS += $(CSTANDARD)

# Command to extract version info data from the repository and source tree
export VERSION_INFO = python $(SRCROOT)/libraries/$(AP_HAL)/support/scripts/version-info.py --path=$(SRCROOT)

toprel = $(subst $(realpath $(SRCROOT))/,,$(abspath $(1)))

# General directory independent build rules, generate dependency information
$(BUILD_PATH)/%.o: %.c
	$(SILENT_CC) $(CC) $(CFLAGS) $(LIBRARY_INCLUDES) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(BUILD_PATH)/%.o: %.cpp
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBRARY_INCLUDES) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(BUILD_PATH)/%.o: %.S
	$(SILENT_AS) $(AS) $(ASFLAGS) $(LIBRARY_INCLUDES) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<
	
%.bin: %.o
	$(MSG_LOAD_FILE) $(call toprel, $@)
	$(V1) $(OBJCOPY) -O binary $< $@

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(BUILD_PATH)/$(notdir $(basename $(1))).o : $(1)
	$(MSG_COMPILING) $$(call toprel, $$<)
	$(V1) $(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef

# OpenPilot firmware image template
#  $(1) = path to bin file
#  $(2) = boardtype in hex
#  $(3) = board revision in hex
define OPFW_TEMPLATE
FORCE:

$(1).firmware_info.c: $(1) $(SRCROOT)/libraries/$(AP_HAL)/support/templates/firmware_info.c.template FORCE
	$(MSG_FWINFO) $$(call toprel, $$@)
	$(V1) $(VERSION_INFO) \
		--template=$(SRCROOT)/libraries/$(AP_HAL)/support/templates/firmware_info.c.template \
		--outfile=$$@ \
		--image=$(1) \
		--type=$(2) \
		--revision=$(3) \
		--uavodir=$(SRCROOT)/libraries/$(AP_HAL)/support/templates
		
$(eval $(call COMPILE_C_TEMPLATE, $(1).firmware_info.c))

$(BUILD_PATH)/$(notdir $(basename $(1))).opfw : $(1) $(1).firmware_info.bin
	$(MSG_OPFIRMWARE) $$(call toprel, $$@)
	$(V1) $(CAT) $(1) $(1).firmware_info.bin > $$@
endef
	
$(eval $(call OPFW_TEMPLATE,$(BUILD_PATH)/$(BOARD).bin,$(BOARD_TYPE),$(BOARD_REVISION)))	
