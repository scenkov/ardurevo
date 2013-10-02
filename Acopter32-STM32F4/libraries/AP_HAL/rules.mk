# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)
BUILDDIRS += $(BUILD_PATH)/$(d)/utility

# Local flags
CFLAGS_$(d) := 

# Local rules and targets
cSRCS_$(d) :=

cppSRCS_$(d) :=
cppSRCS_$(d) += utility/Print.cpp
cppSRCS_$(d) += utility/ftoa_engine.cpp
cppSRCS_$(d) += utility/print_vprintf.cpp
cppSRCS_$(d) += utility/utoa_invert.cpp
cppSRCS_$(d) += Console.cpp
cppSRCS_$(d) += Util.cpp
cppSRCS_$(d) += UARTDriver.cpp

cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)

OBJS_$(d) := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o) \
             $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
DEPS_$(d) := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include $(DEPS_$(d))
d := $(dirstack_$(sp))
sp := $(basename $(sp))