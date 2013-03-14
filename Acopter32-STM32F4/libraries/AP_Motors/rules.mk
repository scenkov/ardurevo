# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)

# Local flags
CFLAGS_$(d) := -Wall -Werror

# Local rules and targets
cSRCS_$(d) :=

cppSRCS_$(d) :=
cppSRCS_$(d) += AP_Motors_Class.cpp
cppSRCS_$(d) += AP_MotorsHeli.cpp
cppSRCS_$(d) += AP_MotorsHexa.cpp
cppSRCS_$(d) += AP_MotorsMatrix.cpp
cppSRCS_$(d) += AP_MotorsOcta.cpp
cppSRCS_$(d) += AP_MotorsOctaQuad.cpp
cppSRCS_$(d) += AP_MotorsQuad.cpp
cppSRCS_$(d) += AP_MotorsTri.cpp
cppSRCS_$(d) += AP_MotorsY6.cpp

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