
#ifndef __AP_HAL_VRBRAIN_UTIL_H__
#define __AP_HAL_VRBRAIN_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_VRBRAIN_Namespace.h"

class VRBRAIN::VRBRAINUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};

#endif // __AP_HAL_VRBRAIN_UTIL_H__
