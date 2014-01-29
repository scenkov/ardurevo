
#ifndef __AP_HAL_VRBRAIN_CLASS_H__
#define __AP_HAL_VRBRAIN_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_HAL_VRBRAIN_Namespace.h"
#include <wirish.h>
#include <hal.h>

class HAL_VRBRAIN : public AP_HAL::HAL {
public:
    HAL_VRBRAIN();
    void init(int argc, char * const argv[]) const;
};

extern uint8_t g_ext_mag_detect;

extern const HAL_VRBRAIN AP_HAL_VRBRAIN;

#endif // __AP_HAL_VRBRAIN_CLASS_H__
#endif // __HAL_BOARD_VRBRAIN__
