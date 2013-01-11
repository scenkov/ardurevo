
#ifndef __AP_HAL_VRBRAIN_RCINPUT_H__
#define __AP_HAL_VRBRAIN_RCINPUT_H__

#include <AP_HAL_VRBRAIN.h>

class VRBRAIN::VRBRAINRCInput : public AP_HAL::RCInput {
public:
    VRBRAINRCInput();
    void init(void* machtnichts);
    uint8_t  valid();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
};

#endif // __AP_HAL_VRBRAIN_RCINPUT_H__
