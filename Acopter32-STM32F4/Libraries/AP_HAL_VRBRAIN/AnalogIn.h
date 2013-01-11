
#ifndef __AP_HAL_VRBRAIN_ANALOGIN_H__
#define __AP_HAL_VRBRAIN_ANALOGIN_H__

#include <AP_HAL_VRBRAIN.h>

class VRBRAIN::VRBRAINAnalogSource : public AP_HAL::AnalogSource {
public:
    VRBRAINAnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
private:
    float _v;
};

class VRBRAIN::VRBRAINAnalogIn : public AP_HAL::AnalogIn {
public:
    VRBRAINAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
    AP_HAL::AnalogSource* channel(int16_t n, float scale);
};
#endif // __AP_HAL_VRBRAIN_ANALOGIN_H__
