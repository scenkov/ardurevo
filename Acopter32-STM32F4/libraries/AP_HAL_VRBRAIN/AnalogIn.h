
#ifndef __AP_HAL_VRBRAIN_ANALOGIN_H__
#define __AP_HAL_VRBRAIN_ANALOGIN_H__

#include <wirish.h>
#include <AP_HAL_VRBRAIN.h>

class VRBRAIN::VRBRAINAnalogSource : public AP_HAL::AnalogSource {
public:
    VRBRAINAnalogSource(uint8_t pin, float prescale = 1.0);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
private:
    /* following three are used from both an interrupt and normal thread */
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;
    volatile uint16_t _latest;
    float _last_average;

    /* _pin designates the ADC input mux for the sample */
    uint8_t _pin;
    /* prescale scales the raw measurments for read()*/
    const float _prescale;
};

class VRBRAIN::VRBRAINAnalogIn : public AP_HAL::AnalogIn {
public:
    VRBRAINAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t pin);
    AP_HAL::AnalogSource* channel(int16_t pin, float scale);
};
#endif // __AP_HAL_VRBRAIN_ANALOGIN_H__
