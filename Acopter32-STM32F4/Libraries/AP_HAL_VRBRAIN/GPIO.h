
#ifndef __AP_HAL_VRBRAIN_GPIO_H__
#define __AP_HAL_VRBRAIN_GPIO_H__

#include <AP_HAL_VRBRAIN.h>
#include <io.h>
class VRBRAIN::VRBRAINGPIO : public AP_HAL::GPIO {
public:
    VRBRAINGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

};

class VRBRAIN::VRBRAINDigitalSource : public AP_HAL::DigitalSource {
public:
    VRBRAINDigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
private:
    uint8_t _v;
};

#endif // __AP_HAL_VRBRAIN_GPIO_H__
