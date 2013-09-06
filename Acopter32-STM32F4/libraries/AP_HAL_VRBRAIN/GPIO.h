
#ifndef __AP_HAL_VRBRAIN_GPIO_H__
#define __AP_HAL_VRBRAIN_GPIO_H__

#include <AP_HAL.h>
#include "AP_HAL_VRBRAIN_Namespace.h"
#include "gpio_hal.h"

#ifndef HIGH
 #define HIGH 0x1
#endif

#ifndef LOW
 #define LOW  0x0
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
 # define HAL_GPIO_A_LED_PIN        19
 # define HAL_GPIO_B_LED_PIN        20
 # define HAL_GPIO_C_LED_PIN        21
 # define HAL_GPIO_LED_ON           HIGH
 # define HAL_GPIO_LED_OFF          LOW
#endif

class VRBRAIN::VRBRAINGPIO : public AP_HAL::GPIO {
public:
    VRBRAINGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

};

class VRBRAIN::VRBRAINDigitalSource : public AP_HAL::DigitalSource {
public:
    VRBRAINDigitalSource(gpio_dev *device, uint8_t bit): _device(device), _bit(bit){}
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    gpio_dev *_device;
    uint8_t _bit;
};




#endif // __AP_HAL_VRBRAIN_GPIO_H__
