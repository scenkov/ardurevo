
#include "AnalogIn.h"
#include <adc.h>
#include <boards.h>
#include <gpio_hal.h>

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;

VRBRAINAnalogSource::VRBRAINAnalogSource(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _last_value(initial_value)
{
    if ((_pin < 0) || (_pin >= BOARD_NR_GPIO_PINS)) {
            _pin = 200;
        }

    //gpio_set_mode(PIN_MAP[_pin].gpio_device, PIN_MAP[_pin].gpio_bit, GPIO_INPUT_ANALOG);

    hal.gpio->pinMode(_pin, INPUT_ANALOG);
}

float VRBRAINAnalogSource::read_average() {
    float temp;
    temp = (read_latest() * 0.8) + (_last_value * 0.2f);
    return temp;
}

float VRBRAINAnalogSource::read_latest() {
    if ((_pin < 0) || (_pin >= BOARD_NR_GPIO_PINS)) {
            return 0.0;
        }

    const adc_dev *dev = PIN_MAP[_pin].adc_device;
    if (dev == NULL) {
        return 0.0;
    }
    _value = adc_read(dev, PIN_MAP[_pin].adc_channel);
    return _value;
}
float VRBRAINAnalogSource::voltage_average_ratiometric(){
    return _value;
}
void VRBRAINAnalogSource::set_stop_pin(uint8_t p){}
void VRBRAINAnalogSource::set_settle_time(uint16_t settle_time_ms){}
/*
  return voltage in Volts
 */
float VRBRAINAnalogSource::voltage_average()
{
    return (5.0f/4096.0f) * read_average();
}
void VRBRAINAnalogSource::set_pin(uint8_t pin)
{
    if(pin == _pin)
	return;
    _pin = pin;
}


VRBRAINAnalogIn::VRBRAINAnalogIn()
{}

void VRBRAINAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t pin) {
    //if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
    //        return NULL;
    //    }
    return new VRBRAINAnalogSource(pin, 0.0);
}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t pin, float scale) {
    //if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
    //        return NULL;
    //    }
    return new VRBRAINAnalogSource(pin, scale/2);
}

