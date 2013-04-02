
#include "AnalogIn.h"

using namespace VRBRAIN;

VRBRAINAnalogSource::VRBRAINAnalogSource(uint8_t pin, float prescale) :
    _pin(pin),
    _prescale(prescale)
{}

float VRBRAINAnalogSource::read_average() {
	float fullscale = analogRead(_pin);
	float scaled = _prescale * fullscale;
	return scaled;
}

float VRBRAINAnalogSource::read_latest() {
	float fullscale = analogRead(_pin);
	float scaled = _prescale * fullscale;
	return scaled;
}

/*
  return voltage in Volts
 */
float VRBRAINAnalogSource::voltage_average()
{
    return (5.0f/4096.0f) * read_average();
}
void VRBRAINAnalogSource::set_pin(uint8_t p)
{
    _pin = p;
}


VRBRAINAnalogIn::VRBRAINAnalogIn()
{}

void VRBRAINAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t n) {
    return this->channel(n, 0.25);
}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t n, float scale) {
    return this->channel(n, scale/2);
}

