
#include "AnalogIn.h"

using namespace VRBRAIN;

VRBRAINAnalogSource::VRBRAINAnalogSource(float v) :
    _v(v)
{}

float VRBRAINAnalogSource::read_average() {
    return _v;
}

float VRBRAINAnalogSource::read_latest() {
    return _v;
}
float VRBRAINAnalogSource::voltage_average_ratiometric(){
    return _v;
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
void VRBRAINAnalogSource::set_pin(uint8_t p)
{}


VRBRAINAnalogIn::VRBRAINAnalogIn()
{}

void VRBRAINAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t n) {
    return new VRBRAINAnalogSource(1.11);
}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t n, float scale) {
    return new VRBRAINAnalogSource(scale/2);
}

