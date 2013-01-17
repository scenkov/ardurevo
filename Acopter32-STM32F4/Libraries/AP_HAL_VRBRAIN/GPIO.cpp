
#include "GPIO.h"

using namespace VRBRAIN;

VRBRAINGPIO::VRBRAINGPIO()
{
}

void VRBRAINGPIO::init()
{
}

void VRBRAINGPIO::pinMode(uint8_t pin, uint8_t output)
{
}

uint8_t VRBRAINGPIO::read(uint8_t pin)
{
  return 0;
}

void VRBRAINGPIO::write(uint8_t pin, uint8_t value)
{
}

/* Alternative interface: */
AP_HAL::DigitalSource* VRBRAINGPIO::channel(uint16_t n) {
    return new VRBRAINDigitalSource(0);
}

/* Interrupt interface: */
bool VRBRAINGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                  uint8_t mode)
{
  return true;
}

VRBRAINDigitalSource::VRBRAINDigitalSource(uint8_t v) :
  _v(v)
{
}

void VRBRAINDigitalSource::mode(uint8_t output)
{
}

uint8_t VRBRAINDigitalSource::read()
{
  return _v;
}

void VRBRAINDigitalSource::write(uint8_t value)
{
  _v = value;
}
