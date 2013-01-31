/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>


#include <AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

VRBRAINSemaphore VRBRAINSPI2DeviceDriver::_semaphore;

void VRBRAINSPI2DeviceDriver::init() {

}

AP_HAL::Semaphore* VRBRAINSPI2DeviceDriver::get_semaphore() {
    return &_semaphore;
}

inline void VRBRAINSPI2DeviceDriver::_cs_assert() {

}

inline void VRBRAINSPI2DeviceDriver::_cs_release() {
    _cs_pin->write(1);
}

inline uint8_t VRBRAINSPI2DeviceDriver::_transfer(uint8_t data) {

}

void VRBRAINSPI2DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) {

}

void VRBRAINSPI2DeviceDriver::cs_assert() {
    _cs_assert();
}

void VRBRAINSPI2DeviceDriver::cs_release() {
    _cs_release();
}

uint8_t VRBRAINSPI2DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}

void VRBRAINSPI2DeviceDriver::transfer(const uint8_t *data, uint16_t len) {
    while (len--)
        _transfer(*data++);
}


