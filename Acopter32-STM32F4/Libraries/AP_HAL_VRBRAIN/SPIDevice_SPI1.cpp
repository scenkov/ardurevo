#include <AP_HAL.h>


#include <AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52

VRBRAINSemaphore VRBRAINSPI1DeviceDriver::_semaphore;

static volatile bool spi1_transferflag = false;

void VRBRAINSPI1DeviceDriver::init() {

}

AP_HAL::Semaphore* VRBRAINSPI1DeviceDriver::get_semaphore() {
    return &_semaphore;
}

inline void VRBRAINSPI1DeviceDriver::_cs_assert() {

}

inline void VRBRAINSPI1DeviceDriver::_cs_release() {
    _cs_pin->write(1);
}

inline uint8_t VRBRAINSPI1DeviceDriver::_transfer(uint8_t data) {

}

void VRBRAINSPI1DeviceDriver::transfer(const uint8_t *tx, uint16_t len) {

}

void VRBRAINSPI1DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) {

}

void VRBRAINSPI1DeviceDriver::cs_assert() {

}

void VRBRAINSPI1DeviceDriver::cs_release() {

}

uint8_t VRBRAINSPI1DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}


