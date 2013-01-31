
#ifndef __AP_HAL_VRBRAIN_SPI_DEVICES_H__
#define __AP_HAL_VRBRAIN_SPI_DEVICES_H__

#include <AP_HAL.h>
#include "AP_HAL_VRBRAIN_Namespace.h"

class VRBRAIN::VRBRAINSPI1DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    VRBRAINSPI1DeviceDriver(
            VRBRAIN::VRBRAINDigitalSource *cs_pin,
            uint8_t spcr,
            uint8_t spsr
    ) :
        _cs_pin(cs_pin),
        _spcr(spcr),
        _spsr(spsr)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);

    static VRBRAIN::VRBRAINSemaphore _semaphore;

    VRBRAIN::VRBRAINDigitalSource *_cs_pin;
    const uint8_t _spcr;
    const uint8_t _spsr;
};


class VRBRAIN::VRBRAINSPI2DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    VRBRAINSPI2DeviceDriver(
            VRBRAIN::VRBRAINDigitalSource *cs_pin,
            uint8_t ucsr2c,
            uint16_t ubrr2
    ) :
        _cs_pin(cs_pin),
        _ucsr2c(ucsr2c),
        _ubrr2(ubrr2)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);

    static VRBRAIN::VRBRAINSemaphore _semaphore;

    VRBRAIN::VRBRAINDigitalSource *_cs_pin;
    uint8_t _ucsr2c;
    uint16_t _ubrr2;
};



#endif // __AP_HAL_VRBRAIN_SPI_DEVICES_H__
