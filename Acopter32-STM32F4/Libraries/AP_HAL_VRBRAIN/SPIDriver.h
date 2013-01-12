/*
 * SPIDriver.h --- AP_HAL_VRBRAIN SPI driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __AP_HAL_VRBRAIN_SPIDRIVER_H__
#define __AP_HAL_VRBRAIN_SPIDRIVER_H__

#include <AP_HAL_VRBRAIN.h>
#include "Semaphores.h"

#include <wirish.h>

class VRBRAIN::VRBRAINSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    VRBRAINSPIDeviceDriver(spi_bus *bus, spi_device *device);
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
private:
    VRBRAINSemaphore _semaphore;
    struct spi_bus *_bus;
    struct spi_device *_device;
};

class VRBRAIN::VRBRAINSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    VRBRAINSPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(AP_HAL::SPIDevice);
};

#endif // __AP_HAL_VRBRAIN_SPIDRIVER_H__
