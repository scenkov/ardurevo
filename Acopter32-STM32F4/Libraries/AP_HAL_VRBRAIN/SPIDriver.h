
#ifndef __AP_HAL_VRBRAIN_SPIDRIVER_H__
#define __AP_HAL_VRBRAIN_SPIDRIVER_H__

#include <AP_HAL_VRBRAIN.h>
#include "Semaphores.h"

class VRBRAIN::VRBRAINSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    VRBRAINSPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    VRBRAINSemaphore _semaphore;
};

class VRBRAIN::VRBRAINSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    VRBRAINSPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);
private:    VRBRAINSPIDeviceDriver _device;};

#endif // __AP_HAL_VRBRAIN_SPIDRIVER_H__
