
#ifndef __AP_HAL_VRBRAIN_SPIDRIVER_H__
#define __AP_HAL_VRBRAIN_SPIDRIVER_H__

#include <AP_HAL_VRBRAIN.h>
#include "AP_HAL_VRBRAIN_Namespace.h"
#include "GPIO.h"
#include "SPIDevices.h"
#include "Semaphores.h"
#include <spi.h>



class VRBRAIN::VRBRAINSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *machtnichts);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice d);
private:    
    VRBRAINSPI2DeviceDriver* _mpu6k;

    VRBRAINSPI3DeviceDriver* _mpu6k_ext;

    VRBRAINSPI1DeviceDriver* _ms5611;
    VRBRAINSPI1DeviceDriver* _dataflash;

};

#endif // __AP_HAL_VRBRAIN_SPIDRIVER_H__
