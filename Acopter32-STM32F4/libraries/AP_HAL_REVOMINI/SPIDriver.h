
#ifndef __AP_HAL_REVOMINI_SPIDRIVER_H__
#define __AP_HAL_REVOMINI_SPIDRIVER_H__

#include <AP_HAL_REVOMINI.h>
#include "AP_HAL_REVOMINI_Namespace.h"
#include "GPIO.h"
#include "SPIDevices.h"
#include "Semaphores.h"
#include <spi.h>



class REVOMINI::REVOMINISPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *machtnichts);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice d);
private:    
    REVOMINISPI1DeviceDriver* _mpu6k;

    //REVOMINISPI3DeviceDriver* _rfm22b;
    REVOMINISPI3DeviceDriver* _dataflash;
};

#endif // __AP_HAL_REVOMINI_SPIDRIVER_H__
