#include <AP_HAL.h>

#include <AP_HAL.h>
#include "SPIDriver.h"
#include "SPIDevices.h"

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

void VRBRAINSPIDeviceManager::init(void* machtnichts) {

    /* Note that the order of the init() of the MS5611 and MPU6k is
     * critical for the APM2. If you initialise in the wrong order
     * then the MS5611 doesn't initialise itself correctly. This
     * indicates an electrical fault in the APM2 which needs to be
     * investigated. Meanwhile, initialising the MPU6k CS pin before
     * the MS5611 CS pin works around the problem
     */

}

AP_HAL::SPIDeviceDriver* VRBRAINSPIDeviceManager::device(enum AP_HAL::SPIDevice d)
{

}
