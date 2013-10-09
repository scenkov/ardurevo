
#ifndef __AP_HAL_VRBRAIN_NAMESPACE_H__
#define __AP_HAL_VRBRAIN_NAMESPACE_H__

/* While not strictly required, names inside the VRBRAIN namespace are prefixed
 * with VRBRAIN for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace VRBRAIN {
    class VRBRAINUARTDriver;
    class VRBRAINI2CDriver;
    class VRBRAINSPIDeviceManager;
    class VRBRAINSPI1DeviceDriver;
    class VRBRAINSPI2DeviceDriver;
    class VRBRAINSPI3DeviceDriver;
    class VRBRAINAnalogSource;
    class VRBRAINAnalogIn;
    class VRBRAINStorage;
    class VRBRAINGPIO;
    class VRBRAINDigitalSource;
    class VRBRAINRCInput;
    class VRBRAINRCOutput;
    class VRBRAINSemaphore;
    class VRBRAINScheduler;
    class VRBRAINUtil;
}

#endif // __AP_HAL_VRBRAIN_NAMESPACE_H__

