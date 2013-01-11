
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
    class VRBRAINSPIDeviceDriver;
    class VRBRAINAnalogSource;
    class VRBRAINAnalogIn;
    class VRBRAINStorage;
    class VRBRAINConsoleDriver;
    class VRBRAINGPIO;
    class VRBRAINDigitalSource;
    class VRBRAINRCInput;
    class VRBRAINRCOutput;
    class VRBRAINSemaphore;
    class VRBRAINScheduler;
    class VRBRAINUtil;
    class VRBRAINPrivateMember;
}

#endif // __AP_HAL_VRBRAIN_NAMESPACE_H__

