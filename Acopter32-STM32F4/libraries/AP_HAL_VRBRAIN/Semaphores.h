
#ifndef __AP_HAL_VRBRAIN_SEMAPHORES_H__
#define __AP_HAL_VRBRAIN_SEMAPHORES_H__

#include <AP_HAL_VRBRAIN.h>

class VRBRAIN::VRBRAINSemaphore : public AP_HAL::Semaphore {
public:
    VRBRAINSemaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
protected:
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();

    volatile bool _taken;
};

#endif // __AP_HAL_VRBRAIN_SEMAPHORES_H__
