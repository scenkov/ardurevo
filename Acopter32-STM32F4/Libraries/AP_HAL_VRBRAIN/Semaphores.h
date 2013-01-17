
#ifndef __AP_HAL_VRBRAIN_SEMAPHORE_H__
#define __AP_HAL_VRBRAIN_SEMAPHORE_H__

#include <AP_HAL_VRBRAIN.h>

class VRBRAIN::VRBRAINSemaphore : public AP_HAL::Semaphore {
public:
    VRBRAINSemaphore() : _taken(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    bool _taken;
};

#endif // __AP_HAL_VRBRAIN_SEMAPHORE_H__
