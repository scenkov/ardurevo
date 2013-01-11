
#ifndef __AP_HAL_VRBRAIN_SEMAPHORE_H__
#define __AP_HAL_VRBRAIN_SEMAPHORE_H__

#include <AP_HAL_VRBRAIN.h>
#include <FreeRTOS.h>
#include <semphr.h>

class VRBRAIN::VRBRAINSemaphore : public AP_HAL::Semaphore {
public:
    VRBRAINSemaphore();

    void init();
    virtual bool take(uint32_t timeout_ms);
    virtual bool take_nonblocking();
    virtual bool give();

private:
    xSemaphoreHandle m_semaphore;
};

#endif // __AP_HAL_VRBRAIN_SEMAPHORE_H__
