
#ifndef __AP_HAL_VRBRAIN_SCHEDULER_H__
#define __AP_HAL_VRBRAIN_SCHEDULER_H__

#include <AP_HAL_VRBRAIN.h>

class VRBRAIN::VRBRAINScheduler : public AP_HAL::Scheduler {
public:
    VRBRAINScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);
    void     register_timer_process(AP_HAL::TimedProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();
    
    void     register_timer_failsafe(AP_HAL::TimedProc,
                        uint32_t period_us);

    bool     system_initializing();
    void     system_initialized();

    void     panic(const prog_char_t *errormsg);
    void     reboot();

};

#endif // __AP_HAL_VRBRAIN_SCHEDULER_H__
