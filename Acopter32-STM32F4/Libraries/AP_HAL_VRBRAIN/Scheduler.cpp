
#include "Scheduler.h"

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

VRBRAINScheduler::VRBRAINScheduler()
{}

void VRBRAINScheduler::init(void* machtnichts)
{}

void VRBRAINScheduler::delay(uint16_t ms)
{}

uint32_t VRBRAINScheduler::millis() {
    return 10000;
}

uint32_t VRBRAINScheduler::micros() {
    return 200000;
}

void VRBRAINScheduler::delay_microseconds(uint16_t us)
{}

void VRBRAINScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void VRBRAINScheduler::register_timer_process(AP_HAL::TimedProc k)
{}

void VRBRAINScheduler::register_timer_failsafe(AP_HAL::TimedProc,
            uint32_t period_us)
{}

void VRBRAINScheduler::suspend_timer_procs()
{}

void VRBRAINScheduler::resume_timer_procs()
{}

void VRBRAINScheduler::begin_atomic()
{}

void VRBRAINScheduler::end_atomic()
{}

void VRBRAINScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

void VRBRAINScheduler::reboot() {
    for(;;);
}
