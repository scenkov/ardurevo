
#include "Scheduler.h"
#include <delay.h>

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

VRBRAINScheduler::VRBRAINScheduler()
{}

void VRBRAINScheduler::init(void* machtnichts)
{}

void VRBRAINScheduler::delay(uint16_t ms)
{
    uint32 i;
    for (i = 0; i < ms; i++) {
	delay_microseconds(1000);
    }

}

uint32_t VRBRAINScheduler::millis() {
    return systick_uptime();
}

uint32_t VRBRAINScheduler::micros() {
    uint32 ms;
    uint32 cycle_cnt;

    do {
        ms = millis();
        cycle_cnt = systick_get_count();
    } while (ms != millis());

#define US_PER_MS               1000
    /* SYSTICK_RELOAD_VAL is 1 less than the number of cycles it
     * actually takes to complete a SysTick reload */
    return ((ms * US_PER_MS) +
            (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND);
#undef US_PER_MS
}

void VRBRAINScheduler::delay_microseconds(uint16_t us)
{
    delay_us((uint32_t)us);
}

void VRBRAINScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void VRBRAINScheduler::register_timer_process(AP_HAL::TimedProc k)
{}
void VRBRAINScheduler::suspend_timer_procs()
{}

void VRBRAINScheduler::resume_timer_procs()
{}

bool VRBRAINScheduler::in_timerprocess()
{}

void VRBRAINScheduler::register_timer_failsafe(AP_HAL::TimedProc,
            uint32_t period_us)
{}



bool VRBRAINScheduler::system_initializing()
{}

void VRBRAINScheduler::system_initialized()
{}

void VRBRAINScheduler::panic(const prog_char_t *errormsg) {
   hal.console->println_P(errormsg);
    for(;;);
}

void VRBRAINScheduler::reboot() {
    for(;;);
}
