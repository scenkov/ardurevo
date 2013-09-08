
#ifndef __AP_HAL_VRBRAIN_SCHEDULER_H__
#define __AP_HAL_VRBRAIN_SCHEDULER_H__

#include <AP_HAL_VRBRAIN.h>
#include <delay.h>
#include <systick.h>
#include <boards.h>
#include <timer.h>

#define VRBRAIN_SCHEDULER_MAX_TIMER_PROCS 10


class VRBRAIN::VRBRAINScheduler : public AP_HAL::Scheduler {
public:
    VRBRAINScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::TimedProc);
	void 	 register_io_process(AP_HAL::TimedProc proc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::TimedProc, uint32_t period_us);

    bool     system_initializing();
    void     system_initialized();

    void     panic(const prog_char_t *errormsg);
    void     reboot(bool hold_in_bootloader);

private:

    static volatile bool _in_timer_proc;

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    bool _initialized;

    /* _timer_isr_event() and _run_timer_procs are static so they can be
     * called from an interrupt. */
    static void _timer_isr_event();
    static void _run_timer_procs(bool called_from_isr);

    static AP_HAL::TimedProc _failsafe;

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::TimedProc _timer_proc[VRBRAIN_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;

};

#endif // __AP_HAL_VRBRAIN_SCHEDULER_H__
