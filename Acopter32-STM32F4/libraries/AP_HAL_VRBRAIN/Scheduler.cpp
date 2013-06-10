
#include "Scheduler.h"
#include <delay.h>
#include <timer.h>
#include <HardwareTimer.h>
#include <systick.h>

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

AP_HAL::TimedProc VRBRAINScheduler::_failsafe = NULL;
volatile bool VRBRAINScheduler::_timer_suspended = false;
volatile bool VRBRAINScheduler::_timer_event_missed = false;
volatile bool VRBRAINScheduler::_in_timer_proc = false;
AP_HAL::TimedProc VRBRAINScheduler::_timer_proc[VRBRAIN_SCHEDULER_MAX_TIMER_PROCS] = {NULL};
uint8_t VRBRAINScheduler::_num_timer_procs = 0;


VRBRAINScheduler::VRBRAINScheduler()
:
	    _delay_cb(NULL),
	    _min_delay_cb_ms(65535),
	    _initialized(false)
{}

void VRBRAINScheduler::init(void* machtnichts)
{

    timer_pause(TIMER5);
    timer_set_prescaler(TIMER5,41);
    timer_set_count(TIMER5,0);
    timer_set_reload(TIMER5,999);
    timer_attach_interrupt(TIMER5, TIMER_UPDATE_INTERRUPT, _timer_isr_event);
    timer_resume(TIMER5);

    //systick_attach_callback(_timer_isr_event);
}

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

void VRBRAINScheduler::register_delay_callback(AP_HAL::Proc proc,
            uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void VRBRAINScheduler::register_timer_process(AP_HAL::TimedProc proc)
{
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < VRBRAIN_SCHEDULER_MAX_TIMER_PROCS) {
        /* this write to _timer_proc can be outside the critical section
         * because that memory won't be used until _num_timer_procs is
         * incremented. */
        _timer_proc[_num_timer_procs] = proc;
        /* _num_timer_procs is used from interrupt, and multiple bytes long. */
        noInterrupts();
        _num_timer_procs++;
        interrupts();
    }
}

void VRBRAINScheduler::register_io_process(AP_HAL::TimedProc proc) 
{
    // IO processes not supported on AVR
}

void VRBRAINScheduler::register_timer_failsafe(AP_HAL::TimedProc,
            uint32_t period_us)
{}
void VRBRAINScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}


void VRBRAINScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timer_procs(false);
        _timer_event_missed = false;
    }
}

bool VRBRAINScheduler::in_timerprocess()
{
    return _in_timer_proc;
}



void VRBRAINScheduler::_timer_isr_event() {
    // we enable the interrupt again immediately and also enable
    // interrupts. This allows other time critical interrupts to
    // run (such as the serial receive interrupt). We catch the
    // timer calls taking too long using _in_timer_call.
    // This approach also gives us a nice uniform spacing between
    // timer calls

    //TCNT2 = RESET_TCNT2_VALUE;
    interrupts();
    _run_timer_procs(true);
}

void VRBRAINScheduler::_run_timer_procs(bool called_from_isr) {

    uint32_t tnow = hal.scheduler->micros();
    if (_in_timer_proc) {
        // the timer calls took longer than the period of the
        // timer. This is bad, and may indicate a serious
        // driver failure. We can't just call the drivers
        // again, as we could run out of stack. So we only
        // call the _failsafe call. It's job is to detect if
        // the drivers or the main loop are indeed dead and to
        // activate whatever failsafe it thinks may help if
        // need be.  We assume the failsafe code can't
        // block. If it does then we will recurse and die when
        // we run out of stack
        if (_failsafe != NULL) {
            _failsafe(tnow);
        }
        return;
    }

    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i](tnow);
            }
        }
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe(tnow);
    }

    _in_timer_proc = false;
}



bool VRBRAINScheduler::system_initializing()
{
    return !_initialized;
}

void VRBRAINScheduler::system_initialized()
{
    if (_initialized) {
        panic(PSTR("PANIC: scheduler::system_initialized called"
                   "more than once"));
    }
    _initialized = true;
}

void VRBRAINScheduler::panic(const prog_char_t* errormsg) {
   hal.console->println_P(errormsg);
    for(;;);
}

void VRBRAINScheduler::reboot() {
    for(;;);
}
