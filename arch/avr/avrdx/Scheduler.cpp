/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "avrdx/Scheduler.h"
#include "avrdx/utility/ISRRegistry.h"
#include "avrdx/memcheck.h"

using namespace AP_HAL_AVRDx;

extern const AP_HAL::HAL& hal;

/* AVRScheduler timer interrupt period is controlled by TCNT2.
 * 256-124 gives a 500Hz period
 * 256-62 gives a 1kHz period. */
volatile uint8_t AVRDxScheduler::_timer2_reset_value = (256 - 62);

/* Static AVRScheduler variables: */
AVRDxTimer AVRDxScheduler::_timer;

AP_HAL::Proc AVRDxScheduler::_failsafe = NULL;
volatile bool AVRDxScheduler::_timer_suspended = false;
volatile bool AVRDxScheduler::_timer_event_missed = false;
volatile bool AVRDxScheduler::_in_timer_proc = false;

AP_HAL::MemberProc AVRDxScheduler::_timer_proc[AVR_SCHEDULER_MAX_TIMER_PROCS]; /* = {NULL}; */
uint8_t AVRDxScheduler::_num_timer_procs = 0;


AVRDxScheduler::AVRDxScheduler() :
    _delay_cb(NULL),
    _min_delay_cb_ms(65535),
    _initialized(false)
{}

void AVRDxScheduler::init(void* _isrregistry) {
	return;
	// TODO
	/*
    ISRRegistry* isrregistry = (ISRRegistry*) _isrregistry;

    // _timer: sets up timer hardware to implement millis & micros.
    _timer.init();

    / TIMER2: Setup the overflow interrupt to occur at 1khz.
    TIMSK2 = 0;                     // Disable timer interrupt 
    TCCR2A = 0;                     // Normal counting mode 
    TCCR2B = _BV(CS21) | _BV(CS22); // Prescaler to clk/256 
    TCNT2 = 0;                      // Set count to 0 
    TIFR2 = _BV(TOV2);              // Clear pending interrupts 
    TIMSK2 = _BV(TOIE2);            // Enable overflow interrupt

    // Register _timer_isr_event to trigger on overflow
    isrregistry->register_signal(ISR_REGISTRY_TIMER2_OVF, _timer_isr_event);   
    
    // Turn on global interrupt flag, AVR interupt system will start from this point
    sei();

    memcheck_init();
	*/
}

uint32_t AVRDxScheduler::micros() {
    return _timer.micros();
}

uint32_t AVRDxScheduler::millis() {
    return _timer.millis();
}

void AVRDxScheduler::delay_microseconds(uint16_t us) {
    _timer.delay_microseconds(us);
}

void AVRDxScheduler::delay(uint16_t ms)
{
	uint32_t start = _timer.micros();
    
    while (ms > 0) {
        while ((_timer.micros() - start) >= 1000) {
            ms--;
            if (ms == 0) break;
            start += 1000;
        }
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

void AVRDxScheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void AVRDxScheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
	return;
	// TODO
	/*
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < AVR_SCHEDULER_MAX_TIMER_PROCS) {
        // this write to _timer_proc can be outside the critical section
		// because that memory won't be used until _num_timer_procs is
		// incremented
        _timer_proc[_num_timer_procs] = proc;
        // _num_timer_procs is used from interrupt, and multiple bytes long
        uint8_t sreg = SREG;
        cli();
        _num_timer_procs++;
        SREG = sreg;        
    }
	*/
}

void AVRDxScheduler::register_io_process(AP_HAL::MemberProc proc) 
{
    // IO processes not supported on AVR
}

void AVRDxScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) {
    /* XXX Assert period_us == 1000 */
    _failsafe = failsafe;
}

void AVRDxScheduler::suspend_timer_procs() {
    _timer_suspended = true;
}

void AVRDxScheduler::resume_timer_procs() {
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timer_procs(false);
        _timer_event_missed = false;
    }
}

bool AVRDxScheduler::in_timerprocess() {
    return _in_timer_proc;
}

void AVRDxScheduler::_timer_isr_event() {
	return;
	// TODO

    // we enable the interrupt again immediately and also enable
    // interrupts. This allows other time critical interrupts to
    // run (such as the serial receive interrupt). We catch the
    // timer calls taking too long using _in_timer_call.
    // This approach also gives us a nice uniform spacing between
    // timer calls

	/*
    TCNT2 = _timer2_reset_value;
    sei();
    _run_timer_procs(true);
	*/
}

void AVRDxScheduler::_run_timer_procs(bool called_from_isr) {
	return;
	// TODO
	/*
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
            _failsafe();
        }
        return;
    }

    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }

    _in_timer_proc = false;
	*/
}

bool AVRDxScheduler::system_initializing() {
    return !_initialized;
}

void AVRDxScheduler::system_initialized() {
    if (_initialized) {
        panic(PSTR("PANIC: scheduler::system_initialized called"
                   "more than once"));
    }
    _initialized = true;
}

void AVRDxScheduler::panic(const prog_char_t* errormsg) {
    /* Suspend timer processes. We still want the timer event to go off
     * to run the _failsafe code, however. */
    _timer_suspended = true;
    /* Print the error message on both ports */
    hal.uartA->println_P(errormsg);
    hal.uartC->println_P(errormsg);
    /* Spin forever. */
    for(;;);
}

void AVRDxScheduler::reboot(bool hold_in_bootloader) {
	return;
	// TODO
	/*
    hal.uartA->println_P(PSTR("GOING DOWN FOR A REBOOT\r\n"));
    hal.scheduler->delay(100);
    // The APM2 bootloader will reset the watchdog shortly after
	// starting, so we can use the watchdog to force a reboot
	
    cli();
    wdt_enable(WDTO_15MS);
    for(;;);
	*/
}

/**
   set timer speed in Hz. Used by ArduCopter on APM2 to reduce the
   cost of timer interrupts
 */
void AVRDxScheduler::set_timer_speed(uint16_t timer_hz)
{
    if (timer_hz > 1000) {
        timer_hz = 1000;
    }
    if (timer_hz < 250) {
        timer_hz = 250;
    }
    _timer2_reset_value = 256 - (62 * (1000 / timer_hz));
}

