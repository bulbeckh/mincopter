
#include <AP_HAL/AP_HAL.h>

#include <arch/linux/generic/Scheduler.h>

// HASH include <unistd.h>
// HASH include <sys/time.h>
// HASH include <poll.h>
// HASH include <unistd.h>
#include <stdlib.h>
// HASH include <stdio.h>
// HASH include <errno.h>
// HASH include <sys/mman.h>

using namespace generic;

extern const AP_HAL::HAL& hal;

GenericScheduler::GenericScheduler(void)
{
}

void GenericScheduler::init(void* /* unused */ )
{
	// No init really required 
	
	return;
}

void GenericScheduler::delay(uint16_t ms)
{
	// We don't need to delay in simulation - we should just ignore if it is called
	return;
}

uint32_t GenericScheduler::millis(void)
{
	// Retrieve simulation timestamp from the last sim packet. Timestamp
	// is in seconds so multiply by 1000 to get to ms
	return 1000*(uint32_t)hal.sim->last_sensor_state.timestamp;
}

uint32_t GenericScheduler::micros(void)
{
	// We don't have a conception of a micro-second in the simulated environment so we just return the millisecond multiplied by 1000
	return 1e6*(uint32_t)hal.sim->last_sensor_state.timestamp;
}

void GenericScheduler::delay_microseconds(uint16_t us)
{
	// Don't need to delay in simulation
	return;
}

void GenericScheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
	// Ignore delay callbacks in simulation - any IO is handled immediately
	hal.console->printf("[Scheduler] Ignoring delay callback registration\r\n");
	return;
}

void GenericScheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
	// No timer processes in simulation as MinCopter steps at 100Hz anyway. Default
	// timer processes run at 1kHz.
	hal.console->printf("[Scheduler] Ignoring timer process registration\r\n");
	return;
}

void GenericScheduler::register_io_process(AP_HAL::MemberProc proc) 
{
	// Ignore io callbacks in simulation
	hal.console->printf("[Scheduler] Ignoring IO callback registration\r\n");
	return;
}

void GenericScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
	// TODO Should we be ignoring this - how should we implement failsafe in simulation??
	// Ignore failsafe callback in simulation
	hal.console->printf("[Scheduler] Ignoring failsafe callback registration\r\n");
	return;
}

void GenericScheduler::suspend_timer_procs(void)
{
    _timer_suspended = true;
	return;
}

void GenericScheduler::resume_timer_procs(void)
{
    _timer_suspended = false;
	return;
}

void GenericScheduler::panic(const prog_char_t *errormsg) 
{
	hal.console->printf(errormsg);
    hal.scheduler->delay_microseconds(10000);
    exit(1);
}

bool GenericScheduler::in_timerprocess(void)
{
	// In the simulation, we use a single-thread so in_timerprocess will always return false (unless it
	// is called from within a timer process function, but that shouldn't happen)
	
	return false;
}

bool GenericScheduler::system_initializing(void)
{
    return !_initialized;
}

void GenericScheduler::system_initialized(void)
{
    if (_initialized) {
        panic("PANIC: scheduler::system_initialized called more than once");
    }
    _initialized = true;
}

void GenericScheduler::reboot(bool /* unused */) 
{
	// Unsure how to handle this but for now just exit
	hal.console->printf("[Scheduler] Reboot requested. Exiting instead..\r\n");
	exit(1);
}

