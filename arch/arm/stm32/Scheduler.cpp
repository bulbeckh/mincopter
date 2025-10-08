
#include <arch/arm/stm32/Scheduler.h>

using namespace stm32;

extern const AP_HAL::HAL& hal;

STM32Scheduler::STM32Scheduler()
{}

void STM32Scheduler::init(void* machtnichts)
{
	// TODO
	return;
}

void STM32Scheduler::delay(uint16_t ms)
{
	// TODO
	return;
}

uint32_t STM32Scheduler::millis() 
{
	// TODO
	return 0;
}

uint32_t STM32Scheduler::micros() 
{
	// TODO
	return 0;
}

void STM32Scheduler::delay_microseconds(uint16_t us)
{
	// TODO
	return;
}

void STM32Scheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
	// TODO
	return;
}

void STM32Scheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
	// TODO
	return;
}

void STM32Scheduler::register_io_process(AP_HAL::MemberProc proc) 
{
	// TODO
	return;
}

void STM32Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
	// TODO
	return;
}

void STM32Scheduler::suspend_timer_procs()
{
	// TODO
	return;
}

void STM32Scheduler::resume_timer_procs()
{
	// TODO
	return;
}

void STM32Scheduler::panic(const prog_char_t *errormsg) 
{
	// TODO
	return;
}

bool STM32Scheduler::in_timerprocess() 
{
	// TODO
	return false;
}

void STM32Scheduler::begin_atomic()
{
	// TODO
	return;
}

void STM32Scheduler::end_atomic()
{
	// TODO
	return;
}

bool STM32Scheduler::system_initializing() {
	// TODO
	return false;
}

void STM32Scheduler::system_initialized()
{
	// TODO
	return;
}

void STM32Scheduler::reboot(bool hold_in_bootloader) 
{
	// TODO
	return;
}

void STM32Scheduler::time_shift(uint32_t shift_ms)
{
	// TODO
	return;
}

