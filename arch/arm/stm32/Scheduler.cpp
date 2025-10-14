
#include <arch/arm/stm32/Scheduler.h>

#include <stdio.h>

using namespace stm32;

extern const AP_HAL::HAL& hal;

STM32Scheduler::STM32Scheduler() :
	_initialised{false},
	_ms_counter{0},
	_suspended{false},
	_num_timer_procs{0},
	_in_timer_proc{false}

{ }

void STM32Scheduler::init(void* /* unused */)
{
	/* We are using two timers for STM32Scheduler
	 *
	 * Timer 2 (TIM2) is used to run the 1kHz processes that are registered via the register_* functions.
	 *
	 * Timer 1 (TIM1) is used for micro-second delays.
	 *
	 */

	// TODO By default we are using TIM2 here but we should maybe make it configurable
	__HAL_RCC_TIM2_CLK_ENABLE();

	__HAL_RCC_TIM1_CLK_ENABLE();

	// Our timer currently needs to interrupt at 1kHz
	//
	// TIM2 runs on APB1 which for us is clocked at 84MHz

    timer_handle.Instance = TIM2;
    timer_handle.Init.Prescaler = 8399;   // (84 MHz / (8399 + 1)) = 10 kHz
    timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer_handle.Init.Period = 9;      // 10 kHz / (9 + 1) = 1kHz (1 ms)
    timer_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Base_Init(&timer_handle);

    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Setup TIM1 - micro-second delay timer
	//
	// TIM1 runs on APB2 (168MHz) so we use a 84x prescaler to generate a 1us increment
    delay_handle.Instance = TIM1;
    delay_handle.Init.Prescaler = 167;   // (168 MHz / (167 + 1)) = 1MHz
    delay_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    delay_handle.Init.Period = 0x2710;  // 10,000us or 10ms we reset
    delay_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Base_Init(&delay_handle);

	// TODO Maybe we should wait until elsewhere (i.e. the call to Scheduler::initalised() before we start timers
	
	HAL_TIM_Base_Start_IT(&timer_handle);
	HAL_TIM_Base_Start_IT(&delay_handle);

	return;
}


void STM32Scheduler::_run_timer_processes(void)
{
	// Return immediately if we have suspended timer processes
	if (_suspended) return;

	// Set our timer process flag
	_in_timer_proc = true;

	// Run each timer process
	for (uint8_t i=0; i<_num_timer_procs;i++) {
		_timer_proc[i]();
	}

	// For now just run the heartbeat
	_timer_led_heartbeat();

	// Reset our timer process flag
	_in_timer_proc = false;

	return;
}

void STM32Scheduler::delay(uint16_t ms)
{
	// For now, just use the HAL_Delay function
	HAL_Delay(ms);
	return;
}

void STM32Scheduler::delay_microseconds(uint16_t us)
{
	// TODO Add a check that this function is only used for <1000us delays
	
	// Uses the delay timer (TIM1) which ticks at microseconds
	
	uint32_t start = __HAL_TIM_GET_COUNTER(&delay_handle);

	volatile uint8_t _temp = 0;

	while((__HAL_TIM_GET_COUNTER(&delay_handle) - start) < us) {
		_temp +=1;
	}

	return;
}

uint32_t STM32Scheduler::millis(void)
{
	return _ms_counter;
}

uint32_t STM32Scheduler::micros(void)
{
	/* To get the number of elapsed microseconds, we can use the current _ms_counter as a
	 * base and then use the microsecond delay timer (TIM1) for the number of elapsed microseconds.
	 *
	 * The microsecond delay timer ticks at 1us and resets every 10ms so we just need to get the number of microseconds
	 * elapsed since the last millisecond interval (us_ticks%10) and add to the _ms_counter. 
	 *
	 */
	uint32_t us_ticks = __HAL_TIM_GET_COUNTER(&delay_handle);
	return _ms_counter*1000lu + us_ticks%1000;
}

void STM32Scheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
	// TODO
	return;
}

void STM32Scheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
	if (_num_timer_procs >= STM32_SCHEDULER_MAX_TIMER_PROCS) {
		// TODO Notify somewhere of error
		return;
	}

	// Add bound member process to next available slot
	_timer_proc[_num_timer_procs] = proc;

	// Increment timer process counter
	_num_timer_procs += 1;

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

void STM32Scheduler::suspend_timer_procs(void)
{
	_suspended = true;
	return;
}

void STM32Scheduler::resume_timer_procs(void)
{
	_suspended = false;

	return;
}

void STM32Scheduler::panic(const prog_char_t *errormsg) 
{
	// Dump error message to console
	hal.console->printf(PSTR("%s\n"), errormsg);

	// TODO Maybe call one of the provided endless loops like Default_Handler
	// Endless loop
	while(true) { }
}

bool STM32Scheduler::in_timerprocess(void)
{
	return _in_timer_proc;
}

bool STM32Scheduler::system_initializing(void)
{
	return _initialised;
}

void STM32Scheduler::system_initialized(void)
{
	if (_initialised) {
		// We have already initialised
		// TODO call hal panic
	} else {
		_initialised = true;
		return;
	}
}

void STM32Scheduler::reboot(bool /* unused */) 
{
	// Trigger a software reset using NVIC
	HAL_NVIC_SystemReset();
}

void STM32Scheduler::time_shift(uint32_t shift_ms)
{
	// TODO
	return;
}

void STM32Scheduler::_timer_led_heartbeat(void)
{
	/* This timer callback runs at 1kHz (at the timer frequency)
	 * and every second we toggle the heartbeat LED and then send
	 * a heartbeat message on uartA (console). */

	// TODO Will wrap at UINT32_MAX and won't be exactly 1000
	static uint32_t led_counter=0;

	if (led_counter%1000==0) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		// Get number of milliseconds and format into string
		uint32_t ms_elapsed = millis();

		uint8_t s_buffer[128];
		uint32_t char_written = snprintf((char*)s_buffer, sizeof(s_buffer), "MS%lu\r\n", ms_elapsed);
		hal.uartA->write((uint8_t*)s_buffer, char_written);

		delay_microseconds(500);

		uint32_t us_elapsed = micros();
		uint32_t counter = __HAL_TIM_GET_COUNTER(&delay_handle);

		char_written = snprintf((char*)s_buffer, sizeof(s_buffer), "US%lu,%lu\r\n", us_elapsed, counter);
		hal.uartA->write((uint8_t*)s_buffer, char_written);

		hal.uartA->write((uint8_t*)"heartbeat\r\n",11);
	}
	led_counter++;

	return;
}


