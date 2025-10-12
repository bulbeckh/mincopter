
#include <arch/arm/stm32/Scheduler.h>

using namespace stm32;

extern const AP_HAL::HAL& hal;

TIM_HandleTypeDef STM32Scheduler::timer_handle;
TIM_HandleTypeDef STM32Scheduler::delay_handle;

// Initialise the milli-second counter to 0 NOTE Might already be zero-init as a global
uint32_t STM32Scheduler::_ms_counter{0};

STM32Scheduler::STM32Scheduler()
{}

void STM32Scheduler::init(void* machtnichts)
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

    STM32Scheduler::timer_handle.Instance = TIM2;
    STM32Scheduler::timer_handle.Init.Prescaler = 8399;   // (84 MHz / (8399 + 1)) = 10 kHz
    STM32Scheduler::timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    STM32Scheduler::timer_handle.Init.Period = 9;      // 10 kHz / (9 + 1) = 1kHz (1 ms)
    STM32Scheduler::timer_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Base_Init(&STM32Scheduler::timer_handle);

    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Setup TIM1 - micro-second delay timer
	//
	// TIM1 runs on APB1 (84MHz) so we use a 84x prescaler to generate a 1us increment
    STM32Scheduler::delay_handle.Instance = TIM1;
    STM32Scheduler::delay_handle.Init.Prescaler = 83;   // (84 MHz / (83 + 1)) = 1MHz
    STM32Scheduler::delay_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    STM32Scheduler::delay_handle.Init.Period = 0x2710;  // 10,000us or 10ms we reset
    STM32Scheduler::delay_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Base_Init(&STM32Scheduler::delay_handle);

	// TODO Maybe we should wait until elsewhere (i.e. the call to Scheduler::initalised() before we start timers
	
	HAL_TIM_Base_Start_IT(&STM32Scheduler::timer_handle);
	HAL_TIM_Base_Start_IT(&STM32Scheduler::delay_handle);

	return;
}

// Implementation of TIM2 overflow will route to STM32 HAL method
extern "C" {

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&STM32Scheduler::timer_handle);
}

}

// HAL Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* This callback is called at 1kHz (1ms) and hence can be used to increment the ms_counter */
	++STM32Scheduler::_ms_counter;

	// TODO Should be checking if this timer is the TIM2 (Scheduler) timer as other timers use this cb function
	
	// Run the timer processes
	if (htim->Instance == TIM2) {
		STM32Scheduler::_run_timer_processes();
	}

	return;
}

void STM32Scheduler::_run_timer_processes(void)
{
	// TODO
	

	// For now just run the heartbeat
	STM32Scheduler::_timer_led_heartbeat();
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
	
	uint32_t start = __HAL_TIM_GET_COUNTER(&STM32Scheduler::delay_handle);

	while((__HAL_TIM_GET_COUNTER(&STM32Scheduler::delay_handle) - start) < us) {
		// 
	}

	return;
}

uint32_t STM32Scheduler::millis(void)
{
	return STM32Scheduler::_ms_counter;
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
	uint32_t us_ticks = __HAL_TIM_GET_COUNTER(&STM32Scheduler::delay_handle);
	return STM32Scheduler::_ms_counter*1000lu + us_ticks%10;
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

		hal.uartA->write((uint8_t*)"heartbeat\r\n",11);
	}
	led_counter++;

	return;
}


