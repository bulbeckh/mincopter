
#include <arch/arm/stm32/Scheduler.h>

using namespace stm32;

extern const AP_HAL::HAL& hal;

STM32Scheduler::STM32Scheduler()
{}

void STM32Scheduler::init(void* machtnichts)
{
	// TODO By default we are using TIM2 here but we should maybe make it configurable
	
	__HAL_RCC_TIM2_CLK_ENABLE();

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

	return;
}

// Implementation of TIM2 overflow will route to STM32 HAL method
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&STM32Scheduler::timer_handle);
}

// HAL Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Run the timer processes
	STM32Scheduler::_run_timer_processes();

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

void _timer_led_heartbeat(void)
{
	// Runs at 1kHz
	static uint16_t led_counter=0;

	if (led_counter%1000) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		led_counter=0;
	} else {
		led_counter++;
	}

	return;
}


