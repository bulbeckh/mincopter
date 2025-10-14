
#include <arch/arm/stm32/RCOutput.h>

#include "stm32f4xx_hal.h"

using namespace stm32;

void STM32RCOutput::init(void* machtnichts)
{
	/* Will use PC6-PC9 as the four PWM channels (TIM3_CH1 - TIM3_CH4) */
}

void STM32RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
	// TODO Implement different output channel frequencies
	
	return;
}

uint16_t STM32RCOutput::get_freq(uint8_t ch) {
	// TODO
	return 50;
}

void STM32RCOutput::enable_ch(uint8_t ch)
{
	// TODO
}

void STM32RCOutput::disable_ch(uint8_t ch)
{
	// TODO
}

void STM32RCOutput::write(uint8_t ch, uint16_t period_us)
{
	// TODO
}

void STM32RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
	// TODO
}

uint16_t STM32RCOutput::read(uint8_t ch) {
	// TODO
    return 0;
}

void STM32RCOutput::read(uint16_t* period_us, uint8_t len)
{
	// TODO
}

