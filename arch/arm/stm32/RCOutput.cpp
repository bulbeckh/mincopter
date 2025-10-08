
#include <arch/arm/stm32/RCOutput.h>

using namespace stm32;

void STM32RCOutput::init(void* machtnichts)
{
	// TODO
}

void STM32RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
	// TODO
}

uint16_t STM32RCOutput::get_freq(uint8_t ch) {
	// TODO
	return 0;
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

