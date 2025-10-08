
#include <arch/arm/stm32/RCInput.h>

using namespace stm32;

STM32RCInput::STM32RCInput()
{
	// TODO
}

void STM32RCInput::init(void* machtnichts)
{
	// TODO
}

uint8_t STM32RCInput::valid_channels() {
	// TODO
    return 0;
}

uint16_t STM32RCInput::read(uint8_t ch) {
	// TODO
	return 0;
}

uint8_t STM32RCInput::read(uint16_t* periods, uint8_t len) {
	// TODO
    return 0;
}

bool STM32RCInput::set_overrides(int16_t *overrides, uint8_t len) {
	// TODO
    return true;
}

bool STM32RCInput::set_override(uint8_t channel, int16_t override) {
	// TODO
    return true;
}

void STM32RCInput::clear_overrides()
{
	// TODO
}

