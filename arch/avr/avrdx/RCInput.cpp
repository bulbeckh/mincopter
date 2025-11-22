#include <AP_HAL.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <AP_HAL.h>
#include <avrdx/AP_HAL_AVRDx.h>

#include "avrdx/RCInput.h"

#include "avrdx/utility/ISRRegistry.h"

using namespace AP_HAL;
using namespace AP_HAL_AVRDx;

extern const HAL& hal;

void AVRDxRCInput::init(void*)
{
	// TODO
}

uint8_t AVRDxRCInput::valid_channels(void)
{
	// TODO
	return 0;
}

uint16_t AVRDxRCInput::read(uint8_t ch)
{
	// TODO
	return 0;
}

uint8_t AVRDxRCInput::read(uint16_t* periods, uint8_t len)
{
	// TODO
	return 0;
}

bool AVRDxRCInput::set_overrides(int16_t *overrides, uint8_t len)
{
	// TODO
	return 0;
}

bool AVRDxRCInput::set_override(uint8_t channel, int16_t override)
{
	// TODO
	return false;
}

void AVRDxRCInput::clear_overrides(void)
{
	// TODO
	return;
}

