#include <AP_HAL/AP_HAL.h>

#include <arch/linux/rpi/RCInput.h>

using namespace RPI;

RPIRCInput::RPIRCInput()
{
}

void RPIRCInput::init(void* machtnichts)
{}

uint8_t RPIRCInput::valid_channels() {
    return 0;
}

uint16_t RPIRCInput::read(uint8_t ch) {
    return 1500;
}

uint8_t RPIRCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        periods[i] = 1500;
    }
    return len;
}

bool RPIRCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool RPIRCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void RPIRCInput::clear_overrides()
{
}

