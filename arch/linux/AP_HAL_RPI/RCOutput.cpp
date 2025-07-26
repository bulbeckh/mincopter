#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput.h"

using namespace RPI;

void RPIRCOutput::init(void* machtnichts)
{
}

void RPIRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
}

uint16_t RPIRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void RPIRCOutput::enable_ch(uint8_t ch)
{
}

void RPIRCOutput::disable_ch(uint8_t ch)
{
}

void RPIRCOutput::write(uint8_t ch, uint16_t period_us)
{
}

void RPIRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
}

uint16_t RPIRCOutput::read(uint8_t ch) {
    return 900;
}

void RPIRCOutput::read(uint16_t* period_us, uint8_t len)
{
}

#endif // CONFIG_HAL_BOARD
