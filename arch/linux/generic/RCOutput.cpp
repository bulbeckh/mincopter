#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput.h"

using namespace generic;

void GenericRCOutput::init(void* machtnichts) {}

void GenericRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t GenericRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void GenericRCOutput::enable_ch(uint8_t ch)
{}

void GenericRCOutput::disable_ch(uint8_t ch)
{}

void GenericRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void GenericRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t GenericRCOutput::read(uint8_t ch) {
    return 900;
}

void GenericRCOutput::read(uint16_t* period_us, uint8_t len)
{}

#endif // CONFIG_HAL_BOARD
