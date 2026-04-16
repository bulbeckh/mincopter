#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

class Console {
public:
    virtual ~Console() = default;

    virtual Status configure(uint32_t baud_rate, uint16_t rx_buffer_size, uint16_t tx_buffer_size) = 0;
    virtual Status write(const char *text, Timeout timeout, size_t *written) = 0;
    virtual Status write(const uint8_t *data, size_t len, Timeout timeout, size_t *written) = 0;
    virtual int printf(Timeout timeout, const char *fmt, ...) = 0;
};

}  // namespace mc_rtos_hal
