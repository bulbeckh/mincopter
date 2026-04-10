#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

struct UartConfig {
    uint32_t baud_rate;
    uint16_t rx_buffer_size;
    uint16_t tx_buffer_size;
};

class UartPort {
public:
    virtual ~UartPort() = default;
    virtual Status configure(const UartConfig &config) = 0;
    virtual Status write(const uint8_t *data, size_t len, Timeout timeout, size_t *written) = 0;
    virtual Status read(uint8_t *data, size_t len, Timeout timeout, size_t *read_len) = 0;
};

}  // namespace mc_rtos_hal
