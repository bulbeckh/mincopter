#pragma once

#include <arm/stm32/sync.h>
#include <rtos_hal/console.h>
#include <rtos_hal/uart.h>

namespace stm32 {

class Stm32Console final : public mc_rtos_hal::Console {
public:
    Stm32Console();

    void attach(mc_rtos_hal::UartPort &uart);

    mc_rtos_hal::Status configure(uint32_t baud_rate, uint16_t rx_buffer_size, uint16_t tx_buffer_size) override;
    mc_rtos_hal::Status write(const char *text, mc_rtos_hal::Timeout timeout, size_t *written) override;
    mc_rtos_hal::Status write(const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *written) override;
    int printf(mc_rtos_hal::Timeout timeout, const char *fmt, ...) override;

private:
    static constexpr size_t kPrintfBufferSize = 384;

    mc_rtos_hal::UartPort *uart_;
    Stm32Mutex mutex_;
};

}  // namespace stm32
