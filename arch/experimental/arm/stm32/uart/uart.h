#pragma once

#include <memory>

#include <arm/stm32/uart/uart_backend.h>
#include <rtos_hal/uart.h>

namespace stm32 {

class Stm32UartPort final : public mc_rtos_hal::UartPort {
public:
    Stm32UartPort();

    mc_rtos_hal::Status configure_port(const Stm32UartPortConfig &config);

    mc_rtos_hal::Status configure(const mc_rtos_hal::UartConfig &config) override;
    mc_rtos_hal::Status write(const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *written) override;
    mc_rtos_hal::Status read(uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *read_len) override;

private:
    std::unique_ptr<Stm32UartBackend> backend_;
    mc_rtos_hal::UartConfig config_;
    bool port_configured_;
    bool runtime_configured_;
};

}  // namespace stm32
