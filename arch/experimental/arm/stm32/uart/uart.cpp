#include <arm/stm32/uart/uart.h>

#include <arm/stm32/uart/stm32f4xx/uart_stm32f4xx.h>

namespace stm32 {

Stm32UartPort::Stm32UartPort()
    : backend_(),
      config_{0U, 0U, 0U},
      port_configured_(false),
      runtime_configured_(false) {}

mc_rtos_hal::Status Stm32UartPort::configure_port(const Stm32UartPortConfig &config) {
    backend_ = std::make_unique<Stm32F4xxUartBackend>();
    const mc_rtos_hal::Status status = backend_->init(config);
    port_configured_ = (status == mc_rtos_hal::Status::Ok);
    runtime_configured_ = false;
    return status;
}

mc_rtos_hal::Status Stm32UartPort::configure(const mc_rtos_hal::UartConfig &config) {
    if (!port_configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }

    const mc_rtos_hal::Status status = backend_->configure(config);
    if (status == mc_rtos_hal::Status::Ok) {
        config_ = config;
        runtime_configured_ = true;
    }
    return status;
}

mc_rtos_hal::Status Stm32UartPort::write(const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *written) {
    if (!runtime_configured_ || !backend_) {
        if (written != nullptr) {
            *written = 0U;
        }
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->write(data, len, timeout, written);
}

mc_rtos_hal::Status Stm32UartPort::read(uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *read_len) {
    if (!runtime_configured_ || !backend_) {
        if (read_len != nullptr) {
            *read_len = 0U;
        }
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->read(data, len, timeout, read_len);
}

}  // namespace stm32
