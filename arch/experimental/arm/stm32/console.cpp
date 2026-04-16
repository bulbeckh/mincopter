#include <arm/stm32/console.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace stm32 {

Stm32Console::Stm32Console() : uart_(nullptr), mutex_() {}

void Stm32Console::attach(mc_rtos_hal::UartPort &uart) {
    uart_ = &uart;
}

mc_rtos_hal::Status Stm32Console::configure(uint32_t baud_rate, uint16_t rx_buffer_size, uint16_t tx_buffer_size) {
    if (uart_ == nullptr) {
        return mc_rtos_hal::Status::Unsupported;
    }

    mc_rtos_hal::UartConfig config{};
    config.baud_rate = baud_rate;
    config.rx_buffer_size = rx_buffer_size;
    config.tx_buffer_size = tx_buffer_size;
    return uart_->configure(config);
}

mc_rtos_hal::Status Stm32Console::write(const char *text, mc_rtos_hal::Timeout timeout, size_t *written) {
    if (text == nullptr) {
        if (written != nullptr) {
            *written = 0U;
        }
        return mc_rtos_hal::Status::InvalidArgument;
    }

    return write(reinterpret_cast<const uint8_t *>(text), strlen(text), timeout, written);
}

mc_rtos_hal::Status Stm32Console::write(const uint8_t *data,
                                        size_t len,
                                        mc_rtos_hal::Timeout timeout,
                                        size_t *written) {
    if (written != nullptr) {
        *written = 0U;
    }
    if (uart_ == nullptr) {
        return mc_rtos_hal::Status::Unsupported;
    }
    if (data == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    if (len == 0U) {
        return mc_rtos_hal::Status::Ok;
    }

    const mc_rtos_hal::Status lock_status = mutex_.lock(timeout);
    if (lock_status != mc_rtos_hal::Status::Ok) {
        return lock_status;
    }

    const mc_rtos_hal::Status status = uart_->write(data, len, timeout, written);
    mutex_.unlock();
    return status;
}

int Stm32Console::printf(mc_rtos_hal::Timeout timeout, const char *fmt, ...) {
    if (fmt == nullptr) {
        return -1;
    }

    char buffer[kPrintfBufferSize] = {};
    va_list args;
    va_start(args, fmt);
    const int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0) {
        return len;
    }

    const size_t write_len = static_cast<size_t>(len) < sizeof(buffer)
                                 ? static_cast<size_t>(len)
                                 : sizeof(buffer) - 1U;
    size_t written = 0U;
    const mc_rtos_hal::Status status =
        write(reinterpret_cast<const uint8_t *>(buffer), write_len, timeout, &written);
    return status == mc_rtos_hal::Status::Ok ? static_cast<int>(written) : -1;
}

}  // namespace stm32
