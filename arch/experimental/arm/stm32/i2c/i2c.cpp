#include <arm/stm32/i2c/i2c.h>
#include <arm/stm32/i2c/stm32f4xx/i2c_stm32f4xx.h>

namespace stm32 {

Stm32I2cBus::Stm32I2cBus() : mutex_(), backend_(), configured_(false) {}

mc_rtos_hal::Status Stm32I2cBus::configure(const Stm32I2cBusConfig &config) {
    backend_ = std::make_unique<Stm32F4xxI2cBackend>();
    const mc_rtos_hal::Status status = backend_->init(config);
    configured_ = (status == mc_rtos_hal::Status::Ok);
    return status;
}

mc_rtos_hal::Status Stm32I2cBus::lock(mc_rtos_hal::Timeout timeout) {
    return mutex_.lock(timeout);
}

void Stm32I2cBus::unlock() {
    mutex_.unlock();
}

mc_rtos_hal::Status Stm32I2cBus::write(uint8_t address, const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->write(address, data, len, timeout);
}

mc_rtos_hal::Status Stm32I2cBus::read(uint8_t address, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->read(address, data, len, timeout);
}

mc_rtos_hal::Status Stm32I2cBus::write_register(uint8_t address, uint8_t reg, uint8_t value, mc_rtos_hal::Timeout timeout) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->write_register(address, reg, value, timeout);
}

mc_rtos_hal::Status Stm32I2cBus::read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) {
    if (!configured_ || !backend_) {
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->read_registers(address, reg, data, len, timeout);
}

}  // namespace stm32
