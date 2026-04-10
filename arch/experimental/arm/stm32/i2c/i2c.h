#pragma once

#include <memory>

#include <arm/stm32/i2c/i2c_backend.h>
#include <arm/stm32/sync.h>
#include <rtos_hal/i2c.h>

namespace stm32 {

class Stm32I2cBus final : public mc_rtos_hal::I2cBus {
public:
    Stm32I2cBus();

    mc_rtos_hal::Status configure(const Stm32I2cBusConfig &config);

    mc_rtos_hal::Status lock(mc_rtos_hal::Timeout timeout) override;
    void unlock() override;
    mc_rtos_hal::Status write(uint8_t address, const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) override;
    mc_rtos_hal::Status read(uint8_t address, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) override;
    mc_rtos_hal::Status write_register(uint8_t address, uint8_t reg, uint8_t value, mc_rtos_hal::Timeout timeout) override;
    mc_rtos_hal::Status read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) override;

private:
    Stm32Mutex mutex_;
    std::unique_ptr<Stm32I2cBackend> backend_;
    bool configured_;
};

}  // namespace stm32
