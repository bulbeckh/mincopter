#pragma once

#include <arm/stm32/i2c/i2c_backend.h>

namespace stm32 {

class Stm32F4xxI2cBackend final : public Stm32I2cBackend {
public:
    Stm32F4xxI2cBackend();

    mc_rtos_hal::Status init(const Stm32I2cBusConfig &config) override;
    mc_rtos_hal::Status write(uint8_t address, const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) override;
    mc_rtos_hal::Status read(uint8_t address, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) override;
    mc_rtos_hal::Status write_register(uint8_t address, uint8_t reg, uint8_t value, mc_rtos_hal::Timeout timeout) override;
    mc_rtos_hal::Status read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) override;

private:
    mc_rtos_hal::Status init_gpio(const Stm32I2cBusConfig &config);
    mc_rtos_hal::Status init_peripheral(const Stm32I2cBusConfig &config);
    mc_rtos_hal::Status enable_clock(I2C_TypeDef *instance);
    static uint32_t to_hal_timeout(mc_rtos_hal::Timeout timeout);
    static mc_rtos_hal::Status map_hal_status(int hal_status);

private:
    I2C_HandleTypeDef *handle_;
};

}  // namespace stm32
