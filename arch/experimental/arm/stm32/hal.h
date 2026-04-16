#pragma once

#include <arm/stm32/adc/adc.h>
#include <arm/stm32/console.h>
#include <arm/stm32/gpio/gpio.h>
#include <arm/stm32/i2c/i2c.h>
#include <arm/stm32/pwm/pwm.h>
#include <arm/stm32/rtos.h>
#include <arm/stm32/storage/storage.h>
#include <arm/stm32/time.h>
#include <arm/stm32/uart/uart.h>
#include <arm/stm32/spi/spi.h>
#include <rtos_hal/hal.h>

namespace stm32 {

class Stm32Hal final : public mc_rtos_hal::Hal {
public:
    Stm32Hal();

    mc_rtos_hal::Status init() override;
    mc_rtos_hal::Rtos &rtos() override;
    mc_rtos_hal::Time &time() override;
    mc_rtos_hal::Adc &adc() override;
    mc_rtos_hal::GpioController &gpio() override;
    mc_rtos_hal::SpiBus &spi(size_t index) override;
    mc_rtos_hal::I2cBus &i2c(size_t index) override;
    mc_rtos_hal::UartPort &uart(size_t index) override;
    mc_rtos_hal::UartPort &console_uart() override;
    mc_rtos_hal::Console &console() override;
    mc_rtos_hal::PwmOutput &pwm() override;
    mc_rtos_hal::Storage &storage() override;

    size_t spi_bus_count() const override;
    size_t i2c_bus_count() const override;
    size_t uart_count() const override;

private:
    static constexpr size_t kSpiBusCount = 2;
    static constexpr size_t kI2cBusCount = 2;
    static constexpr size_t kUartCount = 4;

    mc_rtos_hal::Status configure_platform();
    mc_rtos_hal::Status configure_peripherals();

    Stm32Rtos rtos_;
    Stm32Time time_;
    Stm32Adc adc_;
    Stm32GpioController gpio_;
    Stm32SpiBus spi_buses_[kSpiBusCount];
    Stm32I2cBus i2c_buses_[kI2cBusCount];
    Stm32UartPort uart_ports_[kUartCount];
    Stm32Console console_;
    Stm32PwmOutput pwm_;
    Stm32Storage storage_;
    bool initialized_;
};

}  // namespace stm32
