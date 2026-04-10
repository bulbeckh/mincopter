#pragma once

#include <stddef.h>

#include <rtos_hal/adc.h>
#include <rtos_hal/gpio.h>
#include <rtos_hal/i2c.h>
#include <rtos_hal/pwm.h>
#include <rtos_hal/rtos.h>
#include <rtos_hal/storage.h>
#include <rtos_hal/sync.h>
#include <rtos_hal/time.h>
#include <rtos_hal/uart.h>
#include <rtos_hal/spi.h>

namespace mc_rtos_hal {

class Hal {
public:
    virtual ~Hal() = default;

    virtual Rtos &rtos() = 0;
    virtual Time &time() = 0;
    virtual Adc &adc() = 0;
    virtual GpioController &gpio() = 0;
    virtual SpiBus &spi(size_t index) = 0;
    virtual I2cBus &i2c(size_t index) = 0;
    virtual UartPort &uart(size_t index) = 0;
    virtual PwmOutput &pwm() = 0;
    virtual Storage &storage() = 0;

    virtual size_t spi_bus_count() const = 0;
    virtual size_t i2c_bus_count() const = 0;
    virtual size_t uart_count() const = 0;
};

}  // namespace mc_rtos_hal
