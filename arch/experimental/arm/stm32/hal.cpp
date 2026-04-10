#include <arm/stm32/hal.h>

#include <experimental_stm32_board_config.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

Stm32Hal::Stm32Hal() {
    gpio_.configure(experimental_stm32_board_config::gpio_pins, experimental_stm32_board_config::gpio_pin_count);

    if (experimental_stm32_board_config::adc_enabled) {
        adc_.configure(experimental_stm32_board_config::adc_unit,
                       experimental_stm32_board_config::adc_channels,
                       experimental_stm32_board_config::adc_channel_count);
    }

    if (experimental_stm32_board_config::spi0_enabled) {
        spi_buses_[0].configure(experimental_stm32_board_config::spi0, gpio_);
    }
    if (experimental_stm32_board_config::spi1_enabled) {
        spi_buses_[1].configure(experimental_stm32_board_config::spi1, gpio_);
    }

    if (experimental_stm32_board_config::i2c0_enabled) {
        i2c_buses_[0].configure(experimental_stm32_board_config::i2c0);
    }
    if (experimental_stm32_board_config::i2c1_enabled) {
        i2c_buses_[1].configure(experimental_stm32_board_config::i2c1);
    }

    if (experimental_stm32_board_config::uart0_enabled) {
        uart_ports_[0].configure_port(experimental_stm32_board_config::uart0);
    }
    if (experimental_stm32_board_config::uart1_enabled) {
        uart_ports_[1].configure_port(experimental_stm32_board_config::uart1);
    }
    if (experimental_stm32_board_config::uart2_enabled) {
        uart_ports_[2].configure_port(experimental_stm32_board_config::uart2);
    }
    if (experimental_stm32_board_config::uart3_enabled) {
        uart_ports_[3].configure_port(experimental_stm32_board_config::uart3);
    }

    if (experimental_stm32_board_config::pwm_enabled) {
        pwm_.configure(experimental_stm32_board_config::pwm);
    }
}

mc_rtos_hal::Rtos &Stm32Hal::rtos() {
    return rtos_;
}

mc_rtos_hal::Time &Stm32Hal::time() {
    return time_;
}

mc_rtos_hal::Adc &Stm32Hal::adc() {
    return adc_;
}

mc_rtos_hal::GpioController &Stm32Hal::gpio() {
    return gpio_;
}

mc_rtos_hal::SpiBus &Stm32Hal::spi(size_t index) {
    return spi_buses_[index];
}

mc_rtos_hal::I2cBus &Stm32Hal::i2c(size_t index) {
    return i2c_buses_[index];
}

mc_rtos_hal::UartPort &Stm32Hal::uart(size_t index) {
    return uart_ports_[index];
}

mc_rtos_hal::PwmOutput &Stm32Hal::pwm() {
    return pwm_;
}

mc_rtos_hal::Storage &Stm32Hal::storage() {
    return storage_;
}

size_t Stm32Hal::spi_bus_count() const {
    return kSpiBusCount;
}

size_t Stm32Hal::i2c_bus_count() const {
    return kI2cBusCount;
}

size_t Stm32Hal::uart_count() const {
    return kUartCount;
}

}  // namespace stm32
