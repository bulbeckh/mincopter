#include <arm/stm32/hal.h>

#include <experimental_stm32_board_config.h>
#include <stm32f4xx_hal.h>

namespace stm32 {

namespace {

mc_rtos_hal::Status configure_system_clock() {
    RCC_OscInitTypeDef osc{};
    RCC_ClkInitTypeDef clk{};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return mc_rtos_hal::Status::Error;
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        return mc_rtos_hal::Status::Error;
    }

    return mc_rtos_hal::Status::Ok;
}

void enable_cycle_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

}  // namespace

Stm32Hal::Stm32Hal()
    : rtos_(),
      time_(),
      adc_(),
      gpio_(),
      spi_buses_{},
      i2c_buses_{},
      uart_ports_{},
      pwm_(),
      storage_(),
      initialized_(false) {}

mc_rtos_hal::Status Stm32Hal::init() {
    if (initialized_) {
        return mc_rtos_hal::Status::Ok;
    }

    mc_rtos_hal::Status status = configure_platform();
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    status = configure_peripherals();
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    initialized_ = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32Hal::configure_platform() {
    if (HAL_Init() != HAL_OK) {
        return mc_rtos_hal::Status::Error;
    }

    NVIC_SetPriorityGrouping(0U);

    const mc_rtos_hal::Status clock_status = configure_system_clock();
    if (clock_status != mc_rtos_hal::Status::Ok) {
        return clock_status;
    }

    enable_cycle_counter();
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32Hal::configure_peripherals() {
    mc_rtos_hal::Status status =
        gpio_.configure(experimental_stm32_board_config::gpio_pins, experimental_stm32_board_config::gpio_pin_count);
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    if (experimental_stm32_board_config::adc_enabled) {
        status = adc_.configure(experimental_stm32_board_config::adc_unit,
                                experimental_stm32_board_config::adc_channels,
                                experimental_stm32_board_config::adc_channel_count);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }

    if (experimental_stm32_board_config::spi0_enabled) {
        status = spi_buses_[0].configure(experimental_stm32_board_config::spi0, gpio_);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }
    if (experimental_stm32_board_config::spi1_enabled) {
        status = spi_buses_[1].configure(experimental_stm32_board_config::spi1, gpio_);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }

    if (experimental_stm32_board_config::i2c0_enabled) {
        status = i2c_buses_[0].configure(experimental_stm32_board_config::i2c0);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }
    if (experimental_stm32_board_config::i2c1_enabled) {
        status = i2c_buses_[1].configure(experimental_stm32_board_config::i2c1);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }

    if (experimental_stm32_board_config::uart0_enabled) {
        status = uart_ports_[0].configure_port(experimental_stm32_board_config::uart0);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }
    if (experimental_stm32_board_config::uart1_enabled) {
        status = uart_ports_[1].configure_port(experimental_stm32_board_config::uart1);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }
    if (experimental_stm32_board_config::uart2_enabled) {
        status = uart_ports_[2].configure_port(experimental_stm32_board_config::uart2);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }
    if (experimental_stm32_board_config::uart3_enabled) {
        status = uart_ports_[3].configure_port(experimental_stm32_board_config::uart3);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }

    if (experimental_stm32_board_config::pwm_enabled) {
        status = pwm_.configure(experimental_stm32_board_config::pwm);
        if (status != mc_rtos_hal::Status::Ok) {
            return status;
        }
    }

    return mc_rtos_hal::Status::Ok;
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
