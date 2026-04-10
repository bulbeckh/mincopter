#include <arm/stm32/gpio/stm32f4xx/gpio_stm32f4xx.h>

namespace stm32 {

namespace {

uint32_t pin_to_pin_source(uint16_t pin) {
    for (uint32_t i = 0; i < 16U; ++i) {
        if (pin == (1U << i)) {
            return i;
        }
    }
    return 16U;
}

}  // namespace

mc_rtos_hal::Status Stm32F4xxGpioBackend::init_pin(const Stm32HalGpioPinConfig &config, mc_rtos_hal::PinMode mode) {
    if (config.port == nullptr || config.pin == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    if (mode == mc_rtos_hal::PinMode::Alternate) {
        return mc_rtos_hal::Status::Unsupported;
    }

    enable_gpio_clock(config.port);

    GPIO_InitTypeDef init = make_init(config, mode);
    HAL_GPIO_Init(config.port, &init);
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxGpioBackend::write_pin(const Stm32HalGpioPinConfig &config, bool level) {
    if (config.port == nullptr || config.pin == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    HAL_GPIO_WritePin(config.port, config.pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return mc_rtos_hal::Status::Ok;
}

bool Stm32F4xxGpioBackend::read_pin(const Stm32HalGpioPinConfig &config) const {
    if (config.port == nullptr || config.pin == 0U) {
        return false;
    }
    return HAL_GPIO_ReadPin(config.port, config.pin) == GPIO_PIN_SET;
}

mc_rtos_hal::Status Stm32F4xxGpioBackend::toggle_pin(const Stm32HalGpioPinConfig &config) {
    if (config.port == nullptr || config.pin == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    HAL_GPIO_TogglePin(config.port, config.pin);
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxGpioBackend::attach_interrupt(const Stm32HalGpioPinConfig &config, mc_rtos_hal::Edge edge) {
    if (config.port == nullptr || config.pin == 0U || !config.interrupt_capable) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const uint32_t pin_source = pin_to_pin_source(config.pin);
    if (pin_source > 15U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    enable_gpio_clock(config.port);
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef init{};
    init.Pin = config.pin;
    init.Mode = to_exti_mode(edge);
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(config.port, &init);

    const uint32_t port_source = static_cast<uint32_t>(GPIO_GET_INDEX(config.port));
    const uint32_t index = pin_source >> 2U;
    const uint32_t shift = (pin_source & 0x3U) * 4U;
    MODIFY_REG(SYSCFG->EXTICR[index], 0xFU << shift, port_source << shift);

    enable_irq(config.pin);
    return mc_rtos_hal::Status::Ok;
}

void Stm32F4xxGpioBackend::enable_irq(uint16_t pin) {
    const IRQn_Type irqn = irq_for_pin(pin);
    HAL_NVIC_SetPriority(irqn, 5U, 0U);
    HAL_NVIC_EnableIRQ(irqn);
}

void Stm32F4xxGpioBackend::enable_gpio_clock(GPIO_TypeDef *port) {
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (port == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    } else if (port == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    } else if (port == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    } else if (port == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    } else if (port == GPIOI) {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
}

GPIO_InitTypeDef Stm32F4xxGpioBackend::make_init(const Stm32HalGpioPinConfig &config, mc_rtos_hal::PinMode mode) {
    GPIO_InitTypeDef init{};
    init.Pin = config.pin;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Alternate = 0U;

    switch (mode) {
    case mc_rtos_hal::PinMode::Input:
        init.Mode = GPIO_MODE_INPUT;
        break;
    case mc_rtos_hal::PinMode::Output:
        init.Mode = GPIO_MODE_OUTPUT_PP;
        break;
    case mc_rtos_hal::PinMode::Analog:
        init.Mode = GPIO_MODE_ANALOG;
        break;
    case mc_rtos_hal::PinMode::Alternate:
    default:
        init.Mode = GPIO_MODE_INPUT;
        break;
    }

    return init;
}

uint32_t Stm32F4xxGpioBackend::to_exti_mode(mc_rtos_hal::Edge edge) {
    switch (edge) {
    case mc_rtos_hal::Edge::Rising:
        return GPIO_MODE_IT_RISING;
    case mc_rtos_hal::Edge::Falling:
        return GPIO_MODE_IT_FALLING;
    case mc_rtos_hal::Edge::Both:
        return GPIO_MODE_IT_RISING_FALLING;
    default:
        return GPIO_MODE_IT_RISING;
    }
}

IRQn_Type Stm32F4xxGpioBackend::irq_for_pin(uint16_t pin) {
    switch (pin_to_pin_source(pin)) {
    case 0U:
        return EXTI0_IRQn;
    case 1U:
        return EXTI1_IRQn;
    case 2U:
        return EXTI2_IRQn;
    case 3U:
        return EXTI3_IRQn;
    case 4U:
        return EXTI4_IRQn;
    case 5U:
    case 6U:
    case 7U:
    case 8U:
    case 9U:
        return EXTI9_5_IRQn;
    case 10U:
    case 11U:
    case 12U:
    case 13U:
    case 14U:
    case 15U:
    default:
        return EXTI15_10_IRQn;
    }
}

}  // namespace stm32
