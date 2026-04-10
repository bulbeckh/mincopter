#include <arm/stm32/pwm/stm32f4xx/pwm_stm32f4xx.h>

namespace stm32 {

namespace {

constexpr uint32_t kDefaultFrequencyHz = 50U;
constexpr uint32_t kTickHz = 1000000U;

void init_pin(const Stm32PwmPinConfig &pin_config) {
    GPIO_InitTypeDef init{};
    init.Pin = pin_config.pin;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Alternate = pin_config.alternate_function;
    HAL_GPIO_Init(pin_config.port, &init);
}

}  // namespace

Stm32F4xxPwmBackend::Stm32F4xxPwmBackend()
    : handle_{},
      config_{nullptr, 0U, 0U, {}},
      pulse_widths_{0},
      enabled_{false},
      frequency_hz_(kDefaultFrequencyHz),
      tick_hz_(kTickHz),
      period_ticks_(0U),
      initialized_(false) {}

mc_rtos_hal::Status Stm32F4xxPwmBackend::init(const Stm32PwmTimerConfig &config) {
    if (config.instance == nullptr || config.channel_count == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status gpio_status = init_gpio(config);
    if (gpio_status != mc_rtos_hal::Status::Ok) {
        return gpio_status;
    }

    config_ = config;
    const mc_rtos_hal::Status timer_status = init_timer(kDefaultFrequencyHz);
    initialized_ = (timer_status == mc_rtos_hal::Status::Ok);
    return timer_status;
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::enable(uint8_t logical_channel) {
    const int8_t index = index_for_channel(logical_channel);
    if (!initialized_ || index < 0) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    if (HAL_TIM_PWM_Start(&handle_, config_.channels[index].timer_channel) != HAL_OK) {
        return mc_rtos_hal::Status::Error;
    }

    enabled_[logical_channel] = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::disable(uint8_t logical_channel) {
    const int8_t index = index_for_channel(logical_channel);
    if (!initialized_ || index < 0) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    if (HAL_TIM_PWM_Stop(&handle_, config_.channels[index].timer_channel) != HAL_OK) {
        return mc_rtos_hal::Status::Error;
    }

    enabled_[logical_channel] = false;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::set_frequency(uint32_t frequency_hz) {
    if (!initialized_ || frequency_hz == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    return init_timer(frequency_hz);
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::set_pulse_width_us(uint8_t logical_channel, uint16_t pulse_width_us) {
    const int8_t index = index_for_channel(logical_channel);
    if (!initialized_ || index < 0) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    uint32_t compare = pulse_width_us;
    if (compare > period_ticks_) {
        compare = period_ticks_;
    }

    __HAL_TIM_SET_COMPARE(&handle_, config_.channels[index].timer_channel, compare);
    pulse_widths_[logical_channel] = pulse_width_us;
    return mc_rtos_hal::Status::Ok;
}

uint16_t Stm32F4xxPwmBackend::pulse_width_us(uint8_t logical_channel) const {
    if (logical_channel >= 8U) {
        return 0U;
    }
    return pulse_widths_[logical_channel];
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::init_gpio(const Stm32PwmTimerConfig &config) {
    for (uint8_t i = 0; i < config.channel_count; ++i) {
        enable_gpio_clock(config.channels[i].port);
        init_pin(config.channels[i]);
    }
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::init_timer(uint32_t frequency_hz) {
    const mc_rtos_hal::Status clock_status = enable_clock(config_.instance);
    if (clock_status != mc_rtos_hal::Status::Ok) {
        return clock_status;
    }

    handle_.Instance = config_.instance;
    handle_.Init.Prescaler = static_cast<uint32_t>((config_.timer_clock_hz / tick_hz_) - 1U);
    handle_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle_.Init.Period = period_ticks_for(config_.timer_clock_hz, frequency_hz) - 1U;
    handle_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handle_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&handle_) != HAL_OK) {
        return mc_rtos_hal::Status::Error;
    }

    TIM_OC_InitTypeDef config_oc{};
    config_oc.OCMode = TIM_OCMODE_PWM1;
    config_oc.Pulse = 0U;
    config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    config_oc.OCFastMode = TIM_OCFAST_DISABLE;

    for (uint8_t i = 0; i < config_.channel_count; ++i) {
        if (HAL_TIM_PWM_ConfigChannel(&handle_, &config_oc, config_.channels[i].timer_channel) != HAL_OK) {
            return mc_rtos_hal::Status::Error;
        }
    }

    frequency_hz_ = frequency_hz;
    period_ticks_ = handle_.Init.Period + 1U;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxPwmBackend::enable_clock(TIM_TypeDef *instance) {
    if (instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_TIM3_FORCE_RESET();
        __HAL_RCC_TIM3_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_TIM2_FORCE_RESET();
        __HAL_RCC_TIM2_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_TIM4_FORCE_RESET();
        __HAL_RCC_TIM4_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    return mc_rtos_hal::Status::Unsupported;
}

void Stm32F4xxPwmBackend::enable_gpio_clock(GPIO_TypeDef *port) {
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

uint32_t Stm32F4xxPwmBackend::period_ticks_for(uint32_t, uint32_t frequency_hz) {
    return (kTickHz / frequency_hz);
}

int8_t Stm32F4xxPwmBackend::index_for_channel(uint8_t logical_channel) const {
    for (uint8_t i = 0; i < config_.channel_count; ++i) {
        if (config_.channels[i].logical_channel == logical_channel) {
            return static_cast<int8_t>(i);
        }
    }
    return -1;
}

}  // namespace stm32
