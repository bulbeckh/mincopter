#include <arm/stm32/adc/stm32f4xx/adc_stm32f4xx.h>

namespace stm32 {

Stm32F4xxAdcBackend::Stm32F4xxAdcBackend()
    : handle_{},
      initialized_(false) {}

mc_rtos_hal::Status Stm32F4xxAdcBackend::init(const Stm32AdcUnitConfig &config) {
    if (config.instance == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status clock_status = enable_clock(config.instance);
    if (clock_status != mc_rtos_hal::Status::Ok) {
        return clock_status;
    }

    handle_.Instance = config.instance;
    handle_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    handle_.Init.Resolution = ADC_RESOLUTION_12B;
    handle_.Init.ScanConvMode = DISABLE;
    handle_.Init.ContinuousConvMode = DISABLE;
    handle_.Init.DiscontinuousConvMode = DISABLE;
    handle_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    handle_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handle_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    handle_.Init.NbrOfConversion = 1;
    handle_.Init.DMAContinuousRequests = DISABLE;
    handle_.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    const mc_rtos_hal::Status status = map_hal_status(HAL_ADC_Init(&handle_));
    initialized_ = (status == mc_rtos_hal::Status::Ok);
    return status;
}

mc_rtos_hal::Status Stm32F4xxAdcBackend::init_channel(const Stm32AdcChannelHardwareConfig &config) {
    if (!initialized_ || config.port == nullptr || config.pin == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    enable_gpio_clock(config.port);

    GPIO_InitTypeDef init{};
    init.Pin = config.pin;
    init.Mode = GPIO_MODE_ANALOG;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(config.port, &init);
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxAdcBackend::read_raw(const Stm32AdcChannelHardwareConfig &config, uint16_t &value) {
    value = 0U;
    if (!initialized_) {
        return mc_rtos_hal::Status::Unsupported;
    }

    ADC_ChannelConfTypeDef channel{};
    channel.Channel = config.channel;
    channel.Rank = 1;
    channel.SamplingTime = ADC_SAMPLETIME_144CYCLES;

    mc_rtos_hal::Status status = map_hal_status(HAL_ADC_ConfigChannel(&handle_, &channel));
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    status = map_hal_status(HAL_ADC_Start(&handle_));
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    status = map_hal_status(HAL_ADC_PollForConversion(&handle_, 10U));
    if (status == mc_rtos_hal::Status::Ok) {
        value = static_cast<uint16_t>(HAL_ADC_GetValue(&handle_));
    }

    (void)HAL_ADC_Stop(&handle_);
    return status;
}

mc_rtos_hal::Status Stm32F4xxAdcBackend::enable_clock(ADC_TypeDef *instance) {
    if (instance == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == ADC2) {
        __HAL_RCC_ADC2_CLK_ENABLE();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
        return mc_rtos_hal::Status::Ok;
    }
    return mc_rtos_hal::Status::InvalidArgument;
}

void Stm32F4xxAdcBackend::enable_gpio_clock(GPIO_TypeDef *port) {
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

mc_rtos_hal::Status Stm32F4xxAdcBackend::map_hal_status(HAL_StatusTypeDef hal_status) {
    switch (hal_status) {
    case HAL_OK:
        return mc_rtos_hal::Status::Ok;
    case HAL_TIMEOUT:
        return mc_rtos_hal::Status::Timeout;
    case HAL_BUSY:
        return mc_rtos_hal::Status::Busy;
    default:
        return mc_rtos_hal::Status::Error;
    }
}

}  // namespace stm32

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *) {}
