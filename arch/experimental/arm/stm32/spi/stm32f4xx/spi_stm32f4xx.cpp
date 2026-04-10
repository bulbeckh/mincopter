#include <arm/stm32/spi/stm32f4xx/spi_stm32f4xx.h>

namespace stm32 {

namespace {

SPI_HandleTypeDef g_spi_handles[3];

SPI_HandleTypeDef *handle_for_instance(SPI_TypeDef *instance) {
    if (instance == SPI1) {
        return &g_spi_handles[0];
    }
    if (instance == SPI2) {
        return &g_spi_handles[1];
    }
    if (instance == SPI3) {
        return &g_spi_handles[2];
    }
    return nullptr;
}

void init_signal(const Stm32SpiSignalConfig &signal) {
    GPIO_InitTypeDef init{};
    init.Pin = signal.pin;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = signal.alternate_function;
    HAL_GPIO_Init(signal.port, &init);
}

}  // namespace

Stm32F4xxSpiBackend::Stm32F4xxSpiBackend()
    : handle_(nullptr),
      bus_config_{0U, nullptr, {nullptr, 0U, 0U}, {nullptr, 0U, 0U}, {nullptr, 0U, 0U}},
      initialized_(false) {}

mc_rtos_hal::Status Stm32F4xxSpiBackend::init(const Stm32SpiBusConfig &config) {
    handle_ = handle_for_instance(config.instance);
    if (handle_ == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status gpio_status = init_gpio(config);
    if (gpio_status != mc_rtos_hal::Status::Ok) {
        return gpio_status;
    }

    bus_config_ = config;
    initialized_ = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxSpiBackend::configure_device(const mc_rtos_hal::SpiDeviceConfig &config) {
    if (!initialized_ || handle_ == nullptr) {
        return mc_rtos_hal::Status::Unsupported;
    }
    if (config.bits_per_word != 8U || config.mode > 3U || config.frequency_hz == 0U) {
        return mc_rtos_hal::Status::Unsupported;
    }

    return init_peripheral(config);
}

mc_rtos_hal::Status Stm32F4xxSpiBackend::transfer(const uint8_t *tx, uint8_t *rx, size_t len, mc_rtos_hal::Timeout timeout) {
    if (!initialized_ || handle_ == nullptr || len == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const uint32_t hal_timeout = to_hal_timeout(timeout);
    if (tx != nullptr && rx != nullptr) {
        return map_hal_status(HAL_SPI_TransmitReceive(handle_, const_cast<uint8_t *>(tx), rx, len, hal_timeout));
    }
    if (tx != nullptr) {
        return map_hal_status(HAL_SPI_Transmit(handle_, const_cast<uint8_t *>(tx), len, hal_timeout));
    }
    if (rx != nullptr) {
        return map_hal_status(HAL_SPI_Receive(handle_, rx, len, hal_timeout));
    }
    return mc_rtos_hal::Status::InvalidArgument;
}

mc_rtos_hal::Status Stm32F4xxSpiBackend::init_gpio(const Stm32SpiBusConfig &config) {
    enable_gpio_clock(config.sck.port);
    enable_gpio_clock(config.miso.port);
    enable_gpio_clock(config.mosi.port);

    init_signal(config.sck);
    init_signal(config.miso);
    init_signal(config.mosi);
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxSpiBackend::init_peripheral(const mc_rtos_hal::SpiDeviceConfig &config) {
    const mc_rtos_hal::Status clock_status = enable_clock(bus_config_.instance);
    if (clock_status != mc_rtos_hal::Status::Ok) {
        return clock_status;
    }

    handle_->Instance = bus_config_.instance;
    handle_->Init.Mode = SPI_MODE_MASTER;
    handle_->Init.Direction = SPI_DIRECTION_2LINES;
    handle_->Init.DataSize = SPI_DATASIZE_8BIT;
    handle_->Init.NSS = SPI_NSS_SOFT;
    handle_->Init.FirstBit = SPI_FIRSTBIT_MSB;
    handle_->Init.TIMode = SPI_TIMODE_DISABLE;
    handle_->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    handle_->Init.CRCPolynomial = 7U;
    handle_->Init.BaudRatePrescaler = baud_prescaler_for(config.frequency_hz);

    switch (config.mode) {
    case 0:
        handle_->Init.CLKPolarity = SPI_POLARITY_LOW;
        handle_->Init.CLKPhase = SPI_PHASE_1EDGE;
        break;
    case 1:
        handle_->Init.CLKPolarity = SPI_POLARITY_LOW;
        handle_->Init.CLKPhase = SPI_PHASE_2EDGE;
        break;
    case 2:
        handle_->Init.CLKPolarity = SPI_POLARITY_HIGH;
        handle_->Init.CLKPhase = SPI_PHASE_1EDGE;
        break;
    case 3:
        handle_->Init.CLKPolarity = SPI_POLARITY_HIGH;
        handle_->Init.CLKPhase = SPI_PHASE_2EDGE;
        break;
    default:
        return mc_rtos_hal::Status::Unsupported;
    }

    return map_hal_status(HAL_SPI_Init(handle_));
}

mc_rtos_hal::Status Stm32F4xxSpiBackend::enable_clock(SPI_TypeDef *instance) {
    if (instance == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_SPI1_FORCE_RESET();
        __HAL_RCC_SPI1_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == SPI2) {
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_SPI2_FORCE_RESET();
        __HAL_RCC_SPI2_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == SPI3) {
        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_SPI3_FORCE_RESET();
        __HAL_RCC_SPI3_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    return mc_rtos_hal::Status::InvalidArgument;
}

uint32_t Stm32F4xxSpiBackend::to_hal_timeout(mc_rtos_hal::Timeout timeout) {
    return timeout.ms;
}

mc_rtos_hal::Status Stm32F4xxSpiBackend::map_hal_status(HAL_StatusTypeDef hal_status) {
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

uint32_t Stm32F4xxSpiBackend::baud_prescaler_for(uint32_t requested_hz) {
    if (requested_hz >= 21000000U) {
        return SPI_BAUDRATEPRESCALER_2;
    }
    if (requested_hz >= 10500000U) {
        return SPI_BAUDRATEPRESCALER_4;
    }
    if (requested_hz >= 5250000U) {
        return SPI_BAUDRATEPRESCALER_8;
    }
    if (requested_hz >= 2625000U) {
        return SPI_BAUDRATEPRESCALER_16;
    }
    if (requested_hz >= 1312500U) {
        return SPI_BAUDRATEPRESCALER_32;
    }
    if (requested_hz >= 656250U) {
        return SPI_BAUDRATEPRESCALER_64;
    }
    if (requested_hz >= 328125U) {
        return SPI_BAUDRATEPRESCALER_128;
    }
    return SPI_BAUDRATEPRESCALER_256;
}

void Stm32F4xxSpiBackend::enable_gpio_clock(GPIO_TypeDef *port) {
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

}  // namespace stm32
