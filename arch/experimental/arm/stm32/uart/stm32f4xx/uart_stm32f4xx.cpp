#include <arm/stm32/uart/stm32f4xx/uart_stm32f4xx.h>

namespace stm32 {

namespace {

UART_HandleTypeDef g_uart_handles[4];

UART_HandleTypeDef *handle_for_instance(USART_TypeDef *instance) {
    if (instance == USART1) {
        return &g_uart_handles[0];
    }
    if (instance == USART2) {
        return &g_uart_handles[1];
    }
    if (instance == USART3) {
        return &g_uart_handles[2];
    }
    if (instance == UART4) {
        return &g_uart_handles[3];
    }
    return nullptr;
}

void init_signal(const Stm32UartSignalConfig &signal) {
    GPIO_InitTypeDef init{};
    init.Pin = signal.pin;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = signal.alternate_function;
    HAL_GPIO_Init(signal.port, &init);
}

}  // namespace

Stm32F4xxUartBackend::Stm32F4xxUartBackend()
    : handle_(nullptr),
      port_config_{0U, nullptr, {nullptr, 0U, 0U}, {nullptr, 0U, 0U}},
      initialized_(false) {}

mc_rtos_hal::Status Stm32F4xxUartBackend::init(const Stm32UartPortConfig &config) {
    handle_ = handle_for_instance(config.instance);
    if (handle_ == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status gpio_status = init_gpio(config);
    if (gpio_status != mc_rtos_hal::Status::Ok) {
        return gpio_status;
    }

    port_config_ = config;
    initialized_ = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxUartBackend::configure(const mc_rtos_hal::UartConfig &config) {
    if (!initialized_ || handle_ == nullptr || config.baud_rate == 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status clock_status = enable_clock(port_config_.instance);
    if (clock_status != mc_rtos_hal::Status::Ok) {
        return clock_status;
    }

    handle_->Instance = port_config_.instance;
    handle_->Init.BaudRate = config.baud_rate;
    handle_->Init.WordLength = UART_WORDLENGTH_8B;
    handle_->Init.StopBits = UART_STOPBITS_1;
    handle_->Init.Parity = UART_PARITY_NONE;
    handle_->Init.Mode = UART_MODE_TX_RX;
    handle_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle_->Init.OverSampling = UART_OVERSAMPLING_16;
    return map_hal_status(HAL_UART_Init(handle_));
}

mc_rtos_hal::Status Stm32F4xxUartBackend::write(const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *written) {
    if (written != nullptr) {
        *written = 0U;
    }
    if (!initialized_ || handle_ == nullptr || data == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    if (len == 0U) {
        return mc_rtos_hal::Status::Ok;
    }

    const mc_rtos_hal::Status status = map_hal_status(HAL_UART_Transmit(handle_, const_cast<uint8_t *>(data), len, to_hal_timeout(timeout)));
    if (status == mc_rtos_hal::Status::Ok && written != nullptr) {
        *written = len;
    }
    return status;
}

mc_rtos_hal::Status Stm32F4xxUartBackend::read(uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout, size_t *read_len) {
    if (read_len != nullptr) {
        *read_len = 0U;
    }
    if (!initialized_ || handle_ == nullptr || data == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    if (len == 0U) {
        return mc_rtos_hal::Status::Ok;
    }

    const uint32_t start = HAL_GetTick();
    size_t received = 0U;
    while (received < len) {
        if (HAL_UART_Receive(handle_, &data[received], 1U, 1U) == HAL_OK) {
            ++received;
            continue;
        }

        if ((HAL_GetTick() - start) >= timeout.ms) {
            break;
        }
    }

    if (read_len != nullptr) {
        *read_len = received;
    }
    return (received > 0U) ? mc_rtos_hal::Status::Ok : mc_rtos_hal::Status::Timeout;
}

mc_rtos_hal::Status Stm32F4xxUartBackend::init_gpio(const Stm32UartPortConfig &config) {
    enable_gpio_clock(config.tx.port);
    enable_gpio_clock(config.rx.port);

    init_signal(config.tx);
    init_signal(config.rx);
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxUartBackend::enable_clock(USART_TypeDef *instance) {
    if (instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == UART4) {
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    return mc_rtos_hal::Status::InvalidArgument;
}

uint32_t Stm32F4xxUartBackend::to_hal_timeout(mc_rtos_hal::Timeout timeout) {
    return timeout.ms;
}

mc_rtos_hal::Status Stm32F4xxUartBackend::map_hal_status(HAL_StatusTypeDef hal_status) {
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

void Stm32F4xxUartBackend::enable_gpio_clock(GPIO_TypeDef *port) {
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
