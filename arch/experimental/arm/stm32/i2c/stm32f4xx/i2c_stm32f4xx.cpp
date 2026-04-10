#include <arm/stm32/i2c/stm32f4xx/i2c_stm32f4xx.h>

#include <stm32f4xx_hal.h>

namespace stm32 {

namespace {

I2C_HandleTypeDef g_i2c_handles[3];

I2C_HandleTypeDef *handle_for_instance(I2C_TypeDef *instance) {
    if (instance == I2C1) {
        return &g_i2c_handles[0];
    }
    if (instance == I2C2) {
        return &g_i2c_handles[1];
    }
    if (instance == I2C3) {
        return &g_i2c_handles[2];
    }
    return nullptr;
}

void enable_gpio_clock(GPIO_TypeDef *port) {
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

}  // namespace

Stm32F4xxI2cBackend::Stm32F4xxI2cBackend() : handle_(nullptr) {}

mc_rtos_hal::Status Stm32F4xxI2cBackend::init(const Stm32I2cBusConfig &config) {
    handle_ = handle_for_instance(config.instance);
    if (handle_ == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status gpio_status = init_gpio(config);
    if (gpio_status != mc_rtos_hal::Status::Ok) {
        return gpio_status;
    }

    return init_peripheral(config);
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::init_gpio(const Stm32I2cBusConfig &config) {
    enable_gpio_clock(config.scl.port);
    enable_gpio_clock(config.sda.port);

    GPIO_InitTypeDef gpio_init{};
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio_init.Alternate = config.scl.alternate_function;
    gpio_init.Pin = config.scl.pin;
    HAL_GPIO_Init(config.scl.port, &gpio_init);

    gpio_init.Alternate = config.sda.alternate_function;
    gpio_init.Pin = config.sda.pin;
    HAL_GPIO_Init(config.sda.port, &gpio_init);

    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::enable_clock(I2C_TypeDef *instance) {
    if (instance == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == I2C2) {
        __HAL_RCC_I2C2_CLK_ENABLE();
        __HAL_RCC_I2C2_FORCE_RESET();
        __HAL_RCC_I2C2_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    if (instance == I2C3) {
        __HAL_RCC_I2C3_CLK_ENABLE();
        __HAL_RCC_I2C3_FORCE_RESET();
        __HAL_RCC_I2C3_RELEASE_RESET();
        return mc_rtos_hal::Status::Ok;
    }
    return mc_rtos_hal::Status::InvalidArgument;
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::init_peripheral(const Stm32I2cBusConfig &config) {
    const mc_rtos_hal::Status clock_status = enable_clock(config.instance);
    if (clock_status != mc_rtos_hal::Status::Ok) {
        return clock_status;
    }

    handle_->Instance = config.instance;
    handle_->Init.ClockSpeed = config.clock_speed_hz;
    handle_->Init.DutyCycle = I2C_DUTYCYCLE_2;
    handle_->Init.OwnAddress1 = 0;
    handle_->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    handle_->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    handle_->Init.OwnAddress2 = 0;
    handle_->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    handle_->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    return map_hal_status(HAL_I2C_Init(handle_));
}

uint32_t Stm32F4xxI2cBackend::to_hal_timeout(mc_rtos_hal::Timeout timeout) {
    return timeout.ms;
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::map_hal_status(int hal_status) {
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

mc_rtos_hal::Status Stm32F4xxI2cBackend::write(uint8_t address, const uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr || data == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    return map_hal_status(HAL_I2C_Master_Transmit(handle_, static_cast<uint16_t>(address << 1U), const_cast<uint8_t *>(data), len, to_hal_timeout(timeout)));
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::read(uint8_t address, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr || data == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    return map_hal_status(HAL_I2C_Master_Receive(handle_, static_cast<uint16_t>(address << 1U), data, len, to_hal_timeout(timeout)));
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::write_register(uint8_t address, uint8_t reg, uint8_t value, mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    return map_hal_status(HAL_I2C_Mem_Write(handle_, static_cast<uint16_t>(address << 1U), reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, to_hal_timeout(timeout)));
}

mc_rtos_hal::Status Stm32F4xxI2cBackend::read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t len, mc_rtos_hal::Timeout timeout) {
    if (handle_ == nullptr || data == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    return map_hal_status(HAL_I2C_Mem_Read(handle_, static_cast<uint16_t>(address << 1U), reg, I2C_MEMADD_SIZE_8BIT, data, len, to_hal_timeout(timeout)));
}

}  // namespace stm32
