#include <arm/stm32/gpio/gpio.h>

#include <arm/stm32/gpio/stm32f4xx/gpio_stm32f4xx.h>

namespace stm32 {

namespace {

Stm32GpioController *g_gpio_controller = nullptr;
Stm32F4xxGpioBackend g_gpio_backend;

}  // namespace

Stm32GpioPin::Stm32GpioPin()
    : controller_(nullptr),
      backend_(nullptr),
      config_{0U, nullptr, 0U, false, false},
      mode_(mc_rtos_hal::PinMode::Input),
      interrupt_config_{mc_rtos_hal::Edge::Rising, nullptr, nullptr},
      configured_(false),
      interrupt_attached_(false) {}

void Stm32GpioPin::configure(Stm32GpioController *controller, Stm32GpioBackend *backend, const Stm32HalGpioPinConfig &config) {
    controller_ = controller;
    backend_ = backend;
    config_ = config;
    mode_ = mc_rtos_hal::PinMode::Input;
    interrupt_config_ = {mc_rtos_hal::Edge::Rising, nullptr, nullptr};
    configured_ = true;
    interrupt_attached_ = false;
}

bool Stm32GpioPin::is_configured() const {
    return configured_;
}

uint16_t Stm32GpioPin::logical_pin() const {
    return config_.logical_pin;
}

uint16_t Stm32GpioPin::exti_line() const {
    return Stm32GpioController::exti_line_for_mask(config_.pin);
}

const Stm32HalGpioPinConfig &Stm32GpioPin::config() const {
    return config_;
}

void Stm32GpioPin::handle_interrupt() const {
    if (interrupt_attached_ && interrupt_config_.callback != nullptr) {
        interrupt_config_.callback(interrupt_config_.context);
    }
}

mc_rtos_hal::Status Stm32GpioPin::set_mode(mc_rtos_hal::PinMode mode) {
    if (!configured_ || backend_ == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status status = backend_->init_pin(config_, mode);
    if (status == mc_rtos_hal::Status::Ok) {
        mode_ = mode;
    }
    return status;
}

void Stm32GpioPin::write(bool level) {
    if (!configured_ || backend_ == nullptr) {
        return;
    }

    const bool physical_level = config_.active_low ? !level : level;
    (void)backend_->write_pin(config_, physical_level);
}

bool Stm32GpioPin::read() const {
    if (!configured_ || backend_ == nullptr) {
        return false;
    }

    const bool physical_level = backend_->read_pin(config_);
    return config_.active_low ? !physical_level : physical_level;
}

void Stm32GpioPin::toggle() {
    if (!configured_ || backend_ == nullptr) {
        return;
    }

    (void)backend_->toggle_pin(config_);
}

mc_rtos_hal::Status Stm32GpioPin::attach_interrupt(const mc_rtos_hal::GpioInterruptConfig &config) {
    if (!configured_ || controller_ == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const mc_rtos_hal::Status status = controller_->register_interrupt(*this, config);
    if (status == mc_rtos_hal::Status::Ok) {
        interrupt_config_ = config;
        interrupt_attached_ = true;
        mode_ = mc_rtos_hal::PinMode::Input;
    }
    return status;
}

Stm32GpioController::Stm32GpioController()
    : pins_(),
      exti_bindings_{},
      backend_(&g_gpio_backend) {}

mc_rtos_hal::Status Stm32GpioController::configure(const Stm32HalGpioPinConfig *pins, size_t pin_count) {
    if (pins == nullptr && pin_count != 0U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    g_gpio_controller = this;
    for (size_t i = 0; i < pin_count; ++i) {
        const Stm32HalGpioPinConfig &config = pins[i];
        if (config.logical_pin >= kMaxLogicalPins || config.port == nullptr || config.pin == 0U) {
            return mc_rtos_hal::Status::InvalidArgument;
        }
        pins_[config.logical_pin].configure(this, backend_, config);
    }

    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::GpioPin *Stm32GpioController::pin(uint16_t logical_pin) {
    if (logical_pin >= kMaxLogicalPins) {
        return nullptr;
    }
    return pins_[logical_pin].is_configured() ? &pins_[logical_pin] : nullptr;
}

mc_rtos_hal::Status Stm32GpioController::register_interrupt(Stm32GpioPin &pin, const mc_rtos_hal::GpioInterruptConfig &config) {
    if (backend_ == nullptr || config.callback == nullptr || !pin.config().interrupt_capable) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    const uint16_t exti_line = pin.exti_line();
    if (exti_line >= 16U) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    Stm32GpioPin *&binding = exti_bindings_[exti_line];
    if (binding != nullptr && binding != &pin) {
        return mc_rtos_hal::Status::Busy;
    }

    const mc_rtos_hal::Status status = backend_->attach_interrupt(pin.config(), config.edge);
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    binding = &pin;
    return mc_rtos_hal::Status::Ok;
}

void Stm32GpioController::handle_exti_irq(uint16_t pin_mask) {
    const uint16_t exti_line = exti_line_for_mask(pin_mask);
    if (exti_line >= 16U) {
        return;
    }

    Stm32GpioPin *pin = exti_bindings_[exti_line];
    if (pin != nullptr) {
        pin->handle_interrupt();
    }
}

uint16_t Stm32GpioController::exti_line_for_mask(uint16_t pin_mask) {
    for (uint16_t i = 0; i < 16U; ++i) {
        if (pin_mask == static_cast<uint16_t>(1U << i)) {
            return i;
        }
    }
    return 0xFFFFU;
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (g_gpio_controller != nullptr) {
        g_gpio_controller->handle_exti_irq(GPIO_Pin);
    }
}

extern "C" void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

extern "C" void EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

extern "C" void EXTI2_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

extern "C" void EXTI3_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

extern "C" void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

extern "C" void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

extern "C" void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

}  // namespace stm32
