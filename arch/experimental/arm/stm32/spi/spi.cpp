#include <arm/stm32/spi/spi.h>

#include <arm/stm32/spi/stm32f4xx/spi_stm32f4xx.h>

namespace stm32 {

Stm32SpiBus::Stm32SpiBus()
    : mutex_(),
      backend_(),
      gpio_(nullptr),
      chip_select_(nullptr),
      device_config_{0U, 0U, 0U, 0U},
      bus_configured_(false),
      device_configured_(false) {}

mc_rtos_hal::Status Stm32SpiBus::configure(const Stm32SpiBusConfig &config, mc_rtos_hal::GpioController &gpio) {
    backend_ = std::make_unique<Stm32F4xxSpiBackend>();
    gpio_ = &gpio;
    chip_select_ = nullptr;
    device_configured_ = false;

    const mc_rtos_hal::Status status = backend_->init(config);
    bus_configured_ = (status == mc_rtos_hal::Status::Ok);
    return status;
}

mc_rtos_hal::Status Stm32SpiBus::lock(mc_rtos_hal::Timeout timeout) {
    return mutex_.lock(timeout);
}

void Stm32SpiBus::unlock() {
    mutex_.unlock();
}

mc_rtos_hal::Status Stm32SpiBus::configure_device(const mc_rtos_hal::SpiDeviceConfig &config) {
    if (!bus_configured_ || !backend_ || gpio_ == nullptr) {
        return mc_rtos_hal::Status::Unsupported;
    }

    mc_rtos_hal::GpioPin *chip_select = gpio_->pin(config.chip_select_pin);
    if (chip_select == nullptr) {
        return mc_rtos_hal::Status::InvalidArgument;
    }
    if (chip_select->set_mode(mc_rtos_hal::PinMode::Output) != mc_rtos_hal::Status::Ok) {
        return mc_rtos_hal::Status::Error;
    }
    chip_select->write(true);

    const mc_rtos_hal::Status status = backend_->configure_device(config);
    if (status != mc_rtos_hal::Status::Ok) {
        return status;
    }

    chip_select_ = chip_select;
    device_config_ = config;
    device_configured_ = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32SpiBus::transfer(const uint8_t *tx, uint8_t *rx, size_t len, mc_rtos_hal::Timeout timeout) {
    if (!device_configured_ || !backend_ || chip_select_ == nullptr || len == 0U) {
        return mc_rtos_hal::Status::Unsupported;
    }

    chip_select_->write(false);
    const mc_rtos_hal::Status status = backend_->transfer(tx, rx, len, timeout);
    chip_select_->write(true);
    return status;
}

}  // namespace stm32
