#include <arm/stm32/adc/adc.h>

#include <arm/stm32/adc/stm32f4xx/adc_stm32f4xx.h>

namespace stm32 {

Stm32AdcChannel::Stm32AdcChannel()
    : controller_(nullptr),
      hardware_config_{0U, nullptr, 0U, 0U},
      config_{0U, 3.3f, 1.0f},
      bound_(false),
      configured_(false) {}

void Stm32AdcChannel::bind(Stm32Adc *controller, const Stm32AdcChannelHardwareConfig &hardware_config) {
    controller_ = controller;
    hardware_config_ = hardware_config;
    config_ = {hardware_config.logical_index, 3.3f, 1.0f};
    bound_ = true;
    configured_ = false;
}

bool Stm32AdcChannel::is_bound() const {
    return bound_;
}

mc_rtos_hal::Status Stm32AdcChannel::configure(const mc_rtos_hal::AdcChannelConfig &config) {
    if (!bound_ || controller_ == nullptr) {
        return mc_rtos_hal::Status::Unsupported;
    }
    config_ = config;
    configured_ = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32AdcChannel::read_raw(uint16_t &value) {
    if (!bound_ || controller_ == nullptr) {
        value = 0U;
        return mc_rtos_hal::Status::Unsupported;
    }
    return controller_->read_raw(hardware_config_, value);
}

mc_rtos_hal::Status Stm32AdcChannel::read_voltage(float &voltage) {
    uint16_t raw = 0U;
    const mc_rtos_hal::Status status = read_raw(raw);
    if (status != mc_rtos_hal::Status::Ok) {
        voltage = 0.0f;
        return status;
    }

    constexpr float kMaxRaw = 4095.0f;
    voltage = ((static_cast<float>(raw) / kMaxRaw) * config_.reference_voltage) * config_.scale;
    return mc_rtos_hal::Status::Ok;
}

Stm32Adc::Stm32Adc()
    : backend_(),
      channels_(),
      channel_count_(0U),
      initialized_(false) {}

mc_rtos_hal::Status Stm32Adc::configure(const Stm32AdcUnitConfig &unit_config,
                                        const Stm32AdcChannelHardwareConfig *channel_configs,
                                        uint8_t channel_count) {
    if ((channel_configs == nullptr && channel_count != 0U) || channel_count > kMaxChannelCount) {
        return mc_rtos_hal::Status::InvalidArgument;
    }

    backend_ = std::make_unique<Stm32F4xxAdcBackend>();
    mc_rtos_hal::Status status = backend_->init(unit_config);
    if (status != mc_rtos_hal::Status::Ok) {
        initialized_ = false;
        return status;
    }

    for (uint8_t i = 0; i < channel_count; ++i) {
        status = backend_->init_channel(channel_configs[i]);
        if (status != mc_rtos_hal::Status::Ok) {
            initialized_ = false;
            return status;
        }
        channels_[channel_configs[i].logical_index].bind(this, channel_configs[i]);
    }

    channel_count_ = channel_count;
    initialized_ = true;
    return mc_rtos_hal::Status::Ok;
}

mc_rtos_hal::Status Stm32Adc::read_raw(const Stm32AdcChannelHardwareConfig &config, uint16_t &value) {
    if (!initialized_ || !backend_) {
        value = 0U;
        return mc_rtos_hal::Status::Unsupported;
    }
    return backend_->read_raw(config, value);
}

mc_rtos_hal::AdcChannel *Stm32Adc::channel(uint8_t index) {
    if (index >= kMaxChannelCount || !channels_[index].is_bound()) {
        return nullptr;
    }
    return &channels_[index];
}

uint8_t Stm32Adc::channel_count() const {
    return channel_count_;
}

}  // namespace stm32
