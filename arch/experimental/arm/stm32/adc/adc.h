#pragma once

#include <memory>

#include <arm/stm32/adc/adc_backend.h>
#include <rtos_hal/adc.h>

namespace stm32 {

class Stm32Adc;

class Stm32AdcChannel final : public mc_rtos_hal::AdcChannel {
public:
    Stm32AdcChannel();

    void bind(Stm32Adc *controller, const Stm32AdcChannelHardwareConfig &hardware_config);
    bool is_bound() const;

    mc_rtos_hal::Status configure(const mc_rtos_hal::AdcChannelConfig &config) override;
    mc_rtos_hal::Status read_raw(uint16_t &value) override;
    mc_rtos_hal::Status read_voltage(float &voltage) override;

private:
    Stm32Adc *controller_;
    Stm32AdcChannelHardwareConfig hardware_config_;
    mc_rtos_hal::AdcChannelConfig config_;
    bool bound_;
    bool configured_;
};

class Stm32Adc final : public mc_rtos_hal::Adc {
public:
    static constexpr uint8_t kMaxChannelCount = 8;

    Stm32Adc();

    mc_rtos_hal::Status configure(const Stm32AdcUnitConfig &unit_config,
                                  const Stm32AdcChannelHardwareConfig *channel_configs,
                                  uint8_t channel_count);
    mc_rtos_hal::Status read_raw(const Stm32AdcChannelHardwareConfig &config, uint16_t &value);

    mc_rtos_hal::AdcChannel *channel(uint8_t index) override;
    uint8_t channel_count() const override;

private:
    std::unique_ptr<Stm32AdcBackend> backend_;
    Stm32AdcChannel channels_[kMaxChannelCount];
    uint8_t channel_count_;
    bool initialized_;
};

}  // namespace stm32
