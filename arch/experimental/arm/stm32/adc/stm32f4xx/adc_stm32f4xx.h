#pragma once

#include <arm/stm32/adc/adc_backend.h>

namespace stm32 {

class Stm32F4xxAdcBackend final : public Stm32AdcBackend {
public:
    Stm32F4xxAdcBackend();

    mc_rtos_hal::Status init(const Stm32AdcUnitConfig &config) override;
    mc_rtos_hal::Status init_channel(const Stm32AdcChannelHardwareConfig &config) override;
    mc_rtos_hal::Status read_raw(const Stm32AdcChannelHardwareConfig &config, uint16_t &value) override;

private:
    mc_rtos_hal::Status enable_clock(ADC_TypeDef *instance);
    static void enable_gpio_clock(GPIO_TypeDef *port);
    static mc_rtos_hal::Status map_hal_status(HAL_StatusTypeDef hal_status);

    ADC_HandleTypeDef handle_;
    bool initialized_;
};

}  // namespace stm32
