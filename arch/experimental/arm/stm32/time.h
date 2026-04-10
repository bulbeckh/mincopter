#pragma once

#include <stdint.h>

#include <FreeRTOS.h>

#include <rtos_hal/time.h>

namespace stm32 {

class Stm32Time final : public mc_rtos_hal::Time {
public:
    Stm32Time();

    uint32_t millis() const override;
    uint32_t micros() const override;
    void delay_ms(uint32_t delay_ms) override;
    void delay_until_ms(uint32_t &last_wake_ms, uint32_t period_ms) override;
    void busy_wait_us(uint32_t delay_us) override;

private:
    static void ensure_cycle_counter_ready();
    static bool cycle_counter_ready();
};

}  // namespace stm32
