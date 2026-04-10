#include <arm/stm32/time.h>

#include <cmsis_gcc.h>
#include <task.h>

#include <stm32f4xx_hal.h>

namespace stm32 {

namespace {

TickType_t ms_to_ticks(uint32_t delay_ms) {
    if (delay_ms == 0U) {
        return 0;
    }

    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    if (ticks == 0) {
        ticks = 1;
    }
    return ticks;
}

uint32_t ticks_to_ms(TickType_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000ULL) / configTICK_RATE_HZ);
}

bool scheduler_running() {
    return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}

bool cycle_counter_enabled() {
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

uint32_t micros_from_cycle_counter() {
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        return HAL_GetTick() * 1000U;
    }

    return static_cast<uint32_t>(static_cast<uint64_t>(DWT->CYCCNT) / cycles_per_us);
}

}  // namespace

Stm32Time::Stm32Time() {
    ensure_cycle_counter_ready();
}

void Stm32Time::ensure_cycle_counter_ready() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

bool Stm32Time::cycle_counter_ready() {
    return cycle_counter_enabled();
}

uint32_t Stm32Time::millis() const {
    if (scheduler_running() && (__get_IPSR() == 0U)) {
        return ticks_to_ms(xTaskGetTickCount());
    }

    if (scheduler_running()) {
        return ticks_to_ms(xTaskGetTickCountFromISR());
    }

    return HAL_GetTick();
}

uint32_t Stm32Time::micros() const {
    return cycle_counter_ready() ? micros_from_cycle_counter() : (HAL_GetTick() * 1000U);
}

void Stm32Time::delay_ms(uint32_t delay_ms) {
    if (delay_ms == 0U) {
        return;
    }

    if (__get_IPSR() != 0U) {
        busy_wait_us(delay_ms * 1000U);
        return;
    }

    if (scheduler_running()) {
        vTaskDelay(ms_to_ticks(delay_ms));
        return;
    }

    HAL_Delay(delay_ms);
}

void Stm32Time::delay_until_ms(uint32_t &last_wake_ms, uint32_t period_ms) {
    if (period_ms == 0U) {
        return;
    }

    if (scheduler_running() && (__get_IPSR() == 0U)) {
        TickType_t last_wake_tick = pdMS_TO_TICKS(last_wake_ms);
        const TickType_t period_ticks = ms_to_ticks(period_ms);
        vTaskDelayUntil(&last_wake_tick, period_ticks);
        last_wake_ms = ticks_to_ms(last_wake_tick);
        return;
    }

    const uint32_t next_wake_ms = last_wake_ms + period_ms;
    const uint32_t now_ms = millis();
    if ((next_wake_ms - now_ms) <= period_ms) {
        delay_ms(next_wake_ms - now_ms);
    }
    last_wake_ms = next_wake_ms;
}

void Stm32Time::busy_wait_us(uint32_t delay_us) {
    if (delay_us == 0U) {
        return;
    }

    if (!cycle_counter_ready()) {
        const uint32_t start_ms = HAL_GetTick();
        const uint32_t delay_ms = (delay_us + 999U) / 1000U;
        while ((HAL_GetTick() - start_ms) < delay_ms) {
        }
        return;
    }

    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        return;
    }

    const uint32_t start_cycles = DWT->CYCCNT;
    const uint32_t delay_cycles = static_cast<uint32_t>(static_cast<uint64_t>(delay_us) * cycles_per_us);
    while ((DWT->CYCCNT - start_cycles) < delay_cycles) {
    }
}

}  // namespace stm32
