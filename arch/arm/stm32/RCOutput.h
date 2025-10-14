
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

#include "stm32f4xx_hal.h"

#define STM32_RCOUT_MAX_CHANNELS 4

class stm32::STM32RCOutput : public AP_HAL::RCOutput {

	public:
		STM32RCOutput(void);

		/* @brief Initialise the RCOutput PWM timer and signal generation */
		void init(void*);

		// TODO Implementation
		void set_freq(uint32_t chmask, uint16_t freq_hz);

		/* @brief Returns the PWM output frequency in Hz. NOTE currently locked at 50Hz until we implement frequency setting functionality */
		uint16_t get_freq(uint8_t ch);

		/* @brief Enable channel to have PWM signal written to */
		void enable_ch(uint8_t ch);

		/* @brief Stop writing PWM signal to channel */ 
		void disable_ch(uint8_t ch);

		/* @brief Set the PWM period to a channel */
		void write(uint8_t ch, uint16_t period_us);

		// TODO Not implemented (also not used - marked for removal)
		/* @brief Bulk write PWM periods to a contiguous set of channels */
		void write(uint8_t ch, uint16_t* period_us, uint8_t len);

		/* @brief Read the PWM value (in us) of a specific channel */
		uint16_t read(uint8_t ch);

		/* @brief Bulk read the PWM value (in us) of set of channels */
		void read(uint16_t* period_us, uint8_t len);

	private:

		/* @brief Whether this channel is currently enabled (writing a PWM signal to pin) */
		bool _enable_mask[STM32_RCOUT_MAX_CHANNELS];

		/* @brief Array of current PWM signals for each channel */
		uint16_t _channel[STM32_RCOUT_MAX_CHANNELS];

		/* @brief TIM3 handle for initialisation */
		TIM_HandleTypeDef rcout_handle;

};

