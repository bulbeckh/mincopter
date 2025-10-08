
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

class stm32::STM32RCOutput : public AP_HAL::RCOutput {
	public:
		void     init(void* machtnichts);

		void     set_freq(uint32_t chmask, uint16_t freq_hz);

		uint16_t get_freq(uint8_t ch);

		void     enable_ch(uint8_t ch);

		void     disable_ch(uint8_t ch);

		void     write(uint8_t ch, uint16_t period_us);

		void     write(uint8_t ch, uint16_t* period_us, uint8_t len);

		uint16_t read(uint8_t ch);

		void     read(uint16_t* period_us, uint8_t len);

};

