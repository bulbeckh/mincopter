
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

class stm32::STM32AnalogSource : public AP_HAL::AnalogSource {
	public:
		STM32AnalogSource(float v);

		float read_average();

		float read_latest();

		void set_pin(uint8_t p);

		void set_stop_pin(uint8_t p);

		void set_settle_time(uint16_t settle_time_ms);

		float voltage_average();

		float voltage_latest();

		float voltage_average_ratiometric() { return voltage_average(); }

	private:
		float _v;
};

class stm32::STM32AnalogIn : public AP_HAL::AnalogIn {
	public:
		STM32AnalogIn();

		void init(void* implspecific);

		AP_HAL::AnalogSource* channel(int16_t n);

};



