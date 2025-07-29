
#pragma once

#include <arch/linux/rpi/AP_HAL_RPI.h>

class RPI::RPIAnalogSource : public AP_HAL::AnalogSource {
	public:
		RPIAnalogSource(float v);

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

class RPI::RPIAnalogIn : public AP_HAL::AnalogIn {
	public:
		RPIAnalogIn();

		void init(void* implspecific);

		AP_HAL::AnalogSource* channel(int16_t n);
};


