
#pragma once

#include <AP_HAL.h>

#include "avrdx/AP_HAL_AVRDx_Namespace.h"

// NOTE We support 8 ADC inputs for MinCopter v1.
#define AVR_INPUT_MAX_CHANNELS 8

class AP_HAL_AVRDx::AVRDxAnalogSource : public AP_HAL::AnalogSource {
	public:
		friend class AP_HAL_AVRDx::AVRDxAnalogIn;

		/* @brief Constructor. pin designates the ADC input number, or when == AVR_ANALOG_PIN_VCC,
		 * the board vcc */
		AVRDxAnalogSource(uint8_t pin);

		/* @brief AnalogSource Interface Methods */
		float read_average(void) override;
		float read_latest(void) override;
		void set_pin(uint8_t) override;
		float voltage_average(void) override;
		float voltage_latest(void) override;
		float voltage_average_ratiometric(void) override;
		void set_stop_pin(uint8_t p) override;
		void set_settle_time(uint16_t settle_time_ms) override;

	private:
		/* new_sample(): called with value of ADC measurments, from interrput */
		void new_sample(uint16_t);

		/* setup_read(): called to setup ADC registers for next measurment,
		 * from interrupt */
		void setup_read(void);

		/* stop_read(): called to stop device measurement */
		void stop_read(void);

		/* reading_settled(): called to check if we have read for long enough */
		bool reading_settled(void);

		/* read_average: called to calculate and clear the internal average.
		 * implements read_average(), unscaled. */
		float _read_average(void);

		int16_t get_pin(void) { return _pin; };

	private:
		/* following three are used from both an interrupt and normal thread */
		volatile uint8_t _sum_count;
		volatile uint16_t _sum;
		volatile uint16_t _latest;
		float _last_average;

		/* _pin designates the ADC input mux for the sample */
		uint8_t _pin;

		/* _stop_pin designates a digital pin to use for
		   enabling/disabling the analog device */
		uint8_t _stop_pin;
		uint16_t _settle_time_ms;
		uint32_t _read_start_time_ms;
};

class AP_HAL_AVRDx::AVRDxAnalogIn : public AP_HAL::AnalogIn {
	public:
		AVRDxAnalogIn(void);

		void init(void* ap_hal_scheduler) override;
		AP_HAL::AnalogSource* channel(int16_t n) override;

	private: 
		// Private methods return a pointer to derived but our public methods (::channel) return base pointer
		AVRDxAnalogSource* _create_channel(int16_t num);
		AVRDxAnalogSource* _register_channel(int16_t);

		void _timer_event(void);

		AVRDxAnalogSource _channels[AVR_INPUT_MAX_CHANNELS];

		int16_t _num_channels;
		int16_t _active_channel;
		uint16_t _channel_repeat_count;

	private:
		AVRDxAnalogSource _vcc;
};

