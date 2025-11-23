
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

		/// OVERRIDE
		/* @brief Sets the ADC channel as an input */
		void  set_pin(uint8_t) override;

		float voltage_average(void) override;
		float voltage_latest(void) override;
		float voltage_average_ratiometric(void) override;
		void  set_settle_time(uint16_t settle_time_ms) override;

	private:
		/* @brief Called with value of ADC measurments, from interrput */
		void new_sample(uint16_t);

		/* @brief Called to setup ADC registers for next measurment, from interrupt */
		void setup_read(void);

		/* @brief Called to check if we have read for long enough */
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

		/* @brief Contains the ADC number for this AnalogSource (i.e. ADC0, ADC3). Alternatively, will be ANALOG_INPUT_NONE if not yet
		 * configured, or in the special case of the VCC AnalogSource, will be ANALOG_INPUT_BOARD_VCC */
		uint8_t _pin;

		uint16_t _settle_time_ms;
		uint32_t _read_start_time_ms;
};

class AP_HAL_AVRDx::AVRDxAnalogIn : public AP_HAL::AnalogIn {
	public:
		AVRDxAnalogIn(void);

		/* @brief AnalogIn Interface Methods */
		void init(void* ap_hal_scheduler) override;

		/* @brief Returns a pointer to an AnalogSource representing a particular ADC peripheral, for
		 * example ADC0 or ADC4.
		 *
		 * For MinCopter v.01, we support boards with up to 8 ADC channels. To avoid dynamic allocation,
		 * we instantiate all 8 channels during initialisation and return the AnalogSource as an offset
		 * into our array.
		 *
		 * @param channel Channel is an ADC number (from 0 to AVR_INPUT_MAX_CHANNELS-1) or the AVR_INPUT_BOARD_VCC */
		AP_HAL::AnalogSource* channel(int16_t channel) override;

	private: 
		/* @brief Sets up another ADC channel */
		AVRDxAnalogSource* _register_channel(int16_t);

		/* @brief Runs at 1kHz to read/setup ADC reads across all channels */
		void _timer_event(void);

		/* @brief Array of AnalogSource channels (+VCC channel) */
		AVRDxAnalogSource _channels[AVR_INPUT_MAX_CHANNELS + 1];

		/* @brief Special AnalogSource representing VCC voltage */
		AVRDxAnalogSource* _vcc;

	private:
		/* @brief Number of channels used */
		int16_t _num_channels;
		
		/* @brief The channel we are currently reading. Incremented by the _timer_event */
		int16_t _active_channel;

		/* @brief How many reads we have done for this channel. Max reads is defined by macro */
		uint16_t _channel_repeat_count;

};

