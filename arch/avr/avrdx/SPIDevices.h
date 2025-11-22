
#pragma once

#include <AP_HAL.h>
#include "avrdx/AP_HAL_AVRDx_Namespace.h"

class AP_HAL_AVRDx::AVRDxSPI0DeviceDriver : public AP_HAL::SPIDeviceDriver {
	public:
		AVRDxSPI0DeviceDriver(
				AP_HAL_AVRDx::AVRDxDigitalSource *cs_pin,
				uint8_t spcr_lowspeed,
				uint8_t spcr_highspeed,
				uint8_t spsr
		) :
			_cs_pin(cs_pin),
			_spcr_lowspeed(spcr_lowspeed),
			_spcr_highspeed(spcr_highspeed),
			_spcr(spcr_lowspeed),
			_spsr(spsr)
		{}

		void init();
		AP_HAL::Semaphore* get_semaphore();

		void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

		void cs_assert();
		void cs_release();
		uint8_t transfer(uint8_t data);
		void transfer(const uint8_t *data, uint16_t len);
		void set_bus_speed(enum bus_speed speed);

	private:
		void _cs_assert();
		void _cs_release();
		uint8_t _transfer(uint8_t data);
		// used for MPU6k
		void _transfer16(const uint8_t *tx, uint8_t *rx);

		static AP_HAL_AVRDx::AVRDxSemaphore _semaphore;
		static bool _force_low_speed;

		AP_HAL_AVRDx::AVRDxDigitalSource *_cs_pin;
		const uint8_t _spcr_lowspeed;
		const uint8_t _spcr_highspeed;
		uint8_t _spcr;
		const uint8_t _spsr;
};


// NOTE Removed SPI2 and SPI3 drivers

