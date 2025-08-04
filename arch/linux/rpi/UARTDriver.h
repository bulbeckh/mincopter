
#pragma once

#include <arch/linux/rpi/AP_HAL_RPI.h>

class RPI::RPIUARTDriver : public AP_HAL::UARTDriver {
	public:
		RPIUARTDriver(const char* device_path);

		/* Overrides */
		void begin(uint32_t b);
		void begin(uint32_t b, uint16_t rxS, uint16_t txS);
		void end();
		void flush();
		bool is_initialized();
		void set_blocking_writes(bool blocking);
		bool tx_pending();

		/* Linux implementations of Stream virtual methods */
		int16_t available();
		int16_t txspace();
		int16_t read();

		/* Linux implementations of Print virtual methods */
		size_t write(uint8_t c);
		size_t write(const uint8_t *buffer, size_t size);

		//void set_device_path(const char *path);

		//void _timer_tick(void);

	private:

		/* @brief The path to the device file on rpi */
		const char* _device;

		/* This drivers pigpio daemon connection */
		int32_t _pi_ref;

		/* Handle for the serial connection to pigpio daemon */
		int32_t _handle;

		int32_t _baudrate;

		bool _nonblocking_writes;

		/* Whether we have initialised this UART driver */
		volatile bool _initialised;

};


