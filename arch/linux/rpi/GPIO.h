
#pragma once

#include <arch/linux/rpi/AP_HAL_RPI.h>

/* RaspberryPI GPIO Interface */

class RPI::RPIGPIO : public AP_HAL::GPIO {
	public:

		RPIGPIO();

		void    init();

		void    pinMode(uint8_t pin, uint8_t output);

		int8_t  analogPinToDigitalPin(uint8_t pin);

		uint8_t read(uint8_t pin);

		void    write(uint8_t pin, uint8_t value);

		void    toggle(uint8_t pin);

		/* Alternative interface: */
		AP_HAL::DigitalSource* channel(uint16_t n);

		/* Interrupt interface: */
		bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode);

		/* return true if USB cable is connected */
		bool    usb_connected(void);
};

class RPI::RPIDigitalSource : public AP_HAL::DigitalSource {
	public:

		RPIDigitalSource(uint8_t v);

		void    mode(uint8_t output);

		uint8_t read();

		void    write(uint8_t value); 

		void    toggle();

	private:

		uint8_t _v;
};



