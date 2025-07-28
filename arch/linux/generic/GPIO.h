
#pragma once

#include <arch/linux/generic/AP_HAL_Generic.h>

class generic::GenericGPIO : public AP_HAL::GPIO {
	public:
		GenericGPIO();
		void    init();
		void    pinMode(uint8_t pin, uint8_t output);
		int8_t  analogPinToDigitalPin(uint8_t pin);
		uint8_t read(uint8_t pin);
		void    write(uint8_t pin, uint8_t value);
		void    toggle(uint8_t pin);

		/* Alternative interface: */
		AP_HAL::DigitalSource* channel(uint16_t n);

		/* Interrupt interface: */
		bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
				uint8_t mode);

		/* return true if USB cable is connected */
		bool    usb_connected(void);
};

class generic::GenericDigitalSource : public AP_HAL::DigitalSource {
	public:
		GenericDigitalSource(uint8_t v);
		void    mode(uint8_t output);
		uint8_t read();
		void    write(uint8_t value); 
		void    toggle();
	private:
		uint8_t _v;
};

