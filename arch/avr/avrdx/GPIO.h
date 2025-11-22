
#pragma once

#include <AP_HAL.h>
#include "AP_HAL_AVRDx_Namespace.h"

// TODO Why are these defined here
#define HAL_GPIO_A_LED_PIN        27
#define HAL_GPIO_B_LED_PIN        26
#define HAL_GPIO_C_LED_PIN        25
#define HAL_GPIO_LED_ON           LOW
#define HAL_GPIO_LED_OFF          HIGH

// TODO Is this part of the MinCopter v1 interface??
#define HAL_GPIO_USB_MUX_PIN	    23

#define AVR_DIGITALSOURCE_MAX 4

class AP_HAL_AVRDx::AVRDxDigitalSource : public AP_HAL::DigitalSource {

	public:
	  AVRDxDigitalSource(uint8_t bit, uint8_t port) : _bit(bit), _port(port) {}
	  void    mode(uint8_t output);
	  uint8_t read();
	  void    write(uint8_t value);
	  void    toggle();

	  void set_bit(uint8_t bit);
	  void set_port(uint8_t port); 

	private:

	  uint8_t _bit;
	  uint8_t _port;

};

class AP_HAL_AVRDx::AVRDxGPIO : public AP_HAL::GPIO {

	public:
		AVRDxGPIO(void);

		void    init(void) {}
		void    pinMode(uint8_t pin, uint8_t output);
		int8_t  analogPinToDigitalPin(uint8_t pin);
		uint8_t read(uint8_t pin);
		void    write(uint8_t pin, uint8_t value);
		void    toggle(uint8_t pin);
		AP_HAL::DigitalSource* channel(uint16_t);
		bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc proc,
				uint8_t mode);

		/* return true if USB cable is connected */
		bool    usb_connected(void);

	/* private-ish: only to be used from the appropriate interrupt */
		static AP_HAL::Proc _interrupt_6;

	private:

		/* @brief Index to the next free digital source */
		uint8_t _head;

		AVRDxDigitalSource _sources[AVR_DIGITALSOURCE_MAX];

};


