
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

#include "stm32f4xx_hal.h"

#define STM32GPIO_MAX_PINS 16

class stm32::STM32DigitalSource : public AP_HAL::DigitalSource {

	public:
		STM32DigitalSource(void);

		// TODO We need to re-think how a GPIO pin is retrieved and initialised by other peripherals/drivers

		/* @brief Set the mode of the GPIO pin. **output** argument is 1 for output and 0 for input */
		void mode(uint8_t output);

		/* @brief Read the value of this GPIO pin */
		uint8_t read(void);

		/* @brief Write value to this GPIO pin */
		void write(uint8_t value); 

		/* @brief Toggle this GPIO pin */
		void toggle(void);
	
	private:
		/* @brief The GPIO bus that this pin uses (i.e. GPIOA, GPIOB, ...) */
		GPIO_TypeDef* _bus;

		/* @brief The GPIO on the bus (i.e. GPIO_PIN_12) */
		uint16_t _pin;

		/* @brief The mode of this GPIO pin as defined in the HAL_GPIO_Init configuration (GPIO_MODE_INPUT, GPIO_MODE_AF_PP, ..) */
		uint32_t _mode;

		/* @brief Reference to the handle used to initialise this GPIO pin */
		GPIO_InitTypeDef digitalsource_handle;

};

class stm32::STM32GPIO : public AP_HAL::GPIO {

	public:
		STM32GPIO(void);

		/* @brief Initialise GPIO HAL peripheral and GPIO clocks */
		void init(void);

		void    pinMode(uint8_t pin, uint8_t output);

		int8_t  analogPinToDigitalPin(uint8_t pin);

		uint8_t read(uint8_t pin);

		void    write(uint8_t pin, uint8_t value);

		void    toggle(uint8_t pin);

		AP_HAL::DigitalSource* channel(uint16_t n);

		/* Interrupt interface: */
		bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode);

		/* @brief Return true if USB cable is connected */
		bool usb_connected(void);

	private:
		/* @brief Array of GPIO pins that we have configured and can control */
		STM32DigitalSource _gpio_array[STM32GPIO_MAX_PINS];

};


