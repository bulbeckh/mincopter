
#include <arch/arm/stm32/GPIO.h>

#include "stm32f4xx_hal.h"

using namespace stm32;


STM32GPIO::STM32GPIO(void)
{
	// TODO
}

void STM32GPIO::init(void)
{
	// TODO This initialisation is very specific to the current configuration of my STM32F407G-DISC1 dev board and sensors.
	// Later this will be defined via a config file/language
	
	/* Initialise each GPIO set */

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// LED Pins on GPIOD (for DISC1)
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// USART2 and SPI Pins on GPIO A
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// USART3 and I2C Pins on GPIO B
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// 1. Setup LED pins
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);



	// 5. TODO Setup USART4
	
	// 6. TODO Setup USART1

	// 7. Setup I2C SDA/SCL pins - PB6, PB7
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	return;
}

void STM32GPIO::pinMode(uint8_t pin, uint8_t output)
{
	// TODO
}

int8_t STM32GPIO::analogPinToDigitalPin(uint8_t pin)
{
	// TODO
	return -1;
}


uint8_t STM32GPIO::read(uint8_t pin) {
	// TODO
    return 0;
}

void STM32GPIO::write(uint8_t pin, uint8_t value)
{
	// TODO
}

void STM32GPIO::toggle(uint8_t pin)
{
	// TODO
}

AP_HAL::DigitalSource* STM32GPIO::channel(uint16_t n) {
	// TODO Add bounds checks
	return &_gpio_array[n];
}

/* Interrupt interface: */
bool STM32GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) {
	// TODO
    return true;
}

bool STM32GPIO::usb_connected(void)
{
	// TODO USB Not yet implemented on STM32 target
    return false;
}

// STM32DigitalSource methods

STM32DigitalSource::STM32DigitalSource(void)
{
}

void STM32DigitalSource::mode(uint8_t output)
{
	/* Although the pins on the GPIO can have a number of alternate functions like
	 * SPI, I2C, etc, most of the genuine GPIO pins will either be GPIO_MODE_INPUT
	 * or GPIO_MODE_OUTPUT_PP so we will assign either of those as the mode depending
	 * on the **output** parameter */
	if (output) {
		_mode = GPIO_MODE_OUTPUT_PP;
	} else {
		_mode = GPIO_MODE_INPUT;
	}

	return;
}

uint8_t STM32DigitalSource::read(void) {
    GPIO_PinState pinstate = HAL_GPIO_ReadPin(_bus, _pin);

	if (pinstate==GPIO_PIN_SET) {
		return 0xFF;
	} else {
		return 0x00;
	}
}

void STM32DigitalSource::write(uint8_t value)
{
	// Write HIGH for any non-zero value
	if (value) {
		HAL_GPIO_WritePin(_bus, _pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(_bus, _pin, GPIO_PIN_RESET);
	}

	return;
}

void STM32DigitalSource::toggle(void)
{
	HAL_GPIO_TogglePin(_bus, _pin);

	return;
}


