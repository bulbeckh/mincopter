
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

  	// 2. Setup SPI Pins
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure CS pin (ICM20948)
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	// Configure other CS pins
	// TODO

	// 3. Setup USART2 TX/RX Pins - PA2, PA3
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 4. Setup USART3 TX/RX Pins - PB10, PB11
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	// TODO
    return new STM32DigitalSource(0);
}

/* Interrupt interface: */
bool STM32GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) {
	// TODO
    return true;
}

bool STM32GPIO::usb_connected(void)
{
	// TODO
    return false;
}

STM32DigitalSource::STM32DigitalSource(uint8_t v) :
    _v(v)
{
	// TODO
}

void STM32DigitalSource::mode(uint8_t output)
{
	// TODO
}

uint8_t STM32DigitalSource::read() {
	// TODO
    return _v;
}

void STM32DigitalSource::write(uint8_t value) {
	// TODO
    _v = value;
}

void STM32DigitalSource::toggle() {
	// TODO
    _v = !_v;
}


