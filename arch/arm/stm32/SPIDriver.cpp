
#include <arch/arm/stm32/SPIDriver.h>

using namespace stm32;

extern const AP_HAL::HAL& hal;

STM32SPIDeviceDriver::STM32SPIDeviceDriver(SPI_HandleTypeDef* _handle, GPIO_TypeDef* bus, uint16_t pin) :
	_spi{_handle},
	_bus{bus},
	_pin{pin}
{ }

void STM32SPIDeviceDriver::init(void)
{
	// TODO These are individual SPI devices and should represent SS pins so
	// the corresponding GPIO setup should really be done here rather than in GPIO.cpp
	//
	// We should add an argument to the above SPIDeviceDriver constructor with the SS pin
	// to use for each SPI device.
	
	// TODO Fix this
	if (_bus==GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (_bus==GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	} else if (_bus==GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	} else if (_bus==GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = _pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(_bus, &GPIO_InitStruct);

	// Write high to disable CS by default
	HAL_GPIO_WritePin(_bus, _pin, GPIO_PIN_SET);

	return;
}

AP_HAL::Semaphore* STM32SPIDeviceDriver::get_semaphore()
{
	return &_semaphore;
}

void STM32SPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	if (HAL_SPI_TransmitReceive(_spi, tx, rx, len, HAL_MAX_DELAY) != HAL_OK) {
		hal.scheduler->panic("bad transaction\r\n");
	}

	return;
}


void STM32SPIDeviceDriver::cs_assert()
{
	// TODO Semaphore checks
	
	// Pull CS pin low to activate CS
	HAL_GPIO_WritePin(_bus, _pin, GPIO_PIN_RESET);

	return;
}

void STM32SPIDeviceDriver::cs_release()
{
	// TODO Semaphore checks
	
	// Pull CS pin high to de-select pin
	HAL_GPIO_WritePin(_bus, _pin, GPIO_PIN_SET);

	return;
}

uint8_t STM32SPIDeviceDriver::transfer(uint8_t data)
{
	// TODO NOTE is the last argument in ms?
	
	// Attempt to transfer 1 byte with a 1s timeout
	if (HAL_SPI_Transmit(_spi, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		hal.scheduler->panic("bad write\r\n");
	}

	// TODO What should the return value be here
    return 0;
}

void STM32SPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
	// Attempt to transfer 1 byte with a 1s timeout
	if (HAL_SPI_Transmit(_spi, data, len, HAL_MAX_DELAY) != HAL_OK) {
		hal.scheduler->panic("bad write\r\n");
	}

	return;
}

// TODO Update args for these constructors
STM32SPIDeviceManager::STM32SPIDeviceManager() :
	// This is where we specify the SPI devices and the SS bus/pin
	_device_icm20948{&mc_spi, GPIOA, GPIO_PIN_4}
{}

void STM32SPIDeviceManager::init(void*)
{
	// TODO Change to be configurable for any number of devices
    _device_icm20948.init();

	// TODO Make configurable on any SPI
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	// Setup no-CS GPIO pins for SPI1 (SCK, MOSI, MISO)
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Setup SPI
	__HAL_RCC_SPI1_CLK_ENABLE();

	mc_spi.Instance = SPI1;
    mc_spi.Init.Mode = SPI_MODE_MASTER;                 // Master mode
    mc_spi.Init.Direction = SPI_DIRECTION_2LINES;       // Full duplex
    mc_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    mc_spi.Init.CLKPolarity = SPI_POLARITY_LOW;         // Clock idle low
    mc_spi.Init.CLKPhase = SPI_PHASE_1EDGE;             // Data captured on 1st edge
    mc_spi.Init.NSS = SPI_NSS_SOFT;                     // Software control of CS
    mc_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // Clock divider
    mc_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    mc_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    mc_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    mc_spi.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&mc_spi) != HAL_OK) {
		hal.scheduler->panic("Bad init\r\n");
    }

	return;
}

AP_HAL::SPIDeviceDriver* STM32SPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
	// TODO Add remaining SPI devices
	// TODO Make configurable
    switch (dev) {
		case AP_HAL::SPIDevice_ICM20948:
            return &_device_icm20948;
    }

    return NULL;
}

