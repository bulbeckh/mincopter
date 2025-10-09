
#include <arch/arm/stm32/SPIDriver.h>

using namespace stm32;

STM32SPIDeviceDriver::STM32SPIDeviceDriver(uint32_t speed) :
    _speed(speed)
{}

void STM32SPIDeviceDriver::init()
{
	// TODO These are individual SPI devices and should represent SS pins so
	// the corresponding GPIO setup should really be done here rather than in GPIO.cpp
	//
	// We should add an argument to the above SPIDeviceDriver constructor with the SS pin
	// to use for each SPI device.
}

AP_HAL::Semaphore* STM32SPIDeviceDriver::get_semaphore()
{
	// TODO
	return NULL;
}

void STM32SPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	// TODO
}


void STM32SPIDeviceDriver::cs_assert()
{
	// TODO
}

void STM32SPIDeviceDriver::cs_release()
{
	// TODO
}

uint8_t STM32SPIDeviceDriver::transfer(uint8_t data)
{
	// TODO
    return 0;
}

void STM32SPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
    transaction(data, NULL, len);
}

// TODO Update args for these constructors
STM32SPIDeviceManager::STM32SPIDeviceManager() :
    _device_cs0(2600000),
    _device_cs1(1000000)
{}

void STM32SPIDeviceManager::init(void *)
{
	// TODO Change to be configurable for any number of devices
    _device_cs0.init();
    _device_cs1.init();

	// TODO Make configurable on any SPI
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
        // Error_Handler();
		// TODO Change to hal panic
    }

	return;
}

AP_HAL::SPIDeviceDriver* STM32SPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
	// TODO
	/*
    switch (dev) {
        case AP_HAL::SPIDevice_ADS7844:
            return &_device_cs0;
    }
	*/
    return NULL;
}

