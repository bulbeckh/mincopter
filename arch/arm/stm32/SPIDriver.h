
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>
#include <arch/arm/stm32/Semaphores.h>

#include "stm32f4xx_hal.h"

// TODO The design of this function could be better - for instance we store a semaphore object
// for each instance of an SPI device and we also store a pointer to the spi_handle for each device

class stm32::STM32SPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
	public:
		STM32SPIDeviceDriver(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);

		/* @brief Initialise this SPI device SS pin */
		void init(void);

		// TODO Semaphore checks not yet implemented as we only have 1 SPI device
		/* @brief Return the semaphore for this SPI device */
		AP_HAL::Semaphore* get_semaphore(void);

		/* @brief Send and receive data over SPI to this device */
		void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

		/* @brief Enable SPI transaction for this device */
		void cs_assert(void);

		/* @brief Stop SPI transaction for this device */
		void cs_release(void);

		/* @brief Write a single byte of data to this device */
		uint8_t transfer(uint8_t data);

		/* @brief Write a contiguous array of data to this device */
		void transfer(const uint8_t *data, uint16_t len);

	private:
		// TODO Shouldn't the semaphore be on the SPIDeviceManager class rather than the object
		/* @brief Semaphore object for this device */
		stm32::STM32Semaphore _semaphore;

		//uint32_t _speed;
		
		/* @brief The bus for this CS pin (i.e. GPIOA, GPIOB, ..) */
		GPIO_TypeDef* _bus;

		/* @brief The pin number for this CS pin (i.e. GPIO_PIN_2, GPIO_PIN_3, ..) */
		uint16_t _pin;

		/* @brief Pointer to the SPI handle used for sending/receiving data over SPI */
		SPI_HandleTypeDef* _spi;
};

class stm32::STM32SPIDeviceManager : public AP_HAL::SPIDeviceManager {

	public:
		STM32SPIDeviceManager(void);

		/* @brief Setup SPI peripheral - called during HAL initialisation */
		void init(void*);

		/* @brief Retrieve the device driver (CS) for a particular SPI device */
		AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

	private:
		/* @brief SPI HAL Configuration instance */
		SPI_HandleTypeDef mc_spi;

		// TODO Add configuration for more than 2 SPI devices
		// TODO Change the names of these - leftover from RPI names (i.e /dev/cs0)
		STM32SPIDeviceDriver _device_icm20948;
};

