
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

#include "stm32f4xx_hal.h"

class stm32::STM32I2CDriver : public AP_HAL::I2CDriver {
	public:
		STM32I2CDriver(AP_HAL::Semaphore* semaphore, const char *device);

		void begin();

		void end();

		void setTimeout(uint16_t ms);

		void setHighSpeed(bool active);

		/* write: for i2c devices which do not obey register conventions */
		uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
		/* writeRegister: write a single 8-bit value to a register */
		uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
		/* writeRegisters: write bytes to contigious registers */
		uint8_t writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);

		/* read: for i2c devices which do not obey register conventions */
		uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
		/* readRegister: read from a device register - writes the register,
		 * then reads back an 8-bit value. */
		uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);
		/* readRegister: read contigious device registers - writes the first 
		 * register, then reads back multiple bytes */
		uint8_t readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);

		uint8_t lockup_count();

		AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

	private:
		/* @brief HAL I2C instance */
		I2C_HandleTypeDef mc_i2c;

		AP_HAL::Semaphore* _semaphore;

		/*
		int _fd;
		uint8_t _addr;
		const char *_device;
		*/
};

