
#include <arch/arm/stm32/I2CDriver.h>

using namespace stm32;

// TODO Some of these data members we don't need
STM32I2CDriver::STM32I2CDriver(AP_HAL::Semaphore* semaphore, const char *device)
	: _semaphore(semaphore)
{
}

void STM32I2CDriver::begin() 
{
	// TODO Make chosen I2C configurable
	
	__HAL_RCC_I2C1_CLK_ENABLE();

	// TODO Is 100kHz the correct speed here
    mc_i2c.Instance = I2C1;
    mc_i2c.Init.ClockSpeed = 100000;           // 100 kHz
    mc_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    mc_i2c.Init.OwnAddress1 = 0;               // MCU address if used as slave
    mc_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    mc_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    mc_i2c.Init.OwnAddress2 = 0;
    mc_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    mc_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&mc_i2c) != HAL_OK) {
        //Error_Handler();
		// TODO Change to hal panic
    }

	return;
}

void STM32I2CDriver::end() 
{
	// TODO
}

void STM32I2CDriver::setTimeout(uint16_t ms) 
{
    // TODO
}

void STM32I2CDriver::setHighSpeed(bool active) 
{
    // TODO    
}

uint8_t STM32I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
	// TODO
	return 0;
}


uint8_t STM32I2CDriver::writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
	// TODO
	return 0;
}

uint8_t STM32I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
	// TODO
	return 0;
}

uint8_t STM32I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
	// TODO
    return 0;
}

uint8_t STM32I2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
	// TODO
    return 0;
}


uint8_t STM32I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	// TODO
    return 0;
}

uint8_t STM32I2CDriver::lockup_count() 
{
	// TODO
    return 0;
}
