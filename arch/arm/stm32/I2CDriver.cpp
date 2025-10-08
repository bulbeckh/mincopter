
#include <arch/arm/stm32/I2CDriver.h>

using namespace stm32;

// TODO Some of these data members we don't need
STM32I2CDriver::STM32I2CDriver(AP_HAL::Semaphore* semaphore, const char *device) : 
    _semaphore(semaphore),
    _fd(-1),
    _device(device)
{
}

void STM32I2CDriver::begin() 
{
	// TODO
}

void STM32I2CDriver::end() 
{
	// TODO
}

bool STM32I2CDriver::set_address(uint8_t addr)
{
	// TODO
	return true;
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
