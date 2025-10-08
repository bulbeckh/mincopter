
#pragma once

// Components for STM32 MinCopter targets

namespace stm32 {
	class STM32UARTDriver;
	class STM32I2CDriver;
	class STM32SPIDeviceManager;
	class STM32SPIDeviceDriver;
	class STM32AnalogSource;
	class STM32AnalogIn;
	class STM32GPIO;
	class STM32RCInput;
	class STM32RCOutput;
	class STM32DigitalSource;
	class STM32Semaphore;
	class STM32Scheduler;
	class STM32Util;

	/* No AP_HAL::Storage class for STM32 targets */
	/* No AP_HAL::Sim class for STM32 targets */
}


