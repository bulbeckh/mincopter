
#pragma once

/* Namespace for Generic interface which contains each peripheral driver or abstraction. */

namespace generic {
    class GenericUARTDriver;
    class GenericI2CDriver;
    class GenericSPIDeviceManager;
    class GenericSPIDeviceDriver;
    class GenericAnalogSource;
    class GenericAnalogIn;
    class GenericGPIO;
    class GenericRCInput;
    class GenericRCOutput;
    class GenericDigitalSource;

	// NOTE New gazebo interface class specifically for the Generic HAL
	class GenericGZInterface;
}


