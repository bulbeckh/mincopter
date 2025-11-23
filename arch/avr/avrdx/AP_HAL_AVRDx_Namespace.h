
#pragma once

namespace AP_HAL_AVRDx {

	// TODO NOTE The actual HAL class is not part of the namespace
	// TODO NOTE We should question why these should even be encapsulated in a namespace at all if they each
	// have different names to other HALs classes anyway?
    //class HAL_AVRDx;

    class AVRDxUARTDriver;
    class AVRDxI2CDriver;

	// NOTE We have 2 SPIs for DB-series boards
    class AVRDxSPIDeviceManager;

    class AVRDxSPI0DeviceDriver;
    class AVRDxSPI1DeviceDriver;

    class AVRDxAnalogSource;
    class AVRDxAnalogIn;

	// NOTE The AVRDB128xx series have 512b of onboard EEPROM storage we can use
    class AVRDxEEPROMStorage;

    class AVRDxGPIO;
    class AVRDxDigitalSource;

	// TODO We don't use RCInput for MinCopter v1
    class AVRDxRCInput;
    class AVRDxRCOutput;

    class AVRDxScheduler;
    class AVRDxTimer;
    class AVRDxSemaphore;

    class ISRRegistry;

    class AVRDxUtil;
}

