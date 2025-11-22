
#pragma once

namespace AP_HAL_AVRDx {

	// TODO Update these names for the Dx-series
    class HAL_AVRDx;

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

