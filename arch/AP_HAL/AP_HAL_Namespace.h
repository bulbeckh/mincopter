
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

#include "string.h"

#ifdef USE_NEW_DELEGATE
	#include <AP_HAL/utility/NewDelegate.h>
#else
	#include <AP_HAL/utility/FastDelegate.h>
#endif

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;

    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class DigitalSource;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;

	class Sim;
    
    class Util;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Member Procedure) 

       For member functions we use the FastDelegate delegates class
       which allows us to encapculate a member function as a type
     */
    typedef void(*Proc)(void);

#ifdef USE_NEW_DELEGATE
	typedef Delegate<void(void)> MemberProc;
#else
    typedef fastdelegate::FastDelegate0<> MemberProc;
#endif

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDevice {
        SPIDevice_Dataflash,
        SPIDevice_ADS7844,
        SPIDevice_MS5611,
        SPIDevice_MPU6000,
		SPIDevice_ICM20948
    };

}

#ifdef USE_NEW_DELEGATE
	#define AP_HAL_MEMBERPROC(cname, func) Delegate<void(void)>::Create<cname,func>(this)
#else
	// macro to hide the details of AP_HAL::MemberProc
	#define AP_HAL_MEMBERPROC(cname, func) fastdelegate::MakeDelegate(this, func)
#endif

#endif // __AP_HAL_NAMESPACE_H__
