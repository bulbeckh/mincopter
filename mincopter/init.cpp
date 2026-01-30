
#include "mcinstance.h"
#include "mcstate.h"

#ifdef TARGET_ARCH_LINUX
    #include "simulation_logger.h"
#endif

extern MCInstance mincopter;

#include "planner.h"

#include "util.h"
#include "log.h"

void init_ardupilot(void)
{
	mincopter.hal.console->printf_P(PSTR("[INIT] Initialisation started..\n"));

	// Set all board LEDs as outputs
	mincopter.hal.gpio->pinMode(27, 1);
	mincopter.hal.gpio->pinMode(26, 1);
	mincopter.hal.gpio->pinMode(25, 1);
	
	// Switch all board LEDs on
	/*
	mincopter.hal.gpio->write(27, 0);
	mincopter.hal.gpio->write(26, 0);
	mincopter.hal.gpio->write(25, 0);
	*/

    // GPS UART/Serial port initialisation
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
	// NOTE We use uartB for GPS on AVR, otherwise, for boards like RPI we
	// re-use uartA for GPS
#if defined(TARGET_ARCH_AVR) || defined(TARGET_ARCH_STM32)
	if (mincopter.hal.uartB != NULL) mincopter.hal.uartB->begin(38400, 256, 16);
	mincopter.hal.console->printf_P(PSTR("[INIT] uartB initialised\n"));
#else
	if (mincopter.hal.uartA != NULL) mincopter.hal.uartA->begin(38400, 256, 16);
	mincopter.hal.console->printf_P(PSTR("[INIT] uartA initialised\n"));
#endif

#endif

#ifdef HAL_BOARD_APM2
    // Run the timer a bit slower on APM2 to reduce the interrupt load on the CPU
    mincopter.hal.scheduler->set_timer_speed(500);
#endif

    // Initialise battery monitor
    mincopter.battery.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] Battery monitor initialised\n"));

    mincopter.rssi_analog_source      = mincopter.hal.analogin->channel(mincopter.rssi_pin);
    mincopter.board_vcc_analog_source = mincopter.hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    mincopter.barometer.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] Barometer initialised\n"));

	// TODO What is this doing - remove
    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    planner.ap.usb_connected = true;

    //check_usb_mux();

#if CONFIG_HAL_BOARD != HAL_BOARD_AVR
    // we have a 2nd serial port for telemetry on all boards except
    // APM2. We actually do have one on APM2 but it isn't necessary as
    // a MUX is used
	
	// TODO Replace this with the board configuration that checks how many UARTs are enabled 
	if (mincopter.hal.uartB != NULL) {
    	//mincopter.hal.uartB->begin(SERIAL1_BAUD, 128, 128);
		//mincopter.hal.console->printf_P(PSTR("[INIT] uartB initialised\n"));
	}
#endif

	// Telemetry
    if (mincopter.hal.uartC != NULL) {
        mincopter.hal.uartC->begin(57600, 128, 128);

		mincopter.hal.console->printf_P(PSTR("[INIT] uartC initialised\n"));
        //gcs[2].init(hal.uartD);

		mincopter.hal.uartC->printf("Telem Test\r\n");
    }

#if defined(LOGGING_ENABLED)
	/* NOTE The log_structure variable is an array of LogStructure objects. It is referenced in the log.h header
	 * file but here we are getting the sizeof(log_structure) */

    //DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
	/* NOTE: Using 23 different structures instead of counting due to issue in separation of log_structure object */
    mincopter.DataFlash.Init(log_structure, 22);
	mincopter.hal.console->printf_P(PSTR("[INIT] DataFlash initialised\n"));

#endif

	/* NOTE no RC input in auto modes */
    //init_rc_in();               // sets up rc channels from radio
    //init_rc_out();              // sets up motors and output to escs

    //mincopter.hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

	// NOTE This initialises the external ADCs (as opposed to the hal adc) if present
	// We still include because we specify our ADC as AP_ADC_None usually
    mincopter.adc.Init();           // APM ADC library initialization
	mincopter.hal.console->printf_P(PSTR("[INIT] ADC initialised\n"));

    // Do GPS init
    mincopter.g_gps = &mincopter.g_gps_driver;

    // GPS Initialization with correct UART
#if defined(TARGET_ARCH_AVR) || defined(TARGET_ARCH_STM32)
	if (mincopter.hal.uartB != NULL) {
    	mincopter.g_gps->init(mincopter.hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);
#else
	if (mincopter.hal.uartA != NULL) {
    	mincopter.g_gps->init(mincopter.hal.uartA, GPS::GPS_ENGINE_AIRBORNE_1G);
#endif
		mincopter.hal.console->printf_P(PSTR("[INIT] GPS initialised\n"));
	}

	// NOTE TODO Check whether the compass init was successful
    mincopter.compass.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] Compass initialised\n"));

	// TODO NOTE We have removed the barometer calibration and replaced with the update_calibration method called by the planner upon arming
	// TODO Part of this function sets the ground pressure/temperature which should really be done upon arming
	// Also in simulation, this functions hangs as we do not have a reading from gazebo before initialisation
#ifndef TARGET_ARCH_LINUX
	//mincopter.barometer.calibrate();
#endif

	// TODO Why is ins initialised after MCState??
    // Warm up and read Gyro offsets
    mincopter.ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
	mincopter.hal.console->printf_P(PSTR("[INIT] IMU initialised\n"));

	// Set state as landed
	planner.ap.land_complete = 1;

#if defined(LOGGING_ENABLED)
    Log_Write_Startup();
#endif

	// Initialise our mcstate algorithms. This will call the class-specific **init_derived** method
    mcstate.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] MCState initialised\n"));

#ifdef TARGET_ARCH_LINUX
	// Delay 1s
	mincopter.hal.scheduler->delay(1000);
#endif

	mincopter.hal.console->printf_P(PSTR("[INIT] Initialisation complete, post-init RAM:%u\n"), mincopter.hal.util->available_memory());
	return;
}

