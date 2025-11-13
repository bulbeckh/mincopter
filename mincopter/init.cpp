
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

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
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

#if LOGGING_ENABLED == ENABLED
	/* NOTE The log_structure variable is an array of LogStructure objects. It is referenced in the log.h header
	 * file but here we are getting the sizeof(log_structure) */

    //DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
	/* NOTE: Using 23 different structures instead of counting due to issue in separation of log_structure object */
    mincopter.DataFlash.Init(log_structure, 22);
	mincopter.hal.console->printf_P(PSTR("[INIT] DataFlash initialised\n"));

    if (!mincopter.DataFlash.CardInserted()) {
        //gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        mincopter.log_bitmask = 0;
    }
#endif

	/* NOTE no RC input in auto modes */
    //init_rc_in();               // sets up rc channels from radio
    //init_rc_out();              // sets up motors and output to escs

    //mincopter.hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    mincopter.adc.Init();           // APM ADC library initialization
	mincopter.hal.console->printf_P(PSTR("[INIT] ADC initialised\n"));

 #endif // CONFIG_ADC
#endif // HIL_MODE

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
	
#if HIL_MODE != HIL_MODE_DISABLED
    while (!mincopter.barometer.healthy) {
        // the barometer becomes healthy when we get the first
        // HIL_STATE message
        //gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

	// TODO NOTE We are very temporarily turning off barometer initialisation so that STM32 can run
	
    // read Baro pressure at ground
    //init_barometer(true);

	// TODO Why is barometer initialised twice?
	
	// TODO Why is ins initialised after MCState??
    // Warm up and read Gyro offsets
    mincopter.ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
	mincopter.hal.console->printf_P(PSTR("[INIT] IMU initialised\n"));

    // set landed flag
    set_land_complete(true);

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

	// Should initialise both ahrs and inertial_nav
    mcstate.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] MCState initialised\n"));

#ifdef TARGET_ARCH_LINUX
	// Delay 1s
	mincopter.hal.scheduler->delay(1000);
#endif

	mincopter.hal.console->printf_P(PSTR("[INIT] Initialisation complete, post-init RAM:%u\n"), mincopter.hal.util->available_memory());
	return;
}

