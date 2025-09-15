
#include "mcinstance.h"
#include "mcstate.h"

#ifdef TARGET_ARCH_LINUX
    #include "simulation_logger.h"
#endif

extern MCInstance mincopter;
extern MCState mcstate;

#include "planner.h"

#include "util.h"
#include "log.h"

// Forward Declaration - TODO move menu.cpp code to a class and include
void init_cli(AP_HAL::UARTDriver* port);
void run_cli(void);

void init_ardupilot()
{
	mincopter.hal.console->printf_P(PSTR("INITIALISATION STARTED\n"));

	// Run for anything except simulation
#ifndef TARGET_ARCH_LINUX
    if (!mincopter.hal.gpio->usb_connected()) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }
#endif

    // Console serial port
    //mincopter.hal.uartB->begin(SERIAL0_BAUD, 512, 128);
	//mincopter.hal.console->printf_P(PSTR("[INIT] uartB initialised\n"));

    // GPS UART/Serial port initialisation
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
	// NOTE We use uartB for GPS on AVR, otherwise, for boards like RPI we
	// re-use uartA for GPS
#ifdef TARGET_ARCH_AVR
	if (mincopter.hal.uartB != NULL) mincopter.hal.uartB->begin(38400, 256, 16);
	mincopter.hal.console->printf_P(PSTR("[INIT] uartB initialised\n"));
#else
	if (mincopter.hal.uartA != NULL) mincopter.hal.uartA->begin(38400, 256, 16);
	mincopter.hal.console->printf_P(PSTR("[INIT] uartA initialised\n"));
#endif

#endif

	// Send initialisation string
    //mincopter.cliSerial->printf_P(PSTR("PS00-Init, Free RAM: %u\n"), mincopter.hal.util->available_memory());

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    /*
      run the timer a bit slower on APM2 to reduce the interrupt load
      on the CPU
     */
    mincopter.hal.scheduler->set_timer_speed(500);
#endif


    bool enable_external_leds = true;


    // initialise battery monitor
    mincopter.battery.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] Battery monitor initialised\n"));

    mincopter.rssi_analog_source      = mincopter.hal.analogin->channel(mincopter.rssi_pin);
    mincopter.board_vcc_analog_source = mincopter.hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    mincopter.barometer.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] Barometer initialised\n"));

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

    //gcs[1].init(hal.uartC);
#endif

#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (mincopter.hal.uartD != NULL) {
        mincopter.hal.uartD->begin(SERIAL2_BAUD, 128, 128);
		mincopter.hal.console->printf_P(PSTR("[INIT] uartD initialised\n"));
        //gcs[2].init(hal.uartD);
    }
#endif


    // identify ourselves correctly with the ground station
		/*
    mavlink_system.sysid = g.sysid_this_mav;
    mavlink_system.type = 2; //MAV_QUADROTOR;
		*/

#if LOGGING_ENABLED == ENABLED
		// NOTE(henry) This has been removed temporarily due to issue with compilation described below:
		/* The log_structure variable is an array of LogStructure objects. It is referenced in the log.h header file
		* but here we are getting the sizeof(log_structure). 
		*/
    //DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
		/* NOTE: Using 23 different structures instead of counting due to issue in separation of log_structure object */
    mincopter.DataFlash.Init(log_structure, 22);
	mincopter.hal.console->printf_P(PSTR("[INIT] DataFlash initialised\n"));

    if (!mincopter.DataFlash.CardInserted()) {
        //gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        mincopter.log_bitmask = 0;
    } else if (mincopter.DataFlash.NeedErase()) {
        //gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        //gcs[0].reset_cli_timeout();
    }

		//mincopter.cliSerial->println_P(PSTR("Dataflash initialised\n"));
#endif

		/* NOTE no RC input in auto modes */
    //init_rc_in();               // sets up rc channels from radio
    //init_rc_out();              // sets up motors and output to escs

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */

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
#ifdef TARGET_ARCH_AVR
	if (mincopter.hal.uartB != NULL) {
    	mincopter.g_gps->init(mincopter.hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);
#else
	if (mincopter.hal.uartA != NULL) {
    	mincopter.g_gps->init(mincopter.hal.uartA, GPS::GPS_ENGINE_AIRBORNE_1G);
#endif
		mincopter.hal.console->printf_P(PSTR("[INIT] GPS initialised\n"));
	}

    //init_compass();
	// NOTE TODO Check whether the compass init was successful
    mincopter.compass.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] Compass initialised\n"));
	//mincopter.compass.read();
	
    //mcstate.ahrs.set_compass(&mincopter.compass);


#if CLI_ENABLED == ENABLED
    //const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    //cliSerial->println_P(msg);

		/*
    if (gcs[1].initialised) {
        hal.uartC->println_P(msg);
    }
    if (num_gcs > 2 && gcs[2].initialised) {
        hal.uartD->println_P(msg);
    }
		*/
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (!mincopter.barometer.healthy) {
        // the barometer becomes healthy when we get the first
        // HIL_STATE message
        //gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);
#endif
	// TODO Why is barometer initialised twice?

    // initialize commands
    //init_commands();

    // initialise the flight mode and aux switch
		/* NOTE removed */
    //reset_control_switch();

		/* NOTE removed */
    //init_aux_switches();

	// TODO Why is ins initialised after MCState??
    // Warm up and read Gyro offsets
    // -----------------------------
    mincopter.ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
	mincopter.hal.console->printf_P(PSTR("[INIT] IMU initialised\n"));

    // setup fast AHRS gains to get right attitude
    //mcstate.ahrs.set_fast_gains(true);

    // set landed flag
    set_land_complete(true);

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

	// Should initialise both ahrs and inertial_nav
    mcstate.init();
	mincopter.hal.console->printf_P(PSTR("[INIT] MCState initialised\n"));

	/* Dump Log on start */

		/*
		DataFlash.ListAvailableLogs(cliSerial);
		
		uint16_t nl = DataFlash.get_num_logs();
		cliSerial->printf_P(PSTR("Num Logs: %u\n"), nl);

	
		int16_t lognum=19;
		uint16_t dl_start;
		uint16_t dl_end;

		DataFlash.get_log_boundaries(lognum, dl_start, dl_end);
		cliSerial->printf_P(PSTR("Reading Log 19: %u %u\n"), dl_start, dl_end);
		Log_Read((uint16_t)lognum,dl_start, dl_end);
		*/

		// Send initialisation synchronisation string. Now ready for CLI

		/* -- Transmission statistics --
			- Send 100 packets of the same string and measure tranmission time
		*/

		// 1302.930us AVG for 16 byte packet
		// 2700 for 32 byte


		/*
		uint32_t sum=0;
		for (int i=0;i<100;i++) { 
			uint32_t pre = micros();
			// send transmission of 16 byte packet
			mincopter.cliSerial->println_P(PSTR("TEST1234567891-TEST1234567891"));
			sum += micros() - pre;
		}
		float res = sum/100.0;
		mincopter.cliSerial->printf_P(PSTR("Final stat - %f"),res);
		*/

		// Init telemetry uartC

		/* NOTE - See article in README.md for guide on how to move telem to UART2 (uartC) */
    //mincopter.hal.uartC->begin(SERIAL1_BAUD, 128, 128);
		//mincopter.hal.uartC->printf_P(PSTR("TEST-send"));

		// Start Menu
		// NOTE cliSerial is an alias for hal.uartA I think
		//init_cli(mincopter.hal.uartB);

		// Finally, run the profiling test functions that measure approximate time taken to
		// publish serial messages and log messages of different format
		//
		// We run during `init_ardupilot` because it is guaranteed to be 'single-threaded'.

#ifdef TARGET_ARCH_LINUX

		uint32_t ts_now_us = mincopter.hal.scheduler->micros();
		// Delay 1s
		mincopter.hal.scheduler->delay(1000);
		
#endif

		/*
		// TEST 1 - Single 4byte message, 12 repeats, no format
		uint32_t start_time = micros();
		for(int i=0;i<12;i++) {
			mincopter.cliSerial->printf_P(PSTR("TST\n"));
		}
		uint32_t end = micros()-start_time;
		mincopter.cliSerial->printf_P(PSTR("TEST1-%uus\n"), end);

		// TEST 2 - Single 16 byte message, 12 repeats, no format
		start_time = micros();
		for(int i=0;i<12;i++) {
			mincopter.cliSerial->printf_P(PSTR("TESTTESTTESTTES\n"));
		}
		end = micros()-start_time;
		mincopter.cliSerial->printf_P(PSTR("TEST2-%uus\n"), end);

		// TEST 3 - Single 16 byte message, no repeats, no format
		start_time = micros();
		for(int i=0;i<1;i++) {
			mincopter.cliSerial->printf_P(PSTR("TESTTESTTESTTES\n"));
		}
		end = micros()-start_time;
		mincopter.cliSerial->printf_P(PSTR("TEST3-%uus\n"), end);

		// TEST 4 - Single 16 byte message, 12 repeats with 1 format
		start_time = micros();
		for(int i=0;i<12;i++) {
			mincopter.cliSerial->printf_P(PSTR("TS_%dTS_%dTS_%dTS%d\n"), i, i, i, i);
		}
		end = micros()-start_time;
		mincopter.cliSerial->printf_P(PSTR("TEST4-%uus\n"), end);

		// TEST 5 - Single 32 byte message, no repeats, no format
		start_time = micros();
		for(int i=0;i<1;i++) {
			mincopter.cliSerial->printf_P(PSTR("FOURFOURFOURFOURFOURFOURFOURFOU\n"));
		}
		end = micros()-start_time;
		mincopter.cliSerial->printf_P(PSTR("TEST5-%uus\n"), end);
		*/

	mincopter.hal.console->printf_P(PSTR("[INIT] Initialisation complete, post-init RAM:%u\n"), mincopter.hal.util->available_memory());
	return;
}

