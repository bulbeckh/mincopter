
#include "system.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "control_modes.h"
#include "navigation.h"
#include "util.h"
#include "log.h"
#include "radio.h"
#include "parameters.h"

#ifdef TARGET_ARCH_LINUX
	#include <iostream>
#endif

// Forward Declaration - TODO move menu.cpp code to a class and include
void init_cli(AP_HAL::UARTDriver* port);
void run_cli(void);

void init_ardupilot()
{

#ifdef TARGET_ARCH_LINUX
		std::cout << "Target linux: Initialise started\n";
#endif

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
    //
    // The console port buffers are defined to be sufficiently large to support
    // the MAVLink protocol efficiently
    //
#if HIL_MODE != HIL_MODE_DISABLED
    // we need more memory for HIL, as we get a much higher packet rate
    mincopter.hal.uartA->begin(SERIAL0_BAUD, 256, 256);
#else
    // use a bit less for non-HIL operation
    mincopter.hal.uartA->begin(SERIAL0_BAUD, 512, 128);
#endif

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    mincopter.hal.uartB->begin(38400, 256, 16);
#endif

		// Send initialisation string
    mincopter.cliSerial->printf_P(PSTR("PS00-Init, Free RAM: %u\n"), mincopter.hal.util->available_memory());

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    /*
      run the timer a bit slower on APM2 to reduce the interrupt load
      on the CPU
     */
    mincopter.hal.scheduler->set_timer_speed(500);
#endif


    // load parameters from EEPROM
    //load_parameters();

    // FIX: this needs to be the inverse motors mask
    //ServoRelayEvents.set_channel_mask(0xFFF0);

    //relay.init();

    bool enable_external_leds = true;

    // initialise notify system
    // disable external leds if epm is enabled because of pin conflict on the APM
    mincopter.notify.init(enable_external_leds);

    // initialise battery monitor
    mincopter.battery.init();

    mincopter.rssi_analog_source      = mincopter.hal.analogin->channel(mincopter.g.rssi_pin);
    mincopter.board_vcc_analog_source = mincopter.hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    mincopter.barometer.init();

    // init the GCS
		// REMOVE GCS
    //gcs[0].init(hal.uartA);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
		// NO MAVLINK
    //hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    mincopter.ap.usb_connected = true;


    check_usb_mux();

#if CONFIG_HAL_BOARD != HAL_BOARD_APM2
    // we have a 2nd serial port for telemetry on all boards except
    // APM2. We actually do have one on APM2 but it isn't necessary as
    // a MUX is used
    mincopter.hal.uartC->begin(map_baudrate(mincopter.g.serial1_baud, SERIAL1_BAUD), 128, 128);
    //gcs[1].init(hal.uartC);
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (mincopter.hal.uartD != NULL) {
        mincopter.hal.uartD->begin(map_baudrate(mincopter.g.serial2_baud, SERIAL2_BAUD), 128, 128);
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
    if (!mincopter.DataFlash.CardInserted()) {
        //gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        mincopter.g.log_bitmask.set(0);
    } else if (mincopter.DataFlash.NeedErase()) {
        //gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        //gcs[0].reset_cli_timeout();
    }

		//mincopter.cliSerial->println_P(PSTR("Dataflash initialised\n"));
#endif

		/* NOTE no RC input in auto modes */
    //init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */

    mincopter.hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    mincopter.adc.Init();           // APM ADC library initialization
 #endif // CONFIG_ADC
#endif // HIL_MODE

    // Do GPS init
    mincopter.g_gps = &mincopter.g_gps_driver;

    // GPS Initialization
    mincopter.g_gps->init(mincopter.hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);

    if(mincopter.g.compass_enabled)
        init_compass();

		/*
    // init the optical flow sensor
    if(g.optflow_enabled) {
        init_optflow();
    }
		*/

    // initialise inertial nav
    mcstate.inertial_nav.init();


#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

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

    // initialize commands
    //init_commands();

    // initialise the flight mode and aux switch
		/* NOTE removed */
    //reset_control_switch();

		/* NOTE removed */
    //init_aux_switches();

    startup_ground(true);

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

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
		mincopter.cliSerial->println_P(PSTR("PS00-Initialisation Complete"));

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
		init_cli(mincopter.hal.uartA);

		// Finally, run the profiling test functions that measure approximate time taken to
		// publish serial messages and log messages of different format
		//
		// We run during `init_ardupilot` because it is guaranteed to be 'single-threaded'.

#ifdef TARGET_ARCH_LINUX
		std::cout << "Initialise finish\n";
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

}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void startup_ground(bool force_gyro_cal)
{
    //gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    mcstate.ahrs.init();

    // Warm up and read Gyro offsets
    // -----------------------------
    mincopter.ins.init(force_gyro_cal?AP_InertialSensor::COLD_START:AP_InertialSensor::WARM_START,
             mincopter.ins_sample_rate);

    // setup fast AHRS gains to get right attitude
    mcstate.ahrs.set_fast_gains(true);

    // set landed flag
    set_land_complete(true);
}

// returns true if the GPS is ok and home position is set
bool GPS_ok()
{
    if (mincopter.g_gps != NULL && mincopter.ap.home_is_set && mincopter.g_gps->status() == GPS::GPS_OK_FIX_3D && !mincopter.gps_glitch.glitching() && !mcstate.failsafe.gps) {
        return true;
    }else{
        return false;
    }
}

// set_mode - change flight mode and perform any necessary initialisation
/* NOTE deprecated in favour of auto-only modes */
bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !mincopter.motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
		// #TODO control_mode is part of MCInstance but will be moved either as a state variable or the btree
    if (mode == mincopter.control_mode) {
        return true;
    }

    switch(mode) {

        case STABILIZE:
            success = true;
            set_yaw_mode(STABILIZE_YAW);
            set_roll_pitch_mode(STABILIZE_RP);
            set_throttle_mode(STABILIZE_THR);
            set_nav_mode(NAV_NONE);
            break;

        case AUTO:
            // check we have a GPS and at least one mission command (note the home position is always command 0)
            if ((GPS_ok() && mincopter.g.command_total > 1) || ignore_checks) {
                success = true;
                // roll-pitch, throttle and yaw modes will all be set by the first nav command
                //init_commands();            // clear the command queues. will be reloaded when "run_autopilot" calls "update_commands" function
								// NOTE removed commands - need to reconfigure how autpilot starts and runs
            }
            break;

        case LAND:
            success = true;
						// NOTE As with above, need to reconfigure how autopilot works here
            //do_land(NULL);  // land at current location
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        mincopter.control_mode = mode;
        //Log_Write_Mode(control_mode);
    }else{
        // Log error that we failed to enter desired flight mode
        //Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // return success or failure
    return success;
}

// update_auto_armed - update status of auto_armed flag
void update_auto_armed()
{
    // disarm checks
    if(mincopter.ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!mincopter.motors.armed()) {
            set_auto_armed(false);
            return;
        }

    }else{
        // arm checks
        
        // if motors are armed and throttle is above zero auto_armed should be true
        if(mincopter.motors.armed() && mincopter.g.rc_3.control_in != 0) {
            set_auto_armed(true);
        }
    }
}

/*
 *  map from a 8 bit EEPROM baud rate to a real baud rate
 */
uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    //cliSerial->println_P(PSTR("Invalid baudrate"));
    return default_baud;
}

void check_usb_mux(void)
{
    bool usb_check = mincopter.hal.gpio->usb_connected();
    if (usb_check == mincopter.ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    mincopter.ap.usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (mincopter.ap.usb_connected) {
        mincopter.hal.uartA->begin(SERIAL0_BAUD);
    } else {
        mincopter.hal.uartA->begin(map_baudrate(mincopter.g.serial1_baud, SERIAL1_BAUD));
    }
#endif
}

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return mincopter.board_vcc_analog_source->voltage_latest() * 1000;
}

