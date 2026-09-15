
// TODO This should really be renamed to something like sensor_updates.cpp

#include "mcinstance.h"
#include "mcstate.h"

#include "defines.h"
#include "util.h"
#include "log.h"

#include <AP_HAL/AP_HAL.h>

// TODO Move telem into it's own file or even class
// TODO Check how much util the read_telemetry function is using and maybe decrease frequency

void read_receiver_rssi(MCInstance& mincopter)
{
    // avoid divide by zero
    if (mincopter.rssi_range <= 0) {
        mincopter.receiver_rssi = 0;
    }else{
        mincopter.rssi_analog_source->set_pin(mincopter.rssi_pin);
        float ret = mincopter.rssi_analog_source->voltage_average() * 255 / mincopter.rssi_range;
        mincopter.receiver_rssi = constrain_int16(ret, 0, 255);
    }
    return;
}

/* @brief Read incoming telemetry messages. We call this at every iteration an process no more than 8 bytes of
 * a telemetry message */
void read_telemetry(MCInstance& mincopter)
{
	/* Design of simple console to read incoming telemetry commands
	 *
	 * Since we call this function at 100Hz, we read no more than 8 bytes of incoming telemetry (uart) streams.
	 * We use a state machine that persists between calls so that we can process some of a stream before yielding.
	 *
	 * # Packet Stream Design & Command API
	 * TODO see mincopter-terminal repo
	 *
	 * We use four variables to capture the state of our state machine.
	 *
	 * cmd_state : Tracks what part of the packet we are expecting next (0 = sync byte, 1 = command type, 2 = args)
	 * cmd_type  : Tracks what command type we are currently parsing. Set after read of 2nd byte and cleared upon error or full packet
	 * remaining : Tracks how many arguments of this command we have left to read. Set after ready of 2nd byte and cleared upon error
	 * 	or full packet
	 * cmd_arg_buffer : Buffer (uint8_t[8]) containing the arguments for each command. No command has more than 8 bytes of arguments
	 * 	so we keep this as fixed size. NOTE This will change in future versions.
	 *
	 * If at any stage of the stream we encounter an error, we reset the state machine and keep reading until we hit the sync byte. After
	 * an error, we should also re-send a hearbeat message as the heartbeat response for the telemetry may have been corrupted in the stream.
	 */

	// TODO This is another instance where we have a scheduled function that just calls another class function. Bad design - needs to be fixed
	// Read telemetry
	mincopter.telemetry.read(8);

	return;

}

// TODO Same issue with dependency on planner as we have in failsafe_checks function
/*
void send_telemetry_heartbeat(void)
{
	// TODO What do to when we receive a message from an old packet (i.e. identifier number less than what we are expecting
	
	uint8_t _telem_tx_buffer[] = {0x24, 0x0A, 0x00};

	if (!planner.failsafe.telemetry_first_connect) {
		// If we have not yet connected to our telemetry, we keep sending heartbeat messages with
		// a sequence ID of 0x5A
		_telem_tx_buffer[2] = 0x5A;

		// Set heartbeat id
		planner.failsafe.telemetry_last_heartbeat_seq_id = 0x5A;
	} else {
		// Otherwise, we increment the sequence identifier and send
		_telem_tx_buffer[2] = ++planner.failsafe.telemetry_last_heartbeat_seq_id;
	}

	// Write the heartbeat message to telemetry
	mincopter.hal.uartC->write(_telem_tx_buffer, 3);

	return;
}
*/

/* TODO I think this is bad design - having a scheduled function (failsafe checks) be dependent
 * on the planner object. I think instead we need a error state handling which is scheduled when
 * something goes wrong, like we breach the geofence or we fail to plan a valid path. Temporarily
 * disable this scheduled function. */
/*
void failsafe_checks(void)
{
	// This failsafe function runs at 10Hz and checks for breaches of failsafe conditions like low
	// battery, position outside of geo-fence and a telemetry heartbeat miss.
	//
	// TODO We either have a crash check here or do crash checks in it's own scheduled function

	// TODO Add remaining failsafe checks
	// TODO It is strange that we have a flag to run the telemetry failsafe and other failsafe
	// checks. I can't find a reason or state in which they shouldn't be run.
	
	// Run telemetry failsafe if enabled and **only** after we have first connected to a telemetry
	if (planner.failsafe.fs_enabled_telem && planner.failsafe.telemetry_first_connect) {
		// If we have passed 300ms without a response to our heartbeat message, then we consider the failsafe to have been missed and
		// we mark the telemetry_active as false
		//
		// We send heartbeat requests every 100ms and we expect a response back (in the correct sequence). When we receive that then we
		// set the telemetry_last_heartbeat_ms to the time that the response was received.
		//
		// We read (at most) 8 bytes of the telemetry every 10ms so we expect that we can parse a full response between two heartbeat
		// requests

		uint32_t elapsed = mincopter.hal.scheduler->millis() - planner.failsafe.telemetry_last_heartbeat_ms;
		if (elapsed >= 300ul) {
			planner.failsafe.telemetry_active = 0;

			// TODO Run failsafe action
			mincopter.hal.uartA->printf("Failsafe miss.. (%u elapsed)\r\n", mincopter.hal.scheduler->millis() - planner.failsafe.telemetry_last_heartbeat_ms);

			// TODO This is not the ideal behaviour in a failsafe miss and we should also check for at least 2-3 failsafe misses before disarming
			// Disarm immediately
			planner.ap.arm_active = 0;
		} else {
			planner.failsafe.telemetry_active = 1;
		}
	}
	
	// Run GPS failsafe if enabled
	if (planner.failsafe.fs_enabled_gps) {
		// TODO
	}

	// Run battery failsafe if enabled
	if (planner.failsafe.fs_enabled_battery) {
		// TODO
	}

	return;
}
*/

void accumulate_compass(MCInstance& mincopter)
{
	// Accumulate compass readings
	mincopter.compass.accumulate();

	return;
}

void accumulate_barometer(MCInstance& mincopter)
{
	// Accumulate barometer readings
	mincopter.barometer.accumulate();

	return;
}

void read_barometer(MCInstance& mincopter)
{
	// Update barometer
	mincopter.barometer.read();

	return;
}

void read_batt_compass(MCInstance& mincopter)
{
	// Update battery monitor
    mincopter.battery.read();

	// If we are monitoring current, then update the compass to correct for declination
    if (mincopter.battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        mincopter.compass.set_current(mincopter.battery.current_amps());
    }
	
	// Update compass
	mincopter.compass.read();

	// Log compass information
	//if (mincopter.log_bitmask & MASK_LOG_COMPASS) Log_Write_Compass();

	return;
}

// called at 50hz
void update_GPS(MCInstance& mincopter)
{
	// TODO Unused - remove
	static uint32_t last_gps_reading;           // time of last gps message
	static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location

	// Run a GPS update round
	mincopter.g_gps->update();

	// logging and glitch protection run after every gps message
	if (mincopter.g_gps->last_message_time_ms() != last_gps_reading) {
		last_gps_reading = mincopter.g_gps->last_message_time_ms();

		// log GPS message
		//if (mincopter.log_bitmask & MASK_LOG_GPS) Log_Write_GPS();

		// TODO planner.ap.home_is_set is a duplicate flag with mcstate.home_set - Need to decide which to use
		// run glitch protection and update AP_Notify if home has been initialised
		// TODO What is the GPS glitch protection and is it needed?
		//if (planner.ap.home_is_set) mincopter.gps_glitch.check_position();
	}

	// TODO Remove
	// checks to initialise home and take location based pictures
	/*
	if (mincopter.g_gps->new_data && mincopter.g_gps->status() >= GPS::GPS_OK_FIX_3D) {
		// clear new data flag
		mincopter.g_gps->new_data = false;

		// check if we can initialise home yet
		if (!planner.ap.home_is_set) {
			// if we have a 3d lock and valid location
			if (mincopter.g_gps->status() >= GPS::GPS_OK_FIX_3D && mincopter.g_gps->latitude != 0) {
				if (ground_start_count > 0) {
					ground_start_count--;
				} else {
					// after 10 successful reads store home location
					// ap.home_is_set will be true so mincopter will only happen once
					ground_start_count = 0;
						
						// TODO Move mincopter to btree as it initialises the start location on GPS lock
						init_home();

						// set system clock for log timestamps
						mincopter.hal.util->set_system_clock(mincopter.g_gps->time_epoch_usec());

						// Set compass declination automatically
						mincopter.compass.set_initial_location(mincopter.g_gps->latitude, mincopter.g_gps->longitude);
				}
			} else {
				// start again if we lose 3d lock
				ground_start_count = 10;
			}
		}
	}
	*/
}

void MCInstance::init_ardupilot(void)
{
	hal.console->printf_P(PSTR("[INIT] Initialisation started..\n"));

	// Set all board LEDs as outputs
	/*
	hal.gpio->pinMode(27, 1);
	hal.gpio->pinMode(26, 1);
	hal.gpio->pinMode(25, 1);
	*/ 
	
	// Switch all board LEDs on
	/*
	hal.gpio->write(27, 0);
	hal.gpio->write(26, 0);
	hal.gpio->write(25, 0);
	*/

	// GPS UART/Serial port initialisation
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
	// NOTE We use uartB for GPS on AVR, otherwise, for boards like RPI we
	// re-use uartA for GPS
#if defined(TARGET_ARCH_AVR) || defined(TARGET_ARCH_STM32)
	if (hal.uartB != NULL) hal.uartB->begin(38400, 256, 16);
	hal.console->printf_P(PSTR("[INIT] uartB initialised\n"));
#else
	if (hal.uartA != NULL) hal.uartA->begin(38400, 256, 16);
	hal.console->printf_P(PSTR("[INIT] uartA initialised\n"));
#endif

#endif

#ifdef HAL_BOARD_APM2
	// Run the timer a bit slower on APM2 to reduce the interrupt load on the CPU
	hal.scheduler->set_timer_speed(500);
#endif

	// Initialise battery monitor
	battery.init();
	hal.console->printf_P(PSTR("[INIT] Battery monitor initialised\n"));

    	rssi_analog_source      = hal.analogin->channel(rssi_pin);
    	board_vcc_analog_source = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

	// Initialise barometer
    	barometer.init();
	hal.console->printf_P(PSTR("[INIT] Barometer initialised\n"));

	// TODO What is this doing - remove
	// we start by assuming USB connected, as we initialed the serial
	// port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
	//planner.ap.usb_connected = true;

    	//check_usb_mux();

#if CONFIG_HAL_BOARD != HAL_BOARD_AVR
	// we have a 2nd serial port for telemetry on all boards except
	// APM2. We actually do have one on APM2 but it isn't necessary as
	// a MUX is used
	
	// TODO Replace this with the board configuration that checks how many UARTs are enabled 
	if (hal.uartB != NULL) {
		//hal.uartB->begin(SERIAL1_BAUD, 128, 128);
		//hal.console->printf_P(PSTR("[INIT] uartB initialised\n"));
	}
#endif

	// Telemetry
    	if (hal.uartC != NULL) {
        	hal.uartC->begin(57600, 128, 128);
		hal.console->printf_P(PSTR("[INIT] uartC initialised\n"));
        	//gcs[2].init(hal.uartD);
		hal.uartC->printf("Telem Test\r\n");
	}

#if defined(LOGGING_ENABLED)
	/* NOTE The log_structure variable is an array of LogStructure objects. It is referenced in the log.h header
	 * file but here we are getting the sizeof(log_structure) */

    	//DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
	/* NOTE: Using 23 different structures instead of counting due to issue in separation of log_structure object */
	// TODO Remove this camel case
    	DataFlash.Init(log_structure, 22);
	hal.console->printf_P(PSTR("[INIT] DataFlash initialised\n"));

#endif

	/* NOTE no RC input in auto modes */
    	//init_rc_in();               // sets up rc channels from radio
    	//init_rc_out();              // sets up motors and output to escs

    	//hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

	// ADC Initialisation
	// NOTE This initialises the external ADCs (as opposed to the hal adc) if present
	// We still include because we specify our ADC as AP_ADC_None usually
    	adc.Init();
	hal.console->printf_P(PSTR("[INIT] ADC initialised\n"));

	// GPS Initialisation

    	// GPS Initialization with correct UART
#if defined(TARGET_ARCH_AVR) || defined(TARGET_ARCH_STM32)
	if (hal.uartB != NULL) {
    		g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);
#else
	if (hal.uartA != NULL) {
    		g_gps->init(hal.uartA, GPS::GPS_ENGINE_AIRBORNE_1G);
#endif
		hal.console->printf_P(PSTR("[INIT] GPS initialised\n"));
	}

	// Compass Initialisation
    	compass.init();
	hal.console->printf_P(PSTR("[INIT] Compass initialised\n"));

	// TODO NOTE We have removed the barometer calibration and replaced with the update_calibration method called by the planner upon arming
	// TODO Part of this function sets the ground pressure/temperature which should really be done upon arming
	// Also in simulation, this functions hangs as we do not have a reading from gazebo before initialisation
#ifndef TARGET_ARCH_LINUX
	//barometer.calibrate();
#endif

	// IMU Initialisation
    	// Warm up and read Gyro offsets
    	ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
	hal.console->printf_P(PSTR("[INIT] IMU initialised\n"));

	// Set state as landed
	//planner.ap.land_complete = 1;

#if defined(LOGGING_ENABLED)
    	Log_Write_Startup();
#endif

#ifdef TARGET_ARCH_LINUX
	// Delay 1s
	hal.scheduler->delay(1000);
#endif

	hal.console->printf_P(PSTR("[INIT] Initialisation complete, post-init RAM:%u\n"), hal.util->available_memory());
	return;
}

