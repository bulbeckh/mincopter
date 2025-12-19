
// TODO This should really be renamed to something like sensor_updates.cpp

#include "mcinstance.h"
#include "mcstate.h"

/* NOTE The reason why there is a mincopter instance here is because these are not class methods */
extern MCInstance mincopter;

#include "planner.h"
#include "control.h"

#include "defines.h"
#include "util.h"
#include "log.h"

#include <AP_HAL/AP_HAL.h>

// TODO Move telem into it's own file or even class
// TODO Check how much util the read_telemetry function is using and maybe decrease frequency

/* @brief Read incoming telemetry messages. We call this at every iteration an process no more than 8 bytes of
 * a telemetry message */
void read_telemetry(void)
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

void send_telemetry_heartbeat(void)
{
	// TODO What do to when we receive a message from an old packet (i.e. identifier number less than what we are expecting
	
	uint8_t _telem_tx_buffer[] = {0x24, 0x0A, 0x00};

	if (!planner.failsafe.telemetry_first_connect) {
		/* If we have not yet connected to our telemetry, we keep sending heartbeat messages with
		 * a sequence ID of 0x5A */
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

void failsafe_checks(void)
{
	/* This failsafe function runs at 10Hz and checks for breaches of failsafe conditions like low
	 * battery, position outside of geo-fence and a telemetry heartbeat miss.
	 *
	 * TODO We either have a crash check here or do crash checks in it's own scheduled function
	 *
	 *
	 */

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

void accumulate_compass(void)
{
	// Accumulate compass readings
	mincopter.compass.accumulate();

	return;
}

void accumulate_barometer(void)
{
	// Accumulate barometer readings
	mincopter.barometer.accumulate();

	return;
}

void read_barometer(void)
{
	// Update barometer
	mincopter.barometer.read();

	return;
}

void read_batt_compass(void)
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
void update_GPS(void)
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
