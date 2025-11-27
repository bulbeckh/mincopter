
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
	 *
	 * Byte 1. Sync byte - always 0x24 ($)
	 *
	 * Byte 2. Command Type (# args)
	 *  - 0x00 Not a command. Used during state machine reset.
	 * 	- 0x0A Receive Heartbeat (1)
	 * 	- 0x0B Set PWM (2)
	 * 	- 0x0C Run Motor Test (0)
	 * 	- 0x0D Request ARM
	 *
	 * Byte 3+ Args
	 * - 0x0A: [ heartbeat sequence number ]
	 * - 0x0B: [ motor number, pwm val idx (0 = 1100, 1 = 1200, 2 = 1300, ... ) ]
	 * - 0x0C: no args
	 * - 0x0D: no args
	 *
	 * # State Machine Design
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

	// We read up to 8 bytes from the console each iteration
	uint8_t read_counter = 8;

	// Next byte
	uint8_t nb;

	// State machine variables
	static uint8_t cmd_state = 0;
	static uint8_t cmd_type;
	static uint8_t remaining;

	while (read_counter) {
		read_counter--;

		// Read a single byte
		nb = hal.console->read();

		if (nb!=-1) {

			switch (cmd_state) {
				case 0:
					// Sync byte
					if (nb!=0x24) {
						// Log packet miss and reset state
						hal.console->printf("cli pkt miss! resetting\r\n");
						cmd_state = 0;
						cmd_type = 0;
						remaining = 0;
						break;
					}

					// Increment to next state
					cmd_state++;
					break;

				case 1:
					// Command Type
					switch (nb) {
						case 0x0A:
							// Receive heartbeat
							cmd_type = 0x0A;
							
							// Expect 1 packet (sequence identifier)
							remaining = 1;
							cmd_state++;
							break;
						case 0x0B:
							// Set PWM
							cmd_type = 0x0B;
							// Expect 2 packets (motor #, pwm index)
							remaining = 2;
							cmd_state++;
							break;
						case 0x0C:
							// Run test
							cmd_type = 0x0C;
							do_heartbeat = 5;
							remaining = 0;
							break;
						case 0x0D:
							// Attempt arming
						default:
							hal.console->printf("Wrong cmd type! resetting\r\n");
							cmd_state = 0;
							cmd_type = 0;
							remaining = 0;
							break;
					}

					// For some command types, we don't expect any more arguments so we should execute whatever command we need
					// and then reset
					if (!remaining) {
						cmd_state=0;
						cmd_type=0;
					}

					break;

				case 2:
					// Arguments
					remaining--;

					if (cmd_type==0x0A) {
						// Heartbeat Sequence Identifier
						if (nb == planner.failsafe.telemetry_last_heartbeat_seq_id) {
							// Flag that we have connected to telemetry. This will trigger the planner/controller logic
							planner.failsafe.telemetry_first_connect = 1;

							// Reset the heartbeat timestamp
							planner.failsafe.telemetry_last_heartbeat_ms = hal.scheduler->millis();
						} else {
							// TODO 
							// We have found a heartbeat message in the wrong order.
						}
					} else if (cmd_type==0x0B) {
						// Set PWM
						// TODO Use the 'remaining' variable as an index to which argument we are currently parsing
					} else {
						// TODO
						// We should flag here that we have a command type that doesn't take any arguments

						// Reset
						cmd_state = 0;
						cmd_type = 0;
					}
					
					// Reset if we have no more arguments to parse
					if (!remaining) {
						cmd_state = 0;
						cmd_type = 0;
					}

					break;

				default:
					break;
			}
		} else {
			// If nb == -1 then we have reached the end of our input stream and we simply return
			return;
		}
	}
}

void send_telemetry_heartbeat(void)
{
	/* If we have not yet connected to our telemetry, we keep sending heartbeat messages with
	 * a sequence ID of 0x5A */

	if (!planner.failsafe.telemetry_first_connect) {

	}

	/* 1. send a heartbeat message
	 * 2. record the time we sent the message/timer started */
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
	
	// Run telemetry failsafe if enabled
	if (planner.failsafe.fs_enabled_telem) {
		// If we have passed 100ms without a response to our heartbeat message, then we mark the telemetry_active as false
		if (hal.scheduler->millis() - planner.failsafe.telemetry_last_heartbeat_ms >= 1000ul) {
			planner.failsafe.telemetry_active = 0;
			// TODO Run failsafe action
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
	if (mincopter.log_bitmask & MASK_LOG_COMPASS) Log_Write_Compass();

	return;
}

// called at 50hz
void update_GPS(void)
{
	static uint32_t last_gps_reading;           // time of last gps message
	static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location

	mincopter.g_gps->update();

		// logging and glitch protection run after every gps message
		if (mincopter.g_gps->last_message_time_ms() != last_gps_reading) {
			last_gps_reading = mincopter.g_gps->last_message_time_ms();

			// log GPS message
			if (mincopter.log_bitmask & MASK_LOG_GPS) {
					mincopter.DataFlash.Log_Write_GPS(mincopter.g_gps, mcstate.current_loc.alt);
			}

			// run glitch protection and update AP_Notify if home has been initialised
			if (planner.ap.home_is_set) {
					mincopter.gps_glitch.check_position();
			}
		}

		// checks to initialise home and take location based pictures
		if (mincopter.g_gps->new_data && mincopter.g_gps->status() >= GPS::GPS_OK_FIX_3D) {
				// clear new data flag
				mincopter.g_gps->new_data = false;

				// check if we can initialise home yet
				if (!planner.ap.home_is_set) {
						// if we have a 3d lock and valid location
						if(mincopter.g_gps->status() >= GPS::GPS_OK_FIX_3D && mincopter.g_gps->latitude != 0) {
								if( ground_start_count > 0 ) {
										ground_start_count--;
								}else{
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
}
