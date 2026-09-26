
#include "telemetry.h"

// TODO Remove this planner dependency or specify explicitly via reference
#include "planner.h"


/* The mincopter telemetry standard currently defines four functions
 *
 * 1. Heartbeat
 * 2. Arm
 * 3. Disarm
 * 4. Run tests
 * 5. Get flight state
 *
 */

// Stub functions for telemetry callbacks. NOTE: By default we use these functions but in things like test modules, we overwrite them
// with our custom behaviour

// TODO This is also bad design but this is defined in the mincopter.cpp

__attribute__((weak)) void mincopter_telemetry_command_landrequest(void* argptr)
{
	/*
	// Flag that a switch to land mode was requested
	planner.ap.land_requested_telem = 1;
	hal.console->printf("Land requested from telem\r\n");
							
	// Acknowledge land request
	uint8_t telem_tx_buffer[] = {0x24, 0x0F};
	hal.uartC->write(telem_tx_buffer, 2);

	return;
	*/
}

__attribute__((weak)) void mincopter_telemetry_command_heartbeatrequest(void* argptr)
{
	/*
	// Arguments:
	// argptr[0]: last heartbeat sequence ID
	if (((uint8_t*)argptr)[0] == planner.failsafe.telemetry_last_heartbeat_seq_id) {

		if (!planner.failsafe.telemetry_first_connect) {
			// Flag that we have connected to telemetry. This will trigger the planner/controller logic
			planner.failsafe.telemetry_first_connect = 1;
			hal.console->printf("[TELE] Connected to telemetry\r\n");
		}
		planner.failsafe.telemetry_active = 1;

		// Reset the heartbeat timestamp
		planner.failsafe.telemetry_last_heartbeat_ms = hal.scheduler->millis();
	} else {
		// TODO 
		// We have found a heartbeat message in the wrong order.
	}

	return;
	*/
}

__attribute__((weak)) void mincopter_telemetry_command_armrequest(void* argptr)
{
	/*
	// Set arm request flag
	planner.ap.arm_requested_telem = 1;
	hal.console->printf("Arm requested from telem\r\n");

	// Acknowledge the arm request
	uint8_t telem_tx_buffer[] = {0x24, 0x0B};
	hal.uartC->write(telem_tx_buffer, 2);

	return;
	*/
}

__attribute__((weak)) void mincopter_telemetry_command_disarmrequest(void* argptr)
{
	/*
	// Flag that an immediate disarm was requested
	planner.ap.disarm_requested_telem = 1;
	hal.console->printf("Disarm requested from telem\r\n");
							
	// Acknowledge disarm request
	uint8_t telem_tx_buffer[] = {0x24, 0x0E};
	hal.uartC->write(telem_tx_buffer, 2);

	return;
	*/
}

__attribute__((weak)) void mincopter_telemetry_command_flightstaterequest(void* argptr)
{

}

__attribute__((weak)) void mincopter_telemetry_command_testrequest(void* argptr)
{

}

