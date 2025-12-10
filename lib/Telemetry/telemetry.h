
#pragma once

#include <stdint.h>

class Telemetry {

	public:
		Telemetry() { }

	public:
		/* @brief Reads up to n bytes of the telemetry input stream */
		void read(uint8_t max_bytes);

	private:
		/* @brief The next byte in the telemetry stream */
		int16_t nb;

		// State machine variables

		/* @brief The part of the current message we are at */
		uint8_t cmd_state{0};

		/* @brief The command type of the current message */
		uint8_t cmd_type;

		/* @brief The number of arguments remaining to read/parse in the current message */
		uint8_t remaining;

		/* @brief Argument buffer for passing to callback functions. NOTE: We should have **at max** 16 arguments for any command */
		uint8_t argbuffer[8];

};



/* Callback function definitions
 *
 * We define the default (mincopter) implementations with the **weak** attribute so that they can be overridden in tests
 * that use telemetry */
void mincopter_telemetry_command_heartbeatrequest(void* argptr);
void mincopter_telemetry_command_armrequest(void* argptr);
void mincopter_telemetry_command_disarmrequest(void* argptr);
void mincopter_telemetry_command_flightstaterequest(void* argptr);
void mincopter_telemetry_command_testrequest(void* argptr);
void mincopter_telemetry_command_landrequest(void* argptr);


