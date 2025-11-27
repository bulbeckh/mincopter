#pragma once

#include <stdint.h>

// Union and and Failsafe typedefs
typedef union {
	struct {
			uint8_t home_is_set         : 1; // 0
											 //
			uint8_t arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
			uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
											 //
			uint8_t logging_started     : 1; // 6   // true if dataflash logging has started

			uint8_t takeoff_complete    : 1; // 8
			uint8_t land_complete       : 1; // 9   // true if we have detected a landing
											 //
			uint8_t usb_connected       : 1; // 15      // true if APM is powered from USB connection

			uint8_t rc_receiver_present : 1; // 18  // true if we have an rc receiver present (i.e. if we've ever received an update
	};
	uint32_t value;
} flight_state_t;

typedef struct {
	/* Failsafe
	 *
	 * The failsafe struct includes information about the telemetry, gps, and battery monitoring modules, each of which
	 * may trigger a failsafe. The flags are set/cleared throughout MinCopter but the (scheduled) failsafe check is where
	 * we actually take action to respond to a failsafe issue.
	 *
	 * Telemetry Failsafe
	 * - telemetry_active
	 * - telemetry_last_heartbeat_ms
	 * - telemetry_last_heartbeat_seq_id
	 *
	 * Battery Failsafe
	 * - battery_low
	 *
	 * GPS Failsafe
	 * TODO
	 *
	 */

	// BITFIELDS
	
	/* @brief Flags to determine if we execute each failsafe checks */
    uint8_t fs_enabled_battery : 1;
    uint8_t fs_enabled_gps     : 1;
    uint8_t fs_enabled_telem   : 1;

	/* @brief Flag to determine whether we are still connected to the telemetry. This will be set by the scheduled heartbeat function
	 * and checked during the failsafe checks */
	uint8_t telemetry_active : 1;

	/* @brief When we first connect to the telemetry, this is when we start to read/process commands. After this first, connection, we
	 * should not lose connection again, and will trigger a failsafe if we do */
	uint8_t telemetry_first_connect : 1;

	/* @brief Flag for low battery. Set by the battery monitor and checked during the failsafe checks */
	uint8_t battery_low : 1;

	// VARIABLES
	
	/* @brief Time since we last had a heartbeat message (in ms) */
    uint32_t telemetry_last_heartbeat_ms;

	/* @brief The identifier of the last sent telemetry packet */
	uint8_t telemetry_last_heartbeat_seq_id;

	// TODO Add GPS failsafe variables

} failsafe_t;




