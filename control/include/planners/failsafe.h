#pragma once

#include <stdint.h>

// Union and and Failsafe typedefs
typedef union {
		struct {
				uint8_t home_is_set         : 1; // 0
				uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE

				uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
				uint8_t pre_arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
				uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
				uint8_t logging_started     : 1; // 6   // true if dataflash logging has started

				uint8_t do_flip             : 1; // 7   // Used to enable flip code
				uint8_t takeoff_complete    : 1; // 8
				uint8_t land_complete       : 1; // 9   // true if we have detected a landing

				uint8_t new_radio_frame     : 1; // 10      // Set true if we have new PWM data to act on from the Radio
				uint8_t CH7_flag            : 2; // 11,12   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
				uint8_t CH8_flag            : 2; // 13,14   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
				uint8_t usb_connected       : 1; // 15      // true if APM is powered from USB connection
				uint8_t yaw_stopped         : 1; // 16      // Used to manage the Yaw hold capabilities

				uint8_t disable_stab_rate_limit : 1; // 17  // disables limits rate request from the stability controller

				uint8_t rc_receiver_present : 1; // 18  // true if we have an rc receiver present (i.e. if we've ever received an update
		};
		uint32_t value;
} AP_UNION_T;

typedef struct {
    uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
    uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
    uint8_t gps                 : 1; // 3   // A status flag for the gps failsafe
    uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe

    int8_t radio_counter;                  // number of iterations with throttle below throttle_fs_value

    uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} AP_FAILSAFE_T;




