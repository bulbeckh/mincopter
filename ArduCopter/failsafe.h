#pragma once

#include "parameters.h"
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>


#include "navigation.h"
#include "util.h"
#include "motors.h"
#include "system.h"

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

extern uint16_t mainLoop_count;

/* @brief Helper functions for enabling and disabling failsafe
*/
void failsafe_enable();
void failsafe_disable();

/* @brief Tracks failsafe status by modifying in_failsafe parameter. Registered to run by scheduler during init_autopilot
*/
void failsafe_check();


// events.h

/* @brief Called when radio loses connection, triggering the failsafe to kick-in
*/
void failsafe_radio_on_event();

/* @brief Called when returning from a failsafe mode
*/
void failsafe_radio_off_event();

/* @brief Called when a low battery occurs, triggering failsafe
*/
void failsafe_battery_event(void);

/* @brief Called when GPS returns signal
*/
void failsafe_gps_off_event(void);


void fence_check();

extern int32_t home_distance;
extern Parameters g;
extern Vector3f omega;
extern AP_AHRS_DCM ahrs;
extern float G_Dt;
extern AP_MotorsQuad motors;
extern AP_InertialNav inertial_nav;
extern GPS         *g_gps;
extern GPS_Glitch   gps_glitch;
extern AP_Compass_HMC5843 compass;
extern AC_WPNav wp_nav;

extern AP_UNION_T ap;
extern AP_FAILSAFE_T failsafe;
extern AC_Fence fence;

// from arducopter.cpp
extern float cos_roll_x;
extern float cos_pitch_x;
extern float cos_yaw;
extern float sin_yaw;
extern float sin_roll;
extern float sin_pitch;
extern uint8_t rate_targets_frame;    // indicates whether rate targets provided in earth or body frame
extern int32_t roll_rate_target_ef;
extern int32_t pitch_rate_target_ef;
extern int32_t yaw_rate_target_ef;
extern int32_t roll_rate_target_bf;     // body frame roll rate target
extern int32_t pitch_rate_target_bf;    // body frame pitch rate target
extern int32_t yaw_rate_target_bf;      // body frame yaw rate target
extern int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
extern bool throttle_accel_controller_active;   // true when accel based throttle controller is act
extern float throttle_avg;                  // g.throttle_cruise as a float
extern int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
extern float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)

extern struct   Location home;
extern struct   Location current_loc;
extern struct   Location command_nav_queue;
extern struct   Location command_cond_queue;

extern int32_t control_yaw;
extern Vector3f yaw_look_at_WP;
extern int32_t yaw_look_at_WP_bearing;
extern int32_t yaw_look_at_heading;
extern int16_t yaw_look_at_heading_slew;

extern float controller_desired_alt;
extern int32_t altitude_error;
extern int16_t climb_rate;
extern int16_t sonar_alt;
extern uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
extern float target_sonar_alt;      // desired altitude in cm above the ground
extern int32_t baro_alt;

extern int16_t angle_boost;
extern uint16_t land_detector;

extern Vector3f circle_center;
extern float circle_angle;
extern float circle_angle_total;
extern uint8_t circle_desired_rotations;
extern float circle_angular_acceleration;       // circle mode's angular acceleration
extern float circle_angular_velocity;           // circle mode's angular velocity
extern float circle_angular_velocity_max;       // circle mode's max angular velocity
extern uint16_t loiter_time_max;
extern uint32_t loiter_time;



