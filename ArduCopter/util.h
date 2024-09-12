#pragma once

#include <stdint.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Notify.h>
#include <AP_HAL.h>
#include <AP_Motors.h>
#include "parameters.h"
#include <AP_AHRS.h>
#include <AP_InertialNav.h>

#include "defines.h"
#include "log.h"
#include "events.h"
#include "motors.h"
#include "ap_union.h"

extern AP_UNION_T ap;
extern AP_FAILSAFE_T failsafe;

extern const AP_HAL::HAL& hal;
extern AP_MotorsQuad motors;
extern Parameters g;
extern int8_t control_mode;
extern AP_AHRS_DCM ahrs;
extern int32_t baro_alt;
extern float G_Dt;
extern AP_InertialNav inertial_nav;

extern struct   Location home;
extern struct   Location current_loc;

extern AP_Notify notify;

extern float scaleLongUp; 
extern float scaleLongDown;
extern int16_t climb_rate;


// from sensors but added into util
void init_barometer(bool full_calibration);
int32_t read_barometer(void);
void init_compass();
void read_battery(void);
void read_receiver_rssi(void);

// from arducopter.cpp
void update_super_simple_bearing(bool force_update);


// AP_State.pde
void set_auto_armed(bool b);
void set_home_is_set(bool b);
void set_simple_mode(uint8_t b);
void set_failsafe_radio(bool b);
void set_failsafe_battery(bool b);
void set_failsafe_gps(bool b);
//void set_failsafe_gcs(bool b);
void set_takeoff_complete(bool b);
void set_land_complete(bool b);
void set_pre_arm_check(bool b);
void set_pre_arm_rc_check(bool b);

// compat.pde 
void delay(uint32_t ms);

// REMOVE mavlink func
// void mavlink_delay(uint32_t ms);

uint32_t millis();
uint32_t micros();
void pinMode(uint8_t pin, uint8_t output);
void digitalWrite(uint8_t pin, uint8_t out);
uint8_t digitalRead(uint8_t pin);

// crash_check.pde
void crash_check();

// inertia.pde
void read_inertia();
void read_inertial_altitude();

// leds.pde
void update_notify();

// perf_info.pde
/* GLOBALS */
/*
uint16_t perf_info_loop_count;
uint32_t perf_info_max_time;
uint16_t perf_info_long_running;
*/

void perf_info_reset();
void perf_info_check_loop_time(uint32_t time_in_micros);
uint16_t perf_info_get_num_loops();
uint32_t perf_info_get_max_time();
uint16_t perf_info_get_num_long_running();

// position_vector.pde
Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt);
Vector3f pv_location_to_vector(Location loc);
int32_t pv_get_lat(const Vector3f pos_vec);
int32_t pv_get_lon(const Vector3f &pos_vec);
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);

void init_home();



