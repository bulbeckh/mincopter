// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "util.h"

#include "mcinstance.h"
#include "mcstate.h"
#include "config.h"

#include "log.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "planner.h"
#include "planner_waypoint.h"
extern WP_Planner planner;

#include "control.h"
#include "controller_pid.h"
extern PID_Controller controller;

// ap_state.pde
void set_home_is_set(bool b)
{
    // if no change, exit immediately
    if( planner.ap.home_is_set == b )
        return;

    planner.ap.home_is_set 	= b;
    if(b) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( planner.ap.auto_armed == b )
        return;

    planner.ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// TODO Why aren't these in failsafe.cpp
// TODO Remove this whole function
// ---------------------------------------------
void set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(planner.failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        planner.failsafe.radio = b;

        if (planner.failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            //failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            //failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void set_failsafe_battery(bool b)
{
    planner.failsafe.battery = b;
    AP_Notify::flags.failsafe_battery = b;
}


// ---------------------------------------------
void set_failsafe_gps(bool b)
{
    planner.failsafe.gps = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gps = b;
}

// ---------------------------------------------
void set_takeoff_complete(bool b)
{
    // if no change, exit immediately
    if( planner.ap.takeoff_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_TAKEOFF);
    }
    planner.ap.takeoff_complete = b;
}

// ---------------------------------------------
void set_land_complete(bool b)
{
    // if no change, exit immediately
    if( planner.ap.land_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    }else{
        Log_Write_Event(DATA_NOT_LANDED);
    }
    planner.ap.land_complete = b;
}

// ---------------------------------------------

void set_pre_arm_check(bool b)
{
    if(planner.ap.pre_arm_check != b) {
        planner.ap.pre_arm_check = b;
        AP_Notify::flags.pre_arm_check = b;
    }
}

void set_pre_arm_rc_check(bool b)
{
    if(planner.ap.pre_arm_rc_check != b) {
        planner.ap.pre_arm_rc_check = b;
    }
}

// compat.pde

// TODO Why are these two the same? remove one
void delay(uint32_t ms)
{
    mincopter.hal.scheduler->delay(ms);
}

void mavlink_delay(uint32_t ms)
{
    mincopter.hal.scheduler->delay(ms);
}

// TODO inline this
uint32_t millis()
{
    return mincopter.hal.scheduler->millis();
}

uint32_t micros()
{
    return mincopter.hal.scheduler->micros();
}

void pinMode(uint8_t pin, uint8_t output)
{
    mincopter.hal.gpio->pinMode(pin, output);
}

void digitalWrite(uint8_t pin, uint8_t out)
{
    mincopter.hal.gpio->write(pin,out);
}

uint8_t digitalRead(uint8_t pin)
{
    return mincopter.hal.gpio->read(pin);
}


// Code to detect a crash main ArduCopter code
#ifndef CRASH_CHECK_ITERATIONS_MAX
 # define CRASH_CHECK_ITERATIONS_MAX        20      // 2 second (ie. 10 iterations at 10hz) inverted indicates a crash
#endif
#ifndef CRASH_CHECK_ANGLE_DEVIATION_CD
 # define CRASH_CHECK_ANGLE_DEVIATION_CD    2000    // 20 degrees beyond angle max is signal we are inverted
#endif
#ifndef CRASH_CHECK_ALT_CHANGE_LIMIT_CM
 # define CRASH_CHECK_ALT_CHANGE_LIMIT_CM   50      // baro altitude must not change by more than 50cm
#endif

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// should be called at 10hz
void crash_check()
{
    static uint8_t inverted_count;  // number of iterations we have been inverted
    static int32_t baro_alt_prev;

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!mincopter.motors.armed() || (mincopter.rc_3.control_in != 0 && !planner.failsafe.radio)) {
        inverted_count = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
		/*
    if (mincopter.control_mode == ACRO || mincopter.ap.do_flip) {
        inverted_count = 0;
        return;
    }
		*/

    // check angles
    int32_t lean_max = planner.angle_max + CRASH_CHECK_ANGLE_DEVIATION_CD;
    if (labs(mcstate.ahrs.roll_sensor) > lean_max || labs(mcstate.ahrs.pitch_sensor) > lean_max) {
        inverted_count++;

        // if we have just become inverted record the baro altitude
        if (inverted_count == 1) {
            baro_alt_prev = planner.baro_alt;

        // exit if baro altitude change indicates we are moving (probably falling)
        }else if (labs(planner.baro_alt - baro_alt_prev) > CRASH_CHECK_ALT_CHANGE_LIMIT_CM) {
            inverted_count = 0;
            return;

        // check if inverted for 2 seconds
        }else if (inverted_count >= CRASH_CHECK_ITERATIONS_MAX) {
            // log an error in the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
            // send message to gcs
            //gcs_send_text_P(SEVERITY_HIGH,PSTR("Crash: Disarming"));
            // disarm motors
            //init_disarm_motors();
        }
    }else{
        // we are not inverted so reset counter
        inverted_count = 0;
    }
}

// inertia.pde

// read_inertia - read inertia in from accelerometers
void read_inertia()
{
    // inertial altitude estimates
    mcstate.inertial_nav.update(mincopter.G_Dt);
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
void read_inertial_altitude()
{
    // with inertial nav we can update the altitude and climb rate at 50hz
    mcstate.current_loc.alt = mcstate.inertial_nav.get_altitude();
    controller.climb_rate = mcstate.inertial_nav.get_velocity_z();
}

// leds.pde

// updates the status of notify
// should be called at 50hz
void update_notify()
{
    mincopter.notify.update();
}

// position_vector.pde

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt)
{
    Vector3f tmp((lat-mcstate.home.lat) * LATLON_TO_CM, (lon-mcstate.home.lng) * LATLON_TO_CM * planner.scaleLongDown, alt);
    return tmp;
}

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_location_to_vector(Location loc)
{
    Vector3f tmp((loc.lat-mcstate.home.lat) * LATLON_TO_CM, (loc.lng-mcstate.home.lng) * LATLON_TO_CM * planner.scaleLongDown, loc.alt);
    return tmp;
}

// pv_get_lon - extract latitude from position vector
int32_t pv_get_lat(const Vector3f pos_vec)
{
    return mcstate.home.lat + (int32_t)(pos_vec.x / LATLON_TO_CM);
}

// pv_get_lon - extract longitude from position vector
int32_t pv_get_lon(const Vector3f &pos_vec)
{
    return mcstate.home.lng + (int32_t)(pos_vec.y / LATLON_TO_CM * planner.scaleLongUp);
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

void init_barometer(bool full_calibration)
{
    if (full_calibration) {
        mincopter.barometer.calibrate();
    }else{
        mincopter.barometer.update_calibration();
    }
}


void init_compass()
{
    if (!mincopter.compass.init() || !mincopter.compass.read()) {
        return;
    }
    mcstate.ahrs.set_compass(&mincopter.compass);
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (mincopter.rssi_range <= 0) {
        mincopter.receiver_rssi = 0;
    }else{
        mincopter.rssi_analog_source->set_pin(mincopter.rssi_pin);
        float ret = mincopter.rssi_analog_source->voltage_average() * 255 / mincopter.rssi_range;
        mincopter.receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

void init_home()
{
    set_home_is_set(true);
    mcstate.home.id         = 0; //previously MAV_CMD_NAV_WAYPOINT
    mcstate.home.lng        = mincopter.g_gps->longitude;                                 // Lon * 10**7
    mcstate.home.lat        = mincopter.g_gps->latitude;                                  // Lat * 10**7
    mcstate.home.alt        = mincopter.g_gps->altitude_cm;                                                        // Home is always 0

    // Save Home to EEPROM
    // -------------------
    // no need to save this to EPROM
		// REMOVED
    //set_cmd_with_index(home, 0);

    // set inertial nav's home position
	
	// NOTE During the simulation, the simulated values may be set to zero when this is called
    mcstate.inertial_nav.set_home_position(mincopter.g_gps->longitude, mincopter.g_gps->latitude);

    if (mincopter.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(0, &mcstate.home);

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    planner.scaleLongDown = longitude_scale(mcstate.home);
    planner.scaleLongUp   = 1.0f/planner.scaleLongDown;
}


// returns true if the GPS is ok and home position is set
bool GPS_ok()
{
    if (mincopter.g_gps != NULL && planner.ap.home_is_set && mincopter.g_gps->status() == GPS::GPS_OK_FIX_3D && !mincopter.gps_glitch.glitching() && !planner.failsafe.gps) {
        return true;
    }else{
        return false;
    }
}


// update_auto_armed - update status of auto_armed flag
void update_auto_armed()
{
    // disarm checks
    if(planner.ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!mincopter.motors.armed()) {
            set_auto_armed(false);
            return;
        }

    }else{
        // arm checks
        
        // if motors are armed and throttle is above zero auto_armed should be true
        if(mincopter.motors.armed() && mincopter.rc_3.control_in != 0) {
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
    if (usb_check == planner.ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    planner.ap.usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (planner.ap.usb_connected) {
        mincopter.hal.uartA->begin(SERIAL0_BAUD);
    } else {
        mincopter.hal.uartA->begin(map_baudrate(mincopter.serial1_baud, SERIAL1_BAUD));
    }
#endif
}

