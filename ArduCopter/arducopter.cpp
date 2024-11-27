// mincopter - henry

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera		:Auto Compass Declination
 *  Amilcar Lucas		:Camera mount library
 *  Andrew Tridgell		:General development, Mavlink Support
 *  Angel Fernandez		:Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid		:Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel			:DCM, Libraries, Control law advice
 *  Gregory Fletcher	:Camera mount orientation math
 *  Guntars				:Arming safety suggestion
 *  HappyKillmore		:Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle			:Alpha testing
 *  James Goppert		:Mavlink Support
 *  Jani Hiriven		:Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio			:Stabilization Control laws, MPU6k driver
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine			:Tri Support, Graphics
 *  Leonard Hall 		:Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini		:Lead tester
 *  Michael Oborne		:Mission Planner GCS
 *  Mike Smith			:Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraaction Layer (HAL)
 *  Robert Lefebvre		:Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.com/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

// Application dependencies
// #include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Menu.h>
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
// HASH include <AP_Camera.h>          // Photo or video camera
// HASH include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
// HASH include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "parameters.h"
#include "GCS.h"

// New headers
#include "attitude.h"
#include "compat.h"
#include "control_modes.h"
#include "events.h"
#include "failsafe.h"
#include "fence.h"
#include "log.h"
#include "motors.h"
#include "navigation.h"
#include "radio.h"
#include "system.h"
#include "util.h"

#include "serial.h"

// HAL Objects

AP_HAL::BetterStream* cliSerial;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
Parameters g;
AP_Scheduler scheduler;
AP_Notify notify;
DataFlash_APM2 DataFlash;
const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

GPS         *g_gps;
GPS_Glitch   gps_glitch(g_gps);

AP_Int8 *flight_modes = &g.flight_mode1;

AP_ADC_ADS7844 adc;
AP_InertialSensor_MPU6000 ins;

// NOTE which baro
  #if CONFIG_BARO == AP_BARO_BMP085
AP_Baro_BMP085 barometer;
  #elif CONFIG_BARO == AP_BARO_PX4
AP_Baro_PX4 barometer;
  #elif CONFIG_BARO == AP_BARO_MS5611
   #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
   #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
		// Confirmed this is the baro (the I2C version)
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
   #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
   #endif
  #endif

AP_Compass_HMC5843 compass;

// NOTE Almost certain ours is ublox
// real GPS selection
 #if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver;

 #else
  #error Unrecognised GPS_PROTOCOL setting.
 #endif // GPS PROTOCOL

// AHRS DCM
AP_AHRS_DCM ahrs(ins, g_gps);


#include "ap_union.h"
AP_UNION_T ap;

AP_BattMonitor battery;

// Radio
uint8_t oldSwitchPosition;
RCMapper rcmap;

// board specific config
AP_BoardConfig BoardConfig;


// Failsafe
AP_FAILSAFE_T failsafe;

// Motor Output
AP_MotorsQuad motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
AP_Relay relay;
// handle repeated servo and relay events
AP_ServoRelayEvents ServoRelayEvents(relay);
// a pin for reading the receiver RSSI voltage.
AP_HAL::AnalogSource* rssi_analog_source;
// Input sources for battery voltage, battery current, board vcc
AP_HAL::AnalogSource* board_vcc_analog_source;
AC_Fence    fence(&inertial_nav);

AP_InertialNav inertial_nav(&ahrs, &barometer, g_gps, gps_glitch);
AC_WPNav wp_nav(&inertial_nav, &ahrs, &g.pi_loiter_lat, &g.pi_loiter_lon, &g.pid_loiter_rate_lat, &g.pid_loiter_rate_lon);

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);


// receiver RSSI
uint8_t receiver_rssi;

// NOTE can remove tuning
// Placeholder for tuning values
float tuning_value;

/********************
ATTITUDE VARIABLES
********************/
// IMU raw roll rates
Vector3f omega;
// Desired Roll Angle (centi-degrees)
int16_t control_roll;
// Desired Pitch Angle (centi-degrees)
int16_t control_pitch;
// Desired Yaw
int32_t control_yaw;

// Control Mode
int8_t control_mode = STABILIZE;

// Orientation values used by DCM
float cos_roll_x         = 1.0;
float cos_pitch_x        = 1.0;
float cos_yaw            = 1.0;
float sin_yaw;
float sin_roll;
float sin_pitch;

// Rate Frame
uint8_t rate_targets_frame = EARTH_FRAME;

// Rate Controller Targets
int32_t roll_rate_target_ef;
int32_t pitch_rate_target_ef;
int32_t yaw_rate_target_ef;

int32_t roll_rate_target_bf;
int32_t pitch_rate_target_bf;
int32_t yaw_rate_target_bf;

// Throttle variables
int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
float throttle_avg;                  // g.throttle_cruise as a float
int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)

// The (throttle) controller desired altitude in cm
float controller_desired_alt;
// The Commanded Throttle from the autopilot.
int16_t nav_throttle;    // 0-1000 for throttle control

// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
Vector3f yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
int32_t yaw_look_at_heading;
// Deg/s we should turn
int16_t yaw_look_at_heading_slew;

// An additional throttle added to keep the copter at the same altitude when banking
int16_t angle_boost;
// counter to verify landings
uint16_t land_detector;

/********************
NAVIGATION VARIABLES
********************/
// This is the angle from the copter to the next waypoint in centi-degrees
int32_t wp_bearing;
// The original bearing to the next waypoint.  used to point the nose of the copter at the next waypoint
int32_t original_wp_bearing;
// The location of home in relation to the copter in centi-degrees
int32_t home_bearing;
// distance between plane and home in cm
int32_t home_distance;
// distance between plane and next waypoint in cm.
uint32_t wp_distance;

int32_t initial_armed_bearing;

// home location is stored when we have a good GPS lock and arm the copter
struct   Location home;
// Current location of the copter
struct   Location current_loc;

// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
int32_t altitude_error;
// The cm/s we are moving up or down based on filtered data - Positive = UP
int16_t climb_rate;
// The altitude as reported by Baro in cm – Values can be quite high
int32_t baro_alt;

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
float scaleLongDown = 1;
float lon_error, lat_error;      // Used to report how many cm we are from the next waypoint or loiter target position

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
float simple_cos_yaw = 1.0;
float simple_sin_yaw;
int32_t super_simple_last_bearing;
float super_simple_cos_yaw = 1.0;
float super_simple_sin_yaw;

/* FLIGHT MODES */
uint8_t yaw_mode = STABILIZE_YAW;
uint8_t roll_pitch_mode = STABILIZE_RP;
uint8_t throttle_mode = STABILIZE_THR;
uint8_t nav_mode;


// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
float G_Dt = 0.02;

// Performance monitoring
int16_t pmTest1;

// Time in microseconds of main control loop
uint32_t fast_loopTimer;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
uint16_t mainLoop_count;
// Used to exit the roll and pitch auto trim function
uint8_t auto_trim_counter;


void compass_accumulate(void) { if (g.compass_enabled) compass.accumulate(); }
void barometer_accumulate(void) { barometer.accumulate(); }

void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();
    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"),
                            (unsigned)perf_info_get_num_long_running(),
                            (unsigned)perf_info_get_num_loops(),
                            (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

// Forward Declarations
void loop(void);
void fast_loop(void);
void throttle_loop(void);
void read_AHRS();
void update_trig();
void run_rate_controllers();
void set_servos_4();
void read_control_switch();
void update_roll_pitch_mode();
void update_rate_controller_targets();
void update_throttle_mode();
void read_inertial_altitude();
void update_auto_armed();
void tuning(void);
void update_simple_mode(void);
void set_target_alt_for_reporting(float alt_cm);
float get_target_alt_for_reporting();
bool set_yaw_mode(uint8_t new_yaw_mode);
void update_yaw_mode();
void update_roll_pitch_mode();

// func globals

// attitude.h
float roll_in_filtered;     // roll-in in filtered with RC_FEEL_RP parameter
float pitch_in_filtered;    // pitch-in filtered with RC_FEEL_RP parameter

void loop()
{
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
        return;
    }
    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();
    scheduler.run(time_available - 300);
}


// Main loop - 100hz
void fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

    // run low level rate controllers that only require IMU data
    run_rate_controllers();

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // update targets to rate controllers
    update_rate_controller_targets();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

    // check if we've landed
    update_land_detector();

    // check auto_armed status
    update_auto_armed();
}

// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    switch(roll_pitch_mode) {
		// NO ACRO MODE

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        update_simple_mode();

        if(failsafe.radio) {
            // don't allow copter to fly away during failsafe
            get_pilot_desired_lean_angles(0.0f, 0.0f, control_roll, control_pitch);
        } else {
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);
        }

        // pass desired roll, pitch to stabilize attitude controllers
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // copy latest output from nav controller to stabilize controller
        control_roll = wp_nav.get_desired_roll();
        control_pitch = wp_nav.get_desired_pitch();
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;
    }

    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
    }

    if(ap.new_radio_frame) {
        // clear new radio frame info
        ap.new_radio_frame = false;
    }
}

void
init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = cos_yaw;
    simple_sin_yaw = sin_yaw;

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;
        pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;
        pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    g.rc_1.control_in = rollx*cos_yaw + pitchx*sin_yaw;
    g.rc_2.control_in = -rollx*sin_yaw + pitchx*cos_yaw;
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

// throttle_mode_manual - returns true if the throttle is directly controlled by the pilot
bool throttle_mode_manual(uint8_t thr_mode)
{
    return (thr_mode == THROTTLE_MANUAL || thr_mode == THROTTLE_MANUAL_TILT_COMPENSATED || thr_mode == THROTTLE_MANUAL_HELI);
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // return immediately if no change
    if( new_throttle_mode == throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL_TILT_COMPENSATED:
						// NOTE This is throttle mode used during STABILIZE
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);     // reset controller desired altitude to current altitude
            wp_nav.set_desired_alt(controller_desired_alt);                                 // same as above but for loiter controller
            if (throttle_mode_manual(throttle_mode)) {  // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
                set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
            }
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            reset_land_detector();  // initialise land detector
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;
    }

    // update the throttle mode
    if( throttle_initialised ) {
        throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        desired_climb_rate = 0;
        nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;
    int16_t pilot_throttle_scaled;

    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        set_target_alt_for_reporting(0);
        return;
    }

    switch(throttle_mode) {

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, true);

            // update estimate of throttle cruise
						update_throttle_cruise(pilot_throttle_scaled);

            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in wp_nav.get_desired_alt()
        if(ap.auto_armed) {
            // special handling if we are just taking off
            if (ap.land_complete) {
                // tell motors to do a slow start.
                motors.slow_start(true);
            }
            get_throttle_althold_with_slew(wp_nav.get_desired_alt(), -wp_nav.get_descent_velocity(), wp_nav.get_climb_velocity());
            set_target_alt_for_reporting(wp_nav.get_desired_alt()); // To-Do: return get_destination_alt if we are flying to a waypoint
        }else{
            // pilot's throttle must be at zero so keep motors off
            set_throttle_out(0, false);
            // deactivate accel based throttle controller
            throttle_accel_deactivate();
            set_target_alt_for_reporting(0);
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        set_target_alt_for_reporting(0);
        break;
    }
}

// set_target_alt_for_reporting - set target altitude in cm for reporting purposes (logs and gcs)
void set_target_alt_for_reporting(float alt_cm)
{
    target_alt_for_reporting = alt_cm;
}

// get_target_alt_for_reporting - returns target altitude in cm for reporting purposes (logs and gcs)
float get_target_alt_for_reporting()
{
    return target_alt_for_reporting;
}

void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    ahrs.update();
    omega = ins.get_gyro();
}

void update_trig(void){
    Vector2f yawvector;
    const Matrix3f &temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x;     // sin
    yawvector.y     = temp.b.x;         // cos
    yawvector.normalize();

    cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
    cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x     = constrain_float(cos_pitch_x, 0, 1.0);
    // this relies on constrain_float() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x      = constrain_float(cos_roll_x, -1.0, 1.0);

    sin_yaw         = constrain_float(yawvector.y, -1.0, 1.0);
    cos_yaw         = constrain_float(yawvector.x, -1.0, 1.0);

    // added to convert earth frame to body frame for rate controllers
    sin_pitch       = -temp.c.x;
    sin_roll        = temp.c.y / cos_pitch_x;

    // update wp_nav controller with trig values
    wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_pitch_x);

    //flat:
    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}

// read baro and sonar altitude at 20hz
void update_altitude()
{
    // read in baro altitude
    baro_alt            = read_barometer();

    // write altitude info to dataflash logs
		/*
    if (g.log_bitmask & MASK_LOG_CTUN) {
        Log_Write_Control_Tuning();
    }
		*/
}

void tuning(){

    // exit immediately when radio failsafe is invoked so tuning values are not set to zero
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case CH6_STABILIZE_ROLL_PITCH_KP:
        g.pi_stabilize_roll.kP(tuning_value);
        g.pi_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    // Yaw tuning
    case CH6_STABILIZE_YAW_KP:
        g.pi_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case CH6_ALTITUDE_HOLD_KP:
        g.pi_alt_hold.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KP:
        g.pid_throttle_rate.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KD:
        g.pid_throttle_rate.kD(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KP:
        g.pid_throttle_accel.kP(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KI:
        g.pid_throttle_accel.kI(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KD:
        g.pid_throttle_accel.kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case CH6_LOITER_POSITION_KP:
        g.pi_loiter_lat.kP(tuning_value);
        g.pi_loiter_lon.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KP:
        g.pid_loiter_rate_lon.kP(tuning_value);
        g.pid_loiter_rate_lat.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KI:
        g.pid_loiter_rate_lon.kI(tuning_value);
        g.pid_loiter_rate_lat.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KD:
        g.pid_loiter_rate_lon.kD(tuning_value);
        g.pid_loiter_rate_lat.kD(tuning_value);
        break;

    case CH6_WP_SPEED:
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav.set_horizontal_velocity(g.rc_6.control_in);
        break;

    // Acro roll pitch gain
    case CH6_ACRO_RP_KP:
        g.acro_rp_p = tuning_value;
        break;

    // Acro yaw gain
    case CH6_ACRO_YAW_KP:
        g.acro_yaw_p = tuning_value;
        break;

    case CH6_RELAY:
        if (g.rc_6.control_in > 525) relay.on(0);
        if (g.rc_6.control_in < 475) relay.off(0);
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain(g.rc_6.control_in);
        break;
#endif

    case CH6_OPTFLOW_KP:
        g.pid_optflow_roll.kP(tuning_value);
        g.pid_optflow_pitch.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KI:
        g.pid_optflow_roll.kI(tuning_value);
        g.pid_optflow_pitch.kI(tuning_value);
        break;

    case CH6_OPTFLOW_KD:
        g.pid_optflow_roll.kD(tuning_value);
        g.pid_optflow_pitch.kD(tuning_value);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE                                       // do not allow modifying _kp or _kp_yaw gains in HIL mode
    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;
#endif

    case CH6_INAV_TC:
        // To-Do: allowing tuning TC for xy and z separately
        inertial_nav.set_time_constant_xy(tuning_value);
        inertial_nav.set_time_constant_z(tuning_value);
        break;

    case CH6_DECLINATION:
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case CH6_CIRCLE_RATE:
        // set circle rate
        g.circle_rate.set(g.rc_6.control_in/25-20);     // allow approximately 45 degree turn rate in either direction
        break;

    case CH6_SONAR_GAIN:
        // set sonar gain
        g.sonar_gain.set(tuning_value);
        break;

    case CH6_LOIT_SPEED:
        // set max loiter speed to 0 ~ 1000 cm/s
        wp_nav.set_loiter_velocity(g.rc_6.control_in);
        break;
    }
}

// called at 50hz
void update_GPS(void)
{
    static uint32_t last_gps_reading;           // time of last gps message
    static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location
    bool report_gps_glitch;

    g_gps->update();

    // logging and glitch protection run after every gps message
    if (g_gps->last_message_time_ms() != last_gps_reading) {
        last_gps_reading = g_gps->last_message_time_ms();

        // log GPS message
        if (g.log_bitmask & MASK_LOG_GPS) {
            DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
        }

        // run glitch protection and update AP_Notify if home has been initialised
        if (ap.home_is_set) {
            gps_glitch.check_position();
            report_gps_glitch = (gps_glitch.glitching() && !ap.usb_connected);
            if (AP_Notify::flags.gps_glitching != report_gps_glitch) {
                if (gps_glitch.glitching()) {
                    Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_GPS_GLITCH);
                }else{
                    Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_ERROR_RESOLVED);
                }
                AP_Notify::flags.gps_glitching = report_gps_glitch;
            }
        }
    }

    // checks to initialise home and take location based pictures
    if (g_gps->new_data && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        // clear new data flag
        g_gps->new_data = false;

        // check if we can initialise home yet
        if (!ap.home_is_set) {
            // if we have a 3d lock and valid location
            if(g_gps->status() >= GPS::GPS_OK_FIX_3D && g_gps->latitude != 0) {
                if( ground_start_count > 0 ) {
                    ground_start_count--;
                }else{
                    // after 10 successful reads store home location
                    // ap.home_is_set will be true so this will only happen once
                    ground_start_count = 0;
                    init_home();

                    // set system clock for log timestamps
                    hal.util->set_system_clock(g_gps->time_epoch_usec());

                    if (g.compass_enabled) {
                        // Set compass declination automatically
                        compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                    }
                }
            }else{
                // start again if we lose 3d lock
                ground_start_count = 10;
            }
        }
    }

    // check for loss of gps
    failsafe_gps_check();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

#if HIL_MODE != HIL_MODE_ATTITUDE  // don't execute in HIL mode
    if(g.compass_enabled) {
        if (compass.read()) {
            compass.null_offsets();
        }
        // log compass information
        if (g.log_bitmask & MASK_LOG_COMPASS) {
            Log_Write_Compass();
        }
    }
#endif

}

// three_hz_loop - 3.3hz loop
void three_hz_loop()
{
    // check if we've lost contact with the ground station
    //failsafe_gcs_check();

    // check if we have breached a fence
    fence_check();

    update_events();

    if(g.radio_tuning > 0)
        tuning();
}

// one_hz_loop - runs at 1Hz
void one_hz_loop()
{
		// from serial.h
		print_GPS();
		print_RPY();

		// Print num logs
		uint16_t nl = DataFlash.get_num_logs();
		cliSerial->printf_P(PSTR("Num logs: %d\n"), nl);

    // pass latest alt hold kP value to navigation controller
    wp_nav.set_althold_kP(g.pi_alt_hold.kP());

    // update latest lean angle to navigation controller
    wp_nav.set_lean_angle_max(g.angle_max);

    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }

    // auto disarm checks
    auto_disarm_check();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);
    }

    // update assigned functions and enable auxiliar servos
    aux_servos_update_fn();
    enable_aux_servos();

    check_usb_mux();

}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    int16_t pilot_yaw = g.rc_4.control_in;

    // do not process pilot's yaw input during radio failsafe
    if (failsafe.radio) {
        pilot_yaw = 0;
    }

    switch(yaw_mode) {

		/* NOTE REMOVE DUE TO ACRO VARIABLE IN get_yaw_rate_stabilized_ef
    case YAW_HOLD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // heading hold at heading held in control_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(pilot_yaw);
        break;
		*/

		/* REMOVE ACRO
    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        get_yaw_rate_stabilized_bf(pilot_yaw);
        break;
		*/

    case YAW_LOOK_AT_NEXT_WP:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // point towards next waypoint (no pilot input accepted)
            // we don't use wp_bearing because we don't want the copter to turn too much during flight
            control_yaw = get_yaw_slew(control_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // point towards a location held in yaw_look_at_WP
        get_look_at_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

		// REMOVED get_circle_yaw

    case YAW_LOOK_AT_HOME:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // keep heading always pointing at home with no pilot input allowed
            control_yaw = get_yaw_slew(control_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
            control_yaw = get_yaw_slew(control_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        }
        get_stabilize_yaw(control_yaw);
        break;

	case YAW_LOOK_AHEAD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(pilot_yaw);
        break;

    case YAW_RESETTOARMEDYAW:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // changes yaw to be same as when quad was armed
            control_yaw = get_yaw_slew(control_yaw, initial_armed_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }

        break;
    }
}

// get yaw mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_wp_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {
        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
            return YAW_LOOK_AT_NEXT_WP;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if( rtl ) {
                return YAW_HOLD;
            }else{
                return YAW_LOOK_AT_NEXT_WP;
            }
            break;

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return YAW_LOOK_AHEAD;
            break;

        default:
            return YAW_HOLD;
            break;
    }
}

// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
            reset_roll_pitch_in_filters(g.rc_1.control_in, g.rc_2.control_in);
            roll_pitch_initialised = true;
            break;
        case ROLL_PITCH_AUTO:
            roll_pitch_initialised = true;
            break;

    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_RESETTOARMEDYAW:
            control_yaw = ahrs.yaw_sensor; // store current yaw so we can start rotating back to correct one
            yaw_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}


// NOTE henry - move scheduler table to end

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { throttle_loop,         2,     450 },
    { update_GPS,            2,     900 },
    { update_nav_mode,       1,     400 },
    { update_batt_compass,  10,     720 },
    { read_aux_switches,    10,      50 },
    { arm_motors_check,     10,      10 },
    { update_altitude,      10,    1000 },
    { run_nav_updates,      10,     800 },
    { three_hz_loop,        33,      90 },
    { compass_accumulate,    2,     420 },
    { barometer_accumulate,  2,     250 },
    { update_notify,         2,     100 },
    { one_hz_loop,         100,     420 },
    { crash_check,          10,      20 },
    { perf_update,        1000,     200 },
    { read_receiver_rssi,   10,      50 }
};

// Called by HAL
void setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

AP_HAL_MAIN();


