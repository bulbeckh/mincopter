#pragma once

#include <math.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_Motors.h>          // AP Motors library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>


#include "parameters.h"
#include "failsafe.h"

/*
HASH include <AP_Common.h>
HASH include <AP_Progmem.h>
HASH include <AP_Param.h>
HASH include <APM_PI.h>             // PI library
HASH include <AC_PID.h>             // PID library
HASH include <AP_Notify.h>          // Notify library
*/

// HASH include <RC_Channel.h>         // RC Channel Library
// HASH include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
// HASH include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
// HASH include <Filter.h>             // Filter library
// HASH include <AP_Buffer.h>          // APM FIFO Buffer
// HASH include <AP_Vehicle.h>         // needed for AHRS build
// HASH include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
// HASH include <AP_RCMapper.h>        // RC input mapping library

/* MCInstance is an abstraction of the MinCopter inputs and outputs. It holds the interfaces for each of the sensors
* like the IMU (ins), GPS, and also the interface to the motors (ESCs).
*
*
*/

// NOTE This should be in header file of AP_Baro
//extern AP_Baro_MS5611_SPI AP_Baro_MS5611::spi;

class MCInstance {

	public:
		MCInstance() :
			barometer(&AP_Baro_MS5611::spi),
			gps_glitch(g_gps),
			g_gps_driver(&g_gps),
			motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4),
			param_loader(var_info, WP_START_BYTE)
		{
		}

	public:
		/* --- COMPONENTS ---------------------------------------------------------------------
		* The following objects all represent the backend implementations of HAL components.
		* They expose functions for retrieving data and updating state. Their individual driver
		* libraries are found in libraries/ and are component-specific, not board-specific.
		* ---------------------------------------------------------------------------------- */

		AP_BattMonitor battery;

		/* @brief ATMEL DataFlash interface for flash storage */
		DataFlash_APM2 DataFlash;

		/* @brief ADC Instance used to obtain battery voltage levels */
		AP_ADC_ADS7844 adc;

		/* @brief Inertial Measurement Unit interface */
		AP_InertialSensor_MPU6000 ins;
		const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

		/* @brief Barometer instance */
		#if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
		AP_Baro_MS5611 barometer;
		#elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
		// TODO Remove this - I2C is not used for Baro
		// Confirmed this is the baro (the I2C version)
		AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
		#endif

		/* @brief Compass instance */
		AP_Compass_HMC5843 compass;

		/* @brief GPS Interface */
		GPS         *g_gps;
		GPS_Glitch   gps_glitch;

		// NOTE Almost certain ours is ublox
		 #if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
		AP_GPS_Auto     g_gps_driver;

		// TODO Remove the remaining GPS objects
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

		// receiver RSSI
		uint8_t receiver_rssi;
		AP_HAL::AnalogSource* rssi_analog_source;

		/* @brief notify is used to control onboard LED behaviour */
		AP_Notify notify;

		// TODO Remove/Replace
		/* @brief Core parameters class which holds PID controllers and other objects */
		Parameters g;

		// Motor Output
		AP_MotorsQuad motors;

		// a pin for reading the receiver RSSI voltage.
		// Input sources for battery voltage, battery current, board vcc
		AP_HAL::AnalogSource* board_vcc_analog_source;

		/* --- FLIGHT ABSTRACTIONS  -----------------------------------------------------------
		*
		*
		* ---------------------------------------------------------------------------------- */

		/* @brief HAL reference */
		const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

		/* @brief Reference to BetterStream used for communicating over serial */
		AP_HAL::BetterStream* cliSerial;


		// TODO Move to BTree
		/* @brief Array of flight modes, defaulting to 1 */
		AP_Int8 *flight_modes = &g.flight_mode1;

		// TODO Remove/change to BTree
		/* @brief Control mode variable - NOTE will be replaced */
		int8_t control_mode = STABILIZE;

		AP_UNION_T ap;


		// TODO Remove
		// setup the var_info table
		AP_Param param_loader;

		// TODO Move to BTree
		// Rate Frame
		uint8_t rate_targets_frame = EARTH_FRAME;

		// TODO Move to BTree
		/* @brief Rate controller targets updated by update_rate_controller_targets and feed into PID rate controllers */
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

		/* @brief Flight mode variables that get updated during call to set_mode */
		uint8_t yaw_mode = STABILIZE_YAW;
		uint8_t roll_pitch_mode = STABILIZE_RP;
		uint8_t throttle_mode = STABILIZE_THR;
		uint8_t nav_mode;

		/* @brief Integration time (in seconds) for the gyros (DCM algorithm) */
		float G_Dt = 0.02;

		// Performance monitoring
		int16_t pmTest1;

		// Used to exit the roll and pitch auto trim function
		uint8_t auto_trim_counter;

		// attitude.h
		float roll_in_filtered;     // roll-in in filtered with RC_FEEL_RP parameter
		float pitch_in_filtered;    // pitch-in filtered with RC_FEEL_RP parameter


	public:

		// TODO remove the call to ins.update() from the ahrs library and add the sensor update here


};


/* Sensor updates */

void read_compass(void);

void read_baro(void);

void update_altitude(void);

void update_GPS(void);
		
void read_batt_compass(void);

void one_hz_loop(void);
