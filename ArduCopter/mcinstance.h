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
// HASH include <APM_PI.h>
#include <AC_PID.h>
#include <RC_Channel.h>

#ifdef TARGET_ARCH_AVR
	#include <AP_HAL_AVR.h>
#elif TARGET_ARCH_LINUX
	#include <AP_HAL_Linux.h>
	#include "sim_compass.h"
	#include "sim_adc.h"
	#include "sim_inertialsensor.h"
	#include "sim_gps.h"
	#include "sim_barometer.h"
#else
	#error "wrong target from within mcinstance.h"
#endif

#include "failsafe.h"

#include "config.h"

class MCInstance {

	public:
		/* @brief MCInstance is an abstraction of the mincopter inputs and outputs (in dev/). It is used by state and directly
		 * throughout other libraries and classes.
		 *
		 * This class should contain the 'wiring' between the various backend sensors and board architectures.
		 */
		MCInstance() :
#ifdef TARGET_ARCH_LINUX
			DataFlash("/home/henry/Documents/mc-dev/logs"),
#elif TARGET_ARCH_AVR
			DataFlash(),
#endif

#ifdef TARGET_ARCH_LINUX
			barometer(),
#elif TARGET_ARCH_AVR
			barometer(&AP_Baro_MS5611::spi),
#endif

			gps_glitch(g_gps),

#ifdef TARGET_ARCH_AVR
			g_gps_driver(&g_gps),
#elif TARGET_ARCH_LINUX
			g_gps_driver(),
#endif

			compass(),
			motors(&rc_1, &rc_2, &rc_3, &rc_4),
			//param_loader(var_info, WP_START_BYTE)

			// Parameters initialisations
        rc_1                (CH_1),
        rc_2                (CH_2),
        rc_3                (CH_3),
        rc_4                (CH_4),
        rc_5                (CH_5),
        rc_6                (CH_6),
        rc_7                (CH_7),
        rc_8                (CH_8)

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
#ifdef TARGET_ARCH_AVR
		DataFlash_APM2 DataFlash;
#elif TARGET_ARCH_LINUX
		DataFlash_File DataFlash;
#endif

		/* @brief ADC Instance used to obtain battery voltage levels */
#ifdef TARGET_ARCH_AVR
		AP_ADC_ADS7844 adc;
#elif TARGET_ARCH_LINUX
		AP_ADC_Sim adc;
#endif

		/* @brief Inertial Measurement Unit interface */
#ifdef TARGET_ARCH_AVR
		AP_InertialSensor_MPU6000 ins;
#elif TARGET_ARCH_LINUX
		AP_InertialSensor_Sim ins;
#endif

		const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

		/* @brief Barometer instance */
#ifdef TARGET_ARCH_AVR
		#if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
		AP_Baro_MS5611 barometer;
		#elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
		// TODO Remove this - I2C is not used for Baro
		// Confirmed this is the baro (the I2C version)
		AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
		#endif
#elif TARGET_ARCH_LINUX
		AP_Baro_Sim barometer;
#endif

		/* @brief Compass instance */
#ifdef TARGET_ARCH_AVR
		AP_Compass_HMC5843 compass;
#elif TARGET_ARCH_LINUX
		AP_Compass_Sim compass;
#endif

		/* @brief GPS Interface */
		GPS         *g_gps;
		GPS_Glitch   gps_glitch;

#ifdef TARGET_ARCH_AVR
		// NOTE Almost certain ours is ublox
		// TODO I'm pretty sure AP_GPS_Auto will include code for
		// all GPS backends into final executable and determine at
		// runtime. This clogs executable. Change this to a specific
		// GPS backend. I think ublox is correct for APM2.5
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
#elif TARGET_ARCH_LINUX
		AP_GPS_Sim g_gps_driver;
#endif

		// receiver RSSI
		uint8_t receiver_rssi;
		AP_HAL::AnalogSource* rssi_analog_source;

		/* @brief notify is used to control onboard LED behaviour */
		AP_Notify notify;

		// TODO Remove/Replace
		/* @brief Core parameters class which holds PID controllers and other objects */
		//Parameters g;
		
    int16_t        format_version;
    int8_t         software_type;
    int8_t         serial1_baud;
#if MAVLINK_COMM_NUM_BUFFERS > 2
    int8_5         serial2_baud;
#endif

    int8_t         failsafe_battery_enabled;   // battery failsafe enabled
    float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered
    int8_t         failsafe_gps_enabled;       // gps failsafe enabled
    int8_t         failsafe_gcs;               // ground station failsafe behavior
    int16_t        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position
    int8_t         compass_enabled;
    int8_t         rssi_pin;
    float        rssi_range;                 // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
    int16_t        angle_max;                  // maximum lean angle of the copter in centi-degrees
																							 //
    
    int8_t         command_total;
    int8_t         command_index;



    int8_t         failsafe_throttle;
    int16_t        failsafe_throttle_value;
    int16_t        throttle_mid;
    int16_t        log_bitmask;
    int8_t         esc_calibrate;
    int8_t         radio_tuning;
    int16_t        radio_tuning_high;
    int16_t        radio_tuning_low;
    int8_t         frame_orientation;
    int8_t         ch7_option;
    int8_t         ch8_option;
    int8_t         arming_check;

    RC_Channel              rc_1;
    RC_Channel              rc_2;
    RC_Channel              rc_3;
    RC_Channel              rc_4;
    RC_Channel_aux          rc_5;
    RC_Channel_aux          rc_6;
    RC_Channel_aux          rc_7;
    RC_Channel_aux          rc_8;

    int16_t                rc_speed; // speed of fast RC Channels in Hz


		// TODO Move all objects in parameters class here for now and then replace names in each function
		// !!!!!!!!!!!!!!!!!!!!!!!!!

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
		//AP_Int8 *flight_modes = &g.flight_mode1;

		// TODO Remove/change to BTree
		/* @brief Control mode variable - NOTE will be replaced */
		int8_t control_mode = STABILIZE;

		// TODO Remove
		AP_UNION_T ap;

		/****** THESE ARE CONTROL ABSTRACTIONS ******/

		// TODO This is not used - REMOVE
		// Rate Frame
		//uint8_t rate_targets_frame = EARTH_FRAME;


		// Throttle variables
		int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
		float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)

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


		/********************
		NAVIGATION VARIABLES
		********************/
		// The original bearing to the next waypoint.  used to point the nose of the copter at the next waypoint
		int32_t original_wp_bearing;
		// The location of home in relation to the copter in centi-degrees
		int32_t home_bearing;
		// distance between plane and home in cm
		int32_t home_distance;

		int32_t initial_armed_bearing;

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

		/* @brief Flight mode variables that get updated during call to set_mode */
		uint8_t yaw_mode = STABILIZE_YAW;
		uint8_t roll_pitch_mode = STABILIZE_RP;
		uint8_t throttle_mode = STABILIZE_THR;
		uint8_t nav_mode;

		/* @brief Integration time (in seconds) for the gyros (DCM algorithm) */
		// TODO Move this to the control folder as I believe it's the only place its used
		float G_Dt = 0.02;

		// Performance monitoring
		int16_t pmTest1;

		// Used to exit the roll and pitch auto trim function
		uint8_t auto_trim_counter;



	public:

		// TODO remove the call to ins.update() from the ahrs library and add the sensor update here


};


/* --- SENSOR UPDATES -----------------------------------------------------------------
* Each of these functions triggers an update round for a sensor in MCInstance.
* These functions should be called at 100Hz approx.
* ---------------------------------------------------------------------------------- */

/* @brief Triggers accumulation of compass sensor
*/
void read_compass(void);

/* @brief Triggers accumulation of barometer sensor
*/ 
void read_baro(void);

/* @brief Triggers reading of barometer and updates the `baro_alt` variable
*/
void update_altitude(void);

/* @brief Triggers update of the onboard GPS
*/
void update_GPS(void);
		
/* @brief Triggers reading of both the battery sensors (via `battery`) and the reading of the compass
*/
void read_batt_compass(void);

/* @brief Contains functions that should execute at 1Hz. Currently contains disarm checks and USB Mux checks
*/
void one_hz_loop(void);
