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
#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

// HASH include <APM_PI.h>
#include <RC_Channel.h>

#if TARGET_ARCH_LINUX
	#include "sim_compass.h"
	#include "sim_adc.h"
	#include "sim_inertialsensor.h"
	#include "sim_gps.h"
	#include "sim_barometer.h"
#endif

#include "config.h"

class MCInstance {

	public:
		/* @brief MCInstance is an abstraction of the mincopter inputs and outputs (in dev/). It is used by state and directly
		 * throughout other libraries and classes.
		 *
		 * This class should contain the 'wiring' between the various backend sensors and board architectures.
		 */
		MCInstance() :
#ifdef MC_STORAGE_FILE
			DataFlash("/home/henry/Documents/mc-dev/logs"),
#elif  MC_STORAGE_DATAFLASH
			DataFlash(),
#endif

#ifdef MC_BARO_SIM
			barometer(),
#elif  MC_BARO_MS5611
			barometer(&AP_Baro_MS5611::spi),
#elif  MC_BARO_BME280
			barometer(),
#endif

			gps_glitch(g_gps),

#ifdef MC_GPS_AUTO
			g_gps_driver(&g_gps),
#elif  MC_GPS_SIM
			g_gps_driver(),
#elif  MC_GPS_UBLOX
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
#ifdef MC_STORAGE_FILE
		DataFlash_File DataFlash;
#elif  MC_STORAGE_DATAFLASH
		DataFlash_APM2 DataFlash;
#endif

		/* @brief ADC Instance used to obtain battery voltage levels */
#ifdef MC_ADC_ADS7844
		AP_ADC_ADS7844 adc;
#elif  MC_ADC_NONE
		AP_ADC_None adc;
#elif  MC_ADC_SIM
		AP_ADC_Sim adc;
#endif

		/* @brief Inertial Measurement Unit interface */
#ifdef MC_IMU_MPU6000
		AP_InertialSensor_MPU6000 ins;
#elif  MC_IMU_MPU6050
		AP_InertialSensor_MPU6050 ins;
#elif  MC_IMU_SIM
		AP_InertialSensor_Sim ins;
#endif

		/* @brief Barometer instance */
#ifdef MC_BARO_MS5611
		// TODO Whether to use I2C or SPI should be a separate configuration, where the wiring is also specified
		// HASH if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
		AP_Baro_MS5611 barometer;
		// HASH elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
		// TODO Remove this - I2C is not used for Baro
		// Confirmed this is the baro (the I2C version)
		// AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
		// HASH endif
#elif  MC_BARO_BME280
		AP_Baro_BME280 barometer;
#elif  MC_BARO_SIM
		AP_Baro_Sim barometer;
#endif

		/* @brief Compass instance */
#ifdef MC_COMP_HMC5843
		AP_Compass_HMC5843 compass;
#elif  MC_COMP_ICM20948
		AP_Compass_ICM20948 compass;
#elif  MC_COMP_SIM
		AP_Compass_Sim compass;
#endif

		/* @brief GPS Interface */
		GPS         *g_gps;
		GPS_Glitch   gps_glitch;

#ifdef MC_GPS_AUTO
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
#elif MC_GPS_SIM
		AP_GPS_Sim g_gps_driver;
#elif MC_GPS_UBLOX
		AP_GPS_UBLOX g_gps_driver;
#endif

		// receiver RSSI
		uint8_t receiver_rssi;
		AP_HAL::AnalogSource* rssi_analog_source;


		/* RSSI Information - used as part of voltage readers */
    int8_t         rssi_pin;
    float        	 rssi_range;                 // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
																						
		/* Bitmask to determin what to log */
    int16_t        log_bitmask;

		/* Used during startup checks and calibration */
    int8_t         esc_calibrate;

		/* Passed to motors on startup TODO move to compile-time variable */
    int8_t         frame_orientation;

		// TODO Move this to planner or another class that manages arming/disarming
    int8_t         arming_check;

		/* RC Channels
		 *
		 * rc_1 -> Roll
		 * rc_2 -> Pitch
		 * rc_3 -> Throttle
		 * rc_4 -> Yaw
		 * rc_5 -> undefined
		 * rc_6 -> undefined
		 * rc_7 -> undefined
		 * rc_8 -> undefined
		 */
    RC_Channel              rc_1;
    RC_Channel              rc_2;
    RC_Channel              rc_3;
    RC_Channel              rc_4;
    RC_Channel_aux          rc_5;
    RC_Channel_aux          rc_6;
    RC_Channel_aux          rc_7;
    RC_Channel_aux          rc_8;

    int16_t                rc_speed; // speed of fast RC Channels in Hz

		// Motor Output
		AP_MotorsQuad motors;

		// a pin for reading the receiver RSSI voltage.
		// Input sources for battery voltage, battery current, board vcc
		AP_HAL::AnalogSource* board_vcc_analog_source;

		/* @brief HAL reference */
		const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

		/* @brief Reference to BetterStream used for communicating over serial */
		AP_HAL::BetterStream* cliSerial;

		/* @brief Integration time (in seconds) for the gyros (DCM algorithm) */
		// TODO Move this to the control folder as I believe it's the only place its used
		float G_Dt = 0.02;

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
