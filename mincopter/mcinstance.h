#pragma once

#include <math.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

#include "telemetry.h"

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
		MCInstance() :
#ifdef MC_STORAGE_FILE
			DataFlash("/home/henry/Documents/mc-dev/logs"),
#elif  MC_STORAGE_DATAFLASH
			DataFlash(),
#elif  MC_STORAGE_EMPTY
			DataFlash(),
#endif

#ifdef MC_BARO_SIM
			barometer(),
#elif  MC_BARO_MS5611
			barometer(&AP_Baro_MS5611::spi),
#elif  MC_BARO_BME280
			barometer(),
#elif  MC_BARO_NONE
			barometer(),
#endif

			compass(),

			gps_glitch(g_gps),

#ifdef MC_GPS_AUTO
			g_gps_driver(&g_gps)
#elif  MC_GPS_SIM
			g_gps_driver()
#elif  MC_GPS_UBLOX
			g_gps_driver()
#elif  MC_GPS_NONE
			g_gps_driver()
#endif

		{
		}

	public:

		AP_BattMonitor battery;

		Telemetry telemetry;

#ifdef MC_STORAGE_FILE
		DataFlash_File DataFlash;
#elif  MC_STORAGE_DATAFLASH
		DataFlash_APM2 DataFlash;
#elif  MC_STORAGE_EMPTY
		DataFlash_Empty DataFlash;
#endif

#ifdef MC_ADC_ADS7844
		AP_ADC_ADS7844 adc;
#elif  MC_ADC_NONE
		AP_ADC_None adc;
#elif  MC_ADC_SIM
		AP_ADC_Sim adc;
#endif

#ifdef MC_IMU_MPU6000
		AP_InertialSensor_MPU6000 ins;
#elif  MC_IMU_MPU6050
		AP_InertialSensor_MPU6050 ins;
#elif  MC_IMU_ICM20948
		AP_InertialSensor_ICM20948 ins;
#elif  MC_IMU_SIM
		AP_InertialSensor_Sim ins;
#elif  MC_IMU_NONE
		AP_InertialSensor_None ins;
#endif

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
#elif  MC_BARO_NONE
		AP_Baro_None barometer;
#endif

#ifdef MC_COMP_HMC5843
		AP_Compass_HMC5843 compass;
#elif  MC_COMP_ICM20948
		AP_Compass_ICM20948 compass;
#elif  MC_COMP_SIM
		AP_Compass_Sim compass;
#elif  MC_COMP_NONE
		AP_Compass_None compass;
#endif

		GPS* g_gps;

		GPS_Glitch gps_glitch;

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
		AP_GPS_Sim   g_gps_driver;
#elif MC_GPS_UBLOX
		AP_GPS_UBLOX g_gps_driver;
#elif MC_GPS_NONE
		AP_GPS_None  g_gps_driver;
#endif

		uint8_t receiver_rssi;

		AP_HAL::AnalogSource* rssi_analog_source;

    	int8_t rssi_pin;
    	float rssi_range;

		// a pin for reading the receiver RSSI voltage.
		// Input sources for battery voltage, battery current, board vcc
		AP_HAL::AnalogSource* board_vcc_analog_source;

		/* @brief HAL reference */
		const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
};

// TODO Move all of these

/* @brief Run all required failsafe checks */
void failsafe_checks(void);

/* @brief Read and process incoming telemetry commands */
void read_telemetry(void);

/* @brief Send a heartbeat message to the remote telemetry */
void send_telemetry_heartbeat(void);

/* @brief Triggers accumulation of compass sensor
*/
void accumulate_compass(void);

/* @brief Triggers accumulation of barometer sensor
*/ 
void read_barometer(void);

/* @brief Triggers reading of barometer and updates the `baro_alt` variable
*/
void accumulate_barometer(void);

/* @brief Triggers update of the onboard GPS
*/
void update_GPS(void);
		
/* @brief Triggers reading of both the battery sensors (via `battery`) and the reading of the compass
*/
void read_batt_compass(void);
