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
		/* @brief Constructor for MCInstance */
		MCInstance(
			DataFlash_Class& mc_dataflash,
			AP_Baro& mc_barometer,
			Compass& mc_compass,
			GPS* mc_gps,
			GPS_Glitch& mc_gps_glitch,
			AP_BattMonitor& mc_battery,
			Telemetry& mc_telemetry,
			AP_InertialSensor& mc_ins,
			AP_ADC& mc_adc) :

			DataFlash(mc_dataflash),
			barometer(mc_barometer),
			compass(mc_compass),
			g_gps(mc_gps),
			gps_glitch(mc_gps_glitch),
			battery(mc_battery),
			telemetry(mc_telemetry),
			ins(mc_ins),
			adc(mc_adc) { }

	public:

		// TODO Remove this camel case
		// DataFlash
		DataFlash_Class& DataFlash;

		// Barometer
		AP_Baro& barometer;
		
		// Compass
		Compass& compass;

		// GPS Object
		GPS* g_gps;

		GPS_Glitch& gps_glitch;

		// Battery Monitor
		AP_BattMonitor& battery;

		// Telemetry
		Telemetry& telemetry;

		// IMU
		AP_InertialSensor& ins;
		
		// External ADC
		AP_ADC& adc;


		/* @brief Radio rssi signal */
		uint8_t receiver_rssi;

		AP_HAL::AnalogSource* rssi_analog_source;

		int8_t rssi_pin;
		float rssi_range;

		// a pin for reading the receiver RSSI voltage.
		// Input sources for battery voltage, battery current, board vcc
		AP_HAL::AnalogSource* board_vcc_analog_source;

		// TODO Change how the AP_HAL is passed in
		/* @brief HAL reference */
		const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

	public:
		/* @brief Initialise all subsystems under MCInstance */
		void init_ardupilot(void);
};

// TODO Move all of these

/* @brief Run all required failsafe checks */
void failsafe_checks(void);

/* @brief Send a heartbeat message to the remote telemetry */
void send_telemetry_heartbeat(void);

/* @brief Read and process incoming telemetry commands */
void read_telemetry(MCInstance&);


/* @brief Triggers accumulation of compass sensor
*/
void accumulate_compass(MCInstance&);

/* @brief Triggers accumulation of barometer sensor
*/ 
void read_barometer(MCInstance&);

/* @brief Triggers reading of barometer and updates the `baro_alt` variable
*/
void accumulate_barometer(MCInstance&);

/* @brief Triggers update of the onboard GPS
*/
void update_GPS(MCInstance&);
		
/* @brief Triggers reading of both the battery sensors (via `battery`) and the reading of the compass
*/
void read_batt_compass(MCInstance&);

/* @brief Read the receiver RSSI as an 8 bit number */
void read_receiver_rssi(MCInstance&);

