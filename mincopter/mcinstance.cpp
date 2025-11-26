
// TODO This should really be renamed to something like sensor_updates.cpp

#include "mcinstance.h"
#include "mcstate.h"

/* NOTE The reason why there is a mincopter instance here is because these are not class methods */
extern MCInstance mincopter;

#include "planner.h"
#include "control.h"

#include "defines.h"
#include "util.h"
#include "log.h"

void accumulate_compass(void)
{
	// Accumulate compass readings
	mincopter.compass.accumulate();

	return;
}

void accumulate_barometer(void)
{
	// Accumulate barometer readings
	mincopter.barometer.accumulate();

	return;
}

void read_barometer(void)
{
	// Update barometer
	mincopter.barometer.read();

	return;
}

void read_batt_compass(void)
{
	// Update battery monitor
    mincopter.battery.read();

	// If we are monitoring current, then update the compass to correct for declination
    if (mincopter.battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        mincopter.compass.set_current(mincopter.battery.current_amps());
    }
	
	// Update compass
	mincopter.compass.read()

	// Log compass information
	if (mincopter.log_bitmask & MASK_LOG_COMPASS) Log_Write_Compass();

	return;
}

// called at 50hz
void update_GPS(void)
{
	static uint32_t last_gps_reading;           // time of last gps message
	static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location

	mincopter.g_gps->update();

		// logging and glitch protection run after every gps message
		if (mincopter.g_gps->last_message_time_ms() != last_gps_reading) {
			last_gps_reading = mincopter.g_gps->last_message_time_ms();

			// log GPS message
			if (mincopter.log_bitmask & MASK_LOG_GPS) {
					mincopter.DataFlash.Log_Write_GPS(mincopter.g_gps, mcstate.current_loc.alt);
			}

			// run glitch protection and update AP_Notify if home has been initialised
			if (planner.ap.home_is_set) {
					mincopter.gps_glitch.check_position();
			}
		}

		// checks to initialise home and take location based pictures
		if (mincopter.g_gps->new_data && mincopter.g_gps->status() >= GPS::GPS_OK_FIX_3D) {
				// clear new data flag
				mincopter.g_gps->new_data = false;

				// check if we can initialise home yet
				if (!planner.ap.home_is_set) {
						// if we have a 3d lock and valid location
						if(mincopter.g_gps->status() >= GPS::GPS_OK_FIX_3D && mincopter.g_gps->latitude != 0) {
								if( ground_start_count > 0 ) {
										ground_start_count--;
								}else{
										// after 10 successful reads store home location
										// ap.home_is_set will be true so mincopter will only happen once
										ground_start_count = 0;
										
										// TODO Move mincopter to btree as it initialises the start location on GPS lock
										init_home();

										// set system clock for log timestamps
										mincopter.hal.util->set_system_clock(mincopter.g_gps->time_epoch_usec());

										// Set compass declination automatically
										mincopter.compass.set_initial_location(mincopter.g_gps->latitude, mincopter.g_gps->longitude);
								}
						} else {
								// start again if we lose 3d lock
								ground_start_count = 10;
						}
				}
		}
}
