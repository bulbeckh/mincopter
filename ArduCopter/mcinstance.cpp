
// TODO This should really be renamed to something like sensor_updates.cpp

#include "mcinstance.h"
#include "mcstate.h"

/* NOTE The reason why there is a mincopter instance here is because these are not class methods */
extern MCInstance mincopter;
extern MCState mcstate;

#include "planner.h"
#include "planner_waypoint.h"
extern WP_Planner planner;

#include "control.h"
#include "controller_pid.h"
extern PID_Controller controller;

#ifdef TARGET_ARCH_LINUX
	#include <iostream>
	#include "simulation_logger.h"
	extern SimulationLogger simlog;
#endif

#include "defines.h"
#include "util.h"
#include "log.h"
#include "init.h"

void read_compass(void) {
	mincopter.compass.accumulate();
}

// NOTE What is the difference between read_baro and barometer.accumulate?
void read_baro(void) {
	mincopter.barometer.accumulate();
}

void update_altitude()
{
		// read in baro altitude
		mincopter.barometer.read();
		planner.baro_alt = mincopter.barometer.get_altitude() * 100.f;
}

// called at 50hz
void update_GPS(void)
{
		static uint32_t last_gps_reading;           // time of last gps message
		static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location
		bool report_gps_glitch;

		mincopter.g_gps->update();

#ifdef TARGET_ARCH_LINUX
		Vector3f vgps = mincopter.g_gps->velocity_vector();
		simlog.write_gps_state(mincopter.g_gps->latitude,
				mincopter.g_gps->longitude,
				mincopter.g_gps->altitude_cm,
				vgps.x,
				vgps.y,
				vgps.z);
#endif

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
						report_gps_glitch = (mincopter.gps_glitch.glitching() && !planner.ap.usb_connected);
						if (AP_Notify::flags.gps_glitching != report_gps_glitch) {
								if (mincopter.gps_glitch.glitching()) {
										Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_GPS_GLITCH);
								}else{
										Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_ERROR_RESOLVED);
								}
								AP_Notify::flags.gps_glitching = report_gps_glitch;
						}
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

// update_batt_compass - read battery and compass
// should be called at 10hz
void read_batt_compass(void)
{
		// read battery before compass because it may be used for motor interference compensation
    mincopter.battery.read();

    // update compass with current value
    if (mincopter.battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        mincopter.compass.set_current(mincopter.battery.current_amps());
    }

		// TODO Move these checks to ap_state as the failsafe lives there too

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
		/*
    if (!mincopter.ap.usb_connected && !planner.failsafe.battery && mincopter.battery.exhausted(mincopter.fs_batt_voltage, mincopter.fs_batt_mah)) {
        failsafe_battery_event();
    }
		*/

#if HIL_MODE != HIL_MODE_ATTITUDE  // don't execute in HIL mode
		if (mincopter.compass.read()) {
				mincopter.compass.null_offsets();
		}
		// log compass information
		if (mincopter.log_bitmask & MASK_LOG_COMPASS) {
				Log_Write_Compass();
		}
#endif

}

// NOTE REMOVED three_hz_loop - 3.3hz loop

// one_hz_loop - runs at 1Hz
void one_hz_loop()
{

#ifdef TARGET_ARCH_LINUX
		std::cout << "In one hz loop\n";
#endif

		// from serial.h
		//print_GPS();
		//print_RPY();
		//print_roll_rates_and_accel();

		// Print num logs
		/*
		uint16_t nl = DataFlash.get_num_logs();
		cliSerial->printf_P(PSTR("Num logs: %d\n"), nl);
		*/

		// TODO Move these to btree
		// pass latest alt hold kP value to navigation controller
		planner.wp_nav.set_althold_kP(controller.pi_alt_hold.kP());

		// update latest lean angle to navigation controller
		planner.wp_nav.set_lean_angle_max(planner.angle_max);

		// TODO Move arming to btree
		// perform pre-arm checks & display failures every 30 seconds
		static uint8_t pre_arm_display_counter = 15;
		pre_arm_display_counter++;
		/*
		if (pre_arm_display_counter >= 30) {
				pre_arm_checks(true);
				pre_arm_display_counter = 0;
		}else{
				pre_arm_checks(false);
		}
		*/

		// auto disarm checks
		//auto_disarm_check();

		/*
		if (!mincopter.motors.armed()) {
				// make it possible to change ahrs orientation at runtime during initial config
				mcstate.ahrs.set_orientation();

				// check the user hasn't updated the frame orientation
				mincopter.motors.set_frame_orientation(mincopter.frame_orientation);
		}
		*/

		check_usb_mux();

}
