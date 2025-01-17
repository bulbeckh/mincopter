
#include "mcinstance.h"



void MCInstance::read_compass(void) {
	if (g.compass_enabled) compass.accumulate();
}

// NOTE What is the difference between read_baro and barometer.accumulate?
void MCInstance::read_baro(void) {
	barometer.accumulate();
}

void MCInstance::update_altitude()
{
		// read in baro altitude
		baro_alt            = read_barometer();
}

// called at 50hz
void MCInstance::update_GPS(void)
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
void read_batt_compass(void)
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

// NOTE REMOVED three_hz_loop - 3.3hz loop

// one_hz_loop - runs at 1Hz
void one_hz_loop()
{
		// from serial.h
		//print_GPS();
		//print_RPY();
		//print_roll_rates_and_accel();

		// Print num logs
		/*
		uint16_t nl = DataFlash.get_num_logs();
		cliSerial->printf_P(PSTR("Num logs: %d\n"), nl);
		*/

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

		check_usb_mux();

}
