// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "util.h"

#include "mcinstance.h"
#include "mcstate.h"
#include "config.h"

#include "AP_Math.h"
#include "log.h"

// To enable logging over named pipe
#ifdef TARGET_ARCH_LINUX
	#include <cstring>
#endif

extern MCInstance mincopter;

#include "planner.h"
#include "control.h"

/* @brief Periodically log output to a mix of the console and external storage. In simulation, we log
 * directly to a file. Currently, this function only runs during simulation (i.e. TARGET_ARCH_LINUX)
 *
 * Logging output is as follows:
 *
 * --[LOOP, simtime=150000.00]--------------------------------------------
 *
 *  AHRS sensor readings | X    | Y    | Z    |
 *  gyro (rad/s)    | 1.02 | 0.23 | 4.23 |
 *  acc  (m/s2 )    | 9.03 | 1.23 | 0.23 |
 *  mag  (ut   )    | 34.53 | 20.02 | -19.23 |
 *
 *  state estimation (position/velocity)
 *  | x (m)  | y (m)  | z (m)  | dx (m/s) | dy (m/s) | dz (m/s) |
 *  | +40.02 | -20.02 | +30.50 |    +2.01 |     -3.02
 *
 * state estimation (attitude/angvel)
 * | roll   | pitch  | yaw    | droll  | dpitch | dyaw   |
 * | -0.153 | +1.594 | -0.123 | +2.234 | -0.012 | -0.024 |
 *
 * control
 * | F      | RollT   | PitchT   | YawT    |
 * | 20.42N | +1.23Nm | -0.02 Nm | +3.21Nm |
 *
 * pwm output
 * | m0   | m1   | m2   | m3   |
 * | 1000 | 1000 | 1000 | 1000 |
 *
 *
 */
void dump_state(uint32_t _counter) 
{
#ifdef TARGET_ARCH_LINUX
	// Sensor readings
	Vector3f _gyr_meas = mincopter.ins.get_gyro();
	Vector3f _acc_meas = mincopter.ins.get_accel();
	Vector3f _mag_meas = mincopter.compass.get_field();
	GPS::GPS_Status _status = mincopter.g_gps->status();

	// Estimated state
	Vector3f _temp_pos = mcstate.get_position();
	Vector3f _temp_vel = mcstate.get_velocity();
	Vector3f _temp_eul = mcstate.get_euler_angles();
	Vector3f _temp_e_rates = mcstate.get_euler_rates();

	// Actual state


	Quaternion& _temp_att = mcstate.data.attitude;

	float roll,pitch,yaw;
	_temp_att.to_euler(&roll, &pitch, &yaw);

	Matrix3f _temp_rot;
	_temp_att.rotation_matrix(_temp_rot);

	// Dump to pipe @100Hz
	uint8_t log_packet[52];

	uint32_t iterations = (uint32_t)mincopter.hal.sim->last_sensor_state.iterations;

	/* In place of an enum, we use the following type IDs for log messages
	 * 0x01 RPY (euler)
	 * 0x02 Position
	 * 0x03 Velocity
	 * 0x04 Euler Rates
	 *
	 * 0x05 Control Input
	 * 0x06 Motor velocities
	 *
	 * 0x07 Full sensor state (3x imu accel, 3x imu gyro, 3x compass)
	 * 0x08 Actual state (from gazebo)
	 *
	 * 0x09 GPS position and velocity
	 *
	 * 0x0A Barometer readings and inferred altitude
	 */

	// RPY
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &roll, 4);
	std::memcpy(log_packet+8, &pitch, 4);
	std::memcpy(log_packet+12, &yaw, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x01);

	// Position
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_temp_pos.x, 4);
	std::memcpy(log_packet+8, &_temp_pos.y, 4);
	std::memcpy(log_packet+12, &_temp_pos.z, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x02);

	// Velocity
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_temp_vel.x, 4);
	std::memcpy(log_packet+8, &_temp_vel.y, 4);
	std::memcpy(log_packet+12, &_temp_vel.z, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x03);

	// Euler rates
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_temp_e_rates.x, 4);
	std::memcpy(log_packet+8, &_temp_e_rates.y, 4);
	std::memcpy(log_packet+12, &_temp_e_rates.z, 4);
	mincopter.hal.sim->log_state(log_packet, 16, 0x04);

	// Control Input (Force,Torque)
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &mincopter.hal.sim->control_input[0], 4);
	std::memcpy(log_packet+8, &mincopter.hal.sim->control_input[1], 4);
	std::memcpy(log_packet+12, &mincopter.hal.sim->control_input[2], 4);
	std::memcpy(log_packet+16, &mincopter.hal.sim->control_input[3], 4);
	mincopter.hal.sim->log_state(log_packet, 20, 0x05);

	// Motor Velocities PWM
	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &mincopter.hal.sim->motor_out[0], 2);
	std::memcpy(log_packet+6, &mincopter.hal.sim->motor_out[1], 2);
	std::memcpy(log_packet+8, &mincopter.hal.sim->motor_out[2], 2);
	std::memcpy(log_packet+10, &mincopter.hal.sim->motor_out[3], 2);
	mincopter.hal.sim->log_state(log_packet, 12, 0x06);

	// NOTE Downcast to float
	float imu_a_x = (float)mincopter.hal.sim->last_sensor_state.imu_accel_x;
	float imu_a_y = (float)mincopter.hal.sim->last_sensor_state.imu_accel_y;
	float imu_a_z = (float)mincopter.hal.sim->last_sensor_state.imu_accel_z;

	float imu_g_x = (float)mincopter.hal.sim->last_sensor_state.imu_gyro_x;
	float imu_g_y = (float)mincopter.hal.sim->last_sensor_state.imu_gyro_y;
	float imu_g_z = (float)mincopter.hal.sim->last_sensor_state.imu_gyro_z;

	float comp_x = (float)mincopter.hal.sim->last_sensor_state.field_x;
	float comp_y = (float)mincopter.hal.sim->last_sensor_state.field_y;
	float comp_z = (float)mincopter.hal.sim->last_sensor_state.field_z;

	// TODO Change this to the sensor variables and not the simulation variables
	// Sensor state ( (3+3+3)*4)
	std::memcpy(log_packet, &iterations, 4);

	std::memcpy(log_packet+4, &imu_a_x, 4);
	std::memcpy(log_packet+8, &imu_a_y, 4);
	std::memcpy(log_packet+12, &imu_a_z, 4);

	std::memcpy(log_packet+16, &imu_g_x, 4);
	std::memcpy(log_packet+20, &imu_g_y, 4);
	std::memcpy(log_packet+24, &imu_g_z, 4);

	std::memcpy(log_packet+28, &comp_x, 4);
	std::memcpy(log_packet+32, &comp_y, 4);
	std::memcpy(log_packet+36, &comp_z, 4);

	mincopter.hal.sim->log_state(log_packet, 40, 0x07);

	float pos_x_actual = (float)mincopter.hal.sim->last_sensor_state.pos_x;
	float pos_y_actual = (float)mincopter.hal.sim->last_sensor_state.pos_y;
	float pos_z_actual = (float)mincopter.hal.sim->last_sensor_state.pos_z;

	float vel_x_actual = (float)mincopter.hal.sim->last_sensor_state.vel_x;
	float vel_y_actual = (float)mincopter.hal.sim->last_sensor_state.vel_y;
	float vel_z_actual = (float)mincopter.hal.sim->last_sensor_state.vel_z;

	float euler_x_actual = (float)mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_x;
	float euler_y_actual = (float)mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_y;
	float euler_z_actual = (float)mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_z;

	float euler_rate_x = (float)mincopter.hal.sim->last_sensor_state.euler_rate_x;
	float euler_rate_y = (float)mincopter.hal.sim->last_sensor_state.euler_rate_y;
	float euler_rate_z = (float)mincopter.hal.sim->last_sensor_state.euler_rate_z;

	// TODO Repeat for actual angular rates
	
	std::memcpy(log_packet, &iterations, 4);

	std::memcpy(log_packet+4, &pos_x_actual, 4);
	std::memcpy(log_packet+8, &pos_y_actual, 4);
	std::memcpy(log_packet+12, &pos_z_actual, 4);

	std::memcpy(log_packet+16, &vel_x_actual, 4);
	std::memcpy(log_packet+20, &vel_y_actual, 4);
	std::memcpy(log_packet+24, &vel_z_actual, 4);

	std::memcpy(log_packet+28, &euler_x_actual, 4);
	std::memcpy(log_packet+32, &euler_y_actual, 4);
	std::memcpy(log_packet+36, &euler_z_actual, 4);

	std::memcpy(log_packet+40, &euler_rate_x, 4);
	std::memcpy(log_packet+44, &euler_rate_y, 4);
	std::memcpy(log_packet+48, &euler_rate_z, 4);

	mincopter.hal.sim->log_state(log_packet, 52, 0x08);

	// GPS
	std::memcpy(log_packet, &iterations, 4);

	std::memcpy(log_packet+4, &mincopter.g_gps->latitude, 4);
	std::memcpy(log_packet+8, &mincopter.g_gps->longitude, 4);
	std::memcpy(log_packet+12, &mincopter.g_gps->altitude_cm, 4);

	Vector3f gps_vel_vector = mincopter.g_gps->velocity_vector();

	std::memcpy(log_packet+16, &gps_vel_vector.x, 4);
	std::memcpy(log_packet+20, &gps_vel_vector.y, 4);
	std::memcpy(log_packet+24, &gps_vel_vector.z, 4);

	mincopter.hal.sim->log_state(log_packet, 28, 0x09);

	// Barometer pressure/temperature and inferred altitude
	float _pres = mincopter.barometer.get_pressure();
	float _temperature = mincopter.barometer.get_temperature();
	float alt_inferred = mincopter.barometer.get_altitude();

	std::memcpy(log_packet, &iterations, 4);
	std::memcpy(log_packet+4, &_pres, 4);
	std::memcpy(log_packet+8, &_temperature, 4);
	std::memcpy(log_packet+12, &alt_inferred, 4);

	mincopter.hal.sim->log_state(log_packet, 16, 0x10);


	// Update position directly as a test every second
	/*
	if (_counter%100==0 && _counter<500) {
		mincopter.hal.sim->set_mincopter_position(0,0,5);
		mincopter.hal.sim->set_mincopter_linvelocity(0,0,0);
		mincopter.hal.sim->set_mincopter_attitude(0,0,0);
		mincopter.hal.sim->set_mincopter_angvelocity(0,0,0);
	}

	if (_counter%500==0) {
		// Call for simulation reset after 5 seconds
		mincopter.hal.sim->reset();
	}
	*/

	// In linux/generic (simulation) targets we dump all relevant information at 1Hz
	
	// Dump to console @1Hz
	/*
	if (_counter%100==0) {
		mincopter.hal.console->printf("---[LOOP, simtime=%f, iterations=%u]----------------------\r\n", mincopter.hal.sim->last_sensor_state.timestamp, mincopter.hal.sim->last_sensor_state.iterations);

		mincopter.hal.console->printf("AHRS sensor readings | X      | Y      | Z      |\r\n");
		mincopter.hal.console->printf("    gyro (rad/s)     | %+6.2f | %+6.2f | %+6.2f |\r\n", _gyr_meas.x, _gyr_meas.y, _gyr_meas.z);
		mincopter.hal.console->printf("    acc  (m/2  )     | %+6.2f | %+6.2f | %+6.2f |\r\n", _acc_meas.x, _acc_meas.y, _acc_meas.z);
		mincopter.hal.console->printf("    mag  (ut   )     | %+6.2f | %+6.2f | %+6.2f |\r\n\n", _mag_meas.x, _mag_meas.y, _mag_meas.z);

		//mincopter.hal.console->printf("mag : % 6.2f, % 6.2f, % 6.2f\n"), _mag_meas.x, _mag_meas.y, _mag_meas.z);
		//mincopter.hal.console->printf("baro: % 6.2f, % 6.2f\n"), _pres, _temperature);
		//mincopter.hal.console->printf("gpss: %d\n"), _status);
		//mincopter.hal.console->printf("gll : %d, %d\n"), mincopter.g_gps->latitude, mincopter.g_gps->longitude);
		//mincopter.hal.console->printf("galt: %d\n"), mincopter.g_gps->altitude_cm);
		
		mincopter.hal.console->printf("state estimation (position/velocity)\r\n");
		mincopter.hal.console->printf("| x (m)  | y (m)  | z (m)  | dx (m/s) | dy (m/s) | dz (m/s) |\r\n");
		mincopter.hal.console->printf("| %+6.2f | %+6.2f | %+6.2f | %+8.2f | %+8.2f | %+8.2f |\r\n",
				_temp_pos.x, _temp_pos.y, _temp_pos.z,
				_temp_vel.x, _temp_vel.y, _temp_vel.z);

		mincopter.hal.console->printf("state estimation (attitude)\r\n");
		mincopter.hal.console->printf("| roll   | pitch  | yaw    | droll  | dpitch | dyaw  |\r\n");
		mincopter.hal.console->printf("| %+6.2f | %+6.2f | %+6.2f |    - |      - |     - |\r\n",
				roll, pitch, yaw);

		//mincopter.hal.console->printf("att q1,q2,q3,q4: %f, %f, %f, %f\n"), _temp_att[0], _temp_att[1], _temp_att[2], _temp_att[3]);
		//mincopter.hal.console->printf("eul r,p,y      : %f, %f, %f\n"), roll, pitch, yaw);
		//mincopter.hal.console->printf("homelng/lat/alt: %d, %d, %d\n"), mcstate.home.lat, mcstate.home.lng, mcstate.home.alt);

		mincopter.hal.console->printf("control\r\n");
		mincopter.hal.console->printf("| F       | RollT   | PitchT  | YawT    |\r\n");
		mincopter.hal.console->printf("| %+5.2fN | %+5.2fNm | %+5.2fNm | %+5.2fNm |\r\n\n",
				mincopter.hal.sim->control_input[0],
				mincopter.hal.sim->control_input[1],
				mincopter.hal.sim->control_input[2],
				mincopter.hal.sim->control_input[3]
				);

		mincopter.hal.console->printf("pwm output\r\n");
		mincopter.hal.console->printf("| m0   | m1   | m2   | m3   |\r\n");
		mincopter.hal.console->printf("| %4d | %4d | %4d | %4d |\r\n\n",
				mincopter.hal.sim->motor_out[0],
				mincopter.hal.sim->motor_out[1],
				mincopter.hal.sim->motor_out[2],
				mincopter.hal.sim->motor_out[3]
				);

	}
	*/
#endif




};

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

void crash_checks(void)
{
	// TODO Re-write. For now, just checks if we are upside down or >60deg tilt
	
	Vector3f eul = mcstate.get_euler_angles();
	
	// Check if we have roll/tilt greater than 60 degrees
	if (fabs(eul.x) >= 1.05f || fabs(eul.y) >= 1.05f) {
		mincopter.hal.console->printf("Crash flagged during crash check - tilt>=60degc\r\n");
		
		// Disarm
		planner.ap.arm_active = 0;
	}

	/*
    static uint8_t inverted_count;  // number of iterations we have been inverted
    static int32_t baro_alt_prev;

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!mincopter.motors.armed() || (mincopter.rc_3.control_in != 0)) {
        inverted_count = 0;
        return;
    }

    // check angles
    int32_t lean_max = planner.angle_max + CRASH_CHECK_ANGLE_DEVIATION_CD;
    if (labs(mcstate.roll_sensor) > lean_max || labs(mcstate.pitch_sensor) > lean_max) {
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
	*/

	return;
}


// position_vector.pde

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// TODO We keep this function here for now for reference as we will need to implement distance conversions later

/* 
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
*/

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

void init_home(void)
{
	// TODO Change this to update the **flight_state** parameter directly
    //set_home_is_set(true);
    mcstate.home.id         = 0; //previously MAV_CMD_NAV_WAYPOINT
    mcstate.home.lng        = mincopter.g_gps->longitude;                                 // Lon * 10**7
    mcstate.home.lat        = mincopter.g_gps->latitude;                                  // Lat * 10**7
    mcstate.home.alt        = mincopter.g_gps->altitude_cm;                                                        // Home is always 0

	// Set the 'home_set' flag
	// TODO This flag should be cleared when we land/disarm
	mcstate.home_set = true;

	mincopter.hal.console->printf("init home\r\n");

	// TODO We should read ground pressure and ground temperature here. They should be stored as part of the 'home' reading

	// TODO This currently is not implemented but should really have the same functionality as the above code. Consider merging all
    mcstate.set_home_position(mincopter.g_gps->longitude, mincopter.g_gps->latitude);

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    //planner.scaleLongDown = longitude_scale(mcstate.home);
    //planner.scaleLongUp   = 1.0f/planner.scaleLongDown;
	
	return;
}


// returns true if the GPS is ok and home position is set
bool GPS_ok(void)
{
    if (mincopter.g_gps != NULL
			&& planner.ap.home_is_set
			&& mincopter.g_gps->status() == GPS::GPS_OK_FIX_3D
			&& !mincopter.gps_glitch.glitching()) {
        return true;
    }else{
        return false;
    }
}

