// For printing serial information to console

/* serial.h
* This is a library for Serial Communication, either via uartA (hal.console or the cliConsole alias), or uartB (which
* is now the telemetry interface).
*
* For logging to DataFlash, use the interface in log.h
*/

/* @brief Runs at 1Hz. Dumps information to the serial
* output depending on which flags are on
*/
void dump_serial();

/* @brief Prints information about current location
*/
void print_GPS();

/* @brief Prints the current roll/pitch/yaw values
*/
void print_RPY();

/* @brief Prints the current imu roll and accel rates
*/
void print_roll_rates_and_accel();

/*

We want to be able to log the following sensor measurements and state variables

Raw measurements and outputs
1. Compass (Magnetomer)
2. IMU (note: technically the roll rates are state variables and not raw measurements)
3. GPS (skip)
4. Barometer
5. Battery Voltage/Current (skip)
6. RC Out (Motor voltages)

State Variables
1. Location and attitude (from AHRS, DCM, and InertialNav)
2. Flight states and modes
3. PID Values and targets (control_roll, control_pitch) and feedforward values
4. EKF Gains and states
5. Performance and Timing

************************************************

1. Compass (Magnetometer)
  Mag offsets 			: Vector3f off = compass.get_offsets(0)
  Mag motor offsets : Vector3f m_off = compass.get_motor_offsets(0)
	Mag 							: Vector3f mg = compass.get_field(0);
	Declination 			: float dec = compass.get_declination();

2. IMU
	Roll rates (rad/s)    : Vector3f omega = ins.get_gyros()
	Gyro offsets (rad/s)  : Vector3f off = ins.get_gyro_offsets()
	Acceleration (m/s/s)  : Vector3f acc = ins.get_accel()
	Accel offsets (m/s/s) : Vector3f a_off = ins.get_accel_offsets()
	Accel scale 					: Vector3f a_scale = ins.get_accel_scale()

3. GPS
	GPS Status (0-NoGPS,1-NoFIX,2-2DFix,3-3DFix)	: GPS_Status	: status()
	Latitude (deg*1e7) 	: int32_t 		: latitude
	Longitude (deg*1e7) : int32_t 		: longitude
	Altitude (cm) 			: int32_t			: altitude
	Velocity 						: Vector3f		: velocity_vector()
	Ground Speed 				: float 			: last_ground_speed()

4. Barometer
	Pressure : float : get_pressure()
	Altitude (relative, in m) : float : get_altitude()
	Climb Rate (m/s) : float : get_climb_rate()

5. Current/Voltage
  battery_voltage     				: int16_t : (battery.voltage() * 100.0f),
  current_amps        				: int16_t :(battery.current_amps() * 100.0f),
  current draw since startup	: float : battery.current_total_mah()

6. RC Out

*** State Variables ************************************************
Most of these are stored in the MCState mcstate instance.

1. Location and attitude
	Location where copter was armed : Location : home
	Current location								: Location : current_loc

2. Orientation
	DCM Values : float : mcstate.cos_roll_x
	DCM Valeus : float : mcstate.cos_pitch_x
	DCM Values : float : mcstate.cos_yaw
	DCM Values : float : mcstate.sin_yaw
	DCM Values : float : mcstate.sin_roll
	DCM Values : float : mcstate.sin_pitch

3. PID Values

    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;

    AC_PID                  pid_throttle_rate;
    AC_PID                  pid_throttle_accel;

    APM_PI                  pi_stabilize_roll;
    APM_PI                  pi_stabilize_pitch;
    APM_PI                  pi_stabilize_yaw;

4. Other Copter State Variables
	e.g. landing indicator

*/

