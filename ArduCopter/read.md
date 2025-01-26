functions

## attitude.h
- `void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)`
- `int16_t get_pilot_desired_throttle(int16_t throttle_control)`
- `void get_stabilize_roll(int32_t target_angle)`
- `void get_stabilize_pitch(int32_t target_angle)`
- `void get_stabilize_yaw(int32_t target_angle)`
- `int16_t get_rate_roll(int32_t target_rate)`
- `int16_t get_rate_pitch(int32_t target_rate)`
- `int16_t get_rate_yaw(int32_t target_rate)`
- `void get_throttle_rate(float z_target_speed)`
- `void set_throttle_out(int16_t throttle_out, bool apply_angle_boost)`
- `int16_t get_throttle_accel(int16_t z_target_accel)`
- `void get_look_at_yaw()`
- `void get_look_ahead_yaw(int16_t pilot_yaw)`
- `void update_throttle_cruise(int16_t throttle)`
- `int16_t get_angle_boost(int16_t throttle)`
- `void set_throttle_accel_target( int16_t desired_acceleration )`
- `void throttle_accel_deactivate()`
- `int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)`
- `void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)`
- `void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)`
- `void get_throttle_rate_stabilized(int16_t target_rate)`
- `void get_throttle_land()`
- `bool update_land_detector()`
- `void reset_land_detector()`
- `void reset_I_all(void)`
- `void reset_rate_I()`
- `void reset_throttle_I(void)`
- `void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in)`
- `void update_rate_controller_targets()`
- `void run_rate_controllers()`
- `void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)`
- `void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )`
- `void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )`
- `void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )`

## motors.h
- `void arm_motors_check()`
- `void init_arm_motors()`
- `void init_disarm_motors()`
- `void auto_disarm_check()`
- `void pre_arm_checks(bool display_failure)`
- `void pre_arm_rc_checks()`
- `bool pre_arm_gps_checks(bool display_failure)`
- `bool arm_checks(bool display_failure)`
- `void servo_write(uint8_t ch, uint16_t pwm)`

## navigation.h
- `void run_nav_updates(void)`
- `void run_autopilot()`
- `void calc_position()`
- `void calc_distance_and_bearing()`
- `bool set_nav_mode(uint8_t new_nav_mode)`
- `void update_nav_mode()`
- `void reset_nav_params(void)`
- `int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)`

## control_modes.h
- `bool set_throttle_mode( uint8_t new_throttle_mode )`
- `void update_throttle_mode(void)`
- `bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)`
- `void update_roll_pitch_mode(void)`
- `bool set_yaw_mode(uint8_t new_yaw_mode)`
- `void update_yaw_mode(void)`
- `void save_trim()`
- `void auto_trim()`

## failsafe.h
- `void failsafe_enable()`
- `void failsafe_disable()`
- `void failsafe_check()`
- `void failsafe_radio_on_event()`
- `void failsafe_radio_off_event()`
- `void failsafe_battery_event(void)`
- `void failsafe_gps_off_event(void)`

## fence.h (to be merged)
- `void fence_check()`

## radio.h (to be removed)
- `void default_dead_zones()`
- `void init_esc()`
- `void init_rc_in()`
- `void init_rc_out()`
- `void output_min()`
- `void set_throttle_and_failsafe(uint16_t throttle_pwm)`
- `void aux_servos_update_fn()`

## system.h
- `uint16_t board_voltage(void)`
- `void init_ardupilot()`
- `bool set_mode(uint8_t mode)`
- `void startup_ground(bool force_gyro_cal)`
- `bool GPS_ok()`
- `void update_auto_armed()`
- `uint32_t map_baudrate(int8_t rate, uint32_t default_baud)`
- `void check_usb_mux(void)`
- `void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)`

## util.h
- `void init_barometer(bool full_calibration)`
- `int32_t read_barometer(void)`
- `void init_compass()`
- `void read_receiver_rssi(void)`
- `void update_super_simple_bearing(bool force_update)`
- `void set_auto_armed(bool b)`
- `void set_home_is_set(bool b)`
- `void set_simple_mode(uint8_t b)`
- `void set_failsafe_radio(bool b)`
- `void set_failsafe_battery(bool b)`
- `void set_failsafe_gps(bool b)`
- `void set_takeoff_complete(bool b)`
- `void set_land_complete(bool b)`
- `void set_pre_arm_check(bool b)`
- `void set_pre_arm_rc_check(bool b)`
- `void delay(uint32_t ms)`
- `uint32_t millis()`
- `uint32_t micros()`
- `void pinMode(uint8_t pin, uint8_t output)`
- `void digitalWrite(uint8_t pin, uint8_t out)`
- `uint8_t digitalRead(uint8_t pin)`
- `void crash_check()`
- `void read_inertia()`
- `void read_inertial_altitude()`
- `void update_notify()`
- `uint16_t perf_info_loop_count`
- `uint32_t perf_info_max_time`
- `uint16_t perf_info_long_running`
- `void perf_info_reset()`
- `void perf_info_check_loop_time(uint32_t time_in_micros)`
- `uint16_t perf_info_get_num_loops()`
- `uint32_t perf_info_get_max_time()`
- `uint16_t perf_info_get_num_long_running()`
- `Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt)`
- `Vector3f pv_location_to_vector(Location loc)`
- `int32_t pv_get_lat(const Vector3f pos_vec)`
- `int32_t pv_get_lon(const Vector3f &pos_vec)`
- `float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)`
- `float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)`
- `void init_home()`





