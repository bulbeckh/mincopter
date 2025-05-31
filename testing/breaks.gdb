# breaks.gdb

layout src
set print pretty on

#break AP_InertialNav::update
#break GZ_Interface::update_gps_position
#break AP_InertialNav::correct_with_baro
#break AP_InertialNav::check_gps
#break AP_InertialNav::check_baro
#break AP_InertialNav::update
#break AP_InertialNav::set_home_position
#break Compass::set_initial_location
#break update_GPS
break AP_AHRS_DCM::update
#break AP_InertialSensor_Sim::update
#break AP_AHRS_DCM::drift_correction
#break WP_Planner::run
#break WP_Planner::update_nav_mode
#break AC_WPNav::update_wpnav
#break PID_Controller::run_rate_controllers
#break AP_MotorsMatrix::output_armed
#break AP_Baro_Sim::read
#break AP_Baro::init
#break AP_Baro_Sim::init
#break AP_Baro::calibrate
#break AP_Baro_Sim::calibrate
#break GPS::update

## Barometer issue testing
#break AP_Baro.cpp:63
#break AP_Baro.cpp:78
#break AP_Baro.cpp:95
#break AP_Baro.cpp:112
#break AP_Baro.cpp:119

#break WP_Planner::run
#break PID_Controller::run
#break AP_Motors::output

#break GZ_Interface::send_control_output

run > /dev/null 2>&1

