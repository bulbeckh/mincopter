# breaks.gdb

layout src
set print pretty on

#break AP_InertialNav::correct_with_gps
#break AP_InertialNav::check_gps
#break AP_InertialNav::check_baro
#break AP_InertialNav::update
#break WP_Planner::run
#break WP_Planner::update_nav_mode
#break AC_WPNav::update_wpnav
break PID_Controller::run_rate_controllers
run > /dev/null 2>&1
c 100


