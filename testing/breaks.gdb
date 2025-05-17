# breaks.gdb

layout src
#break AP_InertialNav::correct_with_gps
break AP_InertialNav::check_gps
#break AP_InertialNav::check_baro
#break AP_InertialNav::update
#break WP_Planner::run
#break PID_Controller::run
run > /dev/null 2>&1
c 100


