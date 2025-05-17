# breaks.gdb

layout src
#break AP_InertialNav::correct_with_gps
break AP_InertialNav::check_baro
run > /dev/null 2>&1
c 500
