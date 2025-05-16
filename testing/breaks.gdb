# breaks.gdb

layout src
break AP_AHRS_DCM::matrix_update
run > /dev/null 2>&1
c 500
