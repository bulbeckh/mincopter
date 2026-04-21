#
# Example feature-set definition for the experimental RTOS runtime.
#
# This file answers:
# - which device drivers should be compiled
# - which state/planner/controller modules should be used
# - which optional runtime features are enabled
#
# It does not answer:
# - which pins those devices use
# - which UART/SPI/I2C instance a board routes them to
# - which MCU is being targeted
#

set(MC_FEATURE_SET_NAME "experimental_sensor_stack")

## Simulation
set(MC_SIM_TYPE "NONE")
set(MC_SIM_ENV "GAZEBO")
set(MC_SIM_LEVEL "0")

## Flight stack selection
set(MC_CONTROLLER_TYPE "NONE")
set(MC_PLANNER_TYPE "NONE")
set(MC_STATE_TYPE "NONE")

## Experimental device selection
set(MC_IMU_TYPE "BMI270")
set(MC_COMP_TYPE "ICM20948")
set(MC_BARO_TYPE "BME280")
set(MC_GPS_TYPE "UBLOX")
set(MC_STORAGE_TYPE "DATAFLASH")
set(MC_ADC_TYPE "NONE")

## Experimental runtime/backend selection
set(MC_USE_EXPERIMENTAL_RTOS_HAL "ON")
set(MC_USE_EXPERIMENTAL_MINCOPTER_RUNTIME "ON")

## Optional features
set(MC_SIMLOG "NONE")
set(MC_ENABLE_LOGGING "ON")
