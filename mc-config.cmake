
## File where we specify the (non-HAL/architecture) configuration parameters and
## wiring, including things like the the choice of planner/controller and
## sensor backends.

#set(MC_CONTROLLER_TYPE "PID" CACHE STRING "Controller type for mincopter (MPC, PID, NONE)")
#set(MC_PLANNER_TYPE "NONE" CACHE STRING "Planner type for mincopter (WAYPOINT, NONE)")

## Specify simulation here
set(MC_SIM_TYPE "NONE") ## (SIM, NONE)
set(MC_SIM_ENV "GAZEBO") ## (GAZEBO)

#[[ Level of Simulation
- 0 device peripherals (UART, I2C, ..) are simulated using renode or simuAVR. Used to test device functionality
- 1 peripherals are bypassed but gazebo emulates sensor readings (i.e. IMU gyros) with optional noise. Used to test state functionality
- 2 no sensor readings or state estimation. state (position, velocity) is passed directly from gazebo. Used to test control/planning functionality
]]
set(MC_SIM_LEVEL "2") ## 

## Specify control/planning here
set(MC_CONTROLLER_TYPE "PID")
set(MC_PLANNER_TYPE "NONE") ## (NONE, WAYPOINT)

## Specify state here
## TODO Change these names
set(MC_INAV_TYPE "NONE") ## (DEFAULT, EKF, NONE)
set(MC_AHRS_TYPE "NONE") ## (DCM, EKF, NONE)

## Specify backend dev (ignored for sim)
## TODO Get ride of this
set(MC_GPS_TYPE "UBLOX") ## (NONE, UBLOX)
set(MC_IMU_TYPE "MPU6050")
set(MC_COMP_TYPE "ICM20948") ## (ICM20948, HMC5843)
set(MC_BARO_TYPE "BME280")
set(MC_STORAGE_TYPE "DATAFLASH") ## (FILE, DATAFLASH)
set(MC_ADC_TYPE "NONE") ## ('SIM', 'ADS7844')

## Specify wiring for each peripheral (ignored for sim)
# TODO


## Other flags
set(MC_SIMLOG "NONE") ## (TRUE, NONE)


