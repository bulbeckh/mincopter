
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
set(MC_CONTROLLER_TYPE "NONE")
set(MC_PLANNER_TYPE "NONE") ## (NONE, WAYPOINT)

## Specify state here
## TODO Change these names
set(MC_INAV_TYPE "NONE") ## (DEFAULT, EKF, NONE)
set(MC_AHRS_TYPE "NONE") ## (DCM, EKF, NONE, COMPLEMENTARY)

## Specify backend dev (ignored for sim)
set(MC_GPS_TYPE "NONE") 			## (NONE, UBLOX, AUTO, SIM)
set(MC_IMU_TYPE "NONE") 			## (NONE, MPU6050, MPU6000, ICM20948, L3G4200D, SIM)
set(MC_COMP_TYPE "NONE") 			## (NONE, ICM20948, HMC5843, SIM)
set(MC_BARO_TYPE "NONE")  			## (NONE, BME280, MS5611, BMP085, SIM)
set(MC_STORAGE_TYPE "NONE") 		## (NONE, FILE, DATAFLASH)
set(MC_ADC_TYPE "NONE") 			## (NONE, ADS7844, SIM)

## Specify wiring for each peripheral (ignored for sim)
# TODO


## Other flags
set(MC_SIMLOG "NONE") ## (TRUE, NONE)


