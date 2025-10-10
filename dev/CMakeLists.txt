
#### Build - dev/

#file(GLOB_RECURSE MC_LIBS_DEV "${CMAKE_SOURCE_DIR}/dev/*.cpp")

## TODO For now we just compile all motor types
file(GLOB_RECURSE MC_DEV_MOTOR_SOURCES "${CMAKE_SOURCE_DIR}/dev/AP_Motors/*.cpp")

list(APPEND MC_LIBS_DEV
	${CMAKE_SOURCE_DIR}/dev/AP_Compass/Compass.cpp
	${CMAKE_SOURCE_DIR}/dev/AP_Baro/AP_Baro.cpp
	${CMAKE_SOURCE_DIR}/dev/AP_InertialSensor/AP_InertialSensor.cpp
	${CMAKE_SOURCE_DIR}/dev/AP_GPS/GPS.cpp
	${CMAKE_SOURCE_DIR}/dev/AP_GPS/AP_GPS_Glitch.cpp
	## No DataFlash common sources
	${CMAKE_SOURCE_DIR}/dev/AP_ADC/AP_ADC.cpp
	${CMAKE_SOURCE_DIR}/dev/AP_Declination/AP_Declination.cpp
	${CMAKE_SOURCE_DIR}/dev/AP_BattMonitor/AP_BattMonitor.cpp
	${CMAKE_SOURCE_DIR}/dev/DataFlash/DataFlash_Block.cpp
	${CMAKE_SOURCE_DIR}/dev/DataFlash/LogFile.cpp
	${MC_DEV_MOTOR_SOURCES}
)


## Compass
if(MC_COMP_TYPE STREQUAL "HMC5843")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_Compass/AP_Compass_HMC5843.cpp ) 
elseif(MC_COMP_TYPE STREQUAL "ICM20948")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_Compass/AP_Compass_ICM20948.cpp )
elseif(MC_COMP_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_Compass/sim_compass.cpp)
else()
	message(FATAL_ERROR "Compass device not supported.")
endif()

## Barometer
if(MC_BARO_TYPE STREQUAL "MS5611")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_Baro/AP_Baro_MS5611.cpp)
elseif(MC_BARO_TYPE STREQUAL "BME280")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_Baro/AP_Baro_BME280.cpp)
elseif(MC_BARO_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_Baro/sim_barometer.cpp)
	## TODO Add the other baro type here (BSPxx)
else()
	message(FATAL_ERROR "Barometer device not supported.")
endif()

## IMU
if(MC_IMU_TYPE STREQUAL "MPU6000")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_InertialSensor/AP_InertialSensor_MPU6000.cpp)
elseif(MC_IMU_TYPE STREQUAL "MPU6050")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_InertialSensor/AP_InertialSensor_MPU6050.cpp)
elseif(MC_IMU_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_InertialSensor/sim_inertialsensor.cpp)
	## TODO Add the other IMU
else()
	message(FATAL_ERROR "IMU device not supported.")
endif()

## GPS
if(MC_GPS_TYPE STREQUAL "AUTO")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_GPS/AP_GPS_AUTO.cpp)
elseif(MC_GPS_TYPE STREQUAL "UBLOX")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_GPS/AP_GPS_UBLOX.cpp)
elseif(MC_GPS_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_GPS/sim_gps.cpp)
	## TODO Add remaining GPS modules
else()
	message(FATAL_ERROR "GPS device not supported.")
endif()

## Storage
if(MC_STORAGE_TYPE STREQUAL "FILE")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/DataFlash/DataFlash_File.cpp)
elseif(MC_STORAGE_TYPE STREQUAL "DATAFLASH")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/DataFlash/DataFlash_APM2.cpp)
elseif(MC_STORAGE_TYPE STREQUAL "NONE")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/DataFlash/DataFlash_Empty.cpp)
	## TODO Add other dataflash types
else()
	message(FATAL_ERROR "Storage device not supported.")
endif()

## ADC
if(MC_ADC_TYPE STREQUAL "ADS7844")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_ADC/AP_ADC_ADS7844.cpp)
elseif(MC_ADC_TYPE STREQUAL "NONE")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_ADC/AP_ADC_None.cpp)
elseif(MC_ADC_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_DEV ${CMAKE_SOURCE_DIR}/dev/AP_ADC/sim_adc.cpp)
else()
	message(FATAL_ERROR "ADC device not supported.")
endif()

add_library(mc-dev STATIC ${MC_LIBS_DEV})

target_include_directories(mc-dev PUBLIC
	${MC_INCLUDE_DIRS}
)
