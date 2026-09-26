#cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
#project(ardupilot_gazebo)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
#find_package(ament_cmake QUIET)
#if(${ament_cmake_FOUND})
#message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
#endif()

# --------------------------------------------------------------------------- #
# Compile as C++14.
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.

# Harmonic (default)
if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()


# --------------------------------------------------------------------------- #
# Build plugin.

add_library(ArduPilotPlugin
    SHARED
    src/ArduPilotPlugin.cc
    src/Util.cc
)

target_include_directories(ArduPilotPlugin PRIVATE
	${MC_INCLUDE_DIRS}
)
target_link_libraries(ArduPilotPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  mc-common
)

## Generate a configuration file that exports Gazebo env variables
configure_file(
	${CMAKE_SOURCE_DIR}/toolchains/sim/setup_gz.sh.in
	${CMAKE_BINARY_DIR}/setup_gz.sh
	@ONLY
)

# --------------------------------------------------------------------------- #
# Install.

#install(
#TARGETS
#ArduPilotPlugin
#DESTINATION lib/${PROJECT_NAME}
#)

#install(
#DIRECTORY
#models/
#DESTINATION share/${PROJECT_NAME}/models
#)

#install(
#  DIRECTORY
#  worlds/
#  DESTINATION share/${PROJECT_NAME}/worlds
#)

# --------------------------------------------------------------------------- #
# Register as an ament package if ament_cmake is available.
#if(${ament_cmake_FOUND})
#ament_environment_hooks(
#"${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
#ament_environment_hooks(
#"${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

#ament_package()
#endif()
