if(MC_EXPERIMENTAL_BUILD AND (MC_IMU_TYPE STREQUAL "MPU6050" OR MC_IMU_TYPE STREQUAL "BMI270"))
    add_executable(test-imu-task
        test_imu_task.cpp
        ${CMAKE_SOURCE_DIR}/dev/experimental/led/LedDevice.cpp
        ${CMAKE_SOURCE_DIR}/experimental/mincopter/RuntimeConfig.cpp
    )

    target_include_directories(test-imu-task PUBLIC
        ${MC_INCLUDE_DIRS}
    )

    target_link_libraries(test-imu-task
        mc-experimental-arch
        mc-experimental-dev
    )

    if(TARGET_ARCH_FAMILY STREQUAL "stm32")
        target_link_libraries(test-imu-task
            -Wl,--whole-archive
            stm32_platform
            -Wl,--no-whole-archive
        )
    endif()
endif()
