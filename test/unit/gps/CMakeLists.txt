if(MC_EXPERIMENTAL_BUILD AND MC_GPS_TYPE STREQUAL "UBLOX")
    add_executable(test-gps-task
        test_gps_task.cpp
        ${CMAKE_SOURCE_DIR}/dev/experimental/led/LedDevice.cpp
        ${CMAKE_SOURCE_DIR}/experimental/mincopter/RuntimeConfig.cpp
    )

    target_include_directories(test-gps-task PUBLIC
        ${MC_INCLUDE_DIRS}
    )

    target_link_libraries(test-gps-task
        mc-experimental-arch
        mc-experimental-dev
    )

    if(TARGET_ARCH_FAMILY STREQUAL "stm32")
        target_link_libraries(test-gps-task
            -Wl,--whole-archive
            stm32_platform
            -Wl,--no-whole-archive
        )
    endif()
endif()
