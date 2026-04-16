if(MC_EXPERIMENTAL_BUILD AND NOT MC_COMP_TYPE STREQUAL "NONE")
    add_executable(test-compass-task
        test_compass_task.cpp
        ${CMAKE_SOURCE_DIR}/dev/experimental/led/LedDevice.cpp
        ${CMAKE_SOURCE_DIR}/experimental/mincopter/RuntimeConfig.cpp
    )

    target_include_directories(test-compass-task PUBLIC
        ${MC_INCLUDE_DIRS}
    )

    target_link_libraries(test-compass-task
        mc-experimental-arch
        mc-experimental-dev
    )

    if(TARGET_ARCH_FAMILY STREQUAL "stm32")
        target_link_libraries(test-compass-task
            -Wl,--whole-archive
            stm32_platform
            -Wl,--no-whole-archive
        )
    endif()
endif()
