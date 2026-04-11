cmake_minimum_required(VERSION 3.13)

##################################################################
# TARGET / TOOLCHAIN SELECTION
##################################################################

function(set_toolchain TARGET_ARCH)
    message(STATUS "Using target architecture ${TARGET_ARCH}")

    if(TARGET_ARCH STREQUAL "atmega2560" OR TARGET_ARCH STREQUAL "atmega2561")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/avr-mega.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "avrmega" PARENT_SCOPE)
        set(BOARD_DEFINE "ATmega2560Family" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "atmega1280" OR TARGET_ARCH STREQUAL "atmega1281")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/avr-mega.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "avrmega" PARENT_SCOPE)
        set(BOARD_DEFINE "ATmega1280Family" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "avr64db28"
            OR TARGET_ARCH STREQUAL "avr64db32"
            OR TARGET_ARCH STREQUAL "avr64db48"
            OR TARGET_ARCH STREQUAL "avr64db64"
            OR TARGET_ARCH STREQUAL "avr128db28"
            OR TARGET_ARCH STREQUAL "avr128db32"
            OR TARGET_ARCH STREQUAL "avr128db48"
            OR TARGET_ARCH STREQUAL "avr128db64"
            OR TARGET_ARCH STREQUAL "avr128da28"
            OR TARGET_ARCH STREQUAL "avr128da32"
            OR TARGET_ARCH STREQUAL "avr128da48"
            OR TARGET_ARCH STREQUAL "avr128da64"
            OR TARGET_ARCH STREQUAL "avr64da28"
            OR TARGET_ARCH STREQUAL "avr64da32"
            OR TARGET_ARCH STREQUAL "avr64da48"
            OR TARGET_ARCH STREQUAL "avr64da64"
            OR TARGET_ARCH STREQUAL "avr64dd28"
            OR TARGET_ARCH STREQUAL "avr64dd32"
            OR TARGET_ARCH STREQUAL "avr64du28"
            OR TARGET_ARCH STREQUAL "avr64du32")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/avrXX.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "avrdx" PARENT_SCOPE)
        set(BOARD_DEFINE "${TARGET_ARCH}" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "stm32f407xx")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/stm32.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "stm32" PARENT_SCOPE)
        set(BOARD_DEFINE "STM32F407xx" PARENT_SCOPE)
        set(MCPU "cortex-m4" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "stm32f405xx")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/stm32.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "stm32" PARENT_SCOPE)
        set(BOARD_DEFINE "STM32F405xx" PARENT_SCOPE)
        set(MCPU "cortex-m4" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "stm32f401xe")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/stm32.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "stm32" PARENT_SCOPE)
        set(BOARD_DEFINE "STM32F401xE" PARENT_SCOPE)
        set(MCPU "cortex-m4" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "stm32f401xc")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/stm32.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "stm32" PARENT_SCOPE)
        set(BOARD_DEFINE "STM32F401xC" PARENT_SCOPE)
        set(MCPU "cortex-m4" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "generic")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/x86_64.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "generic" PARENT_SCOPE)
        set(BOARD_DEFINE "GENERIC" PARENT_SCOPE)
    elseif(TARGET_ARCH STREQUAL "rpi2modelb")
        set(TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchains/rpi.cmake" PARENT_SCOPE)
        set(TARGET_ARCH_FAMILY "rpi" PARENT_SCOPE)
        set(BOARD_DEFINE "RPI2MODEL_B" PARENT_SCOPE)
    else()
        message(FATAL_ERROR "Unsupported target architecture: ${TARGET_ARCH}")
    endif()
endfunction()

if(NOT DEFINED TARGET_ARCH)
    message(FATAL_ERROR "Target architecture not supplied. Use -DTARGET_ARCH=<arch>.")
endif()

if(NOT DEFINED MC_BOARD)
    message(FATAL_ERROR "Board configuration not supplied. Use -DMC_BOARD=<board-file-without-extension>.")
endif()

if(NOT DEFINED MC_FEATURE_SET)
    message(FATAL_ERROR "Feature set not supplied. Use -DMC_FEATURE_SET=<feature-file-without-extension>.")
endif()

set_toolchain(${TARGET_ARCH})
include(${TOOLCHAIN_FILE})
enable_language(C ASM)

project(mincopter)

##################################################################
# CONFIGURATION LOADING
##################################################################

set(MC_BOARD_FILE "${CMAKE_SOURCE_DIR}/config/boards/${MC_BOARD}.cmake")
set(MC_FEATURE_FILE "${CMAKE_SOURCE_DIR}/config/features/${MC_FEATURE_SET}.cmake")

if(NOT EXISTS "${MC_BOARD_FILE}")
    message(FATAL_ERROR "Board configuration file not found: ${MC_BOARD_FILE}")
endif()

if(NOT EXISTS "${MC_FEATURE_FILE}")
    message(FATAL_ERROR "Feature configuration file not found: ${MC_FEATURE_FILE}")
endif()

include("${MC_BOARD_FILE}")
include("${MC_FEATURE_FILE}")

if(NOT DEFINED MC_BOARD_MCU)
    message(FATAL_ERROR "Board file must define MC_BOARD_MCU")
endif()

if(NOT "${MC_BOARD_MCU}" STREQUAL "${BOARD_DEFINE}")
    message(FATAL_ERROR "Board MCU (${MC_BOARD_MCU}) does not match TARGET_ARCH MCU (${BOARD_DEFINE})")
endif()

if(NOT DEFINED MC_USE_EXPERIMENTAL_RTOS_HAL)
    set(MC_USE_EXPERIMENTAL_RTOS_HAL "OFF")
endif()

if(NOT DEFINED MC_USE_EXPERIMENTAL_MINCOPTER_RUNTIME)
    set(MC_USE_EXPERIMENTAL_MINCOPTER_RUNTIME "OFF")
endif()

set(MC_EXPERIMENTAL_BUILD OFF)
if(MC_USE_EXPERIMENTAL_RTOS_HAL AND MC_USE_EXPERIMENTAL_MINCOPTER_RUNTIME)
    set(MC_EXPERIMENTAL_BUILD ON)
endif()

##################################################################
# BASIC VALIDATION
##################################################################

if(MC_EXPERIMENTAL_BUILD)
    if(NOT TARGET_ARCH_FAMILY STREQUAL "stm32")
        message(FATAL_ERROR "The current experimental RTOS runtime is only wired for STM32 targets.")
    endif()

    if(MC_IMU_TYPE STREQUAL "MPU6050")
        foreach(required_var MC_ROLE_IMU_I2C_INDEX MC_IMU_I2C_ADDRESS)
            if(NOT DEFINED ${required_var})
                message(FATAL_ERROR "Experimental MPU6050 configuration requires ${required_var}")
            endif()
        endforeach()
    endif()

    if(MC_COMP_TYPE STREQUAL "HMC5843")
        foreach(required_var MC_ROLE_COMPASS_I2C_INDEX MC_COMPASS_I2C_ADDRESS)
            if(NOT DEFINED ${required_var})
                message(FATAL_ERROR "Experimental HMC5843 configuration requires ${required_var}")
            endif()
        endforeach()
    endif()

    if(MC_BARO_TYPE STREQUAL "BME280")
        foreach(required_var MC_ROLE_BARO_I2C_INDEX MC_BARO_I2C_ADDRESS)
            if(NOT DEFINED ${required_var})
                message(FATAL_ERROR "Experimental BME280 configuration requires ${required_var}")
            endif()
        endforeach()
    endif()

    if(MC_GPS_TYPE STREQUAL "UBLOX")
        foreach(required_var MC_ROLE_GPS_UART_INDEX MC_ROLE_GPS_UART_BAUD)
            if(NOT DEFINED ${required_var})
                message(FATAL_ERROR "Experimental UBLOX configuration requires ${required_var}")
            endif()
        endforeach()
    endif()

    if(MC_STORAGE_TYPE STREQUAL "DATAFLASH")
        foreach(required_var MC_ROLE_STORAGE_SPI_INDEX MC_STORAGE_CS_PIN)
            if(NOT DEFINED ${required_var})
                message(FATAL_ERROR "Experimental DATAFLASH configuration requires ${required_var}")
            endif()
        endforeach()
    endif()
endif()

##################################################################
# GENERATED CONFIG HEADER
##################################################################

set(MC_IMU_ENABLED_NUM 0)
set(MC_COMPASS_ENABLED_NUM 0)
set(MC_BAROMETER_ENABLED_NUM 0)
set(MC_GPS_ENABLED_NUM 0)
set(MC_STORAGE_ENABLED_NUM 0)

set(MC_ROLE_IMU_I2C_INDEX_VALUE 0)
set(MC_IMU_I2C_ADDRESS_VALUE 0)
set(MC_IMU_DRDY_PIN_VALUE "PZ0")
set(MC_IMU_HAS_DRDY_IRQ_NUM 0)
set(MC_IMU_SAMPLE_RATE_HZ_VALUE 1000)

set(MC_ROLE_COMPASS_I2C_INDEX_VALUE 0)
set(MC_COMPASS_I2C_ADDRESS_VALUE 0)
set(MC_COMPASS_DRDY_PIN_VALUE "PZ0")
set(MC_COMPASS_HAS_DRDY_IRQ_NUM 0)
set(MC_COMPASS_SAMPLE_RATE_HZ_VALUE 50)

set(MC_ROLE_BARO_I2C_INDEX_VALUE 0)
set(MC_BARO_I2C_ADDRESS_VALUE 0)
set(MC_BARO_DRDY_PIN_VALUE "PZ0")
set(MC_BARO_HAS_DRDY_IRQ_NUM 0)
set(MC_BARO_SAMPLE_RATE_HZ_VALUE 50)

set(MC_ROLE_GPS_UART_INDEX_VALUE 0)
set(MC_ROLE_GPS_UART_BAUD_VALUE 0)
set(MC_GPS_EXPECTED_FIX_RATE_HZ_VALUE 5)

set(MC_ROLE_CONSOLE_UART_INDEX_VALUE 0)
set(MC_ROLE_CONSOLE_UART_BAUD_VALUE 115200)

set(MC_ROLE_STORAGE_SPI_INDEX_VALUE 0)
set(MC_STORAGE_CS_PIN_VALUE "PZ0")
set(MC_STORAGE_RESET_PIN_VALUE "PZ0")
set(MC_STORAGE_SPI_FREQUENCY_HZ_VALUE 1000000)

set(MC_STM32_I2C0_ENABLED_NUM 0)
set(MC_STM32_I2C0_INSTANCE_VALUE "I2C1")
set(MC_STM32_I2C0_SCL_PORT_VALUE "GPIOB")
set(MC_STM32_I2C0_SCL_PIN_VALUE "GPIO_PIN_8")
set(MC_STM32_I2C0_SCL_AF_VALUE "GPIO_AF4_I2C1")
set(MC_STM32_I2C0_SDA_PORT_VALUE "GPIOB")
set(MC_STM32_I2C0_SDA_PIN_VALUE "GPIO_PIN_9")
set(MC_STM32_I2C0_SDA_AF_VALUE "GPIO_AF4_I2C1")
set(MC_STM32_I2C0_SPEED_HZ_VALUE 400000)

set(MC_STM32_I2C1_ENABLED_NUM 0)
set(MC_STM32_I2C1_INSTANCE_VALUE "I2C2")
set(MC_STM32_I2C1_SCL_PORT_VALUE "GPIOB")
set(MC_STM32_I2C1_SCL_PIN_VALUE "GPIO_PIN_10")
set(MC_STM32_I2C1_SCL_AF_VALUE "GPIO_AF4_I2C2")
set(MC_STM32_I2C1_SDA_PORT_VALUE "GPIOB")
set(MC_STM32_I2C1_SDA_PIN_VALUE "GPIO_PIN_11")
set(MC_STM32_I2C1_SDA_AF_VALUE "GPIO_AF4_I2C2")
set(MC_STM32_I2C1_SPEED_HZ_VALUE 400000)

set(MC_STM32_SPI0_ENABLED_NUM 0)
set(MC_STM32_SPI0_INSTANCE_VALUE "SPI1")
set(MC_STM32_SPI0_SCK_PORT_VALUE "GPIOA")
set(MC_STM32_SPI0_SCK_PIN_VALUE "GPIO_PIN_5")
set(MC_STM32_SPI0_MISO_PORT_VALUE "GPIOA")
set(MC_STM32_SPI0_MISO_PIN_VALUE "GPIO_PIN_6")
set(MC_STM32_SPI0_MOSI_PORT_VALUE "GPIOA")
set(MC_STM32_SPI0_MOSI_PIN_VALUE "GPIO_PIN_7")
set(MC_STM32_SPI0_AF_VALUE "GPIO_AF5_SPI1")

set(MC_STM32_SPI1_ENABLED_NUM 0)
set(MC_STM32_SPI1_INSTANCE_VALUE "SPI2")
set(MC_STM32_SPI1_SCK_PORT_VALUE "GPIOB")
set(MC_STM32_SPI1_SCK_PIN_VALUE "GPIO_PIN_13")
set(MC_STM32_SPI1_MISO_PORT_VALUE "GPIOB")
set(MC_STM32_SPI1_MISO_PIN_VALUE "GPIO_PIN_14")
set(MC_STM32_SPI1_MOSI_PORT_VALUE "GPIOB")
set(MC_STM32_SPI1_MOSI_PIN_VALUE "GPIO_PIN_15")
set(MC_STM32_SPI1_AF_VALUE "GPIO_AF5_SPI2")

set(MC_STM32_UART0_ENABLED_NUM 0)
set(MC_STM32_UART0_INSTANCE_VALUE "USART1")
set(MC_STM32_UART0_TX_PORT_VALUE "GPIOA")
set(MC_STM32_UART0_TX_PIN_VALUE "GPIO_PIN_9")
set(MC_STM32_UART0_RX_PORT_VALUE "GPIOA")
set(MC_STM32_UART0_RX_PIN_VALUE "GPIO_PIN_10")
set(MC_STM32_UART0_AF_VALUE "GPIO_AF7_USART1")

set(MC_STM32_UART1_ENABLED_NUM 0)
set(MC_STM32_UART1_INSTANCE_VALUE "USART2")
set(MC_STM32_UART1_TX_PORT_VALUE "GPIOA")
set(MC_STM32_UART1_TX_PIN_VALUE "GPIO_PIN_2")
set(MC_STM32_UART1_RX_PORT_VALUE "GPIOA")
set(MC_STM32_UART1_RX_PIN_VALUE "GPIO_PIN_3")
set(MC_STM32_UART1_AF_VALUE "GPIO_AF7_USART2")

set(MC_STM32_UART2_ENABLED_NUM 0)
set(MC_STM32_UART2_INSTANCE_VALUE "USART3")
set(MC_STM32_UART2_TX_PORT_VALUE "GPIOB")
set(MC_STM32_UART2_TX_PIN_VALUE "GPIO_PIN_10")
set(MC_STM32_UART2_RX_PORT_VALUE "GPIOB")
set(MC_STM32_UART2_RX_PIN_VALUE "GPIO_PIN_11")
set(MC_STM32_UART2_AF_VALUE "GPIO_AF7_USART3")

set(MC_STM32_UART3_ENABLED_NUM 0)
set(MC_STM32_UART3_INSTANCE_VALUE "UART4")
set(MC_STM32_UART3_TX_PORT_VALUE "GPIOA")
set(MC_STM32_UART3_TX_PIN_VALUE "GPIO_PIN_0")
set(MC_STM32_UART3_RX_PORT_VALUE "GPIOA")
set(MC_STM32_UART3_RX_PIN_VALUE "GPIO_PIN_1")
set(MC_STM32_UART3_AF_VALUE "GPIO_AF8_UART4")

set(MC_STM32_ADC_ENABLED_NUM 0)
set(MC_STM32_ADC_INSTANCE_VALUE "ADC1")
set(MC_STM32_ADC_CHANNEL_CONFIG_ENTRIES "")

set(MC_STM32_PWM_ENABLED_NUM 0)
set(MC_STM32_PWM_TIMER_VALUE "TIM3")
set(MC_STM32_PWM_TIMER_CLOCK_HZ_VALUE 84000000)
set(MC_STM32_PWM_CHANNEL_COUNT_VALUE 0)
set(MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES "")

set(MC_STM32_GPIO_CONFIG_ENTRIES "")

if(MC_IMU_TYPE STREQUAL "MPU6050")
    set(MC_IMU_ENABLED_NUM 1)
    set(MC_ROLE_IMU_I2C_INDEX_VALUE ${MC_ROLE_IMU_I2C_INDEX})
    set(MC_IMU_I2C_ADDRESS_VALUE ${MC_IMU_I2C_ADDRESS})
    if(DEFINED MC_IMU_DRDY_PIN)
        set(MC_IMU_DRDY_PIN_VALUE "${MC_IMU_DRDY_PIN}")
        set(MC_IMU_HAS_DRDY_IRQ_NUM 1)
    endif()
endif()

if(MC_COMP_TYPE STREQUAL "HMC5843")
    set(MC_COMPASS_ENABLED_NUM 1)
    set(MC_ROLE_COMPASS_I2C_INDEX_VALUE ${MC_ROLE_COMPASS_I2C_INDEX})
    set(MC_COMPASS_I2C_ADDRESS_VALUE ${MC_COMPASS_I2C_ADDRESS})
    if(DEFINED MC_COMPASS_DRDY_PIN)
        set(MC_COMPASS_DRDY_PIN_VALUE "${MC_COMPASS_DRDY_PIN}")
        set(MC_COMPASS_HAS_DRDY_IRQ_NUM 1)
    endif()
endif()

if(MC_BARO_TYPE STREQUAL "BME280")
    set(MC_BAROMETER_ENABLED_NUM 1)
    set(MC_ROLE_BARO_I2C_INDEX_VALUE ${MC_ROLE_BARO_I2C_INDEX})
    set(MC_BARO_I2C_ADDRESS_VALUE ${MC_BARO_I2C_ADDRESS})
    if(DEFINED MC_BARO_DRDY_PIN)
        set(MC_BARO_DRDY_PIN_VALUE "${MC_BARO_DRDY_PIN}")
        set(MC_BARO_HAS_DRDY_IRQ_NUM 1)
    endif()
endif()

if(MC_GPS_TYPE STREQUAL "UBLOX")
    set(MC_GPS_ENABLED_NUM 1)
    set(MC_ROLE_GPS_UART_INDEX_VALUE ${MC_ROLE_GPS_UART_INDEX})
    set(MC_ROLE_GPS_UART_BAUD_VALUE ${MC_ROLE_GPS_UART_BAUD})
endif()

if(DEFINED MC_ROLE_CONSOLE_UART_INDEX)
    set(MC_ROLE_CONSOLE_UART_INDEX_VALUE ${MC_ROLE_CONSOLE_UART_INDEX})
endif()

if(DEFINED MC_ROLE_CONSOLE_UART_BAUD)
    set(MC_ROLE_CONSOLE_UART_BAUD_VALUE ${MC_ROLE_CONSOLE_UART_BAUD})
endif()

if(MC_STORAGE_TYPE STREQUAL "DATAFLASH")
    set(MC_STORAGE_ENABLED_NUM 1)
    set(MC_ROLE_STORAGE_SPI_INDEX_VALUE ${MC_ROLE_STORAGE_SPI_INDEX})
    set(MC_STORAGE_CS_PIN_VALUE "${MC_STORAGE_CS_PIN}")
    if(DEFINED MC_STORAGE_RESET_PIN)
        set(MC_STORAGE_RESET_PIN_VALUE "${MC_STORAGE_RESET_PIN}")
    endif()
endif()

function(mc_set_stm32_pin_values PIN_NAME OUT_PORT_VAR OUT_PIN_VAR)
    string(SUBSTRING "${PIN_NAME}" 1 1 _port_letter)
    string(SUBSTRING "${PIN_NAME}" 2 -1 _pin_number)
    set(${OUT_PORT_VAR} "GPIO${_port_letter}" PARENT_SCOPE)
    set(${OUT_PIN_VAR} "GPIO_PIN_${_pin_number}" PARENT_SCOPE)
endfunction()

function(mc_append_stm32_gpio_config_entry OUT_VAR PIN_NAME INTERRUPT_CAPABLE)
    if(NOT PIN_NAME OR "${PIN_NAME}" STREQUAL "PZ0")
        set(${OUT_VAR} "${${OUT_VAR}}" PARENT_SCOPE)
        return()
    endif()

    string(SUBSTRING "${PIN_NAME}" 1 1 _port_letter)
    string(SUBSTRING "${PIN_NAME}" 2 -1 _pin_number)
    if(_port_letter STREQUAL "A")
        set(_port_index 0)
    elseif(_port_letter STREQUAL "B")
        set(_port_index 1)
    elseif(_port_letter STREQUAL "C")
        set(_port_index 2)
    elseif(_port_letter STREQUAL "D")
        set(_port_index 3)
    elseif(_port_letter STREQUAL "E")
        set(_port_index 4)
    elseif(_port_letter STREQUAL "F")
        set(_port_index 5)
    elseif(_port_letter STREQUAL "G")
        set(_port_index 6)
    elseif(_port_letter STREQUAL "H")
        set(_port_index 7)
    elseif(_port_letter STREQUAL "I")
        set(_port_index 8)
    else()
        message(FATAL_ERROR "Invalid STM32 GPIO pin name: ${PIN_NAME}")
    endif()
    math(EXPR _logical_pin "(${_port_index} * 16) + ${_pin_number}")
    set(_entry "    {${_logical_pin}, GPIO${_port_letter}, GPIO_PIN_${_pin_number}, false, ${INTERRUPT_CAPABLE}},\n")
    set(${OUT_VAR} "${${OUT_VAR}}${_entry}" PARENT_SCOPE)
endfunction()

function(mc_append_stm32_adc_channel_entry OUT_VAR LOGICAL_INDEX PIN_NAME ADC_CHANNEL)
    if(NOT PIN_NAME OR "${PIN_NAME}" STREQUAL "PZ0")
        set(${OUT_VAR} "${${OUT_VAR}}" PARENT_SCOPE)
        return()
    endif()
    string(SUBSTRING "${PIN_NAME}" 1 1 _port_letter)
    string(SUBSTRING "${PIN_NAME}" 2 -1 _pin_number)
    set(_entry "    {${LOGICAL_INDEX}, GPIO${_port_letter}, GPIO_PIN_${_pin_number}, ${ADC_CHANNEL}},\n")
    set(${OUT_VAR} "${${OUT_VAR}}${_entry}" PARENT_SCOPE)
endfunction()

function(mc_append_stm32_pwm_channel_entry OUT_VAR LOGICAL_CHANNEL PIN_NAME AF TIM_CHANNEL)
    if(NOT PIN_NAME OR "${PIN_NAME}" STREQUAL "PZ0")
        set(${OUT_VAR} "${${OUT_VAR}}" PARENT_SCOPE)
        return()
    endif()
    string(SUBSTRING "${PIN_NAME}" 1 1 _port_letter)
    string(SUBSTRING "${PIN_NAME}" 2 -1 _pin_number)
    set(_entry "        {${LOGICAL_CHANNEL}, GPIO${_port_letter}, GPIO_PIN_${_pin_number}, ${AF}, TIM_CHANNEL_${TIM_CHANNEL}},\n")
    set(${OUT_VAR} "${${OUT_VAR}}${_entry}" PARENT_SCOPE)
endfunction()

if(TARGET_ARCH_FAMILY STREQUAL "stm32")
    if(DEFINED MC_ROLE_IMU_I2C_INDEX AND MC_ROLE_IMU_I2C_INDEX EQUAL 0)
        set(MC_STM32_I2C0_ENABLED_NUM 1)
        set(MC_STM32_I2C0_INSTANCE_VALUE "${MC_ROLE_IMU_I2C_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_IMU_I2C_SCL_PIN}" MC_STM32_I2C0_SCL_PORT_VALUE MC_STM32_I2C0_SCL_PIN_VALUE)
        set(MC_STM32_I2C0_SCL_AF_VALUE "${MC_ROLE_IMU_I2C_AF}")
        mc_set_stm32_pin_values("${MC_ROLE_IMU_I2C_SDA_PIN}" MC_STM32_I2C0_SDA_PORT_VALUE MC_STM32_I2C0_SDA_PIN_VALUE)
        set(MC_STM32_I2C0_SDA_AF_VALUE "${MC_ROLE_IMU_I2C_AF}")
        if(DEFINED MC_ROLE_IMU_I2C_SPEED_HZ)
            set(MC_STM32_I2C0_SPEED_HZ_VALUE "${MC_ROLE_IMU_I2C_SPEED_HZ}")
        endif()
    elseif(DEFINED MC_ROLE_COMPASS_I2C_INDEX AND MC_ROLE_COMPASS_I2C_INDEX EQUAL 0)
        set(MC_STM32_I2C0_ENABLED_NUM 1)
        set(MC_STM32_I2C0_INSTANCE_VALUE "${MC_ROLE_COMPASS_I2C_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_COMPASS_I2C_SCL_PIN}" MC_STM32_I2C0_SCL_PORT_VALUE MC_STM32_I2C0_SCL_PIN_VALUE)
        set(MC_STM32_I2C0_SCL_AF_VALUE "${MC_ROLE_COMPASS_I2C_AF}")
        mc_set_stm32_pin_values("${MC_ROLE_COMPASS_I2C_SDA_PIN}" MC_STM32_I2C0_SDA_PORT_VALUE MC_STM32_I2C0_SDA_PIN_VALUE)
        set(MC_STM32_I2C0_SDA_AF_VALUE "${MC_ROLE_COMPASS_I2C_AF}")
        if(DEFINED MC_ROLE_COMPASS_I2C_SPEED_HZ)
            set(MC_STM32_I2C0_SPEED_HZ_VALUE "${MC_ROLE_COMPASS_I2C_SPEED_HZ}")
        endif()
    elseif(DEFINED MC_ROLE_BARO_I2C_INDEX AND MC_ROLE_BARO_I2C_INDEX EQUAL 0)
        set(MC_STM32_I2C0_ENABLED_NUM 1)
        set(MC_STM32_I2C0_INSTANCE_VALUE "${MC_ROLE_BARO_I2C_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_BARO_I2C_SCL_PIN}" MC_STM32_I2C0_SCL_PORT_VALUE MC_STM32_I2C0_SCL_PIN_VALUE)
        set(MC_STM32_I2C0_SCL_AF_VALUE "${MC_ROLE_BARO_I2C_AF}")
        mc_set_stm32_pin_values("${MC_ROLE_BARO_I2C_SDA_PIN}" MC_STM32_I2C0_SDA_PORT_VALUE MC_STM32_I2C0_SDA_PIN_VALUE)
        set(MC_STM32_I2C0_SDA_AF_VALUE "${MC_ROLE_BARO_I2C_AF}")
        if(DEFINED MC_ROLE_BARO_I2C_SPEED_HZ)
            set(MC_STM32_I2C0_SPEED_HZ_VALUE "${MC_ROLE_BARO_I2C_SPEED_HZ}")
        endif()
    endif()

    if(DEFINED MC_ROLE_IMU_I2C_INDEX AND MC_ROLE_IMU_I2C_INDEX EQUAL 1)
        set(MC_STM32_I2C1_ENABLED_NUM 1)
        set(MC_STM32_I2C1_INSTANCE_VALUE "${MC_ROLE_IMU_I2C_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_IMU_I2C_SCL_PIN}" MC_STM32_I2C1_SCL_PORT_VALUE MC_STM32_I2C1_SCL_PIN_VALUE)
        set(MC_STM32_I2C1_SCL_AF_VALUE "${MC_ROLE_IMU_I2C_AF}")
        mc_set_stm32_pin_values("${MC_ROLE_IMU_I2C_SDA_PIN}" MC_STM32_I2C1_SDA_PORT_VALUE MC_STM32_I2C1_SDA_PIN_VALUE)
        set(MC_STM32_I2C1_SDA_AF_VALUE "${MC_ROLE_IMU_I2C_AF}")
        if(DEFINED MC_ROLE_IMU_I2C_SPEED_HZ)
            set(MC_STM32_I2C1_SPEED_HZ_VALUE "${MC_ROLE_IMU_I2C_SPEED_HZ}")
        endif()
    elseif(DEFINED MC_ROLE_COMPASS_I2C_INDEX AND MC_ROLE_COMPASS_I2C_INDEX EQUAL 1)
        set(MC_STM32_I2C1_ENABLED_NUM 1)
        set(MC_STM32_I2C1_INSTANCE_VALUE "${MC_ROLE_COMPASS_I2C_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_COMPASS_I2C_SCL_PIN}" MC_STM32_I2C1_SCL_PORT_VALUE MC_STM32_I2C1_SCL_PIN_VALUE)
        set(MC_STM32_I2C1_SCL_AF_VALUE "${MC_ROLE_COMPASS_I2C_AF}")
        mc_set_stm32_pin_values("${MC_ROLE_COMPASS_I2C_SDA_PIN}" MC_STM32_I2C1_SDA_PORT_VALUE MC_STM32_I2C1_SDA_PIN_VALUE)
        set(MC_STM32_I2C1_SDA_AF_VALUE "${MC_ROLE_COMPASS_I2C_AF}")
        if(DEFINED MC_ROLE_COMPASS_I2C_SPEED_HZ)
            set(MC_STM32_I2C1_SPEED_HZ_VALUE "${MC_ROLE_COMPASS_I2C_SPEED_HZ}")
        endif()
    elseif(DEFINED MC_ROLE_BARO_I2C_INDEX AND MC_ROLE_BARO_I2C_INDEX EQUAL 1)
        set(MC_STM32_I2C1_ENABLED_NUM 1)
        set(MC_STM32_I2C1_INSTANCE_VALUE "${MC_ROLE_BARO_I2C_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_BARO_I2C_SCL_PIN}" MC_STM32_I2C1_SCL_PORT_VALUE MC_STM32_I2C1_SCL_PIN_VALUE)
        set(MC_STM32_I2C1_SCL_AF_VALUE "${MC_ROLE_BARO_I2C_AF}")
        mc_set_stm32_pin_values("${MC_ROLE_BARO_I2C_SDA_PIN}" MC_STM32_I2C1_SDA_PORT_VALUE MC_STM32_I2C1_SDA_PIN_VALUE)
        set(MC_STM32_I2C1_SDA_AF_VALUE "${MC_ROLE_BARO_I2C_AF}")
        if(DEFINED MC_ROLE_BARO_I2C_SPEED_HZ)
            set(MC_STM32_I2C1_SPEED_HZ_VALUE "${MC_ROLE_BARO_I2C_SPEED_HZ}")
        endif()
    endif()

    if(DEFINED MC_ROLE_STORAGE_SPI_INDEX AND MC_ROLE_STORAGE_SPI_INDEX EQUAL 0)
        set(MC_STM32_SPI0_ENABLED_NUM 1)
        set(MC_STM32_SPI0_INSTANCE_VALUE "${MC_ROLE_STORAGE_SPI_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_STORAGE_SPI_SCK_PIN}" MC_STM32_SPI0_SCK_PORT_VALUE MC_STM32_SPI0_SCK_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_STORAGE_SPI_MISO_PIN}" MC_STM32_SPI0_MISO_PORT_VALUE MC_STM32_SPI0_MISO_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_STORAGE_SPI_MOSI_PIN}" MC_STM32_SPI0_MOSI_PORT_VALUE MC_STM32_SPI0_MOSI_PIN_VALUE)
        set(MC_STM32_SPI0_AF_VALUE "${MC_ROLE_STORAGE_SPI_AF}")
    endif()

    if(DEFINED MC_ROLE_STORAGE_SPI_INDEX AND MC_ROLE_STORAGE_SPI_INDEX EQUAL 1)
        set(MC_STM32_SPI1_ENABLED_NUM 1)
        set(MC_STM32_SPI1_INSTANCE_VALUE "${MC_ROLE_STORAGE_SPI_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_STORAGE_SPI_SCK_PIN}" MC_STM32_SPI1_SCK_PORT_VALUE MC_STM32_SPI1_SCK_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_STORAGE_SPI_MISO_PIN}" MC_STM32_SPI1_MISO_PORT_VALUE MC_STM32_SPI1_MISO_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_STORAGE_SPI_MOSI_PIN}" MC_STM32_SPI1_MOSI_PORT_VALUE MC_STM32_SPI1_MOSI_PIN_VALUE)
        set(MC_STM32_SPI1_AF_VALUE "${MC_ROLE_STORAGE_SPI_AF}")
    endif()

    if(DEFINED MC_ROLE_BATTERY_ADC_INDEX AND DEFINED MC_ROLE_BATTERY_ADC_PIN AND DEFINED MC_ROLE_BATTERY_ADC_CHANNEL)
        set(MC_STM32_ADC_ENABLED_NUM 1)
        mc_append_stm32_adc_channel_entry(MC_STM32_ADC_CHANNEL_CONFIG_ENTRIES
            "${MC_ROLE_BATTERY_ADC_INDEX}"
            "${MC_ROLE_BATTERY_ADC_PIN}"
            "${MC_ROLE_BATTERY_ADC_CHANNEL}")
    endif()

    if(DEFINED MC_ROLE_MOTOR_PWM_TIMER AND DEFINED MC_ROLE_MOTOR_PWM_AF)
        set(MC_STM32_PWM_ENABLED_NUM 1)
        set(MC_STM32_PWM_TIMER_VALUE "${MC_ROLE_MOTOR_PWM_TIMER}")
        set(MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES "")
        set(MC_STM32_PWM_CHANNEL_COUNT_VALUE 0)
        if(DEFINED MC_MOTOR1_PWM_PIN AND DEFINED MC_MOTOR1_PWM_CHANNEL)
            mc_append_stm32_pwm_channel_entry(MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES 0 "${MC_MOTOR1_PWM_PIN}" "${MC_ROLE_MOTOR_PWM_AF}" "${MC_MOTOR1_PWM_CHANNEL}")
            math(EXPR MC_STM32_PWM_CHANNEL_COUNT_VALUE "${MC_STM32_PWM_CHANNEL_COUNT_VALUE} + 1")
        endif()
        if(DEFINED MC_MOTOR2_PWM_PIN AND DEFINED MC_MOTOR2_PWM_CHANNEL)
            mc_append_stm32_pwm_channel_entry(MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES 1 "${MC_MOTOR2_PWM_PIN}" "${MC_ROLE_MOTOR_PWM_AF}" "${MC_MOTOR2_PWM_CHANNEL}")
            math(EXPR MC_STM32_PWM_CHANNEL_COUNT_VALUE "${MC_STM32_PWM_CHANNEL_COUNT_VALUE} + 1")
        endif()
        if(DEFINED MC_MOTOR3_PWM_PIN AND DEFINED MC_MOTOR3_PWM_CHANNEL)
            mc_append_stm32_pwm_channel_entry(MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES 2 "${MC_MOTOR3_PWM_PIN}" "${MC_ROLE_MOTOR_PWM_AF}" "${MC_MOTOR3_PWM_CHANNEL}")
            math(EXPR MC_STM32_PWM_CHANNEL_COUNT_VALUE "${MC_STM32_PWM_CHANNEL_COUNT_VALUE} + 1")
        endif()
        if(DEFINED MC_MOTOR4_PWM_PIN AND DEFINED MC_MOTOR4_PWM_CHANNEL)
            mc_append_stm32_pwm_channel_entry(MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES 3 "${MC_MOTOR4_PWM_PIN}" "${MC_ROLE_MOTOR_PWM_AF}" "${MC_MOTOR4_PWM_CHANNEL}")
            math(EXPR MC_STM32_PWM_CHANNEL_COUNT_VALUE "${MC_STM32_PWM_CHANNEL_COUNT_VALUE} + 1")
        endif()
    endif()

    if(DEFINED MC_ROLE_CONSOLE_UART_INDEX AND MC_ROLE_CONSOLE_UART_INDEX EQUAL 0)
        set(MC_STM32_UART0_ENABLED_NUM 1)
        set(MC_STM32_UART0_INSTANCE_VALUE "${MC_ROLE_CONSOLE_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_TX_PIN}" MC_STM32_UART0_TX_PORT_VALUE MC_STM32_UART0_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_RX_PIN}" MC_STM32_UART0_RX_PORT_VALUE MC_STM32_UART0_RX_PIN_VALUE)
        set(MC_STM32_UART0_AF_VALUE "${MC_ROLE_CONSOLE_UART_AF}")
    elseif(DEFINED MC_ROLE_CONSOLE_UART_INDEX AND MC_ROLE_CONSOLE_UART_INDEX EQUAL 1)
        set(MC_STM32_UART1_ENABLED_NUM 1)
        set(MC_STM32_UART1_INSTANCE_VALUE "${MC_ROLE_CONSOLE_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_TX_PIN}" MC_STM32_UART1_TX_PORT_VALUE MC_STM32_UART1_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_RX_PIN}" MC_STM32_UART1_RX_PORT_VALUE MC_STM32_UART1_RX_PIN_VALUE)
        set(MC_STM32_UART1_AF_VALUE "${MC_ROLE_CONSOLE_UART_AF}")
    elseif(DEFINED MC_ROLE_CONSOLE_UART_INDEX AND MC_ROLE_CONSOLE_UART_INDEX EQUAL 2)
        set(MC_STM32_UART2_ENABLED_NUM 1)
        set(MC_STM32_UART2_INSTANCE_VALUE "${MC_ROLE_CONSOLE_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_TX_PIN}" MC_STM32_UART2_TX_PORT_VALUE MC_STM32_UART2_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_RX_PIN}" MC_STM32_UART2_RX_PORT_VALUE MC_STM32_UART2_RX_PIN_VALUE)
        set(MC_STM32_UART2_AF_VALUE "${MC_ROLE_CONSOLE_UART_AF}")
    elseif(DEFINED MC_ROLE_CONSOLE_UART_INDEX AND MC_ROLE_CONSOLE_UART_INDEX EQUAL 3)
        set(MC_STM32_UART3_ENABLED_NUM 1)
        set(MC_STM32_UART3_INSTANCE_VALUE "${MC_ROLE_CONSOLE_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_TX_PIN}" MC_STM32_UART3_TX_PORT_VALUE MC_STM32_UART3_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_CONSOLE_UART_RX_PIN}" MC_STM32_UART3_RX_PORT_VALUE MC_STM32_UART3_RX_PIN_VALUE)
        set(MC_STM32_UART3_AF_VALUE "${MC_ROLE_CONSOLE_UART_AF}")
    endif()
    if(DEFINED MC_ROLE_GPS_UART_INDEX AND MC_ROLE_GPS_UART_INDEX EQUAL 0)
        set(MC_STM32_UART0_ENABLED_NUM 1)
        set(MC_STM32_UART0_INSTANCE_VALUE "${MC_ROLE_GPS_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_TX_PIN}" MC_STM32_UART0_TX_PORT_VALUE MC_STM32_UART0_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_RX_PIN}" MC_STM32_UART0_RX_PORT_VALUE MC_STM32_UART0_RX_PIN_VALUE)
        set(MC_STM32_UART0_AF_VALUE "${MC_ROLE_GPS_UART_AF}")
    elseif(DEFINED MC_ROLE_GPS_UART_INDEX AND MC_ROLE_GPS_UART_INDEX EQUAL 1)
        set(MC_STM32_UART1_ENABLED_NUM 1)
        set(MC_STM32_UART1_INSTANCE_VALUE "${MC_ROLE_GPS_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_TX_PIN}" MC_STM32_UART1_TX_PORT_VALUE MC_STM32_UART1_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_RX_PIN}" MC_STM32_UART1_RX_PORT_VALUE MC_STM32_UART1_RX_PIN_VALUE)
        set(MC_STM32_UART1_AF_VALUE "${MC_ROLE_GPS_UART_AF}")
    elseif(DEFINED MC_ROLE_GPS_UART_INDEX AND MC_ROLE_GPS_UART_INDEX EQUAL 2)
        set(MC_STM32_UART2_ENABLED_NUM 1)
        set(MC_STM32_UART2_INSTANCE_VALUE "${MC_ROLE_GPS_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_TX_PIN}" MC_STM32_UART2_TX_PORT_VALUE MC_STM32_UART2_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_RX_PIN}" MC_STM32_UART2_RX_PORT_VALUE MC_STM32_UART2_RX_PIN_VALUE)
        set(MC_STM32_UART2_AF_VALUE "${MC_ROLE_GPS_UART_AF}")
    elseif(DEFINED MC_ROLE_GPS_UART_INDEX AND MC_ROLE_GPS_UART_INDEX EQUAL 3)
        set(MC_STM32_UART3_ENABLED_NUM 1)
        set(MC_STM32_UART3_INSTANCE_VALUE "${MC_ROLE_GPS_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_TX_PIN}" MC_STM32_UART3_TX_PORT_VALUE MC_STM32_UART3_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_GPS_UART_RX_PIN}" MC_STM32_UART3_RX_PORT_VALUE MC_STM32_UART3_RX_PIN_VALUE)
        set(MC_STM32_UART3_AF_VALUE "${MC_ROLE_GPS_UART_AF}")
    endif()
    if(DEFINED MC_ROLE_TELEMETRY_UART_INDEX AND MC_ROLE_TELEMETRY_UART_INDEX EQUAL 0)
        set(MC_STM32_UART0_ENABLED_NUM 1)
        set(MC_STM32_UART0_INSTANCE_VALUE "${MC_ROLE_TELEMETRY_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_TX_PIN}" MC_STM32_UART0_TX_PORT_VALUE MC_STM32_UART0_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_RX_PIN}" MC_STM32_UART0_RX_PORT_VALUE MC_STM32_UART0_RX_PIN_VALUE)
        set(MC_STM32_UART0_AF_VALUE "${MC_ROLE_TELEMETRY_UART_AF}")
    elseif(DEFINED MC_ROLE_TELEMETRY_UART_INDEX AND MC_ROLE_TELEMETRY_UART_INDEX EQUAL 1)
        set(MC_STM32_UART1_ENABLED_NUM 1)
        set(MC_STM32_UART1_INSTANCE_VALUE "${MC_ROLE_TELEMETRY_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_TX_PIN}" MC_STM32_UART1_TX_PORT_VALUE MC_STM32_UART1_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_RX_PIN}" MC_STM32_UART1_RX_PORT_VALUE MC_STM32_UART1_RX_PIN_VALUE)
        set(MC_STM32_UART1_AF_VALUE "${MC_ROLE_TELEMETRY_UART_AF}")
    elseif(DEFINED MC_ROLE_TELEMETRY_UART_INDEX AND MC_ROLE_TELEMETRY_UART_INDEX EQUAL 2)
        set(MC_STM32_UART2_ENABLED_NUM 1)
        set(MC_STM32_UART2_INSTANCE_VALUE "${MC_ROLE_TELEMETRY_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_TX_PIN}" MC_STM32_UART2_TX_PORT_VALUE MC_STM32_UART2_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_RX_PIN}" MC_STM32_UART2_RX_PORT_VALUE MC_STM32_UART2_RX_PIN_VALUE)
        set(MC_STM32_UART2_AF_VALUE "${MC_ROLE_TELEMETRY_UART_AF}")
    elseif(DEFINED MC_ROLE_TELEMETRY_UART_INDEX AND MC_ROLE_TELEMETRY_UART_INDEX EQUAL 3)
        set(MC_STM32_UART3_ENABLED_NUM 1)
        set(MC_STM32_UART3_INSTANCE_VALUE "${MC_ROLE_TELEMETRY_UART_INSTANCE}")
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_TX_PIN}" MC_STM32_UART3_TX_PORT_VALUE MC_STM32_UART3_TX_PIN_VALUE)
        mc_set_stm32_pin_values("${MC_ROLE_TELEMETRY_UART_RX_PIN}" MC_STM32_UART3_RX_PORT_VALUE MC_STM32_UART3_RX_PIN_VALUE)
        set(MC_STM32_UART3_AF_VALUE "${MC_ROLE_TELEMETRY_UART_AF}")
    endif()

    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_STATUS_LED0_PIN}" false)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_STATUS_LED1_PIN}" false)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_STATUS_LED2_PIN}" false)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_STATUS_LED3_PIN}" false)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_IMU_DRDY_PIN_VALUE}" true)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_COMPASS_DRDY_PIN_VALUE}" true)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_BARO_DRDY_PIN_VALUE}" true)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_STORAGE_CS_PIN_VALUE}" false)
    mc_append_stm32_gpio_config_entry(MC_STM32_GPIO_CONFIG_ENTRIES "${MC_STORAGE_RESET_PIN_VALUE}" false)
endif()

set(MC_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${MC_GENERATED_DIR}")

set(MC_ENABLE_LOGGING_NUM 0)
if(DEFINED MC_ENABLE_LOGGING AND MC_ENABLE_LOGGING)
    set(MC_ENABLE_LOGGING_NUM 1)
endif()

set(MC_EXPERIMENTAL_BUILD_NUM 0)
if(MC_EXPERIMENTAL_BUILD)
    set(MC_EXPERIMENTAL_BUILD_NUM 1)
endif()

file(WRITE "${MC_GENERATED_DIR}/experimental_build_config.h"
"#pragma once\n"
"namespace experimental_build_config {\n"
"static constexpr const char* board_name = \"${MC_BOARD_NAME}\";\n"
"static constexpr const char* board_mcu = \"${MC_BOARD_MCU}\";\n"
"static constexpr const char* feature_set = \"${MC_FEATURE_SET_NAME}\";\n"
"static constexpr bool experimental_build = ${MC_EXPERIMENTAL_BUILD_NUM};\n"
"static constexpr bool logging_enabled = ${MC_ENABLE_LOGGING_NUM};\n"
"namespace imu {\n"
"static constexpr bool enabled = ${MC_IMU_ENABLED_NUM};\n"
"static constexpr unsigned bus_index = ${MC_ROLE_IMU_I2C_INDEX_VALUE};\n"
"static constexpr unsigned address = ${MC_IMU_I2C_ADDRESS_VALUE};\n"
"static constexpr const char* drdy_pin = \"${MC_IMU_DRDY_PIN_VALUE}\";\n"
"static constexpr bool has_data_ready_irq = ${MC_IMU_HAS_DRDY_IRQ_NUM};\n"
"static constexpr unsigned sample_rate_hz = ${MC_IMU_SAMPLE_RATE_HZ_VALUE};\n"
"}\n"
"namespace compass {\n"
"static constexpr bool enabled = ${MC_COMPASS_ENABLED_NUM};\n"
"static constexpr unsigned bus_index = ${MC_ROLE_COMPASS_I2C_INDEX_VALUE};\n"
"static constexpr unsigned address = ${MC_COMPASS_I2C_ADDRESS_VALUE};\n"
"static constexpr const char* drdy_pin = \"${MC_COMPASS_DRDY_PIN_VALUE}\";\n"
"static constexpr bool has_data_ready_irq = ${MC_COMPASS_HAS_DRDY_IRQ_NUM};\n"
"static constexpr unsigned sample_rate_hz = ${MC_COMPASS_SAMPLE_RATE_HZ_VALUE};\n"
"}\n"
"namespace barometer {\n"
"static constexpr bool enabled = ${MC_BAROMETER_ENABLED_NUM};\n"
"static constexpr unsigned bus_index = ${MC_ROLE_BARO_I2C_INDEX_VALUE};\n"
"static constexpr unsigned address = ${MC_BARO_I2C_ADDRESS_VALUE};\n"
"static constexpr const char* drdy_pin = \"${MC_BARO_DRDY_PIN_VALUE}\";\n"
"static constexpr bool has_data_ready_irq = ${MC_BARO_HAS_DRDY_IRQ_NUM};\n"
"static constexpr unsigned sample_rate_hz = ${MC_BARO_SAMPLE_RATE_HZ_VALUE};\n"
"}\n"
"namespace gps {\n"
"static constexpr bool enabled = ${MC_GPS_ENABLED_NUM};\n"
"static constexpr unsigned uart_index = ${MC_ROLE_GPS_UART_INDEX_VALUE};\n"
"static constexpr unsigned baud = ${MC_ROLE_GPS_UART_BAUD_VALUE};\n"
"static constexpr unsigned expected_fix_rate_hz = ${MC_GPS_EXPECTED_FIX_RATE_HZ_VALUE};\n"
"}\n"
"namespace heartbeat {\n"
"static constexpr bool enabled = true;\n"
"static constexpr unsigned uart_index = ${MC_ROLE_CONSOLE_UART_INDEX_VALUE};\n"
"static constexpr unsigned baud = ${MC_ROLE_CONSOLE_UART_BAUD_VALUE};\n"
"}\n"
"namespace leds {\n"
"static constexpr bool enabled = true;\n"
"static constexpr const char* status_led0_pin = \"${MC_STATUS_LED0_PIN}\";\n"
"static constexpr const char* status_led1_pin = \"${MC_STATUS_LED1_PIN}\";\n"
"static constexpr const char* status_led2_pin = \"${MC_STATUS_LED2_PIN}\";\n"
"static constexpr const char* status_led3_pin = \"${MC_STATUS_LED3_PIN}\";\n"
"static constexpr bool active_high = true;\n"
"}\n"
"namespace storage {\n"
"static constexpr bool enabled = ${MC_STORAGE_ENABLED_NUM};\n"
"static constexpr unsigned spi_index = ${MC_ROLE_STORAGE_SPI_INDEX_VALUE};\n"
"static constexpr const char* chip_select_pin = \"${MC_STORAGE_CS_PIN_VALUE}\";\n"
"static constexpr const char* reset_pin = \"${MC_STORAGE_RESET_PIN_VALUE}\";\n"
"static constexpr unsigned spi_frequency_hz = ${MC_STORAGE_SPI_FREQUENCY_HZ_VALUE};\n"
"}\n"
"}\n")

file(WRITE "${MC_GENERATED_DIR}/experimental_stm32_board_config.h"
"#pragma once\n"
"#include <arm/stm32/i2c/i2c_backend.h>\n"
"#include <arm/stm32/gpio/gpio_backend.h>\n"
"#include <arm/stm32/spi/spi_backend.h>\n"
"#include <arm/stm32/uart/uart_backend.h>\n"
"#include <arm/stm32/adc/adc_backend.h>\n"
"#include <arm/stm32/pwm/pwm_backend.h>\n"
"#include <stm32f4xx_hal.h>\n"
"namespace experimental_stm32_board_config {\n"
"static constexpr bool i2c0_enabled = ${MC_STM32_I2C0_ENABLED_NUM};\n"
"static constexpr bool i2c1_enabled = ${MC_STM32_I2C1_ENABLED_NUM};\n"
"static constexpr bool spi0_enabled = ${MC_STM32_SPI0_ENABLED_NUM};\n"
"static constexpr bool spi1_enabled = ${MC_STM32_SPI1_ENABLED_NUM};\n"
"static constexpr bool uart0_enabled = ${MC_STM32_UART0_ENABLED_NUM};\n"
"static constexpr bool uart1_enabled = ${MC_STM32_UART1_ENABLED_NUM};\n"
"static constexpr bool uart2_enabled = ${MC_STM32_UART2_ENABLED_NUM};\n"
"static constexpr bool uart3_enabled = ${MC_STM32_UART3_ENABLED_NUM};\n"
"static constexpr bool adc_enabled = ${MC_STM32_ADC_ENABLED_NUM};\n"
"static constexpr bool pwm_enabled = ${MC_STM32_PWM_ENABLED_NUM};\n"
"static constexpr stm32::Stm32I2cBusConfig i2c0 = {\n"
"    0,\n"
"    ${MC_STM32_I2C0_INSTANCE_VALUE},\n"
"    {${MC_STM32_I2C0_SCL_PORT_VALUE}, ${MC_STM32_I2C0_SCL_PIN_VALUE}, ${MC_STM32_I2C0_SCL_AF_VALUE}},\n"
"    {${MC_STM32_I2C0_SDA_PORT_VALUE}, ${MC_STM32_I2C0_SDA_PIN_VALUE}, ${MC_STM32_I2C0_SDA_AF_VALUE}},\n"
"    ${MC_STM32_I2C0_SPEED_HZ_VALUE}\n"
"};\n"
"static constexpr stm32::Stm32I2cBusConfig i2c1 = {\n"
"    1,\n"
"    ${MC_STM32_I2C1_INSTANCE_VALUE},\n"
"    {${MC_STM32_I2C1_SCL_PORT_VALUE}, ${MC_STM32_I2C1_SCL_PIN_VALUE}, ${MC_STM32_I2C1_SCL_AF_VALUE}},\n"
"    {${MC_STM32_I2C1_SDA_PORT_VALUE}, ${MC_STM32_I2C1_SDA_PIN_VALUE}, ${MC_STM32_I2C1_SDA_AF_VALUE}},\n"
"    ${MC_STM32_I2C1_SPEED_HZ_VALUE}\n"
"};\n"
"static constexpr stm32::Stm32SpiBusConfig spi0 = {\n"
"    0,\n"
"    ${MC_STM32_SPI0_INSTANCE_VALUE},\n"
"    {${MC_STM32_SPI0_SCK_PORT_VALUE}, ${MC_STM32_SPI0_SCK_PIN_VALUE}, ${MC_STM32_SPI0_AF_VALUE}},\n"
"    {${MC_STM32_SPI0_MISO_PORT_VALUE}, ${MC_STM32_SPI0_MISO_PIN_VALUE}, ${MC_STM32_SPI0_AF_VALUE}},\n"
"    {${MC_STM32_SPI0_MOSI_PORT_VALUE}, ${MC_STM32_SPI0_MOSI_PIN_VALUE}, ${MC_STM32_SPI0_AF_VALUE}}\n"
"};\n"
"static constexpr stm32::Stm32SpiBusConfig spi1 = {\n"
"    1,\n"
"    ${MC_STM32_SPI1_INSTANCE_VALUE},\n"
"    {${MC_STM32_SPI1_SCK_PORT_VALUE}, ${MC_STM32_SPI1_SCK_PIN_VALUE}, ${MC_STM32_SPI1_AF_VALUE}},\n"
"    {${MC_STM32_SPI1_MISO_PORT_VALUE}, ${MC_STM32_SPI1_MISO_PIN_VALUE}, ${MC_STM32_SPI1_AF_VALUE}},\n"
"    {${MC_STM32_SPI1_MOSI_PORT_VALUE}, ${MC_STM32_SPI1_MOSI_PIN_VALUE}, ${MC_STM32_SPI1_AF_VALUE}}\n"
"};\n"
"static constexpr stm32::Stm32UartPortConfig uart0 = {\n"
"    0,\n"
"    ${MC_STM32_UART0_INSTANCE_VALUE},\n"
"    {${MC_STM32_UART0_TX_PORT_VALUE}, ${MC_STM32_UART0_TX_PIN_VALUE}, ${MC_STM32_UART0_AF_VALUE}},\n"
"    {${MC_STM32_UART0_RX_PORT_VALUE}, ${MC_STM32_UART0_RX_PIN_VALUE}, ${MC_STM32_UART0_AF_VALUE}}\n"
"};\n"
"static constexpr stm32::Stm32UartPortConfig uart1 = {\n"
"    1,\n"
"    ${MC_STM32_UART1_INSTANCE_VALUE},\n"
"    {${MC_STM32_UART1_TX_PORT_VALUE}, ${MC_STM32_UART1_TX_PIN_VALUE}, ${MC_STM32_UART1_AF_VALUE}},\n"
"    {${MC_STM32_UART1_RX_PORT_VALUE}, ${MC_STM32_UART1_RX_PIN_VALUE}, ${MC_STM32_UART1_AF_VALUE}}\n"
"};\n"
"static constexpr stm32::Stm32UartPortConfig uart2 = {\n"
"    2,\n"
"    ${MC_STM32_UART2_INSTANCE_VALUE},\n"
"    {${MC_STM32_UART2_TX_PORT_VALUE}, ${MC_STM32_UART2_TX_PIN_VALUE}, ${MC_STM32_UART2_AF_VALUE}},\n"
"    {${MC_STM32_UART2_RX_PORT_VALUE}, ${MC_STM32_UART2_RX_PIN_VALUE}, ${MC_STM32_UART2_AF_VALUE}}\n"
"};\n"
"static constexpr stm32::Stm32UartPortConfig uart3 = {\n"
"    3,\n"
"    ${MC_STM32_UART3_INSTANCE_VALUE},\n"
"    {${MC_STM32_UART3_TX_PORT_VALUE}, ${MC_STM32_UART3_TX_PIN_VALUE}, ${MC_STM32_UART3_AF_VALUE}},\n"
"    {${MC_STM32_UART3_RX_PORT_VALUE}, ${MC_STM32_UART3_RX_PIN_VALUE}, ${MC_STM32_UART3_AF_VALUE}}\n"
"};\n"
"static constexpr stm32::Stm32AdcUnitConfig adc_unit = {\n"
"    ${MC_STM32_ADC_INSTANCE_VALUE}\n"
"};\n"
"static constexpr stm32::Stm32AdcChannelHardwareConfig adc_channels[] = {\n"
"${MC_STM32_ADC_CHANNEL_CONFIG_ENTRIES}"
"};\n"
"static constexpr uint8_t adc_channel_count = sizeof(adc_channels) / sizeof(adc_channels[0]);\n"
"static constexpr stm32::Stm32PwmTimerConfig pwm = {\n"
"    ${MC_STM32_PWM_TIMER_VALUE},\n"
"    ${MC_STM32_PWM_TIMER_CLOCK_HZ_VALUE},\n"
"    ${MC_STM32_PWM_CHANNEL_COUNT_VALUE},\n"
"    {\n"
"${MC_STM32_PWM_CHANNEL_CONFIG_ENTRIES}"
"    }\n"
"};\n"
"static constexpr stm32::Stm32HalGpioPinConfig gpio_pins[] = {\n"
"${MC_STM32_GPIO_CONFIG_ENTRIES}"
"};\n"
"static constexpr size_t gpio_pin_count = sizeof(gpio_pins) / sizeof(gpio_pins[0]);\n"
"}\n")

if(TARGET_ARCH_FAMILY STREQUAL "stm32")
    file(WRITE "${MC_GENERATED_DIR}/experimental_platform_hal.h"
"#pragma once\n"
"#include <arm/stm32/hal.h>\n"
"namespace experimental_platform {\n"
"using Hal = stm32::Stm32Hal;\n"
"}\n")
endif()

##################################################################
# DEFINES
##################################################################

add_compile_definitions(USE_NEW_DELEGATE)
add_compile_definitions(MC_BOARD_NAME=\"${MC_BOARD_NAME}\")
add_compile_definitions(MC_BOARD_MCU=\"${MC_BOARD_MCU}\")
add_compile_definitions(MC_FEATURE_SET_NAME=\"${MC_FEATURE_SET_NAME}\")

if(MC_ENABLE_LOGGING)
    add_compile_definitions(LOGGING_ENABLED)
endif()

if(MC_USE_EXPERIMENTAL_RTOS_HAL)
    add_compile_definitions(MC_USE_EXPERIMENTAL_RTOS_HAL)
endif()

if(MC_USE_EXPERIMENTAL_MINCOPTER_RUNTIME)
    add_compile_definitions(MC_USE_EXPERIMENTAL_MINCOPTER_RUNTIME)
endif()

if(MC_GPS_TYPE STREQUAL "AUTO")
    add_compile_definitions(MC_GPS_AUTO)
elseif(MC_GPS_TYPE STREQUAL "UBLOX")
    add_compile_definitions(MC_GPS_UBLOX)
elseif(MC_GPS_TYPE STREQUAL "SIM")
    add_compile_definitions(MC_GPS_SIM)
elseif(MC_GPS_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_GPS_NONE)
else()
    message(FATAL_ERROR "GPS device not supported: ${MC_GPS_TYPE}")
endif()

if(MC_IMU_TYPE STREQUAL "MPU6000")
    add_compile_definitions(MC_IMU_MPU6000)
elseif(MC_IMU_TYPE STREQUAL "MPU6050")
    add_compile_definitions(MC_IMU_MPU6050)
elseif(MC_IMU_TYPE STREQUAL "ICM20948")
    add_compile_definitions(MC_IMU_ICM20948)
elseif(MC_IMU_TYPE STREQUAL "L3G4200D")
    add_compile_definitions(MC_IMU_L3G4200D)
elseif(MC_IMU_TYPE STREQUAL "SIM")
    add_compile_definitions(MC_IMU_SIM)
elseif(MC_IMU_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_IMU_NONE)
else()
    message(FATAL_ERROR "IMU device not supported: ${MC_IMU_TYPE}")
endif()

if(MC_COMP_TYPE STREQUAL "HMC5843")
    add_compile_definitions(MC_COMP_HMC5843)
elseif(MC_COMP_TYPE STREQUAL "ICM20948")
    add_compile_definitions(MC_COMP_ICM20948)
elseif(MC_COMP_TYPE STREQUAL "SIM")
    add_compile_definitions(MC_COMP_SIM)
elseif(MC_COMP_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_COMP_NONE)
else()
    message(FATAL_ERROR "Compass device not supported: ${MC_COMP_TYPE}")
endif()

if(MC_BARO_TYPE STREQUAL "MS5611")
    add_compile_definitions(MC_BARO_MS5611)
elseif(MC_BARO_TYPE STREQUAL "BME280")
    add_compile_definitions(MC_BARO_BME280)
elseif(MC_BARO_TYPE STREQUAL "BMP085")
    add_compile_definitions(MC_BARO_BMP085)
elseif(MC_BARO_TYPE STREQUAL "SIM")
    add_compile_definitions(MC_BARO_SIM)
elseif(MC_BARO_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_BARO_NONE)
else()
    message(FATAL_ERROR "Barometer device not supported: ${MC_BARO_TYPE}")
endif()

if(MC_STORAGE_TYPE STREQUAL "FILE")
    add_compile_definitions(MC_STORAGE_FILE)
elseif(MC_STORAGE_TYPE STREQUAL "DATAFLASH")
    add_compile_definitions(MC_STORAGE_DATAFLASH)
elseif(MC_STORAGE_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_STORAGE_EMPTY)
else()
    message(FATAL_ERROR "Storage device not supported: ${MC_STORAGE_TYPE}")
endif()

if(MC_ADC_TYPE STREQUAL "ADS7844")
    add_compile_definitions(MC_ADC_ADS7844)
elseif(MC_ADC_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_ADC_NONE)
elseif(MC_ADC_TYPE STREQUAL "SIM")
    add_compile_definitions(MC_ADC_SIM)
else()
    message(FATAL_ERROR "ADC device not supported: ${MC_ADC_TYPE}")
endif()

if(MC_STATE_TYPE STREQUAL "NONE")
    add_compile_definitions(MC_STATE_NONE)
elseif(MC_STATE_TYPE STREQUAL "COMPLEMENTARY")
    add_compile_definitions(MC_STATE_COMPLEMENTARY)
elseif(MC_STATE_TYPE STREQUAL "MADGWICK")
    add_compile_definitions(MC_STATE_MADGWICK)
elseif(MC_STATE_TYPE STREQUAL "EKF")
    add_compile_definitions(MC_STATE_EKF)
elseif(MC_STATE_TYPE STREQUAL "SIM")
    add_compile_definitions(MC_STATE_SIM)
else()
    message(FATAL_ERROR "Unsupported state type: ${MC_STATE_TYPE}")
endif()

if(MC_CONTROLLER_TYPE STREQUAL "MPC")
    add_compile_definitions(CONTROLLER_MPC)
elseif(MC_CONTROLLER_TYPE STREQUAL "PID")
    add_compile_definitions(CONTROLLER_PID)
elseif(MC_CONTROLLER_TYPE STREQUAL "LQR")
    add_compile_definitions(CONTROLLER_LQR)
elseif(MC_CONTROLLER_TYPE STREQUAL "CSC")
    add_compile_definitions(CONTROLLER_CSC)
elseif(MC_CONTROLLER_TYPE STREQUAL "NONE")
    add_compile_definitions(CONTROLLER_NONE)
else()
    message(FATAL_ERROR "Unsupported controller type: ${MC_CONTROLLER_TYPE}")
endif()

if(MC_PLANNER_TYPE STREQUAL "WAYPOINT")
    add_compile_definitions(PLANNER_WAYPOINT)
elseif(MC_PLANNER_TYPE STREQUAL "NONE")
    add_compile_definitions(PLANNER_NONE)
else()
    message(FATAL_ERROR "Unsupported planner type: ${MC_PLANNER_TYPE}")
endif()

##################################################################
# INCLUDE DIRECTORIES
##################################################################

list(APPEND MC_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/arch
    ${CMAKE_SOURCE_DIR}/arch/experimental
    ${CMAKE_SOURCE_DIR}/dev
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/AP_Math
    ${CMAKE_SOURCE_DIR}/lib/AP_Common
    ${CMAKE_SOURCE_DIR}/lib/AP_Curve
    ${CMAKE_SOURCE_DIR}/lib/Filter
    ${CMAKE_SOURCE_DIR}/lib/Telemetry
    ${CMAKE_SOURCE_DIR}/dev/experimental
    ${CMAKE_SOURCE_DIR}/experimental
    ${CMAKE_SOURCE_DIR}/mincopter
    ${MC_GENERATED_DIR}
)

##################################################################
# OUTPUT DIRECTORIES
##################################################################

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

##################################################################
# EXPERIMENTAL BUILD
##################################################################

if(MC_EXPERIMENTAL_BUILD)
    file(GLOB_RECURSE MC_EXPERIMENTAL_ARCH_SOURCES
        "${CMAKE_SOURCE_DIR}/arch/experimental/*.cpp"
    )

    set(MC_EXPERIMENTAL_DEV_SOURCES)

    if(MC_IMU_TYPE STREQUAL "MPU6050")
        list(APPEND MC_EXPERIMENTAL_DEV_SOURCES
            ${CMAKE_SOURCE_DIR}/dev/experimental/imu/mpu6050/Mpu6050ImuDevice.cpp
        )
    endif()

    if(MC_COMP_TYPE STREQUAL "HMC5843")
        list(APPEND MC_EXPERIMENTAL_DEV_SOURCES
            ${CMAKE_SOURCE_DIR}/dev/experimental/compass/hmc5843/Hmc5843CompassDevice.cpp
        )
    endif()

    if(MC_BARO_TYPE STREQUAL "BME280")
        list(APPEND MC_EXPERIMENTAL_DEV_SOURCES
            ${CMAKE_SOURCE_DIR}/dev/experimental/barometer/bme280/Bme280BarometerDevice.cpp
        )
    endif()

    if(MC_GPS_TYPE STREQUAL "UBLOX")
        list(APPEND MC_EXPERIMENTAL_DEV_SOURCES
            ${CMAKE_SOURCE_DIR}/dev/experimental/gps/ublox_neo/UbxNeoGpsDevice.cpp
        )
    endif()

    if(MC_STORAGE_TYPE STREQUAL "DATAFLASH")
        list(APPEND MC_EXPERIMENTAL_DEV_SOURCES
            ${CMAKE_SOURCE_DIR}/dev/experimental/storage/at45db321d/At45Db321dStorageDevice.cpp
        )
    endif()

    if(TARGET_ARCH_FAMILY STREQUAL "stm32")
        set(FREERTOS_KERNEL_DIR "${CMAKE_SOURCE_DIR}/arch/experimental/freertos")
        set(FREERTOS_CONFIG_DIR "${MC_GENERATED_DIR}" CACHE PATH "Directory containing FreeRTOSConfig.h")

        set(FREERTOS_KERNEL_SOURCES
            "${FREERTOS_KERNEL_DIR}/tasks.c"
            "${FREERTOS_KERNEL_DIR}/queue.c"
            "${FREERTOS_KERNEL_DIR}/list.c"
            "${FREERTOS_KERNEL_DIR}/timers.c"
            "${FREERTOS_KERNEL_DIR}/event_groups.c"
            "${FREERTOS_KERNEL_DIR}/stream_buffer.c"
            "${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F/port.c"
            "${FREERTOS_KERNEL_DIR}/portable/MemMang/heap_4.c"
        )

        add_library(freertos_config INTERFACE)
        target_include_directories(freertos_config INTERFACE
            "${FREERTOS_CONFIG_DIR}"
        )

        add_library(freertos_kernel STATIC ${FREERTOS_KERNEL_SOURCES})
        target_include_directories(freertos_kernel PUBLIC
            "${FREERTOS_KERNEL_DIR}/include"
            "${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F"
            "${CMAKE_SOURCE_DIR}/arch/arm"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
        )
        target_link_libraries(freertos_kernel PUBLIC freertos_config)
    endif()

    add_library(mc-experimental-arch STATIC ${MC_EXPERIMENTAL_ARCH_SOURCES})
    target_include_directories(mc-experimental-arch PUBLIC ${MC_INCLUDE_DIRS})

    if(TARGET_ARCH_FAMILY STREQUAL "stm32")
        target_link_libraries(mc-experimental-arch PUBLIC freertos_kernel)
    endif()

    add_library(mc-experimental-dev STATIC ${MC_EXPERIMENTAL_DEV_SOURCES})
    target_include_directories(mc-experimental-dev PUBLIC ${MC_INCLUDE_DIRS})
    target_link_libraries(mc-experimental-dev PUBLIC mc-experimental-arch)

    add_executable(mincopter
        experimental/mincopter/main.cpp
        experimental/mincopter/init.cpp
        experimental/mincopter/ExperimentalRuntime.cpp
        experimental/mincopter/RuntimeConfig.cpp
        experimental/mincopter/RuntimeAssembly.cpp
        experimental/mincopter/EstimatorTask.cpp
        experimental/mincopter/HeartbeatTask.cpp
        dev/experimental/led/LedDevice.cpp
    )

    target_include_directories(mincopter PUBLIC ${MC_INCLUDE_DIRS})
    target_link_libraries(mincopter
        mc-experimental-arch
        mc-experimental-dev
    )

    if(TARGET_ARCH_FAMILY STREQUAL "stm32")
        file(GLOB STM32_HAL_LIBS
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
        )

        add_library(stm32_platform STATIC
            ${STM32_HAL_LIBS}
            "${CMAKE_SOURCE_DIR}/arch/arm/startup_stm32f407vgtx.s"
            "${CMAKE_SOURCE_DIR}/arch/arm/syscalls.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/sysmem.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/stm32f4xx_it.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/system_stm32f4xx.c"
        )

        target_include_directories(stm32_platform PUBLIC
            ${MC_INCLUDE_DIRS}
            "${CMAKE_SOURCE_DIR}/arch/experimental/freertos/include"
            ### TODO This should not be hardcoded - we need to select based on our STM32 board
            "${CMAKE_SOURCE_DIR}/arch/experimental/freertos/portable/GCC/ARM_CM4F"
            "${CMAKE_SOURCE_DIR}/arch/arm"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
        )

        target_include_directories(mc-experimental-arch PUBLIC
            ${MC_INCLUDE_DIRS}
            "${CMAKE_SOURCE_DIR}/arch/arm"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
        )

        target_link_libraries(mincopter
            -Wl,--whole-archive
            stm32_platform
            -Wl,--no-whole-archive
        )
    endif()

else()
    ##################################################################
    # LEGACY BUILD
    ##################################################################

    list(APPEND MC_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/arch/AP_HAL
        ${CMAKE_SOURCE_DIR}/arch/avr
        ${CMAKE_SOURCE_DIR}/arch/arm/stm32
        ${CMAKE_SOURCE_DIR}/arch/linux
        ${CMAKE_SOURCE_DIR}/arch/linux/generic
        ${CMAKE_SOURCE_DIR}/arch/linux/rpi
        ${CMAKE_SOURCE_DIR}/arch/linux/beaglebone
        ${CMAKE_SOURCE_DIR}/arch/AP_Progmem
        ${CMAKE_SOURCE_DIR}/arch/AP_Scheduler
        ${CMAKE_SOURCE_DIR}/lib/AP_Common
        ${CMAKE_SOURCE_DIR}/lib/AP_Curve
        ${CMAKE_SOURCE_DIR}/lib/AP_Math
        ${CMAKE_SOURCE_DIR}/lib/Filter
        ${CMAKE_SOURCE_DIR}/lib/Telemetry
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/state/ekf
        ${CMAKE_SOURCE_DIR}/state
        ${CMAKE_SOURCE_DIR}/dev/AP_ADC
        ${CMAKE_SOURCE_DIR}/dev/AP_ADC_AnalogSource
        ${CMAKE_SOURCE_DIR}/dev/AP_Baro
        ${CMAKE_SOURCE_DIR}/dev/AP_BattMonitor
        ${CMAKE_SOURCE_DIR}/dev/AP_Compass
        ${CMAKE_SOURCE_DIR}/dev/AP_Declination
        ${CMAKE_SOURCE_DIR}/dev/AP_GPS
        ${CMAKE_SOURCE_DIR}/dev/AP_InertialSensor
        ${CMAKE_SOURCE_DIR}/dev/DataFlash
        ${CMAKE_SOURCE_DIR}/control/include/controllers/AC_PID
        ${CMAKE_SOURCE_DIR}/control/include/controllers
        ${CMAKE_SOURCE_DIR}/control/include/planners/AC_Fence
        ${CMAKE_SOURCE_DIR}/control/include/planners/AC_WPNav
        ${CMAKE_SOURCE_DIR}/control/include/planners
        ${CMAKE_SOURCE_DIR}/sim
    )

    file(GLOB_RECURSE MC_ARCH_COMMON
        "${CMAKE_SOURCE_DIR}/arch/AP_HAL/*.cpp"
        "${CMAKE_SOURCE_DIR}/arch/AP_HAL/utility/*.cpp"
        "${CMAKE_SOURCE_DIR}/arch/AP_HAL/*.c"
        "${CMAKE_SOURCE_DIR}/arch/AP_Scheduler/*.cpp"
        "${CMAKE_SOURCE_DIR}/arch/AP_Progmem/*cpp"
    )

    if(${TARGET_ARCH_FAMILY} STREQUAL "generic")
        file(GLOB MC_LIBS_ARCH
            "${CMAKE_SOURCE_DIR}/arch/linux/generic/*.cpp"
            "${CMAKE_SOURCE_DIR}/arch/linux/*.cpp"
        )
    elseif(${TARGET_ARCH_FAMILY} STREQUAL "avrmega")
        file(GLOB MC_LIBS_ARCH
            "${CMAKE_SOURCE_DIR}/arch/avr/mega/*.cpp"
            "${CMAKE_SOURCE_DIR}/arch/avr/mega/*.c"
            "${CMAKE_SOURCE_DIR}/arch/avr/mega/utility/*.c"
            "${CMAKE_SOURCE_DIR}/arch/avr/mega/utility/*.cpp"
        )
    elseif(${TARGET_ARCH_FAMILY} STREQUAL "avrdx")
        file(GLOB MC_LIBS_ARCH
            "${CMAKE_SOURCE_DIR}/arch/avr/avrdx/*.cpp"
            "${CMAKE_SOURCE_DIR}/arch/avr/avrdx/*.c"
            "${CMAKE_SOURCE_DIR}/arch/avr/avrdx/utility/*.c"
            "${CMAKE_SOURCE_DIR}/arch/avr/avrdx/utility/*.cpp"
        )
    elseif(${TARGET_ARCH_FAMILY} STREQUAL "rpi")
        file(GLOB MC_LIBS_ARCH
            "${CMAKE_SOURCE_DIR}/arch/linux/rpi/*.cpp"
            "${CMAKE_SOURCE_DIR}/arch/linux/*.cpp"
        )
    elseif(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
        file(GLOB STM32_HAL_LIBS
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
        )

        file(GLOB MC_LIBS_ARCH
            "${CMAKE_SOURCE_DIR}/arch/arm/stm32/*.cpp"
            ${STM32_HAL_LIBS}
        )
    else()
        message(FATAL_ERROR "Unsupported legacy TARGET_ARCH_FAMILY: ${TARGET_ARCH_FAMILY}")
    endif()

    add_library(mc-arch STATIC ${MC_LIBS_ARCH} ${MC_ARCH_COMMON})
    target_include_directories(mc-arch PUBLIC ${MC_INCLUDE_DIRS})

    if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
        target_include_directories(mc-arch PUBLIC
            "${CMAKE_SOURCE_DIR}/arch/arm"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
        )
    endif()

    add_subdirectory(${CMAKE_SOURCE_DIR}/control)
    add_subdirectory(${CMAKE_SOURCE_DIR}/state)
    add_subdirectory(${CMAKE_SOURCE_DIR}/dev)
    add_subdirectory(${CMAKE_SOURCE_DIR}/lib)

    add_executable(mincopter
        mincopter/mincopter.cpp
        mincopter/init.cpp
        mincopter/mcinstance.cpp
    )

    target_link_libraries(mincopter
        mc-arch
        mc-dev
        mc-control
        mc-state
        mc-common
    )

    if(${TARGET_ARCH_FAMILY} STREQUAL "stm32")
        add_library(stm32_static STATIC
            "${CMAKE_SOURCE_DIR}/arch/arm/startup_stm32f407vgtx.s"
            "${CMAKE_SOURCE_DIR}/arch/arm/syscalls.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/sysmem.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/stm32f4xx_it.c"
            "${CMAKE_SOURCE_DIR}/arch/arm/system_stm32f4xx.c"
        )

        target_include_directories(stm32_static PUBLIC
            "${CMAKE_SOURCE_DIR}/arch/arm"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc"
            "${CMAKE_SOURCE_DIR}/arch/arm/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
        )

        target_link_libraries(mincopter
            -Wl,--whole-archive
            stm32_static
            -Wl,--no-whole-archive
        )
    endif()

    target_include_directories(mincopter PUBLIC ${MC_INCLUDE_DIRS})

    add_subdirectory(test)
endif()
