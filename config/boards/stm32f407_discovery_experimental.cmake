#
# Example board wiring definition for the experimental RTOS runtime.
#
# This file answers:
# - which MCU/board this is
# - how logical roles map to concrete peripherals
# - which pins are used for each routed signal
# - per-device wiring details such as addresses, CS pins, and DRDY pins
#
# It does not answer:
# - which device types are enabled in the build
# - which estimator/controller/planner modules are selected
#

set(MC_BOARD_NAME "stm32f407_discovery_experimental")
set(MC_BOARD_FAMILY "stm32")
set(MC_BOARD_MCU "STM32F407xx")

## LEDs / generic board GPIO
set(MC_STATUS_LED0_PIN "PD12")
set(MC_STATUS_LED1_PIN "PD13")
set(MC_STATUS_LED2_PIN "PD14")
set(MC_STATUS_LED3_PIN "PD15")

## Console UART role
set(MC_ROLE_CONSOLE_UART_INSTANCE "USART2")
set(MC_ROLE_CONSOLE_UART_INDEX 0)
set(MC_ROLE_CONSOLE_UART_TX_PIN "PA2")
set(MC_ROLE_CONSOLE_UART_RX_PIN "PA3")
set(MC_ROLE_CONSOLE_UART_AF "GPIO_AF7_USART2")
set(MC_ROLE_CONSOLE_UART_BAUD 115200)

## GPS UART role
set(MC_ROLE_GPS_UART_INSTANCE "USART1")
set(MC_ROLE_GPS_UART_INDEX 1)
set(MC_ROLE_GPS_UART_TX_PIN "PB6")
set(MC_ROLE_GPS_UART_RX_PIN "PB7")
set(MC_ROLE_GPS_UART_AF "GPIO_AF7_USART1")
set(MC_ROLE_GPS_UART_BAUD 9600)

## Telemetry UART role
set(MC_ROLE_TELEMETRY_UART_INSTANCE "USART3")
set(MC_ROLE_TELEMETRY_UART_INDEX 2)
set(MC_ROLE_TELEMETRY_UART_TX_PIN "PD8")
set(MC_ROLE_TELEMETRY_UART_RX_PIN "PD9")
set(MC_ROLE_TELEMETRY_UART_AF "GPIO_AF7_USART3")
set(MC_ROLE_TELEMETRY_UART_BAUD 57600)

## IMU I2C role
set(MC_ROLE_IMU_I2C_INSTANCE "I2C1")
set(MC_ROLE_IMU_I2C_INDEX 0)
set(MC_ROLE_IMU_I2C_SCL_PIN "PB8")
set(MC_ROLE_IMU_I2C_SDA_PIN "PB9")
set(MC_ROLE_IMU_I2C_AF "GPIO_AF4_I2C1")

## Compass I2C role
## For this example the compass shares the same physical I2C bus as the IMU.
set(MC_ROLE_COMPASS_I2C_INSTANCE "I2C1")
set(MC_ROLE_COMPASS_I2C_INDEX 0)
set(MC_ROLE_COMPASS_I2C_SCL_PIN "PB8")
set(MC_ROLE_COMPASS_I2C_SDA_PIN "PB9")
set(MC_ROLE_COMPASS_I2C_AF "GPIO_AF4_I2C1")

## Barometer I2C role
## For this example the barometer also shares I2C1.
set(MC_ROLE_BARO_I2C_INSTANCE "I2C1")
set(MC_ROLE_BARO_I2C_INDEX 0)
set(MC_ROLE_BARO_I2C_SCL_PIN "PB8")
set(MC_ROLE_BARO_I2C_SDA_PIN "PB9")
set(MC_ROLE_BARO_I2C_AF "GPIO_AF4_I2C1")

## Storage SPI role
set(MC_ROLE_STORAGE_SPI_INSTANCE "SPI2")
set(MC_ROLE_STORAGE_SPI_INDEX 1)
set(MC_ROLE_STORAGE_SPI_SCK_PIN "PB13")
set(MC_ROLE_STORAGE_SPI_MISO_PIN "PB14")
set(MC_ROLE_STORAGE_SPI_MOSI_PIN "PB15")
set(MC_ROLE_STORAGE_SPI_AF "GPIO_AF5_SPI2")

## Device-local wiring
set(MC_ROLE_BATTERY_ADC_INDEX 0)
set(MC_ROLE_BATTERY_ADC_PIN "PA1")
set(MC_ROLE_BATTERY_ADC_CHANNEL "ADC_CHANNEL_1")

set(MC_IMU_BUS_ROLE "imu_i2c")
set(MC_IMU_I2C_ADDRESS "0x68")
set(MC_IMU_DRDY_PIN "PC4")

## ICM20948 settings
set(MC_COMPASS_BUS_ROLE "compass_i2c")
set(MC_COMPASS_I2C_ADDRESS "0x69")
#set(MC_COMPASS_DRDY_PIN "PC5")

set(MC_BARO_BUS_ROLE "baro_i2c")
set(MC_BARO_I2C_ADDRESS "0x76")

set(MC_GPS_UART_ROLE "gps_uart")
set(MC_GPS_BAUD "9600")

set(MC_STORAGE_SPI_ROLE "storage_spi")
set(MC_STORAGE_CS_PIN "PB12")

## PWM role for motors
set(MC_ROLE_MOTOR_PWM_TIMER "TIM3")
set(MC_ROLE_MOTOR_PWM_AF "GPIO_AF2_TIM3")
set(MC_MOTOR1_PWM_PIN "PC6")
set(MC_MOTOR1_PWM_CHANNEL 1)
set(MC_MOTOR2_PWM_PIN "PC7")
set(MC_MOTOR2_PWM_CHANNEL 2)
set(MC_MOTOR3_PWM_PIN "PC8")
set(MC_MOTOR3_PWM_CHANNEL 3)
set(MC_MOTOR4_PWM_PIN "PC9")
set(MC_MOTOR4_PWM_CHANNEL 4)
