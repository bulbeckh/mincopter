
#include <AP_HAL/AP_HAL.h>

#include <arch/arm/stm32/HAL_STM32_Class.h>
#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>
#include <arch/arm/stm32/AP_HAL_STM32_Private.h>

#include "stm32f4xx_hal.h"

static stm32::STM32Semaphore  i2cSemaphore;
static stm32::STM32Scheduler schedulerInstance;
static stm32::STM32Util utilInstance;

// TODO NOTE What is the bool arg here?
static stm32::STM32UARTDriver uartADriver(true,  stm32::UART::MC_USART2);
static stm32::STM32UARTDriver uartBDriver(false, stm32::UART::MC_USART3);
static stm32::STM32UARTDriver uartCDriver(false, stm32::UART::MC_USART4);
static stm32::STM32UARTDriver uartDDriver(false, stm32::UART::MC_USART5);

static stm32::STM32I2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static stm32::STM32SPIDeviceManager spiDeviceManager;
static stm32::STM32AnalogIn analogIn;
static stm32::STM32GPIO gpioDriver;
static stm32::STM32RCInput rcinDriver;
static stm32::STM32RCOutput rcoutDriver;

// Forward Declaration of STM32 HAL Clock Configuration function
static void SystemClock_Config(void);
void Error_Handler(void);

/* STM32 Wiring NOTE later this wiring will be configurable via a configuration file but for now we put explicitly
 *
 *
 * I2C
 * - BME280 (Barometer)
 * - MPU6050 (IMU - Accel/Gyro)
 *
 * SPI
 * - ICM20948 (Compass)
 *
 * UART
 * - UBLOX (GPS)
 * - Console (primary)
 * - Telemetry (alternate UART)
 * - Unused but still initialised (alternate UART)
 *
 * GPIO Pins
 * - x1 CS for ICM
 * - x2 LEDs (onboard for STM32F407G-DISC1)
 *
 *
 * GPIO Configuration
 * - SPI_SS PA4
 * - SPI_SCK PA5
 * - SPI_MISO PA6
 * - SPI_MOSI PA7
 *
 * - USART2_TX PA2
 * - USART2_RX PA3
 *
 * - USART3_TX PB10
 * - USART3_RX PB11
 *
 * - I2C1_SCL PB6
 * - I2C1_SDA PB7
 *
 *
 */

HAL_STM32::HAL_STM32() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        NULL,   /* No Storage Driver for STM32 */
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
		NULL)  /* No Sim Driver for STM32 */
{}

void HAL_STM32::init(int argc,char* const argv[]) const 
{
	// TODO NOTE Won't need this if we switch to STM32 LL
	// Initialise STM32 HAL functions
	HAL_Init();

	// Configure STM32 clock tree
	SystemClock_Config();

    scheduler->init(NULL);

	// Initialise GPIO pins first as other peripherals depend on GPIO configuration
	gpio->init();

	// Turn on LED to indicate that we've reached GPIO init
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

	// TODO Really, we should initialise the UART which was specified as the console/default UART rather than uartA explicitly
	// Start the console UART so we can log HAL initialisation messages from all other classes
    uartA->begin(115200);




    i2c->begin();

    spi->init(NULL);


}

static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* STM32 Clock Tree configured as follows:
	 *
	 *
	 * Current board uses an 8MHz external crystal
	 *
	 * PLLM = 8 -> 8MHz / 8 = 1MHz
	 * PLLN = 336 -> 1MHz * 336 = 336MHz
	 * PLLP = DIV2 -> 336MHz / 2 = 168MHz
	 *
	 * This is SYSCLK = 168MHz
	 *
	 * AHB Prescaler = 1 -> HCLK = 168MHz / 1 = 168MHz
	 *
	 * APB1 Prescaler = 4 -> PLCK1 = 168MHz / 4 = 42MHz (84MHz actual)
	 * APB2 Prescaler = 4 -> PLCK2 = 168MHz / 2 = 84MHz (168MHz actual)
	 *
	 * APB1 Peripherals
	 * - USART2,3,4,5
	 * - SPI2,3
	 * - I2C1,2,3
	 * - TIM2-7, TIM12-14
	 * - DAC
	 *
	 * APB2 Peripherals
	 * - USART1,6
	 * - SPI1,4,5,6
	 * - ADC1,2,3
	 * - EXTI (interrupts)
	 * - TIM1,8,9-11
	 *
	 */

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}

	return;
}

void Error_Handler(void)
{
	// TODO Replace calls to Error_Handler() with hal.scheduler->panic() throughout the HAL initialisation

	__disable_irq();
	while(1) {}
}

// STM32 HAL Instance;
const HAL_STM32 AP_HAL_STM32;


