
#include <arch/arm/stm32/UARTDriver.h>

/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

extern const AP_HAL::HAL& hal;

using namespace stm32;

STM32UARTDriver::STM32UARTDriver(bool default_console, stm32::UART utype)
	: _utype(utype)
{
}

void STM32UARTDriver::set_device_path(const char *path)
{
	// TODO
	return;
}

void STM32UARTDriver::begin(uint32_t b) 
{
    begin(b, 0, 0);
}

void STM32UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Set correct UARTs
	//
	// NOTE UART4 and UART5 are UART not USART
	//
	if (_utype == UART::MC_USART2 ) {
		__HAL_RCC_USART2_CLK_ENABLE();
		mc_uart.Instance = USART2;

		// TODO Add GPIO CLK enables

		// TODO Make configurable
		// 3. Setup USART2 TX/RX Pins - PA2, PA3
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	} else if (_utype == UART::MC_USART3 ) {
		__HAL_RCC_USART3_CLK_ENABLE();
		mc_uart.Instance = USART3;

		// 4. Setup USART3 TX/RX Pins - PB10, PB11
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	} else if (_utype == UART::MC_USART4 ) {
		__HAL_RCC_USART3_CLK_ENABLE();
		mc_uart.Instance = UART4;
		// TODO Add GPIO TX/RX setup
	} else if (_utype == UART::MC_USART5 ) {
		__HAL_RCC_USART3_CLK_ENABLE();
		mc_uart.Instance = UART5;
		// TODO Add GPIO TX/RX setup
	}

	// NOTE Should add functionality to default to 115200
    mc_uart.Init.BaudRate = b;
    mc_uart.Init.WordLength = UART_WORDLENGTH_8B;
    mc_uart.Init.StopBits = UART_STOPBITS_1;
    mc_uart.Init.Parity = UART_PARITY_NONE;
    mc_uart.Init.Mode = UART_MODE_TX_RX;
    mc_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    mc_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&mc_uart) != HAL_OK)
    {
        //Error_Handler();
		//TODO Add hal panic
    }

	return;
}

void STM32UARTDriver::end() 
{
	// TODO
}


void STM32UARTDriver::flush() 
{
	// TODO
}

bool STM32UARTDriver::is_initialized() 
{
	// TODO
	return false;
}

void STM32UARTDriver::set_blocking_writes(bool blocking) 
{
	// TODO
	return;
}

bool STM32UARTDriver::tx_pending() 
{ 
	// TODO
	return false;
}

int16_t STM32UARTDriver::available() 
{ 
	// TODO
	return 0;
}

int16_t STM32UARTDriver::txspace() 
{ 
	// TODO
	return 0;
}

int16_t STM32UARTDriver::read() 
{ 
	uint8_t _rx_byte;
	// NOTE This returns a single byte - we should be 'reading' the UART transmission and storing
	// in a separate buffer asynchronously. This function is to retrieve from the UART buffer
	HAL_UART_Receive(&mc_uart, &_rx_byte, 1, HAL_MAX_DELAY);

	return _rx_byte;
}

size_t STM32UARTDriver::write(uint8_t c) 
{
	// NOTE Need to do blocking write if we pass address of argument
	HAL_UART_Transmit(&mc_uart, &c, 1, HAL_MAX_DELAY);

    return 1;
}

size_t STM32UARTDriver::write(const uint8_t *buffer, size_t size)
{
	// Use HAL UART interface
	
	// TODO Check return status
	HAL_UART_Transmit(&mc_uart, buffer, size, HAL_MAX_DELAY);

    return size;
}

