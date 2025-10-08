
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
	// TODO
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
	if (_utype == UART::MC_USART2 ) {
		__HAL_RCC_USART2_CLK_ENABLE();
		mc_uart.Instance = USART2;
	} else if (_utype == UART::MC_USART3 ) {
		__HAL_RCC_USART3_CLK_ENABLE();
		mc_uart.Instance = USART3;
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
	// TODO
	return 0;
}

size_t STM32UARTDriver::write(uint8_t c) 
{ 
	// TODO
    return 1;
}

size_t STM32UARTDriver::write(const uint8_t *buffer, size_t size)
{
	// TODO
    return 0;
}

void STM32UARTDriver::_timer_tick(void)
{
	// TODO
}
