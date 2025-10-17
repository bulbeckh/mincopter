
// Motor (simulation) test

#include <iostream>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include "stm32f4xx_hal.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/* In this core test, we need to test the HAL read/write over SPI, I2C, and UART */

int main(void)
{

	AP_HAL_STM32.init(0, NULL);

	// Stop the callback
	hal.scheduler->suspend_timer_procs();

	hal.uartA->write((uint8_t*)"newstring\n", 10);

	// I2C Test - Get the device ID (0x68) from the MPU6050
	
	uint8_t mpu6050_id = 0x00;
	hal.i2c->readRegister(0x68, 0x75, &mpu6050_id);

	hal.uartA->printf("ID0x%x\r\n", mpu6050_id);

	// SPI Test - Get the device ID (0xEA) from the ICM20948
	/*
	auto spidev = hal.spi->device(AP_HAL::SPIDevice::SPIDevice_ICM20948);
	spidev->cs_assert();

	hal.scheduler->delay(1);

	uint8_t _tx[16] = {0x7F, 0x00};

	// Write 2 bytes - register bank 0 select
	spidev->transfer(_tx, 2);

	// Reset the device
	_tx[0] = 0x06;
	_tx[1] = 0x80;
	spidev->transfer(_tx, 2);

	hal.scheduler->delay(100);

	// Read the device id
	uint8_t _rx[16] = {0x00};
	_tx[0] = 0x80 | 0x00;
	_tx[1] = 0x00;
	spidev->transaction(_tx, _rx, 2);

	spidev->cs_release();
	*/

	auto spi_bme = hal.spi->device(AP_HAL::SPIDevice::SPIDevice_BME280);
	spi_bme->cs_assert();

	uint8_t _tx[16];
	uint8_t _rx[16];
	_tx[0] = 0x80 | 0xD0;

	// Read ID
	spi_bme->transaction(_tx, _rx, 2);

	spi_bme->cs_release();

	hal.uartA->printf("BM 0x%x,0x%x\r\n",_rx[0],_rx[1]);

	// UART Test - Get the device ID of the GPS module
	
	// Manually start uartB
	hal.uartB->begin(9600);
	hal.uartA->printf("UD");
	uint8_t rx_buf[16];
	for (uint8_t i=0;i<16;i++) {
		rx_buf[i] = 0xff;
		rx_buf[i] = hal.uartB->read();
		hal.uartA->printf("0x%x,", rx_buf[i]);
	}
	hal.uartA->printf("\r\n");
	//hal.uartA->printf("UD 0x%x,0x%x,0x%x,0x%x\r\n",rx_buf[0],rx_buf[1], rx_buf[2], rx_buf[3]);

	// Switch on ORANGE led if all true
	if (mpu6050_id == 0x68 && _rx[1]==0x60) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	}

	// loop
	volatile uint32_t counter=0;
	while(true) {
		counter +=1;
	}

	return 0;

}


