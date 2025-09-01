
#include "AP_Compass_ICM20948.h"

//#include <stdio.h>

extern const AP_HAL::HAL& hal;

// ICM20948 Definitions
#define SPI_READ_BIT 0x80
#define ICM20948_REG_BANK_SEL 0x7F

// Bank 0 Registers (not exhaustive)
#define ICM20948_WHOAMI 0x00
#define ICM20948_LP_CONFIG 0x05
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_PWR_MGMT_2 0x07
#define ICM20948_USER_CTRL 0x03
#define ICM20948_ACCEL_XOUT_H 0x2D
#define ICM20948_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_INT_STATUS_1 0x1A

// Bank 1 Registers (not exhaustive)

// Bank 2 Registers (not exhaustive)
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG 0x14
#define ICM20948_GYRO_SMPLRT_DIV 0x00
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_GYRO_CONFIG_2 0x02

// Bank 3 Registers (not exhaustive)
#define ICM20948_I2C_MST_ODR_CONFIG 0x00
#define ICM20948_I2C_MST_CTRL 0x01
#define ICM20948_I2C_SLV0_ADDR 0x03
#define ICM20948_I2C_SLV0_REG 0x04
#define ICM20948_I2C_SLV0_CTRL 0x05
#define ICM20948_I2C_SLV0_DO 0x06


AP_Compass_ICM20948::AP_Compass_ICM20948()
	: Compass()
{
}

bool AP_Compass_ICM20948::init()
{
	// Retrieve SPI device
	_spi = hal.spi->device(AP_HAL::SPIDevice_ICM20948);
	if (_spi==NULL) hal.scheduler->panic("ICM Init: no SPI device\n");

	// TODO For now, initialise both gyro/accel and magnetometer but change to separate init functions soon
	
	// Reset device after power-on
	_icm_reset();
	
	uint8_t _pkt[2];

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Remove from sleep mode and set clock to AUTO
	_pkt[0] = ICM20948_PWR_MGMT_1;
	_pkt[1] = 0x01;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	_icm_init_gyro_accel();

	_icm_init_magnetometer();

	// Prepare the state for a magnetometer read
	_icm_prep_magnetometer_measure();

	// Assign magnetometer as healthy
	_healthy = true;

	// Do read to get first value
	read();

	return 0;
}

bool AP_Compass_ICM20948::read()
{
	//printf("AP Compass - read called\n");

	uint8_t _pkt[2];

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// TODO Soon these two will be split into separate functions
	
	// Accel/Gyro read
	uint8_t _raw_read[13];
	uint8_t _tx_buf[13];
	for (int i=0;i<13;i++) _tx_buf[i]=0;
	_tx_buf[0] = ICM20948_ACCEL_XOUT_H | SPI_READ_BIT;

	// Mag read
	uint8_t _mag_raw_read[7];
	uint8_t _mag_tx_buf[7];
	_mag_tx_buf[0] = ICM20948_EXT_SLV_SENS_DATA_00 | SPI_READ_BIT;

	// NOTE This will halt until data is ready
	while(_poll_data_ready());

	// Send 13 bytes (read 12)
	_spi->transaction(_tx_buf, _raw_read, 13);
	//uint8_t status = icm20948_read(_tx_buf, _raw_read, 13);
	//if (status!=0) printf("BAD READ\n");

	_spi->transaction(_mag_tx_buf, _mag_raw_read, 7);
	//uint8_t mag_status = icm20948_read(_mag_tx_buf, _mag_raw_read, 7);
	//if (mag_status!=0) printf("BAD MAG READ\n");

	// Accel & Gyro
	int16_t accel_xout = (_raw_read[1] << 8 ) | (_raw_read[2] );
	int16_t accel_yout = (_raw_read[3] << 8 ) | (_raw_read[4] );
	int16_t accel_zout = (_raw_read[5] << 8 ) | (_raw_read[6] );

	int16_t gyro_xout = (_raw_read[7] << 8 ) | (_raw_read[8] );
	int16_t gyro_yout = (_raw_read[9] << 8 ) | (_raw_read[10] );
	int16_t gyro_zout = (_raw_read[11] << 8 ) | (_raw_read[12] );

	float accel_x = accel_xout / 16384.0f;
	float accel_y = accel_yout / 16384.0f;
	float accel_z = accel_zout / 16384.0f;

	float gyro_x = gyro_xout / 131.0f;
	float gyro_y = gyro_yout / 131.0f;
	float gyro_z = gyro_zout / 131.0f;

	// Mag
	int16_t mag_x = (_mag_raw_read[2]<<8) | (_mag_raw_read[1]);
	int16_t mag_y = (_mag_raw_read[4]<<8) | (_mag_raw_read[3]);
	int16_t mag_z = (_mag_raw_read[6]<<8) | (_mag_raw_read[5]);

	// TODO Make the scaling configurable
	// TODO Make compass offset configurable
	// Update compass _field vector with compensated reading (scaled and then offset)
	// NOTE Static offsets computed by measurement
	_field.x = mag_x*0.15f +20.175;
	_field.y = mag_y*0.15f -4.35;
	_field.z = mag_z*0.15f +2.235;

	// Rotate mag readings into ENU frame
	// NOTE This is faster than doing a matrix multiplication and still maintains rotation
	// NOTE Rotation comes from kalman_ins.py script
	float _temp_x = _field.x;
	_field.x = _field.y;
	_field.y = _temp_x;
	_field.z = -1*_field.z;

	// Magnetometer reading is valid
	_healthy = true;

	return true;
}

void AP_Compass_ICM20948::accumulate()
{
	/* TODO Implement */
	return;
}

uint8_t AP_Compass_ICM20948::_icm_reset()
{
	uint8_t _pkt[2];

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Reset device
	_pkt[0] = ICM20948_PWR_MGMT_1;
	_pkt[1] = 0x80;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// TODO Do we really need to delay this long?
	// Delay 100ms
	hal.scheduler->delay(100);
	//time_sleep(1e-1);

	return 0;
}

uint8_t AP_Compass_ICM20948::_icm_init_gyro_accel()
{
	uint8_t _pkt[2];

	// Select bank 2
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x20;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Accel config
	_pkt[0] = ICM20948_ACCEL_CONFIG;
	// +-2g, accel DLPF enabled, RATE=1125/(1+ACCEL_SMPLRT_DIV)=1125/(1+7)=~140Hz
	_pkt[1] = 0x29;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Accel scaling (ACCEL_SMPLRT_DIV=7)
	_pkt[0] = ICM20948_ACCEL_SMPLRT_DIV_1;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);
	_pkt[0] = ICM20948_ACCEL_SMPLRT_DIV_2;
	_pkt[1] = 0x07;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Gyro config (+-250dps, gyro DLPF enabled)
	_pkt[0] = ICM20948_GYRO_CONFIG_1;
	_pkt[1] = 0x29;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Gyro scaling (GYRO_SMPLRT_DIV=7)
	_pkt[0] = ICM20948_GYRO_SMPLRT_DIV;
	_pkt[1] = 0x07;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	return 0;
}

uint8_t AP_Compass_ICM20948::_icm_prep_magnetometer_measure()
{
	uint8_t _pkt[2];

	/* 1. Set the magnetometer to update at 100Hz */

	// Select bank 3
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x30;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Configure I2C external sensor
	_pkt[0] = ICM20948_I2C_SLV0_ADDR;
	_pkt[1] = 0x0C; // Configure for write
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Device ID Register
	_pkt[0] = ICM20948_I2C_SLV0_REG;
	_pkt[1] = 0x31; // CNTL2 register
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Set continuous mode 4
	_pkt[0] = ICM20948_I2C_SLV0_DO;
	_pkt[1] = 0x08;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Write configuration to AK0xx
	_pkt[0] = ICM20948_I2C_SLV0_CTRL;
	_pkt[1] = 0x81;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Delay 10ms
	hal.scheduler->delay(10);
	//time_sleep(0.1f);

	/* 2. Set the ICM to do the I2C transaction periodically via SLV0 */

	// Set to read 
	_pkt[0] = ICM20948_I2C_SLV0_ADDR;
	_pkt[1] = 0x8C; // Configure for read
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Point to data register
	_pkt[0] = ICM20948_I2C_SLV0_REG;
	_pkt[1] = 0x11; // Magnetometer X High
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Set to read 
	_pkt[0] = ICM20948_I2C_SLV0_CTRL;
	_pkt[1] = 0x88; // Read 8 bytes at sample frequency
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	hal.scheduler->delay(10);
	//time_sleep(0.1f);

	return 0;
}


uint8_t AP_Compass_ICM20948::_icm_init_magnetometer()
{
	uint8_t _pkt[2];

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Sync I2C Magnetometer reads with accelerometer
	_pkt[0] = ICM20948_LP_CONFIG;
	_pkt[1] = (0x01 << 6);
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Set I2C_MST_EN
	_pkt[0] = ICM20948_USER_CTRL;
	_pkt[1] = (0x01 << 5);
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	/* 1. Setup clock for I2C Master */

	// Select bank 3
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x30;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Configure ICM I2C read ODR
	_pkt[0] = ICM20948_I2C_MST_ODR_CONFIG;
	_pkt[1] = 0x03;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Configure I2C clock
	_pkt[0] = ICM20948_I2C_MST_CTRL;
	_pkt[1] = 0x07; // Formerly 0x08
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	/* 2. Write configuration registers to magnetometer */

	// Configure I2C external sensor
	_pkt[0] = ICM20948_I2C_SLV0_ADDR;
	_pkt[1] = 0x0C; // write
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Configure CNTL2 register
	_pkt[0] = ICM20948_I2C_SLV0_REG;
	_pkt[1] = 0x31; // CNTL2 Register for magnetometer
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Configure continuous measurement
	_pkt[0] = ICM20948_I2C_SLV0_DO;
	_pkt[1] = 0x01; // Changed previously 0x08
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Write configuration to AK0xx
	_pkt[0] = ICM20948_I2C_SLV0_CTRL;
	_pkt[1] = 0x81;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	hal.scheduler->delay(100);
	//time_sleep(0.1f);
	
	return 0;
}

uint8_t AP_Compass_ICM20948::_icm_whoami()
{
	uint8_t _pkt[2];

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	uint8_t tx_buf[2] = {ICM20948_WHOAMI | SPI_READ_BIT, 0x00};
	uint8_t rx_buf[2] = {0};
	_spi->transaction(tx_buf, rx_buf, 2);
	//spi_xfer(_pi_ref, _handle, tx_buf, rx_buf, 2);
	
	hal.scheduler->delay(10);

	return rx_buf[1];
}

uint8_t AP_Compass_ICM20948::_mgt_whoami()
{
	uint8_t _pkt[2];

	// Select bank 3
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x30;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Configure I2C external sensor
	_pkt[0] = ICM20948_I2C_SLV0_ADDR;
	_pkt[1] = 0x8C; // Configure for read
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Device ID Register
	_pkt[0] = ICM20948_I2C_SLV0_REG;
	_pkt[1] = 0x01;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	// Write configuration to AK0xx
	_pkt[0] = ICM20948_I2C_SLV0_CTRL;
	_pkt[1] = 0x81;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	hal.scheduler->delay(10);
	//time_sleep(0.01f);

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt, 2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	uint8_t tx_buf[2] = {ICM20948_EXT_SLV_SENS_DATA_00 | SPI_READ_BIT};
	uint8_t rx_buf[2];
	_spi->transaction(tx_buf, rx_buf, 2);
	//spi_xfer(_pi_ref, _handle, tx_buf, rx_buf, 2);

	hal.scheduler->delay(10);

	return rx_buf[1];
}

uint8_t AP_Compass_ICM20948::_poll_data_ready()
{
	uint8_t _pkt[2];

	// Select bank 0
	_pkt[0] = ICM20948_REG_BANK_SEL;
	_pkt[1] = 0x00;
	_spi->transfer(_pkt,2);
	//spi_write(_pi_ref, _handle, _pkt, 2);

	uint8_t tx_buf[2] = {ICM20948_INT_STATUS_1 | SPI_READ_BIT, 0x00};
	uint8_t rx_buf[2];
	_spi->transaction(tx_buf, rx_buf, 2);
	//spi_xfer(_pi_ref, _handle, tx_buf, rx_buf, 2);
	
	hal.scheduler->delay(10);

	if (rx_buf[2] & 0x01) {
		return 0;
	} else {
		// Not ready
		return 1;
	}
}


