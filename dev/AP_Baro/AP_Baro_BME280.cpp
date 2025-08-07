
#include <AP_Common.h>
#include <AP_Math.h>

#include <AP_HAL.h>
#include "AP_Baro_BME280.h"

#include <stdio.h>

#include <pigpiod_if2.h>

// BME280 Register Definitions
#define BME280_ADDR  0x77
#define BME280_ID 0xD0
#define BME280_RESET  0xE0
#define BME280_PRESS_START  0xF7
#define BME280_CTRL_MEAS 0xF4

// BME280 Compensation Parameter Definitions
#define DIG_T_START 0x88
#define DIG_P_START 0x8E
#define DIG_H_START 0xA1
#define DIG_H_START_2 0xE1

// BME280 Device Characteristics
#define BME280_DEVICE_ID 0x60

// BME280 Oversample Values
#define BME280_OVERSAMPLE_1 0x01
#define BME280_OVERSAMPLE_2 0x02
#define BME280_OVERSAMPLE_4 0x03
#define BME280_OVERSAMPLE_8 0x04
#define BME280_OVERSAMPLE_16 0x05

// BME280 Modes
#define BME280_MODE_SLEEP 0x00
#define BME280_MODE_NORMAL 0x03
#define BME280_MODE_FORCED 0x01

// BME280 Register Bitmasks
#define BME280_MASK_MODE 0x03
#define BME280_MASK_OVERSAMPLE_PRESS_TEMP 0xFC

extern const AP_HAL::HAL& hal;

bool AP_Baro_BME280::init()
{
	// Wake-up BME280
    if (hal.i2c->writeRegister(BME280_ADDR, BME280_RESET, 0xB6) != 0) {
        fprintf(stderr, "Failed to reset BME-280\n");
		return false;
	}

	// Wait 100ms after BME280 reset
	hal.scheduler->delay(100);

	// Read device ID to assert it is equal to BME280_DEVICE_ID
	uint8_t _baro_id;
	hal.i2c->readRegister(BME280_ADDR, BME280_ID, &_baro_id);
	if(_baro_id!=BME280_DEVICE_ID) hal.scheduler->panic(PSTR("Baro Init: got incorrect device ID\n"));

	// Retrieve compensation parameters
	if (init_compensation_parameters()) return false;

	// Set reading for pressure and temperature
	if (set_bme280_oversample(BME280_OVERSAMPLE_1, BME280_OVERSAMPLE_1)) return false;

	// Set mode to normal to start measuring
	if (set_bme280_mode(BME280_MODE_NORMAL)) return false;

	healthy = true;

    return true;
}

uint8_t AP_Baro_BME280::set_bme280_mode(uint8_t mode)
{
	uint8_t raw_mode;
	// TODO Add return value here for this read
	hal.i2c->readRegister(BME280_ADDR, BME280_CTRL_MEAS, &raw_mode);

	// Clear bit field
	uint8_t target_ctrl_meas = raw_mode & (~BME280_MASK_MODE);
	target_ctrl_meas |= mode;
	
	// Write byte
	hal.i2c->writeRegister(BME280_ADDR, BME280_CTRL_MEAS, target_ctrl_meas);
	// TODO Add return check
	//fprintf(stderr, "SET MODE: Failed to write\n");

	return 0;
}

uint8_t AP_Baro_BME280::set_bme280_oversample(uint8_t pressure_oversample, uint8_t temp_oversample)
{
	uint8_t raw_register;
	// TODO Add return check
	hal.i2c->readRegister(BME280_ADDR, BME280_CTRL_MEAS, &raw_register);

	// Clear bits w bit-mask
	uint8_t target_ctrl_meas = raw_register & (~BME280_MASK_OVERSAMPLE_PRESS_TEMP);
	target_ctrl_meas |= (temp_oversample << 5);
	target_ctrl_meas |= (pressure_oversample << 2);

	// TODO Add return check
	hal.i2c->writeRegister(BME280_ADDR, BME280_CTRL_MEAS, target_ctrl_meas);
	//fprintf(stderr, "SET READING: Failed to write\n");

	return 0;
}

void AP_Baro_BME280::accumulate(void)
{
	/* Implement */
}

uint8_t AP_Baro_BME280::read()
{
	uint8_t _buffer[6];
	hal.i2c->readRegisters(BME280_ADDR, BME280_PRESS_START, 6, _buffer);

	// Combined into raw readings
	uint32_t pressure_uncompensated = (_buffer[0] << 12) | (_buffer[1] << 4) | (0x0F & _buffer[2]);
	uint32_t temperature_uncompensated = (_buffer[3] << 12) | (_buffer[4] << 4) | (0x0F & _buffer[5]);

	// Use Bosch compensation functions to convert
	int32_t comp_temp = BME280_compensate_T_int32((int32_t)temperature_uncompensated);
	uint32_t comp_pressure = BME280_compensate_P_int32((int32_t)pressure_uncompensated);

	//printf("TEMP: %d, PRES: %u\n", comp_temp, comp_pressure);
	//printf("hexT: %LX, hexP: %LX\n", pressure_uncompensated, temperature_uncompensated);

	// TODO Make this scaling configurable or read from sensor device on init
	// Update the pressure and temperature readings
	_temperature = comp_temp / 100.0f;
	_pressure = comp_pressure;

    return 0;
}

uint8_t AP_Baro_BME280::init_compensation_parameters(void)
{
	uint8_t comp_reading[24];

	/* Reading Temperature and Pressure compensation parameters */
	if (hal.i2c->readRegisters(BME280_ADDR, DIG_T_START, 24, comp_reading)) {
		fprintf(stderr, "Failed to read compensation parameter data.\n");
		return 1;
	}

	// Temperature
	_dig.dig_T1 = (uint16_t)((comp_reading[1]<<8) | (comp_reading[0]));
	_dig.dig_T2 = (int16_t)((comp_reading[3]<<8) | (comp_reading[2]));
	_dig.dig_T3 = (int16_t)((comp_reading[5]<<8) | (comp_reading[4]));

	// Pressure
	_dig.dig_P1 = (uint16_t)((comp_reading[7]<<8) | (comp_reading[6]));
	_dig.dig_P2 = (int16_t)((comp_reading[9]<<8) | (comp_reading[8]));
	_dig.dig_P3 = (int16_t)((comp_reading[11]<<8) | (comp_reading[10]));
	_dig.dig_P4 = (int16_t)((comp_reading[13]<<8) | (comp_reading[12]));
	_dig.dig_P5 = (int16_t)((comp_reading[15]<<8) | (comp_reading[14]));
	_dig.dig_P6 = (int16_t)((comp_reading[17]<<8) | (comp_reading[16]));
	_dig.dig_P7 = (int16_t)((comp_reading[19]<<8) | (comp_reading[18]));
	_dig.dig_P8 = (int16_t)((comp_reading[21]<<8) | (comp_reading[20]));
	_dig.dig_P9 = (int16_t)((comp_reading[23]<<8) | (comp_reading[22]));

	return 0;
}

int32_t AP_Baro_BME280::BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)_dig.dig_T1<<1))) * ((int32_t)_dig.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)_dig.dig_T1)) * ((adc_T>>4) - ((int32_t)_dig.dig_T1))) >> 12) * ((int32_t)_dig.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t AP_Baro_BME280::BME280_compensate_P_int32(int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)_dig.dig_P6);
	var2 = var2 + ((var1*((int32_t)_dig.dig_P5))<<1);
	var2 = (var2>>2) + (((int32_t)_dig.dig_P4)<<16);
	var1 = (((_dig.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)_dig.dig_P2) * var1)>>1))>>18;
	var1 = ((((32768+var1))*((int32_t)_dig.dig_P1))>>15);
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)var1);
	} else {
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)_dig.dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)_dig.dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + _dig.dig_P7) >> 4));
	return p;
}

