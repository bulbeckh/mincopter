
#include <inttypes.h>

#include <AP_Common.h>
#include <AP_Math.h>

#include <AP_HAL.h>
#include "AP_Baro_BME280.h"

#include <stdio.h>

#include <pigpiod_if2.h>

// BME280 definitions
#define BME280_ADDR  0x77

#define BME280_ID 0xD0
#define BME280_RESET  0xE0
#define BME280_PRESS_START  0xF7


// Compensation Parameter Definitions
#define DIG_T_START 0x88
#define DIG_P_START 0x8E
#define DIG_H_START 0xA1
#define DIG_H_START_2 0xE1

extern const AP_HAL::HAL& hal;

bool AP_Baro_BME280::init()
{

	// Wake-up BME280
    if (hal.i2c->writeRegister(BME280_ADDR, BME280_RESET, 0xB6) != 0) {
        fprintf(stderr, "Failed to reset BME-280\n");
		return false;
	}

	// Read device id to assert it is equal to 0x60
	uint8_t _baro_id;
	hal.i2c->readRegister(BME280_ADDR, BME280_ID, &_baro_id);
	if(_baro_id!=0x60) hal.scheduler->panic(PSTR("Baro Init: got incorrect device ID\n"));

	// Retrieve compensation parameters
	init_compensation_parameters();

	healthy = true;

    return true;
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
	uint32_t pressure_uncompensated = (_buffer[0] << 12) | (_buffer[1] << 4) | _buffer[2];
	uint32_t temperature_uncompensated = (_buffer[3] << 12) | (_buffer[4] << 4) | _buffer[5];

	uint16_t humidity_uncompensated = (_buffer[6] << 8) | _buffer[7];

	int32_t comp_temp = BME280_compensate_T_int32((int32_t)pressure_uncompensated);

	//printf("BARO: %u, %u, %u\n", pressure_uncompensated, temperature_uncompensated, humidity_uncompensated);
	printf("BARO: %d\n", comp_temp);

	/* Implement */
    return 0;
}

float AP_Baro_BME280::get_pressure() {
    //return Press;
	return 1000.0f;
}

float AP_Baro_BME280::get_temperature() {
    //return Temp;
	return 20.0f;
}

void AP_Baro_BME280::init_compensation_parameters()
{
	static uint8_t comp_reading[24];

	/* Reading Temperature and Pressure compensation parameters */
	if (hal.i2c->readRegisters(BME280_ADDR, DIG_T_START, 24, comp_reading)) {
		fprintf(stderr, "Failed to read compensation parameter data.\n");
		return;
	}

	// Temperature
	dig_T1 = (int16_t)((comp_reading[1]<<8) | (comp_reading[0]));
	dig_T2 = (int16_t)((comp_reading[3]<<8) | (comp_reading[2]));
	dig_T3 = (int16_t)((comp_reading[5]<<8) | (comp_reading[4]));

	// Pressure
	dig_P1 = (int16_t)((comp_reading[7]<<8) | (comp_reading[6]));
	dig_P2 = (int16_t)((comp_reading[9]<<8) | (comp_reading[8]));
	dig_P3 = (int16_t)((comp_reading[11]<<8) | (comp_reading[10]));
	dig_P4 = (int16_t)((comp_reading[13]<<8) | (comp_reading[12]));
	dig_P5 = (int16_t)((comp_reading[15]<<8) | (comp_reading[14]));
	dig_P6 = (int16_t)((comp_reading[17]<<8) | (comp_reading[16]));
	dig_P7 = (int16_t)((comp_reading[19]<<8) | (comp_reading[18]));
	dig_P8 = (int16_t)((comp_reading[21]<<8) | (comp_reading[20]));
	dig_P9 = (int16_t)((comp_reading[23]<<8) | (comp_reading[22]));

	return;
}

int32_t AP_Baro_BME280::BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

/* Returns compensated pressure reading */
uint32_t AP_Baro_BME280::BME280_compensate_P_int32(int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);

	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)var1);
	} else {
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}

