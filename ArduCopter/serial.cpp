
#include "serial.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "system.h"

void print_RPY()
{
	mincopter.cliSerial->printf_P(PSTR("RPY: %f %f %f\n"), mcstate.cos_roll_x, mcstate.cos_pitch_x, mcstate.cos_yaw);
	return;
}

void print_roll_rates_and_accel()
{
	// Get roll rates
	Vector3f roll_rates = mincopter.ins.get_gyro();

	mincopter.cliSerial->printf_P(PSTR("IMUR:%9.4f,%9.4f,%9.4f\n"), roll_rates.x, roll_rates.y, roll_rates.z);

	// Get acceleration from IMU
	Vector3f accel = mincopter.ins.get_accel();

	mincopter.cliSerial->printf_P(PSTR("IMUA:%9.4f,%9.4f,%9.4f\n"), accel.x, accel.y, accel.z);
} 

void print_GPS()
{
	if (GPS_ok()) {
		mincopter.cliSerial->println_P(PSTR("GPS is OK"));
	} else {
		mincopter.cliSerial->println_P(PSTR("GPS bad"));
	}

	mincopter.cliSerial->printf_P(PSTR("GPS: %d %d %d\n"), mcstate.current_loc.alt, mcstate.current_loc.lat, mcstate.current_loc.lng);

	return;
}

