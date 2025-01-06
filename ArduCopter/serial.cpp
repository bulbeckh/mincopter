
#include "serial.h"

void print_RPY()
{
	cliSerial->printf_P(PSTR("RPY: %f %f %f\n"), cos_roll_x, cos_pitch_x, cos_yaw);
	return;
}

void print_roll_rates_and_accel()
{
	// Get roll rates
	Vector3f roll_rates = ins.get_gyro();

	cliSerial->printf_P(PSTR("IMUR:%9.4f,%9.4f,%9.4f\n"), roll_rates.x, roll_rates.y, roll_rates.z);

	// Get acceleration from IMU
	Vector3f accel = ins.get_accel();

	cliSerial->printf_P(PSTR("IMUA:%9.4f,%9.4f,%9.4f\n"), accel.x, accel.y, accel.z);
} 

void print_GPS()
{
	if (GPS_ok()) {
		cliSerial->println_P(PSTR("GPS is OK"));
	} else {
		cliSerial->println_P(PSTR("GPS bad"));
	}

	cliSerial->printf_P(PSTR("GPS: %d %d %d\n"), current_loc.alt, current_loc.lat, current_loc.lng);

	return;
}

