
#include "serial.h"

void print_RPY()
{
	cliSerial->printf_P(PSTR("RPY: %f %f %f\n"), cos_roll_x, cos_pitch_x, cos_yaw);
	return;
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

