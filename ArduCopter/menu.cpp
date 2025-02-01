
#include <AP_Menu.h>

#include "mcinstance.h"

extern MCInstance mincopter;

// TODO remove this
static int8_t do_none(uint8_t argc, const Menu::arg* argv) {
	mincopter.cliSerial->println_P(PSTR("in none function - messaged received"));
	return 0;
}

/* @brief Prints the number of available logs to the serial
*/
static int8_t run_show_logs(uint8_t argc, const Menu::arg* argv) {
	//mincopter.DataFlash.ListAvailableLogs(cliSerial);
	
	uint16_t number_logs = mincopter.DataFlash.get_num_logs();
	for (uint16_t i=0;i<number_logs;i++) {
		// For each log, get the start and end address and calculate size and send to serial
		uint16_t log_start, log_end;
		mincopter.DataFlash.get_log_boundaries(i, log_start, log_end);	
		
		mincopter.cliSerial->printf_P(PSTR("SL00%d%d\n"), i, log_end-log_start);
	}
	/*
		DataFlash.ListAvailableLogs(cliSerial);
		
		uint16_t nl = DataFlash.get_num_logs();
		cliSerial->printf_P(PSTR("Num Logs: %u\n"), nl);

	
		int16_t lognum=19;
		uint16_t dl_start;
		uint16_t dl_end;

		DataFlash.get_log_boundaries(lognum, dl_start, dl_end);
		cliSerial->printf_P(PSTR("Reading Log 19: %u %u\n"), dl_start, dl_end);
		Log_Read((uint16_t)lognum,dl_start, dl_end);
	*/
	
	// Terminate with an "END0" string
	mincopter.cliSerial->printf_P(PSTR("END0"));
	return 0;
}

// NOTE This interfaces here needs to match the Python console interface
const struct Menu::command main_menu_commands[] PROGMEM = {
	{"none", do_none},
	{"non2", do_none},
	{"showlogs", run_show_logs}
};

MENU(main_menu, "FIRMWARE", main_menu_commands);

void run_cli(AP_HAL::UARTDriver* port)
{
	Menu::set_port(port);
	port->set_blocking_writes(true);

	mincopter.hal.scheduler->register_delay_callback(NULL,5);

	while (1) {
		main_menu.run();
	} 

}
