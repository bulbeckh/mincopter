
#include <AP_Menu.h>

#include "log.h"
#include "util.h"

#include "mcinstance.h"

/* Menu interface
 *
 *
 *
 *
 *
 * If any function returns -2 then the menu exits
 */

extern MCInstance mincopter;

// TODO remove this
static int8_t do_none(uint8_t argc, const Menu::arg* argv) {
	mincopter.cliSerial->println_P(PSTR("in none function - messaged received"));
	return -2;
}

/* @brief Retrieves and dumps a log message to the console
*/
static int8_t run_get_log(uint8_t argc, const Menu::arg* argv) {
	
	int16_t lognum = argv[1].i;
	uint16_t dl_start, dl_end;
	mincopter.DataFlash.get_log_boundaries(lognum, dl_start, dl_end);

	mincopter.cliSerial->printf_P(PSTR("GL00-Retrieving log %d start:%u end:%u\n"),lognum, dl_start, dl_end);

	Log_Read((uint16_t)lognum,dl_start, dl_end);

	mincopter.cliSerial->printf_P(PSTR("END0\n"));
	
	return -2;
}

/* @brief Dumps page info for the dataflash logs
*/
static int8_t list_logs(uint8_t argc, const Menu::arg* argv) {

	mincopter.DataFlash.ListAvailableLogs(mincopter.cliSerial);
	
	return -2;
}

/* @brief Prints the number of available logs to the serial
*/
static int8_t run_show_logs(uint8_t argc, const Menu::arg* argv) {
	//mincopter.DataFlash.ListAvailableLogs(cliSerial);
	
	uint16_t number_logs = mincopter.DataFlash.get_num_logs();
	mincopter.cliSerial->printf_P(PSTR("SL00-logs-%d\n"), number_logs);

	// NOTE Are logs really 0 indexed in DataFlash?
	for (uint16_t i=1;i<number_logs;i++) {
		// For each log, get the start and end address and calculate size and send to serial
		uint16_t log_start, log_end;
		mincopter.DataFlash.get_log_boundaries(i, log_start, log_end);	
		
		mincopter.cliSerial->printf_P(PSTR("SL01-%d-%d\n"), i, log_end-log_start);
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
	mincopter.cliSerial->printf_P(PSTR("END0\n"));
	return -2;
}

/* @brief Returns the number and count of arguments given
* 
* The argv vector represents each of the elements of the command
* argv[0] -> command name
* argv[1] -> argument 1
* argv[2] -> argument 2
* ...
* argv[LAST] -> the firmware version or something? Can be ignored but note it
* 	inflates the argcount by 1
*/
static int8_t return_args(uint8_t argc, const Menu::arg* argv) {
	mincopter.cliSerial->printf_P(PSTR("RA00-%u\n"),argc);
	for (int i=0;i<argc;i++) {
		mincopter.cliSerial->printf_P(PSTR("RA01-%s\n"),argv[i].str);
	}
	return -2;
}

// NOTE This interfaces here needs to match the Python console interface
const struct Menu::command main_menu_commands[] PROGMEM = {
	{"none", do_none},
	{"non2", do_none},
	{"showlogs", run_show_logs},
	{"returnargs", return_args},
	{"getlog", run_get_log},
	{"listlogs", list_logs}
};

MENU(main_menu, "FIRMWARE", main_menu_commands);

/* @brief Initialises the CLI for communication
 */
void init_cli(AP_HAL::UARTDriver* port)
{
	Menu::set_port(port);
	port->set_blocking_writes(true);

	mincopter.hal.scheduler->register_delay_callback(NULL,5);

	return;
}


/* @brief Run the command line interface
 *
 * In order to make it asynchronous, we should put estimates of the max execution
 * time of each function and periodically call the run_cli function. This function
 * is non-blocking meaning that it will only execute one command and then return.
 * There is no guarantee that the command will run within its alotted time, only
 * that the scheduler with notify that it overrun.
 *
 * In order to ensure the a function called through the CLI does not exceed the
 * allotted time, a time could be used. However, an alternative is to abstract function
 * execution down into roughly equal time sub-parts.
 *
 * For example, a function that writes logs to the screen knows that it can log 2KBytes
 * in 1000us and so it will do that in these chunks.
 */

// Macro to log an output string every n iterations
#define LOG_N_ITER(unique_pref, n, outstr) static int unique_pref_##i=0;\
																						if (unique_pref_##i>=n) { \
																							mincopter.cliSerial->printf_P(PSTR(outstr));\
																							unique_pref_##i=0;\
																						} else { \
																							unique_pref_##i++;\
																						}

#define CLI_MAX_TIME_US 500
void run_cli(void)
{
	int32_t cli_remaining = CLI_MAX_TIME_US;

	LOG_N_ITER(cli_counter, 10, "ET01-In run_cli\n")

	while (cli_remaining>0) {
		uint32_t cli_run_start;

		cli_run_start = micros();
		main_menu.run();

		uint32_t cli_et = micros() - cli_run_start;
		cli_remaining -= cli_et;
		
		// Dump menu execution time to console
		//mincopter.cliSerial->printf_P(PSTR("ET00-Function took %dus\n"), cli_et);
	}

	return;
}
