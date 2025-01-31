
#include <AP_Menu.h>

#include "mcinstance.h"

extern MCInstance mincopter;

static int8_t do_none(uint8_t argc, const Menu::arg* argv) {
	mincopter.cliSerial->println_P(PSTR("in none function - messaged received"));
	return 0;
}

const struct Menu::command main_menu_commands[] PROGMEM = {
	{"none", do_none},
	{"non2", do_none}
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
