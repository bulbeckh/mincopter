#pragma once

int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
int8_t   test_compass(uint8_t argc,              const Menu::arg *argv);
int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
int8_t   test_motors(uint8_t argc,               const Menu::arg *argv);
int8_t   test_motorsync(uint8_t argc,            const Menu::arg *argv);
int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
int8_t   test_radio_pwm(uint8_t argc,            const Menu::arg *argv);
int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
int8_t   test_shell(uint8_t argc,                const Menu::arg *argv);
int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
void print_hit_enter();
