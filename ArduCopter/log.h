#pragma once


#include <AP_Menu.h>
/*
HASH include "parameters.h"
HASH include <AP_Math.h>
HASH include <AP_AHRS.h>
HASH include <AP_Motors.h>
HASH include <AP_InertialNav.h>
HASH include <GPS.h>
HASH include <AP_GPS_Glitch.h>
HASH include <AP_Compass.h>
HASH include <AC_Fence.h>
HASH include <AC_WPNav.h>
HASH include "navigation.h"
HASH include "util.h"
HASH include "motors.h"
HASH include "failsafe.h"
HASH include "config.h"
*/

// TODO Is this the correct way to declare this?
extern const struct LogStructure log_structure[];

/* @brief Various logging functions
*/
bool     print_log_menu(void);
int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);
void do_erase_logs(void);

/* @brief Commence logging of variables. Called after arming is complete
*/
void start_logging(void);

/* @brief Wrapper for reading DataFlash logs
* @param log_num Number of log to read
* @param start_page First page of log
* @param end_page Last page of log
*/
void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);

// NOTE this can be removed
int8_t process_logs(uint8_t argc, const Menu::arg *argv);

// NOTE can remove some of these like optflow
/* @brief Functions to write logs
*/
void Log_Write_Startup();
void Log_Write_Cmd(uint8_t num, const struct Location *wp);
void Log_Write_Mode(uint8_t mode);
void Log_Write_IMU();
void Log_Write_GPS();
#if AUTOTUNE == ENABLED
void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp);
void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds);
#endif
void Log_Write_Current();
void Log_Write_Compass();
void Log_Write_Attitude();
void Log_Write_Data(uint8_t id, int16_t value);
void Log_Write_Data(uint8_t id, uint16_t value);
void Log_Write_Data(uint8_t id, int32_t value);
void Log_Write_Data(uint8_t id, uint32_t value);
void Log_Write_Data(uint8_t id, float value);
void Log_Write_Event(uint8_t id);
void Log_Write_Optflow();
void Log_Write_Nav_Tuning();
void Log_Write_Control_Tuning();
void Log_Write_Performance();
void Log_Write_Camera();
void Log_Write_Error(uint8_t sub_system, uint8_t error_code);


