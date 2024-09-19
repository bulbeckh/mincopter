#pragma once

#include <AP_Menu.h>

#include "parameters.h"
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <AP_InertialNav.h>
#include <GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Compass.h>
#include <AC_Fence.h>
#include <AC_WPNav.h>

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"

// system.cpp
bool set_mode(uint8_t mode);

extern int16_t control_roll;
extern int16_t control_pitch;
//extern uint32_t throttle_integrator;
float get_target_alt_for_reporting();
extern int16_t pmTest1;

extern Parameters g;
extern Vector3f omega;
extern AP_AHRS_DCM ahrs;
extern float G_Dt;
extern AP_MotorsQuad motors;
extern AP_InertialNav inertial_nav;
extern GPS         *g_gps;
extern GPS_Glitch   gps_glitch;
extern AP_Compass_HMC5843 compass;
extern AC_WPNav wp_nav;

extern AP_UNION_T ap;
extern AP_FAILSAFE_T failsafe;
extern AC_Fence fence;
#include "config.h"

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


