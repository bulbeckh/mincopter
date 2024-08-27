#pragma once

#include <stdint.h>

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
#include "config.h"

#include "ap_union.h"
#include "navigation.h"
#include "util.h"
#include "motors.h"

// system.cpp
bool set_mode(uint8_t mode);

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

extern uint8_t rtl_state;
extern uint8_t land_state;
extern int16_t command_nav_index;
extern int32_t condition_value;
extern uint32_t condition_start;
extern int16_t command_nav_index;
extern uint32_t rtl_loiter_start_time;


// commands_process.pde
void change_command(uint8_t cmd_index);
void update_commands();
void execute_nav_command(void);
void verify_commands(void);
int16_t find_next_nav_index(int16_t search_index);
void exit_mission();

// commands.pde
void init_commands();
struct Location get_cmd_with_index(int i);
void set_cmd_with_index(struct Location temp, int i);
int32_t get_RTL_alt();
void init_home();

// commands_logic.pde
void process_nav_command();
void process_cond_command();
void process_now_command();
bool verify_nav_command();
bool verify_cond_command();
void do_RTL(void);
void do_takeoff();
void do_nav_wp();
void do_land(const struct Location *cmd);
void do_loiter_unlimited();
void do_circle();
void do_loiter_time();
bool verify_takeoff();
bool verify_land();
bool verify_nav_wp();
bool verify_loiter_unlimited();
bool verify_loiter_time();
bool verify_circle();
bool verify_RTL();
void do_wait_delay();
void do_change_alt();
void do_within_distance();
void do_yaw();
bool verify_wait_delay();
bool verify_change_alt();
bool verify_within_distance();
bool verify_yaw();
void do_guided(const struct Location *cmd);
void do_change_speed();
void do_jump();
void do_set_home();
void do_roi();
void do_take_picture();
