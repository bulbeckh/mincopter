#pragma once

#include <stdint.h>

#include "AP_Common.h"
#include "AP_Progmem.h"

/* TODO We currently have multiple disparate logging systems, particularly between simulation and hardware targets that
 * need to be unified. We have currently:
 *
 * dump_state (in lib/util.cpp)
 *
 * Simulation Logger (in sim/)
 *
 * ArduPilot logging (dataflash) */


/* MinCopter Log Message Types
 *
 * ## Sensors
 * GPS Lat
 * GPS Lng
 * GPS Alt
 * GPS xVel
 * GPS yVel
 * GPS zVel
 * Status
 *
 * Barometer Temp (float)
 * Barometer Pressure
 *
 * Compass X
 * Compass Y
 * Compass Z
 *
 * Accel X
 * Accel Y
 * Accel Z
 * Gyro X
 * Gyro Y
 * Gyro Z
 *
 * ## State
 * position (x3 float)
 * velocity (x3 float)
 * attitude (x3 float)
 * angular velocity (x3 float)
 *
 * ## State (custom)
 * These are things that we log for specific state implementations (like complementary filter, ekf, etc.)
 *
 * ## Control (and Mixer)
 * m0
 * m1
 * m2
 * m3
 * force
 * rt
 * pt
 * yt
 *
 * ## Control (custom)
 * Controller specific logging messages
 *
 * ## Planner
 * target position (x3 float)
 * target velocity (x3 float)
 * target attitude (x3 float)
 * target angular velocity (x3 float)
 *
 * ## Planner specific
 * Probably won't be much - maybe planner mode changes or arming/disarming
 *
 */


/* Logging and External Storage
 *
 * We should write the logging interface without assuming that we use external storage (i.e. the storage
 * interface should also match hal.storage).
 *
 * The storage device can either be block storage (like the ATMEL dataflash) or to a filesystem (i.e. linux/generic).
 *
 *
 * Decision about how the logs are stored/retrieved should be left to the device implementation.
 *
 * 1. Log Interface
 *
 * get_num_logs
 * get_logs
 * write_log
 * init - needs be called at the start and passed an instance of external storage to use
 *
 * 2a. Block Interface
 * Uses pages for storage with headers for each page. Exposes methods
 *
 *
 *
 *
 */




/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149
							
#define PGM_UINT8(addr) pgm_read_byte((const prog_char *)addr)

// structure used to define logging format
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char name[5];
    const char format[16];
    const char labels[64];
};

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_out;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
};

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_in;
    int16_t  angle_boost;
    int16_t  throttle_out;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
		/*
    int16_t  desired_sonar_alt;
    int16_t  sonar_alt;
		*/
    int16_t  desired_climb_rate;
    int16_t  climb_rate;
};

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
};

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
    uint16_t ins_error_count;
    uint8_t inav_error_count;
};

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
};

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
};

struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float   altitude;
    float   pressure;
    int16_t temperature;
};

struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    int16_t  hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
    float    vel_z;
    uint32_t apm_time;
};

struct PACKED log_RCIN {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
};

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
};

/* log structures common to all vehicle types */
struct PACKED log_Format {
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    char msg[64];
};





/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode
 */


// TODO Is this the correct way to declare this?
extern const struct LogStructure log_structure[];

/* @brief Commence logging of variables. Called after arming is complete
*/
void start_logging(void);

/* @brief Wrapper for reading DataFlash logs
* @param log_num Number of log to read
* @param start_page First page of log
* @param end_page Last page of log
*/
void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);


// Write functions

/* @brief Functions to write logs */
void Log_Write_Startup();

void Log_Write_Format(const struct LogStructure *s);

void Log_Write_IMU();
void Log_Write_Baro();
void Log_Write_GPS();

void Log_Write_RCIN();
void Log_Write_RCOUT();

void Log_Write_Current();
void Log_Write_Compass();
void Log_Write_Attitude();

// Remove - we won't need to log arbitrary data
void Log_Write_Data(uint8_t id, int16_t value);
void Log_Write_Data(uint8_t id, uint16_t value);
void Log_Write_Data(uint8_t id, int32_t value);
void Log_Write_Data(uint8_t id, uint32_t value);
void Log_Write_Data(uint8_t id, float value);

void Log_Write_Nav_Tuning();
void Log_Write_Control_Tuning();
void Log_Write_Performance();
void Log_Write_Format(const struct LogStructure *s);
void Log_Write_Message(const char *message);
void Log_Write_Message_P(const prog_char_t *message);
void Log_Write_Error(uint8_t sub_system, uint8_t error_code);


