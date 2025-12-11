#pragma once

#include <stdint.h>

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

// NOTE can remove some of these like optflow
/* @brief Functions to write logs
*/
void Log_Write_Startup();
void Log_Write_Cmd(uint8_t num, const struct Location *wp);
void Log_Write_Mode(uint8_t mode);
void Log_Write_IMU();
void Log_Write_GPS();

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


