/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#pragma once

/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */

#include <AP_Common.h>

#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <stdint.h>

#include "log.h"

/* DataFlash/Storage has the following backends:
 *
 * APM2 (Microchip/Atmel DataFlash)
 *
 *
 * DataFlash_Class 	------> DataFlash_File
 * 		|
 * 		|
 * 		|
 * DataFlash_Block	---------
 * 		|					|
 * DataFlash_Empty	DataFlash_SITL
 *
 */
class DataFlash_Class
{
public:
    // initialisation
    virtual void Init(const struct LogStructure *structure, uint8_t num_types);
    virtual bool CardInserted(void) = 0;

    // erase handling
    virtual bool NeedErase(void) = 0;
    virtual void EraseAll() = 0;

    /* Write a block of data at current offset */
    virtual void WriteBlock(const void *pBuffer, uint16_t size) = 0;

    // high level interface
    virtual uint16_t find_last_log(void) = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs(void) = 0;
    virtual void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port) = 0;

    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;

    /* logging methods common to all vehicles */
    uint16_t StartNewLog(void);
    void EnableWrites(bool enable) { _writes_enabled = enable; }

	// All moved to log.cpp
	/*
    void Log_Write_Format(const struct LogStructure *structure);
    void Log_Write_GPS(const GPS *gps, int32_t relative_alt);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_Baro(AP_Baro &baro);
    void Log_Write_Message(const char *message);
    void Log_Write_Message_P(const prog_char_t *message);
	*/

    bool logging_started(void) const { return log_write_started; }

	/*
      every logged packet starts with 3 bytes
    */
    struct log_Header {
        uint8_t head1, head2, msgid;
    };

protected:
    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type, 
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);
    
    virtual uint16_t start_new_log(void) = 0;

    const struct LogStructure *_structures;
    uint8_t _num_types;
    bool _writes_enabled;
    bool log_write_started;

    /*
      read a block
    */
    virtual void ReadBlock(void *pkt, uint16_t size) = 0;

};

#include "DataFlash_Block.h"
#include "DataFlash_File.h"

