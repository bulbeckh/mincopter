// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "log.h"

#if LOGGING_ENABLED == ENABLED

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "planner.h"

#include "control.h"

#include "util.h"

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = mincopter.DataFlash.find_last_log();

    if (dump_log == -2) {
        mincopter.DataFlash.DumpPageInfo(mincopter.cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        mincopter.cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log <= (last_log_num - mincopter.DataFlash.get_num_logs())) || (static_cast<uint16_t>(dump_log) > last_log_num)) {
        mincopter.cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    mincopter.DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}

void do_erase_logs(void)
{
	//gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    mincopter.DataFlash.EraseAll();
	//gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
}

int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    //in_mavlink_delay = true;
    do_erase_logs();
    //in_mavlink_delay = false;
    return 0;
}

int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        mincopter.cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(RCIN);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(RCOUT);
        TARG(OPTFLOW);
        TARG(COMPASS);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        mincopter.log_bitmask = mincopter.log_bitmask | bits;
    }else{
        mincopter.log_bitmask = mincopter.log_bitmask & ~bits;
    }

    return(0);
}

#if AUTOTUNE == ENABLED
struct PACKED log_AutoTune {
    LOG_PACKET_HEADER;
    uint8_t axis;           // roll or pitch
    uint8_t tune_step;      // tuning PI or D up or down
    float   rate_min;       // maximum achieved rotation rate
    float   rate_max;       // maximum achieved rotation rate
    float   new_gain_rp;       // newly calculated gain
    float   new_gain_rd;       // newly calculated gain
    float   new_gain_sp;       // newly calculated gain
};

// Write an Current data packet
void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        axis        : axis,
        tune_step   : tune_step,
        rate_min    : rate_min,
        rate_max    : rate_max,
        new_gain_rp  : new_gain_rp,
        new_gain_rd  : new_gain_rd,
        new_gain_sp  : new_gain_sp
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    int16_t angle_cd;       // lean angle in centi-degrees
    float   rate_cds;       // current rotation rate in centi-degrees / second
};

// Write an Current data packet
void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_out;
    //uint32_t throttle_integrator;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
};

// Write an Current data packet
void Log_Write_Current()
{
    uint16_t bv = mincopter.board_vcc_analog_source->voltage_latest() * 1000;

    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms             : mincopter.hal.scheduler->millis(),
        throttle_out        : mincopter.rc_3.servo_out,
        //throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (mincopter.battery.voltage() * 100.0f),
        current_amps        : (int16_t) (mincopter.battery.current_amps() * 100.0f),
        board_voltage       : bv,
        current_total       : mincopter.battery.current_total_mah()
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// TODO This can be removed
struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    int16_t dx;
    int16_t dy;
    uint8_t surface_quality;
    int16_t x_cm;
    int16_t y_cm;
    int32_t roll;
    int32_t pitch;
};

// Write an optical flow packet
/*
void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        dx              : optflow.dx,
        dy              : optflow.dy,
        surface_quality : optflow.surface_quality,
        x_cm            : (int16_t) optflow.x_cm,
        y_cm            : (int16_t) optflow.y_cm,
        roll            : of_roll,
        pitch           : of_pitch
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}
*/

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float    desired_pos_x;
    float    desired_pos_y;
    float    pos_x;
    float    pos_y;
    float    desired_vel_x;
    float    desired_vel_y;
    float    vel_x;
    float    vel_y;
    float    desired_accel_x;
    float    desired_accel_y;
};

// Write an Nav Tuning packet
void Log_Write_Nav_Tuning()
{
    const Vector3f &desired_position = Vector3f(0,0,0); /* planner.wp_nav.get_loiter_target();*/
    const Vector3f &position = mcstate.get_position();
    const Vector3f &velocity = mcstate.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_ms         : mincopter.hal.scheduler->millis(),
        desired_pos_x   : desired_position.x,
        desired_pos_y   : desired_position.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : 0, /* planner.wp_nav.desired_vel.x, */
        desired_vel_y   : 0, /* planner.wp_nav.desired_vel.y, */
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : 0, /* planner.wp_nav.desired_accel.x, */
        desired_accel_y : 0  /* planner.wp_nav.desired_accel.y */
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

// Write a control tuning packet
void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_ms             : mincopter.hal.scheduler->millis(),
        throttle_in         : mincopter.rc_3.control_in,
        angle_boost         : 0.0, /*controller.angle_boost, */
        throttle_out        : mincopter.rc_3.servo_out,
        desired_alt         : 0.0, // NOTE rmeoved the following function: get_target_alt_for_reporting() / 100.0f,
        inav_alt            : mcstate.current_loc.alt / 100.0f,
        baro_alt            : 0, /* planner.baro_alt, */
				/*
        desired_sonar_alt   : (int16_t)target_sonar_alt,
        sonar_alt           : sonar_alt,
				*/
        desired_climb_rate  : 0, /* planner.desired_climb_rate, */
        climb_rate          : controller.climb_rate
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

// Write a Compass packet
void Log_Write_Compass()
{
    const Vector3f &mag_offsets = mincopter.compass.get_offsets();
    const Vector3f &mag_motor_offsets = mincopter.compass.get_motor_offsets();
    const Vector3f &mag = mincopter.compass.get_field();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : mincopter.hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

// Write a performance monitoring packet
void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : 0, /* mcstate.ahrs.renorm_range_count, */
        renorm_blowup    : 0, /* mcstate.ahrs.renorm_blowup_count, */
        num_long_running : 0, /* perf_info_get_num_long_running(), */
        num_loops        : 0, /* perf_info_get_num_loops(), */
        max_time         : 0, /* perf_info_get_max_time(), */
        pm_test          : 0, /* mincopter.pmTest1, */
        i2c_lockup_count : mincopter.hal.i2c->lockup_count(),
        ins_error_count  : 0, /* mincopter.ins.error_count(), */
        inav_error_count : 0, /* mcstate.inertial_nav.error_count() */
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint8_t command_total;
    uint8_t command_number;
    uint8_t waypoint_id;
    uint8_t waypoint_options;
    uint8_t waypoint_param1;
    int32_t waypoint_altitude;
    int32_t waypoint_latitude;
    int32_t waypoint_longitude;
};

// Write a command processing packet
void Log_Write_Cmd(uint8_t num, const struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        command_total       : 0, /* mincopter.command_total, */
        command_number      : num,
        waypoint_id         : wp->id,
        waypoint_options    : wp->options,
        waypoint_param1     : wp->p1,
        waypoint_altitude   : wp->alt,
        waypoint_latitude   : wp->lat,
        waypoint_longitude  : wp->lng
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

// Write an attitude packet
void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms         : mincopter.hal.scheduler->millis(),
        control_roll    : (int16_t)controller.control_roll,
        roll            : (int16_t)mcstate.roll_sensor,
        control_pitch   : (int16_t)controller.control_pitch,
        pitch           : (int16_t)mcstate.pitch_sensor,
        control_yaw     : (uint16_t)controller.control_yaw,
        yaw             : (uint16_t)mcstate.yaw_sensor
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet
void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : 0.0f /*controller.throttle_cruise, */
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
void Log_Write_Event(uint8_t id)
{
    if (mincopter.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
void Log_Write_Data(uint8_t id, int16_t value)
{
    if (mincopter.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (mincopter.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Log_Write_Data(uint8_t id, int32_t value)
{
    if (mincopter.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (mincopter.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
void Log_Write_Data(uint8_t id, float value)
{
    if (mincopter.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

// REMOVED CAMERA LOGGING

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "BBfffff",       "Axis,TuneStep,RateMin,RateMax,RPGain,RDGain,SPGain" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "cf",          "Angle,Rate" },
#endif
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhIhhhf",     "TimeMS,ThrOut,ThrInt,Volt,Curr,Vcc,CurrTot" },
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "hhBccee",   "Dx,Dy,SQual,X,Y,Roll,Pitch" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Iffffffffff", "TimeMS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Ihhhffecchh", "TimeMS,ThrIn,AngBst,ThrOut,DAlt,Alt,BarAlt,DSAlt,SAlt,DCRt,CRt" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2","Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "BBHHIhBHB",    "RenCnt,RenBlw,NLon,NLoop,MaxT,PMT,I2CErr,INSErr,INAVErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),                 
      "CMD", "BBBBBeLL",     "CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccccCC",      "TimeMS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw" },
    { LOG_MODE_MSG, sizeof(log_Mode),
      "MODE", "Mh",          "Mode,ThrCrs" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
};

// Read the DataFlash log memory
void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
	mincopter.DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             NULL,
                             mincopter.cliSerial);
}

// start a new log
void start_logging() 
{
	// TODO Why is the logging check/status in planner?
	/*
    if (mincopter.log_bitmask != 0) {
        if (!planner.ap.logging_started) {
            planner.ap.logging_started = true;
            //in_mavlink_delay = true;
            mincopter.DataFlash.StartNewLog();
            //in_mavlink_delay = false;
            mincopter.DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            mincopter.DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

            // write system identifier as well if available
            char sysid[40];
            if (mincopter.hal.util->get_system_id(sysid)) {
                mincopter.DataFlash.Log_Write_Message(sysid);
            }

            // log the flight mode
            //Log_Write_Mode(mincopter.control_mode);
        }
        // enable writes
        mincopter.DataFlash.EnableWrites(true);
    }
	*/
}

#else // LOGGING_ENABLED

void Log_Write_Startup() {}
void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
void Log_Write_Mode(uint8_t mode) {}
void Log_Write_IMU() {}
void Log_Write_GPS() {}
#if AUTOTUNE == ENABLED
void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) {}
void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds) {}
#endif
void Log_Write_Current() {}
void Log_Write_Compass() {}
void Log_Write_Attitude() {}
void Log_Write_Data(uint8_t id, int16_t value){}
void Log_Write_Data(uint8_t id, uint16_t value){}
void Log_Write_Data(uint8_t id, int32_t value){}
void Log_Write_Data(uint8_t id, uint32_t value){}
void Log_Write_Data(uint8_t id, float value){}
void Log_Write_Event(uint8_t id){}
void Log_Write_Nav_Tuning() {}
void Log_Write_Control_Tuning() {}
void Log_Write_Performance() {}
void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED

