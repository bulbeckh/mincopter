
#include "log.h"

#if defined(LOGGING_ENABLED)

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;

#include "planner.h"
#include "control.h"

#include "util.h"

#include "AP_Progmem.h"


void Log_Write_Current(void)
{
    uint16_t bv = mincopter.board_vcc_analog_source->voltage_latest() * 1000;

    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms             : mincopter.hal.scheduler->millis(),
        throttle_out        : mincopter.rc_3.servo_out,
        battery_voltage     : (int16_t) (mincopter.battery.voltage() * 100.0f),
        current_amps        : (int16_t) (mincopter.battery.current_amps() * 100.0f),
        board_voltage       : bv,
        current_total       : mincopter.battery.current_total_mah()
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

// TODO 
void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms         : mincopter.hal.scheduler->millis(),
        control_roll    : 0, /* (int16_t)controller.control_roll, ) */
        roll            : 0, /* (int16_t)mcstate.roll_sensor, ) */
        control_pitch   : 0, /* (int16_t)controller.control_pitch, ) */
        pitch           : 0, /* (int16_t)mcstate.pitch_sensor, ) */
        control_yaw     : 0, /* (uint16_t)controller.control_yaw, ) */
        yaw             : 0 /* (uint16_t)mcstate.yaw_sensor */
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write Startup packet
void Log_Write_Startup(void)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

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

void Log_Write_IMU(void)
{
    uint32_t tstamp = mincopter.hal.scheduler->millis();
    const Vector3f &gyro = mincopter.ins.get_gyro();
    const Vector3f &accel = mincopter.ins.get_accel();
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        timestamp : tstamp,
        gyro_x  : gyro.x,
        gyro_y  : gyro.y,
        gyro_z  : gyro.z,
        accel_x : accel.x,
        accel_y : accel.y,
        accel_z : accel.z
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
    return;
}

void Log_Write_Baro(void)
{
    struct log_BARO pkt = {
        LOG_PACKET_HEADER_INIT(LOG_BARO_MSG),
        timestamp     : mincopter.hal.scheduler->millis(),
        altitude      : 0, /* baro.get_altitude(), */
        pressure	  : 0, /* baro.get_pressure(), */
        temperature   : 0, /* (int16_t)(baro.get_temperature() * 100), */
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
	return;
}

// Write an GPS packet
void Log_Write_GPS(void)
{
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
    	status        : (uint8_t)mincopter.g_gps->status(),
    	gps_week_ms   : mincopter.g_gps->time_week_ms,
    	gps_week      : mincopter.g_gps->time_week,
        num_sats      : mincopter.g_gps->num_sats,
        hdop          : mincopter.g_gps->hdop,
        latitude      : mincopter.g_gps->latitude,
        longitude     : mincopter.g_gps->longitude,
        rel_altitude  : 0, /* relative_alt, */
        altitude      : mincopter.g_gps->altitude_cm,
        ground_speed  : mincopter.g_gps->ground_speed_cm,
        ground_course : mincopter.g_gps->ground_course_cd,
        vel_z         : mincopter.g_gps->velocity_down(),
        apm_time      : mincopter.hal.scheduler->millis()
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
	return;
}

// Write an RCIN packet
void Log_Write_RCIN(void)
{
    struct log_RCIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        timestamp     : mincopter.hal.scheduler->millis(),
        chan1         : mincopter.hal.rcin->read(0),
        chan2         : mincopter.hal.rcin->read(1),
        chan3         : mincopter.hal.rcin->read(2),
        chan4         : mincopter.hal.rcin->read(3),
        chan5         : mincopter.hal.rcin->read(4),
        chan6         : mincopter.hal.rcin->read(5),
        chan7         : mincopter.hal.rcin->read(6),
        chan8         : mincopter.hal.rcin->read(7)
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
	return;
}

// Write an SERVO packet
void Log_Write_RCOUT(void)
{
    struct log_RCOUT pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
        timestamp     : mincopter.hal.scheduler->millis(),
        chan1         : mincopter.hal.rcout->read(0),
        chan2         : mincopter.hal.rcout->read(1),
        chan3         : mincopter.hal.rcout->read(2),
        chan4         : mincopter.hal.rcout->read(3),
        chan5         : mincopter.hal.rcout->read(4),
        chan6         : mincopter.hal.rcout->read(5),
        chan7         : mincopter.hal.rcout->read(6),
        chan8         : mincopter.hal.rcout->read(7)
    };
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
	return;
}

/* write a structure format to the log */
/* // TODO This has been moved directly to the log initialisation in DataFlash. It should only be called when creating a new log anyway
void Log_Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;

    memset(&pkt, 0, sizeof(pkt));

    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = PGM_UINT8(&s->msg_type);
    pkt.length = PGM_UINT8(&s->msg_len);
    strncpy_P(pkt.name, s->name, sizeof(pkt.name));
    strncpy_P(pkt.format, s->format, sizeof(pkt.format));
    strncpy_P(pkt.labels, s->labels, sizeof(pkt.labels));

    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));

	return;
}
*/

void Log_Write_Message(const char *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        msg  : {}
    };
    strncpy(pkt.msg, message, sizeof(pkt.msg));
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
	return;
}

void Log_Write_Message_P(const prog_char_t *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        msg  : {}
    };
    strncpy_P(pkt.msg, message, sizeof(pkt.msg));
    mincopter.DataFlash.WriteBlock(&pkt, sizeof(pkt));
	return;
}


// Read the DataFlash log memory
void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
	mincopter.DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             NULL,
                             mincopter.hal.console);
}

// start a new log
void start_logging(void)
{

	uint8_t temp = 0;
    if (mincopter.log_bitmask != 0) {
        if (!planner.ap.logging_active) {
            planner.ap.logging_active = true;

            mincopter.DataFlash.StartNewLog();

			// TODO Replace with another first line initialisation method (with model number, time, etc.)
            //mincopter.DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));
        }

        // enable writes
        mincopter.DataFlash.EnableWrites(true);
    }

	return;
}

#else // LOGGING_ENABLED

void Log_Write_Startup() {}
void Log_Write_IMU() {}
void Log_Write_GPS() {}
void Log_Write_Current() {}
void Log_Write_Compass() {}
void Log_Write_Attitude() {}
void Log_Write_Data(uint8_t id, int16_t value){}
void Log_Write_Data(uint8_t id, uint16_t value){}
void Log_Write_Data(uint8_t id, int32_t value){}
void Log_Write_Data(uint8_t id, uint32_t value){}
void Log_Write_Data(uint8_t id, float value){}
void Log_Write_Control_Tuning() {}
void Log_Write_Performance() {}
void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}

#endif // LOGGING_DISABLED

