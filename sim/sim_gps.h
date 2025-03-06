
#ifndef __AP_GPS_SIM_H__
#define __AP_GPS_SIM_H__

#include <AP_HAL.h>
#include <AP_Common.h>
#include "GPS.h"


class AP_GPS_Sim: public GPS
{
public:
	AP_GPS_Sim() :
		GPS(),
		_step(0),
		_msg_id(0),
		_payload_length(0),
		_payload_counter(0),
		_fix_count(0),
		_disable_counter(0),
		next_fix(GPS::FIX_NONE),
        need_rate_update(false),
        rate_update_step(0)
		{}

    // Methods
    virtual void                    init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);
    virtual bool                    read();

    float       get_lag() { return 0.5; }   // ublox lag is lower than the default 1second

    // State machine state
    uint8_t         _step;
    uint8_t         _msg_id;
    uint16_t        _payload_length;
    uint16_t        _payload_counter;

	// 8 bit count of fix messages processed, used for periodic
	// processing
    uint8_t			_fix_count;

    uint8_t         _class;

    // do we have new position information?
    bool            _new_position;

    // do we have new speed information?
    bool            _new_speed;

    uint8_t         _disable_counter;

    // Buffer parse & GPS state update
    bool        _parse_gps();

    // used to update fix between status and position packets
    Fix_Status  next_fix;

    bool need_rate_update;
    uint8_t rate_update_step;
    uint32_t _last_5hz_time;

    void 	    _configure_navigation_rate(uint16_t rate_ms);
    void        _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    void        _configure_gps(void);
    void        _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
    void        _send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);
    void		send_next_rate_update(void);

};

#endif // __AP_GPS_UBLOX_H__
