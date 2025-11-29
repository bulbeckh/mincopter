#pragma once

#include <stdint.h>

#include <AP_Math.h>


void dump_state(void);

void crash_check(void);

void init_home(void);

bool GPS_ok(void);

void read_receiver_rssi(void);


// TODO Removed for now but kept here for reference
// position_vector.pde
/* 
Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt);
Vector3f pv_location_to_vector(Location loc);
int32_t pv_get_lat(const Vector3f pos_vec);
int32_t pv_get_lon(const Vector3f &pos_vec);
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);
*/



