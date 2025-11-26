#pragma once

#include <stdint.h>

#include <AP_Math.h>

void read_receiver_rssi(void);


// AP_State.pde
void set_auto_armed(bool b);


// crash_check.pde
void crash_check();


// position_vector.pde
Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt);
Vector3f pv_location_to_vector(Location loc);
int32_t pv_get_lat(const Vector3f pos_vec);
int32_t pv_get_lon(const Vector3f &pos_vec);
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);

void init_home();

bool GPS_ok();

void update_auto_armed();


