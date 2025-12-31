import math
import numpy as np

def gps_to_pos(lat_cd, lng_cd, alt_cm, home_lat, home_lng, home_alt):
    '''Converts a latitude, longitude, and altitude reading into a NED position (in metres) assuming a provided offset lat/lng/alt'''
    gps_pos = np.array([0,0,0], dtype=np.float64).reshape(3,1)

    ## If we do not yet have a GPS lock, then assume position is 0
    if (lat_cd==0 or lng_cd==0):
        return gps_pos

    lat_offset = (lat_cd - home_lat) / 1e7
    lng_offset = (lng_cd - home_lng) / 1e7

    gps_pos[0] = lat_offset*111320
    gps_pos[1] = 40075000*lng_offset*math.cos(math.pi*lat_cd/(180*1e7))/360
    gps_pos[2] = -(alt_cm-home_alt) / 1e2

    return gps_pos

def baro_press_temp_to_alt(pressure, temperature, ground_pressure, ground_temperature):
    '''Converts pressure (pascals??) and temperature (celsius) into an estimate of altitude (in NED frame, so negative reading)'''
    ## If we do not yet have a valid reading, then return 0 alt
    if (pressure==0 or temperature==0):
        return 0

    scaling = pressure / ground_pressure
    temp = ground_temperature + 273.15
    
    alt = 153.8462 * temp * (1.0 - math.exp(0.190259 * math.log(scaling)))
    
    return -1*alt
    