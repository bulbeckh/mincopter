


## GPS - how it works

`AP_GPS* g_gps` is a pointer to an AP_GPS instance.

`GPS_Glitch gps_glitch` is also an instance of a class.
`AP_GPS_UBLOX g_gps_driver` is an instance of the derived class. Inherits from g_gps

For the AP_GPS_AUTO class, the g_gps pointer is passed in as a parameter, I assume so that it can determine the correct GPS to use


During **init.cpp**, the g_gps is assigned directly to the g_gps_driver so that all future GPS calles (via mincopter.g_gps)
are directed to the correct gps driver.

### Init
Called during init_ardupilot. Assigns a UART to the GPS. Also virtualised and implemented by derived class

### GPS Methods
- Scheduler function: update_GPS (called at 50Hz by scheduler)
- **read** is the virtualised function implemented by derived classes




