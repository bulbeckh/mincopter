
GPS -> 


## List of sensors

| Sensor Type | Class Name | Backends |
| --- | --- | --- |
| GPS | AP_GPS | |
| IMU | AP_InertialSensor | |
| Barometer | AP_Baro | |
| Analog-Digital Converter (External) | AP_ADC | |
| Analog-Digital Converter (Onboard) | AP_AnalogIn (part of HAL) | |
| Storage | DataFlash | |
| Compass | AP_Compass | |
| --- | --- | --- |

#### ADC, AnalogSource, and AnalogIn
- The HAL defines an `AP_HAL::AnalogSource` as a pin where a voltage can be read as well as an `AP_HAL::AnalogIn` class which stores a set of these pins.
- `AP_ADC` is a separate class that is use for external analog to digital converters. Isn't currently used in AP_HAL_AVR nor AP_BattMonitor as these use the onboard `AnalogSource` and `AnalogIn` ADC variables.

#### AP_BattMonitor
- Initialised via call to ::init during startup.
- Can obtain battery voltage, current, and percentage remaining.
- Class is primarily used for logging purposes currently but should be used for failsafe and path planning later (TODO).
- Class maintains two `AP_HAL::AnalogSource` variables - one for current and one for voltage.
- Can get current via `AP_BattMonitor::current_amps()` and voltage via `AP_BattMonitor::voltage()`.

#### Compass
- Initialised via call to ::init (which is called indirectly during startup via `init_compass`.
- Maintains a `_field` variable (as Vector3f) which stores magnetic field readings from the compass.
- Both `read` and `accumulate` are virtualised. Calls to `accumulate` in turn calls `read_raw` which reads raw sensors measurements and will populate the `_mag_<x,y,z>` variable. During `accumulate`, these are stored in `_mag_<x,y,z>_accum`.
- `read` then averages the values added during accumulate and then multiples by a `calibration` variable as well as adding offsets.
- Compass is used by state estimation backends (like AP_AHRS) which call `calculate_heading`, `get_field`, and `get_declination`.

#### IMU
- Maintains two `Vector3f` variables - `_accel` and `_gyro`, both accessible via `get_accel` and `get_gyro`.
- Also maintains two offsets vectors which are just simple additions to the readings, `_accel_offset` and `_gyro_offset`, again, both accessible via `get_gyro_offsets` and `get_accel_offsets`
- Maintains an `_accel_scale` variable which is multiplied by the reading.
- While the `update` method actually sets the `_accel_` and `_gyro` values, there is a timer/interrupt method that is registered with the HAL and is called when new data is available over SPI. This sets `_gyro_sum` and `_accel_sum` and `sum_count`
- Initialisation is done via call to ::init in startup.
- Accelerometer is in m/s/s and Gyrometer is in rad/s.

#### Barometer
- Like the IMU, the raw values are ready during a timer/interrupt and a call to barometer `read` will actually perform the calculation which is detailed in the `_calculate` method
- Pressure reading is retrieved via `get_pressure` which returns a value in units of `millibar*100`
- Temperature is retrieved via `get_temperature` which returns a value in units of `degrees*100`
- `AP_Baro` also has a standard `get_altitude` method which uses the barometer pressure reading (and ground pressure from calibration/takeoff) to estimate an altitude

#### GPS
- Initialisation during startup via call to ::init
- `::update` functions called at 50Hz from scheduler. This calls `read` which processes the byte stream.
- `_parse_gps` is a private functions which is called from `read` and updates latitude, longitude, altitude_cm, ground_speed_cm, \_vel_north, \_vel_east, \_vel_down.

[DONE] DataFlash
- Use `DataFlash_File` for Linux
- Use `DataFlash_APM2` for AVR

Sensor readings 
- Vector3f accel (3)
- Vector3f gyro  (3)
- float pressure (1)
- float temperature (1)
- Vector3f field (3)
- Vector3f lat/lon/alt (3)
- Vector3f NED vel (3)

