

### List of sensors

[DONE] DataFlash
- Use `DataFlash_File` for Linux
- Use `DataFlash_APM2` for AVR

[DONE] ADC (for battery voltage readings)
- xx

[DONE] Compass
- Maintains a `_field` variable which stores magnetic field readings from the compass
- Both `read` and `accumulate` are virtualised. Calls to `accumulate` in turn calls `read_raw` which reads raw sensors measurements and will populate the `_mag_<x,y,z>` variable. During `accumulate`, these are stored in `_mag_<x,y,z>_accum`
- `read` then averages the values added during accumulate and then multiples by a `calibration` variable as well as adding offsets.
- `calculate_heading` will use the current orientation (via `dcm_matrix`) and the field readings to determine a compass heading

[DONE] InertialSensor (IMU)
- Maintains two `Vector3f` variables - `_accel` and `_gyro`, both accessible via `get_accel` and `get_gyro`
- Also maintains two offsets vectors which are just simple additions to the readings, `_accel_offset` and `_gyro_offset`, again, both accessible via `get_gyro_offsets` and `get_accel_offsets`
- Maintains an `_accel_scale` variable which is multiplied by the reading.
- While the `update` method actually sets the `_accel_` and `_gyro` values, there is a timer/interrupt method that is registered with the HAL and is called when new data is available over SPI. This sets `_gyro_sum` and `_accel_sum` and `sum_count`
- <--section about initialisation and calibration-->
- Accelerometer is in m/s/s and Gyrometer is in rad/s

[DONE] Barometer
- Like the IMU, the raw values are ready during a timer/interrupt and a call to barometer `read` will actually perform the calculation which is detailed in the `_calculate` method
- Pressure reading is retrieved via `get_pressure` which returns a value in units of `millibar*100`
- Temperature is retrieved via `get_temperature` which returns a value in units of `degrees*100`
- `AP_Baro` also has a standard `get_altitude` method which uses the barometer pressure reading (and ground pressure from calibration/takeoff) to estimate an altitude

[DONE] GPS
- Uses `init`, `read`, and `update` functions to set state variables
- Maintains a `latitude`, `longitude` and `attitude` variable as well as a north-east-down velocity measurement

Sensor readings 
- Vector3f accel (3)
- Vector3f gyro  (3)
- float pressure (1)
- float temperature (1)
- Vector3f field (3)
- Vector3f lat/lon/alt (3)
- Vector3f NED vel (3)


AHRS (DCM)



