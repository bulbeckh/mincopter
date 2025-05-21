## `AP\_AHRS overview



Euler angles (radians)
```c
float roll;
float pitch;
float yaw;
```

integer Euler angles (Degrees * 100)
```c
int32_t roll_sensor;
int32_t pitch_sensor;
int32_t yaw_sensor;
```

How often our attitude representation has gone out of range
`uint8_t renorm_range_count`

How often our attitude representation has blown up completely
`uint8_t renorm_blowup_count`

`float _kp_yaw`
`float _kp`
`float gps_gain`














## AHRS Update loop

1. Matrix update - rotates matrix using omega, omega_I, omega_P and omega_yaw_P
2. Normalize the DCM matrix (\_dcm\_matrix)
3. Drift correction - using drift_correction and drift_correction_yaw
4. Call the euler_angles to convert roll,pitch,yaw to centi-degrees (roll_sensor, pitch_sensor, yaw_sensor)




