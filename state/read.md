AP\_AHRS overview

## Key methods

init()
update()


## Key fields

float roll, pitch, yaw;
int32\_t roll\_sensor, pitch\_sensor, yaw\_sensor;

```c
Vector3f _omega_P;                          // accel Omega proportional correction
Vector3f _omega_yaw_P;                      // proportional yaw correction
Vector3f _omega_I;                          // Omega Integrator correction
Vector3f _omega;                            // Corrected Gyro_Vector data
```


The only thing really used by the rest of mincopter is the three yaw_sensor, pitch_sensor, and roll_sensor variables.

Also ahrs.get_accel_ef().

The flight loop will call ahrs.update() periodically to update state.


## AHRS Update loop

1. Matrix update - rotates matrix using omega, omega_I, omega_P and omega_yaw_P
2. Normalize the DCM matrix (\_dcm\_matrix)
3. Drift correction - using drift_correction and drift_correction_yaw
4. Call the euler_angles to convert roll,pitch,yaw to centi-degrees (roll_sensor, pitch_sensor, yaw_sensor)




