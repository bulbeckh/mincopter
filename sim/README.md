### Simulation Interface to GZ MinCopter Simulation
 
Because we are using a 'soft' HAL abstraction/interface, one approach we can take is to add an extra 'sim' object to the Generic HAL so that when and only when we choose 'Generic' (which will always be using the simulated sensors), instead of calling the hal.i2c and hal.uart comms, we can call the sim to directly retrieve objects.


We want to log the following information from the gazebo simulation

NOTE: LOGSTREAMER is not currently working

## Simulation information
- int32_t simulation time (s)

- float(3) position (inertial frame)
- float(3) linear velocity (inertial frame)
- float(3) orientation
- float(3) angular velocity

- float(3) accel
- float(3) gyro
- float(3) compass
- float(3) gps (lat,lng,alt)
- float(2) baro (temp, press)


## MinCopter logging
- float(3) position
- float(3) velocity
- float(3) euler angles
- float(3) euler rates
- float(4) control input (force, torques)
- int16_t(4) motor velocities


