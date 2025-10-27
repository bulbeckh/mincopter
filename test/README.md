## MinCopter Testing



### Unit
Testing of specific devices both simulated and physical. In simulation, we can interact with the simulation by calling the reset method and then setting the appropriate position/attitude/velocity/angularvelocity and then reading the measurement to ensure we have the correct reading.

#### IMU
- Rotate to three different orientations and measure the acceleration. Compare with the expected reference vector.
- Set angular velocity for three different axes and measure the gyroscope. Compare with the expected reference vector.

#### Compass
- Rotate to three different orientations and compare with the expected reference vector. Check that the magnetic field is correct for the current world coordinates.

### GPS
- Set position to reference and record GPS. Update position and then compare GPS measurement change against expected change.
- Set horizontal planar velocity (and vertical to zero) over a few iterations and measure GPS reported velocity. Compare against expected velocity.



#### Integration
Testing of integration between two or more modules i.e. the state requires sensor readings and the inertial navigation. Another example is the control and planning interfaces.

#### System
Testing of entire system in flight



#### List of Tests

`unit/motors`
- Motors spin in correct sequence

