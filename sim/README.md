### Simulation Interface to GZ MinCopter Simulation
 
Because we are using a 'soft' HAL abstraction/interface, one approach we can take is to add an extra 'sim' object to the Generic HAL so that when and only when we choose 'Generic' (which will always be using the simulated sensors), instead of calling the hal.i2c and hal.uart comms, we can call the sim to directly retrieve objects.


