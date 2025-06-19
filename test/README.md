### MinCopter Testing


#### Unit
Testing of specific devices both simulated and physical. For simulation, we need a gazebo environment sdf file to be setup for the purposes
of testing. For physical, we need a target architecture specified so that we can upload the test executable. We will also need a method of mocking the environment for the physical sensor tests.

#### Integration
Testing of integration between two or more modules i.e. the state requires sensor readings and the inertial navigation. Another example is the control and planning interfaces.

#### System
Testing of entire system in flight


