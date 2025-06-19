<picture>
    <img src="./docs/mincopter-logo.png">
</picture>

**MinCopter** is a modular end-to-end UAV quadrotor flight system, optimized for speed and size. Designed to work across multiple backend architectures, **MinCopter** contains high-performance planners, controllers, and state estimation libraries. **MinCopter** uses multiple abstraction layers, allowing users to easily switch between controller, planner, and state estimation implementations. Through use of a hardware abstraction layer, both the physical sensors and the underlying MCU are decoupled from the flight software.


## Examples
TODO

## Planning
- [ ] KF/EKF Theory and Implementation with linearised model

## mistakes/learnings
- wrong frame for sensors
- incorrect sensor refresh frequency
- not initialising reference setpoints (temperature, pressure, lat/lng)
- did not zero out all PWM channels before sending packet leading to large gimbal/camera joint velocities which caused instability
- did not include simulation send/receive x10 time in fastloop timer

## Performance
TODO

| Backend | Size | CPU Utilisation |
| -- | -- | -- |
| simulation (linux) | xx | xx |
| AVR | xx | xx |
| ARM | xx | xx |

## Baseline against other open source flight controllers
| Project |
| -- |
| ArduPilot |
| PX4 |
| Betaflight |
| iNav |
| Papparazzi UAV |

What about AutoQuad, 3DR, .. - these are hardware vendors no?

Look at dronecode.org for more partners of the drone software ecosystem


## Dependencies
TODO - Add each of the backend compilers (gcc-libc, avr-gcc, gcc-arm-none-eabi)

testing update

## Building (see docs)
1. To use the MPC controller, we need to additionally running the MPC design notebook in control/design/src/RapidQuadrocopterTrajectories/combined-osqp.ipynb
```bash
cd control/design/src/RapidQuadrocopterTrajectories/
jupyter notebook
## Run the combined-osqp.ipynb which will generate the code in cgen/ as well as generate the controller_mpc_constructor.cpp file in control/
```

2. Then build the generated C code and the static OSQP library
```bash
cd cgen/
mkdir build
cd build
cmake ..
make
```
3. Navigate to build/
```bash
cd ./build
```

4. Run CMake, specifying the target architecture. A full list is in ./xx
```bash
cmake .. -DTARGET_ARCH=<target_architecture>
```

5. Run Make from project build directory
```bash
cd build
make -j4 <target>
```

To upload: flashing board via wsl

In an Admin PowerShell:
1. `usbipd list`
2. `usbipd bind --busid <bus>`
3. `usbipd attach --wsl --busid <bus>`

## Documentation 
See docs/

## TODO & Roadmap
TODO

## FAQs
**What is the aim of MinCopter?**
To have a minimal, modular, and optimised UAV QuadCopter runtime that can support multiple architectures (AVR, ARM, x86-64). It should be modular enough to support multiple different state estimation/representation (DCM, Quaternion) and sensor fusion libraries (NavEKF, InertialNav) and control algorithms (PID, MPC, RL).

**Why is it called MinCopter?**
The primary goal of MinCopter is to have a very minimal (in terms of storage) flight software that can run on compute-constrained microcontrollers.

**What is target state for completion?**
- HAL (Hardware Abstraction Layer) with backends for Linux and AVR. (Support for ARM in later release)
- Flight controller libraries for PID control and MPC control. (Support for RL in later release)
- Asynchronous console interface (client application written in Python) for retrieving logs and streaming live sensor and variable readings
- Actual hardware re-implementation of drone board with different sensors to showcase HAL benefit

**How is this different to ArduCopter?**
MinCopter started as a hard fork from ArduCopter, specifically `ardupilot-3.2.1`. A number of changes have been made since.
- Moved from .pde files to .cpp and header files
- Migrated build system to cmake
- Removed unused/extra libraries
- Removed manual flight modes and functionality
- Removed GCS communication and Mavlink protocol support

**NOTE**
I had previously been using ArduPilot-3.1.2 as the 'last supported' firmware for APM2.5, however, I have recently found out that this was just a typo and instead, the actual last supported version is `ArduCopter-3.2.1`.

I have now included `AP_HAL_Linux`, however, the sensors associated with the Linux-based boards are different. I need to implement a pure SITL HAL that is based on Linux but uses a software 'sensor' interface to mimic sensor readings.

**What other projects is MinCopter based on?**
The navigation stack (planners, controllers) is inspired by the ROS2 Nav2 project.

## Contributing
First step to contributing would be to read the developer guide which explains the codebase and how the different modules fit together. It also explains how new planners/controllers/state libraries are written.

## Acknowledgements
Add list of ArduCopter contributors for each state library
Add license

## Structure
| Directory | Contents | 
| --- | --- | 
| dev/ | device (sensor) abstractions and backends including GPS, Barometer, IMU, Flash Storage, ... |
| lib/ | libraries for math, logging, navigation |
| control/controllers | controller implementation for 'local' planning of control output based on desired state computed by planner |
| control/planners | planner implementations for computing higher-level goals and trajectories |
| state/ | state estimation libraries which update copter state object (position, velocity, angular accel/rotation) |
| arch/ | HAL and architecture-specific backends |
| mincopter/ | entry point and initialisation code and scheduling of main flight loops |

### `TODO`
- [ ] Rewrite AP_Scheduler in arch/
- [ ] Add RTOS capability for some microcontrollers
- [ ] Rewrite CMakeLists for new structure
- [ ] Test other ISA backends (ARM) and split codebase
- [ ] Identify areas where architecture can decrease runtime (hardware acceleration)
- [ ] Implement NavEKF3 in mincopter
- [ ] Implement MPC
- [ ] Move code to btree and implement btree call in `fast_loop`
- [ ] Test Rust implementations of some functions
- [ ] Add Rust HAL using Embedded-HAL
- [ ] Design console functionality (log retrieval, live sensor readings)
- [ ] Implement truly asynchronous console interface
- [ ] Merge serial.h and menu.h into a single serial interface unit
- [ ] Implement custom logging messages
- [ ] Write script for easy log retrieval, parsing, and storage in python
- [ ] Compare executable size and storage regions (text, data, bss) with original ac-3.2.1
- [ ] Add simulated sensor backends in sim/ for using w AP\_HAL\_Linux
- [ ] Integrate Gazebo simulation to for sensor interface
- [ ] Add dockerfile for unified development environment
- [x] Move to a single PID controller class
- [x] Build console client (in Python)
- [x] Build console functionality (via AP\_Menu)
- [x] Write code to profile function runtime
- [x] Build console `echo` script to echo console messages to stdout


