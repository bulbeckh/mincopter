# ac-dev mincopter temp repo

### `Changes since hard-fork (ArduPilot-3.2.1)`
- Moved from .pde files to .cpp and header files
- Migrated build system to cmake
- Removed unused/extra libraries (see below table)
- Removed manual flight modes and functionality
- Removed GCS communication and Mavlink protocol support

### What is the aim?
To have a minimal, modular, and optimised UAV QuadCopter runtime that can support multiple architectures (AVR, ARM, x86-64). It should be modular enough to support multiple different state estimation/representation (DCM, Quaternion) and sensor fusion libraries (NavEKF, InertialNav) and control algorithms (PID, MPC, RL).

### What is target state for completion?
- HAL (Hardware Abstraction Layer) with backends for Linux and AVR. (Support for ARM in later release)
- Flight controller libraries for PID control and MPC control. (Support for RL in later release)
- Asynchronous console interface (client application written in Python) for retrieving logs and streaming live sensor and variable readings
- Actual hardware re-implementation of drone board with different sensors to showcase HAL benefit

### `TODO`

Control
- [ ] Move to a single PID controller class
- [ ] Implement NavEKF3 in mincopter
- [ ] Implement MPC
- [ ] Move code to btree and implement btree call in `fast_loop`

HAL
- [ ] Test Rust implementations of some functions
- [ ] Add Rust HAL using Embedded-HAL

Communications
- [ ] Design console functionality (log retrieval, live sensor readings)
- [x] Build console client (in Python)
- [x] Build console functionality (via AP\_Menu)
- [ ] Implement truly asynchronous console interface
- [x] Build console `echo` script to echo console messages to stdout
- [ ] Merge serial.h and menu.h into a single serial interface unit
- [ ] Implement custom logging messages
- [ ] Write script for easy log retrieval, parsing, and storage in python

Performance
- [x] Write code to profile function runtime
- [ ] Compare executable size and storage regions (text, data, bss) with original ac-3.2.1

Testing & Development
- [ ] Add SITL via AP\_HAL\_Linux
- [ ] Add dockerfile for unified development environment


### Performance Testing

#### Serial Printing
From initial testing
- ~84us for printf\_P overhead and ~85us per bit transmitted

| Function | Execution Limit (us) | Actual |
| --- | --- | --- |
| update_GPS | 900 | 58 | 
| read_batt_compass | 720 | 250 | 
| update_altitude | 1000 | 574 | 
| read_compass | 420 | 547 |
| read_baro | 250 | 6 | 
| one_hz_loop | 420 | 58 | 
| dump_serial | 500 | 6 |
| run_cli | 500 | 853 |
| throttle_loop | 450 | 34 |
| run_nav_updates | 800 | 19 |
| update_nav_mode | 400 | 7 |



### Mincopter Binary Analysis
TODO


### Structure
TODO

### Flashing board via wsl
In an Admin PowerShell:
1. `usbipd list`
2. `usbipd bind --busid <bus>`
3. `usbipd attach --wsl --busid <bus>`


## Serial and Log Communications
What do we actually want to dump to serial and log at runtime?

- Flight State Values (orientation, translational/angular velocities, accelerations)
- Raw Sensor Readings
- Control Outputs (Motor Voltages)
- PID Errors


