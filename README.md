# ac-dev mincopter temp repo

### `Changes since hard-fork (ArduPilot-3.2.1)`
- Moved from .pde files to .cpp and header files
- Migrated build system to cmake
- Removed unused/extra libraries (see below table)
- Removed manual flight modes and functionality
- Removed GCS communication and Mavlink protocol support

### What is target state for completion?
- HAL (written in Rust using embedded HAL)
- Multiple different flight controllers with modular code allowing easy switching
- Console interface (client application written in Python) for things like retrieving logs, obtaining live raw sensor and value readings
- Actual hardware re-implementation of drone board with different sensors to showcase HAL benefit

### `TODO`
- [ ] Move to a single PID controller class
- [ ] Implement NavEKF3 in mincopter
- [ ] Implement MPC
- [ ] Design console functionality (log retrieval, live sensor readings)
- [x] Build console client (in Python)
- [x] Build console functionality (via AP\_Menu)
- [ ] Implement truly asynchronous console interface
- [x] Build console `echo` script to echo console messages to stdout
- [ ] Move code to btree and implement btree call in `fast_loop`
- [x] Write code to profile function runtime
- [ ] Merge serial.h and menu.h into a single serial interface unit
- [ ] Implement custom logging messages
- [ ] Write script for easy log retrieval, parsing, and storage in python
- [ ] Add SITL via AP\_HAL\_Linux
- [ ] Test Rust implementations of some functions
- [ ] Add Rust HAL using Embedded-HAL
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


