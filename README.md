# ac-dev mincopter temp repo

**NOTE**
I had previously been using ArduPilot-3.1.2 as the 'last supported' firmware for APM2.5, however, I have recently found out that this was just a typo and instead, the actual last supported version is `ArduCopter-3.2.1`.

I have now included `AP_HAL_Linux`, however, the sensors associated with the Linux-based boards are different. I need to implement a pure SITL HAL that is based on Linux but uses a software 'sensor' interface to mimic sensor readings.

### `Changes since hard-fork (ArduPilot-3.1.2)`
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
- [ ] Move to a single PID controller class
- [ ] Implement NavEKF3 in mincopter
- [ ] Implement MPC
- [ ] Move code to btree and implement btree call in `fast_loop`
- [ ] Test Rust implementations of some functions
- [ ] Add Rust HAL using Embedded-HAL
- [ ] Design console functionality (log retrieval, live sensor readings)
- [x] Build console client (in Python)
- [x] Build console functionality (via AP\_Menu)
- [ ] Implement truly asynchronous console interface
- [x] Build console `echo` script to echo console messages to stdout
- [ ] Merge serial.h and menu.h into a single serial interface unit
- [ ] Implement custom logging messages
- [ ] Write script for easy log retrieval, parsing, and storage in python
- [x] Write code to profile function runtime
- [ ] Compare executable size and storage regions (text, data, bss) with original ac-3.2.1
- [ ] Add SITL via AP\_HAL\_Linux
- [ ] Add dockerfile for unified development environment

### Upload: Flashing board via wsl
In an Admin PowerShell:
1. `usbipd list`
2. `usbipd bind --busid <bus>`
3. `usbipd attach --wsl --busid <bus>`

