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
- [ ] Build console client (in Python)
- [ ] Build console functionality (via AP\_Menu)
- [ ] Move code to btree and implement btree call in `fast\_loop`
- [ ] Write code to measure function runtime
- [ ] Merge serial.h and menu.h into a single serial interface unit

### Performance Testing
TODO

### Mincopter Binary Analysis
TODO


### Structure
TODO

### Flashing board via wsl
In an Admin PowerShell:
1. `usbipd list`
2. `usbipd bind --busid <bus>`
3. `usbipd attach --wsl --busid <bus>`

