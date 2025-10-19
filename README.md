<picture>
    <img src="./docs/mincopter-logo.png">
</picture>

**MinCopter** is a modular end-to-end UAV quadrotor flight system, optimized for speed and size. Designed to work across multiple backend architectures, **MinCopter** contains high-performance planners, controllers, and state estimation libraries. **MinCopter** uses multiple abstraction layers, allowing users to easily switch between controller, planner, and state estimation implementations. Through use of a hardware abstraction layer, both the physical sensors and the underlying MCU are decoupled from the flight software.

## Build Status
Build status of each supported target microcontroller

| Target | Architecture | Size | Util | Compilation |
| -- | -- | -- | -- | -- |
| `mega2560` | `avr6` |  55K | - |  passed, 3/3 built, 868 warnings |
| `mega2561` | `avr6` |  - | - |  failed, 0/3 built, 397 warnings |
| `mega1280` | `avr5` |  55K | - |  passed, 3/3 built, 868 warnings |
| `mega1281` | `avr5` |  - | - |  failed, 0/3 built, 397 warnings |
| `stm32f407` | `arm-v7` |  222K | - |  passed, 3/3 built, 945 warnings |
| `stm32f405` | `arm-v7` |  222K | - |  passed, 3/3 built, 945 warnings |

## FAQs
**What is the aim of MinCopter?**
To have a minimal, modular, and optimised UAV QuadCopter flight software that can run on multiple architectures (AVR, ARM, x86). MinCopter has implementations of multiple state representations (DCM, Quaternion), state estimation/sensor fusion (EKF, Complementary filters) and control algorithms (PID, MPC).

**Why is it called MinCopter?**
The primary goal of MinCopter is to be minimal (in terms of size) so as to run on compute-constrained and storage-constrained microcontrollers.

**How is this different to ArduCopter and other flight softwares (PX4, Betaflight, Papparazzi UAV)?**
While other flight softwares support specific flight controllers (CUAV, ARKV6X, Pixhawk, ..), MinCopter aims to provide implementations for specific microcontrollers (e.g. stm32f4 series, microchip megaAVR series, ..) while leaving choice of sensors/peripherals and board design to the user.

MinCopter started as a hard fork from ArduCopter, specifically `ardupilot-3.2.1`. A few of the major changes:
- Migrated build system to cmake from 'Arduino' build environment (.pde files etc.)
- Removed manual flight modes
- Removed Mavlink protocol support
- New sensor drivers

## Dependencies
Because of it's modularity, the dependencies of MinCopter change heavily between targets. Cross-compilation on a linux host is only supported for now.

| Target Architecture | Dependencies |
| --- | --- |
| `avr` | `avr-libc`, `binutils-avr`, `gcc-avr`, `avrdude` (for flashing) |
| `arm` | `binutils-arm-none-eabi`, `gcc-arm-none-eabi`, `stlink-tools` **TODO** Add submodule for arm CMSIS, vendor BSPs and stm32 HAL (if used) |
| `linux/generic` | `gz-harmonic` **TODO** [See gz-harmonic install](https://gazebosim.org/docs/harmonic/install/) for instructions on how to install |
| `linux/rpi` | **TODO** |

## Building
1. Before building, quite a bit of configuration must be specified. In the root directory, there is a template specification file `mc-config.cmake` that should be copied to the build directory
```bash
mkdir ./build
cp mc-config.cmake ./build
```

2. Inside the config file, we must specify the sensor devices that we are using, the state estimation library, and the controllers/planners. If we are using a the simulation environment (gazebo) then we also specify some configuration parameters here.

3. Run CMake, specifying the target architecture. A full list is in the root `CMakeLists.txt`. Running cmake without the TARGET\_ARCH variable also prints a list of supported targets.
```bash
cd build
cmake .. -DTARGET_ARCH=<target_architecture>
make -j4
```

## Documentation 
See `docs/` for more documentation.

## Authors
MinCopter is forked from [ArduPilot/ArduCopter-3.2.1](https://github.com/ArduPilot/ardupilot/tree/ArduCopter-3.2.1) - see the repository for maintainers/contributors, some of which are used in MinCopter. Unless otherwise specified, all code written by Henry Bulbeck [@bulbeckh](https://github.com/bulbeckh).

## Structure
| Directory | Contents | 
| --- | --- | 
| dev/ | Sensor (device) drivers and interfaces for GPS, Barometer, IMU, ADC, Compass, ESCs, and External Storage (Flash, SD) |
| lib/ | Common libraries for math and logging |
| control/include/controllers | Controllers for computation of control output based on desired state computed by planner. Implementations of **PID (cascaded)**, **LQR**, and **MPC (WIP)** controllers |
| control/include/planners | Planner implementations for computing higher-level goals/trajectories |
| state/ | State estimation and sensor-fusion libraries including **ekf** and complementary filters. A standard 12-state representation (position, attitude, and derivatives) is computed by most state estimators |
| arch/ | HAL Interface and implementations for each supported microcontroller. Includes `arch/arm`, `arch/avr`, and `arch/linux` |
| mincopter/ | Contains initialisation code, `main` function, scheduling of non-core flight functions and flight loop |
| ap-gz/ | Gazebo models and plugins used by MinCopter. Forked from [ArduPilot/ardupilot\_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) with modifications |
| docs/ | Documentation and state graphing libraries for logfiles |
| test/ | Tests (unit/integration) for the HAL, state, and device drivers |


