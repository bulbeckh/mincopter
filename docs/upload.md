
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
