## x86-64 (Linux) Toolchain File

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(CMAKE_C_COMPILER x86_64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER x86_64-linux-gnu-g++)
set(CMAKE_C_FLAGS "-march=x86-64 -O2 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "-march=x86-64 -O2 -Wall -Wextra")

message(STATUS "Loaded x86_64 toolchain file")


