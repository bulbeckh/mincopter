
Currently, we only have an implementation for STM32 arm chips so we add both CMSIS and the vendor-specific packages (like the STM32 HAL drivers and initialisation code). The latter will later reside in stm32/ 

`TODO` We need to add a git submodule to both the ARM CMSIS and the STM32 drivers


For each supported STM32 board, we have a submodule to the git repo containing the CMSIS and BSP. We also need to have the following:


- Linker script: xx
- startup code: Defines the assembly for things like the Reset_Handler
- Interrupt handlers: \*\_it.h and \*\_it.c defining interrupt functions
- HAL Configuration header: Declares which HAL modules we are using (needed for proper linkage)
- syscalls.c and sysmem.c: xx
- STM32 system file: xx


Here is an example for the 



