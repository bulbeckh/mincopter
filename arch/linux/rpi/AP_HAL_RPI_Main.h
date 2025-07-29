
#pragma once

#define AP_HAL_MAIN() extern "C" {\
int main (int argc, char * const argv[]) {        \
	hal.init(argc, argv);			\
        setup();\
        hal.scheduler->system_initialized(); \
        for(;;) loop();\
        return 0;\
    }\
    }

