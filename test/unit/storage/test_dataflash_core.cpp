
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include "DataFlash.h"
#include "log.h"


// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

struct __attribute__((__packed__)) LogTestStructure {
	LOG_PACKET_HEADER;
	uint8_t b1;
	uint8_t b2;
};


// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(DataFlash_Class& df)
{
	/* We test the following dataflash functionality:
	 *
	 * DumpPageInfo
	 * ShowDeviceInfo
	 * ListAvailableLogs
	 *
	 * LogReadProcess
	 *
	 * get_log_info
	 * get_log_data
	 * get_log_boundaries
	 * get_num_logs
	 *
	 * WriteBlock
	 *
	 */

	// Initialisation
	
	// log_structure variable is declared in DataFlash.h
	df.Init(log_structure, 22);

	df.StartNewLog();
	df.EnableWrites(true);

	struct LogTestStructure ltest = {
		LOG_PACKET_HEADER_INIT(0x0F),
		b1 : 0x9A,
		b2 : 0x34
	};

	// Write block
	df.WriteBlock(&ltest, sizeof(ltest));
	hal.console->printf("Wrote log \r\n");

	for (uint16_t i=0;i<1e6;i++) {

		hal.console->printf("[TEST] 1. Listing available logs\r\n");
		df.ListAvailableLogs(hal.console);

		hal.console->printf("[TEST] 2. Show device info\r\n");
		df.ShowDeviceInfo(hal.console);

		hal.console->printf("[TEST] 3. Dump page info\r\n");
		//df.DumpPageInfo(hal.console);

		hal.console->printf("[TEST] 4. Get number of logs\r\n");
		hal.console->printf("# logs:%u\r\n", df.get_num_logs());

		hal.console->printf("[TEST] 5. Find last log\r\n");
		hal.console->printf("last log:%u\r\n", df.find_last_log());

		hal.console->printf("[TEST] 6. Print log details\r\n");

		uint16_t last_log = df.find_last_log();
		uint16_t lcount = df.get_num_logs();

		// Iterate over each log and print specs to console
		while (lcount) {
			uint32_t lsize, ltime;
			df.get_log_info(last_log-lcount+1, lsize, ltime);

			uint16_t lstart, lend;
			df.get_log_boundaries(last_log-lcount+1, lstart, lend);

			hal.console->printf("Log %u, size=%lu, time=%lu, page_start=%u, page_end=%u\r\n", last_log-lcount+1, lsize, ltime, lstart, lend);

			hal.console->printf("Log contents:\r\n");
			df.LogReadProcess(last_log-lcount+1, lstart, lend, NULL, hal.console);

			lcount--;
		}

		//hal.console->printf("[TEST] 7. Read log\r\n");
		

		//hal.console->printf("[TEST] 8. Erase logs\r\n");
		//df.EraseAll();



		// Wait for 5 seconds
		hal.scheduler->delay(5000);
	}


	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// TODO NOTE We would never have an instance where a hardware target has all types of IMUs should maybe
	// remove the MC_TEST_IMU_ALL flag/option

#ifdef MC_STORAGE_FILE
	// No default path
	DataFlash_File df("/tmp/mincopter/test-log-out.txt");
#elif MC_STORAGE_DATAFLASH
	DataFlash_APM2 df;
#elif MC_STORAGE_EMPTY
	DataFlash_Empty df;
#endif

	// Run tests
	run_unit_tests(df);

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

