
#include "telemetry.h"

#include "AP_HAL/AP_HAL.h"
extern const AP_HAL::HAL& hal;

void Telemetry::read(uint8_t max_bytes)
{
	/* Design of simple console to read incoming telemetry commands
	 *
	 * Since we call this function at 100Hz, we read no more than 8 bytes of incoming telemetry (uart) streams.
	 * We use a state machine that persists between calls so that we can process some of a stream before yielding.
	 *
	 * # Packet Stream Design & Command API
	 * TODO see mincopter-terminal repo
	 *
	 * We use four variables to capture the state of our state machine.
	 *
	 * cmd_state : Tracks what part of the packet we are expecting next (0 = sync byte, 1 = command type, 2 = args)
	 * cmd_type  : Tracks what command type we are currently parsing. Set after read of 2nd byte and cleared upon error or full packet
	 * remaining : Tracks how many arguments of this command we have left to read. Set after ready of 2nd byte and cleared upon error
	 * 	or full packet
	 * cmd_arg_buffer : Buffer (uint8_t[8]) containing the arguments for each command. No command has more than 8 bytes of arguments
	 * 	so we keep this as fixed size. NOTE This will change in future versions.
	 *
	 * If at any stage of the stream we encounter an error, we reset the state machine and keep reading until we hit the sync byte. After
	 * an error, we should also re-send a hearbeat message as the heartbeat response for the telemetry may have been corrupted in the stream.
	 */

	// TODO Remove - we can just use max_bytes directly
	// We read up to 8 bytes from the console each iteration
	uint8_t read_counter = max_bytes;

	while (read_counter) {
		read_counter--;

		// Read a single byte
		nb = hal.uartC->read();

		// Prepare buffer for if we have to send acknowledgements - typically 2 bytes only
		uint8_t telem_tx_buffer[] = {0x24, 0x00};

		if (nb!=-1) {

			switch (cmd_state) {
				case 0:
					// Sync byte
					if (nb!=0x24) {
						// Log packet miss and reset state
						hal.console->printf("cli pkt miss! resetting %d\r\n", nb);
						cmd_state = 0;
						cmd_type = 0;
						remaining = 0;
						break;
					}

					// Increment to next state
					cmd_state++;
					break;

				case 1:
					// Command Type
					switch (nb) {
						case 0x0A:
							// Receive heartbeat
							cmd_type = 0x0A;
							
							// Expect 1 packet (sequence identifier)
							remaining = 1;
							cmd_state++;
							break;
						case 0x0B:
							// TODO Not required - cmd_type is zeroe-d at end of this function
							// Arm request
							cmd_type = 0x0B;

							// Callback
							mincopter_telemetry_command_armrequest(NULL);

							remaining = 0;
							break;

						case 0x0C:
							// Flight state requested
							cmd_type = 0x0C;

							// Callback
							mincopter_telemetry_command_flightstaterequest(NULL);

							remaining = 0;
							break;

						case 0x0D:
							// Test command
							cmd_type = 0x0D;

							// For the test command, we read 8 bytes of arguments, however, the remaining flag is modified midway through
							//
							// The first two are [command type, number of command arguments]
							//

							cmd_state++;
							remaining = 8;
							break;
						case 0x0E:
							// Disarm request
							cmd_type = 0x0E;

							// Callback
							mincopter_telemetry_command_disarmrequest(NULL);

							remaining = 0;
							break;

						default:
							hal.console->printf("Wrong cmd type! resetting\r\n");
							cmd_state = 0;
							cmd_type = 0;
							remaining = 0;
							break;
					}

					// For some command types, we don't expect any more arguments so we should execute whatever command we need
					// and then reset
					if (!remaining) {
						cmd_state=0;
						cmd_type=0;
					}

					break;

				case 2:
					remaining--;

					// We store arguments in reverse order to avoid having another variable tracking the index of the current argument.
					// Commands should know how many arguments they each take
					argbuffer[remaining] = nb;

					if (cmd_type==0x0A) {
						// Callback
						mincopter_telemetry_command_heartbeatrequest(argbuffer);
					} else if (cmd_type==0x0D) {
						if (remaining==6) {
							// We have parsed the first two arguments of the testrequest command and now need to update the 'remaining'
							// variable to the actual number of expected remaining arguments, which was the previous byte.
							//
							// If this test doesn't actually have any arguments, then argbuffer[6] will be 0x00 and we will set 'remaining'
							// to 0 and the check at the end of this case should cause this switch to break.
							//
							// The structure of argbuffer may be somewhat confusing for the test command. I will illustrate below with
							// a few examples. Regardless of how many arguments our test command actually takes, we allocate an 8-byte buffer
							// called argbuffer. For other telemetry message like the heartbeat, the remaining flag is started at an index
							// with the number of arguments. In the heartbeat case this is 1 and the argbuffer looks like the following:
							//
							// heartbeat message argbuffer
							// seq_id  1     2     3     4     5     6     7
							// [0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
							//
							// With the test command however, we start the argbuffer at the end and then jump when we actually know how
							// many arguments this test command has. For a test command with two test arguments, the buffer looks like:
							//
							// test message argbuffer (test has 2 arguments)
							//  arg1  arg0   2     3     4     5    #arg cmd_id
							// [0x45, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x02, 0xF0]
							//
							// And again with four arguments
							//
							// test message argbuffer (test has 4 arguments)
							//  arg3  arg2  arg1  arg0   4     5    #arg cmd_id
							// [0x45, 0x3F, 0x10, 0x65, 0x00, 0x00, 0x02, 0xF0]
							//

							remaining = ((uint8_t*)argbuffer)[6];
						}

						if (remaining==0) {
							// Callback
							mincopter_telemetry_command_testrequest(argbuffer);
						}

					} else {
						// TODO
						// We should flag here that we have a command type that doesn't take any arguments

						// Reset
						cmd_state = 0;
						cmd_type = 0;
					}
					

					// Reset if we have no more arguments to parse
					if (!remaining) {
						cmd_state = 0;
						cmd_type = 0;
					}

					break;

				default:
					break;
			}
		} else {
			// If nb == -1 then we have reached the end of our input stream and we simply return
			return;
		}
	}



}
