
#include "telemetry.h"

#include "mcinstance.h"
extern MCInstance mincopter;

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
		nb = mincopter.hal.uartC->read();

		// Prepare buffer for if we have to send acknowledgements - typically 2 bytes only
		uint8_t telem_tx_buffer[] = {0x24, 0x00};

		if (nb!=-1) {

			switch (cmd_state) {
				case 0:
					// Sync byte
					if (nb!=0x24) {
						// Log packet miss and reset state
						mincopter.hal.console->printf("cli pkt miss! resetting %d\r\n", nb);
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
							// TOOD
							remaining = 0;
							break;
						case 0x0E:
							// Disarm request
							cmd_type = 0x0E;

							// Callback
							mincopter_telemetry_command_disarmrequest(NULL);

							remaining = 0;
							break;

						default:
							mincopter.hal.console->printf("Wrong cmd type! resetting\r\n");
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
						// TODO Test command
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
