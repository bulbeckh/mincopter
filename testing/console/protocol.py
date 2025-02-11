import asyncio
import serial_asyncio

from helper import eprint

class SerialClient(asyncio.Protocol):
    def __init__(self, queue):
        self.queue = queue  # Queue to pass complete lines to UI
        self.transport = None
        self.buffer = bytearray()  # Buffer for incomplete messages

    def connection_made(self, transport):
        self.transport = transport
        print("Serial connection established")

    def data_received(self, data):
        eprint(f'receiving data {len(data)}')
        self.buffer.extend(data)

        '''Wait until a newline characters if found, otherwise keep appending bytes'''
        while b'\n' in self.buffer:  # Check if there's a full line
            line, _, self.buffer = self.buffer.partition(b'\n')  # Extract line
            message = line.decode('utf-8')
            self.queue.put(message)  # Send to UI task

    def send_command(self, command):
        if self.transport:
            self.transport.write(command.encode('utf-8') + b'\n')  # Ensure newline


async def serial_task(serial_queue, cmd_queue):

	loop = asyncio.get_running_loop()

	'''This creates the serial connection and effectively 'registers' callbacks
	like `data_received` to handle incoming data.
	'''
	transport, protocol = await serial_asyncio.create_serial_connection(
		loop, lambda: SerialClient(serial_queue), "/dev/ttyACM0", baudrate=115200
	)
	
	while True:
		command = await cmd_queue.get()
		protocol.send_command(command)

