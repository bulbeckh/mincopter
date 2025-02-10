import asyncio
import serial_asyncio

class SerialClient(asyncio.Protocol):
    def __init__(self, queue):
        self.queue = queue  # Queue to pass complete lines to UI
        self.transport = None
        self.buffer = bytearray()  # Buffer for incomplete messages

    def connection_made(self, transport):
        self.transport = transport
        print("Serial connection established")

    def data_received(self, data):
        self.buffer.extend(data)  # Append received bytes to buffer

        while b'\n' in self.buffer:  # Check if there's a full line
            line, _, self.buffer = self.buffer.partition(b'\n')  # Extract line
            message = line.decode().strip()  # Decode and strip whitespace
            self.queue.put_nowait(message)  # Send to UI task

    def send_command(self, command):
        if self.transport:
            self.transport.write(command.encode() + b'\n')  # Ensure newline


async def serial_task(cmd_queue):

	loop = asyncio.get_running_loop()

	transport, protocol = await serial_asyncio.create_serial_connection(
		loop, lambda: SerialClient(), "/dev/ttyACM0", baudrate=115200
	)
	
	while True:
		command = await cmd_queue.get()
		protocol.send_command(command)

