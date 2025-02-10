

import curses
import asyncio
from queue import SimpleQueue

from protocol import serial_task
from ui import ui_task

async def main(stdscr):
	cmd_queue = asyncio.Queue()
	serial_queue = SimpleQueue()

	## Need to add a third task that handles routing of serial messages to the correct nodes
	tasks = [
		asyncio.create_task(serial_task(serial_queue, cmd_queue)),
		asyncio.create_task(ui_task(stdscr, serial_queue, cmd_queue))
	]

	await asyncio.gather(*tasks)

if __name__=="__main__":
	curses.wrapper(lambda stdscr: asyncio.run(main(stdscr)))
