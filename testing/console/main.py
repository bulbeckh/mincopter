

import curses
import asyncio
from queue import SimpleQueue

from protocol import serial_task
from ui import ui_task, key_task
from state import StateManager
from handler import NavHandler, NoneHandler, LogHandler

async def main(stdscr):
	curses.curs_set(0)
	## This sets the screen to refresh non-blocking
	stdscr.nodelay(True)
	stdscr.clear()

	cmd_queue = asyncio.Queue()
	serial_queue = SimpleQueue()

	## TODO Terrible
	sm = StateManager(
		NavHandler({
			'l1': NavHandler({
				'logs': LogHandler(queues=(serial_queue, cmd_queue)), 
				'None': NoneHandler(queues=(serial_queue, cmd_queue)),
				'AlsoNone': NoneHandler(queues=(serial_queue, cmd_queue))
				}, queues=(serial_queue, cmd_queue)),
			'l2': NoneHandler(queues=(serial_queue, cmd_queue)),
			'l3': NoneHandler(queues=(serial_queue, cmd_queue))
		}, queues=(serial_queue, cmd_queue))
	)

	## Need to add a third task that handles routing of serial messages to the correct nodes
	tasks = [
		asyncio.create_task(serial_task(serial_queue, cmd_queue)),
		asyncio.create_task(ui_task(stdscr, sm, serial_queue, cmd_queue)),
		asyncio.create_task(key_task(stdscr, sm))
	]

	await asyncio.gather(*tasks)

if __name__=="__main__":
	curses.wrapper(lambda stdscr: asyncio.run(main(stdscr)))
