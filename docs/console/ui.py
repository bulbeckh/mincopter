
from helper import eprint


import curses
import asyncio

async def key_task(stdscr, sm):
	while (True):
		## Blocking
		key = stdscr.getch()

		if key!=-1:
			sm.respond_keypress(key)

		## I guess this calls the key_task at 10Hz
		await asyncio.sleep(0.01)

async def ui_task(stdscr, sm, serial_queue, cmd_queue):

	while True:
		## Trigger drawing of current state
		sm.current_state.draw(stdscr)

		while not serial_queue.empty():
			message = serial_queue.get()

		## 10Hz
		await asyncio.sleep(0.1)
