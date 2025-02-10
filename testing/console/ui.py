
from helper import eprint

import curses
import asyncio

async def ui_task(stdscr, serial_queue, cmd_queue):
	curses.curs_set(0)
	stdscr.nodelay(True)
	stdscr.clear()
	
	while True:
		stdscr.addstr(0,0,'here')
		stdscr.refresh()
		
		## is this blocking??
		#key = stdscr.getch()

		while not serial_queue.empty():
			message = serial_queue.get()
			eprint(f'Message: {message}')

		await asyncio.sleep(0.1)


