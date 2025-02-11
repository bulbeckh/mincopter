
from helper import eprint
from state import StateManager

from handler import NavHandler, NoneHandler, LogHandler

import curses
import asyncio

async def ui_task(stdscr, serial_queue, cmd_queue):
	curses.curs_set(0)
	## This sets the screen to refresh non-blocking
	#stdscr.nodelay(True)
	stdscr.clear()
	

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

	i=0
	while True:
		#i+=1 
		#if (i%1000==0):
		eprint(i)

		## Trigger drawing of current state
		sm.current_state.draw(stdscr)
		#stdscr.refresh()

		## Respond to keypress
		key = stdscr.getch()
		sm.respond_keypress(key)
		

		while not serial_queue.empty():
			message = serial_queue.get()
			eprint(f'Message: {message}')

		await asyncio.sleep(0.1)
