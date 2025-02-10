

import curses
import asyncio

from protocl import SerialTask

async def main(stdscr):
	cmd_queue = asyncio.Queue()
	## No serial_queue - will get routed to correct node directly

	tasks = [
		asyncio.create_task(serial_task(cmd_queue)),
		asyncio.create_task(ui_task(stdscr, cmd_queue)
	]

	await asyncio.gather(*tasks)

if __name__="__main__":
	curses.wrapper(lambda stdscr: asyncio.run(main(stdscr)))
