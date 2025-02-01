''' An serial interface between local machine and the mincopter instance.

Simple 2-level state-machine that sends and receives commands between ArduCopter

Currently, this will be used to retrieve and get information about the onboard logs but in future should
be able to abstract all commands and act as a comprehensive interface to access onboard state and sensor readings

Each line in the 2nd level is associated with a command that is sent to the mincopter.

After a command is sent, the menu blocks and waits for a sequence of responses that ends with an 'END0' line

Logging
	- Show logs : Moves to a 3rd level which shows each log and its associated size

'''

import serial
import time
import re
import curses
import time
import random

states = {
	'Logging': [
			'Get log count',
			'Get log'
		],
	'None': [],
	'AlsoNone': []
}

ctx = {
	'top': None,
	'line': 0,
	'rows': None
}

def draw_table(stdscr):
	stdscr.clear()

	height, width = stdscr.getmaxyx()

	stdscr.addstr(0, 0, "TYPE", curses.A_BOLD)
	stdscr.addstr(0, 20, "VAL", curses.A_BOLD)

	## Handle location within the hierarchy
	if ctx['top']==None:
		## In root location
		keys = list(states.keys())
	else:
		keys = states[ctx['top']]
	
	ctx['rows'] = len(keys)

	## Add elements to screen
	for i in range(0,ctx['rows']):
		if i == ctx['line']:
			stdscr.addstr(i+1,0, keys[i].ljust(width), curses.A_REVERSE)
		else:
			stdscr.addstr(i+1,0, keys[i])

	## debug
	## stdscr.addstr(10, 0, f'{ctx["top"]} {ctx["line"]} {ctx["rows"]}')

	stdscr.refresh()


def read_response():
	'''Read a string of data until the 'END' message is received'''
	while(True):
		resp = ser.readline()
		content = resp.decode('utf-8')
		
		if content=="END0":
			return
		else:
			print(content)

class MCCommand:
	def __init__(self, cmd, args):
		pass

	def parse_response():
		pass
	
def send_command():
	'''Call a command and wait for response from mincopter'''
	ser.write('showlogs\n'.encode('utf-8'))
	
	read_response()


def curses_main(stdscr):
	curses.curs_set(0)
	curses.noecho()
	key = None
	
	while(True):
		## Trigger redraw
		draw_table(stdscr)

		## Handle keypresses
		key = stdscr.getch()
		if key==curses.KEY_UP:
			ctx['line'] = max(ctx['line']-1, 0)
		elif key==curses.KEY_DOWN:
			ctx['line'] = min(ctx['line']+1, ctx['rows']-1)
		elif key==curses.KEY_BACKSPACE:
			if ctx['top'] == None:
				## Already at top level
				continue
			else:
				## Traverse back to top level
				ctx['top'] = None
		elif key==curses.KEY_ENTER or key==10 or key==13:
			## Enter, \r, and \n
			if ctx['top'] == None:
				## Descend to 2nd layer by navigating to level of current line
				ctx['top'] = list(states.keys())[ctx['line']]
			else:
				## Run command associated w this level
				print("Executing command")
				send_command()

if __name__=="__main__":
	while(True):
		try:
			ser = serial.Serial('/dev/ttyACM0', 115200)
			print("Connection found")
			break
		except serial.SerialException:
			print("Waiting....")
			time.sleep(0.5)
	
	curses.wrapper(curses_main)

