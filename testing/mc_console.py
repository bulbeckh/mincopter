''' An serial interface between local machine and the mincopter instance.

Simple 2-level state-machine that sends and receives commands between ArduCopter

Currently, this will be used to retrieve and get information about the onboard logs but in future should
be able to abstract all commands and act as a comprehensive interface to access onboard state and sensor
readings.

Each line in the 2nd level is associated with a command that is sent to the mincopter.

After a command is sent, the menu blocks and waits for a sequence of responses that ends with an 'END0' line

Logging
	- Show logs : Moves to a 3rd level which shows each log and its associated size

The State of the console can either be in the navigation (navigating between screens) or it can be on a 
particular screen (i.e. the screen used for showing and retrieving log messages.

'''

import serial
import time
import re
import curses
import time
import random

class StateManager:
	'''Responsible for managing navigation between screens and routing data between the different
		handler classes'''

	def __init__(self, navtree):
		self.tree = navtree
		self.path = []
		
		# This is the variable that is updated in response to keypresses
		# Starts out at the root of the tree
		self.current_state = self.tree

	def respond_keypress(self, keypress):
		'''Updates state according to keypresses'''
		## NOTE I actually think this should just forward the keypress to the screen if it is NOT a navigation screen

		response = self.current_state.keypress(keypress)
		
		if (response==None):
			return
		elif (response=='back'):
			## handle back up hierarchy
			## TODO
			pass
		elif (response=='forward'):
			## descend hierarchy
			## TODO
			pass
		

''' Handlers (Log, Navigation)

Each handler class must implement the following methods:

	def run(self):
		pass

	def draw(self, stdscr):
		pass

	def keypress(self, key):
		pass

'''

class NoneHandler:
	'''The most basic handler class. Does not do anything'''
	def __init__(self):
		return

	def run(self):
		return

	def draw(self, stdscr):
		return

	def keypress(self, key):
		return

class Log:
	'''Represents a single log entry in the LogHandler'''
	def __init__(self, lognum : int, logsize : int, content : bytes):
		self.lognum = lognum
		self.logsize = logsize
		self.content = content

class LogHandler:
	'''Responsible for parsing and storing log messages'''
	def __init__(self):
		self.n_message=0
		self.logs = []

	def parse_entry(self, msg):
		'''msg is the string sent from the mincopter

		For logs, this is of the format "SL00-%d-%d"
		'''
		temp = msg.split('-')
		log_num = temp[1]
		log_size = temp[2]
		
		## Add a new entry into the list
		self.logs.append(Log(log_num, log_size, None))

	def run(self):
		'''Commence communication between console and mincopter to request logs and
		parse responses'''
		pass

	def keypress(self, key):
		pass

	def draw(self, stdscr):
		pass

class NavHandler:
	'''Responsible for navigation to a different screen'''
	def __init__(self, navtree):
		self.tree = navtree
		self.line = 0
	
		## These are the rows of the navigation
		self.rows = list(navtree.keys())

	def keypress(self, key):
		if key==curses.KEY_UP:
			self.line = max(self.line-1, 0)
			return None
		elif key==curses.KEY_DOWN:
			self.line = min(self.line+1, len(self.rows)-1)
			return None
		elif key==curses.KEY_BACKSPACE:
			return 'back'
		elif key==curses.KEY_ENTER or key==10 or key==13:
			return 'forward'

	def draw(self, stdscr):
		ctx = {
			'line': self.line,
			'rows': self.rows
		}
		draw_navigation(stdscr, ctx)
		

''' Drawing
Functions for updating the screen with the content from a manager
'''
def draw_navigation(stdscr, ctx):
	'''Draw a navigation screen, given a navigation context (ctx) that contains elements'''
	stdscr.clear()

	height, width = stdscr.getmaxyx()

	## Add heading
	stdscr.addstr(0, 0, "Item", curses.A_BOLD)

	## Add elements to screen
	for i in range(0,len(ctx['rows'])):
		if i == ctx['line']:
			stdscr.addstr(i+1,0, ctx['rows'][i].ljust(width), curses.A_REVERSE)
		else:
			stdscr.addstr(i+1,0, ctx['rows'][i])

	stdscr.refresh()

def draw_log_screen(stdscr, lh : LogHandler):
	'''Draw a list of logs on the screen'''
	stdscr.clear()

	height, width = stdscr.getmaxyx()
	
	## Add header
	stdscr.addstr(0, 0, "Log Number", curses.A_BOLD)
	stdscr.addstr(0, 20, "Log Size (bytes)", curses.A_BOLD)

	## Add each log
	for i, e in enumerate(lh.logs):
		## TODO pad the string two align with heading widths
		stdscr.addstr(i+1, 0, f'{e.lognum} {e.logsize}')

	stdscr.refresh()

def read_response():
	'''Read a stream of serial messages until the 'END' message is received'''

	while(True):
		resp = ser.readline().decode('utf-8')

		if content=="END0":
			return
		else:
			lghandler.add(resp)

def send_command():
	'''Call a command and wait for response from mincopter'''
	ser.write('showlogs\n'.encode('utf-8'))
	
	read_response()

def curses_main(stdscr):
	curses.curs_set(0)
	curses.noecho()
	#key = None
	
	'''
	Part 1. Use the StateManager to get the current screen drawing method and then call it
	Part 2. Wait for keypresses to trigger state changes and then pass the pressed key onto the handler
	'''
	
	sm = StateManager(
		NavHandler({
			'l1': NavHandler({
				'logs': LogHandler(), 
				'None': NoneHandler(),
				'AlsoNone': NoneHandler()
				}),
			'l2': NoneHandler(),
			'l3': NoneHandler()
		})
	)

	while(True):
		## Trigger drawing of current state
		sm.current_state.draw(stdscr)
		
		## Handle keypresses
		key = stdscr.getch()
		sm.respond_keypress(key)


if __name__=="__main__":

	## Establish connection over serial port
	'''
	while(True):
		try:
			ser = serial.Serial('/dev/ttyACM0', 115200)
			print("Connection found")
			break
		except serial.SerialException:
			print("Waiting....")
			time.sleep(0.5)
	'''
	
	## Run the top-level curses loop
	curses.wrapper(curses_main)

