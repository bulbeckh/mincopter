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
import sys

def eprint(*args, **kwargs):
	print(*args, file=sys.stderr, **kwargs)

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

		response, node = self.current_state.keypress(keypress)
		
		if (response==None):
			return
		elif (response=='back'):
			## handle back up hierarchy
			if (len(self.path)==0):
				## At root node
				return 
			else:
				self.current_state = self.path.pop()

		elif (response=='forward'):
			## descend hierarchy
			self.path.append(self.current_state)
			self.current_state = node
			return
		

''' Handlers (Log, Navigation)

Each handler class must implement the following methods:

	def run(self):
		pass

	def draw(self, stdscr):
		pass

	def keypress(self, key):
		pass

	def parse(self, msg)
		pass

'''

class NoneHandler:
	'''The most basic handler class. Does not do anything'''
	def __init__(self):
		return

	def run(self):
		return

	def draw(self, stdscr):
		draw_none(stdscr, None)
		return

	def keypress(self, key):
		if key==curses.KEY_BACKSPACE:
			return ('back', None)
		else:
			return (None, None)

	def parse(self, msg):
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

		self.running = False

	def parse(self, msg):
		'''msg is the string sent from the mincopter

		For logs, this is of the format "SL00-%d-%d"
		'''
		eprint('parse method called from loghandler')
	
		if (msg[0:4] != 'SL00'):
			eprint(f'non-std message found: {msg}')
			return

		temp = msg.split('-')
		log_num = temp[1]
		log_size = temp[2]
		
		## Add a new entry into the list
		self.logs.append(Log(log_num, log_size, None))

	def run(self):
		'''Commence communication between console and mincopter to request logs and
		parse responses'''

		eprint("Sending showlogs commands")
		send_command('showlogs')
		
		while(True):
			resp = ser.readline().decode('utf-8')
			if resp=="END0":
				return
			else:
				self.parse(resp)

		return

	def keypress(self, key):
		if key==curses.KEY_BACKSPACE:
			return ('back', None)
		else:
			return (None, None)

	def draw(self, stdscr):
		'''On first call to draw, start the communication process'''
		if (self.running==False):
			self.runnning = True
			self.run()
	
		ctx = {
			'numlogs': len(self.logs)
		}
		draw_logs(stdscr, ctx)

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
			return (None, None)
		elif key==curses.KEY_DOWN:
			self.line = min(self.line+1, len(self.rows)-1)
			return (None, None)
		elif key==curses.KEY_BACKSPACE:
			return ('back', None)
		elif key==curses.KEY_ENTER or key==10 or key==13:
			## TODO Fix this - horrible
			return ('forward', self.tree[list(self.tree.keys())[self.line]])

	def draw(self, stdscr):
		ctx = {
			'line': self.line,
			'rows': self.rows
		}
		draw_navigation(stdscr, ctx)

	def parse(self, msg):
		return
		

''' Drawing
Functions for updating the screen with the content from a manager
'''

def draw_none(stdscr, ctx):
	stdscr.clear()
	stdscr.addstr(0,100, 'THIS IS A NONE SCREEN', curses.A_REVERSE)
	stdscr.addstr(0,100, 'THIS IS A NONE SCREEN')
	stdscr.refresh()
	return

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

def draw_logs(stdscr, ctx):
	'''Draw a list of logs on the screen'''
	stdscr.clear()

	height, width = stdscr.getmaxyx()
	
	stdscr.addstr(0, 0, f'Number of logs: {ctx["numlogs"]}', curses.A_BOLD)

	stdscr.refresh()

def send_command(command_name):
	'''Call a command and wait for response from mincopter'''
	if (command_name not in ['showlogs']):
		## TODO Log error
		pass
	else:
		ser.write(f'{command_name}\n'.encode('utf-8'))
	
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

		eprint(ser.inWaiting())

		#resp = ser.readline().decode('utf-8')
		#eprint(f'msg: {resp}')

if __name__=="__main__":

	## Establish connection over serial port
	while(True):
		try:
			ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
			eprint("Connection found")
			break
		except serial.SerialException:
			eprint("Waiting....")
			time.sleep(0.5)
	
	## Run the top-level curses loop
	curses.wrapper(curses_main)

