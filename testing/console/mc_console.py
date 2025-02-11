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

