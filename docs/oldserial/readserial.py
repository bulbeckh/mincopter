#/usr/bin/python3

import serial
import time
import re
import curses
import time
import random

''' Fast Loop Monitoring
Fast loop runs at 100hz meaning each loop should run for ~10ms which is ~10000us
'''


packetTypes = {
	'T_RR': 0.0,
	'T_UMOD': 0.0,
	'T_RA': 0.0,
	'T_UT': 0.0,
	'T_UMOT': 0.0,
	'T_RI': 0.0
}

class MC_SerialPacket:
	
	def __init__(self, raw):
		self.raw = raw
		self.text = raw.decode('utf-8')

		parts = self.text.split(':')
		
		self.type = parts[0]
		if (self.type not in packetTypes.keys()):
			self.invalid = True
			#print(f'Invalid serial pack found with type {self.type}')
			return
		else:
			self.invalid = False

		self.val = float(parts[1].strip()[0:-2])

		#print(self.type, self.val, type(self.val))
	
def draw_table(stdscr):
	stdscr.clear()

	stdscr.addstr(0, 0, "TYPE", curses.A_BOLD)
	stdscr.addstr(0, 20, "VAL", curses.A_BOLD)
	
	for i, x in enumerate(packetTypes.keys()):
		stdscr.addstr(i+1, 0, x)
		stdscr.addstr(i+1, 20, str(packetTypes[x]))
		stdscr.addstr(i+1, 40, f'{100.0*packetTypes[x]/10000.00:7.3f}%')
	
	## totals
	ptlen = len(packetTypes.keys())
	stdscr.addstr(ptlen+1, 0, '------------------------------------------------')
	stdscr.addstr(ptlen+2, 20, str(sum(packetTypes.values())))
	stdscr.addstr(ptlen+2, 40, f'{100.0*sum(packetTypes.values())/10000.00:7.3f}%')
	
	


	stdscr.refresh()

def curses_main(stdscr):
	curses.curs_set(0)
	
	while(True):
		resp = ser.readline()
		pack = MC_SerialPacket(resp)

		if(pack.invalid):
			continue

		## Update the dict
		packetTypes[pack.type] = pack.val

		## Trigger redraw
		draw_table(stdscr)

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



