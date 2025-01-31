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
			print(f'Invalid serial pack found with type {self.type}')
			return
		else:
			self.invalid = False

		self.val = float(parts[1].strip()[0:-2])

		print(self.type, self.val, type(self.val))
	
if __name__=="__main__":

	while(True):
		try:
			ser = serial.Serial('/dev/ttyACM0', 115200)
			print("Connection found")
			break
		except serial.SerialException:
			print("Waiting....")
			time.sleep(0.5)

	## Wait for packet containing menu
	while(True):
		resp = ser.readline()
		if 'Initialisation complete' in resp.decode('utf-8'):
			print("init finished")
			ser.write("none\n".encode())
		else:
			print(resp.decode('utf-8'))

		##pack = MC_SerialPacket(resp)
	



