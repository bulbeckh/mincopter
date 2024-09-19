#/usr/bin/python3

import serial
import time

if __name__=="__main__":

	while(True):
		try:
			ser = serial.Serial('/dev/ttyACM0', 115200)
			print("Connection found")
			break
		except serial.SerialException:
			print("Waiting....")
			time.sleep(0.5)
	
	while(True):
		resp = ser.readline()
		print(resp)
	
		'''
		for e in resp:
			#print(type(e), chr(e) if 32<=e<=126 else '.')
			
			if (bytes(e) == b'\xfe'):
				print()
			else:
				print(chr(e) if 32<=e<=126 else '.', end='')
		'''
	


