#/usr/bin/python3

import serial
import time
import re
import curses
import time
import random

if __name__=="__main__":

	while(True):
		try:
			ser = serial.Serial('/dev/ttyUSB0', 57600)
			print("Connection found")
			break
		except serial.SerialException:
			print("Waiting....")
			time.sleep(0.5)

	while(True):
		resp = ser.readline()
		print(resp)




