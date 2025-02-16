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
            ser = serial.Serial('/dev/ttyACM0', 115200)
            print("Connection found")
            break
        except serial.SerialException:
            print("Waiting....")
            time.sleep(0.5)

    ## Wait for initialisation
    while(True):
        raw_resp = ser.readline()
        resp = raw_resp.decode('utf-8')
        ## last char is always newline
        print(resp[:-1])
        
        if ('Initialisation Complete' in resp):
            break

    ## Send command
    ## NOTE lg was lognumber and was previously 19 onwards
    for lg in range(0,3):
        #cmd = f'getlog {lg}'.encode('utf-8') + b'\n'
        #cmd = 'showlogs'.encode('utf-8') + b'\n'
        #cmd = 'listlogs'.encode('utf-8') + b'\n'

        cmds = [('none', 'in none function'), ('showlogs','END0'), ('getlog 19', 'END0')]
        cmd = cmds[lg][0].encode('utf-8') + b'\n'
        
        ser.write(cmd)

        while(True):
            raw_resp = ser.readline()
            resp = raw_resp.decode('utf-8')
            print(f'{lg} ' + resp[:-1])
            
            ##Check the correct return status
            if cmds[lg][1] in resp:
                break
        



