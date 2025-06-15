#!/usr/bin/env python3

import sys
import time

""" These are each of the sensor classes. Each class should define an __init__, parse, and plot method

def __init(self, title)
def parse(self, vals)
def plot()

"""
from sensorplot import IMUSensor, BaroSensor, CompassSensor
from ahrsplot import AHRS
from inavplot import INav
from mpcplot import MPC
## TODO Add in inavc graphs

import matplotlib.pyplot as plt

def read_log(path):
    tgt = None
    with open(path, 'r') as rfile:
        tgt = rfile.readlines()

    return tgt

if __name__=="__main__":

    if sys.argv[1]=='--help':
        print(""" man page



        """)
        exit()

    lines = read_log("../../build/standard.mcsimlog")

    ## List of all potential objects that this library can plot
    ## The value of this dict is a tuple where first is instantiated type (only instantiated if value is
    ## passed as commnd line argument) and the second is the class object that is used to instantiate
    objlist = {
        'ahrs': [None, AHRS],
        'inav': [None, INav],
        'pid': [None, None],
        'imu': [None, IMUSensor],
        'baro': [None, BaroSensor],
        'gps': [None, None],
        'compass': [None, CompassSensor],
        'motor' : [None, None],
        'mpc' : [None, MPC]
    }

    for arg in sys.argv:
        if arg.lower() in objlist.keys():
            ## This is so needlessly complex lol. objlist stores the class object and we are just instantiating it here
            objlist[arg.lower()][0] = objlist[arg.lower()][1](arg.lower)
        else:
            print(f'"{arg}" not a plottable type')


    for l in lines:
        splits = l.split(",")
        if splits[0]=='baro' and objlist['baro'][0] is not None:
            objlist['baro'][0].parse(splits[1:])
        if splits[0]=='imu' and objlist['imu'][0] is not None:
            objlist['imu'][0].parse(splits[1:])
        if splits[0]=='comp' and objlist['compass'][0] is not None:
            objlist['compass'][0].parse(splits[1:])
        if splits[0]=='gps' and objlist['gps'][0] is not None:
            objlist['gps'][0].parse(splits[1:])
        if splits[0]=='ah' and objlist['ahrs'][0] is not None:
            objlist['ahrs'][0].parse(splits[1:])
        if (splits[0]=='inav' or splits[0]=='inavc') and objlist['inav'][0] is not None:
            objlist['inav'][0].parse(splits[1:])
        if splits[0]=='mpc' and objlist['mpc'][0] is not None:
            objlist['mpc'][0].parse(splits[1:])

    for k,v in objlist.items():
        if v[0] is not None:
            v[0].plot()

    #plt.tight_layout()
    plt.show()
