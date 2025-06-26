#!/usr/bin/env python3

import sys
import time

from mincopter import MinCopter
import matplotlib.pyplot as plt

from ahrs import plot_ahrs
from inav import plot_inav
from motor import plot_motors
from mpcplot import plot_mpc
from sensorplot import plot_imu


if __name__=="__main__":

    if len(sys.argv)>2 and sys.argv[1]=='--help':
        print(""" man page



        """)
        exit()

    m = MinCopter()
    m.parse('../../build/standard.mcsimlog')

    #plot_ahrs(m)
    #plot_inav(m)
    #plot_motors(m)
    plot_mpc(m)
    #plot_imu(m)

    plt.show()



