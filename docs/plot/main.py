#!/usr/bin/env python3

import sys
import time

from mincopter import MinCopter
import matplotlib.pyplot as plt

from ahrs import plot_ahrs

if __name__=="__main__":

    
    if len(sys.argv)>2 and sys.argv[1]=='--help':
        print(""" man page



        """)
        exit()

    m = MinCopter()
    m.parse('../../build/standard.mcsimlog')

    plot_ahrs(m)


