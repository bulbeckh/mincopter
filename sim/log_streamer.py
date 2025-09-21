# reader.py
import os
import struct
from collections import deque

from matplotlib import pyplot as plt
import matplotlib.animation as animation

import numpy as np


fifo_path = "/tmp/mincopter_log"

max_points = 500

fig, ax = plt.subplots()
xdata = deque(maxlen=max_points)
ydata = deque(maxlen=max_points)

line, = ax.plot([], [], lw=2)
ax.set_xlim(0, max_points)
ax.set_ylim(-1.5, 1.5)

def init():
    line.set_data([], [])
    return line,

def update(frame):
    new_y = np.sin(frame / 10.0) + np.random.normal(0, 0.1)

    xdata.append(frame)
    ydata.append(new_y)

    # Update line data
    line.set_data(range(len(ydata)), ydata)
    
    # Optional: dynamically adjust y-limits if needed
    # ax.set_ylim(min(ydata) - 0.5, max(ydata) + 0.5)

    return line,


# Ensure the FIFO exists (in case reader is started first)
if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

print("Waiting for data...")

def parse(frame):
    print(f"Frame found, {len(frame)}, {type(frame)}")

    if frame[2]==0x01:
        roll, pitch, yaw = struct.unpack('<fff', frame[4:])
        print(f"(Roll,Pitch,Yaw): {roll}, {pitch}, {yaw}")
    elif frame[2]==0x02:
        pos_x, pos_y, pos_z = struct.unpack('<fff', frame[4:])
        print(f"(x,y,z): {pos_x}, {pos_y}, {pos_z}")
    elif frame[2]==0x03:
        vel_x, vel_y, vel_z = struct.unpack('<fff', frame[4:])
        print(f"(dx,dy,dz): {vel_x}, {vel_y}, {vel_z}")
    elif frame[2]==0x04:
        er_x, er_y, er_z = struct.unpack('<fff', frame[4:])
        print(f"(droll,dpitch,dyaw): {er_x}, {er_y}, {er_z}")
    elif frame[2]==0x05:
        force, rt, pt, yt = struct.unpack('<ffff', frame[4:])
        print(f"(force,rt,pt,yt): {force}, {rt}, {pt}, {yt}")
    elif frame[2]==0x06:
        m0, m1, m2, m3 = struct.unpack('<hhhh', frame[4:])
        print(f"(m0,m1,m2,m3): {m0}, {m1}, {m2}, {m3}")
    else:
        print(f"bad type: {frame[2]}")

    return

#ani = animation.FuncAnimation(
        #fig, update, init_func=init, interval=500, blit=True
        #)

#plt.show()

with open(fifo_path, "rb") as fifo:

    while True:
        data = fifo.read(2048)

        if not data:
            break

        frame = bytearray()

        in_frame = False
        remaining = 0

        for b in data:

            ## If we are in a frame currently and still have bytes left then we just append
            if in_frame and remaining>0:
                frame.append(b)
                remaining -= 1
                continue

            ## If we are in a frame but have no bytes then we have a valid frame and should parse
            if in_frame and remaining==0:
                parse(frame)
                frame = bytearray()
                in_frame = False

            ## If we are not in a valid frame, we are either waiting for the sync bytes 
            if not in_frame and len(frame)==4:
                remaining = int(frame[-1]) - 1 
                frame.append(b)
                in_frame = True
                continue

            ## If we are not currently building a frame payload, then we are either
            elif not in_frame:
                if len(frame)==0 and b==0x2A:
                    frame.append(b)
                elif len(frame)==1 and b==0x4E:
                    frame.append(b)
                elif len(frame)<=3:
                    frame.append(b)
                else:
                    print("bad sync - resetting frame")
                    ## Reset frame
                    frame = bytearray()



