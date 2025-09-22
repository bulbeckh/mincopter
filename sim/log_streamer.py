# reader.py
import os
import struct
from collections import deque

from matplotlib import pyplot as plt
import matplotlib.animation as animation

import numpy as np

def parse(frame, ctx):
    print(f"Frame found, {len(frame)}, {type(frame)}")

    if frame[2]==0x01:
        roll, pitch, yaw = struct.unpack('<fff', frame[4:])
        print(f"(Roll,Pitch,Yaw): {roll}, {pitch}, {yaw}")

        ctx['roll'].append(roll)
        ctx['pitch'].append(pitch)
        ctx['yaw'].append(yaw)

    elif frame[2]==0x02:
        pos_x, pos_y, pos_z = struct.unpack('<fff', frame[4:])
        print(f"(x,y,z): {pos_x}, {pos_y}, {pos_z}")

        ctx['pos_x'].append(pos_x)
        ctx['pos_y'].append(pos_y)
        ctx['pos_z'].append(pos_z)

    elif frame[2]==0x03:
        vel_x, vel_y, vel_z = struct.unpack('<fff', frame[4:])
        print(f"(dx,dy,dz): {vel_x}, {vel_y}, {vel_z}")

        ctx['vel_x'].append(vel_x)
        ctx['vel_y'].append(vel_y)
        ctx['vel_z'].append(vel_z)

    elif frame[2]==0x04:
        er_x, er_y, er_z = struct.unpack('<fff', frame[4:])
        print(f"(droll,dpitch,dyaw): {er_x}, {er_y}, {er_z}")

        ctx['er_x'].append(er_x)
        ctx['er_y'].append(er_y)
        ctx['er_z'].append(er_z)

    elif frame[2]==0x05:
        force, rt, pt, yt = struct.unpack('<ffff', frame[4:])
        print(f"(force,rt,pt,yt): {force}, {rt}, {pt}, {yt}")

        ctx['force'].append(force)
        ctx['rt'].append(rt)
        ctx['pt'].append(pt)
        ctx['yt'].append(yt)

    elif frame[2]==0x06:
        m0, m1, m2, m3 = struct.unpack('<hhhh', frame[4:])
        print(f"(m0,m1,m2,m3): {m0}, {m1}, {m2}, {m3}")

        ctx['m0'].append(m0)
        ctx['m1'].append(m1)
        ctx['m2'].append(m2)
        ctx['m3'].append(m3)

    else:
        print(f"bad type: {frame[2]}")

    if ctx['i']%100==0:
        # Refresh graph
        for i,l in enumerate(ctx['lines'].keys()):
            ctx['lines'][l].remove()
            ctx['lines'][l], = ax[ctx['locs'][l][0], ctx['locs'][l][1]].plot(range(0,len(ctx[l])), ctx[l], color='red')

        plt.pause(0.1)

    return

if __name__=="__main__":
    fifo_path = "/tmp/mincopter_log"

    plt.ion()

    fig, ax = plt.subplots(10,2, figsize=(16, 24))

    locs = {'roll': (0,0),
              'pitch': (1,0),
              'yaw': (2,0),
              'pos_x': (3,0),
              'pos_y': (4,0),
              'pos_z': (5,0),
              'er_x': (0,1),
              'er_y': (1,1),
              'er_z': (2,1),
              'vel_x': (3,1),
              'vel_y': (4,1),
              'vel_z': (5,1),
              'force': (6,0), 
              'rt': (7,0),
              'pt': (8,0),
              'yt': (9,0),
              'm0': (6,1),
              'm1': (7,1),
              'm2': (8,1),
              'm3': (9,1)}
    
    ctx = {'i': 0, 'lines': {}, 'ax': ax, 'locs': locs}

    for i,e in enumerate(locs.keys()):
        ctx[e] = []
        ctx['lines'][e], = ax[locs[e][0],locs[e][1]].plot([],[], color='red')

        ax[locs[e][0], locs[e][1]].set_title(e)

    plt.pause(0.5)

    # Ensure the FIFO exists (in case reader is started first)
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)

    print("Waiting for data...")

    with open(fifo_path, "rb") as fifo:

        frame_counter = 0

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
                    ctx['i'] = frame_counter
                    frame_counter += 1

                    parse(frame, ctx)
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



