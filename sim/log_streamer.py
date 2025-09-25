# reader.py
import os
import struct
from collections import deque

from matplotlib import pyplot as plt
import matplotlib.animation as animation

import numpy as np

def parse(frame, ctx):
    print(f"Frame found, {len(frame)}, {type(frame)} {hex(frame[0])} {hex(frame[1])} {hex(frame[2])} {hex(frame[3])}")

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

    elif frame[2]==0x07:
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z = struct.unpack('<fffffffff', frame[4:])
        print(f"(acc, gyro, comp): {accel_x}, {accel_y}, {accel_z}, {gyro_x}, {gyro_y}, {gyro_z}, {comp_x}, {comp_y}, {comp_z}")

        ctx['accel_x'].append(accel_x)
        ctx['accel_y'].append(accel_y)
        ctx['accel_z'].append(accel_z)

        ctx['gyro_x'].append(gyro_x)
        ctx['gyro_y'].append(gyro_y)
        ctx['gyro_z'].append(gyro_z)

        ctx['comp_x'].append(comp_x)
        ctx['comp_y'].append(comp_y)
        ctx['comp_z'].append(comp_z)

    else:
        print(f"bad type: {frame[2]}")

    if ctx['i']%1000==0:
        # Refresh graph
        for i,l in enumerate(ctx['lines'].keys()):
            ctx['lines'][l].remove()

            if l in ctx['state_locs'].keys():
                ctx['lines'][l], = state_ax[ctx['state_locs'][l][0], ctx['state_locs'][l][1]].plot(range(0,len(ctx[l])), ctx[l], color='red')
            elif l in ctx['control_locs'].keys():
                ctx['lines'][l], = control_ax[ctx['control_locs'][l][0], ctx['control_locs'][l][1]].plot(range(0,len(ctx[l])), ctx[l], color='red')
            elif l in ctx['sensor_locs'].keys():
                ctx['lines'][l], = sensor_ax[ctx['sensor_locs'][l][0]].plot(range(0,len(ctx[l])), ctx[l], color='red')
            else:
                print("L not found")

        plt.pause(0.1)

    return

if __name__=="__main__":
    fifo_path = "/tmp/mincopter_log"

    plt.ion()

    ## Sensor figure
    sensor_fig, sensor_ax = plt.subplots(9,1, figsize=(12,8))
    sensor_fig.tight_layout()

    ## State figure
    state_fig, state_ax = plt.subplots(6, 2, figsize=(14,14))
    state_fig.tight_layout()

    ## Control figure
    control_fig, control_ax = plt.subplots(4,2, figsize=(14,14))
    control_fig.tight_layout()

    sensor_locs = {'accel_x': (0,0, r"$a_{x}$ (m/s2)"),
                   'accel_y': (1,0, r"$a_{y}$ (m/s2)"),
                   'accel_z': (2,0, r"$a_{z}$ (m/s2)"),
                   'gyro_x': (3,0, r"$g_{x}$ (rad/s)"),
                   'gyro_y': (4,0, r"$g_{y}$ (rad/s)"),
                   'gyro_z': (5,0, r"$g_{z}$ (rad/s)"),
                   'comp_x': (6,0, r"$m_{x}$ (ut)"),
                   'comp_y': (7,0, r"$m_{y}$ (ut)"),
                   'comp_z': (8,0, r"$m_{z}$ (ut)")}

    state_locs = {'roll': (0,0,r"$\phi$,roll (rad)"),
              'pitch': (1,0,r"$\theta$, pitch (rad)"),
              'yaw': (2,0,r"$\psi$, yaw (rad)"),
              'er_x': (0,1,r"$\dot{\phi}$ (rad/s)"),
              'er_y': (1,1,r"$\dot{\theta}$ (rad/s)"),
              'er_z': (2,1,r"$\dot{\psi}$ (rad/s)"),
              'pos_x': (3,0,r"$x$ (m)"),
              'pos_y': (4,0,r"$y$ (m)"),
              'pos_z': (5,0,r"$z$ (m)"),
              'vel_x': (3,1,r"$\dot{x}$ (m/s)"),
              'vel_y': (4,1,r"$\dot{y}$ (m/s)"),
              'vel_z': (5,1,r"$\dot{z}$ (m/s)")}

    control_locs = {
              'force': (0,0,r"$F$ (N)"), 
              'rt': (1,0,r"$\tau_{\phi}$ (Nm)"),
              'pt': (2,0,r"$\tau_{\theta}$ (Nm)"),
              'yt': (3,0,r"$\tau_{\psi}$ (Nm)"),
              'm0': (0,1,r"$\omega_{0}$ (rad/s)"),
              'm1': (1,1,r"$\omega_{1}$ (rad/s)"),
              'm2': (2,1,r"$\omega_{2}$ (rad/s)"),
              'm3': (3,1,r"$\omega_{3}$ (rad/s)")}


    ctx = {'i': 0, 'lines': {},
           'control_ax': control_ax, 'control_locs': control_locs,
           'state_ax': state_ax, 'state_locs': state_locs,
           'sensor_ax': sensor_ax, 'sensor_locs': sensor_locs}

    ## Setup sensor plots
    for i,e in enumerate(sensor_locs.keys()):
        ctx[e] = []
        ctx['lines'][e], = sensor_ax[sensor_locs[e][0]].plot([],[], color='red')

        sensor_ax[sensor_locs[e][0]].set_title(sensor_locs[e][2])

    ## Setup state plots
    for i,e in enumerate(state_locs.keys()):
        ctx[e] = []
        ctx['lines'][e], = state_ax[state_locs[e][0],state_locs[e][1]].plot([],[], color='red')

        state_ax[state_locs[e][0], state_locs[e][1]].set_title(state_locs[e][2])

    ## Setup control plots
    for i,e in enumerate(control_locs.keys()):
        ctx[e] = []
        ctx['lines'][e], = control_ax[control_locs[e][0],control_locs[e][1]].plot([],[], color='red')

        control_ax[control_locs[e][0], control_locs[e][1]].set_title(control_locs[e][2])

        if e in ['m0', 'm1', 'm2', 'm3']:
            control_ax[control_locs[e][0], control_locs[e][1]].set_ylim(950, 2050)

    plt.pause(0.5)

    # Ensure the FIFO exists (in case reader is started first)
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)

    print("Waiting for data...")

    with open(fifo_path, "rb") as fifo:

        frame_counter = 0

        frame = bytearray()

        remaining = 0
        step = 0 

        while True:
            data = fifo.read(2048)
            data = bytearray(data)

            if not data:
                break

            while (len(data)):
                ## parse
                if step==0 and data.pop(0)==0x2A:
                    frame.append(0x2A)
                    step += 1
                    continue
                if step==1 and data.pop(0)==0x4E:
                    frame.append(0x4E)
                    step += 1
                    continue
                if step==2:
                    frame.append(data.pop(0)) ## Type
                    step += 1
                    continue
                if step==3:
                    frame.append(data.pop(0)) ## Length
                    step += 1
                    continue
                if step==4:
                    ## Append to the frame but only either the full payload or the end of the data packet
                    print(frame[-1], type(frame[-1]))
                    frame.append(data[0:min(len(data), int(frame[-1]))])
                    ## Parse and reset
                    parse(frame, ctx)
                    frame = bytearray()
                    step = 0
                else:
                    print(f"step:{step} b:{hex(b)}: bad sync - reset to 0")
                    ## Reset frame
                    frame = bytearray()
                    step = 0
            
            '''
            for b in data:
                #print(f"[{step} -- {b}]")

                if step==0 and b==0x2A:
                    frame.append(b)
                    step += 1
                elif step==1 and b==0x4E:
                    frame.append(b)
                    step += 1
                elif step==2:
                    frame.append(b) ## Type
                    step += 1
                elif step==3:
                    frame.append(b)
                    remaining = int(b) ## Length
                    step += 1
                elif step==4 and remaining:
                    frame.append(b)
                    remaining -= 1
                elif step==4 and not remaining and b==0x2A:
                    ## parse packet and reset
                    parse(frame, ctx)
                    frame = bytearray()
                    frame.append(b)
                    step = 1
                else:
                    print(f"step:{step} b:{hex(b)}: bad sync - reset to 0")
                    ## Reset frame
                    frame = bytearray()
                    step = 0
                    remaining = 0
            '''


