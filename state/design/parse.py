## Parse class for use in all libraries

import math
import struct

class Parser:
    def __init__(self):
        pass

    def read_gz_log(self):
        '''Reads a gz log file which logs sensor readings at 1kHz
        
        File typically located at /tmp/gz_imu_logfile.txt'''
        
        x = {'ax': [],
             'ay': [],
             'az': [],
             'gx': [],
             'gy': [],
             'gz': [],
             'mx': [],
             'my': [],
             'mz': []
            }
        
        y = {'ax': [],
             'ay': [],
             'az': [],
             'gx': [],
             'gy': [],
             'gz': [],
             'mx': [],
             'my': [],
             'mz': []
            }

        with open('/tmp/gz_imu_logfile.txt','r') as rfile:
            lines = rfile.readlines()
        
        for i,e in enumerate(lines):
            line = e.split(',')
        
            try:
                x_next = int(line[0])
                y_next = float(line[2])
            except:
                continue
        
            x[line[1]].append(x_next)
            y[line[1]].append(y_next)

        return x,y

    def _parse_mincopter(self, frame, ctx):
        #print(f"pkt found {len(frame)} {frame[2]}")
        if frame[2]==0x01:
            iterations, roll, pitch, yaw = struct.unpack('<Ifff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['roll'].append(roll)
            ctx['pitch'].append(pitch)
            ctx['yaw'].append(yaw)
    
        elif frame[2]==0x02:
            iterations, pos_x, pos_y, pos_z = struct.unpack('<Ifff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['pos_x'].append(pos_x)
            ctx['pos_y'].append(pos_y)
            ctx['pos_z'].append(pos_z)
    
        elif frame[2]==0x03:
            iterations, vel_x, vel_y, vel_z = struct.unpack('<Ifff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['vel_x'].append(vel_x)
            ctx['vel_y'].append(vel_y)
            ctx['vel_z'].append(vel_z)
    
        elif frame[2]==0x04:
            iterations, er_x, er_y, er_z = struct.unpack('<Ifff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['er_x'].append(er_x)
            ctx['er_y'].append(er_y)
            ctx['er_z'].append(er_z)
    
        elif frame[2]==0x05:
            iterations, force, rt, pt, yt = struct.unpack('<Iffff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['force'].append(force)
            ctx['rt'].append(rt)
            ctx['pt'].append(pt)
            ctx['yt'].append(yt)
    
        elif frame[2]==0x06:
            iterations, m0, m1, m2, m3 = struct.unpack('<Ihhhh', frame[4:])
            ctx['iter'].append(iterations)
            ctx['m0'].append(m0)
            ctx['m1'].append(m1)
            ctx['m2'].append(m2)
            ctx['m3'].append(m3)
    
        elif frame[2]==0x07:
            iterations, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z = struct.unpack('<Ifffffffff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['accel_x'].append(accel_x)
            ctx['accel_y'].append(accel_y)
            ctx['accel_z'].append(accel_z)
            ctx['gyro_x'].append(gyro_x)
            ctx['gyro_y'].append(gyro_y)
            ctx['gyro_z'].append(gyro_z)
            ctx['comp_x'].append(comp_x)
            ctx['comp_y'].append(comp_y)
            ctx['comp_z'].append(comp_z)
    
        elif frame[2]==0x08:
            iterations, pos_x_a, pos_y_a, pos_z_a, vel_x_a, vel_y_a, vel_z_a, eul_x_a, eul_y_a, eul_z_a, er_x_a, er_y_a, er_z_a = struct.unpack('<Iffffffffffff',frame[4:])
            ctx['iter'].append(iterations)
            ctx['pos_x_a'].append(pos_x_a)
            ctx['pos_y_a'].append(pos_y_a)
            ctx['pos_z_a'].append(pos_z_a)
            ctx['vel_x_a'].append(vel_x_a)
            ctx['vel_y_a'].append(vel_y_a)
            ctx['vel_z_a'].append(vel_z_a)
            ctx['eul_x_a'].append(eul_x_a)
            ctx['eul_y_a'].append(eul_y_a)
            ctx['eul_z_a'].append(eul_z_a)        
            ctx['er_x_a'].append(er_x_a)        
            ctx['er_y_a'].append(er_y_a)        
            ctx['er_z_a'].append(er_z_a)
        
        elif frame[2]==0x09:
            iterations, lat, lng, alt_cm, veln, vele, veld = struct.unpack('<Iiiifff', frame[4:])
            ctx['iter'].append(iterations)
            ctx['gps_lat'].append(lat)
            ctx['gps_lng'].append(lng)
            ctx['gps_alt_cm'].append(alt_cm)
            ctx['gps_vel_north'].append(veln)
            ctx['gps_vel_east'].append(vele)
            ctx['gps_vel_down'].append(veld)
    
    def read_mincopter_log(self, path):
        '''Reads the mincopter log - file typically located in build-generic/mincopter_log.txt'''
        
        with open(path,'rb') as rfile:
            x = rfile.read()
        
        ctx = {
            'iter': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'er_x': [],
            'er_y': [],
            'er_z': [],
            'pos_x': [],
            'pos_y': [],
            'pos_z': [],
            'vel_x': [],
            'vel_y': [],
            'vel_z': [],
            'force': [],
            'rt': [],
            'pt': [],
            'yt': [],
            'm0': [],
            'm1': [],
            'm2': [],
            'm3': [],
            'accel_x': [],
            'accel_y': [],
            'accel_z': [],
            'gyro_x': [],
            'gyro_y': [],
            'gyro_z': [],
            'comp_x': [],
            'comp_y': [],
            'comp_z': [],
            'pos_x_a': [],
            'pos_y_a': [],
            'pos_z_a': [],
            'vel_x_a': [],
            'vel_y_a': [],
            'vel_z_a': [],
            'eul_x_a': [],
            'eul_y_a': [],
            'eul_z_a': [],
            'er_x_a': [],
            'er_y_a': [],
            'er_z_a': [],
            'gps_lat': [],
            'gps_lng': [],
            'gps_alt_cm': [],
            'gps_vel_north': [],
            'gps_vel_east': [],
            'gps_vel_down': []
        }
            
        idx = 0
        while(idx<len(x)):
            frame_type = x[idx+2]
            frame_len = x[idx+3]
            frame = x[idx:idx+frame_len+4]
            self._parse_mincopter(frame, ctx)
            idx += len(frame)
        
        ## Correct a few bad accel readings
        for i in range(0,20):
            ctx['accel_x'][i] = 0
            ctx['accel_y'][i] = 0
            ctx['accel_z'][i] = 0

        return ctx
            