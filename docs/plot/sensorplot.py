
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import math

def plot_imu(mc):
    fig = plt.figure(figsize=(6,1))
    gs = gridspec.GridSpec(6,1, figure=fig)

    ax = fig.add_subplot(gs[0,0])
    ax.plot(list([e[0] for e in mc.accel_x]), list([e[1] for e in mc.accel_x]), color='blue', linestyle='--', label='Accel x')
    ax.set_title(f'Accel X (m/s^2)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[1,0])
    ax.plot(list([e[0] for e in mc.accel_y]), list([e[1] for e in mc.accel_y]), color='blue', linestyle='--', label='Accel y')
    ax.set_title(f'Accel Y (m/s^2)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[2,0])
    ax.plot(list([e[0] for e in mc.accel_z]), list([e[1] for e in mc.accel_z]), color='blue', linestyle='--', label='Accel z')
    ax.set_title(f'Accel Z (m/s^2)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[3,0])
    ax.plot(list([e[0] for e in mc.gyro_x]), list([e[1] for e in mc.gyro_x]), color='blue', linestyle='--', label='Gyro x')
    ax.plot(range(0,len(mc.gyro_x)), len(mc.gyro_x)*[-3*math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.gyro_x)), len(mc.gyro_x)*[3*math.pi], color='red', linestyle='solid')
    ax.set_title(f'Gyro X (rad/s)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[4,0])
    ax.plot(list([e[0] for e in mc.gyro_y]), list([e[1] for e in mc.gyro_y]), color='blue', linestyle='--', label='Gyro y')
    ax.plot(range(0,len(mc.gyro_y)), len(mc.gyro_y)*[-3*math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.gyro_y)), len(mc.gyro_y)*[3*math.pi], color='red', linestyle='solid')
    ax.set_title(f'Gyro Y (rad/s)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[5,0])
    ax.plot(list([e[0] for e in mc.gyro_z]), list([e[1] for e in mc.gyro_z]), color='blue', linestyle='--', label='Gyro z')
    ax.plot(range(0,len(mc.gyro_z)), len(mc.gyro_z)*[-3*math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.gyro_z)), len(mc.gyro_z)*[3*math.pi], color='red', linestyle='solid')
    ax.set_title(f'Gyro Z (rad/s)')
    ax.legend()
    ax.grid(True)

    return

class BaroSensor:
    def __init__(self, title):
        self.title = title

        self.parsed = 0
    

    def plot(self):
        fig = plt.figure(figsize=(3,1))
        gs = gridspec.GridSpec(3,1, figure=fig)

        rs_ax = fig.add_subplot(gs[0,0])
        rs_ax.plot(range(0,self.parsed), self.temperature, color='blue', linestyle='--')
        rs_ax.set_title(f'Temperature (??)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[1,0])
        rs_ax.plot(range(0,self.parsed), self.pressure, color='blue', linestyle='--')
        rs_ax.set_title(f'Pressure (??)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[2,0])
        rs_ax.plot(range(0,self.parsed), self.altitude_calc, color='blue', linestyle='--')
        rs_ax.set_title(f'Altitude calculated (??)')
        rs_ax.legend()
        rs_ax.grid(True)

class CompassSensor:
    def __init__(self, title):
        self.title = title


        self.parsed = 0
    
    def parse(self, vals):
        pass

    def plot(self):
        fig = plt.figure(figsize=(3,1))
        gs = gridspec.GridSpec(3,1, figure=fig)

        rs_ax = fig.add_subplot(gs[0,0])
        rs_ax.plot(range(0,self.parsed), self.field_x, color='blue', linestyle='--')
        rs_ax.set_title(f'Field X (??)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[1,0])
        rs_ax.plot(range(0,self.parsed), self.field_y, color='blue', linestyle='--')
        rs_ax.set_title(f'Field Y (??)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[2,0])
        rs_ax.plot(range(0,self.parsed), self.field_z, color='blue', linestyle='--')
        rs_ax.set_title(f'Field Z (??)')
        rs_ax.legend()
        rs_ax.grid(True)
