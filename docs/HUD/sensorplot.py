
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

class IMUSensor:
    def __init__(self, title):
        self.parsed = 0

    def plot(self):
        fig = plt.figure(figsize=(6,1))
        gs = gridspec.GridSpec(6,1, figure=fig)

        accel_x = fig.add_subplot(gs[0,0])
        accel_x.plot(range(0,self.parsed), self.accel_x, color='blue', linestyle='--')
        accel_x.set_title(f'Accel X (m/s^2)')
        accel_x.legend()
        accel_x.grid(True)

        accel_y = fig.add_subplot(gs[1,0])
        accel_y.plot(range(0,self.parsed), self.accel_y, color='blue', linestyle='--')
        accel_y.set_title(f'Accel Y (m/s^2)')
        accel_y.legend()
        accel_y.grid(True)

        accel_z = fig.add_subplot(gs[2,0])
        accel_z.plot(range(0,self.parsed), self.accel_z, color='blue', linestyle='--')
        accel_z.set_title(f'Accel Z (m/s^2)')
        accel_z.legend()
        accel_z.grid(True)

        gyro_x = fig.add_subplot(gs[3,0])
        gyro_x.plot(range(0,self.parsed), self.gyro_x, color='blue', linestyle='--')
        gyro_x.set_title(f'Gyro X (rad/s)')
        gyro_x.legend()
        gyro_x.grid(True)

        gyro_y = fig.add_subplot(gs[4,0])
        gyro_y.plot(range(0,self.parsed), self.gyro_y, color='blue', linestyle='--')
        gyro_y.set_title(f'Gyro Y (rad/s)')
        gyro_y.legend()
        gyro_y.grid(True)

        gyro_z = fig.add_subplot(gs[5,0])
        gyro_z.plot(range(0,self.parsed), self.gyro_z, color='blue', linestyle='--')
        gyro_z.set_title(f'Gyro Z (rad/s)')
        gyro_z.legend()
        gyro_z.grid(True)

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
