
import time
import math
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


class AHRS:
    def __init__(self, title):
        self.title = title

        self.ah_roll = []
        self.ah_pitch = []
        self.ah_yaw =[]

        self.acc_x = []
        self.acc_y = []
        self.acc_z = []

        self.ah_error_rp = []
        self.ah_error_y = []

        self.sim_roll = []
        self.sim_pitch = []
        self.sim_yaw = []

        self.parsed=0

    def parse(self, vals):

        if len(vals)<11:
            return

        self.ah_roll.append(float(vals[0]))
        self.ah_pitch.append(float(vals[1]))
        self.ah_yaw.append(float(vals[2]))

        self.acc_x.append(float(vals[3]))
        self.acc_y.append(float(vals[4]))
        self.acc_z.append(float(vals[5]))

        self.ah_error_rp.append(float(vals[6]))
        self.ah_error_y.append(float(vals[7]))

        self.sim_roll.append(float(vals[8]))
        self.sim_pitch.append(float(vals[9]))
        self.sim_yaw.append(float(vals[10]))

        self.parsed += 1

    def plot(self):
        fig = plt.figure(figsize=(6,1))
        gs = gridspec.GridSpec(6,1, figure=fig)

        rs_ax = fig.add_subplot(gs[0,0])
        rs_ax.plot(range(0,self.parsed), self.ah_roll, color='blue', linestyle='--', label='Roll (AHRS)')
        rs_ax.plot(range(0,self.parsed), self.sim_roll, color='green', linestyle='-', label='Roll (sim)')
        rs_ax.plot(range(0,self.parsed), self.parsed*[-math.pi/2], color='red', linestyle='solid')
        rs_ax.plot(range(0,self.parsed), self.parsed*[math.pi/2], color='red', linestyle='solid')
        rs_ax.set_title(f'AHRS Roll (rad)')
        rs_ax.legend()
        rs_ax.grid(True)

        ps_ax = fig.add_subplot(gs[1,0])
        ps_ax.plot(range(0,self.parsed), self.ah_pitch, color='blue', linestyle='--', label='Pitch (AHRS)')
        ps_ax.plot(range(0,self.parsed), self.sim_pitch, color='green', linestyle='-', label='Pitch (sim)')
        ps_ax.plot(range(0,self.parsed), self.parsed*[-math.pi/2], color='red', linestyle='solid')
        ps_ax.plot(range(0,self.parsed), self.parsed*[math.pi/2], color='red', linestyle='solid')
        ps_ax.set_title(f'AHRS Pitch (deg*100)')
        ps_ax.legend()
        ps_ax.grid(True)

        ys_ax = fig.add_subplot(gs[2,0])
        ys_ax.plot(range(0,self.parsed), self.ah_yaw, color='blue', linestyle='--', label='Yaw (AHRS)')
        ys_ax.plot(range(0,self.parsed), self.sim_yaw, color='green', linestyle='-', label='Yaw (sim)')
        ys_ax.plot(range(0,self.parsed), self.parsed*[-math.pi/2], color='red', linestyle='solid')
        ys_ax.plot(range(0,self.parsed), self.parsed*[math.pi/2], color='red', linestyle='solid')
        ys_ax.set_title(f'AHRS Yaw (deg*100)')
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[3,0])
        ys_ax.plot(range(0,self.parsed), self.acc_x, color='blue', linestyle='--', label='Accel-X (AHRS)')
        ys_ax.set_title(f'AHRS Accel X (m/s2)')
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[4,0])
        ys_ax.plot(range(0,self.parsed), self.acc_y, color='blue', linestyle='--', label='Accel-Y (AHRS)')
        ys_ax.set_title(f'AHRS Accel Y (m/s2)')
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[5,0])
        ys_ax.plot(range(0,self.parsed), self.acc_z, color='blue', linestyle='--', label='Accel-Z (AHRS)')
        ys_ax.set_title(f'AHRS Accel Z (m/s2)')
        ys_ax.legend()
        ys_ax.grid(True)


