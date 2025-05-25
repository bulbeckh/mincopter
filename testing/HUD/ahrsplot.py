
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


class AHRS:
    def __init__(self, title):
        self.title = title

        self.ah_roll = []
        self.ah_pitch = []
        self.ah_yaw =[]
        self.ah_error_rp = []
        self.ah_error_y = []

        self.parsed=0

    def parse(self, vals):

        if len(vals)<5:
            return

        self.ah_roll.append(float(vals[0]))
        self.ah_pitch.append(float(vals[1]))
        self.ah_yaw.append(float(vals[2]))
        self.ah_error_rp.append(float(vals[3]))
        self.ah_error_y.append(float(vals[4]))

        self.parsed += 1

    def plot(self):
        fig = plt.figure(figsize=(3,1))
        gs = gridspec.GridSpec(3,1, figure=fig)

        rs_ax = fig.add_subplot(gs[0,0])
        rs_ax.plot(range(0,self.parsed), self.ah_roll, color='blue', linestyle='--')
        rs_ax.set_title(f'AHRS Roll (deg*100)')
        rs_ax.legend()
        rs_ax.grid(True)

        ps_ax = fig.add_subplot(gs[1,0])
        ps_ax.plot(range(0,self.parsed), self.ah_pitch, color='blue', linestyle='--')
        ps_ax.set_title(f'AHRS Pitch (deg*100)')
        ps_ax.legend()
        ps_ax.grid(True)

        ys_ax = fig.add_subplot(gs[2,0])
        ys_ax.plot(range(0,self.parsed), self.ah_yaw, color='blue', linestyle='--')
        ys_ax.set_title(f'AHRS Pitch (deg*100)')
        ys_ax.legend()
        ys_ax.grid(True)



