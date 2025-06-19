
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

class INav:
    def __init__(self, title):
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []
        self.vel_x = []
        self.vel_y = []
        self.vel_z = []

        self.sim_px = []
        self.sim_py = []
        self.sim_pz = []
        self.sim_vx = []
        self.sim_vy = []
        self.sim_vz = []

        self.pos_corr_x = []
        self.pos_corr_y = []
        self.pos_corr_z = []

        self.pos_err_x = []
        self.pos_err_y = []
        self.pos_err_z = []

        self.accel_corr_x = []
        self.accel_corr_y = []
        self.accel_corr_z = []

        self.parsed_inav=0
        self.parsed_inavc=0

    def parse_inav(self, vals):
        self.pos_x.append(float(vals[0]))
        self.pos_y.append(float(vals[1]))
        self.pos_z.append(float(vals[2]))
        self.sim_px.append(float(vals[3]))
        self.sim_py.append(float(vals[4]))
        self.sim_pz.append(float(vals[5]))

        self.vel_x.append(float(vals[6]))
        self.vel_y.append(float(vals[7]))
        self.vel_z.append(float(vals[8]))
        self.sim_vx.append(float(vals[9]))
        self.sim_vy.append(float(vals[10]))
        self.sim_vz.append(float(vals[11]))
        
        self.parsed_inav += 1

    def parse_inavc(self, vals):
        self.pos_corr_x.append(float(vals[0]))
        self.pos_corr_y.append(float(vals[1]))
        self.pos_corr_z.append(float(vals[2]))
        self.pos_err_x.append(float(vals[3]))
        self.pos_err_y.append(float(vals[4]))
        self.pos_err_z.append(float(vals[5]))
        self.accel_corr_x.append(float(vals[6]))
        self.accel_corr_y.append(float(vals[7]))
        self.accel_corr_z.append(float(vals[8]))

        self.parsed_inavc += 1

    def parse(self, vals):
        ## TODO There should be a better way to direct which inav line type we are on
        if len(vals)==9:
            self.parse_inavc(vals)
        elif len(vals)==12:
            self.parse_inav(vals)
        else:
            print("INAV: line found with not enough fields")
            return

    def plot(self):
        fig = plt.figure(figsize=(6,1))
        gs = gridspec.GridSpec(6,1, figure=fig)

        ## INav Position and Velocities
        rs_ax = fig.add_subplot(gs[0,0])
        rs_ax.plot(range(0,self.parsed_inav), self.pos_x, color='blue', linestyle='--', label='X (inav)')
        rs_ax.plot(range(0,self.parsed_inav), self.sim_px, color='green', linestyle='-', label='X (sim)')
        rs_ax.set_title(f'INav pos x (??)')
        rs_ax.legend()
        rs_ax.grid(True)

        ps_ax = fig.add_subplot(gs[1,0])
        ps_ax.plot(range(0,self.parsed_inav), self.pos_y, color='blue', linestyle='--', label='Y (inav)')
        ps_ax.plot(range(0,self.parsed_inav), self.sim_py, color='green', linestyle='-', label='Y (sim)')
        ps_ax.set_title(f'INav pos y (??)')
        ps_ax.legend()
        ps_ax.grid(True)

        ys_ax = fig.add_subplot(gs[2,0])
        ys_ax.plot(range(0,self.parsed_inav), self.pos_z, color='blue', linestyle='--', label='Z (inav)')
        ys_ax.plot(range(0,self.parsed_inav), self.sim_pz, color='green', linestyle='-', label='Z (sim)')
        ys_ax.set_title(f'INav pos z (??)')
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[3,0])
        ys_ax.plot(range(0,self.parsed_inav), self.vel_x, color='blue', linestyle='--', label='Vel-X (inav)')
        ys_ax.plot(range(0,self.parsed_inav), self.sim_vx, color='green', linestyle='-', label='Vel-X (sim)')
        ys_ax.set_title(f'INav vel x (??)')
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[4,0])
        ys_ax.plot(range(0,self.parsed_inav), self.vel_y, color='blue', linestyle='--', label='Vel-Y (inav)')
        ys_ax.plot(range(0,self.parsed_inav), self.sim_vy, color='green', linestyle='-', label='Vel-Y (sim)')
        ys_ax.set_title(f'INav vel y (??)')
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[5,0])
        ys_ax.plot(range(0,self.parsed_inav), self.vel_z, color='blue', linestyle='--', label='Vel-Z (inav)')
        ys_ax.plot(range(0,self.parsed_inav), self.sim_vz, color='green', linestyle='-', label='Vel-Z (sim)')
        ys_ax.set_title(f'INav vel z (??)')
        ys_ax.legend()
        ys_ax.grid(True)

