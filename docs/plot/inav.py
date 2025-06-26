
import time


import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot_inav(mc):
    fig = plt.figure(figsize=(6,1))
    gs = gridspec.GridSpec(6,1, figure=fig)

    ## INav Position and Velocities
    ax = fig.add_subplot(gs[0,0])
    ax.plot(list([e[0] for e in mc.pos_x]), list([e[1] for e in mc.pos_x]), color='blue', linestyle='--', label='X (inav)')
    ax.plot(list([e[0] for e in mc.sim_px]), list([e[1] for e in mc.sim_px]), color='green', linestyle='-', label='X (sim)')
    ax.set_title(f'INav pos x (m)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[1,0])
    ax.plot(list([e[0] for e in mc.pos_y]), list([e[1] for e in mc.pos_y]), color='blue', linestyle='--', label='Y (inav)')
    ax.plot(list([e[0] for e in mc.sim_py]), list([e[1] for e in mc.sim_py]), color='green', linestyle='-', label='Y (sim)')
    ax.set_title(f'INav pos y (m)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[2,0])
    ax.plot(list([e[0] for e in mc.pos_z]), list([e[1] for e in mc.pos_z]), color='blue', linestyle='--', label='Z (inav)')
    ax.plot(list([e[0] for e in mc.sim_pz]), list([e[1] for e in mc.sim_pz]), color='green', linestyle='-', label='Z (sim)')
    ax.set_title(f'INav pos z (m)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[3,0])
    ax.plot(list([e[0] for e in mc.vel_x]), list([e[1] for e in mc.vel_x]), color='blue', linestyle='--', label='Vel-X (inav)')
    ax.plot(list([e[0] for e in mc.sim_vx]), list([e[1] for e in mc.sim_vx]), color='green', linestyle='-', label='Vel-X (sim)')
    ax.plot(range(0,len(mc.vel_x)), len(mc.vel_x)*[-5], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.vel_x)), len(mc.vel_x)*[5], color='red', linestyle='solid')
    ax.set_title(f'INav vel x (m)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[4,0])
    ax.plot(list([e[0] for e in mc.vel_y]), list([e[1] for e in mc.vel_y]), color='blue', linestyle='--', label='Vel-Y (inav)')
    ax.plot(list([e[0] for e in mc.sim_vy]), list([e[1] for e in mc.sim_vy]), color='green', linestyle='-', label='Vel-Y (sim)')
    ax.plot(range(0,len(mc.vel_y)), len(mc.vel_y)*[-5], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.vel_y)), len(mc.vel_y)*[5], color='red', linestyle='solid')
    ax.set_title(f'INav vel y (m)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[5,0])
    ax.plot(list([e[0] for e in mc.vel_z]), list([e[1] for e in mc.vel_z]), color='blue', linestyle='--', label='Vel-Z (inav)')
    ax.plot(list([e[0] for e in mc.sim_vz]), list([e[1] for e in mc.sim_vz]), color='green', linestyle='-', label='Vel-Z (sim)')
    ax.plot(range(0,len(mc.vel_z)), len(mc.vel_z)*[-5], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.vel_z)), len(mc.vel_z)*[5], color='red', linestyle='solid')
    ax.set_title(f'INav vel z (m)')
    ax.legend()
    ax.grid(True)

    return

