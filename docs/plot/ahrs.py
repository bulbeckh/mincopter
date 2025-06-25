
import time
import math
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot_ahrs(mc):

    fig = plt.figure(figsize=(6,1))
    gs = gridspec.GridSpec(6,1, figure=fig)

    ax = fig.add_subplot(gs[0,0])
    ax.plot(mc.ah_roll, color='blue', linestyle='--', label='Roll (AHRS)')
    ax.plot(mc.sim_roll, color='green', linestyle='-', label='Roll (sim)')
    ax.plot(range(0,len(mc.ah_roll)), len(mc.ah_roll)*[-math.pi/2], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_roll)), len(mc.ah_roll)*[math.pi/2], color='red', linestyle='solid')
    ax.set_title(f'AHRS Roll (rad)')
    ax.legend()
    ax.grid(True)

    plt.show()
    return

    ps_ax = fig.add_subplot(gs[1,0])
    ps_ax.plot(range(0,mc.parsed), mc.ah_pitch, color='blue', linestyle='--', label='Pitch (AHRS)')
    ps_ax.plot(range(0,mc.parsed), mc.sim_pitch, color='green', linestyle='-', label='Pitch (sim)')
    ps_ax.plot(range(0,mc.parsed), mc.parsed*[-math.pi/2], color='red', linestyle='solid')
    ps_ax.plot(range(0,mc.parsed), mc.parsed*[math.pi/2], color='red', linestyle='solid')
    ps_ax.set_title(f'AHRS Pitch (deg*100)')
    ps_ax.legend()
    ps_ax.grid(True)

    ys_ax = fig.add_subplot(gs[2,0])
    ys_ax.plot(range(0,mc.parsed), mc.ah_yaw, color='blue', linestyle='--', label='Yaw (AHRS)')
    ys_ax.plot(range(0,mc.parsed), mc.sim_yaw, color='green', linestyle='-', label='Yaw (sim)')
    ys_ax.plot(range(0,mc.parsed), mc.parsed*[-math.pi/2], color='red', linestyle='solid')
    ys_ax.plot(range(0,mc.parsed), mc.parsed*[math.pi/2], color='red', linestyle='solid')
    ys_ax.set_title(f'AHRS Yaw (deg*100)')
    ys_ax.legend()
    ys_ax.grid(True)

    ys_ax = fig.add_subplot(gs[3,0])
    ys_ax.plot(range(0,mc.parsed), mc.acc_x, color='blue', linestyle='--', label='Accel-X (AHRS)')
    ys_ax.set_title(f'AHRS Accel X (m/s2)')
    ys_ax.legend()
    ys_ax.grid(True)

    ys_ax = fig.add_subplot(gs[4,0])
    ys_ax.plot(range(0,mc.parsed), mc.acc_y, color='blue', linestyle='--', label='Accel-Y (AHRS)')
    ys_ax.set_title(f'AHRS Accel Y (m/s2)')
    ys_ax.legend()
    ys_ax.grid(True)

    ys_ax = fig.add_subplot(gs[5,0])
    ys_ax.plot(range(0,mc.parsed), mc.acc_z, color='blue', linestyle='--', label='Accel-Z (AHRS)')
    ys_ax.set_title(f'AHRS Accel Z (m/s2)')
    ys_ax.legend()
    ys_ax.grid(True)


