
import time
import math
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot_ahrs(mc):

    fig = plt.figure(figsize=(6,1))
    gs = gridspec.GridSpec(6,1, figure=fig)

    ax = fig.add_subplot(gs[0,0])
    ax.plot(list([e[0] for e in mc.ah_roll]), list([e[1] for e in mc.ah_roll]), color='blue', linestyle='--', label='Roll (AHRS)')
    ax.plot(list([e[0] for e in mc.sim_roll]), list([e[1] for e in mc.sim_roll]),color='green', linestyle='-', label='Roll (sim)')
    ax.plot(range(0,len(mc.ah_roll)), len(mc.ah_roll)*[-math.pi/2], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_roll)), len(mc.ah_roll)*[math.pi/2], color='red', linestyle='solid')
    ax.set_title(f'AHRS Roll (rad)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[1,0])
    ax.plot(list([e[0] for e in mc.ah_pitch]), list([e[1] for e in mc.ah_pitch]), color='blue', linestyle='--', label='Pitch (AHRS)')
    ax.plot(list([e[0] for e in mc.sim_pitch]), list([e[1] for e in mc.sim_pitch]),color='green', linestyle='-', label='Pitch (sim)')
    ax.plot(range(0,len(mc.ah_pitch)), len(mc.ah_pitch)*[-math.pi/2], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_pitch)), len(mc.ah_pitch)*[math.pi/2], color='red', linestyle='solid')
    ax.set_title(f'AHRS Pitch (deg*100)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[2,0])
    ax.plot(list([e[0] for e in mc.ah_yaw]), list([e[1] for e in mc.ah_yaw]), color='blue', linestyle='--', label='Yaw (AHRS)')
    ax.plot(list([e[0] for e in mc.sim_yaw]), list([e[1] for e in mc.sim_yaw]),color='green', linestyle='-', label='Yaw (sim)')
    ax.plot(range(0,len(mc.ah_yaw)), len(mc.ah_yaw)*[-math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_yaw)), len(mc.ah_yaw)*[math.pi], color='red', linestyle='solid')
    ax.set_title(f'AHRS Yaw (deg*100)')
    ax.legend()
    ax.grid(True)

    '''
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
    '''

    return


