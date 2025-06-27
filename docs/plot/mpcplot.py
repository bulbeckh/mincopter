
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

''' MPC Plot

Should plot each of the state vectors and then the corresponding MPC output 

'''

def plot_mpc(mc):
    fig = plt.figure(figsize=(8,1))
    gs = gridspec.GridSpec(8,1, figure=fig)

    ## MPC Thrust and Torques
    ax = fig.add_subplot(gs[0,0])
    ax.plot(list([e[0] for e in mc.control_thrust]), list([e[1] for e in mc.control_thrust]), color='blue', linestyle='--', label='Thrust')
    ax.set_title(f'Total Thrust (N)')
    a.set_ylim(0,50)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[1,0])
    ax.plot(list([e[0] for e in mc.control_rollt]), list([e[1] for e in mc.control_rollt]), color='blue', linestyle='--', label='Roll Torque')
    ax.set_title(f'Roll Torque(??)')
    ax.set_ylim(-2,2)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[2,0])
    ax.plot(list([e[0] for e in mc.control_pitcht]), list([e[1] for e in mc.control_pitcht]), color='blue', linestyle='--', label='Pitch Torque')
    ax.set_title(f'Pitch Torque(??)')
    ax.set_ylim(-2,2)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[3,0])
    ax.plot(list([e[0] for e in mc.control_yawt]), list([e[1] for e in mc.control_yawt]), color='blue', linestyle='--', label='Yaw Torque')
    ax.set_title(f'Yaw Torque(??)')
    ax.set_ylim(-0.05,0.05)
    ax.legend()
    ax.grid(True)

    ## PWM Signals (rotor speeds)
    ax = fig.add_subplot(gs[4,0])
    ax.plot(list([e[0] for e in mc.p0]), list([e[1] for e in mc.p0]), color='blue', linestyle='--', label='Motor 1 pwm')
    ax.set_title(f'Rotor0 Speed OR PWM (rad/s)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[5,0])
    ax.plot(list([e[0] for e in mc.p1]), list([e[1] for e in mc.p1]), color='blue', linestyle='--', label='Motor 2 pwm')
    ax.set_title(f'Rotor1 Speed OR PWM (rad/s)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[6,0])
    ax.plot(list([e[0] for e in mc.p2]), list([e[1] for e in mc.p2]), color='blue', linestyle='--', label='Motor 3 pwm')
    ax.set_title(f'Rotor2 Speed OR PWM (rad/s)')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[7,0])
    ax.plot(list([e[0] for e in mc.p3]), list([e[1] for e in mc.p3]), color='blue', linestyle='--', label='Motor 4 pwm')
    ax.set_title(f'Rotor3 Speed OR PWM (rad/s)')
    ax.legend()
    ax.grid(True)


