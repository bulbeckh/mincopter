
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import math

''' MPC Plot

Should plot each of the state vectors and then the corresponding MPC output 

1. inav.x
2. inav.y
3. inav.z
4. ahrs.roll
5. ahrs.pitch
6. ahrs.yaw
7. inav.vx
8. inav.vy
9. inav.vz
10. imu.gyrox
11. imu.gyroy
12. imu.gyroz

13. MPC force
14. MPC r torque
15. MPC p torque
16. MPC y torque

17,18,19,20 MPC PWM outputs

'''

def plot_mpc(mc):
    fig = plt.figure(figsize=(20,1))
    gs = gridspec.GridSpec(20,1, figure=fig)

    ## Determine x range
    pmax = -1e9
    pmin = 1e9
    for ptype in [mc.pos_x,
                  mc.pos_y,
                  mc.pos_x, 
                  mc.ah_roll,
                  mc.ah_pitch,
                  mc.ah_yaw,
                  mc.vel_x,
                  mc.vel_y,
                  mc.vel_z,
                  mc.gyro_x,
                  mc.gyro_y,
                  mc.gyro_z,
                  mc.control_thrust,
                  mc.control_rollt,
                  mc.control_pitcht,
                  mc.control_yawt,
                  mc.p0, 
                  mc.p1, 
                  mc.p2,
                  mc.p3]:

        cmax = max(list([e[0] for e in ptype]))
        cmin = min(list([e[0] for e in ptype]))
        if cmax > pmax:
            pmax=cmax
        if cmin < pmin:
            pmin=cmin

    ## INav
    ax = fig.add_subplot(gs[0,0])
    ax.plot(list([e[0] for e in mc.pos_x]), list([e[1] for e in mc.pos_x]), color='blue', linestyle='--', label='X (inav)')
    ax.plot(list([e[0] for e in mc.sim_px]), list([e[1] for e in mc.sim_px]), color='green', linestyle='-', label='X (sim)')
    ax.set_title(f'INav pos x (m)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[1,0])
    ax.plot(list([e[0] for e in mc.pos_y]), list([e[1] for e in mc.pos_y]), color='blue', linestyle='--', label='Y (inav)')
    ax.plot(list([e[0] for e in mc.sim_py]), list([e[1] for e in mc.sim_py]), color='green', linestyle='-', label='Y (sim)')
    ax.set_title(f'INav pos y (m)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[2,0])
    ax.plot(list([e[0] for e in mc.pos_z]), list([e[1] for e in mc.pos_z]), color='blue', linestyle='--', label='Z (inav)')
    ax.plot(list([e[0] for e in mc.sim_pz]), list([e[1] for e in mc.sim_pz]), color='green', linestyle='-', label='Z (sim)')
    ax.set_title(f'INav pos z (m)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ## AHRS
    ax = fig.add_subplot(gs[3,0])
    ax.plot(list([e[0] for e in mc.ah_roll]), list([e[1] for e in mc.ah_roll]), color='blue', linestyle='--', label='Roll (AHRS)')
    ax.plot(list([e[0] for e in mc.sim_roll]), list([e[1] for e in mc.sim_roll]),color='green', linestyle='-', label='Roll (sim)')
    ax.plot(range(0,len(mc.ah_roll)), len(mc.ah_roll)*[-math.pi/2], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_roll)), len(mc.ah_roll)*[math.pi/2], color='red', linestyle='solid')
    ax.set_title(f'AHRS Roll (rad)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[4,0])
    ax.plot(list([e[0] for e in mc.ah_pitch]), list([e[1] for e in mc.ah_pitch]), color='blue', linestyle='--', label='Pitch (AHRS)')
    ax.plot(list([e[0] for e in mc.sim_pitch]), list([e[1] for e in mc.sim_pitch]),color='green', linestyle='-', label='Pitch (sim)')
    ax.plot(range(0,len(mc.ah_pitch)), len(mc.ah_pitch)*[-math.pi/2], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_pitch)), len(mc.ah_pitch)*[math.pi/2], color='red', linestyle='solid')
    ax.set_title(f'AHRS Pitch (deg*100)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[5,0])
    ax.plot(list([e[0] for e in mc.ah_yaw]), list([e[1] for e in mc.ah_yaw]), color='blue', linestyle='--', label='Yaw (AHRS)')
    ax.plot(list([e[0] for e in mc.sim_yaw]), list([e[1] for e in mc.sim_yaw]),color='green', linestyle='-', label='Yaw (sim)')
    ax.plot(range(0,len(mc.ah_yaw)), len(mc.ah_yaw)*[-math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.ah_yaw)), len(mc.ah_yaw)*[math.pi], color='red', linestyle='solid')
    ax.set_title(f'AHRS Yaw (deg*100)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ## Inav vel
    ax = fig.add_subplot(gs[6,0])
    ax.plot(list([e[0] for e in mc.vel_x]), list([e[1] for e in mc.vel_x]), color='blue', linestyle='--', label='Vel-X (inav)')
    ax.plot(list([e[0] for e in mc.sim_vx]), list([e[1] for e in mc.sim_vx]), color='green', linestyle='-', label='Vel-X (sim)')
    ax.plot(range(0,len(mc.vel_x)), len(mc.vel_x)*[-5], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.vel_x)), len(mc.vel_x)*[5], color='red', linestyle='solid')
    ax.set_title(f'INav vel x (m)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[7,0])
    ax.plot(list([e[0] for e in mc.vel_y]), list([e[1] for e in mc.vel_y]), color='blue', linestyle='--', label='Vel-Y (inav)')
    ax.plot(list([e[0] for e in mc.sim_vy]), list([e[1] for e in mc.sim_vy]), color='green', linestyle='-', label='Vel-Y (sim)')
    ax.plot(range(0,len(mc.vel_y)), len(mc.vel_y)*[-5], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.vel_y)), len(mc.vel_y)*[5], color='red', linestyle='solid')
    ax.set_title(f'INav vel y (m)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[8,0])
    ax.plot(list([e[0] for e in mc.vel_z]), list([e[1] for e in mc.vel_z]), color='blue', linestyle='--', label='Vel-Z (inav)')
    ax.plot(list([e[0] for e in mc.sim_vz]), list([e[1] for e in mc.sim_vz]), color='green', linestyle='-', label='Vel-Z (sim)')
    ax.plot(range(0,len(mc.vel_z)), len(mc.vel_z)*[-5], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.vel_z)), len(mc.vel_z)*[5], color='red', linestyle='solid')
    ax.set_title(f'INav vel z (m)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ## Imu Gyro Vel
    ax = fig.add_subplot(gs[9,0])
    ax.plot(list([e[0] for e in mc.gyro_x]), list([e[1] for e in mc.gyro_x]), color='blue', linestyle='--', label='Gyro x')
    ax.plot(range(0,len(mc.gyro_x)), len(mc.gyro_x)*[-3*math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.gyro_x)), len(mc.gyro_x)*[3*math.pi], color='red', linestyle='solid')
    ax.set_title(f'Gyro X (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[10,0])
    ax.plot(list([e[0] for e in mc.gyro_y]), list([e[1] for e in mc.gyro_y]), color='blue', linestyle='--', label='Gyro y')
    ax.plot(range(0,len(mc.gyro_y)), len(mc.gyro_y)*[-3*math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.gyro_y)), len(mc.gyro_y)*[3*math.pi], color='red', linestyle='solid')
    ax.set_title(f'Gyro Y (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[11,0])
    ax.plot(list([e[0] for e in mc.gyro_z]), list([e[1] for e in mc.gyro_z]), color='blue', linestyle='--', label='Gyro z')
    ax.plot(range(0,len(mc.gyro_z)), len(mc.gyro_z)*[-3*math.pi], color='red', linestyle='solid')
    ax.plot(range(0,len(mc.gyro_z)), len(mc.gyro_z)*[3*math.pi], color='red', linestyle='solid')
    ax.set_title(f'Gyro Z (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ## MPC Thrust and Torques
    ax = fig.add_subplot(gs[12,0])
    ax.plot(list([e[0] for e in mc.control_thrust]), list([e[1] for e in mc.control_thrust]), color='blue', linestyle='--', label='Thrust')
    ax.set_title(f'Total Thrust (N)')
    ax.set_xlim(pmin, pmax)
    ax.set_ylim(0,50)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[13,0])
    ax.plot(list([e[0] for e in mc.control_rollt]), list([e[1] for e in mc.control_rollt]), color='blue', linestyle='--', label='Roll Torque')
    ax.set_title(f'Roll Torque(??)')
    ax.set_xlim(pmin, pmax)
    ax.set_ylim(-2,2)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[14,0])
    ax.plot(list([e[0] for e in mc.control_pitcht]), list([e[1] for e in mc.control_pitcht]), color='blue', linestyle='--', label='Pitch Torque')
    ax.set_title(f'Pitch Torque(??)')
    ax.set_xlim(pmin, pmax)
    ax.set_ylim(-2,2)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[15,0])
    ax.plot(list([e[0] for e in mc.control_yawt]), list([e[1] for e in mc.control_yawt]), color='blue', linestyle='--', label='Yaw Torque')
    ax.set_title(f'Yaw Torque(??)')
    ax.set_xlim(pmin, pmax)
    ax.set_ylim(-0.05,0.05)
    ax.legend()
    ax.grid(True)

    ## PWM Signals (rotor speeds)
    ax = fig.add_subplot(gs[16,0])
    ax.plot(list([e[0] for e in mc.p0]), list([e[1] for e in mc.p0]), color='blue', linestyle='--', label='Motor 1 pwm')
    ax.set_title(f'Rotor0 Speed OR PWM (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[17,0])
    ax.plot(list([e[0] for e in mc.p1]), list([e[1] for e in mc.p1]), color='blue', linestyle='--', label='Motor 2 pwm')
    ax.set_title(f'Rotor1 Speed OR PWM (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[18,0])
    ax.plot(list([e[0] for e in mc.p2]), list([e[1] for e in mc.p2]), color='blue', linestyle='--', label='Motor 3 pwm')
    ax.set_title(f'Rotor2 Speed OR PWM (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(gs[19,0])
    ax.plot(list([e[0] for e in mc.p3]), list([e[1] for e in mc.p3]), color='blue', linestyle='--', label='Motor 4 pwm')
    ax.set_title(f'Rotor3 Speed OR PWM (rad/s)')
    ax.set_xlim(pmin, pmax)
    ax.legend()
    ax.grid(True)


