
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

class MPC:
    def __init__(self, title):
        self.control_thrust = []
        self.control_rollt = []
        self.control_pitcht = []
        self.control_yawt = []

        ## PWM Signals
        self.p0 = []
        self.p1 = []
        self.p2 = []
        self.p3 = []

        self.parsed_mpc=0

    def parse_mpc(self, vals):
        self.control_thrust.append(float(vals[0]))
        self.control_rollt.append(float(vals[1]))
        self.control_pitcht.append(float(vals[2]))
        self.control_yawt.append(float(vals[3]))

        self.p0.append(float(vals[4]))
        self.p1.append(float(vals[5]))
        self.p2.append(float(vals[6]))
        self.p3.append(float(vals[7]))
        
        self.parsed_mpc += 1

    def parse(self, vals):
        ## TODO There should be a better way to direct which inav line type we are on
        if len(vals)==8:
            self.parse_mpc(vals)
        else:
            print("MPC: line found with not enough fields")
            return

    def plot(self):
        fig = plt.figure(figsize=(8,1))
        gs = gridspec.GridSpec(8,1, figure=fig)

        ## MPC Thrust and Torques
        rs_ax = fig.add_subplot(gs[0,0])
        rs_ax.plot(range(0,self.parsed_mpc), self.control_thrust, color='blue', linestyle='--')
        rs_ax.set_title(f'Total Thrust (N)')
        rs_ax.set_ylim(0,50)
        rs_ax.legend()
        rs_ax.grid(True)

        ps_ax = fig.add_subplot(gs[1,0])
        ps_ax.plot(range(0,self.parsed_mpc), self.control_rollt, color='blue', linestyle='--')
        ps_ax.set_title(f'Roll Torque(??)')
        ps_ax.set_ylim(-2,2)
        ps_ax.legend()
        ps_ax.grid(True)

        ys_ax = fig.add_subplot(gs[2,0])
        ys_ax.plot(range(0,self.parsed_mpc), self.control_pitcht, color='blue', linestyle='--')
        ys_ax.set_title(f'Pitch Torque(??)')
        ys_ax.set_ylim(-2,2)
        ys_ax.legend()
        ys_ax.grid(True)

        ys_ax = fig.add_subplot(gs[3,0])
        ys_ax.plot(range(0,self.parsed_mpc), self.control_yawt, color='blue', linestyle='--')
        ys_ax.set_title(f'Yaw Torque(??)')
        ys_ax.set_ylim(-0.05,0.05)
        ys_ax.legend()
        ys_ax.grid(True)

        ## PWM Signals (rotor speeds)
        rs_ax = fig.add_subplot(gs[4,0])
        rs_ax.plot(range(0,self.parsed_mpc), self.p0, color='blue', linestyle='--')
        rs_ax.set_title(f'Rotor0 Speed OR PWM (rad/s)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[5,0])
        rs_ax.plot(range(0,self.parsed_mpc), self.p1, color='blue', linestyle='--')
        rs_ax.set_title(f'Rotor1 Speed OR PWM (rad/s)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[6,0])
        rs_ax.plot(range(0,self.parsed_mpc), self.p2, color='blue', linestyle='--')
        rs_ax.set_title(f'Rotor2 Speed OR PWM (rad/s)')
        rs_ax.legend()
        rs_ax.grid(True)

        rs_ax = fig.add_subplot(gs[7,0])
        rs_ax.plot(range(0,self.parsed_mpc), self.p3, color='blue', linestyle='--')
        rs_ax.set_title(f'Rotor3 Speed OR PWM (rad/s)')
        rs_ax.legend()
        rs_ax.grid(True)


