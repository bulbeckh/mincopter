
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def read_log(path):
    tgt = None
    with open(path, 'r') as rfile:
        tgt = rfile.readlines()

    return tgt

class PID_history:
    def __init__(self, title, units):
        self.units = units

        self.target = []
        self.error = []
        self.out = []
        self.pid_max = []
        self.pid_min = []

        self.parsed=0

    def parse(self, vals):
        ## line should have format: target, error, out, max, min

        ## Return if we don't have enough values
        if len(vals)<5:
            return

        self.target.append(int(vals[0]))
        self.error.append(int(vals[1]))
        self.out.append(int(vals[2]))
        self.pid_max.append(int(vals[3]))
        self.pid_min.append(int(vals[4]))
        
        self.parsed += 1

    def plot(self, control_ax, error_ax, title):
        control_ax.plot(range(0,self.parsed), self.target, color='blue', linestyle='--', label=f'Target ({self.units["target"]})')
        control_ax.plot(range(0,self.parsed), self.error, color='red', linewidth=2, label=f'err ({self.units["target"]})')
        control_ax.set_title(f'{title[0]}')
        control_ax.legend()
        control_ax.set_facecolor('#fce4ec')
        control_ax.grid(True)

        error_ax.plot(range(0,self.parsed), self.out, label=f'Output ({self.units["output"]})')
        error_ax.plot(range(0,self.parsed), self.pid_min, color='purple', linewidth=2, linestyle=':', label=f'Min ({self.units["output"]})')
        error_ax.plot(range(0,self.parsed), self.pid_max, color='purple', linewidth=2, linestyle=':', label=f'Max ({self.units["output"]})')
        error_ax.legend()
        error_ax.set_facecolor('#e3f2fd')
        error_ax.set_title(f'{title[1]}')





if __name__=="__main__":

    lines = read_log("../../build/standard.mcsimlog")

    hl_rp_units = {'target' : 'deg*100', 'output' : 'deg*100/s'}
    ll_rp_units = {'target' : 'deg*100/s', 'output' : 'no units' }

    hl_throt_units = {'target' : '', 'output' : ''}

    hl_roll = PID_history('Stabilize Roll', units=hl_rp_units)
    hl_pitch = PID_history('Stabilize Pitch', units=hl_rp_units)
    hl_yaw = PID_history('Stabilize Yaw', units=hl_rp_units)
    hl_throttle = PID_history('Rate Throttle', units=hl_throt_units)

    ll_roll = PID_history('Roll Rate', units=ll_rp_units)
    ll_pitch = PID_history('Pitch Rate', units=ll_rp_units)
    ll_yaw = PID_history('Yaw Rate', units=ll_rp_units)
    ll_throttle = PID_history('Accel Throttle', units=hl_throt_units)

    throttle_althold = PID_history('Throttle Althold', units=hl_throt_units)

    for l in lines:
        splits = l.split(",")
        if splits[0]=='pid' and splits[1]=='stb_roll':
            hl_roll.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='stb_pitch':
            hl_pitch.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='stb_yaw':
            hl_yaw.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='throttle_rate':
            hl_throttle.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='rate_roll':
            ll_roll.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='rate_pitch':
            ll_pitch.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='rate_yaw':
            ll_yaw.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='throttle_accel':
            ll_throttle.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='throttle_althold':
            throttle_althold.parse(splits[2:])

    #fig, ax = plt.subplots(6,1, figsize=(14,8))
    #fig = plt.figure(figsize=(14,8))
    fig = plt.figure()
    gs = gridspec.GridSpec(12,1, figure=fig)

    hl_roll.plot(fig.add_subplot(gs[0,0]), fig.add_subplot(gs[1,0]), ['Roll', 'Roll Error'])
    hl_pitch.plot(fig.add_subplot(gs[2,0]), fig.add_subplot(gs[3,0]), ['Pitch', 'Pitch Error'])
    hl_yaw.plot(fig.add_subplot(gs[4,0]), fig.add_subplot(gs[5,0]), ['Yaw', 'Yaw Error'])

    ll_roll.plot(fig.add_subplot(gs[6,0]), fig.add_subplot(gs[7,0]), ['Roll Rate', 'RR Error'])
    ll_pitch.plot(fig.add_subplot(gs[8,0]), fig.add_subplot(gs[9,0]), ['Pitch Rate', 'PR Error'])
    ll_yaw.plot(fig.add_subplot(gs[10,0]), fig.add_subplot(gs[11,0]), ['Yaw Rate', 'YR Error'])

    fig2 = plt.figure()
    gs2 = gridspec.GridSpec(6,1,figure=fig2)
    hl_throttle.plot(fig2.add_subplot(gs2[0,0]), fig2.add_subplot(gs2[1,0]), ['Throttle', 'Throttle Error'])
    ll_throttle.plot(fig2.add_subplot(gs2[2,0]), fig2.add_subplot(gs2[3,0]), ['Throttle Rate', 'TR Error'])
    throttle_althold.plot(fig2.add_subplot(gs2[4,0]), fig2.add_subplot(gs2[5,0]), ['Throttle Althold', 'TA Error'])

    plt.tight_layout()
    plt.show()



