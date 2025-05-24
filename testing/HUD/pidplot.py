
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def read_log(path):
    tgt = None
    with open(path, 'r') as rfile:
        tgt = rfile.readlines()

    return tgt




class PID_history:
    def __init__(self, title):
        self.title = title

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

    def plot(self, control_ax, error_ax):
        control_ax.plot(range(0,self.parsed), self.target, color='blue', linestyle='--', label='target')
        control_ax.plot(range(0,self.parsed), self.out, label='output')
        control_ax.plot(range(0,self.parsed), self.pid_min, color='purple', linestyle=':', label='min')
        control_ax.plot(range(0,self.parsed), self.pid_max, color='purple', linestyle=':', label='max')
        control_ax.set_title(f'Target ({self.title})')
        control_ax.legend()
        control_ax.grid(True)

        error_ax.plot(range(0,self.parsed), self.error, color='red', linewidth=2)
        error_ax.set_title(f'Error ({self.title}')





if __name__=="__main__":

    lines = read_log("../../build/standard.mcsimlog")

    hl_roll = PID_history('Stabilize Roll')
    hl_pitch = PID_history('Stabilize Pitch')
    hl_throttle = PID_history('Rate Throttle')

    ll_roll = PID_history('Roll Rate')
    ll_pitch = PID_history('Pitch Rate')
    ll_throttle = PID_history('Accel Throttle')

    throttle_althold = PID_history('Throttle Althold')

    for l in lines:
        splits = l.split(",")
        if splits[0]=='pid' and splits[1]=='stb_roll':
            hl_roll.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='stb_pitch':
            hl_pitch.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='throttle_rate':
            hl_throttle.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='rate_roll':
            ll_roll.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='rate_pitch':
            ll_pitch.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='throttle_accel':
            ll_throttle.parse(splits[2:])
        elif splits[0]=='pid' and splits[1]=='throttle_althold':
            throttle_althold.parse(splits[2:])

    #fig, ax = plt.subplots(6,1, figsize=(14,8))
    fig = plt.figure(figsize=(14,8))
    gs = gridspec.GridSpec(8,2, figure=fig)

    hl_roll.plot(fig.add_subplot(gs[0,0]), fig.add_subplot(gs[1,0]))
    hl_pitch.plot(fig.add_subplot(gs[2,0]), fig.add_subplot(gs[3,0]))
    hl_throttle.plot(fig.add_subplot(gs[4,0]), fig.add_subplot(gs[5,0]))

    ll_roll.plot(fig.add_subplot(gs[0,1]), fig.add_subplot(gs[1,1]))
    ll_pitch.plot(fig.add_subplot(gs[2,1]), fig.add_subplot(gs[3,1]))
    ll_throttle.plot(fig.add_subplot(gs[4,1]), fig.add_subplot(gs[5,1]))

    throttle_althold.plot(fig.add_subplot(gs[6,0]), fig.add_subplot(gs[7,0]))

    plt.tight_layout()
    plt.show()



