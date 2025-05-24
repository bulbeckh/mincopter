
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
        control_ax.plot(range(0,self.parsed), self.target, color='blue', linestyle='--')
        control_ax.plot(range(0,self.parsed), self.out)
        control_ax.plot(range(0,self.parsed), self.pid_min, color='purple', linestyle=':')
        control_ax.plot(range(0,self.parsed), self.pid_max, color='purple', linestyle=':')
        control_ax.set_title(f'Target ({self.title})')
        control_ax.legend()
        control_ax.grid(True)

        error_ax.plot(range(0,self.parsed), self.error, color='red', linewidth=2)
        error_ax.set_title(f'Error ({self.title}')





if __name__=="__main__":

    lines = read_log("../../build/standard.mcsimlog")

    ah_roll=[]
    ah_pitch=[]
    ah_yaw=[]
    ah_error_rp=[]
    ah_error_y=[]

    pcount=0

    for l in lines:
        splits = l.split(",")
        if splits[0]=='ah':
            pcount+=1
            ah_roll.append(float(splits[1]))
            ah_pitch.append(float(splits[2]))
            ah_yaw.append(float(splits[3]))
            ah_error_rp.append(float(splits[4]))
            ah_error_y.append(float(splits[5]))

    fig = plt.figure(figsize=(14,8))
    gs = gridspec.GridSpec(5,1, figure=fig)

    axr = fig.add_subplot(gs[0,0])
    axr.plot(range(0,pcount), ah_roll, color='blue', linestyle='--')
    axr.set_title(f'AHRS Roll (deg*100)')
    axr.legend()
    axr.grid(True)

    axp = fig.add_subplot(gs[1,0])
    axp.plot(range(0,pcount), ah_pitch, color='blue', linestyle='--')
    axp.set_title(f'AHRS Pitch (deg*100)')
    axp.legend()
    axp.grid(True)

    axp = fig.add_subplot(gs[2,0])
    axp.plot(range(0,pcount), ah_yaw, color='blue', linestyle='--')
    axp.set_title(f'AHRS Yaw (deg*100)')
    axp.legend()
    axp.grid(True)

    '''
    axp = fig.add_subplot(gs[3,0])
    axp.plot(range(0,pcount), ah_error_rp, color='blue', linestyle='--')
    axp.set_title(f'AHRS Error RP (rad)')
    axp.legend()
    axp.grid(True)

    axp = fig.add_subplot(gs[4,0])
    axp.plot(range(0,pcount), ah_error_y, color='blue', linestyle='--')
    axp.set_title(f'AHRS Error Y (rad)')
    axp.legend()
    axp.grid(True)
    '''


    plt.tight_layout()
    plt.show()



