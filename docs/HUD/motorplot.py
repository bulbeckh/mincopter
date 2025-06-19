
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def read_log(path):
    tgt = None
    with open(path, 'r') as rfile:
        tgt = rfile.readlines()

    return tgt

class MotorHistory:
    def __init__(self, title, units):
        self.units = units

        self.mot1 = []
        self.mot2 = []
        self.mot3 = []
        self.mot4 = []

        self.parsed=0

    def parse(self, vals):
        ## line should have format: target, error, out, max, min

        ## Return if we don't have enough values
        if len(vals)<4:
            return

        self.mot1.append(int(vals[0]))
        self.mot2.append(int(vals[1]))
        self.mot3.append(int(vals[2]))
        self.mot4.append(int(vals[3]))
        self.parsed += 1

    def plot(self, m_ax, title):
        m_ax[0].plot(range(0,self.parsed), self.mot1, color='blue', linestyle='--', label=f'Target ({self.units["target"]})')
        m_ax[0].set_title('Motor0')
        m_ax[0].legend()
        m_ax[0].set_facecolor('#fce4ec')
        m_ax[0].grid(True)

        m_ax[1].plot(range(0,self.parsed), self.mot2, color='blue', linestyle='--', label=f'Target ({self.units["target"]})')
        m_ax[1].set_title('Motor1')
        m_ax[1].legend()
        m_ax[1].set_facecolor('#fce4ec')
        m_ax[1].grid(True)

        m_ax[2].plot(range(0,self.parsed), self.mot3, color='blue', linestyle='--', label=f'Target ({self.units["target"]})')
        m_ax[2].set_title('Motor0')
        m_ax[2].legend()
        m_ax[2].set_facecolor('#fce4ec')
        m_ax[2].grid(True)

        m_ax[3].plot(range(0,self.parsed), self.mot4, color='blue', linestyle='--', label=f'Target ({self.units["target"]})')
        m_ax[3].set_title('Motor0')
        m_ax[3].legend()
        m_ax[3].set_facecolor('#fce4ec')
        m_ax[3].grid(True)




if __name__=="__main__":

    lines = read_log("../../build/standard.mcsimlog")

    mot_units = {'target' : 'pwm'}
    motor = MotorHistory('Stabilize Roll', units=mot_units)

    for l in lines:
        splits = l.split(",")
        if splits[0]=='m':
            motor.parse(splits[1:])

    fig = plt.figure()
    gs = gridspec.GridSpec(4,1, figure=fig)

    motor.plot(
        [
            fig.add_subplot(gs[0,0]),
            fig.add_subplot(gs[1,0]),
            fig.add_subplot(gs[2,0]),
            fig.add_subplot(gs[3,0])
        ],
        ['Roll', 'Roll Error']
    )

    plt.show()



