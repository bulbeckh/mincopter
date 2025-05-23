
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def read_log(path):
    tgt = None
    with open(path, 'r') as rfile:
        tgt = rfile.readlines()

    return tgt

def plot(title, series, ax, fmt):
    ax.plot(range(0,len(series)), series, **fmt)
    ax.set_title(title)
    return

class INav:
    def __init__(self):
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []

        self.vel_x = []
        self.vel_y = []
        self.vel_z = []

        self.pos_corr_x = []
        self.pos_corr_y = []
        self.pos_corr_z = []

        self.pos_err_x = []
        self.pos_err_y = []
        self.pos_err_z = []

        self.accel_corr_x = []
        self.accel_corr_y = []
        self.accel_corr_z = []

    def parse_inav(self, vals):
        if len(vals)<6:
            return

        self.pos_x.append(float(vals[0]))
        self.pos_y.append(float(vals[1]))
        self.pos_z.append(float(vals[2]))
        self.vel_x.append(float(vals[3]))
        self.vel_y.append(float(vals[4]))
        self.vel_z.append(float(vals[5]))

    def parse_inavc(self, vals):
        if (len(vals)<9):
            return
        
        self.pos_corr_x.append(float(vals[0]))
        self.pos_corr_y.append(float(vals[1]))
        self.pos_corr_z.append(float(vals[2]))
        self.pos_err_x.append(float(vals[3]))
        self.pos_err_y.append(float(vals[4]))
        self.pos_err_z.append(float(vals[5]))
        self.accel_corr_x.append(float(vals[6]))
        self.accel_corr_y.append(float(vals[7]))
        self.accel_corr_z.append(float(vals[8]))


if __name__=="__main__":

    lines = read_log("../../build/standard.mcsimlog")

    inav = INav()

    for l in lines:
        splits = l.split(",")
        if splits[0]=='inav':
            inav.parse_inav(splits[1:])
        elif splits[0]=='inavc':
            inav.parse_inavc(splits[1:])

    #fig, ax = plt.subplots(6,1, figsize=(14,8))
    fig = plt.figure(figsize=(14,8))
    gs = gridspec.GridSpec(12,1, figure=fig)

    #baro.plot(fig.add_subplot(gs[0,0]),
              #fig.add_subplot(gs[1,0]),
              #fig.add_subplot(gs[2,0]))
    
    fmt = { 'color': 'blue', 'linestyle':'--'}
    plot('pos_x (cm)', inav.pos_x, fig.add_subplot(gs[0,0]), fmt)
    plot('pos_y (cm)', inav.pos_y, fig.add_subplot(gs[1,0]), fmt)
    plot('pos_z (cm)', inav.pos_z, fig.add_subplot(gs[2,0]), fmt)
    plot('vel_x (cm/s)', inav.vel_x, fig.add_subplot(gs[3,0]), fmt)
    plot('vel_y (cm/s)', inav.vel_y, fig.add_subplot(gs[4,0]), fmt)
    plot('vel_z (cm/s)', inav.vel_z, fig.add_subplot(gs[5,0]), fmt)

    plot('pos_err_x (cm)', inav.pos_err_x, fig.add_subplot(gs[6,0]), fmt)
    plot('pos_err_y (cm)', inav.pos_err_y, fig.add_subplot(gs[7,0]), fmt)
    plot('pos_err_z (cm)', inav.pos_err_z, fig.add_subplot(gs[8,0]), fmt)
    plot('pos_corr_x (cm)', inav.pos_corr_x, fig.add_subplot(gs[9,0]), fmt)
    plot('pos_corr_y (cm)', inav.pos_corr_y, fig.add_subplot(gs[10,0]), fmt)
    plot('pos_corr_z (cm)', inav.pos_corr_z, fig.add_subplot(gs[11,0]), fmt)

    '''
    imu.plot(fig.add_subplot(gs[0,0]),
             fig.add_subplot(gs[1,0]),
             fig.add_subplot(gs[2,0]),
             fig.add_subplot(gs[3,0]),
             fig.add_subplot(gs[4,0]),
             fig.add_subplot(gs[5,0]))
    '''

    #plt.tight_layout()
    plt.show()



