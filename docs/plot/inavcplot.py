
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def read_log(path):
    tgt = None
    with open(path, 'r') as rfile:
        tgt = rfile.readlines()

    return tgt

class IMUSensor:
    def __init__(self, title):
        self.title = title
        self.parsed = 0

        self.accel_x = []
        self.accel_y = []
        self.accel_z = []

        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

    def parse(self, vals):
        if len(vals)<6:
            return

        self.accel_x.append(float(vals[0]))
        self.accel_y.append(float(vals[1]))
        self.accel_z.append(float(vals[2]))
        self.gyro_x.append(float(vals[3]))
        self.gyro_y.append(float(vals[4]))
        self.gyro_z.append(float(vals[5]))

        self.parsed += 1

    def plot(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):

        accel_x.plot(range(0,self.parsed), self.accel_x, color='blue', linestyle='--')
        accel_x.set_title(f'Target ({self.title})')
        accel_x.legend()
        accel_x.grid(True)

        accel_y.plot(range(0,self.parsed), self.accel_y, color='blue', linestyle='--')
        accel_y.set_title(f'Target ({self.title})')
        accel_y.legend()
        accel_y.grid(True)

        accel_z.plot(range(0,self.parsed), self.accel_z, color='blue', linestyle='--')
        accel_z.set_title(f'Target ({self.title})')
        accel_z.legend()
        accel_z.grid(True)

        gyro_x.plot(range(0,self.parsed), self.gyro_x, color='blue', linestyle='--')
        gyro_x.set_title(f'Target ({self.title})')
        gyro_x.legend()
        gyro_x.grid(True)

        gyro_y.plot(range(0,self.parsed), self.gyro_y, color='blue', linestyle='--')
        gyro_y.set_title(f'Target ({self.title})')
        gyro_y.legend()
        gyro_y.grid(True)

        gyro_z.plot(range(0,self.parsed), self.gyro_z, color='blue', linestyle='--')
        gyro_z.set_title(f'Target ({self.title})')
        gyro_z.legend()
        gyro_z.grid(True)

if __name__=="__main__":

    lines = read_log("../../build/standard.mcsimlog")

    imu = IMUSensor('IMU')

    ## NOTE We are re-using the imu variable names but this is really for inav
    for l in lines:
        splits = l.split(",")
        if splits[0]=='inavc':
            imu.parse(splits[1:])

    #fig, ax = plt.subplots(6,1, figsize=(14,8))
    fig = plt.figure(figsize=(14,8))
    gs = gridspec.GridSpec(6,1, figure=fig)

    #baro.plot(fig.add_subplot(gs[0,0]),
              #fig.add_subplot(gs[1,0]),
              #fig.add_subplot(gs[2,0]))

    imu.plot(fig.add_subplot(gs[0,0]),
             fig.add_subplot(gs[1,0]),
             fig.add_subplot(gs[2,0]),
             fig.add_subplot(gs[3,0]),
             fig.add_subplot(gs[4,0]),
             fig.add_subplot(gs[5,0]))

    plt.tight_layout()
    plt.show()



