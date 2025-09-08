
import math
import numpy as np

from matplotlib import pyplot as plt

data = []

class ComplementaryFilter:

    def __init__(self):
        self.att = [0.0, 0.0, 0.0]

    def update(self, reading, idx):
        ''' *reading* is a dictionary entry with all sensor readings
        idx is the iteration number'''


        theta_mag = [math.atan2(reading['ay'], reading['az']),
                     math.atan2(-reading['ax'], math.sqrt(reading['ay']**2+reading['az']**2)),
                     0]

        ## TODO Compensate magnetometer readings first both for bias and then for inclination/declination

        b_mat = np.array([
            [math.cos(theta_mag[0]), math.sin(theta_mag[0])*math.sin(theta_mag[1]), math.sin(theta_mag[0])*math.cos(theta_mag[1])],
            [0, math.cos(theta_mag[1]), -math.sin(theta_mag[1])],
            [-math.sin(theta_mag[0]), math.cos(theta_mag[0])*math.sin(theta_mag[1]), math.cos(theta_mag[0])*math.cos(theta_mag[1])]], dtype=np.float64)

        b_val = b_mat @ np.array([reading['mx'], reading['my'], reading['mz']]).reshape(3,-1)

        ## TODO NOTE There is a computationally shorter way of finding theta_z
        theta_z = math.atan2(-b_val[1], b_val[0])


        theta_gyro = [self.att[0] + reading['gx']*(reading['dt']*1e-6),
                      self.att[1] + reading['gy']*(reading['dt']*1e-6),
                      self.att[2] + reading['gz']*(reading['dt']*1e-6)]


        alpha = 0.5

        ## Fuse on everything excep first run
        if idx==0:
            self.att[0] = theta_mag[0]
            self.att[1] = theta_mag[1]
            self.att[2] = theta_z
        else:
            self.att[0] = theta_mag[0]*alpha + theta_gyro[0]*(1-alpha)
            self.att[1] = theta_mag[1]*alpha + theta_gyro[1]*(1-alpha)
            self.att[2] = theta_z*alpha + theta_gyro[2]*(1-alpha)

        print(self.att)

        return

if __name__=="__main__":

    with open('./screenlog.0', 'r') as rfile:
        for idx, g in enumerate(rfile.readlines()):
            i = g.split(',')

            data.append({
                'ax': float(i[0]),
                'ay': float(i[1]),
                'az': float(i[2]),
                'gx': float(i[3]),
                'gy': float(i[4]),
                'gz': float(i[5]),
                'mx': float(i[6]),
                'my': float(i[7]),
                'mz': float(i[8]),
                'dt': (0 if idx==0 else (int(i[9].strip('\n')) - last_time))
                })

            last_time = int(i[9].strip('\n'))

    cf = ComplementaryFilter()

    fig, axes = plt.subplots(3, 1, figsize=(8, 10))

    # Add titles to each subplot for clarity
    axes[0].set_title("Roll")
    axes[1].set_title("Pitch")
    axes[2].set_title("Yaw")

    for i, d in enumerate(data):
        cf.update(d, i)
        #print(i, cf.att[0])
        axes[0].scatter(i, cf.att[0], color='blue', linestyle='-', linewidth=1)
        axes[1].scatter(i, cf.att[1], color='blue', linestyle='-', linewidth=1)
        axes[2].scatter(i, cf.att[2], color='blue', linestyle='-', linewidth=1)

    # Adjust layout
    plt.tight_layout()
    plt.show()

    ## Implement EKF and other code


