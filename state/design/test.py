
import math
import numpy as np

## Computing magnetic field inclinatino and declination

## Rotation about y-axis
rot_y = np.array([
        [math.cos(68*math.pi/180), 0, math.sin(68*math.pi/180)],
        [0, 1, 0],
        [-math.sin(68*math.pi/180), 0, math.cos(68*math.pi/180)]], dtype=np.float64)

## Rotation about z-axis
rot_z = np.array([
            [math.cos(11*math.pi/180), -math.sin(11*math.pi/180), 0],
            [math.sin(11*math.pi/180), math.cos(11*math.pi/180), 0],
            [0, 0, 1]], dtype=np.float64)


## We rotate extrinsically here
print(rot_z @ rot_y)
