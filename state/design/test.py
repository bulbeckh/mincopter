
import math
import numpy as np

mag_ned = np.array([
        [math.cos(68*math.pi/180), 0, math.sin(68*math.pi/180)],
        [0, 1, 0],
        [-math.sin(68*math.pi/180), 0, math.cos(68*math.pi/180)]], dtype=np.float64) @ np.array([
            [math.cos(11*math.pi/180), -math.sin(11*math.pi/180), 0],
            [math.sin(11*math.pi/180), math.cos(11*math.pi/180), 0],
            [0, 0, 1]], dtype=np.float64)

print(mag_ned)
